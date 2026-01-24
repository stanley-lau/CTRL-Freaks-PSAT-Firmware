#include <driverlib.h>
#include <msp430.h>
#include <stdint.h>
#include "util.h"
#include <math.h> 

#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"

#include "pin_mappings.hpp"
#include "bmp.hpp"
#include "bmp_buffer.hpp"
#include "accl.hpp"

// ==== Define Constants ==== //
// PWM
#define PWM_PERIOD 500

// ==== Interrupts ==== //

// BMP_Interrupt: Whenever an interrupt from Port 2 triggers, this code will run.
volatile bool bmp_data_ready = false;

#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void) {
    if (P2IFG & BIT4) {                     // if Bit4 on Port2 is set (BMP_INT)
        bmp_data_ready = true;                // Signal main loop
        P2IFG &= ~BIT4;                     // MUST clear flag
    }
}

// ADC Interrupt: interrupt that read ADC memory
volatile uint16_t adc_result;
    #pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void) {
    switch (__even_in_range(ADCIV, 0x0C)) //Interrupt Source: ADC memory Interrupt flag; Interrupt Flag: ADCIFG0
    {
        case 0x0C:
            adc_result = ADCMEM0;   // Read ADC result
            LPM0_EXIT;             // Wakes up CPU
            break;
        default:
            break;
    }
}

// ==== FlightStates ==== // 
enum FlightState {PREFLIGHT, FLIGHT, LANDED, SHUTDOWN };
enum FlightState current_flight_state = PREFLIGHT;
enum FlightState prev_flight_state;

float ground_pressure = 0.0f;
float initial_altitude = 0.0f;

// InitCoilPWM initialises PWM for Port P5.1 (PWM_Coil)
// Avoid writing to TB2CTL after init
void InitCoilPWM(){
    // Setup Pins
    P5DIR  |= BIT1;                             // Set BIT1 of P5DIR to 1. (1 = Output) BIT1 Corresponds to PIN1 on the Port.
    
    /*
    P5SEL0 &= ~BIT1;                            // Clear BIT1 in P5SEL0. Now P5SEL0 BIT1 = 0
	P5SEL1 |= BIT1;                             // Set BIT1 in P5SEL1. Now P5SEL1 BIT1 = 1
    // 10b = Secondary module function is selected which correspond to the timer Timer_B2.
    */


    P5SEL0 |= BIT1;     // Set P5SEL0 = 1
    P5SEL1 &= ~BIT1;    // Set P5SEL1 = 0
    //Page 104 --> 01b = select timer Timer_B2
    
    PM5CTL0 &= ~LOCKLPM5;                       // Unlock GPIO

    // Setup Compare Reg
    TB2CCR0 = PWM_PERIOD;                       // Roll-over from LOW to HIGH - Customise 
    TB2CCR2 = 0;                                 // 0% duty cycle
    TB2CCTL2 = OUTMOD_7;                        // Reset/Set PWM mode, refer to YT vid
    // TB2CCRn is the timer attached to Port P5 from Table 6-67

    // Setup Timer_B2
    TB2CTL =    TBCLR       |       TBSSEL_2    | MC_1;
    //     = Clear Timer_B0 |  Set SMCLK as clk | Up-Mode: Count upwards
}

// SetCoilPWM sets the duty_cyle for the coil.
void SetCoilPWM(uint8_t duty_cycle) {
    // PWM Coil P5.1 on schematic. 
    // duty_cyle must be a positive integer between 0 and 100;
    // SMCLK is used at 1Mhz

    if (duty_cycle > 100) {
        duty_cycle = 100;
    }
    
    // Update Duty cycle.
    if (duty_cycle >= 100) {
        TB2CCR2 = TB2CCR0 + 1; // force always-high
    } else {
        TB2CCR2 = (TB2CCR0 * duty_cycle) / 100;     // // Set LOW when TB0CCR1 is reached
    }
}

// Both placeholder value that should be checked 
#define ACCL_THRESHOLD        5 
#define ALTITUDE_THRESHOLD    5  
/*
    Modified from updateFlightState();
    This code should ultilse BOTH sensor's readings to determine the state 
*/
void UpdateFlightState() {
    float delta;
    
    if (!AltWindow_GetDelta(&delta)){
        return;
    }

    float accl_magnitude = ReadACCL();

    switch (current_flight_state) {
        
        case PREFLIGHT:
        // Detect launch: altitude change (current altitude jumps from ground altitude_ + movement (acceleration value > 0)
            if (delta > ALTITUDE_THRESHOLD && accl_magnitude > ACCL_THRESHOLD) {
                prev_flight_state = PREFLIGHT;
                current_flight_state = FLIGHT;
            }
            break;
            
        case FLIGHT:
        // Detect landing: altitude change small/altitude approx ground altitude AND near-zero acceleration
            if (delta < ALTITUDE_THRESHOLD && accl_magnitude < ACCL_THRESHOLD) {
                prev_flight_state = FLIGHT;
                current_flight_state = LANDED;
            }
            break;

        case LANDED:
            // do nothing;
            // Stay here to run recovery mode or manual shutdown 
            break;
            
        case SHUTDOWN:
            break;
    }
}


/* -------------------------------ADC------------------------------- */



#define ADCMAX 4095.00 //Correlating to 12 bits
#define VREF  2.5
#define RESISTOR 10000
#define BETA 3380 
#define RTHERM 10000
#define T0_K 298.15 //25 degrees Celsius in Kelvin 

// All these thresholds are placeholder values!!! Must be checked and updated!!! 
#define CURRENT_MAX 2.0   // Max current in Amps
#define BATTERY_MAX 2.0
#define CHAMBER_MAX 2.0

#define R_SENSE 0.1       // Shunt resistor in Ohms

#define ADC_CHAM_THERM   ADCINCH_8    // P5.0 / A8
#define ADC_BAT_THERM    ADCINCH_10   // P5.2 / A10
#define ADC_CUR_SENSE    ADCINCH_11   // P5.3 / A11

void InitADC () {
    /*
    may need to write code for this init. 
    clock_init();
    gpio_init();
    */

    PMMCTL2 = REFVSEL_2 | INTREFEN_1;           // Set reference voltage to be 2.5V, then enable the internal reference. 
    while (!(PMMCTL2 & REFGENRDY));             // Wait until V_ref is ready to be used. Previously: __delay_cycles(1000);
  
    ADCCTL1 = ADCDIV_2 | ADCSHP_0;              // Divide input clock by 3 and select source to be sample-input

    ADCCTL2 &= ~ADCRES;                         // Clear bits/resolution value 
   
    ADCCTL2 |= ADCRES_2;                        // 12-bit resolution for the conversion result

    ADCIE |= ADCIE0;                            // Enable interrupts 

    ADCCTL0 = ADCSHT_4 | ADCON;                 // sample and hold for 64 clock cycles, enable ADC.

    __delay_cycles(100);						// Wait for reference to settle

    PM5CTL0 &= ~LOCKLPM5;                       // Unlock GPIO / LPMx.5 Lock Bit 
}

void StartADC(uint8_t channel) {
    //ADCCTL0 &= ADCENC_0;                // Stop ADC (Using "_0 / _1" can be problematic as they're meant to be assignemts)
    ADCCTL0 &= ~ADCENC;                  // Clear the bit that enables ADC conversion. AKA stop ADC

    ADCMCTL0 = ADCSREF_1 | channel;     // Select input channel + reference  1: 001b = {VR+ = VREF and VR– = AVSS}

    ADCCTL0 |= ADCENC | ADCSC;          // Enable + start conversion
}

int16_t TempConversion(int16_t adcValue ) {
	//converting adc count to voltage 
	double voltage_temp = (adcValue / ADCMAX) * VREF; 
	
	//calculate thermistor resistance 
	double ThermistorResistance = RESISTOR * (VREF - voltage_temp)/ voltage_temp; // Should add error checking. ie dividing by 0

	//apply beta formula 
	double InvertedTemp = (1.0/T0_K) + (1.0/ BETA) * log(ThermistorResistance/RTHERM); // Log not defined. Will need to find a way to allow this without heavy resource consumption
	double TempInKelvin = 1/InvertedTemp; 
	
	//convert to Celsius 
	double TempInCelsius = TempInKelvin - 273.15;

	return (int16_t)TempInCelsius;
}

// Return the temperature of the chamber 
int16_t ChamberTemp () {
    uint16_t chamThermADC;

    // Chamber thermistor
    StartADC(ADC_CHAM_THERM);
    LPM0;                                 //Code only continues when interrupt handler has collected all data
    chamThermADC = adc_result; 

    // Convert temperatures
    return TempConversion(chamThermADC);
}   

// Return the temperature of the battery 
int16_t BatteryTemp () {
    uint16_t batThermADC;

    // Battery thermistor
    StartADC(ADC_BAT_THERM);
    LPM0;
    batThermADC = adc_result;

    return TempConversion(batThermADC);
}

// Return the value of current 
double CurrentSense () { 
    uint16_t curSenseADC;

    // Current sense
    StartADC(ADC_CUR_SENSE);
    LPM0;
    curSenseADC = adc_result;       
    double current = ((double)curSenseADC / ADCMAX) * VREF / R_SENSE;       // Convert ADC to current 
    return current;
}   

/* Code to turn off current when it exceeds a threshold. HERE IF NEEDED. 
void currExceedThreshold() {
    P5DIR |= BIT3;                       // Set P5.3 as output
    double current = currentSense ();
    if (current > CURRENT_MAX) {         // Threshold currently has placeholder value 
            P5OUT &= ~BIT3;              // Turn off current (port 5, pin 3) if it exceeds a value 
        } else {
            P5OUT |= BIT3;               // Turns back on if current return to a safe low value
        }
}
*/

bool CurrExceedsThreshold() {
    double current = CurrentSense();  // Gets current value 
    return (current > CURRENT_MAX);   // Threshold currently has placeholder value. Returns true if over threshold, false otherwise
}

bool BatExceedsThreshold() {
    double battery = BatteryTemp();   
    return (battery > BATTERY_MAX);   // Threshold currently has placeholder value. 
}

bool ChamExceedsThreshold() {
    double chamber = ChamberTemp();   
    return (chamber > CHAMBER_MAX);   // Threshold currently has placeholder value. 
}





// ====== Working Background Timer Dump ====== //

// Global variables
volatile uint16_t delay_seconds = 0;
volatile uint8_t timer_expired = 0;

// Call the function to set an X second counter in the BACKGROUND
void start_delay_seconds(uint16_t seconds)
{
    __disable_interrupt();
    delay_seconds = seconds;
    timer_expired = 0;
    __enable_interrupt();
}

// Init Timer_B for 1 second tick
void Timer3_init_1s_tick(void)
{
    TB3CTL = TBSSEL__ACLK | MC__UP | TBCLR;
    TB3CCR0 = 32768 - 1;     // 1 second
    TB3CCTL0 = CCIE;
}

// ISR
#pragma vector = TIMER3_B0_VECTOR
__interrupt void Timer3_B0_ISR(void)
{
    if (delay_seconds > 0)
    {
        delay_seconds--;

        if (delay_seconds == 0)
        {
            timer_expired = 1;
        }
    }
}

// Example Template of Background Timer in main()
/*
    WDTCTL = WDTPW | WDTHOLD;

        Timer3_init_1s_tick();
        __enable_interrupt();

        start_delay_seconds(600);   // start 10-minute timer

        while (1)
        {
            if (timer_expired)
            {
                timer_expired = 0;
                // do the thing after 10 minutes

                // You can Rearm the timer here byt calling start_delay_seconds(6000) here for example.
            }

            // other background work here
            // state machines, IO, comms, etc.
        }

*/

// ====== Working Background Timer Dump ====== //

#define RED_LED BIT0
#define GREEN_LED BIT6

// =======================================


void RecoveryMode2(void) {
    uint8_t duty_cycle = 0;
    bool pwm_enabled = false;
    bool initial_delay_done = false;

    DisableBMP();
    DisableACCL();

    Timer3_init_1s_tick();
    __enable_interrupt();
    
    start_delay_seconds(450); // Initial 7.5 minute delay

    while (1)
    {
        // Stage 1: During first 7.5 minutes: Communicating with LoRa to receive GPS information. After 7.5min, move onto stage 2. 
        if (!initial_delay_done)
        {
            // LoRa + GPS work here
            if (timer_expired)
            {
                timer_expired = 0;
                initial_delay_done = true;

                // Start first PWM ON window
                pwm_enabled = true;
                duty_cycle = 100;
                SetCoilPWM(duty_cycle);

                start_delay_seconds(10);
            }
        }

        // Stage 2: Timer for 10 seconds has started. The PWM Coil is on. Continuous monitoring of current and temp 
    . 
        if (pwm_enabled) {

            // This loop is blocking so there's no other background task that happens within the 10 seconds. 
            while (!timer_expired) {
                // Safety checks
                if (CurrExceedsThreshold() || BatExceedsThreshold() || ChamExceedsThreshold()) {
                    duty_cycle = 0;
                    SetCoilPWM(duty_cycle); }
            }

            timer_expired = 0;

            // Finish stage 2. After 10 seconds has passed, turn off PWM. 
            pwm_enabled = false;
            SetCoilPWM(0);

            // Start cooldown
            start_delay_seconds(300);
        }

        // Stage 3: 
        if (initial_delay_done && !pwm_enabled)
        {
            SetCoilPWM(0);

            // Other background tasks here

            if (timer_expired)
            {
                timer_expired = 0;

                // Restart PWM ON window
                pwm_enabled = true;
                duty_cycle = 100;
                SetCoilPWM(duty_cycle);

                start_delay_seconds(10);
            }
        }
    }
}

/*
Main code; should be kept as clean as possible.
*/



int main(void) {
    // bool recovery_active = false;

    // // LoRa
    
    // InitADC();
    // InitCoilPWM();

    // InitACCL();
    // ConfigureACCL();

    // InitBMP();
    // ConfigureBMP();

    // ground_pressure = CalibrateGroundPressure(100);
    // initial_altitude = PressureToAltitude(ground_pressure);

    // current_flight_state = PREFLIGHT;

    // // LoRa to ensure init was done succesfully 

    // __enable_interrupt(); 

    // while(1){
    //     /*
    //     Read from sensors here
    //     */

    //     // 1. Check if new BMP sample ready
    //     if (bmp_data_ready) {
    //         bmp_data_ready = false;
    //         GetPressure();
    //             if (available_samples > 0) {
    //                 // read the next sample from buffer
    //                 //BMPData raw = bmpBuffer[read_index]; // RAW WTF???
    //                 const volatile BMPData& raw = bmpBuffer[read_index];
    //                 //I think this is meant to be raw_pressure?

    //                 // advance read_index and decrease available_samples
    //                 read_index = (read_index + 1) % BMP_BUFFER_SIZE;
    //                 available_samples--;

    //                 // convert to altitude and push to sliding window
    //                 // float altitude = PressureToAltitude(bmpBuffer[read_index].pressure);
    //                 float altitude = PressureToAltitude(raw.pressure); //This is meant to raw_pressure?
    //                 AltWindow_Push(altitude);
    //             }
    //     }


    //     // Update Flight State // should pass in small sample of data for flight determination.
    //     UpdateFlightState();

    //     // State-depened behaviour
    //     switch (current_flight_state) {
    //         case PREFLIGHT:
    //             break;
    //         case FLIGHT:
    //             break;
    //         case LANDED:
    //             if (!recovery_active){
    //                 // Enter recovery
    //                 RecoveryMode();
    //                 recovery_active = true;
    //             }
    //             break;
    //         case SHUTDOWN:
    //             break;
    //     }
    // }


    /*
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    InitCoilPWM();
    
    while (1)
	{
		for (uint16_t i = 0; i < 100; i++)
		{
            SetCoilPWM(i);
            __delay_cycles(10 * PWM_PERIOD);
		}
		for (uint16_t i = 100; i > 0; i--)
		{
			SetCoilPWM(i);
            __delay_cycles(10 * PWM_PERIOD);
		}
	}
    */

    WDTCTL = WDTPW | WDTHOLD;

    // Init timer
    Timer3_init_1s_tick();

    // Init PWM
    InitCoilPWM();
    
    // Init LEDs
    P1DIR |= RED_LED; // Equivalent of P1DIR |= BIT0; due to the #DEFINE at the top of the program
    P6DIR |= GREEN_LED;

    // Turn LEDs off.
    P1OUT &= ~RED_LED;
    P6OUT &= ~GREEN_LED;

    __enable_interrupt();

    // Being 1-minute timer
    start_delay_seconds(60);   // start 1-minute timer
    SetCoilPWM(100); // Turn LED ON

    // Set initial values - red LED on, green LED off.
    P1OUT |= RED_LED;
    P6OUT &= ~GREEN_LED;

    while (1)
    {
        if (timer_expired)
        {
            timer_expired = 0;
            // 👇 do the thing after 1 minutes
            SetCoilPWM(0); // Turn LED OFF (Mock P:WM Coil)
        }

        // other background work here. in this case it would be LoRa
        // state machines, IO, comms, etc.

        P1OUT ^= RED_LED;
        P6OUT ^= GREEN_LED;
        __delay_cycles(100000); // 100ms delay
    }
}

