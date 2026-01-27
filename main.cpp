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
#define PWM_PERIOD 500                  // PWM Period

// LED
#define RED_LED BIT0                    // Onboard LED
#define GREEN_LED BIT6                  // Onboard LED

// ADC
#define ADCMAX 4095.00f //Correlating to 12 bits
#define VREF  3.3f
#define RESISTOR 9815.0f
#define BETA 3380.0f 
#define RTHERM 10000.0f
#define T0_K 298.15f //25 degrees Celsius in Kelvin 

// ADC pins
#define ADC_CHAM_THERM   ADCINCH_8    // P5.0 / A8
#define ADC_BAT_THERM    ADCINCH_10   // P5.2 / A10
#define ADC_CUR_SENSE    ADCINCH_11   // P5.3 / A11

// UpdateFlightState
#define ACCL_THRESHOLD        3       // m^2/s. 
#define ALTITUDE_THRESHOLD    5       // meters

// Threshold values based on testing and material properties. 
#define CURRENT_MAX 7.26   // Max current in Amps
#define BATTERY_MAX 60     // Max temperature in Celsius
#define CHAMBER_MAX 200

#define R_SENSE 0.1       // Shunt resistor in Ohms

// GPS 
#define GPS_BUFFER_SIZE 128

// ==== Interrupts ==== //

// BMP_Interrupt: Whenever an interrupt from Port 2 triggers, this code run to notify BMP Data is ready. 
volatile bool bmp_data_ready = false;
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void) {
    if (P2IFG & BIT4) {                     // If Bit4 on Port2 is set (BMP_INT)
        bmp_data_ready = true;              // Signal main loop that BMP data is ready. 
        P2IFG &= ~BIT4;                     // MUST clear flag
    }
}

// ADC Interrupt: Interrupt that read ADC memory into a global variable
volatile uint16_t adc_result;
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void) {
    switch (__even_in_range(ADCIV, ADCIV_ADCIFG))
    {
        case ADCIV__ADCIFG0:
            adc_result = ADCMEM0;   // After memory is read, ADCIFG0 is cleared. 
            P1OUT ^= BIT0;
            __bic_SR_register_on_exit(CPUOFF);
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
    P5DIR  |= BIT1;                             // Set BIT1 of P5DIR to output. BIT1 Corresponds to PIN1 on the Port.
    
    P5SEL0 |= BIT1;     // Set BIT 1 of P5SEL0 = 1
    P5SEL1 &= ~BIT1;    // Set BIT 1 of P5SEL1 = 0
    //Page 104 --> 01b = select timer Timer_B2
    
    PM5CTL0 &= ~LOCKLPM5;                       // Unlock GPIO

    // Setup Compare Reg
    TB2CCR0 = PWM_PERIOD;                       // Roll-over from LOW to HIGH - Customise 
    TB2CCR2 = 0;                                // 0% duty cycle
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

// Utilise BOTH sensor's reading of altitude and acceleration to determine current flight state. 
// Compare value against initial calculated altitude and pressure to determine change. 
void UpdateFlightState() {
    float delta;
    
    if (!AltWindow_GetDelta(&delta)){
        return;
    }

    float accl_magnitude = ReadACCL();

    switch (current_flight_state) {
        
        case PREFLIGHT:
        // Detect launch: altitude change (current altitude jumps from ground altitude) + movement (acceleration value > 0)
            if (delta > ALTITUDE_THRESHOLD && accl_magnitude > ACCL_THRESHOLD) {
                prev_flight_state = PREFLIGHT;
                current_flight_state = FLIGHT;
            }
            break;
            
        case FLIGHT:
        // Detect landing: altitude change negligible/altitude approx ground altitude AND negligible acceleration
            if (delta < ALTITUDE_THRESHOLD && accl_magnitude < ACCL_THRESHOLD) {
                prev_flight_state = FLIGHT;
                current_flight_state = LANDED;
            }
            break;

        case LANDED:
            break;
            
        case SHUTDOWN:
            break;
    }
}

// Demonstration of how sensor's reading of altitude and acceleration is used to determine current flight state. 
void UpdateFlightStateDemo(float delta, float accl_magnitude) {
    switch (current_flight_state) {
        case PREFLIGHT:
        // Detect launch: altitude change (current altitude jumps from ground altitude) + movement (acceleration value > 0)
            if (delta > ALTITUDE_THRESHOLD && accl_magnitude > ACCL_THRESHOLD) {
                prev_flight_state = PREFLIGHT;
                current_flight_state = FLIGHT;
            }
            break;
            
        case FLIGHT:
        // Detect landing: altitude change negligible/altitude approx ground altitude AND negligible acceleration
            if (delta < ALTITUDE_THRESHOLD && accl_magnitude < ACCL_THRESHOLD) {
                prev_flight_state = FLIGHT;
                current_flight_state = LANDED;
            }
            break;

        case LANDED:
            break;
            
        case SHUTDOWN:
            break;
    }
}

/* -------------------------------ADC------------------------------- */

// Initialise ADC for use. 
void InitADC () {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    PMMCTL2 |= REFVSEL_2 + INTREFEN_1;          // Enable and set the internal reference voltage. 
    while (!(PMMCTL2 & REFGENRDY));             // Wait until V_ref is ready to be used.
}

// InitADCGPIO intialises all 3 GPIO pins as analog inputs for the ADC
void InitADCGPIO(){
    // P5.0 (Chamber Thermister Temperature)
    P5SEL0 |= BIT0;
    P5SEL1 |= BIT0;

    // P5.2 (Battery Thermister Temperature)
    P5SEL0 |= BIT2;
    P5SEL1 |= BIT2;

    // P5.3 (Current Sense Value)
    P5SEL0 |= BIT3;
    P5SEL1 |= BIT3;

}

void ConfigADC() {
    ADCCTL0 = ADCSHT_4 | ADCON;                         // Sample and Hold time for 64 clk cycles | enable ADC (Start conversion later)
    ADCCTL1 = ADCSSEL_2 | ADCSHP;                       // Select SMCLK | signal sourced from sampling timer.
    ADCCTL2 = ADCRES_2;                                 // Set 12-Bit ADC Resolution
    ADCIE = ADCIE0;                                     // Enable interrupts
}

void StartADC(uint8_t channel) {

    ADCCTL0 &= ~ADCENC;                  // Must disable ADC
    ADCMCTL0 = channel | ADCSREF_0;      // Set channel + AVCC ref
    ADCCTL0 |= ADCENC | ADCSC;           // Start conversion
    __bis_SR_register( GIE);             // Go into low power mode 0 with interrupts enabled
}

int16_t TempConversion(volatile int16_t adcValue ) {
	//converting adc count to voltage 
	volatile double voltage_temp = (adcValue / ADCMAX) * VREF; 
	
	//calculate thermistor resistance 
	volatile double ThermistorResistance = RESISTOR * (VREF - voltage_temp)/ voltage_temp; // Should add error checking. ie dividing by 0

	//apply beta formula 
	volatile double InvertedTemp = (1.0/T0_K) + (1.0/ BETA) * log(ThermistorResistance/RTHERM); 
	volatile double TempInKelvin = 1/InvertedTemp; 
	
	//convert to Celsius 
	volatile double TempInCelsius = TempInKelvin - 273.15;

	return (int16_t)TempInCelsius;
}

// Return the temperature of the chamber 
int16_t GetChamberTemp () {
    volatile uint16_t chamThermADC;
    uint16_t cham_ADC_temperature;

    // Chamber thermistor
    StartADC(ADC_CHAM_THERM);
    __bis_SR_register(CPUOFF | GIE);
    chamThermADC = adc_result; 

    
    // Convert temperatures
    cham_ADC_temperature = TempConversion(chamThermADC);
    return cham_ADC_temperature;
}   

// Return the temperature of the battery 
int16_t GetBatteryTemp () {
    volatile uint16_t batThermADC;

    // Battery thermistor
    StartADC(ADC_BAT_THERM);
    __bis_SR_register(CPUOFF | GIE);
    batThermADC = adc_result;

    return TempConversion(batThermADC);
}

// Return the value of current 
double CurrentSense () { 
    uint16_t curSenseADC;

    // Current sense
    StartADC(ADC_CUR_SENSE);
    __bis_SR_register(CPUOFF + GIE);        
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
    return (current > CURRENT_MAX);   // Returns true if over threshold, false otherwise
}

bool BatExceedsThreshold() {
    double battery = GetBatteryTemp();   
    return (battery > BATTERY_MAX);   
}

bool ChamExceedsThreshold() {
    double chamber = GetChamberTemp();   
    return (chamber > CHAMBER_MAX);   
}



// ====== Working Background Timer ====== //

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

// Initialise Timer_B for tick every 1 second. 
void Timer3_init_1s_tick(void)
{
    TB3CTL = TBSSEL__ACLK | MC__UP | TBCLR;
    TB3CCR0 = 32768 - 1;     // 1 second
    TB3CCTL0 = CCIE;
}

// ISR. Interrupt is called every second, decrementing delay time until 0. 
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

// Example Working Template of Background Timer in main()
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


// ======================  GPS   ======================

volatile char gps_buffer[GPS_BUFFER_SIZE];
volatile uint8_t gps_index = 0;
volatile uint8_t gps_line_ready = 0;

// Initialise UART pins for GPS
void InitGPSUART(){
    // Configure pins for UART
    P1SEL0 |= BIT6 | BIT7;   // RX + TX
    P1SEL1 &= ~(BIT6 | BIT7);
    // TX not used

    // Configure UART
    // Put eUSCI in reset
    UCA0CTLW0 = UCSWRST;              

    // Select SMCLK as clock source
    UCA0CTLW0 |= UCSSEL__SMCLK;

    // // Baud rate: 16MHz / 9600
    // UCA0BRW = 104;                      
    // UCA0MCTLW = UCOS16 | UCBRF_2 | 0x4900;

    // Baud rate 115200, SMCLK = 16MHz, oversampling
    UCA0BRW = 8;                   // Integer part
    UCA0MCTLW = UCOS16 | (11 << 4) | 0xD600; // UCBRFx = 11, UCBRSx = 0xD6
    
    // Release eUSCI from reset
    UCA0CTLW0 &= ~UCSWRST;

    PM5CTL0 &= ~LOCKLPM5; // Unlock GPIO

    // Enable RX interrupt before releasing reset
    UCA0IE |= UCRXIE;
}

// UART ISR
#pragma vector = EUSCI_A0_VECTOR // UART RX Interrupt Vector
__interrupt void USCIA1RX_ISR(void){
    
    P1OUT ^= BIT0;              // Toggle LED for debugging
    char received = UCA0RXBUF;
    if (received == '\n') {
        gps_buffer[gps_index] = '\0';
        gps_index = 0;
        gps_line_ready = 1;
    }
    else {
        if (gps_index < GPS_BUFFER_SIZE - 1) {
            gps_buffer[gps_index++] = received;
        } else {
            gps_index = 0;
        }
    }
}

void InitClock16MHz(void){
    CSCTL0_H = 0xA5;          // Unlock CS registers
    CSCTL1 = 0x0040;          // DCO = 16 MHz
    CSCTL2 = 0x0033;          // SMCLK = MCLK = DCO
    CSCTL3 = 0x0000;          // No division
    CSCTL0_H = 0x00;          // Lock CS registers
}

void ClearGPSBuffer(void){
    uint8_t i;
    for (i = 0; i < GPS_BUFFER_SIZE; i++) {
        gps_buffer[i] = 0;
    }
    gps_index = 0;
    gps_line_ready = 0;
}

// =======================================

void RecoveryMode(void) {
    uint8_t duty_cycle = 0;
    bool pwm_enabled = false;
    bool initial_delay_done = false;

    DisableBMP();
    DisableACCL();

    InitCoilPWM(); 

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

                // Start PWM ON window for 15 seconds. Smoke release. 
                pwm_enabled = true;
                duty_cycle = 100;
                SetCoilPWM(duty_cycle);

                start_delay_seconds(15);
            }
        }

        // Stage 2: Timer for 15 seconds has started. The PWM Coil is on. Continuous monitoring of current and temperature.
        if (pwm_enabled) {

            // This loop is blocking so there's no other background task that happens within the 15 seconds. 
            while (!timer_expired) {
                // Safety checks
                if (CurrExceedsThreshold() || BatExceedsThreshold() || ChamExceedsThreshold()) {
                    duty_cycle = 0;
                    SetCoilPWM(duty_cycle); }
            }

            timer_expired = 0;

            // Finish stage 2. After 15 seconds has passed, turn off PWM. 
            pwm_enabled = false;
            SetCoilPWM(0);

            // Start cooldown
            start_delay_seconds(45);
        }

        // Stage 3: 
        if (initial_delay_done && !pwm_enabled)
        {
            // 45 second cooldown. Allow wick to resaturate. 
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

// =======================================

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
    // GPS Testing. Entering interrupts to get NMEA values.  
    WDTCTL = WDTPW | WDTHOLD;
    InitClock16MHz();

    P1DIR |= BIT0;             // RED LED as output
    P1OUT &= ~BIT0;            // Turn RED LED off
    ClearGPSBuffer();
    InitGPSUART();

	__bis_SR_register(LPM0_bits + GIE); // Enter LPM0, Enable Interrupt
    */


    /*
    // Demonstrate change in flight state. Passing in mock delta altitude value, and acceleration magnitude. 
    current_flight_state = PREFLIGHT;
    UpdateFlightStateDemo(10, 10); 
    UpdateFlightStateDemo(2, 2); 

    if (current_flight_state == LANDED) {
        RecoveryMode();
    }
    */
    
    
    


    
    //Successful pulsing of PWM using LED
    WDTCTL = WDTPW + WDTHOLD; // Stop Watchdog Timer
    InitCoilPWM();

    while (1)
	{
		for (uint16_t i = 0; i < 100; i++)
		{
            SetCoilPWM(i);
            __delay_cycles(50 * PWM_PERIOD);
		}
		for (uint16_t i = 100; i > 0; i--)
		{
			SetCoilPWM(i);
            __delay_cycles(50 * PWM_PERIOD);
		}
	}
    
    



    /*
    //Sucessful testing of toggling LEDs with background timer.
    WDTCTL = WDTPW | WDTHOLD;

    // Initialise timer
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

    // Being 40 seconds timer
    start_delay_seconds(40);   // start 40 seconds timer
    SetCoilPWM(100); // Turn LED ON

    // Set initial values - red LED on, green LED off.
    P1OUT |= RED_LED;
    P6OUT &= ~GREEN_LED;

    while (1)
    {
        if (timer_expired)
        {
            timer_expired = 0;
            // Execute task after 40 seconds end
            SetCoilPWM(0); // Turn LED OFF (Mock PWM Coil)
        }

        // other background work here. in this case it would be LoRa
        // state machines, IO, comms, etc.

        P1OUT ^= RED_LED;
        P6OUT ^= GREEN_LED;
        __delay_cycles(100000); // 100ms delay
    }
    */


    /*
    //ADC Testing:
    InitADC();

    P1DIR |= BIT0;             // RED LED as output
    P1OUT &= ~BIT0;            // Turn RED LED off

    // Configure P5.0
    InitADCGPIO();

    ConfigADC();

    PM5CTL0 &= ~LOCKLPM5;   // Unlock GPIO pins

    uint16_t chamber_temperature;
    uint16_t battery_temperature;

    while(1)
    {
        // Start Conversion
        chamber_temperature = GetChamberTemp();
        battery_temperature = GetBatteryTemp();
        
    }
    */
} 
