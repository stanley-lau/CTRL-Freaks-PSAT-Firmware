#include <driverlib.h>
#include <msp430.h>
#include <stdint.h>
#include "util.h"

#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"

#define PWM_PERIOD 500

// Templates for BMP 
Pin<P4,4> BMP_CS;
Pin<P4,5> BMP_CLK;
Pin<P4,6> BMP_MOSI;
Pin<P4,7> BMP_MISO;
SpiMaster<SPI_B1> BMP_SPI;

// initCoilPWM initialises PWM for Port P5.1 (PWM_Coil)
// Avoid writing to TB2CTL after init
void initCoilPWM(){
    // Setup Pins
    P5DIR  |= BIT1;                             // Set BIT1 of P5DIR to 1. (1 = Output) BIT1 Corresponds to PIN1 on the Port.
    P5SEL0 &= ~BIT1;                            // Clear BIT1 in P5SEL0. Now P5SEL0 BIT1 = 0
	P5SEL1 |= BIT1;                             // Set BIT1 in P5SEL1. Now P5SEL1 BIT1 = 1
    // 10b = Secondary module function is selected which correspond to the timer Timer_B2.
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

// setCoilPWM sets the duty_cyle for the coil.
void setCoilPWM(uint8_t duty_cycle){
    // PWM Coil P5.1 on schematic. 
    // duty_cyle must be a positive integer between 0 and 100;
    // SMCLK is used at 1Mhz

    if (duty_cyle > 100) {
        duty_cyle = 100;
    }
    
    // Update Duty cycle.
    if (duty_cyle >= 100) {
        TB2CCR2 = TB2CCR0 + 1; // force always-high
    } else {
        TB2CCR2 = (TB2CCR0 * duty_cycle) / 100;     // // Set LOW when TB0CCR1 is reached
    }
}

// Accl SPI pins are routed incorrectly. Cannot be resolved in software. Function written assuming routing is correctly
// initAccl initialises SPI functionality 
void initAccl(){
    /*
    Assumes:
    P4.0 = ACCL_CS
    P4.1 = ACCL_CLK
    P4.2 = ACCL_MISO (SOMI) (incorrect on schematic)
    P4.3 = ACCL_MOSI (SIMO) (incorrect on schematic)

    P6.0 = ACCL_INT1
    P2.3 = ACCL_INT2
    */
    
    // Configure Pin function for ACCL by setting them to Primary Module function (CLK, SOMI, SIMO)
    P4SEL0 |= BIT1 | BIT2 | BIT3;            // Set bit0
    P4SEL1 &= ~(BIT1 | BIT2 | BIT3);         // Clear bit1

    // Configure CS pin as GPIO
    P4SEL0 &= ~BIT0;
    P4SEL1 &= ~BIT0;
    
    // Configure CS pin as Output, and Pulling it HIGH (Active LOW)
    P4OUT |= BIT0;                           // Pull CS High
    P4DIR |= BIT0;                           // Set BIT0 of P4DIR to 1. (1 = Output) BIT0 Corresponds to PIN 0 on the Port.
    

    // Configure Interrupts by setting them as INPUT pins.
    P6DIR &= ~BIT0;                         // Clear Bit 0 in P6DIR to set P6.0 as input
    P6IES &= ~BIT0;                         // Rising edge (active HIGH)
    P6IFG &= ~BIT0;                         // Clear flag
    P6IE  |=  BIT0;                         // Enable interrupt

    P2DIR &= ~BIT3;                         // Clear Bit 3 in P2DIR to set P2.3 as input
    P2IES &= ~BIT3;                         // Rising edge (active HIGH)
    P2IFG &= ~BIT3;                         // Clear flag
    P2IE  |=  BIT3;                         // Enable interrupt

    // Init SPI specifications. 
    UCA1CTLW0 = UCSWRST;                                // Hold in RESET data to allow modification of UCA1CTLW0
    UCA1CTLW0 |=    UCMSB   |       UCSYNC     |    UCMST;      
    //        |= MSB First, | Synchronous mode | Master mode
    UCA1CTLW0 |= UCSSEL__SMCLK;                         // Seledt SMCLK as Clock
    UCA1BRW = 8;                                        // SPI CLK -> SMCLK/8 (Clock prescaler)
    UCA1CTLW0 &= ~UCSWRST;                              // Release RESET to allow the operation of SPI module

    /*
    Notes:
    Look to set clock phase and polarity to match the sensor's SPI timing
    UCA1CTLW0 &= ~(UCCKPL | UCCKPH);       // CPOL = 0, CPHA = 0 -> MODE 0

    To do:
    Configure ACCL by writing to its registers
    Configure INT behaviour on ACCL to Polarity = Active HIGH, Drive = PUSH PULL, Mode= Latched. 
    Must be done after initAccl() via SPI writes.
    */
}


void initBMP(){
    // Configure GPIO for SPI mode
    BMP_MOSI.function(PinFunction::Primary);
    BMP_MISO.function(PinFunction::Primary);
    BMP_CLK.function(PinFunction::Primary);
    BMP_CS.toOutput().setHigh(); // Chip select
    gpioUnlock();

    // Initialise BMP's SPI
    BMP_SPI.init(SpiMode::MODE_0(), ClockSource::Smclk, 1);
}

void configureBMP(){
    bool dataReadyInterrupt = false;
    bool pressureDataReady  = false;
    
    uint8_t intStatus;  
    uint8_t status;

    // "INT_CTRL" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x19 & 0x7F);         // AND-ing reg 0x19 with 0x7F forces bit 7 = 0, indicating a write operation.
        BMP_SPI.writeByte(0x76);   // 0111 0110 --> (right to left: push-pull, active_high INT pin, enable latching, disable FIFO watermark, enable FIFO interrupt, barometer interrupt pin high, enable temperature/pressure data ready) 
        BMP_SPI.flush();
    BMP_CS.setHigh();
    
    // "IF_CONF" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1A & 0x7F);   
        BMP_SPI.writeByte(0x00);   // bit0 = 0 --> SPI 4-wire mode
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // "PWR_CTRL" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1B & 0x7F);   // register address (write)  "0x1B" = "0001 1011". To the BMP390, this means PWR_CTRL Register
        BMP_SPI.writeByte(0x31);   // register value --> 0011 0001 --> (right to left: enable pressure sensor, disable temperature sensor, 0,0 normal mode, 0, 0)
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // "OSR" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1C & 0x7F);   
        BMP_SPI.writeByte(0x03);   // 0000 0011 --> 8x oversampling pressure measurement
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // "ODR" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1D & 0x7F);   
        BMP_SPI.writeByte(0x02);   // 0x02 --> 20ms sampling period
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // "INT_STATUS" register 
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x11 | 0x80);   // Reading register. Register is cleared.
        intStatus = BMP_SPI.readByte();   // Sends dummy byte to clock data out
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // Checking for data ready interrupt in the INT_STATUS register
    if (intStatus & (1 << 3)) {           // 1<<3  == 0000 1000
        dataReadyInterrupt = true;
    }
    
    // "STATUS" register 
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x03 | 0x80);   // Reading STATUS
        status = BMP_SPI.readByte();
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // Checking if pressure sensor data is ready 
    if (status & (1 << 5)) {
        pressureDataReady = true;
    }
}


/* -------------------------------ADC------------------------------- */

volatile uint16_t adcResult;

#define ADCMAX 4095.00 //Correlating to 12 bits
#define VREF  2.5
#define RESISTOR 10000
#define BETA 3380 
#define RTHERM 10000
#define T0_K 298.15 //25 degrees Celsius in Kelvin 

#define CURRENT_MAX 2.0   // Max current in Amps
#define R_SENSE 0.1       // Shunt resistor in Ohms
// Values need to be checked 

#define ADC_CHAM_THERM   ADCINCH_8    // P5.0 / A8
#define ADC_BAT_THERM    ADCINCH_10   // P5.2 / A10
#define ADC_CUR_SENSE    ADCINCH_11   // P5.3 / A11

void initADC () {
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

    ADCIE |= ADCIE0;                          // Enable interrupts 

    ADCCTL0 = ADCSHT_4 | ADCON;                 // sample and hold for 64 clock cycles, enable ADC.

    __delay_cycles(100);						// Wait for reference to settle

    PM5CTL0 &= ~LOCKLPM5;                       // Unlock GPIO / LPMx.5 Lock Bit 
}

void startADC(uint8_t channel)
{
    //ADCCTL0 &= ADCENC_0;                // Stop ADC (Using "_0 / _1" can be problematic as they're meant to be assignemts)
    ADCCTL0 &= ~ADCENC;                  // Clear the bit that enables ADC conversion. AKA stop ADC

    ADCMCTL0 = ADCSREF_1 | channel;     // Select input channel + reference  1: 001b = {VR+ = VREF and VR– = AVSS}

    ADCCTL0 |= ADCENC | ADCSC;          // Enable + start conversion
}

// Interrupt that read ADC memory
    #pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    switch (__even_in_range(ADCIV, ADCIV__ADCIFG0))
    {
        case ADCIV__ADCIFG0:
            adcResult = ADCMEM0;   // Read ADC result
            LPM0_EXIT;             // Wakes up CPU
            break;
        default:
            break;
    }
}

int16_t tempConversion(int16_t adcValue ) {
	//converting adc count to voltage 
	double VoltageTemp = (adcValue / ADCMAX) * VREF; 
	
	//calculate thermistor resistance 
	double thermistorResistance = RESISTOR * (VREF - VoltageTemp)/ VoltageTemp; // Should add error checking. ie dividing by 0

	//apply beta formula 
	double invertedTemp = (1.0/T0_K) + (1.0/ BETA) * log(thermistorResistance/RTHERM); // Log not defined. Will need to find a way to allow this without heavy resource consumption
	double tempInKelvin = 1/invertedTemp; 
	
	//convert to Celsius 
	double tempInCelsius = tempInKelvin - 273.15;

	return (int16_t)tempInCelsius;
}

int main(void) {  
    
    initADC(); 
     
    __enable_interrupt();   
    
    uint16_t chamThermADC;
    uint16_t batThermADC; 
    uint16_t curSenseADC;

    P5DIR |= BIT3;                   // Set P5.3 as output

    while (1)
    {
        // Chamber thermistor
        startADC(ADC_CHAM_THERM);
        LPM0;                                 //Code only continues when interrupt handler has collected all data
        chamThermADC = adcResult;

        // Battery thermistor
        startADC(ADC_BAT_THERM);
        LPM0;
        batThermADC = adcResult;

        // Current sense
        startADC(ADC_CUR_SENSE);
        LPM0;
        curSenseADC = adcResult;        
        double current = ((double)curSenseADC / ADCMAX) * VREF / R_SENSE;       // Convert ADC to current
        if (current > CURRENT_MAX) {
            P5OUT &= ~BIT3;              // Turn off current (port 5, pin 3) if it exceeds a value 
        } else {
            P5OUT |= BIT3;               // Turns back on if current return to a safe low value
        }
    

        // Convert temperatures
        int16_t chamTempC = tempConversion(chamThermADC);
        int16_t batTempC  = tempConversion(batThermADC);

        // Set sampling frequency
        __delay_cycles(800000);   // 10 Hz sampling
        // For 8 MHz: Delay cycles: 800000 = 10Hz, 80000 = 100Hz, 8000000 = 1Hz
    }

}