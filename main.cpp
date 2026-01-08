#include <driverlib.h>
#include <msp430.h>
#include <stdint.h>
#include "util.h"
#include "hal/blocking_spi.hpp"

#define PWM_PERIOD 500

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
void setCoilPWM(uint8_t duty_cyle){
    // PWM Coil P5.1 on schematic. 
    // duty_cyle must be a positive integer between 0 and 100;
    // SMCLK is used at 1Mhz

    if (duty_cyle > 100) {
        duty_cyle = 100;
    }
    
    // Update Duty cycle.
    TB2CCR2 = (TB2CCR0 * duty_cyle) / 100;      // Set LOW when TB0CCR1 is reached
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


int main(void) { 

    /*
    may need to write code for this init. 
    clock_init();
    gpio_init();
    */

    // ADC Channel 
    ADCMCTL0 = ADCSREF_1 | ADCINCH_8;      // Chamber thermistor (P5.0 / A8)
    
    // Battery thermistor (P5.2 / A10)
    // Current sense (P5.3 / A11)

    // ADC Conversion Memory Control Register 
    // Select reference 1: 001b = {VR+ = VREF and VR– = AVSS}
    // Select input channel for thermistor and sense 

    ADCCTL1 |= ADCDIV_2 + ADCSHP_0;               // Divide input clock by 3
    // ADC Control Register 1
    // ADC clock divider. Divide 24MHz by 3 to get 8MHz (typical)
    // ADC sample-and-hold pulse-mode select. Selects sampling signal's source to be sample-input signal directly.

    ADCCTL2 &= ~ADCRES;                         // Clear bits
    // ADC Control Register 2 
    // Usage of not operators to clear the resolution value 

    ADCCTL2 |= ADCRES_2;                        // 12-bit resolution
    // Sets ADC resolution, number defines resolution of the conversion result 

    ADCIE |= ADCIE0_1;                            // Enable interrupts
    // ADC Interrupt Enable Register
    // Enable the interrupt request for a completed ADC conversion 
    //ADC_enableInterrupt(ADC_BASE, ADCIE0_1);

    ADCCTL0 |= ADCSHT_4 + ADCON;                // sample and hold for 64 clock cycles, enable ADC.
    // AC Control Register 0 
    // Define number of ADCCLK cycles in sampling period -> 4 = 64 cycles 
    // Turn ADC on 

    __delay_cycles(100);						// Wait for reference to settle
    // Delay for 100ms to wait for reference to settle 

    PM5CTL0 &= ~LOCKLPM5;                       // Unlock GPIO
    // Power mode 5 control register 0
    // LPMx.5 Lock Bit

    
    }
