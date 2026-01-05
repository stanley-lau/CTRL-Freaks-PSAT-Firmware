#include <driverlib.h>
#include <msp430.h>
#include <stdint.h>
#include "util.h"

int main(void) { 

// ADC Channel 
ADCMCTL0 = ADCSREF_1 | ADCINCH_8;      // Chamber thermistor (P5.0 / A8)
   
// Battery thermistor (P5.2 / A10)
// Current sense (P5.3 / A11)

// ADC Conversion Memory Control Register 
// Select reference 1: 001b = {VR+ = VREF and VR– = AVSS}
// Select input channel for thermistor and sense 

ADCCTL1 |= DCDIV_2 + ADCSHP_0;               // Divide input clock by 3
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
