#include <driverlib.h>
#include <msp430.h>
#include <stdint.h>

int main(void) {
    WDTCTL = WDTPW + WDTHOLD;                 	// Stop watchdog timer

    // Intialise ADCs
}
