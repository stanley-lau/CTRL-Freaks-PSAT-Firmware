#ifndef CLOCK_HPP
#define CLOCK_HPP

#include <msp430.h>
#include <stdint.h>

/// Wait until the FLL has locked onto the target frequency
void waitForFllLock() {
    while ((CSCTL7 & FLLUNLOCK) != FLLUNLOCK_0);
}

/// Change the DCOCLK frequency using FLL stabilisation. See section 3.2.5 and 3.2.5.1 in the user manual.
/// Takes a target frequency between 33kHz and 24,000 kHz. It clamps the value between these.
/// The function either waits for the desired frequency to stabilise (default) or can return immediately.
void changeClockFreq(uint16_t targetFreqKHz, bool waitUntilStabilised = true) {
    if (targetFreqKHz > 24000) {
        targetFreqKHz = 24000;
    }
    else if (targetFreqKHz < 33) {
        targetFreqKHz = 33;
    }

    // The FLL ensures that the clock frequency (relative to the ref clock) remains constant even as voltage/temp varies.
    // The FLL can use either REFOCLK or the external XT1CLK as its reference clock. We assume 32768Hz in either case.
    const uint16_t fllRefFreq_hz = 32768;

    // Disable FLL. This is needed to prevent the FLL from acting as we make modifications to the clock setup
    __bis_SR_register(SCG0);

    // Clear DCO bits. These will be set by the FLL after it's re-enabled.
    CSCTL0 &= ~DCO;

    // Tell the FLL how many multiples of the ref clock we want
    CSCTL2 = ((uint32_t(targetFreqKHz)*1000 / fllRefFreq_hz) - 1);

    // Set coarse frequency regime
    CSCTL1 &= ~DCORSEL; // Clear all DCORSEL bits
    if (targetFreqKHz <= 1500) {
        CSCTL1 |= DCORSEL_0;
    }
    else if (targetFreqKHz <= 3000) {
        CSCTL1 |= DCORSEL_1;
    }
    else if (targetFreqKHz <= 6000) {
        CSCTL1 |= DCORSEL_2;
    }
    else if (targetFreqKHz <= 10000) {
        CSCTL1 |= DCORSEL_3;
    }
    else if (targetFreqKHz <= 14000) {
        CSCTL1 |= DCORSEL_4;
    }
    else if (targetFreqKHz <= 18000) {
        CSCTL1 |= DCORSEL_5;
    }
    else if (targetFreqKHz <= 22000) {
        CSCTL1 |= DCORSEL_6;
    }
    else if (targetFreqKHz <= 24000) {
        CSCTL1 |= DCORSEL_7;
    }

    // The FRAM on the MSP430 can only operate at 8MHz, so if the CPU goes faster we have to add some delays.
    // See section 5.3 of the MSP430FR2355 datasheet for details
    if (targetFreqKHz > 16000) {
        FRCTL0 = FRCTLPW | NWAITS_2;
    }
    else if (targetFreqKHz > 8000) {
        FRCTL0 = FRCTLPW | NWAITS_1;
    }
    else {
        FRCTL0 = FRCTLPW | NWAITS_0;
    }

    // Re-enable FLL
    __bic_SR_register(SCG0);

    if waitUntilStabilised {
        waitForFllLock();
    }
}

#endif
