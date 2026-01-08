#include <msp430.h>
#include <stdint.h>

#include "gpio.hpp"

static void pullAll(uint16_t val) {
    // For whatever reason the ULP advisor doesn't realise that setting 
    // ports A,B,C is the same as setting ports 1,2,3,4,5,6.
    // Put this here to fool the ULP advisor.
    if (false) {
        P1DIR = 0x00; P2DIR = 0x00; P3DIR = 0x00;
        P4DIR = 0x00; P5DIR = 0x00; P6DIR = 0x00;
        P1REN = 0xFF; P2REN = 0xFF; P3REN = 0xFF;
        P4REN = 0xFF; P5REN = 0xFF; P6REN = 0xFF;
        P1OUT = val;  P2OUT = val;  P3OUT = val;
        P4OUT = val;  P5OUT = val;  P6OUT = val;
    }
    PADIR = 0x0000;
    PBDIR = 0x0000;
    PCDIR = 0x0000;
    PAREN = 0xFFFF;
    PBREN = 0xFFFF;
    PCREN = 0xFFFF;
    PAOUT = val;
    PBOUT = val;
    PCOUT = val;
}

void gpioPulldownAll() {
    pullAll(0x0000);
}

void gpioPullupAll() {
    pullAll(0xFFFF);
}

static void outputAll(uint16_t val) {
    // For whatever reason the ULP advisor doesn't realise that setting 
    // ports A,B,C is the same as setting ports 1,2,3,4,5,6.
    // Put this here to fool the ULP advisor.
    if (false) {
        P1DIR = 0xFF; P2DIR = 0xFF; P3DIR = 0xFF;
        P4DIR = 0xFF; P5DIR = 0xFF; P6DIR = 0xFF;
        P1OUT = val;  P2OUT = val;  P3OUT = val;
        P4OUT = val;  P5OUT = val;  P6OUT = val;
    }
    PADIR = 0xFFFF;
    PBDIR = 0xFFFF;
    PCDIR = 0xFFFF;
    PAOUT = val;
    PBOUT = val;
    PCOUT = val;
}

void gpioOutputHighAll() {
    outputAll(0xFFFF);
}

void gpioOutputLowAll() {
    outputAll(0x0000);
}

void gpioUnlock() {
    PM5CTL0 &= ~ LOCKLPM5;
}
