#ifndef EUSCI_HPP
#define EUSCI_HPP

#include <msp430.h>

// Options common to more than one EUSCI protocol (UART / SPI / I2C)

enum BitOrder {
    MsbFirst = UCMSB_1,
    LsbFirst = UCMSB_0,
};

enum PacketLength {
    _7Bit = UC7BIT__7BIT,
    _8Bit = UC7BIT__8BIT,
};

enum ClockSource {
    Aclk  = UCSSEL__ACLK,
    Smclk = UCSSEL__SMCLK,
};

#endif
