#ifndef BLOCKING_UART_HPP
#define BLOCKING_UART_HPP

#include <msp430.h>
#include <stdint.h>

#include "util.hpp"
#include "gpio.hpp"
#include "eusci.hpp"

enum StopBits {
    OneStop = UCSPB_0,
    TwoStop = UCSPB_1,
};

enum class ParityBit {
    Disabled,
    Even,
    Odd,
};

struct BaudConfig {
    uint16_t ucbr;
    uint8_t  ucbrs;
    uint8_t  ucbrf;
    bool ucos16;

    /// Returns a BaudConfig for 9600 baud when SMCLK at the default speed (1.048576 MHz)
    static BaudConfig defaultSmclk9600Baud() {
        return {ucos16: true, ucbr: 6, ucbrf: 13, ucbrs: 0x22};
    }

    /// Returns a BaudConfig for 115200 baud when SMCLK at the default speed (1.048576 MHz)
    static BaudConfig defaultSmclk115200Baud() {
        return {ucos16: false, ucbr: 9, ucbrf: 0, ucbrs: 0x08};
    }
};


#define UART_A0 &UCA0CTLW0, &UCA0BRW, &UCA0MCTLW, &UCA0STATW, &UCA0RXBUF, &UCA0TXBUF, &UCA0IE, &UCA0IFG, &UCA0IV
#define UART_A1 &UCA1CTLW0, &UCA1BRW, &UCA1MCTLW, &UCA1STATW, &UCA1RXBUF, &UCA1TXBUF, &UCA1IE, &UCA1IFG, &UCA1IV

// Unfortunately the register offsets are different between USCIA and USCIB, so we can't just take the base address and add offsets.
template<
    volatile uint16_t* CTLW0,
    volatile uint16_t* BRW,
    volatile uint16_t* MCTLW,
    volatile uint16_t* STATW,
    volatile uint16_t* RXBUF,
    volatile uint16_t* TXBUF,
    volatile uint16_t* IE,
    volatile uint16_t* IFG,
    volatile uint16_t* IV
>
struct Uart {
    private:
    static uint16_t ucpen(ParityBit parity) {
        switch (parity) {
            case ParityBit::Disabled: return UCPEN_0; 
            case ParityBit::Even:
            case ParityBit::Odd:      
            default:                  return UCPEN_1;
        }
    }
    static uint16_t ucpar(ParityBit parity) {
        switch (parity) {
            case ParityBit::Even:     return UCPAR__EVEN;
            case ParityBit::Disabled: // Don't care 
            case ParityBit::Odd:      
            default:                  return UCPAR__ODD;
        }
    }

    public:
    /// Initialise a UART peripheral. This function assumes that the GPIO pins for the Tx and/or Rx pins have been correctly configured.
    static void init(
        ClockSource clock, 
        const BaudConfig& baud,
        ParityBit parity,
        StopBits nStopBits = StopBits::OneStop,
        BitOrder order = BitOrder::LsbFirst,
        PacketLength length = PacketLength::_8Bit) {
        
        SET_BITS(CTLW0, UCSWRST);

        //                                                                  |        UART         |       | Misc intr flags | 
        *CTLW0 = ucpen(parity) | ucpar(parity) | order | length | nStopBits | UCMODE_0 | UCSYNC_0 | clock |   0b00000 << 1  | UCSWRST;
        *BRW = baud.ucbr;
        *MCTLW = uint16_t(baud.ucbrs) << 8 | (baud.ucbrf & 0x0F) << 4 | baud.ucos16;

        CLEAR_BITS(CTLW0, UCSWRST);
    }

    static bool txBufFull() {
        return (*IFG & UCTXIFG) == 0;
    }

    static bool rxBufEmpty() {
        return (*IFG & UCRXIFG) == 0;
    }

    /// Write a single byte over the UART channel.
    static void writeByte(uint8_t byte) {
        while (txBufFull());
        *TXBUF = byte;
    }

    /// Block until a single byte is received from the UART channel.
    static uint8_t readByte() {
        while (rxBufEmpty());
        return *RXBUF;
    }

    /// Block until any previously buffered bytes have been written to the UART channel.
    static void flush() {
        while( IS_SET(STATW, UCBUSY) );
    }

    /// Write `len` bytes over the UART channel.
    static void write(const uint8_t send[], uint16_t len) {
        #pragma diag_suppress 1544 // Suppress loop counting up remark. We can't count down here. 
        for (uint16_t i = 0; i < len; i++) {
            writeByte(send[i]);
        }
        #pragma diag_default 1544
    }

    /// Block until `len` bytes are received from the UART channel.
    static void read(uint8_t recv[], uint16_t len) {
        #pragma diag_suppress 1544 // Suppress loop counting up remark. We can't count down here. 
        for (uint16_t i = 0; i < len; i++) {
            recv[i] = readByte();
        }
        #pragma diag_default 1544
    }
};

#endif
