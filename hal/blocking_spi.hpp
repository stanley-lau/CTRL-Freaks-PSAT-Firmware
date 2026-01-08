#ifndef BLOCKING_SPI_HPP
#define BLOCKING_SPI_HPP

#include <msp430.h>
#include <stdint.h>

#include "util.hpp"
#include "gpio.hpp"
#include "eusci.hpp"

enum ClockPhase {
    CaptureOnFirst  = UCCKPH_1,
    CaptureOnSecond = UCCKPH_0,
};

enum ClockPolarity {
    IdleHigh = UCCKPL__HIGH,
    IdleLow  = UCCKPL__LOW,
};

struct SpiMode {
    ClockPhase phase;
    ClockPolarity polarity;

    inline static constexpr SpiMode MODE_0() { return {CaptureOnFirst,  IdleLow};  }
    inline static constexpr SpiMode MODE_1() { return {CaptureOnSecond, IdleLow};  }
    inline static constexpr SpiMode MODE_2() { return {CaptureOnFirst,  IdleHigh}; }
    inline static constexpr SpiMode MODE_3() { return {CaptureOnSecond, IdleHigh}; }
};

// 4-pin SPI modes not yet implemented due to USCI50


#define SPI_A0 &UCA0CTLW0, &UCA0BRW, &UCA0STATW, &UCA0RXBUF, &UCA0TXBUF, &UCA0IE, &UCA0IFG, &UCA0IV
#define SPI_A1 &UCA1CTLW0, &UCA1BRW, &UCA1STATW, &UCA1RXBUF, &UCA1TXBUF, &UCA1IE, &UCA1IFG, &UCA1IV
#define SPI_B0 &UCB0CTLW0, &UCB0BRW, &UCB0STATW, &UCB0RXBUF, &UCB0TXBUF, &UCB0IE, &UCB0IFG, &UCB0IV
#define SPI_B1 &UCB1CTLW0, &UCB1BRW, &UCB1STATW, &UCB1RXBUF, &UCB1TXBUF, &UCB1IE, &UCB1IFG, &UCB1IV

// Unfortunately the register offsets are different between USCIA and USCIB, so we can't just take the base address and add offsets.
template<
    volatile uint16_t* CTLW0,
    volatile uint16_t* BRW0,
    volatile uint16_t* STATW,
    volatile uint16_t* RXBUF,
    volatile uint16_t* TXBUF,
    volatile uint16_t* IE,
    volatile uint16_t* IFG,
    volatile uint16_t* IV
>
struct SpiMaster {
    private:
    static bool txBufFull() {
        return !IS_SET(IFG, UCTXIFG);
    }

    static bool rxBufEmpty() {
        return !IS_SET(IFG, UCRXIFG);
    }

    public:
    /// Initialise an SPI peripheral. This function assumes that the GPIO pins for MOSI, MISO, and SCK have been correctly configured.
    static void init(
        const SpiMode& mode, 
        ClockSource clock, 
        uint16_t prescaler, 
        BitOrder order = BitOrder::MsbFirst, 
        PacketLength length = PacketLength::_8Bit) {
            
        SET_BITS(CTLW0, UCSWRST);

        //                                          |Master |3-Pin mode|                |   Reserved  |STE unused|
        *CTLW0 = mode.phase | mode.polarity | order | UCMST | UCMODE_0 | UCSYNC | clock | 0b0000 << 2 | UCSTEM_0 | length | UCSWRST;
        *BRW0 = prescaler;

        CLEAR_BITS(CTLW0, UCSWRST);
    }

    /// Send and recieve a single byte. Does not manage the chip select pin.
    static uint8_t transferByte(uint8_t data) {
        while(txBufFull());
        *TXBUF = data;
        while(rxBufEmpty());
        return *RXBUF;
    }

    /// Send a single byte, discarding the received byte. Does not manage the chip select pin.
    static void writeByte(uint8_t data) {
        transferByte(data);
    }

    /// Send a single dummy byte (0x00) so the slave can send a byte. Does not manage the chip select pin.
    static uint8_t readByte() {
        return transferByte(0x00);
    }

    /// Block until the SPI bus has finished sending. This should be called before deasserting a chip select pin.
    static void flush() {
        while( IS_SET(STATW, UCBUSY) );
    }

    /// Transfers some number of bytes in a single SPI transaction. Manages the chip select automatically.
    /// If the recieve buffer is longer than the send buffer then once the send buffer is exhausted dummy bytes (0x00) are sent.
    /// If the send buffer is longer than the recieve buffer then once the recieve buffer is full any further received bytes are discarded. 
    template<typename Pin>
    static void transfer(const uint8_t sendBuf[], uint16_t sendLen, uint8_t recvBuf[], uint16_t recvLen, Pin& chipSel) {
        static_assert(Gpio::isPinType<Pin>(), "chipSel must be of type Pin<...>");
        // Determine which buffer is shorter
        uint16_t min;
        if (sendLen <= recvLen) {
            min = sendLen;
        } else {
            min = recvLen;
        }

        chipSel.setLow();

        #pragma diag_suppress 1544 // Suppress loop counting up remarks. We can't count down here. 
        // While the send and recieve buffers both exist, transfer
        for (uint16_t i = 0; i < min; i++) {
            recvBuf[i] = transferByte(sendBuf[i]);
        }

        // Deal with whichever buffer is larger (if any)
        if (recvLen > sendLen) {
            for (uint16_t i = min; i < recvLen; i++) {
                recvBuf[i] = readByte();
            }
        } else if (sendLen > recvLen) {
            for (uint16_t i = min; i < sendLen; i++) {
                writeByte(sendBuf[i]);
            }
        }
        #pragma diag_default 1544

        flush();
        chipSel.setHigh();
    }

    /// Transfers some number of bytes in a single SPI transaction. 
    /// Uses a single buffer, the contents of which are replaced with the received bytes.
    /// Manages the chip select automatically.
    template<typename Pin>
    static void transferInPlace(uint8_t buf[], uint16_t bufLen, Pin& chipSel) {
        transfer(buf, bufLen, buf, bufLen, chipSel);
    }

    /// Writes some number of bytes in a single SPI transaction. Manages the chip select automatically.
    template<typename Pin>
    static void write(const uint8_t sendBuf[], uint16_t sendLen, Pin& chipSel) {
        transfer(sendBuf, sendLen, nullptr, 0, chipSel);
    }

    /// Reads some number of bytes in a single SPI transaction. Manages the chip select automatically.
    template<typename Pin>
    static void read(uint8_t recvBuf[], uint16_t recvLen, Pin& chipSel) {
        transfer(nullptr, 0, recvBuf, recvLen, chipSel);
    }
};

#endif
