#ifndef BLOCKING_I2C_HPP
#define BLOCKING_I2C_HPP

#include <stdint.h>
#include <msp430.h>

#include "util.hpp"

enum class I2cClockSource {
    ExternalUClki = UCSSEL__UCLKI,
    Smclk = UCSSEL__SMCLK,
    Aclk = UCSSEL__ACLK,
};

enum class DeglitchTime {
    _50ns   = UCGLIT_0,
    _25ns   = UCGLIT_1,
    _12_5ns = UCGLIT_2,
    _6_25ns = UCGLIT_3,
};

enum class I2cDirection {
    Transmit,
    Receive,
};

struct I2cOperation {
    I2cDirection dir;
    uint8_t* data;
    uint16_t len;
};

#define I2C_B0 &UCB0CTLW0, &UCB0CTLW1, &UCB0BRW, &UCB0STATW, &UCB0RXBUF, &UCB0TXBUF, &UCB0I2CSA, &UCB0IE, &UCB0IFG, &UCB0IV
#define I2C_B1 &UCB1CTLW0, &UCB1CTLW1, &UCB1BRW, &UCB1STATW, &UCB1RXBUF, &UCB1TXBUF, &UCB1I2CSA, &UCB1IE, &UCB1IFG, &UCB1IV

// Unfortunately the register offsets are different between USCIA and USCIB, so we can't just take the base address and add offsets.
template<
    volatile uint16_t* CTLW0,
    volatile uint16_t* CTLW1,
    volatile uint16_t* BRW,
    volatile uint16_t* STATW,
    volatile uint16_t* RXBUF,
    volatile uint16_t* TXBUF,
    volatile uint16_t* SA,
    volatile uint16_t* IE,
    volatile uint16_t* IFG,
    volatile uint16_t* IV
>
struct I2cMaster {
    private:
    /// Listen for errors. For an I2C single master this is really just if we receive a NACK.
    static bool err_occurred() {
        // basically, if (NACK flag is raised), {}
        if (IS_SET(IFG, UCNACKIFG)) {  // IFG is Interrupt Flag Register. UCNACKIFG is NACK which is a flag raised if the slave does not ack. The master must react with either a STOP condition or a repeated START condition.
            
            // Send stop and wait for it to finish

            // Generate a STOP by setting UCTXSTP bit to "1"
            SET_BITS(CTLW0, UCTXSTP_1);  // UCTXSTP_1 sets the UCTXSTP bit in the CTLW0 reg  to "1".  Eqv to using a raw bit mask like 0000 0000 0000 0100
            
            // busy-wait loop that waits until the STOP condition has finished.
            while(IS_SET(CTLW0, UCTXSTP)); // While the STOP is being transmitted on the I2C bus: UCTXSTP remains 1, and is cleared when the STOP has finished generating.
            return true;
        }
        return false;
    }

    /// Edge case for zero byte writes (i.e. just the slave address byte)
    /// Address already configured by writeBytes().
    static int16_t zeroByteWrite() {
        SET_BITS(CTLW0, UCTXSTT_1);
        SET_BITS(CTLW0, UCTXSTP_1);
        *TXBUF = 0; // Bus stalls if nothing Tx, even if a stop is scheduled
        // Wait for both STT and STP to go low, monitoring error flags
        while (IS_SET(CTLW0, UCTXSTT | UCTXSTP)) {
            if (err_occurred()) {
                return 0;
            }
        }
        if (err_occurred()) {
            return 0;
        }
        return -1;
    }


    // Here, len is the length of data we want to write. function call below
    //              writeBytes(address,         op.data,        op.len,     sendStart,      sendStop);
    static int16_t writeBytes(uint8_t address, uint8_t buf[], uint16_t len, bool sendStart, bool sendStop) {
        // Clear old flags, set slave address and Tx mode
        *IFG = 0;
        *SA = address;
        SET_BITS(CTLW0, UCTR__TX);

        // Handle zero byte writes separately
        if (len == 0) {
            return zeroByteWrite();
        }

        if (sendStart) {
            SET_BITS(CTLW0, UCTXSTT_1);
        }

        // Write all the bytes to the Tx buffer
        #pragma diag_suppress 1544 // Suppress loop counting up remark. We can't count down here. 
        #pragma diag_suppress 1543 // Suppress DMA remark. FR2355 has no DMA.

        // write bytes itself
        for (uint16_t i = 0; i < len; i++) {
            // Wait for Tx buffer to be empty. Listen for errors while waiting.
            while (1) {
                if (err_occurred()) {
                    // Subtract index because buffer fills before any NACKs come through
                    if (i > 0) {
                        return i-1;
                    } else {
                        return 0;
                    }
                }
                if (IS_SET(IFG, UCTXIFG)) {
                    // Tx buffer empty, can send next byte
                    break; // breaks out of the while loop
                }
            }
            *TXBUF = buf[i]; // bytes are physically written here.
        }
        #pragma diag_default 1544
        #pragma diag_default 1543

        // Check errors for last byte
        while (!IS_SET(IFG, UCTXIFG)) {  // UCTXIFG0 is set when UCBxTXBUF is empty in master mode or in slave mode,
                                         // UCTXIFG = 1 → TX buffer is empty and ready for the next byte
                                         // UCTXIFG = 0 → TX buffer still busy sending the previous byte
            if (err_occurred()) {
                if (len > 0) {
                    return len-1;
                } else {
                    return 0;
                }
            }
        }


        // TO READ
        if (sendStop) {
            // Send stop, and listen again for possible errors from that last byte
            SET_BITS(CTLW0, UCTXSTP_1); // Stop generated. Set UCTXSTP bit in CTLW0 reg to high to generate a stop
            
            while(IS_SET(CTLW0, UCTXSTP)); // Wait for STOP completion???? as UCTXSTP clears when STOP condition is sent.
            
            while (IS_SET(STATW, UCBUSY)) {
                if (err_occurred()) {
                    return len;
                }
            }   
        }
        return -1;
    }

    // TO READ


    /// Read some number of bytes from the slave. 
    /// Returns -1 if OK, otherwise returns the byte where a NACK was received.
    /// (i.e. 0 = slave address byte, 1 = first data byte, etc.)
    static int16_t readBytes(uint8_t address, uint8_t buf[], uint16_t len, bool sendStart, bool sendStop) {
        // Clear old flags, set slave address and Rx mode
        *IFG = 0;
        *SA = address;
        CLEAR_BITS(CTLW0, UCTR);

        if (sendStart) {
            SET_BITS(CTLW0, UCTXSTT_1);
            while(IS_SET(CTLW0, UCTXSTT));
        }

        #pragma diag_suppress 1544 // Suppress loop counting up remark. We can't count down here. 
        #pragma diag_suppress 1543 // Suppress DMA remark. The Fr2355 doesn't have DMA. 
        for (uint16_t i = 0; i < len; i++) {
            if (sendStop & (i == len - 1)) {
                // Schedule a stop before we read the last byte
                SET_BITS(CTLW0, UCTXSTP_1);
            }
            // Monitor error flags while waiting for the Rx flag
            while (1) {
                if (err_occurred()) {
                    return i;
                }
                if (IS_SET(IFG, UCRXIFG)) {
                    break;
                }
            }
            buf[i] = *RXBUF;
        }
        #pragma diag_default 1544
        #pragma diag_default 1543 

        if (sendStop) {
            while(IS_SET(CTLW0, UCTXSTP));
        }
        return -1;
    }

    
    // To Read
    public:
    /// Initialise an I2C peripheral into master mode. This function assumes that the GPIO pins for SCL and SDA have been correctly configured.
    /// Currently only supports 7-bit addressing.
    static void init(
        I2cClockSource clockSource,
        uint16_t prescaler,
        DeglitchTime glitchTime = DeglitchTime::_50ns) {
        
        uint16_t clk    = static_cast<uint16_t>(clockSource);
        uint16_t glitch = static_cast<uint16_t>(glitchTime);
        SET_BITS(CTLW0, UCSWRST);

        //     |     7-bit addressing    | Single master |  master mode  |     I2C mode       | clock source | 
        *CTLW0 = UCA10_0 | UCSLA10__7BIT | UCMM__SINGLE  | UCMST__MASTER | UCMODE_3 | UCSYNC  |      clk     | UCSWRST;
        
        //       | No timeout | glitch filter
        *CTLW1 =    UCCLTO_0  | glitch;

        *BRW = prescaler;

        CLEAR_BITS(CTLW0, UCSWRST);
    }

    /// Perform an arbitrary number of read and write operations to the device at `address` in a single I2C transaction.
    /// A Start is sent before the first operation, and between operations of dissimilar types (i.e. Read -> Write causes a repeated start).
    /// A Stop is sent after the last operation.
    /// Returns -1 if no NACKs were received, otherwise returns the byte number from which the NACK was received.

    // Here, 'len' in the number of operations (write, read, mix).
    static int16_t transaction(uint8_t address, I2cOperation operations[], uint16_t len) {
        // Total number of bytes sent so far. Used to correctly offset the byte counter in case of an error.
        int16_t bytesSent = 0;
        #pragma diag_suppress 1544 // Suppress loop counting up remark. We can't count down here. 
        for (uint16_t i = 0; i < len; i++) {
            bool sendStart = false;
            // Send start only if this is the first op, or if the previous was a different type, 
            // e.g. op[0] is a Read and op[1] is a Write

            // first operation? || direction of prev operation != direction of current operation
            if ((i == 0) || (operations[i-1].dir != operations[i].dir)) {
                sendStart = true;
            }

            // set sendstop if this is the last operation
            bool sendStop = (i == len-1);
            
            I2cOperation op = operations[i]; // Set current operation as operations[i]
            int16_t result;

            // Call writeBytes or readBytes depending on 'direction' of the current operation
            if (op.dir == I2cDirection::Transmit) {
                result = writeBytes(address, op.data, op.len, sendStart, sendStop);
            } else {
                result = readBytes(address, op.data, op.len, sendStart, sendStop);
            }

            // If err, make the count match the total number of bytes sent
            if (result != -1) {
                return bytesSent + result;
            }

            bytesSent += op.len;
        }
        #pragma diag_default 1544 
        return -1;
    }

    /// Send a Start, write `len` bytes to the device at `address`, then send a Stop.
    /// Returns -1 if no NACKs were received, otherwise returns the byte number from which the NACK was received.
    static int16_t write(uint8_t address, uint8_t buf[], uint16_t len) {

        // Note: I2cOperation is a struct with dir (direction), data, and length
        // Create one operation 'op' to write 'len' bytes from buf
        I2cOperation op = {I2cDirection::Transmit, buf, len};

        // Create an array 'ops' with one element, and intilise with 'op' -- an array of 1 operation.
        I2cOperation ops[1] = { op };
        return transaction(address, ops, 1); // Note, we pass in an array 'ops' because func 'transaction' expects an array
        // '1' as there is operation (write)
    }

    /// Sent a Start, read `len` bytes from the device at `address`, then send a Stop.
    /// Returns -1 if no NACKs were received, otherwise returns the byte number from which the NACK was received.
    static int16_t read(uint8_t address, uint8_t buf[], uint16_t len) {
        I2cOperation op = {I2cDirection::Receive, buf, len};
        I2cOperation ops[1] = { op };
        return transaction(address, ops, 1);
    }

    /// Send a Start, write `sendLen` bytes to the device at `address`, send a repeated start and read `recvLen` bytes, then send a Stop.
    /// Commonly used for operations like reading register values, etc.
    /// Returns -1 if no NACKs were received, otherwise returns the byte number from which the NACK was received.
    static int16_t write_read(uint8_t address, uint8_t send[], uint16_t sendLen, uint8_t recv[], uint16_t recvLen) {
        I2cOperation ops[2] = { 
            {I2cDirection::Transmit, send, sendLen},
            {I2cDirection::Receive,  recv, recvLen},
        };
        return transaction(address, ops, 2);
    }
};

#endif
