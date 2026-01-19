#include "accl.hpp"
#include "pin_mappings.hpp"
#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"
#include <math.h>

volatile bool ACCLReadyFlag = false;

// Accl SPI pins are routed incorrectly. Cannot be resolved in software. Function written assuming routing is correctly
// InitACCL initialises SPI functionality 
void InitACCL() {
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
    P6IES &= ~BIT0;                         // Rising edge trigger (active HIGH)
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

void ConfigureACCL() {
    //PWR_MGMT0 register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x1F & 0x7F);  
        ACCL_SPI.writeByte((1 << 0) | (1 << 1));         // Make 0, 1 = 1 --> Places accelerometer in Low Noise (LN) Mode
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
    __delay_cycles(500);                  // Small safety delay

    //INT_CONFIG register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x06 & 0x7F);  // Bit 7 (MSB) = 0 --> write mode 
        ACCL_SPI.writeByte(0x3F);         // = 0011 1111 = INT1: active high, push-pull, latched, INT2: active high, push-pull, latched
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
    __delay_cycles(500);               

    //INT_SOURCE0 register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x2B & 0x7F);   
        ACCL_SPI.writeByte(1 << 3);       // Make bit 3 = 1 --> Data ready interrupt routed to INT1
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
    __delay_cycles(500);          

    //WOM_CONFIG register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x27 & 0x7F);   
        ACCL_SPI.writeByte(1 << 0);       // Make bit 0 = 1 --> Wake On Motion (WOM) enabled 
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
    __delay_cycles(500); 

    //INT_SOURCE4 register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x2E & 0x7F);   
        ACCL_SPI.writeByte((1 << 0) | (1 << 1) | (1 << 2));       // Make bit 0, 1, 2 = 1 --> X, Y, Z axis WOM routed to INT2
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
    __delay_cycles(500);

    //INT_CONFIG0 register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x04 & 0x7F);   
        ACCL_SPI.writeByte((1 << 5) | (0 << 4));       // Make bit 5 = 1, bit 4 = 0 --> Data ready interrupt cleared on sensor register read
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
    __delay_cycles(500);

    //ACCEL_CONFIG0 register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x21 & 0x7F);   
        ACCL_SPI.writeByte((0 << 5) | (0 << 6));       // Make bit 5 and 6 = 0 --> Accelerometer output = ±16g full-scale
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
    __delay_cycles(500);

}

void UpdateACCLStatus() {
    uint8_t int_status;
    uint8_t int_status_drdys;

    // "INT_STATUS_DRDYS" register 
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x39 | 0x80);    // ORing with 0x08 forces bit 7 = 1 --> reading register mode. Register is cleared.
        int_status_drdys = ACCL_SPI.readByte();   // Sends dummy byte to clock data out
        ACCL_SPI.flush();
    ACCL_CS.setHigh();

    // Checking for data ready interrupt in the INT_STATUS register
    if (int_status_drdys & (1 << 0)) {      // Bit 0 is automatically sets to 1 when a Data Ready interrupt is generated
        ACCLReadyFlag = true;               // Data is ready. After register has been read, bit 0 is set to 0. 
    }
}

// Check with data sheet on how this should be read and if the register's auto increment.
// Return the acceleration magnitude 
float ReadACCL() {
    //xh indicates high byte in the x axis. xl = low byte in the x axis. etc. 
    uint8_t xh, xl, yh, yl, zh, zl;
    //Raw acceleration values in the x, y, z directions 
    int16_t ax_raw, ay_raw, az_raw;
    float ax, ay, az;

    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x0B | 0x80);  // Reading ACCEL_DATA_X1 register 
        xh = ACCL_SPI.readByte();
        ACCL_SPI.writeByte(0x0C | 0x80);  // Reading ACCEL_DATA_X0 register 
        xl = ACCL_SPI.readByte();
        ACCL_SPI.writeByte(0x0D | 0x80);  // Reading ACCEL_DATA_Y1 register 
        yh = ACCL_SPI.readByte();
        ACCL_SPI.writeByte(0x0E | 0x80);  // Reading ACCEL_DATA_Y0 register 
        yl = ACCL_SPI.readByte();
        ACCL_SPI.writeByte(0x0F | 0x80);  // Reading ACCEL_DATA_Z1 register 
        zh = ACCL_SPI.readByte();
        ACCL_SPI.writeByte(0x10 | 0x80);  // Reading ACCEL_DATA_Z0 register 
        zl = ACCL_SPI.readByte();
        ACCL_SPI.flush();
    ACCL_CS.setHigh();

    // Combine high + low bytes of x, y, z axis 
    ax_raw = (int16_t)((xh << 8) | xl);
    ay_raw = (int16_t)((yh << 8) | yl);
    az_raw = (int16_t)((zh << 8) | zl);

    // Convert to g using ±16g full-scale
    ax = ax_raw / 2048.0f;
    ay = ay_raw / 2048.0f;
    az = az_raw / 2048.0f;

    // Magnitude of acceleration vector
    return sqrtf(ax*ax + ay*ay + az*az);
}

void DisableACCL() {
    //PWR_MGMT0 register
    ACCL_CS.setLow();
        ACCL_SPI.writeByte(0x1F & 0x7F);  
        ACCL_SPI.writeByte(0x00);         // Turns accelerometer and gyroscope off
        ACCL_SPI.flush();
    ACCL_CS.setHigh();
}
