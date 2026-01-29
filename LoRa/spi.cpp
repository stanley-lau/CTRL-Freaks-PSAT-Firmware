#include <msp430.h>
#include <stdint.h>
#include "spi.hpp"
#include <stdbool.h>
#include <driverlib.h>

/** SPI Pins **/
#define CLK BIT5 //P4.5 - CLK (Output Line)
#define SOMI BIT7 //P4.7 - SOMI/MISO (Input Line)
#define SIMO BIT6 //P4.6 - SIMO/MOSI (Output Line)

void spi_B1_init(void){
    UCB1CTLW0 |= UCSWRST; //Software reset enabled

    //Master mode selected, Synchronous Mode, SPI mode 0, SMCLK, MSB first
    UCB1CTLW0 |= UCMST | UCSYNC__SYNC | UCCKPH_1 | UCSSEL__SMCLK | UCMSB_1; 

    UCB1CTLW0 &= ~UCMODE; // 3-pin SPI

    //CLK, SOMI, SIMO set to 2nd function (SPI)
    P4SEL1 &= ~(CLK | SOMI | SIMO); //P4SEL1 = 0
    P4SEL0 |= CLK | SOMI | SIMO; //P4SEL0 = 1

    UCB1CTLW0 &= ~UCSWRST; //Software reset disabled
}

static bool spi_B1_tx_buffer_empty(void){
    if(UCB1IFG & UCTXIFG){
        return true;
    } else {
        return false;
    }
}

static bool spi_B1_rx_buffer_empty(void){
    //Check if RX interrupt is pending
    if(!(UCB1IFG & UCRXIFG)){
        return true;
    } else {
        return false;
    }
}

void spi_B1_send_byte(uint8_t byte){ //sending byte, and discarding what was received
    while(!spi_B1_tx_buffer_empty());
    UCB1TXBUF = byte; //write to, data begins to transmit

    while(spi_B1_rx_buffer_empty());
    uint8_t dummy_read = UCB1RXBUF; //Read RX buffer to avoid overflow being set
}

uint8_t spi_B1_receive_byte(void){ //reading, and sending a dummy value
    while(!spi_B1_tx_buffer_empty());
    UCB1TXBUF = 0x00; //Send dummy byte so slave can respond

    while(spi_B1_rx_buffer_empty()); 
    return UCB1RXBUF;
}

uint8_t spi_B1_transfer_byte(uint8_t byte){ //sending and receiving
    while(!spi_B1_tx_buffer_empty());
    UCB1TXBUF = byte;

    while(spi_B1_rx_buffer_empty());
    return UCB1RXBUF;
}

void spi_B1_flush(void){
    while(UCB1STATW & UCBUSY);
}
