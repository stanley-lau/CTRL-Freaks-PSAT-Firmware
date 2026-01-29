#ifndef SPI_H
#define SPI_H

/** Function Prototypes **/
void spi_B1_init(void);
void spi_B1_send_byte(uint8_t byte);
uint8_t spi_B1_receive_byte(void);
uint8_t spi_B1_transfer_byte(uint8_t byte);
void spi_B1_flush(void);

#endif
