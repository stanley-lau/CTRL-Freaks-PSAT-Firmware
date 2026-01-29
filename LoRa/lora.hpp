#ifndef LORA_H
#define LORA_H

#include "gpio.hpp"

/** Default Initialisation **/
#define LORA_MODE 0b10000000
#define FIFO_ADDR_PTR 0b00000000
#define LNA_BOOST_HF 0b00000011
#define AGC_AUTO_ON 0b00000100
#define OUTPUT_POWER 0xF //= 15, output power = 17dBm w/ LoRa, Pout=17-(15-OutputPower)
#define PA_BOOST 0b10000000

/** Frequency **/
#define HIGH_FREQ_MODE 0x00 //REG_OP_MODE Register

/** Device Mode Check **/
#define DEVICE_MODE_CHECK_MASK 0b00000111
#define DEVICE_SLEEP 0b00000000
#define DEVICE_STDBY 0b00000001
#define DEVICE_FSTX 0b00000010 //Frequency synthesis TX
#define DEVICE_TX 0b00000011
#define DEVICE_FSRX 0b00000100 //Frequency synthesis RX
#define DEVICE_RXCONTINUOUS 0b00000101
#define DEVICE_RXSINGLE 0b00000110
#define DEVICE_CAD 0b00000111 //Channel activity detection

/** Receiving Status Checks **/
#define REG_IRQ_FLAGS_CHECK_MASK 0b11100000
#define PAYLOAD_CRC_ERROR_CHECK_MASK 0b00100000
#define VALID_HEADER_CHECK_MASK 0b00010000
#define RX_TIMEOUT_CHECK_MASK 0b10000000
#define RX_DONE_CHECK_MASK 0b01000000
#define FLAGS_CHECK_VALID_RX_MASK 0xF0
#define CLEAR_RX_FLAGS 0b11110000

/** Transmitting **/
#define TX_DONE_CHECK_MASK 0b00001000

/** Setting Timeout **/
#define SYMBOL_BITS_BITMASK 0b0000001111111111
#define GET_LSB 0x00FF //To mask off first 8 MSBs in a 16 bit address/value

/* Setting frequency to 915MHz, Frf = 14991360*/
#define FRF_MSB 0xE4
#define FRF_MID 0xC0
#define FRF_LSB 0x00

/** Registers **/
#define REG_OP_MODE 0x01
#define REG_INVERT_IQ 0x33
#define REG_MODEM_CONFIG2 0x1E
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG3 0x26
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_LNA 0x0C
#define REG_PA_CONFIG 0x09
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_SYNC_WORD 0x39
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_IRQ_FLAGS 0x12
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_RX_NB_BYTES 0x13
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_FIFO 0x00
#define REG_SYMB_TIMEOUT_LSB 0x1F
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_PAYLOAD_LENGTH 0x22
#define REG_PKT_SNR_VALUE 0x19
#define REG_FR_MSB 0x06
#define REG_FR_MID 0x07
#define REG_FR_LSB 0x08

/** Bit Masks **/
#define DEVICE_MODE_BITMASK 0b11111000
#define BANDWIDTH_BITMASK 0b00001111
#define CODING_RATE_BITMASK 0b11110001
#define CRC_MODE_BITMASK 0b11111011
#define HEADER_MODE_BITMASK 0b11111110
#define POLARITY_BITMASK 0b10111111
#define SPREADING_FACTOR_BITMASK 0b00001111
#define LORA_MODE_BITMASK 0b01111111
#define FREQUENCY_MODE_BITMASK 0b11110111
#define FIFO_ADDR_PTR_BITMASK 0b00000000
#define LNA_BOOST_BITMASK 0b11111100
#define AGC_AUTO_BITMASK 0b11111011
#define OUTPUT_POWER_BITMASK 0xF0
#define PREAMBLE_LENGTH_BITMASK 0b00000000
#define SYMB_TIMEOUT_BITMASK 0b11111100
#define PA_SELECT_BITMASK 0b01111111

/** Types **/
enum bandwidth { //REG_MODEM_CONFIG1 Register
    BANDWIDTH7_8K = 0x00,
    BANDWIDTH10_4K = 0x10,
    BANDWIDTH15_6K = 0x20,
    BANDWIDTH20_8K = 0x30,
    BANDWIDTH31_25K = 0x40,
    BANDWIDTH41_7K = 0x50,
    BANDWIDTH62_5K = 0x60,
    BANDWIDTH125K = 0x70,
    BANDWIDTH250K = 0x80,
    BANDWIDTH500K = 0x90
};

enum coding_rate { //REG_MODEM_CONFIG1 Register
    CODINGRATE4_5 = 0b00000010,
    CODINGRATE4_6 = 0b00000100,
    CODINGRATE4_7 = 0b00000110,
    CODINGRATE4_8 = 0b00001000
};

enum crc_mode { //REG_MODEM_CONFIG2 Register
    CRC_ENABLE = 0b00000100,
    CRC_DISABLE = 0x00
};

enum header_mode{ //REG_MODEM_CONFIG1 Register
    EXPLICIT_HEADER_MODE = 0x00,
    IMPLICIT_HEADER_MODE = 0b00000001
};

enum polarity { //REG_INVERT_IQ Register
    POLARITY_NORMAL_MODE = 0x00,
    POLARITY_INVERTED_MODE = 0b01000000
};

enum spreading_factor { //REG_MODEM_CONFIG2 Register
    SPREADINGFACTOR64 = 0x60,
    SPREADINGFACTOR128 = 0x70,
    SPREADINGFACTOR256 = 0x80,
    SPREADINGFACTOR512 = 0x90,
    SPREADINGFACTOR1024 = 0xA0,
    SPREADINGFACTOR2048 = 0xB0,
    SPREADINGFACTOR4096 = 0xC0
};

enum sync_word {
    SYNC_WORD_RESET = 0x12
};

enum radio_status{
    RADIO_TX_IDLE = 0,
    RADIO_TX_TRANSMITTING = 1,
    RADIO_RX_IDLE = 2,
    RADIO_RX_RECEIVING = 3,
    RADIO_SLEEP = 4,
    RADIO_STBY = 5
};

// Transmission completion status
enum RadioTxStatus {
    TX_OK = 0,             // Transmission finished successfully
    TX_STILL_TRANSMITTING = 1, // Transmission is still in progress
    TX_NOT_STARTED = 2         // transmit_start() hasn’t been called yet
};

enum RadioRxStatus {
    ERR_STILL_RECEIVING = 0,
    ERR_VALID_HEADER_FAIL = 1, //ValidHeader Interrupt has been asserted
    ERR_CRC_FAIL = 2, //PayloadCrcError Interrupt has been asserted
    ERR_RX_DONE_FAIL = 3, //RxDone Interrupt has been asserted
    ERR_RX_TIMEOUT_FAIL = 4, //RxTimeout Interrupt has been asserted
    SUCCESSFUL_RX = 5,
    ERR_NO_RX = 6 // Radio not in standby mode nor is currently receiving anything
};

/** Function Prototypes **/
uint8_t read_register(uint8_t address, GpioPin& chip_sel);
void write_to_register(uint8_t address, uint8_t modified_register, GpioPin& chip_sel);
void modify_register(uint8_t address, uint8_t bit_mask, uint8_t data, GpioPin& chip_sel);
void lora_configure(uint8_t bandwidth, uint8_t coding_rate, uint8_t crc_mode, uint8_t header_mode, uint8_t polarity, uint16_t preamble_length, uint8_t spreading_factor, 
uint8_t sync_word, GpioPin& chip_sel);
void set_preamble_length(uint16_t preamble_length, GpioPin& chip_sel);
bool radio_transmit_start(uint8_t data_ptr[], uint16_t payload_length, GpioPin& chip_sel);
RadioTxStatus radio_transmit_is_complete(GpioPin& chip_sel);
bool radio_is_receiving(GpioPin& chip_sel);
void radio_receive_start(uint16_t timeout_symbols, GpioPin& chip_sel);
RadioRxStatus radio_receive_is_complete(uint8_t data[255], uint16_t* message_length_bytes, GpioPin& chip_sel);
int16_t get_packet_rssi(GpioPin& chip_sel);
int8_t get_packet_snr(GpioPin& chip_sel);
int16_t get_packet_strength(GpioPin& chip_sel);
radio_status radio_status_check(GpioPin& chip_sel);
void modify_register_with_check(uint8_t address, uint8_t bit_mask, uint8_t data, GpioPin& chip_sel);

#endif
