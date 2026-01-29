#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <driverlib.h>
#include "lora.hpp"
#include "spi.hpp"
#include "gpio.hpp"

uint8_t read_register(uint8_t address, GpioPin& chip_sel){
    *chip_sel.pout &= ~chip_sel.pin; //Set LOW
    spi_B1_send_byte((0<<7) + address); //read access
    uint8_t data = spi_B1_receive_byte();
    spi_B1_flush();
    *chip_sel.pout |= chip_sel.pin; //Set HIGH
    return data;
}

void write_to_register(uint8_t address, uint8_t modified_register, GpioPin& chip_sel){
    *chip_sel.pout &= ~chip_sel.pin; //Set LOW
    spi_B1_send_byte((1<<7) + address);
    spi_B1_send_byte(modified_register);
    spi_B1_flush();
    *chip_sel.pout |= chip_sel.pin; //Set HIGH
}

void modify_register(uint8_t address, uint8_t bit_mask, uint8_t data, GpioPin& chip_sel){
    uint8_t current_register_status = read_register(address, chip_sel);
    uint8_t modified_register = (current_register_status & bit_mask) | data;
    write_to_register(address, modified_register, chip_sel);
}

/** FUNCTIONS FOR USER **/
void lora_configure(uint8_t bandwidth, uint8_t coding_rate, uint8_t crc_mode, uint8_t header_mode, uint8_t polarity, uint16_t preamble_length, uint8_t spreading_factor, 
uint8_t sync_word, GpioPin& chip_sel){
    modify_register(REG_OP_MODE, DEVICE_MODE_BITMASK, DEVICE_SLEEP, chip_sel); //Sleep mode
    modify_register(REG_OP_MODE, LORA_MODE_BITMASK, LORA_MODE, chip_sel); //LoRa mode
    modify_register(REG_OP_MODE, FREQUENCY_MODE_BITMASK, HIGH_FREQ_MODE, chip_sel); //Set to High Frequency mode
    modify_register(REG_MODEM_CONFIG1, BANDWIDTH_BITMASK, bandwidth, chip_sel); //Configure bandwidth
    modify_register(REG_MODEM_CONFIG1, CODING_RATE_BITMASK, coding_rate, chip_sel); //Configure coding rate
    modify_register(REG_MODEM_CONFIG2, CRC_MODE_BITMASK, crc_mode, chip_sel); //Configure CRC mode
    modify_register(REG_MODEM_CONFIG1, HEADER_MODE_BITMASK, header_mode, chip_sel); //Configure header mode
    modify_register(REG_INVERT_IQ, POLARITY_BITMASK, polarity, chip_sel); //Configure polarity
    write_to_register(REG_FIFO_ADDR_PTR, FIFO_ADDR_PTR, chip_sel); //Set FIFO Pointer
    set_preamble_length(preamble_length, chip_sel);
    modify_register(REG_MODEM_CONFIG2, SPREADING_FACTOR_BITMASK, spreading_factor, chip_sel); //Configure spreading factor
    write_to_register(REG_SYNC_WORD, sync_word, chip_sel); //Configure sync word
    modify_register(REG_LNA, LNA_BOOST_BITMASK, LNA_BOOST_HF, chip_sel); //set LNA boost
    modify_register(REG_MODEM_CONFIG3, AGC_AUTO_BITMASK, AGC_AUTO_ON, chip_sel); //set Auto AGC
    modify_register(REG_PA_CONFIG, PA_SELECT_BITMASK, PA_BOOST, chip_sel); //set PA Boost
    modify_register(REG_PA_CONFIG, OUTPUT_POWER_BITMASK, OUTPUT_POWER, chip_sel); //set output power to 17 dBm
    write_to_register(REG_FR_MSB, FRF_MSB, chip_sel); //Setting frequency to 915MHz
    write_to_register(REG_FR_MID, FRF_MID, chip_sel);
    write_to_register(REG_FR_LSB, FRF_LSB, chip_sel);
    modify_register(REG_OP_MODE, DEVICE_MODE_BITMASK, DEVICE_STDBY, chip_sel); //Standby mode
}

void set_preamble_length(uint16_t preamble_length, GpioPin& chip_sel){
    if(preamble_length < 6){
        preamble_length = 6;
    }
    uint8_t preamble_length_lsb = preamble_length & GET_LSB;
    uint8_t preamble_length_msb = preamble_length >> 8;
    write_to_register(REG_PREAMBLE_MSB, preamble_length_msb, chip_sel); //Configure preamble MSB
    write_to_register(REG_PREAMBLE_LSB, preamble_length_lsb, chip_sel); //Configure preamble LSB
}

/** TRANSMITTING **/
bool radio_transmit_start(uint8_t data_ptr[], uint16_t payload_length, GpioPin& chip_sel){ //check if ready to send, if so load data and start transmission
    if((payload_length <= 0) || (payload_length > 256)){
        return false;
    }
    modify_register(REG_OP_MODE, DEVICE_MODE_BITMASK, DEVICE_STDBY, chip_sel); //Mode request STAND-BY
    uint8_t fifo_tx_ptr_base = read_register(REG_FIFO_TX_BASE_ADDR, chip_sel);
    write_to_register(REG_FIFO_ADDR_PTR, fifo_tx_ptr_base, chip_sel); //Set FifoPtrAddr to FifoTxPtrBase
    write_to_register(REG_PAYLOAD_LENGTH, payload_length, chip_sel); //Write payload length to register
    for(uint16_t i = 0; i < payload_length; i++){ //write an SPI burst function that takes an array and a length and sends the whole array in one transaction
        write_to_register(REG_FIFO, data_ptr[i], chip_sel); //Write Data FIFO
    }
    modify_register(REG_IRQ_FLAGS, TX_DONE_CHECK_MASK, TX_DONE_CHECK_MASK, chip_sel); //Clear Tx done flag
    modify_register(REG_OP_MODE, DEVICE_MODE_BITMASK, DEVICE_TX, chip_sel); //Mode Request TX
    return true;
}

RadioTxStatus radio_transmit_is_complete(GpioPin& chip_sel){ //Check if transmission complete
    uint8_t tx_done = read_register(REG_IRQ_FLAGS, chip_sel) & TX_DONE_CHECK_MASK;
    uint8_t mode = read_register(REG_OP_MODE, chip_sel) & DEVICE_MODE_CHECK_MASK;
    if(mode == DEVICE_STDBY){
    //if((tx_done) && (mode == DEVICE_STDBY)){
        return TX_OK;
    }
    if(mode == DEVICE_TX){
    //if ((!tx_done) && (mode == DEVICE_TX)){
        return TX_STILL_TRANSMITTING;
    }
    return TX_NOT_STARTED;
}
/************/


/** RECEIVING **/
bool radio_is_receiving(GpioPin& chip_sel){
    uint8_t mode = read_register(REG_OP_MODE, chip_sel) & DEVICE_MODE_CHECK_MASK;
    if(mode == DEVICE_RXCONTINUOUS){
        return true;
    }
    if(mode == DEVICE_RXSINGLE){
        //Radio is currently receiving until any of RxDone, RxTimeout, or PayloadCrcError are set
        uint8_t rx_complete_flags = read_register(REG_IRQ_FLAGS, chip_sel) & REG_IRQ_FLAGS_CHECK_MASK;
        if(rx_complete_flags == 0){
            return true;
        }
        return false;
    }      
    return false;
}
        

void radio_receive_start(uint16_t timeout_symbols, GpioPin& chip_sel) //Start receiving - RX SINGLE MODE
//timeout_symbols is the timeout specified as a number of symbols, final time depends on bandwidth and spreading factor
{ 
    modify_register(REG_OP_MODE, DEVICE_MODE_BITMASK, DEVICE_STDBY, chip_sel); //Mode request stand-by
    uint8_t fifo_rx_base_addr = read_register(REG_FIFO_RX_BASE_ADDR, chip_sel);
    modify_register(REG_FIFO_ADDR_PTR, FIFO_ADDR_PTR_BITMASK, fifo_rx_base_addr, chip_sel); //Set FifoAddrPtr to FifoRxBaseAddr

    timeout_symbols |= SYMBOL_BITS_BITMASK;
    uint8_t timeout_symbols_lsb = timeout_symbols & GET_LSB;
    uint8_t timeout_symbols_msb = timeout_symbols >> 8;
    modify_register(REG_MODEM_CONFIG2, SYMB_TIMEOUT_BITMASK, timeout_symbols_msb, chip_sel); //Set Timeout MSB (bits 8 - 9)
    write_to_register(REG_SYMB_TIMEOUT_LSB, timeout_symbols_lsb, chip_sel); //Set Timeout LSB (bits 0 - 7)

    modify_register(REG_IRQ_FLAGS, CLEAR_RX_FLAGS, CLEAR_RX_FLAGS, chip_sel); //Clear RxTimeout, RxDone, PayloadCrcError, ValidHeader flags

    uint8_t crc_flag = read_register(REG_IRQ_FLAGS, chip_sel);

    modify_register(REG_OP_MODE, DEVICE_MODE_BITMASK, DEVICE_RXSINGLE, chip_sel); //Selecting operating mode RXSINGLE
}

// Check if receiving complete (and if so, retrieve data)
RadioRxStatus radio_receive_is_complete(uint8_t data[255], uint16_t* message_length_bytes, GpioPin& chip_sel){
    if(radio_is_receiving(chip_sel)){
        return ERR_STILL_RECEIVING;
    } 
    uint8_t mode = read_register(REG_OP_MODE, chip_sel) & DEVICE_MODE_CHECK_MASK;
    if(mode != DEVICE_STDBY){
        return ERR_NO_RX;
    }
    uint8_t irq_flags = read_register(REG_IRQ_FLAGS, chip_sel) & FLAGS_CHECK_VALID_RX_MASK;
    uint8_t invalid_packet_payload = irq_flags & PAYLOAD_CRC_ERROR_CHECK_MASK;
    if(invalid_packet_payload){
        uint8_t rssi = get_packet_rssi(chip_sel);
        uint8_t snr = get_packet_snr(chip_sel);
        uint8_t strength = get_packet_strength(chip_sel);
        return ERR_CRC_FAIL;
    }
    uint8_t rx_timeout = irq_flags & RX_TIMEOUT_CHECK_MASK;
    if(rx_timeout){
        return ERR_RX_TIMEOUT_FAIL;
    }
    uint8_t rx_not_done = !(irq_flags & RX_DONE_CHECK_MASK);
    if(rx_not_done){
        return ERR_RX_DONE_FAIL;
    }
    uint8_t invalid_header = !(irq_flags & VALID_HEADER_CHECK_MASK);
    if(invalid_header){
        return ERR_VALID_HEADER_FAIL;
    }
    uint8_t number_of_bytes_received = read_register(REG_RX_NB_BYTES, chip_sel);
    uint8_t last_packet_location = read_register(REG_FIFO_RX_CURRENT_ADDR, chip_sel);
    modify_register(REG_FIFO_ADDR_PTR, FIFO_ADDR_PTR_BITMASK, last_packet_location, chip_sel); //Set RegFifoAddrPtr to RegFifoRxCurrentAddr
    for(uint16_t i = 0; i < number_of_bytes_received; i++){ //Reading the register RegFifo, RegRxNbBytes times
        uint8_t payload_byte = read_register(REG_FIFO, chip_sel);
        data[i] = payload_byte;
    }
    *message_length_bytes = number_of_bytes_received;
    return SUCCESSFUL_RX;
}
/************/

/// Get the Relative Signal Strength Indicator (RSSI) of the last received packet. Assumes high frequency band.
int16_t get_packet_rssi(GpioPin& chip_sel) {
    int16_t rssi_raw = read_register(REG_PKT_RSSI_VALUE, chip_sel);
    int16_t hf_rssi_offset = -157;

    return rssi_raw + hf_rssi_offset;
}

/// Get the Signal to Noise Ratio (SNR) of the last received packet
int8_t get_packet_snr(GpioPin& chip_sel) {
    return (((int8_t)read_register(REG_PKT_RSSI_VALUE, chip_sel)) / 4);
}

/// Get the signal strength of the last received packet
///
/// # Note
/// Unlike RSSI, this accounts for LoRa's ability to receive packets below the noise floor.
int16_t get_packet_strength(GpioPin& chip_sel){
    int16_t snr = get_packet_snr(chip_sel);
    if(snr < 0){
        int16_t hf_rssi_offset = -157;
        uint8_t packet_rssi = read_register(REG_PKT_RSSI_VALUE, chip_sel);
        uint8_t packet_snr = read_register(REG_PKT_SNR_VALUE, chip_sel);
        return hf_rssi_offset + packet_rssi + packet_snr / 4;
    }
    int16_t rssi = get_packet_rssi(chip_sel);
    return rssi;
}

radio_status radio_status_check(GpioPin& chip_sel){ //Whether the radio is transmitting, receiving, idle, etc.
    uint8_t mode = read_register(REG_OP_MODE, chip_sel) & DEVICE_MODE_CHECK_MASK;
    if (mode == DEVICE_TX){
        if(radio_transmit_is_complete(chip_sel) == TX_STILL_TRANSMITTING){
            return RADIO_TX_TRANSMITTING;
        }
        return RADIO_TX_IDLE;
    }
    if((mode == DEVICE_RXCONTINUOUS) || (mode == DEVICE_RXSINGLE)){
        if(radio_is_receiving(chip_sel)){
            return RADIO_RX_RECEIVING;
        }
        return RADIO_RX_IDLE;
    }
    if(mode == DEVICE_SLEEP) {
        return RADIO_SLEEP;
    }
    return RADIO_STBY;
}
