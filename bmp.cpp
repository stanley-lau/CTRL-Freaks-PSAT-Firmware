#include "bmp.hpp"
#include "pin_mappings.hpp"
#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"
#include "bmp_buffer.hpp"
#include <math.h>


volatile bool pressure_data_ready = false;
volatile bool data_ready_interrupt = false;


void InitBMP() {
    // Configure GPIO for SPI mode
    BMP_MOSI.function(PinFunction::Primary);
    BMP_MISO.function(PinFunction::Primary);
    BMP_CLK.function(PinFunction::Primary);
    BMP_CS.toOutput().setHigh(); // Chip select

    // Enable BMP interupts;
    BMP_INT.toOutput().risingEdgeTrigger().enableInterrupt();

    gpioUnlock();

    // Initialise BMP's SPI
    BMP_SPI.init(SpiMode::MODE_0(), ClockSource::Smclk, 1);
}

void ConfigureBMP() {  
    data_ready_interrupt = false;     // Flags are cleared every time function is called. 
    pressure_data_ready  = false;


    // "INT_CTRL" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x19 & 0x7F);         // AND-ing reg 0x19 with 0x7F forces bit 7 = 0, indicating a write operation.
        BMP_SPI.writeByte(0x76);   // 0111 0110 --> (right to left: push-pull, active_high INT pin, enable latching, disable FIFO watermark, enable FIFO interrupt, barometer interrupt pin high, enable temperature/pressure data ready) 
        BMP_SPI.flush();
    BMP_CS.setHigh();
    
    // "IF_CONF" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1A & 0x7F);   
        BMP_SPI.writeByte(0x00);   // bit0 = 0 --> SPI 4-wire mode
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // "PWR_CTRL" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1B & 0x7F);   // register address (write)  "0x1B" = "0001 1011". To the BMP390, this means PWR_CTRL Register
        BMP_SPI.writeByte(0x33);   // register value --> 0011 0011 --> (right to left: enable pressure sensor, enable temperature sensor, 0,0 normal mode (11), 0, 0)
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // "OSR" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1C & 0x7F);   
        BMP_SPI.writeByte(0x03);   // 0000 0011 --> 8x oversampling pressure measurement
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // "ODR" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1D & 0x7F);  
        BMP_SPI.writeByte(0x02);   // 0x02 --> 20ms sampling period
        BMP_SPI.flush();
    BMP_CS.setHigh();
}

void UpdateBMPStatus () {
    uint8_t int_status;  
    uint8_t status;

    // "INT_STATUS" register 
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x11 | 0x80);   // ORing with 0x08 forces bit 7 = 1 --> reading register mode. Register is cleared.
        int_status = BMP_SPI.readByte();   // Sends dummy byte to clock data out
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // Checking for data ready interrupt in the INT_STATUS register
    if (int_status & (1 << 3)) {           // 1<<3  == 0000 1000
        data_ready_interrupt = true;        // Currently unused
    }
    

    /* I can't find a reason to read this in the datasheet. Except for the fact that it tells us that pressure sensor data is ready. */

    // "STATUS" register 
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x03 | 0x80);   // Reading STATUS
        status = BMP_SPI.readByte();
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // Checking if pressure sensor data is ready 
    if (status & (1 << 5)) {
        pressure_data_ready = true;
    }
    if (status & (1 << 5)){
        
    }
}

/* bmpReadSample() ...
    - combines updateBMPStatus() and readRawPressure into one function()
    - should be called in main() when the variable "bmp_data_ready" is = true;
    - clears the INT_STATUS register by reading from it
    - Reads 6 consecutive registers (Press + temp)
    - Should store the data somewhere
*/
void GetPressure() {
    uint8_t int_status;  
    // Clear BMP's interrupt flag by reading from it.
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x11 | 0x80);   // ORing with 0x08 forces bit 7 = 1 --> reading register mode. Register is cleared.
        int_status = BMP_SPI.readByte();   // Sends dummy byte to clock data out
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // Read from pressure sensor + temperature sensor
    uint8_t buf[6];
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x04 | 0x80); // PRESSURE_XL
        for(int i = 0; i < 6; i++){
            buf[i] = BMP_SPI.readByte();
        }
        BMP_SPI.flush();
    BMP_CS.setHigh();

    // Convert and store in circular buffer
    bmpBuffer[write_index].pressure = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    bmpBuffer[write_index].temperature = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];

    /*
    Data is stored like this:
    bmpBuffer[0] = {pressure_sample0, temperature_sample0}
    bmpBuffer[1] = {pressure_sample1, temperature_sample1}
    bmpBuffer[2] = {pressure_sample2, temperature_sample2}
    */

    // Update circular buffer indices
    write_index = (write_index + 1) % BMP_BUFFER_SIZE;
    if (available_samples < BMP_BUFFER_SIZE) available_samples++;
}

// Review burst reading, and see if the BMP's register's auto increment when being read.
uint32_t readRawPressure() {
    uint8_t data0, data1, data2;

    BMP_CS.setLow();
    BMP_SPI.writeByte(0x04 | 0x80);      // start at DATA_0. Pressure data is stored from 0x04 to 0x06
        data0 = BMP_SPI.readByte();      // Reading 0x04, PRESS_XLSB
    BMP_SPI.writeByte(0x05 | 0x80);
        data1 = BMP_SPI.readByte();      // Reading 0x05, PRESS_LSB
    BMP_SPI.writeByte(0x06 | 0x80);
        data2 = BMP_SPI.readByte();      // Reading 0x06, PRESS_MSB
    BMP_SPI.flush();
    BMP_CS.setHigh();

    return ((uint32_t)data2 << 16) | ((uint32_t)data1 << 8) | data0; 
}

/*
    pressureToAltitude() converts pressure [Pa] into heigh above sea level [m], and returns the result.
*/
float PressureToAltitude(float pressure) {
    // return 44330.0f * (1.0f - powf(pressure / p0, 0.1903f)); // Using barometric formula // eqn where??

    float altitude = h_b + (T_b / L_b) * (powf(pressure / P_b, (-R * L_b) / (g_0 * M)) - 1);
    return altitude;
}

/*
    calibrateGroundPressure takes X samples, calculates the average pressure, and returns the result.
    Polling approach ass
*/
float CalibrateGroundPressure(uint16_t samples) {
    float sum = 0.0f;
    uint16_t collected = 0;

    // Rereads the status register and refreshes data every time 
    while (collected < samples) {
        UpdateBMPStatus();

        if (pressure_data_ready) {
            sum += (float)readRawPressure();
            collected++;
            pressure_data_ready = false;  //Reset pressure_data_ready back to false after reading data.
        }
    }

    return sum / samples;  // Return average pressure [Pa] of 100 samples 
}

/*
    SetupBMP, im guessing, should complete all the initialations, configs, and setups, required for the BMP to function during flight.
    This should be called once at the start of the main() function.
*/
void SetupBMP() {
    // Initialise MCU pins for BMP SPI
    InitBMP();
    
    // Configure BMP's internal registers.
    ConfigureBMP();

    // Polling approaching is okay during pre-flight phase. Poll until Pressure_data_ready is true.
    do {
        UpdateBMPStatus();
    } while (!pressure_data_ready);

    // Reset flag after loop exits.
    pressure_data_ready = false;

    // Calibrate ground pressure witwh 100 samples.
    //ground_pressure = CalibrateGroundPressure(100);
    //initial_altitude = pressureToAltitude(float ground_pressure); 
}


void DisableBMP() {
    // "PWR_CTRL" register
    BMP_CS.setLow();
        BMP_SPI.writeByte(0x1B & 0x7F);   // register address (write)  "0x1B" = "0001 1011". To the BMP390, this means PWR_CTRL Register
        BMP_SPI.writeByte(0x00);          // Sleep mode: disable the temperature and pressure sensor 
        BMP_SPI.flush();
    BMP_CS.setHigh();
}


// AltWindow_Push takes an altitude value, alt, and appends it to the smaller sized circular array "altWin"
void AltWindow_Push(float alt) {
    altWin.buf[altWin.idx] = alt;
    altWin.idx = (altWin.idx + 1) % ALT_WINDOW;

    if (altWin.count < ALT_WINDOW)
        altWin.count++;
}

// AltWindow_GetDelta takes a pointer to a variable "delta" and calculates the change in altitde which is stored in "delta"
bool AltWindow_GetDelta(float *delta) {
    if (altWin.count < ALT_WINDOW)
        return false;  // Not enough data yet

    uint8_t newest = (altWin.idx + ALT_WINDOW - 1) % ALT_WINDOW;
    uint8_t oldest = altWin.idx;  // Next overwrite = oldest

    *delta = altWin.buf[newest] - altWin.buf[oldest];
    return true;
}
