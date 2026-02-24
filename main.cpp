#include <driverlib.h>
#include <msp430.h>
#include <stdint.h>
#include "util.h"
#include <math.h> 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"

#include "pin_mappings.hpp"
#include "bmp.hpp"
#include "bmp_buffer.hpp"
#include "accl.hpp"

#include "LoRa/spi.hpp"
#include "LoRa/lora.hpp"
#include "LoRa/gpio.hpp"

// ==================== Constants =========================//
// PWM
#define PWM_PERIOD 500                  // PWM Period

// LED
#define RED_LED BIT0                    // Onboard LED
#define GREEN_LED BIT6                  // Onboard LED

// ADC
#define ADCMAX 4095.00f //Correlating to 12 bits
#define VREF  3.3f
#define RESISTOR 9815.0f
#define BETA 3380.0f 
#define RTHERM 10000.0f
#define T0_K 298.15f //25 degrees Celsius in Kelvin 

// ADC pins
#define ADC_CHAM_THERM   ADCINCH_8    // P5.0 / A8
#define ADC_BAT_THERM    ADCINCH_4   // P1.4 / A4
#define ADC_CUR_SENSE    ADCINCH_11   // P5.3 / A11

// UpdateFlightState
#define ACCL_THRESHOLD        3       // m^2/s. 
#define ALTITUDE_THRESHOLD    5       // meters

// Threshold values based on testing and material properties. 
#define CURRENT_MAX 7.26   // Max current in Amps
#define BATTERY_MAX 60     // Max temperature in Celsius
#define CHAMBER_MAX 200

#define R_SENSE 0.1       // Shunt resistor in Ohms

// GPS 
#define GPS_BUFFER_SIZE 128

// ==================== Interrupt Service Routines =========================//
// ADC Interrupt: Interrupt that read ADC memory into a global variable
volatile uint16_t adc_result;
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void) {
    switch (__even_in_range(ADCIV, ADCIV_ADCIFG))
    {
        case ADCIV__ADCIFG0:
            adc_result = ADCMEM0;   // After memory is read, ADCIFG0 is cleared. 
            //P1OUT ^= BIT0;
            __bic_SR_register_on_exit(CPUOFF);
            break;

        default:
            break;
    }
}

// With the intention to tidy up the architecture main() should follow: InitGPIOs --> Unlock GPIO (ONCE) --> Configure Peripheral --> while(1) flightloop



// ==================== Coil PWM =========================//
// Initialise and config coil PWM for port 5.1 / timer 2.2
void InitCoilGPIO() {
    // Setup Pins
    P5DIR  |= BIT1;                             // Set BIT1 of P5DIR to output. BIT1 Corresponds to PIN1 on the Port.
    
    P5SEL0 |= BIT1;     // Set BIT 1 of P5SEL0 = 1
    P5SEL1 &= ~BIT1;    // Set BIT 1 of P5SEL1 = 0
}

void ConfigCoilPWM() {
    // Setup Compare Reg
    TB2CCR0 = PWM_PERIOD;                       // Roll-over from LOW to HIGH - Customise 
    TB2CCR2 = 0;                                // 0% duty cycle
    TB2CCTL2 = OUTMOD_7;                        // Reset/Set PWM mode, refer to YT vid
    // TB2CCRn is the timer attached to Port P5 from Table 6-67

    // Setup Timer_B2
    TB2CTL =    TBCLR       |       TBSSEL_2    | MC_1;
}

// SetCoilPWM sets the duty_cyle for the coil.
void SetCoilPWM(uint8_t duty_cycle) {
    // PWM Coil P5.1 on schematic. 
    // duty_cyle must be a positive integer between 0 and 100;
    // SMCLK is used at 1Mhz

    if (duty_cycle > 100) {
        duty_cycle = 100;
    }
    
    // Update Duty cycle.
    if (duty_cycle >= 100) {
        TB2CCR2 = TB2CCR0 + 1; // force always-high
    } else {
        TB2CCR2 = (TB2CCR0 * duty_cycle) / 100;     // // Set LOW when TB0CCR1 is reached
    }
}

// ==================== Voltage Regulator Enable  =========================//
// When PWM is turned on, voltage regulator also needs to be turned on  
void InitRegulatorGPIO() {
    // Force P3.5 to GPIO by setting P3SEL0/1 to 0 
    P3SEL0 &= ~BIT5;  
    P3SEL1 &= ~BIT5;

    // REG_EN as output
    P3DIR |= BIT5;
}

void EnableRegulator() {
    // Enable voltage regulator
    P3OUT |= BIT5;
}

// ==================== Fan PWM =========================//
// Initialise and config fan PWM for port 6.1 / timer 3.2
void InitFanGPIO(){
    // Setup Pins
    P6DIR  |= BIT1;                             // Set BIT1 of P6DIR to output. 

    P6SEL0 |= BIT1;     // Set BIT 1 of P6SEL0 = 1
    P6SEL1 &= ~BIT1;    // Set BIT 1 of P6SEL1 = 0
    //Page 106 --> 01b = select timer Timer_B3.2
}

void ConfigFanPWM(){
    // Setup Compare Reg
    TB3CCR0 = PWM_PERIOD;                       // Roll-over from LOW to HIGH - Customise 
    TB3CCR2 = 0;                                // 0% duty cycle
    TB3CCTL2 = OUTMOD_7;                        // Reset/Set PWM mode, refer to YT vid
    // TB3CCRn is the timer attached to Port P6 from Table 6-68

    // Setup Timer 3.2
    TB3CTL =    TBCLR       |       TBSSEL_2    | MC_1;
    //     = Clear Timer_B0 |  Set SMCLK as clk | Up-Mode: Count upwards
}

// SetFanPWM sets the duty_cyle for the coil.
void SetFanPWM(uint8_t duty_cycle) {
    // PWM Fan P6.1 on schematic. 
    // duty_cyle must be a positive integer between 0 and 100;
    // SMCLK is used at 1Mhz
    
    if (duty_cycle > 100) {
        duty_cycle = 100;
    }

    // Update Duty cycle.
    if (duty_cycle >= 100) {
        TB3CCR2 = TB3CCR0 + 1; // force always-high
    } else {
        TB3CCR2 = (TB3CCR0 * duty_cycle) / 100;     // // Set LOW when TB0CCR1 is reached
    }

}

// ==================== FlightStates =========================//

// Initialise flightstates
enum FlightState {PREFLIGHT, FLIGHT, LANDED};
enum FlightState current_flight_state = PREFLIGHT;
enum FlightState prev_flight_state;

// Utilise BOTH sensor's reading of altitude and acceleration to determine current flight state. 
// Compare value against initial calculated altitude and pressure to determine change. 
void UpdateFlightState() {
    float delta;
    
    if (!AltWindow_GetDelta(&delta)){
        return;
    }

    float accl_magnitude = ReadACCL();

    switch (current_flight_state) {
        
        case PREFLIGHT:
        // Detect launch: altitude change (current altitude jumps from ground altitude) + movement (acceleration value > 0)
            if (delta > ALTITUDE_THRESHOLD && accl_magnitude > ACCL_THRESHOLD) {
                prev_flight_state = PREFLIGHT;
                current_flight_state = FLIGHT;
            }
            break;
            
        case FLIGHT:
        // Detect landing: altitude change negligible/altitude approx ground altitude AND negligible acceleration
            if (delta < ALTITUDE_THRESHOLD && accl_magnitude < ACCL_THRESHOLD) {
                prev_flight_state = FLIGHT;
                current_flight_state = LANDED;
            }
            break;

        case LANDED:
            break;
    }
}

/* =========================== SENSORS =============================== */

#include "hal/blocking_i2c.hpp"

#define BUFFER_SIZE 16

// I2C Temporary Pin Mappings for V2.1 reference board for BMP and ACCL
Pin<P1,3> scl;
Pin<P1,2> sda;
I2cMaster<I2C_B0> i2c;

// Slave addresses
static const uint8_t BMP_ADDR  = 0x76; // Address is 0x76 since BMP_ADR is set to low 
static const uint8_t ACCL_ADDR = 0x68; // Address is 0x68 since ACCL_ADR is set to low

// Write a single byte to register
void I2CWriteRegister(uint8_t devAddr, uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    i2c.write(devAddr, data, 2);
}

// Read a single byte from register
uint8_t I2CReadRegister(uint8_t devAddr, uint8_t reg) {
    uint8_t val;
    volatile int16_t result;

    // Write the register address, then read 1 byte
    result = i2c.write_read(devAddr, &reg, 1, &val, 1);
    if (result != -1) {
        // NAACK were received, indicating error occurred during I2C transaction
        return 0; 
    }

    return val;
}

void InitSensorsGPIO() {
    // BMP_ADR on P6.2
    P6DIR |= BIT2;     // Set as output
    P6OUT &= ~BIT2;    // 0 = low (address 0x76)
    
    // IMU_ADR on P6.4
    P6DIR |= BIT4;     // Set as output
    P6OUT &= ~BIT4;    // 0 = low (address 0x68)

    sda.function(PinFunction::Primary);
    scl.function(PinFunction::Primary);
}

void ConfigSensorsI2C() {
    // Initialise I2C (100kHz)
    i2c.init(I2cClockSource::Smclk, 10);
}

// =========== Barometer ============
// ----- Calibration Functions ------
struct BMP390Calib {
    double par_t1, par_t2, par_t3;
    double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    double t_lin; // intermediate temperature used for pressure compensation
};

BMP390Calib bmp_calib;

void ConvertBMP390Calibration(const uint8_t *c) {
    bmp_calib.par_t1 = (double)((uint16_t)c[1] << 8 | c[0]) / 0.00390625; 
    bmp_calib.par_t2 = (double)((int16_t)c[3] << 8 | c[2]) / 1073741824.0; // 2^30
    bmp_calib.par_t3 = (double)((int8_t)c[4]) / 281474976710656.0;        // 2^48

    bmp_calib.par_p1 = ((double)((int16_t)c[6] << 8 | c[5]) - 16384.0) / 1048576.0;
    bmp_calib.par_p2 = ((double)((int16_t)c[8] << 8 | c[7]) - 16384.0) / 536870912.0;
    bmp_calib.par_p3 = (double)((int8_t)c[9])  / 4294967296.0;           // 2^32
    bmp_calib.par_p4 = (double)((int8_t)c[10]) / 137438953472.0;         // 2^37
    bmp_calib.par_p5 = (double)((uint16_t)c[12] << 8 | c[11]) / 0.125 ; 
    bmp_calib.par_p6 = (double)((uint16_t)c[14] << 8 | c[13]) / 64.0;
    bmp_calib.par_p7 = (double)((int8_t)c[15]) / 256.0;
    bmp_calib.par_p8 = (double)((int8_t)c[16]) / 32768.0;
    bmp_calib.par_p9 = (double)((int16_t)c[18] << 8 | c[17]) / 281474976710656.0; // 2^48
    bmp_calib.par_p10 = (double)((int8_t)c[19]) / 281474976710656.0;              // 2^48
    bmp_calib.par_p11 = (double)((int8_t)c[20]) / 36893488147419103232.0;         // 2^65
}

// Read and set calibration 
void ReadBMP390Calibration(void) {
    uint8_t calib[21];
    uint8_t calibStart = 0x31;
    int16_t result = i2c.write_read(BMP_ADDR, &calibStart, 1, calib, 21);
    if (result != -1) return; // error
    ConvertBMP390Calibration(calib);
}

// Compensation
double CompensateTemperature(uint32_t raw_temp) {
    double partial1 = (double)raw_temp - bmp_calib.par_t1;
    double partial2 = partial1 * bmp_calib.par_t2;
    bmp_calib.t_lin = partial2 + partial1 * partial1 * bmp_calib.par_t3;
    return bmp_calib.t_lin;
}

double CompensatePressure(uint32_t raw_pressure) {
    double t_linear = bmp_calib.t_lin;

    // Step 1: Offset compensation
    double partial1 = bmp_calib.par_p6 * t_linear;
    double partial2 = bmp_calib.par_p7 * t_linear * t_linear;
    double partial3 = bmp_calib.par_p8 * t_linear * t_linear * t_linear;
    double offset = bmp_calib.par_p5 + partial1 + partial2 + partial3;

    // Step 2: Sensitivity compensation
    partial1 = bmp_calib.par_p2 * t_linear;
    partial2 = bmp_calib.par_p3 * t_linear * t_linear;
    partial3 = bmp_calib.par_p4 * t_linear * t_linear * t_linear;
    double sensitivity = bmp_calib.par_p1 + partial1 + partial2 + partial3;

    double compensated_pressure = sensitivity * (double)raw_pressure + offset;

    // Step 3: Second-order correction
    double raw2 = (double)raw_pressure * (double)raw_pressure;
    compensated_pressure += raw2 * (bmp_calib.par_p9 + bmp_calib.par_p10 * t_linear);
    compensated_pressure += raw2 * (double)raw_pressure * bmp_calib.par_p11;

    return compensated_pressure;
}

// ----- Calibration Functions Finish ------

volatile bool pressure_data_ready_i2c = false;
void ConfigBMPI2C() {
    ReadBMP390Calibration();

    // "INT_CTRL" register 
    I2CWriteRegister(BMP_ADDR, 0x19, 0x76); // push-pull, active_high, latching
    
    // "IF_CONF" register
    I2CWriteRegister(BMP_ADDR, 0x1A, 0x00); // Watchdog disabled for I2C  

    // "PWR_CTRL" register 
    I2CWriteRegister(BMP_ADDR, 0x1B, 0x33); // Pressure on, Temperature on, Normal Mode

    // "OSR" register (0x1C)
    I2CWriteRegister(BMP_ADDR, 0x1C, 0x03); // 8x oversampling pressure

    // "ODR" register (0x1D)
    I2CWriteRegister(BMP_ADDR, 0x1D, 0x02); // 20ms sampling period

    // "CONFIG" register (0x1F) 
    // May consider adding IIR Filter 
    I2CWriteRegister(BMP_ADDR, 0x1F, (1 << 3)); // Bit 1 = 1 --> IIR filter coefficient is set to 1. 

    //__delay_cycles(10000);
}

// Read register to determine if new pressure data is ready 
void UpdateBMPStatusI2C () {
    uint8_t status;

    // "STATUS" register 
    status = I2CReadRegister(BMP_ADDR, 0x03); 
    // Checking if pressure sensor data is ready 
    if (status & (1 << 5)) {
        pressure_data_ready_i2c = true;
    }
}

uint32_t ReadRawPressureI2C() {
    uint8_t buf[3];
    uint8_t startReg = 0x04; // pressure register start
    int16_t result;

    // Write register address, then read 3 bytes
    result = i2c.write_read(BMP_ADDR, &startReg, 1, buf, 3);
    // -1 is returned if successful. Otherwise, error has occured. 
    if (result != -1) {
        return 0; // Indicate error has occurred
    }

    return ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0]; 
}

uint32_t ReadRawTemperatureI2C() {
    uint8_t buf[3];
    uint8_t startReg = 0x07; // temperature register start
    int16_t result;

    // Write register address, then read 3 bytes
    result = i2c.write_read(BMP_ADDR, &startReg, 1, buf, 3);
    // Returns -1 if no NACKs were received (successful), otherwise (failure) returns the byte number from which the NACK was received.
    if (result != -1) {
        return 0; 
    }

    return ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0]; 
}

// Get raw pressure, then performs calibration and compensation on it to return current pressure (Pascals)
float GetPressureI2C() {
    volatile uint32_t raw_pressure = ReadRawPressureI2C();
    volatile uint32_t raw_temperature = ReadRawTemperatureI2C();

    CompensateTemperature(raw_temperature);
    volatile double pressure_pa = CompensatePressure(raw_pressure);
    return (float)pressure_pa; 
}

// Turns off the temperature and pressure sensor. Enter sleep mode. 
void DisableBMPI2C() {
    // "PWR_CTRL" register
    I2CWriteRegister(BMP_ADDR, 0x1B, 0x70);  // Sleep mode: disable the temperature and pressure sensor 
}

// CalibrateGroundPressureI2C takes X samples, calculates the average pressure, and returns the result
float CalibrateGroundPressureI2C(uint16_t samples) {
    float sum = 0.0f;
    uint16_t collected = 0;

    // Rereads the status register and refreshes data every time 
    while (collected < samples) {
        UpdateBMPStatusI2C();

        if (pressure_data_ready_i2c) {
            sum += (float)GetPressureI2C();
            collected++;
            pressure_data_ready_i2c = false;  //Reset pressure_data_ready back to false after reading data.
        }
    }

    return (sum / samples);  // Return average pressure [Pa] of 100 samples 
}

float PressureToAltitudeI2C(float pressure) {
    float altitude = h_b + (T_b / L_b) * (powf(pressure / P_b, (-R * L_b) / (g_0 * M)) - 1);
    return altitude;
}

float ground_pressure_i2c; 
float initial_altitude_i2c; 
// Initial set up for BMP, and then calculate ground pressure and initial altitude. 
void CalibrateBMPI2C (float *ground_pressure_i2c, float *initial_altitude_i2c) {
    // Polling approaching is okay during pre-flight phase. Poll until Pressure_data_ready is true.
    do {
        UpdateBMPStatusI2C();
    } while (!pressure_data_ready_i2c);

    // Reset flag after loop exits.
    pressure_data_ready_i2c = false;

    // Calibrate current pressure at ground using 100 samples 
    *ground_pressure_i2c = CalibrateGroundPressureI2C(100); 
    *initial_altitude_i2c = PressureToAltitudeI2C(*ground_pressure_i2c);
}

/* ============== Getting pressure data ready for UpdateFlightState Logic*/

float PressureBuffer[BUFFER_SIZE];
float AltitudeBuffer[BUFFER_SIZE];

// Indexing and tracking
static volatile uint8_t PressureBufferIndex = 0;
static volatile uint8_t AltitudeBufferIndex = 0;
static uint8_t TotalSamplesCollected = 0;

// Reading pressure data into a circular buffer of size 16.
void PushToPressureBuffer(float pressure_value) {
    PressureBuffer[PressureBufferIndex] = pressure_value;
    PressureBufferIndex = (PressureBufferIndex + 1) % BUFFER_SIZE;
}

void PushToAltitudeBufferI2C(float altitude_value) {
    AltitudeBuffer[AltitudeBufferIndex] = altitude_value;
    AltitudeBufferIndex = (AltitudeBufferIndex + 1) % BUFFER_SIZE;
    
    // Track total samples for threshold checks
    if (TotalSamplesCollected < BUFFER_SIZE) {
        TotalSamplesCollected++;
    }
}

// Perform processing: Reading pressure data, storing into pressure buffer, converting to altitude, storing in altitude buffer
void ProcessBMPDataI2C() {
    // Check if the BMP390 has new data ready
    UpdateBMPStatusI2C(); 

    if (pressure_data_ready_i2c) {
        float pressure = GetPressureI2C();
        
        // 3. Store in the pressure circular array
        PushToPressureBuffer(pressure);
        
        float current_alt = PressureToAltitudeI2C(pressure);
        
        // Store in the altitude circular array
        PushToAltitudeBufferI2C(current_alt);

        // Reset the flag for the next cycle
        pressure_data_ready_i2c = false;
    }
}

// Calculate the change in altitude by taking the newest altitude data minus oldest data in circular buffer
bool GetAltitudeDeltaI2C(float *delta_i2c) {
    if (TotalSamplesCollected < BUFFER_SIZE) return false;

    uint8_t newest = (AltitudeBufferIndex + BUFFER_SIZE - 1) % BUFFER_SIZE;
    uint8_t oldest = AltitudeBufferIndex;

    *delta_i2c = AltitudeBuffer[newest] - AltitudeBuffer[oldest];
    return true;
}

// Calculating overall altitude change from ground altitude
float OverallAltitudeChangeI2C = 0.0f;
void OverallAltitudeDeltaI2C(float *OverallAltitudeChange) {
    uint8_t newest = (AltitudeBufferIndex + BUFFER_SIZE - 1) % BUFFER_SIZE;
    uint8_t oldest = AltitudeBufferIndex;

    *OverallAltitudeChange = AltitudeBuffer[newest] - initial_altitude_i2c;
}

// ============================== ACCL ============================== 

void ConfigACCLI2C() {
    //PWR_MGMT0 register
    I2CWriteRegister(ACCL_ADDR, 0x1F, (1 << 0) | (1 << 1)); // Make 0, 1 = 1 --> Places accelerometer in Low Noise (LN) Mode
    // __delay_cycles(500);                  // Small safety delay

    //INT_CONFIG register
    I2CWriteRegister(ACCL_ADDR, 0x06, 0x3F);  // = 0011 1111 = INT1: active high, push-pull, latched, INT2: active high, push-pull, latched
    //__delay_cycles(500);               

    //INT_SOURCE0 register
    I2CWriteRegister(ACCL_ADDR, 0x2B, (1 << 3));       // Make bit 3 = 1 --> Data ready interrupt routed to INT1
    //__delay_cycles(500);          

    //WOM_CONFIG register
    I2CWriteRegister(ACCL_ADDR, 0x27, (1 << 0));       // Make bit 0 = 1 --> Wake On Motion (WOM) enabled 
    //__delay_cycles(500);   

    //INT_SOURCE4 register
    I2CWriteRegister(ACCL_ADDR, 0x2E, (1 << 0) | (1 << 1) | (1 << 2));       // Make bit 0, 1, 2 = 1 --> X, Y, Z axis WOM routed to INT2
    //__delay_cycles(500);

    //INT_CONFIG0 register
    I2CWriteRegister(ACCL_ADDR, 0x04, (1 << 5) | (0 << 4));       // Make bit 5 = 1, bit 4 = 0 --> Data ready interrupt cleared on sensor register read
    //__delay_cycles(500);

    //ACCEL_CONFIG0 register
    I2CWriteRegister(ACCL_ADDR, 0x21, (0 << 5) | (0 << 6));       // Make bit 5 and 6 = 0 --> Accelerometer output = ±16g full-scale
    //__delay_cycles(500);
}

// Checking for new acceleration data
bool accl_data_ready_i2c = false;
void UpdateACCLStatusI2C() {
    uint8_t int_status;

    // "INT_STATUS_DRDYS" register 
    int_status = I2CReadRegister(ACCL_ADDR, 0x39); 

    // Checking for data ready interrupt in the INT_STATUS register
    if (int_status & (1 << 0)) {                  // Bit 0 is automatically sets to 1 when a Data Ready interrupt is generated
        accl_data_ready_i2c = true;               // Data is ready. After register has been read, bit 0 is set to 0. 
    }
}

// Returns value of linear acceleration. Unit of ms^-2. 
float ReadACCLI2C() {
    uint8_t buf[6];
    // Raw acceleration data read in the x, y, and z axis. 
    int16_t ax_raw, ay_raw, az_raw;
    float ax, ay, az;

    // Start at the first register (0x0B) and read 6 bytes in one go. 
    // Internal register is locked and ensure data is consistent. 
    int16_t result = i2c.write_read(ACCL_ADDR, (uint8_t[]){0x0B}, 1, buf, 6);
    
    if (result != -1) return 0.0f; // Handle I2C error

    // Data is stored as buf[0]=x-high, buf[1]=x-low, buf[2]=yh, buf[3]=yl, buf[4]=zh, buf[5]=zl
    ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    // Convert to g using ±16g full-scale
    ax = ax_raw / 2048.0f;
    ay = ay_raw / 2048.0f;
    az = az_raw / 2048.0f;
    
    // Magnitude of acceleration vector (Result is in G's)
    volatile float magnitude_g = sqrtf(ax*ax + ay*ay + az*az);

    // Convert to m/s^2 
    volatile float magnitude_ms2 = magnitude_g * 9.80665f;

    // Subtract Bias (Gravity) and apply Deadband
    // This removes the 9.8 m/s^2 constant and any sensor offset
    volatile float linear_accel = fabsf(magnitude_ms2 - 9.80665f);

    // Deadband: If result is small (noise), acceleration is 0.0
    if (linear_accel < 0.4f) {
        return 0.0f;
    }

    return linear_accel;
}

void DisableACCLI2C() {
    //PWR_MGMT0 register
    I2CWriteRegister(ACCL_ADDR, 0x1F, 0x00); // Turns accelerometer and gyroscope off
}

//===================== Combining ACCL and BMP data. Implemented into UpdateFlightState ====================//

void UpdateFlightStateI2C() {
    float delta_i2c;
   
    // Function won't run if not enough data (less than 16 samples) have been collected
    if (!GetAltitudeDeltaI2C(&delta_i2c)) {
        return; 
    }
    delta_i2c = (fabsf(delta_i2c));
    
    
    //UpdateACCLStatusI2C();

    if (!accl_data_ready_i2c) {
        return;
    }

    //float accl_magnitude = ReadACCLI2C();
    float accl_magnitude = 0; 
    
    OverallAltitudeDeltaI2C(&OverallAltitudeChangeI2C);
    OverallAltitudeChangeI2C = (fabsf(OverallAltitudeChangeI2C));

    switch (current_flight_state) {
        
        case PREFLIGHT:
            if (delta_i2c > ALTITUDE_THRESHOLD && accl_magnitude > ACCL_THRESHOLD) {
                prev_flight_state = PREFLIGHT;
                current_flight_state = FLIGHT;
            }
            break;
            
        case FLIGHT:
            if (delta_i2c < ALTITUDE_THRESHOLD && OverallAltitudeChangeI2C < ALTITUDE_THRESHOLD && accl_magnitude < ACCL_THRESHOLD) {
                prev_flight_state = FLIGHT;
                current_flight_state = LANDED;
            }
            break;

        case LANDED:
            break;
    }
}

// ==================== ADC =========================//

// Initialise ADC for use. 
void InitADCRef() {
    //WDTCTL = WDTPW | WDTHOLD;                   // Stop watchdog timer

    PMMCTL2 |= REFVSEL_2 + INTREFEN_1;          // Enable and set the internal reference voltage. 
    while (!(PMMCTL2 & REFGENRDY));             // Wait until V_ref is ready to be used.
}

// InitADCGPIO intialises all 3 GPIO pins as analog inputs for the ADC
void InitADCGPIO(){
    // P5.0 (Chamber Thermister Temperature)
    P5SEL0 |= BIT0;
    P5SEL1 |= BIT0;

    // P1.4 (Battery Thermister Temperature)
    P1SEL0 |= BIT4;
    P1SEL1 |= BIT4;

    // P5.3 (Current Sense Value)
    P5SEL0 |= BIT3;
    P5SEL1 |= BIT3;

}

void ConfigADC() {
    ADCCTL0 = ADCSHT_4 | ADCON;                         // Sample and Hold time for 64 clk cycles | enable ADC (Start conversion later)

    // Use this clock when 16Mhz is not initialised:
    // ADCCTL1 = ADCSSEL_2 | ADCSHP;                       // Select SMCLK | signal sourced from sampling timer. (This clock causes issues ADC timing, results in incorrect ADC readings. SMCLK changes from ~1MHz (default) to 16MHz, which affects your ADC sample-and-hold timing. The ADC needs proper timing for accurate conversions.)

    // Use this clock when 16Mhz is initialised:
    ADCCTL1 = ADCSSEL_1 | ADCSHP;                      // Select ACLK | signal sourced from sampling timer. ACLK is 32.768kHz. (This should fix ADC timing issues and give correct ADC readings when 16mhz clock is initialised)

    ADCCTL2 = ADCRES_2;                                 // Set 12-Bit ADC Resolution
    ADCIE = ADCIE0;                                     // Enable interrupts
}

// Select ADC channel and set off interrupt to read ADC
void StartADC(uint8_t channel) {

    ADCCTL0 &= ~ADCENC;                  // Disable ADC
    ADCMCTL0 = channel | ADCSREF_0;      // Set channel + AVCC ref
    ADCCTL0 |= ADCENC | ADCSC;           // Start conversion
    __bis_SR_register( GIE);             // Go into low power mode 0 with interrupts enabled
}

// Converts ADC reading into temperature in celsius
int16_t TempConversion(volatile int16_t adcValue ) {
	//converting adc count to voltage 
	volatile double voltage_temp = (adcValue / ADCMAX) * VREF; 
	
	//calculate thermistor resistance 
	volatile double ThermistorResistance = RESISTOR * (VREF - voltage_temp)/ voltage_temp; // Should add error checking. ie dividing by 0

	//apply beta formula 
	volatile double InvertedTemp = (1.0/T0_K) + (1.0/ BETA) * log(ThermistorResistance/RTHERM); 
	volatile double TempInKelvin = 1/InvertedTemp; 
	
	//convert to Celsius 
	volatile double TempInCelsius = TempInKelvin - 273.15;

	return (int16_t)TempInCelsius;
}

// Return the temperature of the chamber 
int16_t GetChamberTemp () {
    volatile uint16_t chamThermADC;
    volatile uint16_t chamber_ADC_temperature;

    // Chamber thermistor
    StartADC(ADC_CHAM_THERM);
    __bis_SR_register(CPUOFF | GIE);            // Enter Sleep Mode and enable interrupts
    chamThermADC = adc_result; 

    
    // Convert temperatures
    chamber_ADC_temperature = TempConversion(chamThermADC);
    return chamber_ADC_temperature;
}   

// Return the temperature of the battery 
int16_t GetBatteryTemp () {
    volatile uint16_t batThermADC;
    volatile uint16_t battery_ADC_temperature;

    // Battery thermistor
    StartADC(ADC_BAT_THERM);
    __bis_SR_register(CPUOFF | GIE);        // Enter Sleep Mode and enable interrupts
    batThermADC = adc_result;
    
    battery_ADC_temperature = TempConversion(batThermADC);
    return battery_ADC_temperature;
}

// Return the value of current 
double CurrentSense () { 
    uint16_t curSenseADC;

    // Current sense
    StartADC(ADC_CUR_SENSE);
    __bis_SR_register(CPUOFF + GIE);        
    curSenseADC = adc_result;   

    double current = ((double)curSenseADC / ADCMAX) * VREF / R_SENSE;       // Convert ADC to current 
    return current;
}   

/* Code to turn off current when it exceeds a threshold. HERE IF NEEDED. 
void currExceedThreshold() {
    P5DIR |= BIT3;                       // Set P5.3 as output
    double current = currentSense ();
    if (current > CURRENT_MAX) {         // Threshold currently has placeholder value 
            P5OUT &= ~BIT3;              // Turn off current (port 5, pin 3) if it exceeds a value 
        } else {
            P5OUT |= BIT3;               // Turns back on if current return to a safe low value
        }
}
*/

bool CurrExceedsThreshold() {
    double current = CurrentSense();  // Gets current value 
    return (current > CURRENT_MAX);   // Returns true if over threshold, false otherwise
}

bool BatExceedsThreshold() {
    double battery = GetBatteryTemp();   
    return (battery > BATTERY_MAX);   
}

bool ChamExceedsThreshold() {
    double chamber = GetChamberTemp();   
    return (chamber > CHAMBER_MAX);   
}


// ==================== Backgorund Timer =========================//

// Global variables
volatile uint16_t delay_seconds = 0;
volatile uint8_t timer_expired = 0;

// Call the function to set an X second counter in the BACKGROUND
void start_delay_seconds(uint16_t seconds) {
    __disable_interrupt();
    delay_seconds = seconds;
    timer_expired = 0;
    __enable_interrupt();
}

// Initialise Timer_B for tick every 1 second. FKA: Timer3_init_1s_tick
void ConfigBackgroundTimer() {
    TB3CTL = TBSSEL__ACLK | MC__UP | TBCLR;
    TB3CCR0 = 32768 - 1;     // 1 second
    TB3CCTL0 = CCIE;
}

// ISR. Interrupt is called every second, decrementing delay time until 0. 
#pragma vector = TIMER3_B0_VECTOR
__interrupt void Timer3_B0_ISR(void) {
    if (delay_seconds > 0)
    {
        delay_seconds--;

        if (delay_seconds == 0)
        {
            timer_expired = 1;
        }
    }
}

// Example Working Template of Background Timer in main()
/*
    WDTCTL = WDTPW | WDTHOLD;

        ConfigBackgroundTimer();
        __enable_interrupt();

        start_delay_seconds(600);   // start 10-minute timer

        while (1)
        {
            if (timer_expired)
            {
                timer_expired = 0;
                // do the thing after 10 minutes

                // You can Rearm the timer here byt calling start_delay_seconds(6000) here for example.
            }

            // other background work here
            // state machines, IO, comms, etc.
        }

*/


// ==================== GPS  =========================//

volatile char gps_buffer[GPS_BUFFER_SIZE];
volatile uint8_t gps_index = 0;
volatile uint8_t gps_line_ready = 0;
// volatile double longitude = 0.0;
// volatile double latitude = 0.0;
// volatile uint8_t gps_sats = 0;
double longitude = 0.0;
double latitude = 0.0;
uint8_t gps_sats = 0;
char gps_time[10] = "00:00:00"; // HH:MM:SS

/*
Connect:
MSP430 4.2 (UART RX) --> GPS "TX" PIN
MSP430 4.3 (UART TX) --> GPS "RX" PIN


Old:
MSP430 P1.6 (TEMP UART RX) <-- GPS "TX" PIN
MSP430 P1.7 (TEMP UART TX) --> GPS "RX" PIN


MSP430 3v3 -- GPS 3v3
MSP430 GND -- GPS GND: 
*/

void InitGPSGPIO(){
    // Configure pins for UART
    P4SEL0 |= BIT2 | BIT3;   // RX + TX
    P4SEL1 &= ~(BIT2 | BIT3);
    // TX not used  
}

void ConfigGPSUART(){
    // Configure UART
    UCA1CTLW0 = UCSWRST;                // Put eUSCI in reset          
    UCA1CTLW0 |= UCSSEL__SMCLK;         // Select SMCLK as clock source
    UCA1BRW   = 8;
    UCA1MCTLW = UCOS16 | (10 << 4) | (0xF7 << 8);           // // 115200 baud @ 16MHz (working 115200 baud for beacon)
    
    // CRITICAL: Release UART from reset, enable interrupt, and unlock GPIO
    UCA1CTLW0 &= ~UCSWRST;                  // Release eUSCI from reset
    UCA1IE |= UCRXIE;                       // Enable RX interrupt
}

// Intialised GPS_enable boolean flag to prevent ISR code running redundantly when GPS reading is not required.
volatile bool GPS_enable = false;

// UART ISR
#pragma vector = EUSCI_A1_VECTOR // UART RX Interrupt Vector
__interrupt void EUSCI_A1_ISR(void) {

    //P1OUT ^= BIT0;              // Toggle LED for debugging
    char received = UCA1RXBUF;
    
    // Only store printable characters, comma, and newline
    if ((received >= 32 && received <= 126) || received == ',' || received == '\n') {
        if (gps_index < GPS_BUFFER_SIZE - 1) {
            gps_buffer[gps_index++] = received;
        } else {
            gps_index = 0; // reset on overflow
        }
    }

    // End of line
    if (received == '\n') {
        gps_buffer[gps_index] = '\0';
        gps_index = 0;
        gps_line_ready = 1;
        __bic_SR_register_on_exit(CPUOFF);
    }
    
}

void InitClock16MHz(void){
    
    __bis_SR_register(SCG0);                 // disable FLL
    CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_5;// DCOFTRIM=3, DCO Range = 16MHz
    CSCTL2 = FLLD_0 + 487;                   // DCODIV = 16MHz (N = 487: (487+1) * 32768 Hz ≈ 16 MHz)
    __delay_cycles(3);
    __bic_SR_register(SCG0);                 // enable FLL

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, MCLK/SMCLK from DCOCLKDIV
}

void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * 16);// Wait FLL lock status (FLLUNLOCK) to be stable

        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;      // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}

void ClearGPSBuffer(void){
    uint8_t i;
    for (i = 0; i < GPS_BUFFER_SIZE; i++) {
        gps_buffer[i] = 0;
    }
    gps_index = 0;
    gps_line_ready = 0;
}

// Locate GNGGA sentences
static inline bool is_gngga(const char *s) {
    return (s[0] == '$' &&
            s[1] == 'G' &&
            s[2] == 'N' &&
            s[3] == 'G' &&
            s[4] == 'G' &&
            s[5] == 'A');
}

/* NOT needed for working LoRa
double nmea_to_decimal(double coord, char hemi) {
    int deg = (int)(coord / 100.0);
    double min = coord - (deg * 100.0);
    double dec = deg + min / 60.0;

    if (hemi == 'S' || hemi == 'W')
        dec = -dec;

    return dec;
}

void parse_gngga(char *line) {

    char *token;
    uint8_t field = 0;

    double lat_raw = 0.0;
    double lon_raw = 0.0;
    char ns = 0, ew = 0;
    uint8_t fix = 0;
    uint8_t sats = 0;

    // Seperate sentence
    token = strtok(line, ",");

    // Locate relevant sections of GNNGA sentence
    while (token) {
        field++;

        if (field == 2) {
            // Parse time field (hhmmss.ss)
            int hh = 0, mm = 0, ss = 0;
            if (strlen(token) >= 6) {
                hh = (token[0] - '0') * 10 + (token[1] - '0');
                mm = (token[2] - '0') * 10 + (token[3] - '0');
                ss = (token[4] - '0') * 10 + (token[5] - '0');
                // Format as HH:MM:SS
                gps_time[0] = '0' + (hh / 10);
                gps_time[1] = '0' + (hh % 10);
                gps_time[2] = ':';
                gps_time[3] = '0' + (mm / 10);
                gps_time[4] = '0' + (mm % 10);
                gps_time[5] = ':';
                gps_time[6] = '0' + (ss / 10);
                gps_time[7] = '0' + (ss % 10);
                gps_time[8] = '\0';
            } else {
                strcpy(gps_time, "00:00:00");
            }
        }
        else if (field == 3) lat_raw = atof(token);
        else if (field == 4) ns = token[0];
        else if (field == 5) lon_raw = atof(token);
        else if (field == 6) ew = token[0];
        else if (field == 7) fix = atoi(token);
        else if (field == 8) sats = atoi(token);

        token = strtok(NULL, ",");
    }

    //if (fix == 0)return;   // no GPS fix

    double lat = nmea_to_decimal(lat_raw, ns);
    double lon = nmea_to_decimal(lon_raw, ew);

    latitude = lat;
    longitude = lon;
    gps_sats = sats;
}
*/

// append_char() appends a character to the current position of a character pointer and then advances the pointer to the next position.
static inline void append_char(char **p, char c) {
    **p = c; (*p)++;
}

// append_str() copies a string into a buffer
static inline void append_str(char **p, const char *s) {
    while (*s) { **p = *s++; (*p)++; }
}

// append_uint_pad() turns a number into text, pads it with leading zeros if needed, and appends it to a buffer.
static void append_uint_pad(char **p, uint32_t v, uint8_t min_width) {
    char tmp[12];  // Increased size for uint32_t
    int8_t i = 0;
    if (v == 0) {
        tmp[i++] = '0';
    } else {
        while (v > 0) {
            tmp[i++] = '0' + (v % 10);
            v /= 10;
        }
    }
    while (i < min_width) tmp[i++] = '0';
    for (int8_t j = i - 1; j >= 0; --j) append_char(p, tmp[j]);
}

// append_coordinate() converts a floating-point coordinate into text with a fixed number of decimal places and appends it to a buffer.
static void append_coordinate(char **p, double val, uint8_t frac_digits) {
    // Handle negative
    if (val < 0.0) {
        append_char(p, '-');
        val = -val;
    }
    
    // Split into whole and fractional parts
    uint32_t whole = (uint32_t)val;
    
    // Calculate fractional part scaled to desired digits
    double frac_float = val - (double)whole;
    uint32_t scale = 1;
    for (uint8_t i = 0; i < frac_digits; i++) scale *= 10;
    uint32_t frac = (uint32_t)(frac_float * scale + 0.5);  // Round
    
    // Handle rounding overflow (like 0.999999 -> 1.000000)
    if (frac >= scale) {
        whole++;
        frac = 0;
    }
    
    // Append whole part
    char tmp[12];
    int8_t i = 0;
    if (whole == 0) {
        tmp[i++] = '0';
    } else {
        while (whole > 0) {
            tmp[i++] = '0' + (whole % 10);
            whole /= 10;
        }
    }
    for (int8_t j = i - 1; j >= 0; --j) append_char(p, tmp[j]);
    
    // Append decimal point
    append_char(p, '.');
    
    // Append fractional part with leading zeros
    append_uint_pad(p, (uint16_t)frac, frac_digits);
}


// ==================== OPUS 4.6 ==================== ///
// INT Approach

// Store coordinates as integers (degrees * 1,000,000) to avoid float precision loss
volatile int32_t latitude_i = 0;   // -36607300 means -36.607300
volatile int32_t longitude_i = 0;  // 174690066 means 174.690066

// Parse NMEA coord token directly to integer (degrees * 1,000,000)
// Input: "3636.44029" (DDMM.MMMMM) or "17441.40395" (DDDMM.MMMMM)
static int32_t parse_nmea_to_int(const char *token) {
    if (token == NULL || token[0] == '\0') return 0;

    // Find decimal point
    const char *dot = token;
    while (*dot && *dot != '.') dot++;
    int dot_pos = (int)(dot - token);
    if (dot_pos < 2) return 0;

    // Parse integer part (DDMM or DDDMM)
    int32_t int_part = 0;
    for (int i = 0; i < dot_pos; i++) {
        int_part = int_part * 10 + (token[i] - '0');
    }

    // Parse fractional part (up to 5 digits for minutes)
    int32_t frac_part = 0;
    int frac_digits = 0;
    if (*dot == '.') {
        const char *fp = dot + 1;
        while (*fp >= '0' && *fp <= '9' && frac_digits < 5) {
            frac_part = frac_part * 10 + (*fp - '0');
            frac_digits++;
            fp++;
        }
    }
    while (frac_digits < 5) {
        frac_part *= 10;
        frac_digits++;
    }

    // Split degrees and minutes
    int32_t degrees = int_part / 100;
    int32_t minutes_int = int_part % 100;

    // Total minutes * 100000 (integer)
    // minutes_int is whole minutes, frac_part is fractional minutes * 10^5
    int32_t total_min_scaled = (int32_t)minutes_int * 100000L + frac_part;

    // Convert minutes to degrees:
    // degrees_frac = total_min_scaled / 60
    // This gives us fractional degrees * 100000 / 60
    // We want result in millionths of a degree (10^6)
    // result = degrees * 1000000 + (total_min_scaled * 10) / 60
    // Because total_min_scaled is minutes * 10^5, multiply by 10 to get * 10^6, then /60

    int32_t frac_deg = (total_min_scaled * 10L + 30L) / 60L;  // +30 for rounding

    return degrees * 1000000L + frac_deg;
}

void parse_gngga(char *line) {
    char *token;
    uint8_t field = 0;

    int32_t lat_raw = 0;
    int32_t lon_raw = 0;
    char ns = 0, ew = 0;
    uint8_t fix = 0;
    uint8_t sats = 0;

    token = strtok(line, ",");

    while (token) {
        field++;

        if (field == 2) {
            if (strlen(token) >= 6) {
                // gps_time[0] = token[0];
                // gps_time[1] = token[1];
                // gps_time[2] = ':';
                // gps_time[3] = token[2];
                // gps_time[4] = token[3];
                // gps_time[5] = ':';
                // gps_time[6] = token[4];
                // gps_time[7] = token[5];
                // gps_time[8] = '\0';

                // below code is more modular in terms of time formatting:
                int hh = (token[0] - '0') * 10 + (token[1] - '0');
                int mm = (token[2] - '0') * 10 + (token[3] - '0');
                int ss = (token[4] - '0') * 10 + (token[5] - '0');

                // Add 12 hour offset and wrap around at 24
                hh = (hh + 13) % 24;

                gps_time[0] = '0' + (hh / 10);
                gps_time[1] = '0' + (hh % 10);
                gps_time[2] = ':';
                gps_time[3] = '0' + (mm / 10);
                gps_time[4] = '0' + (mm % 10);
                gps_time[5] = ':';
                gps_time[6] = '0' + (ss / 10);
                gps_time[7] = '0' + (ss % 10);
                gps_time[8] = '\0';          
            }
        }
        else if (field == 3) lat_raw = parse_nmea_to_int(token);
        else if (field == 4) ns = token[0];
        else if (field == 5) lon_raw = parse_nmea_to_int(token);
        else if (field == 6) ew = token[0];
        else if (field == 7) fix = atoi(token);
        else if (field == 8) sats = atoi(token);

        token = strtok(NULL, ",");
    }

    latitude_i = (ns == 'S') ? -lat_raw : lat_raw;
    longitude_i = (ew == 'W') ? -lon_raw : lon_raw;
    gps_sats = sats;
}

// Append integer coordinate as decimal string: -36607300 -> "-36.607300"
static void append_coord_int(char **p, int32_t val) {
    if (val < 0) {
        append_char(p, '-');
        val = -val;
    }

    int32_t whole = val / 1000000L;
    int32_t frac = val % 1000000L;

    // Append whole part
    char tmp[8];
    int8_t i = 0;
    if (whole == 0) {
        tmp[i++] = '0';
    } else {
        while (whole > 0) {
            tmp[i++] = '0' + (whole % 10);
            whole /= 10;
        }
    }
    for (int8_t j = i - 1; j >= 0; --j) append_char(p, tmp[j]);

    // Decimal point
    append_char(p, '.');

    // Append frac with leading zeros (always 6 digits)
    append_uint_pad(p, (uint32_t)frac, 6);
}
// ====================  OPUS 4.6 ====================  ///

// ==================== LoRa =========================//
/*
LoRa code was written with the SPI_B1 moduel in mind. This corresponds to
UCB1STE = P4.4
UCB1CLK = P4.5
UCB1MOSI = P4.6
UCB1MISO = P4.7

All these pins are broken out on the DEV board which means we can use them temporarily. But, we will need to change them onte CTRL Freaks second revision

*/

//#define MESSAGE {'H','e','l','l','o'}
#define PREAMBLE_LENGTH 8
#define TIMEOUT_IN_SYMBOLS 1023 //max timeout

// Chip Select pin
GpioPin radioChipSelPin = {&P4DIR, &P4OUT, BIT4}; // P4.4

// INT approach version of LoRaTX - uses integer coordinates to avoid floating point in packet construction. Packet format is the same, but lat and lon are sent as integers representing the value multiplied by 1e6 (ie 12.345678 -> 12345678). The append_coordinate function is replaced with a simpler append_coord_int that just appends the integer value with a decimal point inserted at the correct place in the receiver software. This should be more efficient on the MSP430 while still providing sufficient precision for GPS coordinates.
void LoRaTX() {

    // // Default hard code tesd
    // uint8_t data[] = MESSAGE;
    // while(1){
    //     radio_transmit_start(data, 5, radioChipSelPin);

    //     while((radio_transmit_is_complete(radioChipSelPin)) != TX_OK);
    // }
    
    char data[128];
    char *p = data;

    // Copy volatile globals safely
    uint8_t sats = gps_sats;
    int32_t lat = latitude_i;
    int32_t lon = longitude_i;

    // Determine fix type
    char fix_type = (sats == 0) ? 'N' : 'G';
    if (sats > 15) sats = 15;

    // Build packet: #E 00:00:00 UTC; Y; ZZ; LAT,LON; 0.0m\n
    append_char(&p, '#');
    append_char(&p, 'E');
    append_char(&p, ' ');

    // Time
    append_str(&p, gps_time);
    append_str(&p, " UTC; ");

    // Fix type
    append_char(&p, fix_type);
    append_str(&p, "; ");

    // Satellite count (2 digits, zero padded)
    append_uint_pad(&p, sats, 2);
    append_str(&p, "; ");

    // Coordinates
    if (sats > 0) {
        append_coord_int(&p, lat);
        append_char(&p, ',');
        append_coord_int(&p, lon);
    } else {
        append_str(&p, "0.000000,0.000000");
    }

    // Altitude placeholder
    append_str(&p, "; 0.0m\n");

    // Null terminate
    *p = '\0';

    // Transmit
    uint16_t len = (uint16_t)(p - data);
    radio_transmit_start((uint8_t*)data, len, radioChipSelPin);
    while ((radio_transmit_is_complete(radioChipSelPin)) != TX_OK);
}

// Loop until either we received a packet or the Rx operation times out
RadioRxStatus wait_for_received_packet(uint8_t data_received[], uint16_t* message_length_bytes) {
    while(1) {
        RadioRxStatus result = radio_receive_is_complete(data_received, message_length_bytes, radioChipSelPin);
        if (result == SUCCESSFUL_RX) {
            return SUCCESSFUL_RX;
        }
        if (result == ERR_RX_TIMEOUT_FAIL) {
            return ERR_RX_TIMEOUT_FAIL;
        }
    }
}

void InitLoRaGPIO() {
    // Radio chip select pin
    *radioChipSelPin.pdir |= radioChipSelPin.pin; // Set as output
    *radioChipSelPin.pout |= radioChipSelPin.pin; // Set HIGH

    // LEDs: Set P2.0, 2.1, 2.2 to output
    P2DIR |= (BIT0 | BIT1 | BIT2);

    // Unlock GPIO
    // PM5CTL0 &= ~LOCKLPM5; commented out for integration
    P2OUT |= (BIT0 | BIT1 | BIT2);
}

void flash_led_red(void) {
    P2OUT ^= BIT0; // Toggle P2.0
    __delay_cycles(50000);
}

void flash_led_yellow_blue(void) {
    P2OUT ^= BIT1; // Toggle P2.1
    __delay_cycles(50000);
}

void flash_led_green(void) {
    P2OUT ^= BIT2; // Toggle P2.2
    __delay_cycles(50000);
}

// ==================== GPS + LoRa integration =========================//

// TransmitGPS assumes that the GPIO Pins, modules, and initialisations have been completed. This function should read GPS data, and transmit it over LoRa
// The values [Lat, Lon] should be transmitted. [0, 0] should be transmittted when there is no fix. 
void TransmitGPS() {
    //GPS_enable = true;                // Set true - include flag check in ISR!

    __bis_SR_register(GIE);             // Enable Interrupt
    if (gps_line_ready) {

        // Disable UART RX interrupt to prevent ISR corrupting gps_buffer during parse/transmit
        // Temporarily disbale GPS to prevent buffer from being overriden during transmission
        UCA1IE &= ~UCRXIE;

        gps_line_ready = 0;

        // Parse gngga only
        if (is_gngga((char*)gps_buffer)) {

            parse_gngga((char*)gps_buffer);
            LoRaTX(); 
        } 
    }

    // Re-enable UART RX interrupt
    UCA1IE |= UCRXIE;
}

// ==================== Run State Behaviour =========================//

// Need to define recovery_active. Where is this happening?
// Also need to move RecoveryMode above this function. 


void RunStateBehaviour() {
    switch (current_flight_state) {
        case PREFLIGHT:
            break;
        case FLIGHT:
            // Log some data here;
            break;
        case LANDED:
            // if (!recovery_active){
            //     recovery_active = true;
            //     RecoveryMode();
            // }
            break;
        default:
            break;
    }
}


// ==================== Recovery Function =========================//

void RecoveryMode(void) {
    uint8_t duty_cycle = 0;
    bool pwm_enabled = false;
    bool initial_delay_done = false;

    DisableBMP();
    DisableACCL();

    // InitCoilGPIO(); 
    // InitFanGPIO(); 
    // gpioUnlock();
    // ConfigCoilPWM(); 
    // ConfigFanPWM();

    // ConfigBackgroundTimer();
    // __enable_interrupt();
    
    start_delay_seconds(450); // Initial 7.5 minute delay

    while (1)
    {
        // Stage 1: During first 7.5 minutes: Communicating with LoRa to receive GPS information. After 7.5min, move onto stage 2. 
        if (!initial_delay_done)
        {
            // LoRa + GPS work here
            if (timer_expired)
            {
                timer_expired = 0;
                initial_delay_done = true;

                // Start PWM ON window for 15 seconds. Smoke release. 
                pwm_enabled = true;
                duty_cycle = 100;
                SetCoilPWM(duty_cycle);
                InitRegulatorGPIO();
                EnableRegulator();
                SetFanPWM(duty_cycle);

                start_delay_seconds(15);
            }
        }

        // Stage 2: Timer for 15 seconds has started. The PWM Coil is on. Continuous monitoring of current and temperature.
        if (pwm_enabled) {

            // This loop is blocking so there's no other background task that happens within the 15 seconds. 
            while (!timer_expired) {
                // Safety checks
                if (CurrExceedsThreshold() || BatExceedsThreshold() || ChamExceedsThreshold()) {
                    duty_cycle = 0;
                    SetCoilPWM(duty_cycle); 
                    SetFanPWM(duty_cycle);
                }
            }

            timer_expired = 0;

            // Finish stage 2. After 15 seconds has passed, turn off PWM. 
            pwm_enabled = false;
            SetCoilPWM(0);
            SetFanPWM(0);

            // Start cooldown
            start_delay_seconds(45);
        }

        // Stage 3: 
        if (initial_delay_done && !pwm_enabled)
        {
            // 45 second cooldown. Allow wick to resaturate. 
            SetCoilPWM(0);
            SetFanPWM(0);

            // Other background tasks here

            if (timer_expired)
            {
                timer_expired = 0;

                // Restart PWM ON window
                pwm_enabled = true;
                duty_cycle = 100;
                SetCoilPWM(duty_cycle);
                InitRegulatorGPIO();
                EnableRegulator();
                SetFanPWM(duty_cycle);

                start_delay_seconds(15);
            }
        }
    }
}

// InitGPIO() initialises all GPIO pins
void InitGPIO() {
    InitCoilGPIO();                 // P5.1
    InitFanGPIO();                  // P6.1
    InitADCRef();
    InitADCGPIO();                  // P5.0, P1.4, P5.3
    InitRegulatorGPIO();            // P3.5

    InitSensorsGPIO();
    
    InitLoRaGPIO();                 // P4.4 (CS), SPI pins initialised in spi_b1_init(), should be fine.
    InitGPSGPIO();                  // P4.2, P4.3
}

void InitOnboardLEDS() {
    // Init LEDs
    P1DIR |= RED_LED; // Equivalent of P1DIR |= BIT0; due to the #DEFINE at the top of the program
    P6DIR |= GREEN_LED;

    // Turn LEDs off.
    P1OUT &= ~RED_LED;
    P6OUT &= ~GREEN_LED;
}

// Unlock GPIO once here

// ConfigPeripheral() configures all peripherals such as timers, clocks, modules
void ConfigPeripheral() { 
    ConfigCoilPWM();
    ConfigFanPWM();
    ConfigADC();
    
    ConfigSensorsI2C();

    ConfigBackgroundTimer();
    spi_B1_init();          // "ConfigLoRaSPI"
    lora_configure(
            BANDWIDTH125K,              // 125 khz
            CODINGRATE4_5,              // Coding rate 4/5
            CRC_ENABLE,                 // Enable CRC
            EXPLICIT_HEADER_MODE,       // Explicit header
            POLARITY_NORMAL_MODE,       // Normal IQ
            PREAMBLE_LENGTH,            // Preamble 8
            SPREADINGFACTOR128,         // SF7
            SYNC_WORD_RESET,            // 0x12
            radioChipSelPin
    );
    ConfigGPSUART();
}


// ==================== Main Flight Loop (Start) =========================//

int main(void) {
    
    WDTCTL = WDTPW | WDTHOLD;           // Stop Watchdog Timer

    InitClock16MHz();                   // Init 16Mhz CLK
    Software_Trim();                    // Compensate Software Dift

    InitGPIO();                         // Init all GPIO
    gpioUnlock();                       // Unlock GPIO
    ConfigPeripheral();                 // Config Peripherals
    
    //ConfigBMPI2C();
    //CalibrateBMPI2C(&ground_pressure_i2c, &initial_altitude_i2c);    // Averaging samples pre-flight to calculate initial altitude 
    //ConfigACCLI2C();

    __enable_interrupt();               // enable interrupts

    // while(1){
    //     ProcessBMPDataI2C();       // Read sensors 
    //     UpdateFlightStateI2C();    // Update state 
    //     // read sensors (ProcessBMPDataI2C())
    //     // update state (UpdateFlightStateI2C())
    //     // behaviour (RunStateBehaviour())
    // }

    // Testing:
    while(1){
        TransmitGPS();
    }
    
}

// ==================== Main Flight Loop (END) =========================//
    
  


    // old main function
    /*
    bool recovery_active = false;

    // LoRa
    
    InitADCRef();
    InitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 

    InitACCL();
    ConfigureACCL();

    InitBMP();
    ConfigureBMP();

    ground_pressure = CalibrateGroundPressure(100);
    initial_altitude = PressureToAltitude(ground_pressure);

    current_flight_state = PREFLIGHT;

    // LoRa to ensure init was done succesfully 

    __enable_interrupt(); 

    while(1){
        
        //Read from sensors here
        

        // 1. Check if new BMP sample ready
        if (bmp_data_ready) {
            bmp_data_ready = false;
            GetPressure();
                if (available_samples > 0) {
                    // read the next sample from buffer
                    //BMPData raw = bmpBuffer[read_index]; // RAW WTF???
                    const volatile BMPData& raw = bmpBuffer[read_index];
                    //I think this is meant to be raw_pressure?

                    // advance read_index and decrease available_samples
                    read_index = (read_index + 1) % BMP_BUFFER_SIZE;
                    available_samples--;

                    // convert to altitude and push to sliding window
                    // float altitude = PressureToAltitude(bmpBuffer[read_index].pressure);
                    float altitude = PressureToAltitude(raw.pressure); //This is meant to raw_pressure?
                    AltWindow_Push(altitude);
                }
        }


        // Update Flight State // should pass in small sample of data for flight determination.
        UpdateFlightState();

        // State-depened behaviour
        switch (current_flight_state) {
            case PREFLIGHT:
                break;
            case FLIGHT:
                break;
            case LANDED:
                if (!recovery_active){
                    // Enter recovery
                    RecoveryMode();
                    recovery_active = true;
                }
                break;
        }
    }
    */

    
    // Successful pulsing of PWM using LED
    /*
    WDTCTL = WDTPW + WDTHOLD; // Stop Watchdog Timer
    InitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 

    while (1) {
		for (uint16_t i = 0; i < 100; i++) {
            SetCoilPWM(i);
            __delay_cycles(50 * PWM_PERIOD);
		}
		for (uint16_t i = 100; i > 0; i--) {
			SetCoilPWM(i);
            __delay_cycles(50 * PWM_PERIOD);
		}
	}
    */

    // Code to test coil PWM on PCB
    /*
    WDTCTL = WDTPW + WDTHOLD;
    InitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 

    while (1) {
        SetCoilPWM(30);
    }
    */
    
    // Code to test fan PWM
    /*
    WDTCTL = WDTPW + WDTHOLD; // Stop Watchdog Timer
    InitFanGPIO(); 
    gpioUnlock();
    ConfigFanPWM();
    SetFanPWM(100);
    */
    


    // Successful use of background timer interrupts with toggling LEDs (other task)
    /*
    WDTCTL = WDTPW | WDTHOLD;

    // Initialise timer
    ConfigBackgroundTimer();

    // Init PWM
    IInitCoilGPIO(); 
    gpioUnlock();
    ConfigCoilPWM(); 
    
    // Init LEDs
    P1DIR |= RED_LED; // Equivalent of P1DIR |= BIT0; due to the #DEFINE at the top of the program
    P6DIR |= GREEN_LED;

    // Turn LEDs off.
    P1OUT &= ~RED_LED;
    P6OUT &= ~GREEN_LED;

    __enable_interrupt();

    // Being 40 seconds timer
    start_delay_seconds(40);   // start 40 seconds timer
    SetCoilPWM(100); // Turn LED ON

    // Set initial values - red LED on, green LED off.
    P1OUT |= RED_LED;
    P6OUT &= ~GREEN_LED;

    while (1)
    {
        if (timer_expired)
        {
            timer_expired = 0;
            // Execute task after 40 seconds end
            SetCoilPWM(0); // Turn LED OFF (Mock PWM Coil)
        }

        // other background work here. in this case it would be LoRa
        // state machines, IO, comms, etc.

        P1OUT ^= RED_LED;
        P6OUT ^= GREEN_LED;
        __delay_cycles(100000); // 100ms delay at 1MHz (default clock)
    }
    */



    //ADC Testing:
    /*
    WDTCTL = WDTPW | WDTHOLD;    
    InitADCRef();               // I believe setting the reference can be put after InitADCGPIO

    // P1DIR |= BIT0;             // RED LED as output
    // P1OUT &= ~BIT0;            // Turn RED LED off

    // Configure P5.0
    InitADCGPIO();

    ConfigADC();

    PM5CTL0 &= ~LOCKLPM5;   // Unlock GPIO pins

    uint16_t chamber_temperature;
    uint16_t battery_temperature;

    while(1)
    {
        // Start Conversion
        chamber_temperature = GetChamberTemp();
        //battery_temperature = GetBatteryTemp();
        
    }
    */
    
    
    // Reading BMP data (pressure data in Pa), and using it in UpdateFlightState(I2C)
    /*
    volatile float pressure;
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer. 

    InitSensorsGPIO();
    ConfigSensorsI2C();
    ConfigBMPI2C();
    CalibrateBMPI2C(&ground_pressure_i2c, &initial_altitude_i2c); 
    //ConfigACCLI2C();

    while (1) {
        ProcessBMPDataI2C(); 
        UpdateFlightStateI2C();
    }
    */


    // Successful GPS testing: watch LAT LON. Correct values when fixed. ZERO otherwise.
    /*
    WDTCTL = WDTPW | WDTHOLD;
    InitClock16MHz();
    Software_Trim();

    P1DIR |= BIT0;             // RED LED as output
    P1OUT &= ~BIT0;            // Turn RED LED off
    ClearGPSBuffer();
    InitGPSGPIO(); 
    ConfigGPSUART();
    
    while(1){
        __bis_SR_register(GIE); // Enter LPM0, Enable Interrupt
        if (gps_line_ready) {
            gps_line_ready = 0;

            // Loops through buffer to clear NULL
            // --- Add this BEFORE calling parse_gpgga --- sanitise buffer before passing to remove NULL which stops strtok from working
            // for (int i = 0; i < gps_index; i++) {
            //     if (gps_buffer[i] == 0) gps_buffer[i] = ',';  // replace nulls with comma
            // }
            // gps_buffer[gps_index] = '\0'; // ensure string is properly terminated
            
            //parse_gngga((char*)gps_buffer);   // attempt to parase after every sentence

            // Parse gngga only
            if (is_gngga((char*)gps_buffer)) {
                parse_gngga((char*)gps_buffer);
            } 
        }
    }
    */

    // LoRa Code
    /*
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    InitLoRaGPIO();
    PM5CTL0 &= ~LOCKLPM5;   // Unlock GPIO
    spi_B1_init();

    lora_configure(
            BANDWIDTH125K,              // 125 khz
            CODINGRATE4_5,              // Coding rate 4/5
            CRC_ENABLE,                 // Enable CRC
            EXPLICIT_HEADER_MODE,       // Explicit header
            POLARITY_NORMAL_MODE,       // Normal IQ
            PREAMBLE_LENGTH,            // Preamble 8
            SPREADINGFACTOR128,         // SF7
            SYNC_WORD_RESET,            // 0x12
            radioChipSelPin
    );
    
    LoRaTX(); 
    */
 
    // GPS testing for main
    /*
    while(1){
        TransmitGPS();
    }
    */



    // Subsystem list
/*
    coil pwm    done and tested
    fan pwm     done and tested
    bmp         done and tested
    accl        done and tesded
    adc         done and tested
    gps         done and tested
    lora        done and tested
    background timer    done and tested
    regulator enable    done

*/
