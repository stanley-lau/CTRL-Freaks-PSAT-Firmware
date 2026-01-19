#pragma once
#include <stdint.h>

// Circular buffer size for BMP
constexpr uint8_t BMP_BUFFER_SIZE = 32;

// Struct for storing pressure + temperature
struct BMPData {
    int32_t pressure;
    int32_t temperature;
};

// Circular buffer (declared, not defined)
extern volatile BMPData bmpBuffer[BMP_BUFFER_SIZE];
extern volatile uint8_t write_index;
extern volatile uint8_t read_index;
extern volatile uint8_t available_samples;


// ---- ALTITUDE WINDOW ----

// Window size (8 samples ≈ 160 ms @ 50 Hz)
constexpr uint8_t ALT_WINDOW = 8;

// Struct for storing altitude-only data
struct AltitudeWindow {
    float buf[ALT_WINDOW];
    uint8_t idx;
    uint8_t count;
};

// Declare the instance
extern AltitudeWindow altWin;
