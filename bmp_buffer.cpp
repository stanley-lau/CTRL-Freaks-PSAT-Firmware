#include "bmp_buffer.hpp"

// Circular buffer storage
volatile BMPData bmpBuffer[BMP_BUFFER_SIZE];

volatile uint8_t write_index = 0;
volatile uint8_t read_index = 0;
volatile uint8_t available_samples = 0;

// Altitude window storage
AltitudeWindow altWin = {0};
