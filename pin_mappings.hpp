#pragma once
#include <stdint.h>
#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"

// Templates for BMP 
extern Pin<P4,4> BMP_CS;
extern Pin<P4,5> BMP_CLK;
extern Pin<P4,6> BMP_MOSI;
extern Pin<P4,7> BMP_MISO;
extern SpiMaster<SPI_B1> BMP_SPI;
extern Pin<P2,4> BMP_INT;

// Templates for ACCL 
extern Pin<P4,0> ACCL_CS;
extern Pin<P4,1> ACCL_CLK;
extern Pin<P4,2> ACCL_MOSI;
extern Pin<P4,3> ACCL_MISO;
extern SpiMaster<SPI_A1> ACCL_SPI; //CHECK on datasheet
extern Pin<P6,0> ACCL_INT1;
extern Pin<P2,3> ACCL_INT2;
