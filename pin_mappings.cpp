#include "pin_mappings.hpp"

// Templates for BMP 
Pin<P4,4> BMP_CS;
Pin<P4,5> BMP_CLK;
Pin<P4,6> BMP_MOSI;
Pin<P4,7> BMP_MISO;
SpiMaster<SPI_B1> BMP_SPI;
Pin<P2,4> BMP_INT;

// Templates for ACCL 
Pin<P4,0> ACCL_CS;
Pin<P4,1> ACCL_CLK;
Pin<P4,2> ACCL_MOSI;
Pin<P4,3> ACCL_MISO;
SpiMaster<SPI_A1> ACCL_SPI; //CHECK on datasheet
Pin<P6,0> ACCL_INT1;
Pin<P2,3> ACCL_INT2;
