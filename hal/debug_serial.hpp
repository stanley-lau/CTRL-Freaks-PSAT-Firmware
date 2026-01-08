#ifndef DEBUG_SERIAL_HPP
#define DEBUG_SERIAL_HPP

// This includes an embedded-friendly implementation of the C std library print functions. 
// NOTE: the function names all end with an underscore.
// printf_();   // Write to a serial device 
// sprintf_();  // Write to an array/buffer (deprecated - use snprintf() instead)
// snprintf_(); // Write to an array/buffer with buffer overflow prevention

// ... as well as the variants that take the arguments as an array instead.
// vprintf_();
// vsprintf_();
// vsnprintf_();

#include "printf/printf.h"

/** IMPORTANT: You MUST implement void putchar_(char c) to tell the library how to write a byte to serial! **/

// Your implementation of putchar_() will probably look like:
/* 
Uart<UART_A1> debugSerial;
Pin<...> a1_tx;
Pin<...> a1_rx;

void putchar_(char c) {
    debugSerial.writeByte(c);
}

void main() {
    /// Configure GPIO for UART mode and initialise UART before calling print fns!
    a1_tx.function(...);
    a1_rx.function(...);
    debugSerial.init( ... ); 

    printf_("Hello world!");
}
*/

#endif
