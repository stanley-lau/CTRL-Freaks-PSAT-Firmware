#ifndef GPIO_H
#define GPIO_H

struct GpioPin {
    volatile uint8_t* pdir;
    volatile uint8_t* pout;
    uint8_t pin;
};

#endif
