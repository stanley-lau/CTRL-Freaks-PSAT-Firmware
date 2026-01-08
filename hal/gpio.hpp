#ifndef GPIO_HPP
#define GPIO_HPP

#include <msp430.h>
#include <stdint.h>
#include <type_traits>

#include "util.hpp"

enum class PinFunction {
    Gpio,
    Primary,
    Secondary,
    Tertiary,
};

enum class PullDir {
    Floating,
    Pulldown,
    Pullup,
};

#define P1 &P1DIR, &P1IE,   &P1IES,  &P1IFG,  &P1IN, &P1IV,   &P1OUT, &P1REN, &P1SEL0, &P1SEL1
#define P2 &P2DIR, &P2IE,   &P2IES,  &P2IFG,  &P2IN, &P2IV,   &P2OUT, &P2REN, &P2SEL0, &P2SEL1
#define P3 &P3DIR, &P3IE,   &P3IES,  &P3IFG,  &P3IN, &P3IV,   &P3OUT, &P3REN, &P3SEL0, &P3SEL1
#define P4 &P4DIR, &P4IE,   &P4IES,  &P4IFG,  &P4IN, &P4IV,   &P4OUT, &P4REN, &P4SEL0, &P4SEL1
#define P5 &P5DIR, nullptr, nullptr, nullptr, &P5IN, nullptr, &P5OUT, &P5REN, &P5SEL0, &P5SEL1
#define P6 &P6DIR, nullptr, nullptr, nullptr, &P6IN, nullptr, &P6OUT, &P6REN, &P6SEL0, &P6SEL1

#define PIN_PARAMS PxDIR, PxIE, PxIES, PxIFG, PxIN, PxIV, PxOUT, PxREN, PxSEL0, PxSEL1, pin_num

// Ports 5 and 6 don't have interrupt registers so we can't just take the base and calculate offsets
template< 
    volatile uint8_t*  PxDIR, 
    volatile uint8_t*  PxIE,
    volatile uint8_t*  PxIES,
    volatile uint8_t*  PxIFG,
    volatile uint8_t*  PxIN,
    volatile uint16_t* PxIV,
    volatile uint8_t*  PxOUT, 
    volatile uint8_t*  PxREN, 
    volatile uint8_t*  PxSEL0, 
    volatile uint8_t*  PxSEL1, 
    uint8_t pin_num
>
struct Pin {
    static constexpr uint8_t pinMask = (1 << pin_num);
    static constexpr int8_t adcChannel = calculateAdcChannel(PxOUT, pin_num);

    private:
    static constexpr int8_t calculateAdcChannel(volatile uint8_t* port, uint8_t pin_n) {
        if (port == &P1OUT) {
            switch (pin_n) {
                case 0: return 0;
                case 1: return 1;
                case 2: return 2;
                case 3: return 3;
                case 4: return 4;
                case 5: return 5;
                case 6: return 6;
                case 7: return 7;
                default: return -1;
            }
        }
        if (port == &P5OUT) {
            switch (pin_n) {
                case 0: return 8;
                case 1: return 9;
                case 2: return 10;
                case 3: return 11;
                default: return -1;
            }
        }
        return -1;
    }
    public:
    static Pin function(PinFunction fn) {
        switch(fn){
        case PinFunction::Gpio:
            CLEAR_BITS(PxSEL0, pinMask);
            CLEAR_BITS(PxSEL1, pinMask);
            break;
        case PinFunction::Primary:
            SET_BITS(PxSEL0, pinMask);
            CLEAR_BITS(PxSEL1, pinMask);
            break;
        case PinFunction::Secondary:
            CLEAR_BITS(PxSEL0, pinMask);
            SET_BITS(PxSEL1, pinMask);
            break;
        case PinFunction::Tertiary:
            SET_BITS(PxSEL0, pinMask);
            SET_BITS(PxSEL1, pinMask);
            break;
        }
       return Pin<PIN_PARAMS>(); // for operator chaining
    }
    static PinFunction currentFunction() {
    	const bool primaryFn   = IS_SET(PxSEL0, pinMask);
    	const bool secondaryFn = IS_SET(PxSEL1, pinMask);
    	if (primaryFn && secondaryFn){
    		return PinFunction::Tertiary;
    	}
    	else if (primaryFn && !secondaryFn){
    		return PinFunction::Secondary;
    	}
    	else if (!primaryFn && secondaryFn){
			return PinFunction::Primary;
		}
    	else {
    		return PinFunction::Gpio;
    	}
    }
    static bool isOutput() {
		return IS_SET(PxDIR, pinMask);
	}
    static bool isInput() {
		return !isOutput();
	}
    static Pin toOutput() {
        SET_BITS(PxDIR, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin toInput() {
        CLEAR_BITS(PxDIR, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin toggleDirection(){
        TOGGLE_BITS(PxDIR, pinMask);
        return Pin<PIN_PARAMS>();
    }

    // Output pin functions
    static Pin setHigh() {
        SET_BITS(PxOUT, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin setLow() {
        CLEAR_BITS(PxOUT, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin setTo(bool value) {
		if (value){
			setHigh();
		} else {
			setLow();
		}
		return Pin<PIN_PARAMS>();
	}
    static Pin toggle() {
        TOGGLE_BITS(PxOUT, pinMask);
        return Pin<PIN_PARAMS>();
    }

    // Input pin functions
    static Pin floating() {
        CLEAR_BITS(PxREN, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin pullup() {
        SET_BITS(PxOUT, pinMask);
        SET_BITS(PxREN, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin pulldown() {
        CLEAR_BITS(PxOUT, pinMask);
        SET_BITS(PxREN, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin setPull(PullDir dir) {
        switch (dir) {
            case PullDir::Floating: floating(); break;
            case PullDir::Pulldown: pulldown(); break;
            case PullDir::Pullup:   pullup();   break;
        }
        return Pin<PIN_PARAMS>();
    }
    static bool isHigh() {
        return IS_SET(PxIN, pinMask);
    }
    static bool isLow() {
        return !isHigh();
    }
    static bool value() {
		return isHigh();
	}
    static Pin enableInterrupt() {
        static_assert(PxIE != nullptr, "Attempted to use GPIO interrupt methods on port without interrupt behaviour");
        SET_BITS(PxIE, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin disableInterrupt() {
        static_assert(PxIE != nullptr, "Attempted to use GPIO interrupt methods on port without interrupt behaviour");
        CLEAR_BITS(PxIE, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin risingEdgeTrigger() {
        static_assert(PxIE != nullptr, "Attempted to use GPIO interrupt methods on port without interrupt behaviour");
        CLEAR_BITS(PxIES, pinMask);
        CLEAR_BITS(PxIFG, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin fallingEdgeTrigger() {
        static_assert(PxIE != nullptr, "Attempted to use GPIO interrupt methods on port without interrupt behaviour");
        SET_BITS(PxIES, pinMask);
        CLEAR_BITS(PxIFG, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static Pin clearInterruptFlag() {
        static_assert(PxIE != nullptr, "Attempted to use GPIO interrupt methods on port without interrupt behaviour");
        CLEAR_BITS(PxIFG, pinMask);
        return Pin<PIN_PARAMS>();
    }
    static inline uint8_t getInterruptVector() {
        static_assert(PxIE != nullptr, "Attempted to use GPIO interrupt methods on port without interrupt behaviour");
        return __even_in_range(*PxIV, P1IV__P1IFG7); // This is a Port 1 macro, but all the ports only go up to 0x10
    }
};

/// Enable pulldown resistors on all pins. Primarily intended to be called prior to configuration of used pins. 
/// Unconfigured input pins are floating inputs, which massively increase power consumption due to noise causing schmitt trigger toggling.
void gpioPulldownAll();

/// Enable pullup resistors on all pins. Primarily intended to be called prior to configuration of used pins. 
/// Unconfigured input pins are floating inputs, which massively increase power consumption due to noise causing schmitt trigger toggling.
void gpioPullupAll();

/// Puts all GPIO pins into output mode (high) on all pins. Primarily intended to be called prior to configuration of used pins. 
/// Unconfigured input pins are floating inputs, which massively increase power consumption due to noise causing schmitt trigger toggling.
void gpioOutputHighAll();

/// Puts all GPIO pins into output mode (low) on all pins. Primarily intended to be called prior to configuration of used pins. 
/// Unconfigured input pins are floating inputs, which massively increase power consumption due to noise causing schmitt trigger toggling.
void gpioOutputLowAll();

/// Disable the GPIO power-on default high-impedance mode. This synchronises the GPIO pins with the values found in the GPIO registers. 
/// Modifications to GPIO registers do not propagate to the physical GPIO pins until this is called.
/// Until this function is called all GPIO pins are high impedance. 
void gpioUnlock();

namespace detail {
    // Used by the isPinType function below.
    // Mark all types as not a 'Pin'...
    template<typename>
    struct IsPin : std::false_type {};

    // ...except if they're a Pin<...> type
    template< 
        volatile uint8_t*  PxDIR, 
        volatile uint8_t*  PxIE,
        volatile uint8_t*  PxIES,
        volatile uint8_t*  PxIFG,
        volatile uint8_t*  PxIN,
        volatile uint16_t* PxIV,
        volatile uint8_t*  PxOUT, 
        volatile uint8_t*  PxREN, 
        volatile uint8_t*  PxSEL0, 
        volatile uint8_t*  PxSEL1, 
        uint8_t pin_num
    >
    struct IsPin<Pin<PIN_PARAMS>> : std::true_type {};
}

namespace Gpio {
    /// Determines if a template parameter T is of type Pin<...> or not.
    template<typename T>
    constexpr bool isPinType() {
        return detail::IsPin<T>::value;
    }
}

#endif /* GPIO_HPP */
