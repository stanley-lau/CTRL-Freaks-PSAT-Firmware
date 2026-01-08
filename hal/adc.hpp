#ifndef CLOCK_HPP
#define CLOCK_HPP

#include <msp430.h>
#include "pmm.hpp"

/// How much to divide the input clock
enum class AdcPredivider {
    /// Divide the input clock by 1
    _1  = ADCPDIV__1,
    /// Divide the input clock by 4
    _4  = ADCPDIV__4, 
    /// Divide the input clock by 64
    _64 = ADCPDIV__64,
};

/// How much to divide the input clock (after predivision)
enum class AdcClockDivider {
    /// Divide the predivided clock by 1
    _1 = ADCDIV_0,
    /// Divide the predivided clock by 2
    _2 = ADCDIV_1, 
    /// Divide the predivided clock by 3
    _3 = ADCDIV_2,
    /// Divide the predivided clock by 4
    _4 = ADCDIV_3, 
    /// Divide the predivided clock by 5
    _5 = ADCDIV_4,
    /// Divide the predivided clock by 6
    _6 = ADCDIV_5, 
    /// Divide the predivided clock by 7
    _7 = ADCDIV_6,
    /// Divide the predivided clock by 8
    _8 = ADCDIV_7, 
};

/// How many ADC clock cycles to sample and hold the input for
enum class AdcSampleTime {
    /// Sample for 4 ADCCLK cycles
    _4    = ADCSHT_0,
    /// Sample for 8 ADCCLK cycles
    _8    = ADCSHT_1,
    /// Sample for 16 ADCCLK cycles
    _16   = ADCSHT_2,
    /// Sample for 32 ADCCLK cycles
    _32   = ADCSHT_3,
    /// Sample for 64 ADCCLK cycles
    _64   = ADCSHT_4,
    /// Sample for 96 ADCCLK cycles
    _96   = ADCSHT_5,
    /// Sample for 128 ADCCLK cycles
    _128  = ADCSHT_6,
    /// Sample for 192 ADCCLK cycles
    _192  = ADCSHT_7,
    /// Sample for 256 ADCCLK cycles
    _256  = ADCSHT_8,
    /// Sample for 384 ADCCLK cycles
    _384  = ADCSHT_9,
    /// Sample for 512 ADCCLK cycles
    _512  = ADCSHT_10,
    /// Sample for 768 ADCCLK cycles
    _768  = ADCSHT_11,
    /// Sample for 1024 ADCCLK cycles
    _1024 = ADCSHT_12,
};

/// Which clock source to use to clock the ADC
enum class AdcClockSource {
    /// MODCLK (3.8MHz +- 21%)
    ModClk = ADCSSEL_0,
    /// ACLK
    Aclk   = ADCSSEL_1,
    /// SMCLK 
    Smclk  = ADCSSEL_2,
};

/// How many bits an ADC conversion will be
enum class AdcResolution {
    /// 8-bit ADC conversion result. The conversion step takes 10 ADCCLK cycles.
    _8Bit  = ADCRES_0,
    /// 10-bit ADC conversion result. The conversion step takes 12 ADCCLK cycles.
    _10Bit = ADCRES_1,
    /// 10-bit ADC conversion result. The conversion step takes 14 ADCCLK cycles.
    _12Bit = ADCRES_2,
};

/// Whether the ADC acts in a high performance mode, or a more power efficient mode with a lower maximum sample rate
enum class AdcPowerMode {
    /// Maximum of 200 kSps, higher power usage.
    HighSpeed = 0,
    /// Maximum of 50 kSps, lower power usage.
    LowPower  = ADCSR,
};

// The following two enums collectively set ADCSREF, so the macro values don't quite line up how you might initially expect,
// but they should work. AdcPosRef sets the bottom two bits, AdcNegRef sets the top bit.

/// List of positive ADC reference values. This controls what voltage corresponds to a maximum count (which varies based on the ADC resolution mode).
enum class AdcPosRef {
    /// AVCC, i.e. External MSP430 supply voltage.
    Vcc = ADCSREF_0,
    /// Internal reference voltage Vref. Vref must be configured before use.
    Vref = ADCSREF_1,
    /// External reference voltage VEref+. VEref+ GPIO pin must be configured before use.
    VErefPlus = ADCSREF_3,
    /// External reference voltage VEref+, buffered using internal ADC buffer. VEref+ GPIO pin must be configured before use.
    VErefPlusBuf = ADCSREF_2,
};

/// List of negative ADC reference values. This controls what voltage corresponds to a count of 0.
enum class AdcNegRef {
    /// AVSS, i.e. GND
    Vss = ADCSREF_0,
    /// External reference voltage VEref-. VEref- GPIO pin must be configured before use.
    VErefMinus = ADCSREF_4,
};

// Internal implementation details
namespace detail {
    struct Vss {
        static constexpr int8_t adcChannel = 14;
    };
    struct Vcc {
        static constexpr int8_t adcChannel = 15;
    };

    // Boilerplate used by isAdcChannelType() to determine template types
    // Mark all types as not an ADC channel...
    template<typename>
    struct IsAdcChannel : std::false_type {};

    // ...except if they're a Pin<...> type...
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
    struct IsAdcChannel<Pin<PIN_PARAMS>> : std::true_type {};

    // ...or are one of the non-GPIO channels
    struct IsAdcChannel<const Vss>  : std::true_type {};
    struct IsAdcChannel<const Vcc>  : std::true_type {};
    struct IsAdcChannel<Vref> : std::true_type {};
    struct IsAdcChannel<TempSensor> : std::true_type {};
}

namespace AdcChannel {
    /// Struct representing the 14th ADC channel (VSS). Pass this to the ADC if you want to read channel 14.
    constexpr detail::Vss VSS;

    /// Struct representing the 15th ADC channel (VCC). Pass this to the ADC if you want to read channel 15.
    constexpr detail::Vcc VCC;
}

struct Adc {
    private:
    /// Determines if a template parameter T is a valid ADC channel or not.
    template<typename T>
    static constexpr bool isAdcChannelType() {
        return detail::IsAdcChannel<T>::value;
    }
    static void setChannel(uint8_t channel) {
        ADCMCTL0 = (ADCMCTL0 & ~ADCINCH) | (channel & ADCINCH);
    }
    static void start() {
        ADCCTL0 |= ADCENC | ADCSC;
    }

    public:
    /// Initialise the ADC for sampling
    static void init(
        AdcClockSource clockSource, 
        AdcPredivider predivider, 
        AdcClockDivider divider, 
        AdcSampleTime sampleTime, 
        AdcPosRef positiveRef = AdcPosRef::Vcc,
        AdcNegRef negativeRef = AdcNegRef::Vss,
        AdcResolution resolution = AdcResolution::_12Bit, 
        AdcPowerMode powerMode = AdcPowerMode::HighSpeed ) {

        uint16_t div    = static_cast<uint16_t>(divider);
        uint16_t time   = static_cast<uint16_t>(sampleTime);
        uint16_t prediv = static_cast<uint16_t>(predivider);
        uint16_t clkSrc = static_cast<uint16_t>(clockSource);
        uint16_t res    = static_cast<uint16_t>(resolution);
        uint16_t adcsr  = static_cast<uint16_t>(powerMode);
        uint16_t posRef = static_cast<uint16_t>(positiveRef);
        uint16_t negRef = static_cast<uint16_t>(negativeRef);

        // Clear ADCENC prior to configuration
        disable();

        //        Sample time  | Multi-sample off
        ADCCTL0 =     time     | ADCMSC_0;

        //        ADCSC start | Internal sample timer | Noninverted sampler | Clock divider |  Clock source  | Single conversion
        ADCCTL1 =   ADCSHS_0  |        ADCSHP_1       |      ADCISSH_0      |      div      |     clkSrc     | ADCCONSEQ_0;

        //        Clock predivider | ADC Resolution | Unsigned result | Max sampling rate
        ADCCTL2 =      prediv      |       res      |     ADCDF_0     | adcsr;

        // ADC positive and negative reference voltages
        ADCMCTL0 = posRef | negRef;
    }

    /// Begin an ADC conversion and wait for it to finish, returning the result.
    template<typename Pin>
    static uint16_t blockingConversion(Pin& adcPin) {
        startConversion(adcPin);
        while(isBusy());
        return getConversionResult();
    }

    /// Begin an ADC conversion and immediately return without waiting for it to complete.
    template<typename Pin>
    static void startConversion(Pin& adcPin) {
        static_assert(isAdcChannelType<Pin>(), "The type of adcPin must be one of 'Pin<...>', 'Vref', 'TempSensor', 'Vcc', or 'Vss'.");
        static_assert(adcPin.adcChannel >= 0, "Attempted to read from pin not connected to ADC");
        disable();
        setChannel(adcPin.adcChannel);
        enable();
        start();
    }

    /// Returns true while the ADC is still converting. Returns false when the ADC result is ready to read. The inverse of adcResultReady().
    static bool isBusy() {
        return ADCCTL1 & ADCBUSY;
    }

    /// Returns true when the ADC conversion is ready. The inverse of adcIsBusy().
    static bool resultReady() {
        return !isBusy();
    }

    /// Get the result
    static uint16_t getConversionResult() {
        return ADCMEM0;
    }

    /// Convert a count into millivolts, given a reference voltage.
    /// Assumes that the resolution of the ADC has not changed since the count was measured.
    static uint16_t countToMillivolts(uint16_t count, uint16_t refVoltageMillivolts) {
        uint8_t resolutionBits;
        switch (ADCCTL2 & ADCRES) {
            case ADCRES_0: resolutionBits =  8; break;
            case ADCRES_1: resolutionBits = 10; break;
            default:       resolutionBits = 12; break;
        }
        return uint16_t((uint32_t(count) * uint32_t(refVoltageMillivolts)) >> resolutionBits);
    }

    /// Enable the ADC, ready to begin conversions.
    /// This is called automatically when a conversion is begun.
    static void enable() {
        ADCCTL0 |= ADCON;
    }

    /// Disable the ADC to save power.
    static void disable() {
        ADCCTL0 &= ~(ADCON | ADCENC);
    }
};

#endif
