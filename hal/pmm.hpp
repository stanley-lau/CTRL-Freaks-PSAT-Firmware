#ifndef PMM_HPP
#define PMM_HPP

#include <msp430.h>
#include <stdint.h>

/// List of possible values that the internal voltage reference can take on.
enum class VrefValue {
    _2V5 = REFVSEL_2,
    _2V  = REFVSEL_1,
    _1V5 = REFVSEL_0,
};

/// Struct representing a properly configured internal volage reference. Can be passed to the ADC to read channel 13.
struct Vref {
    private:
    Vref() = delete;

    public:
    static constexpr int8_t adcChannel = 13;

    /// Enable the internal voltage reference.
    /// Returns a token that proves the voltage reference is active. Can be passed to the ADC for reading.
    static const Vref enable(VrefValue vref) {
        uint8_t refvsel = static_cast<uint16_t>(vref);

        PMMCTL2 = (PMMCTL2 & ~REFVSEL) | refvsel | INTREFEN_1;
        return Vref {};
    }
};

/// Struct representing a properly configured internal temperature sensor. Can be passed to the ADC to read channel 12.
struct TempSensor {
    private:
    TempSensor() = delete;

    public:
    static constexpr int8_t adcChannel = 12;

    /// Enable the internal temperature sensor. Requires the internal voltage reference to be enabled already.
    /// Returns a token that proves the temp sensor is active. Can be passed to the ADC for reading.
    static const TempSensor enable(Vref& Vref) {
        PMMCTL2 |= TSENSOREN_1;
        return TempSensor {};
    }
};

#endif
