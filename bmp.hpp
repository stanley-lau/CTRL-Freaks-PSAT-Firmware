#pragma once
#include "pin_mappings.hpp"
#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"
#include <stdint.h>
#include <stdbool.h>

//constants:
// Pressure-Altitude Conversion || https://www.mide.com/air-pressure-at-altitude-calculator
constexpr float P_b = 101325;              // Static pressure at sea level [Pa]
constexpr float T_b = 288;                 // Standard temp at sea level [K]
constexpr float L_b = -0.0065;             // Started temp lapse rate [K/m]
constexpr float h_b = 0;                   // height at the bottom of atmospheric layer [m]
constexpr float R = 8.31432;               // universal gas constant [Nm/molK]
constexpr float g_0 = 9.80665;             // Gravity constant
constexpr float M = 0.0289644;             // Molar Mass of Earth's Air [kg/mol]


void InitBMP();
void ConfigureBMP();
void UpdateBMPStatus ();
void GetPressure();
uint32_t readRawPressure();
float PressureToAltitude(float pressure);
float CalibrateGroundPressure(uint16_t samples);
void SetupBMP();
void DisableBMP();
void AltWindow_Push(float alt);
bool AltWindow_GetDelta(float *delta);


extern volatile bool data_ready_interrupt;
extern volatile bool pressure_data_ready;
