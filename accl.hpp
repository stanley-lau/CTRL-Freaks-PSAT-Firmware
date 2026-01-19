#pragma once
#include "pin_mappings.hpp"
#include "hal/gpio.hpp"
#include "hal/blocking_spi.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>

void InitACCL();
void ConfigureACCL();
void UpdateACCLStatus();
float ReadACCL();
void DisableACCL();

extern volatile bool ACCLReadyFlag;
