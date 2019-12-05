#pragma once
#include "PinNames.h"
#include "gsl/gsl"
/**
 * @brief 
 * 
 * @param pin 
 */
void set_gpio_power(PinName pin);

/**
 * @brief 
 * 
 * @return gsl::span<PinName const> 
 */
gsl::span<PinName const> get_v18_table();