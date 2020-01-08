#pragma once
#include "PinNames.h"
#include "gsl/gsl"
// enum class Gpio_power
// {
//     v33,
//     v18
// };
void set_gpio_power(PinName pin);

gsl::span<PinName const> get_v18_table();