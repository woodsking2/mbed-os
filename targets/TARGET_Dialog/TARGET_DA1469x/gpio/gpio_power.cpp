#include "gpio_power.h"
#include <algorithm>
extern "C"
{
#include "default_config.h"
#include "hw_gpio.h"
#include "hw_sys.h"
}
using namespace std;
void set_gpio_power(PinName pin)
{
    auto v18_table = get_v18_table();
    auto iterator = find(v18_table.begin(), v18_table.end(), pin);
    if (iterator == v18_table.end())
    {
        hw_gpio_configure_pin_power(PinName_to_port(pin), PinName_to_pin(pin), HW_GPIO_POWER_V33);
        return;
    }
    hw_gpio_configure_pin_power(PinName_to_port(pin), PinName_to_pin(pin), HW_GPIO_POWER_VDD1V8P);
}
// Gpio_power get_gpio_power(PinName pin)
// {

// }
