#include "gpio_api.h"
#include "gsl/gsl"
#include "gpio_power.h"
extern "C"
{
#include "default_config.h"
#include "hw_gpio.h"
#include "hw_sys.h"
}
using namespace gsl;
/**
 * \defgroup hal_gpio GPIO HAL functions
 *
 * # Defined behavior
 * * ::gpio_init and other init functions can be called with NC or a valid PinName for the target - Verified by ::gpio_nc_test
 * * ::gpio_is_connected can be used to test whether a gpio_t object was initialized with NC - Verified by ::gpio_nc_test
 *
 * # Undefined behavior
 * * Calling any ::gpio_mode, ::gpio_dir, ::gpio_write or ::gpio_read on a gpio_t object that was initialized
 *   with NC.
 * * Calling ::gpio_set with NC.
 *
 * @{
 */

/** Checks if gpio object is connected (pin was not initialized with NC)
 * @param obj The GPIO object
 * @return 0 if object was initialized with NC
 * @return non-zero if object was initialized with a valid PinName
 **/
int gpio_is_connected(const gpio_t *obj)
{
    return obj->pin != NC;
}

/** Initialize the GPIO pin
 *
 * @param obj The GPIO object to initialize
 * @param pin The GPIO pin to initialize (may be NC)
 */
void gpio_init(gpio_t *obj, PinName pin)
{
    obj->pin = pin;
    obj->direction = PIN_DIRECTION_UNSET;
}
/** Set the input pin mode
 *
 * @param obj  The GPIO object (must be connected)
 * @param mode The pin mode to be set
 */
void gpio_mode(gpio_t *obj, PinMode mode)
{
    if (obj->direction != PIN_INPUT)
    {
        return;
    }
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    HW_GPIO_MODE hw_mode{HW_GPIO_MODE_INPUT};
    switch (mode)
    {
    case PinMode::PullUp:
        hw_mode = HW_GPIO_MODE_INPUT_PULLUP;
        break;
    case PinMode::PullDown:
        hw_mode = HW_GPIO_MODE_INPUT_PULLDOWN;
        break;
    case PinMode::PullNone:
        hw_mode = HW_GPIO_MODE_INPUT;
        break;
    }
    hw_gpio_set_pin_function(PinName_to_port(obj->pin), PinName_to_pin(obj->pin), hw_mode, HW_GPIO_FUNC_GPIO);
    set_gpio_power(obj->pin);
    hw_gpio_pad_latch_enable(PinName_to_port(obj->pin), PinName_to_pin(obj->pin));
    hw_gpio_pad_latch_disable(PinName_to_port(obj->pin), PinName_to_pin(obj->pin));
}

/** Set the pin direction
 *
 * @param obj       The GPIO object (must be connected)
 * @param direction The pin direction to be set
 */
void gpio_dir(gpio_t *obj, PinDirection direction)
{
    switch (direction)
    {
    case PinDirection::PIN_DIRECTION_UNSET:
        Expects(false);
        return;
    case PinDirection::PIN_INPUT:
    {
        obj->direction = PIN_INPUT;
        return;
    }

    case PinDirection::PIN_OUTPUT:
    {
        // hw_sys_pd_com_enable();
        // auto _ = finally([&]() { hw_sys_pd_com_disable(); });
        obj->direction = PIN_OUTPUT;
        // hw_gpio_set_pin_function(PinName_to_port(obj->pin), PinName_to_pin(obj->pin), HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
        gpio_write(obj, obj->value);
    }
    break;
    }
}

/** Set the output value
 *
 * @param obj   The GPIO object (must be connected)
 * @param value The value to be set
 */
void gpio_write(gpio_t *obj, int value)
{
    obj->value = value;
    if (obj->direction != PIN_OUTPUT)
    {
        return;
    }
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    hw_gpio_set_pin_function(PinName_to_port(obj->pin), PinName_to_pin(obj->pin), HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
    set_gpio_power(obj->pin);
    if (value)
    {
        hw_gpio_set_active(PinName_to_port(obj->pin), PinName_to_pin(obj->pin));
    }
    else
    {
        hw_gpio_set_inactive(PinName_to_port(obj->pin), PinName_to_pin(obj->pin));
    }
    hw_gpio_pad_latch_enable(PinName_to_port(obj->pin), PinName_to_pin(obj->pin));
    hw_gpio_pad_latch_disable(PinName_to_port(obj->pin), PinName_to_pin(obj->pin));
}

/** Read the input value
 *
 * @param obj The GPIO object (must be connected)
 * @return An integer value 1 or 0
 */
int gpio_read(gpio_t *obj)
{
    switch (obj->direction)
    {
    case PinDirection::PIN_DIRECTION_UNSET:
        Expects(false);
        return 0;
    case PinDirection::PIN_INPUT:
    case PinDirection::PIN_OUTPUT:
    {
        hw_sys_pd_com_enable();
        auto _ = finally([&]() { hw_sys_pd_com_disable(); });
        return hw_gpio_get_pin_status(PinName_to_port(obj->pin), PinName_to_pin(obj->pin));
    }
    }
    Expects(false);
    return 0;
}