#include "gpio_irq_api.h"
#include "gsl/gsl"
#include "Main_thread.h"
#include "interrupt_manager.h"
#include "interrupt_instance.h"

using namespace gsl;

/** Initialize the GPIO IRQ pin
 *
 * @param obj     The GPIO object to initialize
 * @param pin     The GPIO pin name
 * @param handler The handler to be attached to GPIO IRQ
 * @param id      The object ID (id != 0, 0 is reserved)
 * @return -1 if pin is NC, 0 otherwise
 */
int gpio_irq_init(gpio_irq_t *obj, PinName pin, gpio_irq_handler handler, uint32_t id)
{
    auto instance = new Interrupt_instance(pin, handler, id);
    Expects(instance);
    obj->instance = instance;
    Interrupt_manager::get_instance().add(pin, instance);
    return 0;
}

/** Release the GPIO IRQ PIN
 *
 * @param obj The gpio object
 */
void gpio_irq_free(gpio_irq_t *obj)
{
    Expects(obj->instance);
    Interrupt_instance *instance = reinterpret_cast<Interrupt_instance *>(obj->instance);
    delete instance;
    obj->instance = 0;
}

/** Enable/disable pin IRQ event
 *
 * @param obj    The GPIO object
 * @param event  The GPIO IRQ event
 * @param enable The enable flag
 */
void gpio_irq_set(gpio_irq_t *obj, gpio_irq_event event, uint32_t enable)
{
    Expects(obj->instance);
    Interrupt_instance *instance = reinterpret_cast<Interrupt_instance *>(obj->instance);
    instance->set_event(event);
    instance->set_enable(enable);
}

/** Enable GPIO IRQ
 *
 * This is target dependent, as it might enable the entire port or just a pin
 * @param obj The GPIO object
 */
void gpio_irq_enable(gpio_irq_t *obj)
{
    Expects(obj->instance);
    Interrupt_instance *instance = reinterpret_cast<Interrupt_instance *>(obj->instance);
    instance->set_enable(true);    
}

/** Disable GPIO IRQ
 *
 * This is target dependent, as it might disable the entire port or just a pin
 * @param obj The GPIO object
 */
void gpio_irq_disable(gpio_irq_t *obj)
{
    Expects(obj->instance);
    Interrupt_instance *instance = reinterpret_cast<Interrupt_instance *>(obj->instance);
    instance->set_enable(false);
}
