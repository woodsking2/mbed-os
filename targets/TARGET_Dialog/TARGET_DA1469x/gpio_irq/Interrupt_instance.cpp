#include "Interrupt_instance.h"
#include "gsl/gsl"
#include "interrupt_manager.h"
#include "mbed_debug.h"
extern "C"
{
#include "default_config.h"
#include "hw_wkup.h"
#include "interrupts.h"
#include "hw_pdc.h"
}
using namespace std;
using namespace gsl;
namespace
{

HW_WKUP_PIN_STATE gpio_irq_event_to_hw_event(gpio_irq_event event)
{
    switch (event)
    {
    case IRQ_NONE:
        Expects(false);
        break;
    case IRQ_RISE:
        return HW_WKUP_PIN_STATE_HIGH;
    case IRQ_FALL:
        return HW_WKUP_PIN_STATE_LOW;
    }
    Expects(false);
    return HW_WKUP_PIN_STATE_HIGH;
}
} // namespace

class Interrupt_instance::Impl
{
  private:
    PinName m_pin;
    gpio_irq_handler m_handler;
    uint32_t m_id;
    gpio_irq_event m_event;
    uint32_t m_pdc_index;
    bool m_enable;

    void set_hw_interrupt(gpio_irq_event event);

  public:
    Impl(PinName pin, gpio_irq_handler handler, uint32_t id);
    ~Impl();
    void set_event(gpio_irq_event event);
    void set_enable(bool enable);
    void triggered();
};
void Interrupt_instance::Impl::triggered()
{
    debug("%d triggered\n", static_cast<int>(m_pin));
}
void Interrupt_instance::Impl::set_hw_interrupt(gpio_irq_event event)
{
    auto hw_event = gpio_irq_event_to_hw_event(event);
    hw_wkup_gpio_configure_pin(PinName_to_port(m_pin), PinName_to_pin(m_pin), true, hw_event);
}
void Interrupt_instance::Impl::set_event(gpio_irq_event event)
{
    m_event = event;
}
void Interrupt_instance::Impl::set_enable(bool enable)
{
    m_enable = enable;
    if (m_enable)
    {
        set_hw_interrupt(m_event);
    }
}
Interrupt_instance::Impl::Impl(PinName pin, gpio_irq_handler handler, uint32_t id) : m_pin(pin), m_handler(handler), m_id(id)
{
    auto idx = hw_pdc_add_entry(HW_PDC_LUT_ENTRY_VAL(HW_PDC_TRIG_SELECT(pin), PinName_to_pin(pin), HW_PDC_MASTER_CM33, HW_PDC_LUT_ENTRY_EN_XTAL));
    Expects(HW_PDC_INVALID_LUT_INDEX);
    hw_pdc_set_pending(idx);
    hw_pdc_acknowledge(idx);
    m_pdc_index = idx;
}
Interrupt_instance::Impl::~Impl()
{
    hw_pdc_remove_entry(m_pdc_index);
    Interrupt_manager::get_instance().remove(m_pin);
}

Interrupt_instance::Interrupt_instance(PinName pin, gpio_irq_handler handler, uint32_t id) : m_impl(make_unique<Interrupt_instance::Impl>(pin, handler, id))
{
}
Interrupt_instance::~Interrupt_instance() = default;
void Interrupt_instance::set_event(gpio_irq_event event)
{
    m_impl->set_event(event);
}
void Interrupt_instance::set_enable(bool enable)
{
    m_impl->set_enable(enable);
}
void Interrupt_instance::triggered()
{
    m_impl->triggered();
}