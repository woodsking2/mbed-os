#include "interrupt_manager.h"
#include "mbed.h"
#include "gsl/gsl"
// #include "Main_thread.h"
#include <array>
#include "PlatformMutex.h"
extern "C"
{
#include "default_config.h"
#include "hw_wkup.h"
#include "interrupts.h"
#include "hw_pdc.h"
}
using namespace gsl;
using namespace std;

namespace
{
HW_WKUP_PIN_STATE pin_state_to_hw_event(bool pin_state)
{
    if (pin_state)
    {
        return HW_WKUP_PIN_STATE_HIGH;
    }
    else
    {
        return HW_WKUP_PIN_STATE_LOW;
    }
}
void interrupt_handler()
{
    Interrupt_manager::get_instance().interrupt_from_isr();
}

} // namespace
class Interrupt_manager::Impl
{
  private:
    PlatformMutex m_mutex;
    array<Interrupt_instance *, PinCount> m_instances;
    void enable_interrupt();
    void interrupt();
    void lock();
    void unlock();    

  public:
    Impl();
    void add(PinName pin, Interrupt_instance *);
    void remove(PinName pin);
    void interrupt_from_isr();
    void set_hw_interrupt(PinName pin, bool pin_state);
};
void Interrupt_manager::Impl::set_hw_interrupt(PinName pin, bool pin_state)
{
    lock();
    auto _ = finally([&]() { unlock(); });
    auto hw_event = pin_state_to_hw_event(pin_state);
    hw_wkup_gpio_configure_pin(PinName_to_port(pin), PinName_to_pin(pin), true, hw_event);
}
void Interrupt_manager::Impl::lock()
{
    hw_wkup_unregister_interrupts();
    // m_mutex.lock();
}
void Interrupt_manager::Impl::unlock()
{
    // m_mutex.unlock();
    enable_interrupt();
}
void Interrupt_manager::Impl::interrupt_from_isr()
{
    // hw_wkup_unregister_interrupts();
    // m_high_prio_event->call(callback(this, &Interrupt_manager::Impl::interrupt));
    uint32_t status{};
    PinName pin{};
    Interrupt_instance *instance{};
    for (int port = HW_GPIO_PORT_0; port < HW_GPIO_PORT_MAX; ++port)
    {
        status = hw_wkup_get_status(static_cast<HW_GPIO_PORT>(port));
        for (auto bit = 0; bit < HW_GPIO_PIN_MAX; bit++)
        {
            if (status & (1 << bit))
            {
                pin = port_pin_to_PinName(static_cast<HW_GPIO_PORT>(port), static_cast<HW_GPIO_PIN>(bit));
                instance = m_instances.at(pin);
                if (instance != 0)
                {
                    instance->triggered();
                }
            }
        }
        hw_wkup_clear_status(static_cast<HW_GPIO_PORT>(port), 0xFFFFFFFF);
    }
    enable_interrupt();
}
void Interrupt_manager::Impl::enable_interrupt()
{
    // lock();
    // auto _ = finally([&]() { unlock(); });
    hw_wkup_register_gpio_p0_interrupt(interrupt_handler, PRIORITY_15);
    hw_wkup_register_gpio_p1_interrupt(interrupt_handler, PRIORITY_15);
}
Interrupt_manager::Impl::Impl() //: m_high_prio_event(mbed_highprio_event_queue())
{
    lock();
    auto _ = finally([&]() { unlock(); });
    hw_wkup_init(0);
    enable_interrupt();
}
void Interrupt_manager::Impl::add(PinName pin, Interrupt_instance *instance)
{
    lock();
    auto _ = finally([&]() { unlock(); });
    m_instances.at(static_cast<int>(pin)) = instance;
}
void Interrupt_manager::Impl::remove(PinName pin)
{
    lock();
    auto _ = finally([&]() { unlock(); });
    m_instances.at(static_cast<int>(pin)) = 0;
}
Interrupt_manager::Interrupt_manager() : m_impl(make_unique<Interrupt_manager::Impl>())
{
}
Interrupt_manager::~Interrupt_manager() = default;
Interrupt_manager &Interrupt_manager::get_instance()
{
    static Interrupt_manager instance;
    return instance;
}

void Interrupt_manager::add(PinName pin, Interrupt_instance *instance)
{
    m_impl->add(pin, instance);
}
void Interrupt_manager::remove(PinName pin)
{
    m_impl->remove(pin);
}

void Interrupt_manager::interrupt_from_isr()
{
    m_impl->interrupt_from_isr();
}
void Interrupt_manager::set_hw_interrupt(PinName pin, bool pin_state)
{
    m_impl->set_hw_interrupt(pin, pin_state);
}