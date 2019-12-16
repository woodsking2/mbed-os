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
    EventQueue *m_high_prio_event;
    void enable_interrupt();
    void interrupt();

  public:
    Impl();
    void add(PinName pin, Interrupt_instance *);
    void remove(PinName pin);
    void interrupt_from_isr();
};
void Interrupt_manager::Impl::interrupt()
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    // debug("Interrupt_manager::Impl::interrupt\n");
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
void Interrupt_manager::Impl::interrupt_from_isr()
{
    hw_wkup_unregister_interrupts();
    m_high_prio_event->call(callback(this, &Interrupt_manager::Impl::interrupt));
}
void Interrupt_manager::Impl::enable_interrupt()
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    hw_wkup_register_gpio_p0_interrupt(interrupt_handler, PRIORITY_15);
    hw_wkup_register_gpio_p1_interrupt(interrupt_handler, PRIORITY_15);
}
Interrupt_manager::Impl::Impl() : m_high_prio_event(mbed_highprio_event_queue())
{
    hw_wkup_init(0);
    enable_interrupt();
}
void Interrupt_manager::Impl::add(PinName pin, Interrupt_instance *instance)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    m_instances.at(static_cast<int>(pin)) = instance;
}
void Interrupt_manager::Impl::remove(PinName pin)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
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