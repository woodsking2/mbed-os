#include "interrupt_manager.h"
#include "mbed.h"
#include "gsl/gsl"
#include "Main_thread.h"
#include <array>
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
    array<Interrupt_instance *, PinCount> m_instances;
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
    debug("Interrupt_manager::Impl::interrupt\n");
}
void Interrupt_manager::Impl::interrupt_from_isr()
{
    hw_wkup_unregister_interrupts();
    Main_thread::get_instance().queue().call(callback(this, &Interrupt_manager::Impl::interrupt));
}
void Interrupt_manager::Impl::enable_interrupt()
{
    hw_wkup_register_gpio_p0_interrupt(interrupt_handler, PRIORITY_15);
    hw_wkup_register_gpio_p1_interrupt(interrupt_handler, PRIORITY_15);
}
Interrupt_manager::Impl::Impl()
{
    hw_wkup_init(0);
}
void Interrupt_manager::Impl::add(PinName pin, Interrupt_instance *instance)
{
    m_instances.at(static_cast<int>(pin)) = instance;
}
void Interrupt_manager::Impl::remove(PinName pin)
{
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