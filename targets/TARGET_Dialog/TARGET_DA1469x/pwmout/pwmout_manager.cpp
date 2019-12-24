#include "pwmout_manager.h"
#include <array>
#include "gsl/gsl"
#include "mbed.h"
using namespace std;
using namespace gsl;
class Pwmout_manager::Impl
{
  public:
    Impl();
    Type acquire(PinName pin);
    void release(Type type);

  private:
    static constexpr auto timer_count = 3;
    array<PinName, timer_count> m_acquired;
    PlatformMutex m_mutex;
};
Pwmout_manager::Impl::Impl()
{
    for (auto &value : m_acquired)
    {
        value = NC;
    }
}
Pwmout_manager::Type Pwmout_manager::Impl::acquire(PinName pin)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    auto const find_result = find(m_acquired.begin(), m_acquired.end(), pin);
    if (find_result != m_acquired.end())
    {
        return static_cast<Pwmout_manager::Type>(find_result - m_acquired.begin());
    }
    PinName invalid_pin{NC};
    auto const empty_iterator = find(m_acquired.begin(), m_acquired.end(), invalid_pin);
    if (empty_iterator == m_acquired.end())
    {
        Ensures(false);
        return Pwmout_manager::Type::timer_1;
    }
    *empty_iterator = pin;
    return static_cast<Pwmout_manager::Type>(empty_iterator - m_acquired.begin());
}
void Pwmout_manager::Impl::release(Pwmout_manager::Type type)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    PinName invalid_pin{NC};
    m_acquired[static_cast<int>(type)] = invalid_pin;
}
Pwmout_manager::Pwmout_manager() : m_impl(make_unique<Pwmout_manager::Impl>())
{
}
Pwmout_manager::~Pwmout_manager() = default;
Pwmout_manager &Pwmout_manager::get_instance()
{
    static Pwmout_manager instance;
    return instance;
}
Pwmout_manager::Type Pwmout_manager::acquire(PinName pin)
{
    return m_impl->acquire(pin);
}
void Pwmout_manager::release(Pwmout_manager::Type type)
{
    m_impl->release(type);
}