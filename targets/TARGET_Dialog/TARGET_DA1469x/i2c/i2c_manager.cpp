#include "i2c_manager.h"
#include "gsl/gsl"
#include "PlatformMutex.h"
#include <array>
#include "mbed_debug.h"
using namespace std;
using namespace gsl;
class I2c_manager::Impl
{
  public:
    Impl();
    Type acquire(PinName sda, PinName scl);
    void release(I2c_manager::Type type);

  private:
    class I2c_pins
    {
      public:
        PinName m_sda;
        PinName m_scl;
        I2c_pins(PinName sda = NC, PinName scl = NC) : m_sda(sda), m_scl(scl)
        {
        }
        bool operator==(const I2c_pins &a) noexcept
        {
            return m_sda == a.m_sda && m_scl == a.m_scl;
        }
    };
    static constexpr auto i2c_count = 2;
    array<I2c_pins, i2c_count> m_acquired;
    PlatformMutex m_mutex;
};
I2c_manager::Impl::Impl() = default;
I2c_manager::Type I2c_manager::Impl::acquire(PinName sda, PinName scl)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    I2c_pins pins{sda, scl};

    auto const find_result = find(m_acquired.begin(), m_acquired.end(), pins);
    if (find_result != m_acquired.end())
    {
        return static_cast<I2c_manager::Type>(find_result - m_acquired.begin());
    }
    I2c_pins invalid_pins{};
    auto const empty_iterator = find(m_acquired.begin(), m_acquired.end(), invalid_pins);
    if (empty_iterator == m_acquired.end())
    {
        Ensures(false);
        return I2c_manager::Type::i2c_1;
    }
    *empty_iterator = pins;
    return static_cast<I2c_manager::Type>(empty_iterator - m_acquired.begin());
}
void I2c_manager::Impl::release(I2c_manager::Type type)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    I2c_pins invalid_pins{};
    m_acquired[static_cast<int>(type)] = invalid_pins;
}

I2c_manager::I2c_manager() : m_impl(make_unique<I2c_manager::Impl>())
{
}
I2c_manager::~I2c_manager() = default;
I2c_manager &I2c_manager::get_instance()
{
    static I2c_manager instance;
    return instance;
}
I2c_manager::Type I2c_manager::acquire(PinName sda, PinName scl)
{
    return m_impl->acquire(sda, scl);
}
void I2c_manager::release(I2c_manager::Type type)
{
    m_impl->release(type);
}