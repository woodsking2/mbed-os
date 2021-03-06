#include "spi_manager.h"
#include "gsl/gsl"
#include "PlatformMutex.h"
#include <array>
#include "mbed_debug.h"
using namespace std;
using namespace gsl;
class Spi_manager::Impl
{
  public:
    Impl();
    Type acquire(PinName mosi, PinName miso, PinName mclk);
    void release(Spi_manager::Type type);

  private:
    class Spi_pins
    {
      public:
        PinName m_mosi;
        PinName m_miso;
        PinName m_clk;
        Spi_pins(PinName mosi = NC, PinName miso = NC, PinName mclk = NC) : m_mosi(mosi), m_miso(miso), m_clk(mclk)
        {
        }
        bool operator==(const Spi_pins &a) noexcept
        {      
            return m_mosi == a.m_mosi && m_miso == a.m_miso && m_clk == a.m_clk;
        }
    };
    static constexpr auto spi_count = 2;
    array<Spi_pins, spi_count> m_acquired;
    PlatformMutex m_mutex;
};

Spi_manager::Impl::Impl() = default;
Spi_manager::Type Spi_manager::Impl::acquire(PinName mosi, PinName miso, PinName mclk)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    Spi_pins pins{mosi, miso, mclk};

    auto const find_result = find(m_acquired.begin(), m_acquired.end(), pins);
    if (find_result != m_acquired.end())
    {
        return static_cast<Spi_manager::Type>(find_result - m_acquired.begin());
    }
    Spi_pins invalid_pins{};
    // debug("acqure list\n");
    // for (auto value : m_acquired)
    // {
    //     debug("%d %d %d\n", value.m_mosi, value.m_miso, value.m_clk);
    // }
    auto const empty_iterator = find(m_acquired.begin(), m_acquired.end(), invalid_pins);
    if (empty_iterator == m_acquired.end())
    {
#warning debug
        Ensures(false);
        return Spi_manager::Type::spi_1;
    }
    *empty_iterator = pins;
    // debug("acquire %d\n", empty_iterator - m_acquired.begin());
    return static_cast<Spi_manager::Type>(empty_iterator - m_acquired.begin());
}
void Spi_manager::Impl::release(Spi_manager::Type type)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    Spi_pins invalid_pins{};
    // debug("release %d\n", (int)type);
    // debug("release_pre list\n");
    // for (auto value : m_acquired)
    // {
    //     debug("%d %d %d\n", value.m_mosi, value.m_miso, value.m_clk);
    // }
    m_acquired[static_cast<int>(type)] = invalid_pins;
    // debug("release list\n");
    // for (auto value : m_acquired)
    // {
    //     debug("%d %d %d\n", value.m_mosi, value.m_miso, value.m_clk);
    // }
}
Spi_manager::Spi_manager() : m_impl(make_unique<Spi_manager::Impl>())
{
}
Spi_manager::~Spi_manager() = default;
Spi_manager &Spi_manager::get_instance()
{
    static Spi_manager instance;
    return instance;
}
Spi_manager::Type Spi_manager::acquire(PinName mosi, PinName miso, PinName mclk)
{
    return m_impl->acquire(mosi, miso, mclk);
}
void Spi_manager::release(Spi_manager::Type type)
{
    m_impl->release(type);
}
SPIName Spi_manager::get_spi_name(Type type)
{
    switch (type)
    {
    case Type::spi_1:
        return SPIName::SPI_1;
    case Type::spi_2:
        return SPIName::SPI_1;
    }
    Ensures(false);
    return SPIName::SPI_1;
}