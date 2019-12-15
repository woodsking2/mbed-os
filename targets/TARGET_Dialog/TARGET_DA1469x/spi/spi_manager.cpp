#include "spi_manager.h"
#include "gsl/gsl"
#include "PlatformMutex.h"
#include <array>
#include "mbed_debug.h"
using namespace std;
using namespace gsl;
class Spi_manager::Impl
{
  private:
    static constexpr auto spi_count = 2;
    array<bool, spi_count> m_acquired;
    PlatformMutex m_mutex;

  public:
    Impl();
    Type acquire();
    void release(Spi_manager::Type type);
};
Spi_manager::Impl::Impl()
{
}
Spi_manager::Type Spi_manager::Impl::acquire()
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    for (gsl::index i = 0; i < spi_count; i++)
    {
        if (!m_acquired.at(i))
        {
            debug("spi %d acquire\n", i);
            m_acquired.at(i) = true;
            return static_cast<Spi_manager::Type>(i);
        }
    }
    Expects(false);
    return Spi_manager::Type::spi_1;
}
void Spi_manager::Impl::release(Spi_manager::Type type)
{
    m_mutex.lock();
    auto _ = finally([&]() { m_mutex.unlock(); });
    m_acquired.at(static_cast<int>(type)) = false;
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
Spi_manager::Type Spi_manager::acquire()
{
    return m_impl->acquire();
}
void Spi_manager::release(Spi_manager::Type type)
{
    m_impl->release(type);
}