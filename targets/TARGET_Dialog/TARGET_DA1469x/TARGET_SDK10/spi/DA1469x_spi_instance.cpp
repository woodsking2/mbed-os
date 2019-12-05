#include "DA1469x_spi_instance.h"
#include "gsl/gsl"
using namespace gsl;

DA1469x::Spi_instance &DA1469x::Spi_instance::get_instance()
{
    static Spi_instance instance;
    return instance;
}

bool DA1469x::Spi_instance::try_acquire(DA1469x::Spi_instance::Id id)
{
    int id_index = static_cast<int>(id);
    if (m_acquired[id_index])
    {
        return false;
    }
    m_acquired[id_index] = true;
    return true;
}
DA1469x::Spi_instance::Id DA1469x::Spi_instance::acquire(DA1469x::Spi_instance::Type type)
{
    m_mutex.lock();
    auto _ = finally([this] { m_mutex.unlock(); });
    switch (type)
    {
    case DA1469x::Spi_instance::Type::plus:
    {
        auto plus_0 = try_acquire(DA1469x::Spi_instance::Id::plus_0);
        if (plus_0)
        {
            return DA1469x::Spi_instance::Id::plus_0;
        }
        auto plus_1 = try_acquire(DA1469x::Spi_instance::Id::plus_1);
        Expects(plus_1);
        return DA1469x::Spi_instance::Id::plus_1;
    }
    break;
    // case DA1469x::Spi_instance::Type::lcd:
    //     Expects(try_acquire(DA1469x::Spi_instance::Id::lcd));
    //     break;
    }
    Expects(false);
}
void DA1469x::Spi_instance::release(Id id)
{
    m_mutex.lock();
    auto _ = finally([this] { m_mutex.unlock(); });
    int id_index = static_cast<int>(id);
    Expects(m_acquired[id_index]);
    m_acquired[id_index] = false;
}