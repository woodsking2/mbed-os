#pragma once
#include "Mutex.h"
namespace DA1469x
{
class Spi_instance
{
  public:
    enum class Id
    {
        plus_0,
        plus_1,
        // lcd,
        count,
    };
    enum class Type
    {
        plus,
        // lcd,
    };

  private:
    Spi_instance() = default;
    ~Spi_instance() = default;
    rtos::Mutex m_mutex;
    bool m_acquired[static_cast<int>(Id::count)];
    bool try_acquire(Id);

  public:
    static Spi_instance &get_instance();
    Id acquire(Type type);
    void release(Id id);
};
} // namespace DA1469x