#define HAL_1469X
#include "i2c_instance.h"
extern "C"
{
    #include "default_config.h"
    #include "hw_i2c.h"
}
using namespace std;

class I2c_instance::Impl
{
  public:
    Impl(PinName sda, PinName scl);
    ~Impl();
    void set_frequency(int hz);
    int start();
    int stop();
    int read(int address, char *data, int length, int stop);
    int write(int address, const char *data, int length, int stop);
    void reset();
    int byte_read(int last);
    int byte_write(int data);
};
I2c_instance::Impl::Impl(PinName sda, PinName scl)
{
}
I2c_instance::Impl::~Impl()
{
}
void I2c_instance::Impl::set_frequency(int hz)
{
}
int I2c_instance::Impl::start()
{
}
int I2c_instance::Impl::stop()
{
}
int I2c_instance::Impl::read(int address, char *data, int length, int stop)
{
}
int I2c_instance::Impl::write(int address, const char *data, int length, int stop)
{
}
void I2c_instance::Impl::reset()
{
}
int I2c_instance::Impl::byte_read(int last)
{
}
int I2c_instance::Impl::byte_write(int data)
{
}
I2c_instance::I2c_instance(PinName sda, PinName scl) : m_impl(make_unique<I2c_instance::Impl>(sda, scl))
{
}
I2c_instance::~I2c_instance() = default;
void I2c_instance::set_frequency(int hz)
{
    m_impl->set_frequency(hz);
}
int I2c_instance::start()
{
    return m_impl->start();
}
int I2c_instance::stop()
{
    return m_impl->stop();
}
int I2c_instance::read(int address, char *data, int length, int stop)
{
    return m_impl->read(address, data, length, stop);
}
int I2c_instance::write(int address, const char *data, int length, int stop)
{
    return m_impl->write(address, data, length, stop);
}
void I2c_instance::reset()
{
    m_impl->reset();
}
int I2c_instance::byte_read(int last)
{
    return m_impl->byte_read(last);
}
int I2c_instance::byte_write(int data)
{
    return m_impl->byte_write(data);
}