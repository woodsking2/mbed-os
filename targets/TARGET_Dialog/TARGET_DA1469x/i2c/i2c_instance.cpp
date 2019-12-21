#define HAL_1469X
#include "i2c_instance.h"
#include "i2c_manager.h"
#include "gsl/gsl"
extern "C"
{
#include "default_config.h"
#include "hw_i2c.h"
#include "hw_sys.h"
}
using namespace std;
using namespace gsl;

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

  private:
    I2c_manager::Type m_type;
    PinName m_sda;
    PinName m_scl;

    void acquire_pin();
    void release_pin();
};
I2c_instance::Impl::Impl(PinName sda, PinName scl) : m_type(I2c_manager::get_instance().acquire(sda, scl)), m_sda(sda), m_scl(scl)
{
    hw_sys_pd_com_enable();
    acquire_pin();
}
I2c_instance::Impl::~Impl()
{
    release_pin();
    I2c_manager::get_instance().release(m_type);
    hw_sys_pd_com_disable();
}
void I2c_instance::Impl::acquire_pin()
{
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    Expects(m_sda != NC);
    Expects(m_scl != NC);
    // hw_gpio_set_pin_function(PinName_to_port(m_sclk), PinName_to_pin(m_sclk), HW_GPIO_MODE_OUTPUT, get_clock_func());
    // hw_gpio_pad_latch_enable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
    // hw_gpio_set_pin_function(PinName_to_port(m_sclk), PinName_to_pin(m_sclk), HW_GPIO_MODE_OUTPUT, get_clock_func());
    // hw_gpio_pad_latch_enable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
}
void I2c_instance::Impl::release_pin()
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