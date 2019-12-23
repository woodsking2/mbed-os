#define HAL_1469X
#include "i2c_instance.h"
#include "i2c_manager.h"
#include "gsl/gsl"
#include "mbed_debug.h"
extern "C"
{
#include "default_config.h"
#include "hw_i2c.h"
#include "hw_sys.h"
#include "hw_gpio.h"
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
    int m_frequency;
    uint16_t m_address;

    constexpr static auto invalid_address{0xFFFF};
    void initialize_hw();
    void acquire_pin();
    void release_pin();
    HW_GPIO_FUNC get_clock_func();
    HW_GPIO_FUNC get_data_func();
    HW_I2C_ID get_hw_id();
    HW_I2C_SPEED get_hw_frequency();
    static HW_I2C_SPEED frequency_convert(int frequency);
};
void I2c_instance::Impl::initialize_hw()
{
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    acquire_pin();
    i2c_config const config = {
        .clock_cfg =
            {
                .ss_hcnt = 0,
                .ss_lcnt = 0,
                .fs_hcnt = 0,
                .fs_lcnt = 0,
                .hs_hcnt = 0,
                .hs_lcnt = 0,
            },
        .speed = get_hw_frequency(),
        .mode = HW_I2C_MODE_MASTER,
        .addr_mode = HW_I2C_ADDRESSING_7B,
        .address = m_address,
        .event_cb = 0,
    };
    hw_i2c_init(get_hw_id(), &config);
    hw_i2c_reset_abort_source(get_hw_id());
    hw_i2c_reset_int_all(get_hw_id());
}
HW_I2C_SPEED I2c_instance::Impl::frequency_convert(int frequency)
{
    if (frequency >= 3400000)
    {
        return HW_I2C_SPEED_HIGH;
    }
    else if (frequency >= 400000)
    {
        return HW_I2C_SPEED_FAST;
    }
    else
    {
        return HW_I2C_SPEED_STANDARD;
    }
}
HW_I2C_SPEED I2c_instance::Impl::get_hw_frequency()
{
    return frequency_convert(m_frequency);
}
HW_I2C_ID I2c_instance::Impl::get_hw_id()
{
    if (m_type == I2c_manager::Type::i2c_1)
    {
        return HW_I2C1;
    }
    else
    {
        return HW_I2C2;
    }
}
HW_GPIO_FUNC I2c_instance::Impl::get_clock_func()
{
    if (m_type == I2c_manager::Type::i2c_1)
    {
        return HW_GPIO_FUNC_I2C_SCL;
    }
    else
    {
        return HW_GPIO_FUNC_I2C2_SCL;
    }
}
HW_GPIO_FUNC I2c_instance::Impl::get_data_func()
{
    if (m_type == I2c_manager::Type::i2c_1)
    {
        return HW_GPIO_FUNC_I2C_SDA;
    }
    else
    {
        return HW_GPIO_FUNC_I2C2_SDA;
    }
}
I2c_instance::Impl::Impl(PinName sda, PinName scl) : m_type(I2c_manager::get_instance().acquire(sda, scl)), m_sda(sda), m_scl(scl), m_frequency(100000), m_address(invalid_address)
{
    // hw_sys_pd_com_enable();
    // auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    // initialize_hw();
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
    debug("m_sda:%d, m_scl: %d\n", m_sda, m_scl);
    hw_gpio_set_pin_function(PinName_to_port(m_scl), PinName_to_pin(m_scl), HW_GPIO_MODE_OUTPUT, get_clock_func());
    hw_gpio_pad_latch_enable(PinName_to_port(m_scl), PinName_to_pin(m_scl));
    hw_gpio_pad_latch_disable(PinName_to_port(m_scl), PinName_to_pin(m_scl));
    hw_gpio_set_pin_function(PinName_to_port(m_sda), PinName_to_pin(m_sda), HW_GPIO_MODE_OUTPUT, get_data_func());
    hw_gpio_pad_latch_enable(PinName_to_port(m_sda), PinName_to_pin(m_sda));
    hw_gpio_pad_latch_disable(PinName_to_port(m_sda), PinName_to_pin(m_sda));
}
void I2c_instance::Impl::release_pin()
{

    hw_gpio_set_default(m_scl);
    hw_gpio_set_default(m_sda);
}
void I2c_instance::Impl::set_frequency(int hz)
{
    if (frequency_convert(m_frequency) == frequency_convert(hz))
    {
        return;
    }
    m_frequency = hz;
    // hw_sys_pd_com_enable();
    // auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    // hw_i2c_set_speed(get_hw_id(), get_hw_frequency());
}
int I2c_instance::Impl::start()
{
    Expects(false);
    return -1;
}
int I2c_instance::Impl::stop()
{
    Expects(false);
    return -1;
}
int I2c_instance::Impl::read(int address, char *data, int length, int stop)
{
    int addres_7_bit = address >> 1;
    Ensures((addres_7_bit & 0b1111111) == addres_7_bit);
    if (addres_7_bit != m_address)
    {
        // hw_i2c_set_target_address(get_hw_id(), addres_7_bit);
        m_address = addres_7_bit;
    }    
    hw_sys_pd_com_enable();
    initialize_hw();        
    hw_i2c_enable(get_hw_id());
    hw_gpio_pad_latch_enable(PinName_to_port(m_scl), PinName_to_pin(m_scl));
    hw_gpio_pad_latch_enable(PinName_to_port(m_sda), PinName_to_pin(m_sda));
    auto _ = finally([&]() {
        hw_gpio_pad_latch_disable(PinName_to_port(m_scl), PinName_to_pin(m_scl));
        hw_gpio_pad_latch_disable(PinName_to_port(m_sda), PinName_to_pin(m_sda));
        hw_i2c_disable(get_hw_id());
        hw_sys_pd_com_disable();
    });
    HW_I2C_ABORT_SOURCE abort_code{};
    uint32_t flags = HW_I2C_F_NONE;
    if (stop)
    {
        flags = HW_I2C_F_ADD_STOP;
    }
    auto read_result = hw_i2c_read_buffer_sync(get_hw_id(), reinterpret_cast<uint8_t *>(data), length, &abort_code, flags);
    if (HW_I2C_ABORT_NONE != abort_code)
    {
        debug("read i2c fail[%d]: %d\n", read_result, abort_code);
    }
    return read_result;
}
int I2c_instance::Impl::write(int address, const char *data, int length, int stop)
{
    int addres_7_bit = address >> 1;
    Ensures((addres_7_bit & 0b1111111) == addres_7_bit);
    if (addres_7_bit != m_address)
    {
        // hw_i2c_set_target_address(get_hw_id(), addres_7_bit);
        m_address = addres_7_bit;
    }    
    hw_sys_pd_com_enable();
    initialize_hw();        
    hw_i2c_enable(get_hw_id());
    hw_gpio_pad_latch_enable(PinName_to_port(m_scl), PinName_to_pin(m_scl));
    hw_gpio_pad_latch_enable(PinName_to_port(m_sda), PinName_to_pin(m_sda));
    auto _ = finally([&]() {
        hw_gpio_pad_latch_disable(PinName_to_port(m_scl), PinName_to_pin(m_scl));
        hw_gpio_pad_latch_disable(PinName_to_port(m_sda), PinName_to_pin(m_sda));
        hw_i2c_disable(get_hw_id());
        hw_sys_pd_com_disable();
    });
    HW_I2C_ABORT_SOURCE abort_code{};
    uint32_t flags = HW_I2C_F_NONE;
    if (stop)
    {
        flags = HW_I2C_F_ADD_STOP;
    }
    auto read_result = hw_i2c_write_buffer_sync(get_hw_id(), reinterpret_cast<uint8_t const *>(data), length, &abort_code, flags);
    if (HW_I2C_ABORT_NONE != abort_code)
    {
        debug("write i2c fail[%d]: %d\n", read_result, abort_code);
    }
    return read_result;
}
void I2c_instance::Impl::reset()
{
    hw_i2c_reset_abort_source(get_hw_id());
    hw_i2c_reset_int_all(get_hw_id());
}
/**
 * @brief
 *
 *  @param last Acknoledge ,0 mean  ack ,1 mean no ack
 *  @return The read byte
 */
int I2c_instance::Impl::byte_read(int last)
{
    Expects(false);
    return -1;
}
/**
 * @brief
 *
 *  @param data Byte to be written
 *  @return 0 if NAK was received, 1 if ACK was received, 2 for timeout.
 */
int I2c_instance::Impl::byte_write(int data)
{
    Expects(false);
    return -1;
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