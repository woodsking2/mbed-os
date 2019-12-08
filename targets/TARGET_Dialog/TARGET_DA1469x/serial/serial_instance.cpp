#include "serial_instance.h"
#include "gsl/gsl"
extern "C"
{
#include "default_config.h"
#include "hw_uart.h"
#include "hw_gpio.h"
#include "hw_sys.h"
}
using namespace gsl;
using namespace std;

class Serial_instance::impl
{
  public:
    static Serial_instance *acquire();
    void release();
    void initialize(PinName tx, PinName rx);
    void set_baudrate(int baudrate);
    void set_format(int data_bits, SerialParity parity, int stop_bits);
    bool write_able();
    void write(int c);
    impl();

  private:
    enum class Uart_id
    {
        uart_1,
        uart_2,
        uart_3,
    };
    static constexpr auto default_baudrate{115200};
    static constexpr auto default_data_bits{8};
    static constexpr auto default_parity{ParityNone};
    static constexpr auto default_stop_bits{1};
    static constexpr auto serial_count = 3;
    static bool acauired[serial_count];
    static PlatformMutex mutex;
    void hw_initialize();
    void hw_reinitialize();
    uart_config const get_hw_config();
    HW_UART_ID get_hw_uart();
    HW_UART_BAUDRATE get_hw_baudrate();
    HW_UART_DATABITS get_hw_data_bits();
    HW_UART_PARITY get_hw_parity();
    HW_UART_STOPBITS get_hw_stop_bits();
    HW_GPIO_FUNC get_hw_tx_func();

    int m_baudrate;
    int m_data_bits;
    SerialParity m_parity;
    int m_stop_bits;
    bool m_is_init;
    Uart_id m_id;
    PinName m_tx;
    PinName m_rx;
};
bool Serial_instance::impl::acauired[serial_count];
PlatformMutex Serial_instance::impl::mutex;
HW_GPIO_FUNC Serial_instance::impl::get_hw_tx_func()
{
    switch (m_id)
    {
    case Uart_id::uart_1:
        return HW_GPIO_FUNC_UART_TX;
    case Uart_id::uart_2:
        return HW_GPIO_FUNC_UART2_TX;
    case Uart_id::uart_3:
        return HW_GPIO_FUNC_UART3_TX;
    }
    Expects(false);
    return HW_GPIO_FUNC_GPIO;
}
HW_UART_DATABITS Serial_instance::impl::get_hw_data_bits()
{
    switch (m_data_bits)
    {
    case 5:
        return HW_UART_DATABITS_5;
    case 6:
        return HW_UART_DATABITS_6;
    case 7:
        return HW_UART_DATABITS_7;
    case 8:
        return HW_UART_DATABITS_8;
    }
    Expects(false);
    return HW_UART_DATABITS_8;
}
HW_UART_PARITY Serial_instance::impl::get_hw_parity()
{
    switch (m_parity)
    {
    case ParityNone:
        return HW_UART_PARITY_NONE;
    case ParityOdd:
        return HW_UART_PARITY_ODD;
    case ParityEven:
        return HW_UART_PARITY_EVEN;
    case ParityForced1:
    case ParityForced0:
        Expects(false);
        return HW_UART_PARITY_NONE;
    }
    Expects(false);
    return HW_UART_PARITY_NONE;
}
HW_UART_STOPBITS Serial_instance::impl::get_hw_stop_bits()
{
    switch (m_stop_bits)
    {
    case 1:
        return HW_UART_STOPBITS_1;
    case 2:
        return HW_UART_STOPBITS_2;
    }
    Expects(false);
    return HW_UART_STOPBITS_1;
}
HW_UART_BAUDRATE Serial_instance::impl::get_hw_baudrate()
{
    switch (m_baudrate)
    {
    case 4800:
        return HW_UART_BAUDRATE_4800;
    case 9600:
        return HW_UART_BAUDRATE_9600;
    case 14400:
        return HW_UART_BAUDRATE_14400;
    case 19200:
        return HW_UART_BAUDRATE_19200;
    case 28800:
        return HW_UART_BAUDRATE_28800;
    case 57600:
        return HW_UART_BAUDRATE_57600;
    case 115200:
        return HW_UART_BAUDRATE_115200;
    case 230400:
        return HW_UART_BAUDRATE_230400;
    case 500000:
        return HW_UART_BAUDRATE_500000;
    case 1000000:
        return HW_UART_BAUDRATE_1000000;
    }
    Expects(false);
    return HW_UART_BAUDRATE_115200;
}
HW_UART_ID Serial_instance::impl::get_hw_uart()
{
    switch (m_id)
    {
    case Uart_id::uart_1:
        return HW_UART1;
    case Uart_id::uart_2:
        return HW_UART2;
    case Uart_id::uart_3:
        return HW_UART3;
    }
    Expects(false);
    return 0;
}
uart_config const Serial_instance::impl::get_hw_config()
{
    uart_config const config = {
        .baud_rate = get_hw_baudrate(),
        .data = get_hw_data_bits(),
        .parity = get_hw_parity(),
        .stop = get_hw_stop_bits(),
        .auto_flow_control = false,
        .use_dma = false,
        .use_fifo = true,
    };
    return config;
}
void Serial_instance::impl::hw_initialize()
{
    auto config = get_hw_config();
    hw_uart_init(get_hw_uart(), &config);
}
void Serial_instance::impl::hw_reinitialize()
{
    auto config = get_hw_config();
    hw_uart_reinit(get_hw_uart(), &config);
}
Serial_instance::impl::impl() : m_baudrate(default_baudrate), m_data_bits(default_data_bits), m_parity(default_parity), m_stop_bits(1)
{
}
Serial_instance *Serial_instance::impl::acquire()
{
    mutex.lock();
    auto _ = finally([&]() { mutex.unlock(); });
    for (gsl::index i = 0; i < serial_count; i++)
    {
        if (!acauired[i])
        {
            Serial_instance *instance = new Serial_instance();
            Ensures(instance);
            instance->m_impl->m_id = static_cast<Uart_id>(i);
            acauired[i] = true;
            return instance;
        }
    }
    Ensures(false);
    return 0;
}
void Serial_instance::impl::release()
{
    mutex.lock();
    auto _ = finally([&]() { mutex.unlock(); });
    if (m_tx != NC)
    {
        hw_gpio_set_default(m_tx);
    }
    if (m_rx != NC)
    {
        hw_gpio_set_default(m_rx);
    }
    acauired[static_cast<int>(m_id)] = false;
}
void Serial_instance::impl::initialize(PinName tx, PinName rx)
{
    m_tx = tx;
    m_rx = rx;
}
void Serial_instance::impl::set_baudrate(int baudrate)
{
    if (baudrate == m_baudrate)
    {
        return;
    }
    m_baudrate = baudrate;
    if (!m_is_init)
    {
        return;
    }
    hw_uart_baudrate_set(get_hw_uart(), get_hw_baudrate());
}
void Serial_instance::impl::set_format(int data_bits, SerialParity parity, int stop_bits)
{
    if (data_bits == m_data_bits && parity == m_parity && stop_bits == m_stop_bits)
    {
        return;
    }
    m_data_bits = data_bits;
    m_parity = parity;
    m_stop_bits = stop_bits;
    if (!m_is_init)
    {
        return;
    }
    hw_reinitialize();
}
bool Serial_instance::impl::write_able()
{
    return hw_uart_transmit_fifo_not_full(get_hw_uart());
}
void Serial_instance::impl::write(int c)
{
    Expects(m_tx != NC);
    hw_sys_pd_com_enable();
    hw_gpio_set_pin_function(PinName_to_port(m_tx), PinName_to_pin(m_tx), HW_GPIO_MODE_OUTPUT, get_hw_tx_func());    
    hw_gpio_pad_latch_enable(PinName_to_port(m_tx), PinName_to_pin(m_tx));
    auto _ = finally([&]() {
        hw_gpio_pad_latch_disable(PinName_to_port(m_tx), PinName_to_pin(m_tx));
        hw_sys_pd_com_disable();
    });
    if (!m_is_init)
    {
        hw_initialize();
    }
    hw_uart_write(get_hw_uart(), c);
    while (hw_uart_is_busy(get_hw_uart()))
    {
        __NOP();
    }
}

Serial_instance::Serial_instance() : m_impl{make_unique<Serial_instance::impl>()}
{
}
Serial_instance::~Serial_instance() = default;

Serial_instance *Serial_instance::acquire()
{
    return Serial_instance::impl::acquire();
}
void Serial_instance::release()
{
    m_impl->release();
}
void Serial_instance::initialize(PinName tx, PinName rx)
{
    m_impl->initialize(tx, rx);
}
void Serial_instance::set_baudrate(int baudrate)
{
    m_impl->set_baudrate(baudrate);
}
void Serial_instance::set_format(int data_bits, SerialParity parity, int stop_bits)
{
    m_impl->set_format(data_bits, parity, stop_bits);
}
bool Serial_instance::write_able()
{
    return m_impl->write_able();
}
void Serial_instance::write(int c)
{
    m_impl->write(c);
}