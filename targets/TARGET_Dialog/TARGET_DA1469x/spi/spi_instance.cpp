#define HAL_1469X
#include "spi_instance.h"
#include "gsl/gsl"
#include "spi_manager.h"
#include "DigitalOut.h"
#include "gpio_power.h"
extern "C"
{
#include "default_config.h"
#include "hw_spi.h"
#include "hw_sys.h"
#include "hw_gpio.h"
}
using namespace std;
using namespace gsl;
class Spi_instance::Impl
{
  public:
    Impl(PinName mosi, PinName miso, PinName sclk, PinName ssel);
    ~Impl();
    void set_format(int bits, int mode, int slave);
    void set_frequency(int hz);
    int write(int value);
    int write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill);

  private:
    Spi_manager::Type m_type;
    PinName m_mosi;
    PinName m_miso;
    PinName m_sclk;
    // PinName m_ssel;
    mbed::DigitalOut m_ssel;
    int m_frequency;
    int m_bits;
    int m_mode;
    int m_slave;

    void initialize_hw();
    void acquire_pin();
    void release_pin();
    HW_GPIO_FUNC get_clock_func();
    HW_GPIO_FUNC get_mosi_func();
    HW_GPIO_FUNC get_miso_func();
    HW_SPI_ID get_hw_id();
    HW_SPI_WORD get_hw_word();
    HW_SPI_MODE get_hw_master_slave();
    HW_SPI_POL get_hw_polarity();
    HW_SPI_PHA get_hw_phase();
    HW_SPI_MINT get_hw_interrupt_mode();
    HW_SPI_FREQ get_hw_frequency();
    HW_SPI_FIFO get_hw_fifo_mode();
    static HW_SPI_FREQ frequency_convert(int frequency);
};
void Spi_instance::Impl::initialize_hw()
{
    acquire_pin();
    spi_config const config = {
        .word_mode = get_hw_word(),
        .smn_role = get_hw_master_slave(),
        .polarity_mode = get_hw_polarity(),
        .phase_mode = get_hw_phase(),
        .mint_mode = get_hw_interrupt_mode(),
        .xtal_freq = get_hw_frequency(),
        .fifo_mode = get_hw_fifo_mode(),
        .disabled = false,
        .ignore_cs = true,
        .use_dma = false,
        .rx_dma_channel = HW_DMA_CHANNEL_INVALID,
        .tx_dma_channel = HW_DMA_CHANNEL_INVALID,
    };
    hw_spi_init(get_hw_id(), &config);
}
int Spi_instance::Impl::write(int value)
{
    // hw_sys_pd_com_enable();
    // initialize_hw();
    m_ssel.write(0);
    // hw_gpio_pad_latch_enable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
    // if (m_miso != NC)
    // {
    //     hw_gpio_pad_latch_enable(PinName_to_port(m_miso), PinName_to_pin(m_miso));
    // }
    // if (m_mosi != NC)
    // {
    //     hw_gpio_pad_latch_enable(PinName_to_port(m_mosi), PinName_to_pin(m_mosi));
    // }
    auto _ = finally([&]() {
        m_ssel.write(1);
        // hw_gpio_pad_latch_disable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
        // if (m_miso != NC)
        // {
        //     hw_gpio_pad_latch_disable(PinName_to_port(m_miso), PinName_to_pin(m_miso));
        // }
        // if (m_mosi != NC)
        // {
        //     hw_gpio_pad_latch_disable(PinName_to_port(m_mosi), PinName_to_pin(m_mosi));
        // }
        // hw_sys_pd_com_disable();
    });
    switch (m_bits)
    {
    case 8:
        return hw_spi_writeread(get_hw_id(), value & 0xFF);
    case 9:
        return hw_spi_writeread(get_hw_id(), value & 0x01FF);
    case 16:
        return hw_spi_writeread(get_hw_id(), value & 0xFFFF);
    case 32:
        return hw_spi_writeread32(get_hw_id(), value);
    }
    return 0;
}
int Spi_instance::Impl::write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
    // hw_sys_pd_com_enable();
    // initialize_hw();
    m_ssel.write(0);
    // hw_gpio_pad_latch_enable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
    // if (m_miso != NC)
    // {
    //     hw_gpio_pad_latch_enable(PinName_to_port(m_miso), PinName_to_pin(m_miso));
    // }
    // if (m_mosi != NC)
    // {
    //     hw_gpio_pad_latch_enable(PinName_to_port(m_mosi), PinName_to_pin(m_mosi));
    // }
    auto _ = finally([&]() {
        m_ssel.write(1);
        // hw_gpio_pad_latch_disable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
        // if (m_miso != NC)
        // {
        //     hw_gpio_pad_latch_disable(PinName_to_port(m_miso), PinName_to_pin(m_miso));
        // }
        // if (m_mosi != NC)
        // {
        //     hw_gpio_pad_latch_disable(PinName_to_port(m_mosi), PinName_to_pin(m_mosi));
        // }
        // hw_sys_pd_com_disable();
    });
    auto total = (tx_length > rx_length) ? tx_length : rx_length;
    span<uint8_t const> tx{};
    span<uint8_t> rx{};
    unique_ptr<uint8_t[]> tx_cache;
    unique_ptr<uint8_t[]> rx_cache;
    if (tx_length == total)
    {
        tx = make_span<uint8_t const>(reinterpret_cast<uint8_t const *>(tx_buffer), total);
    }
    else
    {
        tx_cache = make_unique<uint8_t[]>(total);
        memset(tx_cache.get(), write_fill, total);
        memcpy(tx_cache.get(), tx_buffer, tx_length);
        tx = make_span<uint8_t const>(tx_cache.get(), total);
    }
    if (rx_buffer != 0 && rx_length != 0)
    {
        if (rx_length == total)
        {
            rx = make_span<uint8_t>(reinterpret_cast<uint8_t *>(rx_buffer), total);
        }
        else
        {
            rx_cache = make_unique<uint8_t[]>(total);
            memset(rx_cache.get(), 0xFF, total);
            rx = make_span<uint8_t>(rx_cache.get(), total);
        }
    }
    hw_spi_writeread_buf(get_hw_id(), tx.data(), rx.data(), total, 0, 0);
    if (rx_buffer != 0 && rx_length != 0 && rx_length != total)
    {
        memcpy(rx_buffer, rx_cache.get(), rx_length);
    }
    return total;
}

HW_SPI_FREQ Spi_instance::Impl::frequency_convert(int frequency)
{
    if (frequency >= 16000000)
    {
        return HW_SPI_FREQ_DIV_2;
    }
    else if (frequency >= 8000000)
    {
        return HW_SPI_FREQ_DIV_4;
    }
    else if (frequency >= 4000000)
    {
        return HW_SPI_FREQ_DIV_8;
    }
    else
    {
        return HW_SPI_FREQ_DIV_14;
    }
}
HW_SPI_POL Spi_instance::Impl::get_hw_polarity()
{
    switch (m_mode)
    {
    case 0:
    case 1:
        return HW_SPI_POL_LOW;
    }
    return HW_SPI_POL_HIGH;
}
HW_SPI_PHA Spi_instance::Impl::get_hw_phase()
{
    switch (m_mode)
    {
    case 0:
    case 2:
        return HW_SPI_PHA_MODE_0;
    }
    return HW_SPI_PHA_MODE_1;
}
HW_SPI_MINT Spi_instance::Impl::get_hw_interrupt_mode()
{
    return HW_SPI_MINT_DISABLE;
}
HW_SPI_FREQ Spi_instance::Impl::get_hw_frequency()
{
    return frequency_convert(m_frequency);
}
HW_SPI_FIFO Spi_instance::Impl::get_hw_fifo_mode()
{
    return HW_SPI_FIFO_RX_TX;
}
HW_SPI_MODE Spi_instance::Impl::get_hw_master_slave()
{
    if (m_slave)
    {
        return HW_SPI_MODE_SLAVE;
    }
    else
    {
        return HW_SPI_MODE_MASTER;
    }
}
HW_SPI_WORD Spi_instance::Impl::get_hw_word()
{
    switch (m_bits)
    {
    case 8:
        return HW_SPI_WORD_8BIT;
    case 9:
        return HW_SPI_WORD_9BIT;
    case 16:
        return HW_SPI_WORD_16BIT;
    case 32:
        return HW_SPI_WORD_32BIT;
    }
    Expects(false);
    return HW_SPI_WORD_8BIT;
}
HW_SPI_ID Spi_instance::Impl::get_hw_id()
{
    if (m_type == Spi_manager::Type::spi_1)
    {
        return HW_SPI1;
    }
    else
    {
        return HW_SPI2;
    }
}
HW_GPIO_FUNC Spi_instance::Impl::get_clock_func()
{
    if (m_type == Spi_manager::Type::spi_1)
    {
        return HW_GPIO_FUNC_SPI_CLK;
    }
    else
    {
        return HW_GPIO_FUNC_SPI2_CLK;
    }
}
HW_GPIO_FUNC Spi_instance::Impl::get_mosi_func()
{
    if (m_type == Spi_manager::Type::spi_1)
    {
        return HW_GPIO_FUNC_SPI_DO;
    }
    else
    {
        return HW_GPIO_FUNC_SPI2_DO;
    }
}
HW_GPIO_FUNC Spi_instance::Impl::get_miso_func()
{
    if (m_type == Spi_manager::Type::spi_1)
    {
        return HW_GPIO_FUNC_SPI_DI;
    }
    else
    {
        return HW_GPIO_FUNC_SPI2_DI;
    }
}
void Spi_instance::Impl::set_format(int bits, int mode, int slave)
{
    Expects(slave == 0);
    if (m_bits == bits && mode == m_mode)
    {
        return;
    }
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    m_bits = bits;
    m_mode = mode;
    hw_spi_set_word_size(get_hw_id(), get_hw_word());
    hw_spi_set_clock_phase(get_hw_id(), get_hw_phase());
    hw_spi_set_clock_polarity(get_hw_id(), get_hw_polarity());
}
void Spi_instance::Impl::set_frequency(int hz)
{
    if (frequency_convert(m_frequency) == frequency_convert(hz))
    {
        return;
    }
    m_frequency = hz;
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    hw_spi_set_clock_freq(get_hw_id(), get_hw_frequency());
}

void Spi_instance::Impl::acquire_pin()
{
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    Expects(m_sclk != NC);
    hw_gpio_set_pin_function(PinName_to_port(m_sclk), PinName_to_pin(m_sclk), HW_GPIO_MODE_OUTPUT, get_clock_func());
    set_gpio_power(m_sclk);
    hw_gpio_pad_latch_enable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
    // hw_gpio_pad_latch_disable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
    if (m_miso != NC)
    {
        hw_gpio_set_pin_function(PinName_to_port(m_miso), PinName_to_pin(m_miso), HW_GPIO_MODE_INPUT, get_miso_func());
        set_gpio_power(m_miso);
        hw_gpio_pad_latch_enable(PinName_to_port(m_miso), PinName_to_pin(m_miso));
        // hw_gpio_pad_latch_disable(PinName_to_port(m_miso), PinName_to_pin(m_miso));
    }
    if (m_mosi != NC)
    {
        hw_gpio_set_pin_function(PinName_to_port(m_mosi), PinName_to_pin(m_mosi), HW_GPIO_MODE_OUTPUT, get_mosi_func());
        set_gpio_power(m_mosi);
        hw_gpio_pad_latch_enable(PinName_to_port(m_mosi), PinName_to_pin(m_mosi));
        // hw_gpio_pad_latch_disable(PinName_to_port(m_mosi), PinName_to_pin(m_mosi));
    }
}
void Spi_instance::Impl::release_pin()
{
    if (m_sclk != NC)
    {
        hw_gpio_set_default(m_sclk);
    }
    if (m_miso != NC)
    {
        hw_gpio_set_default(m_miso);
    }
    if (m_mosi != NC)
    {
        hw_gpio_set_default(m_mosi);
    }
}
Spi_instance::Impl::Impl(PinName mosi, PinName miso, PinName sclk, PinName ssel)
    : m_type(Spi_manager::get_instance().acquire(mosi, miso, sclk)), m_mosi(mosi), m_miso(miso), m_sclk(sclk), m_ssel(ssel, true), m_frequency(4000000), m_bits(8), m_mode(0), m_slave(0)
{
    // debug("Spi_instance::Impl::Impl\n");
    hw_sys_pd_com_enable();
    initialize_hw();
    // auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    // acquire_pin();
    // spi_config const config = {
    //     .word_mode = get_hw_word(),
    //     .smn_role = get_hw_master_slave(),
    //     .polarity_mode = get_hw_polarity(),
    //     .phase_mode = get_hw_phase(),
    //     .mint_mode = get_hw_interrupt_mode(),
    //     .xtal_freq = get_hw_frequency(),
    //     .fifo_mode = get_hw_fifo_mode(),
    //     .disabled = false,
    //     .ignore_cs = true,
    //     .use_dma = false,
    //     .rx_dma_channel = HW_DMA_CHANNEL_INVALID,
    //     .tx_dma_channel = HW_DMA_CHANNEL_INVALID,
    // };
    // hw_spi_init(get_hw_id(), &config);
}
Spi_instance::Impl::~Impl()
{
    // hw_sys_pd_com_enable();
    // auto _ = finally([&]() { hw_sys_pd_com_disable(); });
    hw_spi_deinit(get_hw_id());
    release_pin();
    Spi_manager::get_instance().release(m_type);
    hw_sys_pd_com_disable();
}

Spi_instance::Spi_instance(PinName mosi, PinName miso, PinName sclk, PinName ssel) : m_impl(make_unique<Spi_instance::Impl>(mosi, miso, sclk, ssel))
{
}
Spi_instance::~Spi_instance() = default;
void Spi_instance::set_format(int bits, int mode, int slave)
{
    m_impl->set_format(bits, mode, slave);
}
void Spi_instance::set_frequency(int hz)
{
    m_impl->set_frequency(hz);
}
int Spi_instance::write(int value)
{
    return m_impl->write(value);
}
int Spi_instance::write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
    return m_impl->write(tx_buffer, tx_length, rx_buffer, rx_length, write_fill);
}
