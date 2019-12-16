
extern "C"
{
#include "default_config.h"
#include "hw_spi.h"
#include "hw_sys.h"
#include "hw_gpio.h"
}
#include "spi_api.h"
#include "gsl/gsl"
#include "spi_manager.h"
#include "DigitalOut.h"

using namespace gsl;
using namespace std;
using namespace mbed;
namespace
{
class Spi_instance final
{
  public:
    Spi_instance(PinName mosi, PinName miso, PinName sclk, PinName ssel);
    ~Spi_instance();
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
    DigitalOut m_ssel;
    int m_frequency;
    int m_bits;
    int m_mode;
    int m_slave;

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
int Spi_instance::write(int value)
{
    m_ssel.write(0);
    auto _ = finally([&]() { m_ssel.write(1); });
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
int Spi_instance::write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
    m_ssel.write(0);
    auto _ = finally([&]() { m_ssel.write(1); });
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
HW_SPI_FREQ Spi_instance::frequency_convert(int frequency)
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
HW_SPI_POL Spi_instance::get_hw_polarity()
{
    switch (m_mode)
    {
    case 0:
    case 1:
        return HW_SPI_POL_LOW;
    }
    return HW_SPI_POL_HIGH;
}
HW_SPI_PHA Spi_instance::get_hw_phase()
{
    switch (m_mode)
    {
    case 0:
    case 2:
        return HW_SPI_PHA_MODE_0;
    }
    return HW_SPI_PHA_MODE_1;
}
HW_SPI_MINT Spi_instance::get_hw_interrupt_mode()
{
    return HW_SPI_MINT_DISABLE;
}
HW_SPI_FREQ Spi_instance::get_hw_frequency()
{
    return frequency_convert(m_frequency);
}
HW_SPI_FIFO Spi_instance::get_hw_fifo_mode()
{
    return HW_SPI_FIFO_RX_TX;
}
HW_SPI_MODE Spi_instance::get_hw_master_slave()
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
HW_SPI_WORD Spi_instance::get_hw_word()
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
HW_SPI_ID Spi_instance::get_hw_id()
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
HW_GPIO_FUNC Spi_instance::get_clock_func()
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
HW_GPIO_FUNC Spi_instance::get_mosi_func()
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
HW_GPIO_FUNC Spi_instance::get_miso_func()
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
void Spi_instance::set_format(int bits, int mode, int slave)
{
    Expects(slave == 0);
    if (m_bits == bits && mode == m_mode)
    {
        return;
    }
    m_bits = bits;
    m_mode = mode;
    hw_spi_set_word_size(get_hw_id(), get_hw_word());
    hw_spi_set_clock_phase(get_hw_id(), get_hw_phase());
    hw_spi_set_clock_polarity(get_hw_id(), get_hw_polarity());
}
void Spi_instance::set_frequency(int hz)
{
    if (frequency_convert(m_frequency) == frequency_convert(hz))
    {
        return;
    }
    hw_spi_set_clock_freq(get_hw_id(), get_hw_frequency());
}

void Spi_instance::acquire_pin()
{
    hw_sys_pd_com_enable();
    auto _ = finally([&]() { hw_sys_pd_com_disable(); });

    if (m_sclk != NC)
    {
        hw_gpio_set_pin_function(PinName_to_port(m_sclk), PinName_to_pin(m_sclk), HW_GPIO_MODE_OUTPUT, get_clock_func());
        hw_gpio_pad_latch_enable(PinName_to_port(m_sclk), PinName_to_pin(m_sclk));
    }
    if (m_miso != NC)
    {
        hw_gpio_set_pin_function(PinName_to_port(m_miso), PinName_to_pin(m_miso), HW_GPIO_MODE_INPUT, get_miso_func());
        hw_gpio_pad_latch_enable(PinName_to_port(m_miso), PinName_to_pin(m_miso));
    }
    if (m_mosi != NC)
    {
        hw_gpio_set_pin_function(PinName_to_port(m_mosi), PinName_to_pin(m_mosi), HW_GPIO_MODE_OUTPUT, get_mosi_func());
        hw_gpio_pad_latch_enable(PinName_to_port(m_mosi), PinName_to_pin(m_mosi));
    }
    // if (m_ssel != NC)
    // {
    //     hw_gpio_set_pin_function(PinName_to_port(m_ssel), PinName_to_pin(m_ssel), HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
    //     hw_gpio_pad_latch_enable(PinName_to_port(m_ssel), PinName_to_pin(m_ssel));
    // }
}
void Spi_instance::release_pin()
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
    // if (m_ssel != NC)
    // {
    //     hw_gpio_set_default(m_ssel);
    // }
}
Spi_instance::Spi_instance(PinName mosi, PinName miso, PinName sclk, PinName ssel)
    : m_type(Spi_manager::get_instance().acquire()), m_mosi(mosi), m_miso(miso), m_sclk(sclk), m_ssel(ssel, true), m_frequency(4000000), m_bits(8), m_mode(0), m_slave(0)
{
    hw_sys_pd_com_enable();
    acquire_pin();
    spi_config const config = {
        // .cs_pad =
        //     {
        //         .port = PinName_to_port(m_ssel),
        //         .pin = PinName_to_pin(m_ssel),
        //     },
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
} // namespace
Spi_instance::~Spi_instance()
{
    hw_spi_deinit(get_hw_id());
    release_pin();
    Spi_manager::get_instance().release(m_type);
    hw_sys_pd_com_disable();
}
} // namespace

SPIName spi_get_peripheral_name(PinName mosi, PinName miso, PinName mclk)
{
    if (mclk == P0_21)
    {
        return SPI_1;
    }
    return SPI_2;
}
/** Initialize the SPI peripheral
 *
 * Configures the pins used by SPI, sets a default format and frequency, and enables the peripheral
 * @param[out] obj  The SPI object to initialize
 * @param[in]  mosi The pin to use for MOSI
 * @param[in]  miso The pin to use for MISO
 * @param[in]  sclk The pin to use for SCLK
 * @param[in]  ssel The pin to use for SSEL
 */
void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    auto instance = new Spi_instance(mosi, miso, sclk, ssel);
    Expects(instance);
    obj->instance = instance;
}

/** Release a SPI object
 *
 * TODO: spi_free is currently unimplemented
 * This will require reference counting at the C++ level to be safe
 *
 * Return the pins owned by the SPI object to their reset state
 * Disable the SPI peripheral
 * Disable the SPI clock
 * @param[in] obj The SPI object to deinitialize
 */
void spi_free(spi_t *obj)
{
    auto instance = reinterpret_cast<Spi_instance *>(obj->instance);
    Expects(instance);
    delete instance;
    obj->instance = 0;
}

/** Configure the SPI format
 *
 * Set the number of bits per frame, configure clock polarity and phase, shift order and master/slave mode.
 * The default bit order is MSB.
 * @param[in,out] obj   The SPI object to configure
 * @param[in]     bits  The number of bits per frame
 * @param[in]     mode  The SPI mode (clock polarity, phase, and shift direction)
 * @param[in]     slave Zero for master mode or non-zero for slave mode
 */
void spi_format(spi_t *obj, int bits, int mode, int slave)
{
    auto instance = reinterpret_cast<Spi_instance *>(obj->instance);
    Expects(instance);
    instance->set_format(bits, mode, slave);
}

/** Set the SPI baud rate
 *
 * Actual frequency may differ from the desired frequency due to available dividers and bus clock
 * Configures the SPI peripheral's baud rate
 * @param[in,out] obj The SPI object to configure
 * @param[in]     hz  The baud rate in Hz
 */
void spi_frequency(spi_t *obj, int hz)
{
    auto instance = reinterpret_cast<Spi_instance *>(obj->instance);
    Expects(instance);
    instance->set_frequency(hz);
}

/**@}*/
/**
 * \defgroup SynchSPI Synchronous SPI Hardware Abstraction Layer
 * @{
 */

/** Write a byte out in master mode and receive a value
 *
 * @param[in] obj   The SPI peripheral to use for sending
 * @param[in] value The value to send
 * @return Returns the value received during send
 */
int spi_master_write(spi_t *obj, int value)
{
    auto instance = reinterpret_cast<Spi_instance *>(obj->instance);
    Expects(instance);
    return instance->write(value);
}

/** Write a block out in master mode and receive a value
 *
 *  The total number of bytes sent and received will be the maximum of
 *  tx_length and rx_length. The bytes written will be padded with the
 *  value 0xff.
 *
 * @param[in] obj        The SPI peripheral to use for sending
 * @param[in] tx_buffer  Pointer to the byte-array of data to write to the device
 * @param[in] tx_length  Number of bytes to write, may be zero
 * @param[in] rx_buffer  Pointer to the byte-array of data to read from the device
 * @param[in] rx_length  Number of bytes to read, may be zero
 * @param[in] write_fill Default data transmitted while performing a read
 * @returns
 *      The number of bytes written and read from the device. This is
 *      maximum of tx_length and rx_length.
 */
int spi_master_block_write(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
    auto instance = reinterpret_cast<Spi_instance *>(obj->instance);
    Expects(instance);
    return instance->write(tx_buffer, tx_length, rx_buffer, rx_length, write_fill);
}
#if DEVICE_SPISLAVE
/** Check if a value is available to read
 *
 * @param[in] obj The SPI peripheral to check
 * @return non-zero if a value is available
 */
int spi_slave_receive(spi_t *obj)
{
}

/** Get a received value out of the SPI receive buffer in slave mode
 *
 * Blocks until a value is available
 * @param[in] obj The SPI peripheral to read
 * @return The value received
 */
int spi_slave_read(spi_t *obj)
{
}

/** Write a value to the SPI peripheral in slave mode
 *
 * Blocks until the SPI peripheral can be written to
 * @param[in] obj   The SPI peripheral to write
 * @param[in] value The value to write
 */
void spi_slave_write(spi_t *obj, int value)
{
}
#endif
/**@}*/

#if DEVICE_SPI_ASYNCH
#error 123
/**
 * \defgroup AsynchSPI Asynchronous SPI Hardware Abstraction Layer
 * @{
 */

/** Begin the SPI transfer. Buffer pointers and lengths are specified in tx_buff and rx_buff
 *
 * @param[in] obj       The SPI object that holds the transfer information
 * @param[in] tx        The transmit buffer
 * @param[in] tx_length The number of bytes to transmit
 * @param[in] rx        The receive buffer
 * @param[in] rx_length The number of bytes to receive
 * @param[in] bit_width The bit width of buffer words
 * @param[in] event     The logical OR of events to be registered
 * @param[in] handler   SPI interrupt handler
 * @param[in] hint      A suggestion for how to use DMA with this transfer
 */
void spi_master_transfer(spi_t *obj, const void *tx, size_t tx_length, void *rx, size_t rx_length, uint8_t bit_width, uint32_t handler, uint32_t event, DMAUsage hint)
{
}

/** The asynchronous IRQ handler
 *
 * Reads the received values out of the RX FIFO, writes values into the TX FIFO and checks for transfer termination
 * conditions, such as buffer overflows or transfer complete.
 * @param[in] obj     The SPI object that holds the transfer information
 * @return Event flags if a transfer termination condition was met; otherwise 0.
 */
uint32_t spi_irq_handler_asynch(spi_t *obj)
{
}

/** Attempts to determine if the SPI peripheral is already in use
 *
 * If a temporary DMA channel has been allocated, peripheral is in use.
 * If a permanent DMA channel has been allocated, check if the DMA channel is in use.  If not, proceed as though no DMA
 * channel were allocated.
 * If no DMA channel is allocated, check whether tx and rx buffers have been assigned.  For each assigned buffer, check
 * if the corresponding buffer position is less than the buffer length.  If buffers do not indicate activity, check if
 * there are any bytes in the FIFOs.
 * @param[in] obj The SPI object to check for activity
 * @return Non-zero if the SPI port is active or zero if it is not.
 */
uint8_t spi_active(spi_t *obj)
{
}

/** Abort an SPI transfer
 *
 * @param obj The SPI peripheral to stop
 */
void spi_abort_asynch(spi_t *obj)
{
}
#endif
