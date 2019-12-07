#include "DA1469x_spi_plus.h"
#include "newt_sdks.h"
#include "mbed_debug.h"
using namespace gsl;
namespace
{
mcu_gpio_func get_clock_pin_function(DA1469x::Spi_instance::Id id)
{
    switch (id)
    {
    case DA1469x::Spi_instance::Id::plus_0:
        return MCU_GPIO_FUNC_SPI_CLK;
    case DA1469x::Spi_instance::Id::plus_1:
        return MCU_GPIO_FUNC_SPI2_CLK;
    // case DA1469x::Spi_instance::Id::lcd:
    case DA1469x::Spi_instance::Id::count:
        Expects(false);
        break;
    }
    return MCU_GPIO_FUNC_LAST;
}
mcu_gpio_func get_mosi_pin_function(DA1469x::Spi_instance::Id id)
{
    switch (id)
    {
    case DA1469x::Spi_instance::Id::plus_0:
        return MCU_GPIO_FUNC_SPI_DO;
    case DA1469x::Spi_instance::Id::plus_1:
        return MCU_GPIO_FUNC_SPI2_DO;
    // case DA1469x::Spi_instance::Id::lcd:
    case DA1469x::Spi_instance::Id::count:
        Expects(false);
        break;
    }
    return MCU_GPIO_FUNC_LAST;
}
mcu_gpio_func get_miso_pin_function(DA1469x::Spi_instance::Id id)
{
    switch (id)
    {
    case DA1469x::Spi_instance::Id::plus_0:
        return MCU_GPIO_FUNC_SPI_DI;
    case DA1469x::Spi_instance::Id::plus_1:
        return MCU_GPIO_FUNC_SPI2_DI;
    // case DA1469x::Spi_instance::Id::lcd:
    case DA1469x::Spi_instance::Id::count:
        Expects(false);
        break;
    }
    return MCU_GPIO_FUNC_LAST;
}
SPI_Type *get_regs(DA1469x::Spi_instance::Id id)
{
    switch (id)
    {
    case DA1469x::Spi_instance::Id::plus_0:
        return reinterpret_cast<SPI_Type *>(SPI_BASE);
    case DA1469x::Spi_instance::Id::plus_1:
        return reinterpret_cast<SPI_Type *>(SPI2_BASE);
    // case DA1469x::Spi_instance::Id::lcd:
    case DA1469x::Spi_instance::Id::count:
        Expects(false);
        break;
    }
    return 0;
}
void enable_clock(DA1469x::Spi_instance::Id id)
{
    switch (id)
    {
    case DA1469x::Spi_instance::Id::plus_0:
        CRG_COM->RESET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI_CLK_SEL_Msk;
        CRG_COM->SET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI_ENABLE_Msk;
        break;
    case DA1469x::Spi_instance::Id::plus_1:
        CRG_COM->RESET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI2_CLK_SEL_Msk;
        CRG_COM->SET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI2_ENABLE_Msk;
        break;
    // case DA1469x::Spi_instance::Id::lcd:
    case DA1469x::Spi_instance::Id::count:
        Expects(false);
        break;
    }
}

void disable_clock(DA1469x::Spi_instance::Id id)
{
    switch (id)
    {
    case DA1469x::Spi_instance::Id::plus_0:
        CRG_COM->RESET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI_ENABLE_Msk;
        break;
    case DA1469x::Spi_instance::Id::plus_1:
        CRG_COM->RESET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI2_ENABLE_Msk;
        break;
    // case DA1469x::Spi_instance::Id::lcd:
    case DA1469x::Spi_instance::Id::count:
        Expects(false);
        break;
    }
}
} // namespace

void DA1469x::Spi_plus::clear()
{
    m_regs->SPI_CLEAR_INT_REG = 0;
    m_regs->SPI_CTRL_REG = 0;
}
void DA1469x::Spi_plus::set_pin_func()
{
    mcu_gpio_set_pin_function(m_config.m_clock_pin, MCU_GPIO_MODE_OUTPUT, get_clock_pin_function(m_id));
    if (m_config.m_mosi_pin != NC)
    {
        mcu_gpio_set_pin_function(m_config.m_mosi_pin, MCU_GPIO_MODE_OUTPUT, get_mosi_pin_function(m_id));
    }
    if (m_config.m_miso_pin != NC)
    {
        mcu_gpio_set_pin_function(m_config.m_miso_pin, MCU_GPIO_MODE_INPUT, get_miso_pin_function(m_id));
    }
}
void DA1469x::Spi_plus::enable()
{
    m_regs->SPI_CTRL_REG |= SPI_SPI_CTRL_REG_SPI_ON_Msk;
    m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_RST_Msk;
}

void DA1469x::Spi_plus::set_frequency()
{
    m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_CLK_Msk;
    switch (m_config.m_frequency)
    {
    case Spi::Config::Frequency::freq_16m:
        m_regs->SPI_CTRL_REG |= 2U << SPI_SPI_CTRL_REG_SPI_CLK_Pos;
        return;
    case Spi::Config::Frequency::freq_8m:
        m_regs->SPI_CTRL_REG |= 1U << SPI_SPI_CTRL_REG_SPI_CLK_Pos;
        return;
    case Spi::Config::Frequency::freq_4m:
        m_regs->SPI_CTRL_REG |= 0U << SPI_SPI_CTRL_REG_SPI_CLK_Pos;
        return;
    case Spi::Config::Frequency::freq_2_28m:
        m_regs->SPI_CTRL_REG |= 3U << SPI_SPI_CTRL_REG_SPI_CLK_Pos;
        return;
    case Spi::Config::Frequency::unset:
        Expects(false);
        return;
    }
}
void DA1469x::Spi_plus::set_bits()
{
    m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_WORD_Msk;
    switch (m_config.m_bits)
    {
    case Spi::Config::Bits::bits8:
        m_regs->SPI_CTRL_REG |= 0U << SPI_SPI_CTRL_REG_SPI_WORD_Pos;
        return;
    }
}
void DA1469x::Spi_plus::set_mode()
{
    m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_POL_Msk;
    m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_PHA_Msk;
    switch (m_config.m_mode)
    {
    case Spi::Config::Mode::mode_0:
        m_regs->SPI_CTRL_REG |= 0U;
        return;
    case Spi::Config::Mode::mode_1:
        m_regs->SPI_CTRL_REG |= 1U << SPI_SPI_CTRL_REG_SPI_PHA_Pos;
        return;
    case Spi::Config::Mode::mode_2:
        m_regs->SPI_CTRL_REG |= 1U << SPI_SPI_CTRL_REG_SPI_POL_Pos;
        return;
    case Spi::Config::Mode::mode_3:
        m_regs->SPI_CTRL_REG |= (1U << SPI_SPI_CTRL_REG_SPI_PHA_Pos) | (1U << SPI_SPI_CTRL_REG_SPI_POL_Pos);
        return;
    }
}
DA1469x::Spi_plus::Spi_plus(DA1469x::Spi::Config &config) : DA1469x::Impl::Spi(config, Spi_instance::Type::plus), m_cs(config.m_cs_pin, true), m_regs(get_regs(m_id))
{
    set_pin_func();
    clear();
    enable_clock(m_id);
    enable();
    set_frequency();
    set_bits();
    set_mode();
}
DA1469x::Spi_plus::~Spi_plus()
{
    m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_ON_Msk;
    m_regs->SPI_CTRL_REG |= SPI_SPI_CTRL_REG_SPI_RST_Msk;
    disable_clock(m_id);
}

uint8_t DA1469x::Spi_plus::write(uint8_t value)
{
    // debug("write %0X\n", value);
    m_cs.write(false);
    auto _ = finally([this] { m_cs.write(true); });
    while (!(m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_TX_FIFO_EMPTY_Msk))
    {
        __NOP();
    }
    MBED_UNUSED uint32_t dummy{};
    while (!(m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk))
    {
        dummy = m_regs->SPI_RX_TX_REG;
    }
    m_regs->SPI_RX_TX_REG = value;
    while (m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk)
    {
        __NOP();
    }
    uint8_t receive_value = m_regs->SPI_RX_TX_REG & 0xFF;
    return receive_value;
}
int DA1469x::Spi_plus::block_write(gsl::span<uint8_t const> tx, gsl::span<uint8_t> rx, uint8_t write_fill)
{
    m_cs.write(false);
    auto _ = finally([this] { m_cs.write(true); });

    MBED_UNUSED uint32_t unused_value{};
    while (!(m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk))
    {
        unused_value = m_regs->SPI_RX_TX_REG;
    }
    auto total = (tx.size() > rx.size()) ? tx.size() : rx.size();
    uint8_t send_value{};
    int sended_length{};
    int received_length{};
    while (sended_length < total || received_length < total)
    {
        if (sended_length < total && 0 == (m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_TXH_Msk))
        {
            send_value = (sended_length < tx.size()) ? tx[sended_length] : write_fill;
            m_regs->SPI_RX_TX_REG = send_value;
            sended_length++;
        }
        if (received_length < total && 0 == (m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk))
        {
            if (received_length < rx.size())
            {
                rx[received_length] = m_regs->SPI_RX_TX_REG;
            }
            else
            {
                unused_value = m_regs->SPI_RX_TX_REG;
            }
            received_length++;
        }
    }
    return total;
}
