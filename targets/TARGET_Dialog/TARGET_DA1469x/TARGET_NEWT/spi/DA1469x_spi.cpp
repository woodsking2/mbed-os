#include "DA1469x_spi.h"
#include "spi_api.h"
#include "newt_sdks.h"
#include "DA1469xAB.h"
#include "gsl/gsl"
#include "DA1469x_gpio.h"
#include "mbed_debug.h"
#include "DA1469x_spi_plus.h"
// #include "DA1469x_lcd_spi.h"
using namespace gsl;
using namespace std;

// namespace
// {
// enum class Spi_index
// {
//     spi_0,
//     spi_1,
// };
// enum class Spi_bit
// {
//     bit_8,
//     bit_32,
// };
// class Spi_instance
// {
//   public:
//     SPI_Type *m_regs;
//     Spi_index m_index;
//     mcu_gpio_func m_clock_pin_func;
//     mcu_gpio_func m_mosi_pin_func;
//     mcu_gpio_func m_miso_pin_func;
//     mcu_gpio_func m_chip_select_pin_func;
//     IRQn_Type m_irq_num;
//     void enable() const
//     {
//         m_regs->SPI_CTRL_REG |= SPI_SPI_CTRL_REG_SPI_ON_Msk;
//         m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_RST_Msk;
//     }
//     void disable() const
//     {
//         m_regs->SPI_CTRL_REG &= ~SPI_SPI_CTRL_REG_SPI_ON_Msk;
//         m_regs->SPI_CTRL_REG |= SPI_SPI_CTRL_REG_SPI_RST_Msk;
//     }
//     Spi_bit get_bit() const
//     {
//         auto mode_bit = (m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_WORD_Msk) >> SPI_SPI_CTRL_REG_SPI_WORD_Pos;
//         switch (mode_bit)
//         {
//         case 0: // 8
//             return Spi_bit::bit_8;
//             // case 1: // 16
//         case 2: // 32
//             return Spi_bit::bit_32;
//             // case 3: // 9
//         }
//         // debug("get_bit %p\n", reinterpret_cast<void *>(m_regs->SPI_CTRL_REG));
//         Expects(false);
//     }
//     template <class Type> void write_block(void const *tx_buffer, gsl::index tx_buffer_size, void *rx_buffer, gsl::index rx_buffer_size, Type write_fill) const
//     {
//         Ensures(tx_buffer_size % sizeof(Type) == 0);
//         Ensures(rx_buffer_size % sizeof(Type) == 0);
//         auto tx_length = tx_buffer_size / sizeof(Type);
//         auto rx_length = rx_buffer_size / sizeof(Type);
//         auto total = (tx_length > rx_length) ? tx_length : rx_length;
//         span<Type const> tx_span{reinterpret_cast<Type const *>(tx_buffer), tx_length};
//         span<Type> rx_span{reinterpret_cast<Type *>(rx_buffer), rx_length};
//         Type send_value{};
//         int sended_length{};
//         int received_length{};
//         int unused_value{};
//         while (sended_length < total || received_length < total)
//         {
//             if (sended_length < total && 0 == (m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_TXH_Msk))
//             {
//                 send_value = (sended_length < tx_length) ? tx_span[sended_length] : write_fill;
//                 m_regs->SPI_RX_TX_REG = send_value;
//                 sended_length++;
//             }
//             if (received_length < total && 0 == (m_regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk))
//             {
//                 if (received_length < rx_length)
//                 {
//                     rx_span[received_length] = m_regs->SPI_RX_TX_REG;
//                 }
//                 else
//                 {
//                     unused_value = m_regs->SPI_RX_TX_REG;
//                 }
//                 received_length++;
//             }
//         }
//     }
//     void enable_clock() const
//     {
//         switch (m_index)
//         {
//         case Spi_index::spi_0:
//             CRG_COM->RESET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI_CLK_SEL_Msk;
//             CRG_COM->SET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI_ENABLE_Msk;
//             return;
//         case Spi_index::spi_1:
//             CRG_COM->RESET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI2_CLK_SEL_Msk;
//             CRG_COM->SET_CLK_COM_REG = CRG_COM_RESET_CLK_COM_REG_SPI2_ENABLE_Msk;
//             return;
//         }
//         Expects(false);
//     }
// };
// Spi_instance const instance[] = {
//     {
//         (SPI_Type *)SPI_BASE,
//         Spi_index::spi_0,
//         MCU_GPIO_FUNC_SPI_CLK,
//         MCU_GPIO_FUNC_SPI_DO,
//         MCU_GPIO_FUNC_SPI_DI,
//         MCU_GPIO_FUNC_SPI_EN,
//         SPI_IRQn,
//     },
//     {
//         (SPI_Type *)SPI2_BASE,
//         Spi_index::spi_1,
//         MCU_GPIO_FUNC_SPI2_CLK,
//         MCU_GPIO_FUNC_SPI2_DO,
//         MCU_GPIO_FUNC_SPI2_DI,
//         MCU_GPIO_FUNC_SPI2_EN,
//         SPI2_IRQn,
//     },
// };
// bool instance_acquired[MBED_CONF_DRIVERS_SPI_COUNT_MAX]{};
// Spi_instance const &get_released()
// {
//     for (gsl::index i = 0; i < MBED_CONF_DRIVERS_SPI_COUNT_MAX; i++)
//     {
//         if (!instance_acquired[i])
//         {
//             return instance[i];
//         }
//     }
//     Ensures(false);
// }
// void acquire(spi_t *obj, Spi_instance const &instance)
// {
//     obj->instance = &instance;
//     instance_acquired[static_cast<int>(instance.m_index)] = true;
// }
// void release(spi_t *obj, Spi_instance const &instance)
// {
//     obj->instance = 0;
//     instance_acquired[static_cast<int>(instance.m_index)] = false;
// }
// } // namespace
// #include "DigitalOut.h"
// using namespace mbed;
// static DigitalOut *cs;
// void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName clock, PinName chip_select)
// {
//     auto &spi_instance = get_released();
//     acquire(obj, spi_instance);
//     mcu_gpio_set_pin_function(clock, MCU_GPIO_MODE_OUTPUT, spi_instance.m_clock_pin_func);
//     mcu_gpio_set_pin_function(mosi, MCU_GPIO_MODE_OUTPUT, spi_instance.m_mosi_pin_func);
//     if (miso != NC)
//     {
//         mcu_gpio_set_pin_function(miso, MCU_GPIO_MODE_OUTPUT, spi_instance.m_miso_pin_func);
//     }
//     Expects(cs == 0);
//     cs = new DigitalOut(chip_select);
//     cs->write(1);
//     spi_instance.m_regs->SPI_CLEAR_INT_REG = 0;
//     spi_instance.m_regs->SPI_CTRL_REG = 0;

//     spi_instance.enable_clock();
//     spi_instance.enable();
// }
// void spi_free(spi_t *obj)
// {
//     //     while (regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_BUSY_Msk) {
//     // }

//     // regs->SPI_CTRL_REG &= ~(SPI_SPI_CTRL_REG_SPI_ON_Msk |
//     //                         SPI_SPI_CTRL_REG_SPI_INT_BIT_Msk);
//     // regs->SPI_CTRL_REG |= SPI_SPI_CTRL_REG_SPI_RST_Msk;
//     auto instance = reinterpret_cast<Spi_instance const *>(obj->instance);
//     release(obj, *instance);
// }
// void spi_format(spi_t *obj, int bits, int mode, int slave)
// {
//     // debug("bits:%d, mode:%d, slave:%d\n", bits, mode, slave);
//     Expects(slave == 0);
//     auto instance = reinterpret_cast<Spi_instance const *>(obj->instance);
//     // instance->disable();

//     // debug("format base_reg %p\n", instance->m_regs->SPI_CTRL_REG);
//     // auto ctrl_reg =
//     //     instance->m_regs->SPI_CTRL_REG & (SPI_SPI_CTRL_REG_SPI_TX_FIFO_NOTFULL_MASK_Msk | SPI_SPI_CTRL_REG_SPI_DMA_TXREQ_MODE_Msk | SPI_SPI_CTRL_REG_SPI_PRIORITY_Msk |
//     //                                       SPI_SPI_CTRL_REG_SPI_EN_CTRL_Msk | SPI_SPI_CTRL_REG_SPI_SMN_Msk | SPI_SPI_CTRL_REG_SPI_DO_Msk | SPI_SPI_CTRL_REG_SPI_RST_Msk |
//     //                                       SPI_SPI_CTRL_REG_SPI_CLK_Msk);
//     auto ctrl_reg = instance->m_regs->SPI_CTRL_REG & (~(SPI_SPI_CTRL_REG_SPI_PHA_Msk | SPI_SPI_CTRL_REG_SPI_POL_Msk | SPI_SPI_CTRL_REG_SPI_WORD_Msk));
//     // debug("format base_reg_mask %p\n", ctrl_reg);
//     switch (mode)
//     {
//     case 0:
//         /* Bits already zeroed */
//         break;
//     case 1:
//         ctrl_reg |= (1U << SPI_SPI_CTRL_REG_SPI_PHA_Pos);
//         break;
//     case 2:
//         ctrl_reg |= (1U << SPI_SPI_CTRL_REG_SPI_POL_Pos);
//         break;
//     case 3:
//         ctrl_reg |= (1U << SPI_SPI_CTRL_REG_SPI_PHA_Pos) | (1U << SPI_SPI_CTRL_REG_SPI_POL_Pos);
//         break;
//     default:
//         Expects(0);
//         break;
//     }

//     switch (bits)
//     {
//     case 8:
//         break;
//     case 32:
//         ctrl_reg |= (2U << SPI_SPI_CTRL_REG_SPI_WORD_Pos);
//         break;
//     default:
//         Expects(false);
//         break;
//     }
//     instance->m_regs->SPI_CTRL_REG = ctrl_reg;

//     // debug("format %p\n", reinterpret_cast<void *>(instance->m_regs->SPI_CTRL_REG));
// }
// void spi_frequency(spi_t *obj, int hz)
// {
//     // debug("hz:%d\n", hz);
//     auto instance = reinterpret_cast<Spi_instance const *>(obj->instance);
//     // instance->disable();

//     /* Preserve some register fields only */
//     // debug("hz base_reg %p\n", instance->m_regs->SPI_CTRL_REG);
//     // auto ctrl_reg = instance->m_regs->SPI_CTRL_REG & (SPI_SPI_CTRL_REG_SPI_TX_FIFO_NOTFULL_MASK_Msk | SPI_SPI_CTRL_REG_SPI_DMA_TXREQ_MODE_Msk | SPI_SPI_CTRL_REG_SPI_PRIORITY_Msk |
//     //                                                   SPI_SPI_CTRL_REG_SPI_EN_CTRL_Msk | SPI_SPI_CTRL_REG_SPI_SMN_Msk | SPI_SPI_CTRL_REG_SPI_DO_Msk | SPI_SPI_CTRL_REG_SPI_RST_Msk |
//     //                                                   SPI_SPI_CTRL_REG_SPI_POL_Msk | SPI_SPI_CTRL_REG_SPI_PHA_Msk | SPI_SPI_CTRL_REG_SPI_WORD_Msk);
//     auto ctrl_reg = instance->m_regs->SPI_CTRL_REG & (~SPI_SPI_CTRL_REG_SPI_CLK_Msk);
//     // debug("hz base_reg_mask %p\n", ctrl_reg);
//     switch (hz)
//     {
//     case 16000000:
//         ctrl_reg |= (2U << SPI_SPI_CTRL_REG_SPI_CLK_Pos);
//         break;
//     case 8000000:
//         ctrl_reg |= (1U << SPI_SPI_CTRL_REG_SPI_CLK_Pos);
//         break;
//     case 4000000:
//         ctrl_reg |= (0U << SPI_SPI_CTRL_REG_SPI_CLK_Pos);
//         break;
//     default:
//         ctrl_reg |= (3U << SPI_SPI_CTRL_REG_SPI_CLK_Pos);
//         break;
//     }
//     instance->m_regs->SPI_CTRL_REG = ctrl_reg;
//     // instance->enable();
//     // debug("set hz %p\n", reinterpret_cast<void *>(instance->m_regs->SPI_CTRL_REG));
// }
// int spi_master_write(spi_t *obj, int value)
// {
//     cs->write(0);
//     auto instance = reinterpret_cast<Spi_instance const *>(obj->instance);
//     auto regs = instance->m_regs;

//     /* Get rid of old data if any */
//     while (!(regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_TX_FIFO_EMPTY_Msk))
//     {
//     }
//     uint32_t dummy;
//     while (!(regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk))
//     {
//         dummy = regs->SPI_RX_TX_REG;
//         (void)dummy;
//     }
//     regs->SPI_RX_TX_REG = value;
//     while (regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk)
//     {
//     }

//     // ctrl_reg = regs->SPI_CTRL_REG;
//     auto receive_value = regs->SPI_RX_TX_REG;
//     cs->write(1);
//     return receive_value;
// }
// int spi_master_block_write(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
// {
//     cs->write(0);
//     auto instance = reinterpret_cast<Spi_instance const *>(obj->instance);
//     // debug("block write %p\n", reinterpret_cast<void *>(instance->m_regs->SPI_CTRL_REG));
//     auto regs = instance->m_regs;
//     uint32_t val{};
//     while (!(regs->SPI_CTRL_REG & SPI_SPI_CTRL_REG_SPI_RX_FIFO_EMPTY_Msk))
//     {
//         val = regs->SPI_RX_TX_REG;
//     }
//     auto total = (tx_length > rx_length) ? tx_length : rx_length;
//     auto bit = instance->get_bit();
//     switch (bit)
//     {
//     case Spi_bit::bit_8:
//         instance->write_block<uint8_t>(tx_buffer, tx_length, rx_buffer, rx_length, write_fill);
//         break;

//     case Spi_bit::bit_32:
//         instance->write_block<uint32_t>(tx_buffer, tx_length, rx_buffer, rx_length, write_fill);
//         break;
//     }
//     cs->write(1);
//     return total;
// }
void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName clock, PinName chip_select)
{
    Expects(obj->instance == 0);
    obj->instance = new Spi_instance{};
    auto &config = obj->instance->m_config;
    config.m_mosi_pin = mosi;
    config.m_miso_pin = miso;
    config.m_clock_pin = clock;
    config.m_cs_pin = chip_select;
    obj->instance->initialize();
}
int spi_master_block_write(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
    Expects(obj->instance);
    Expects(obj->instance->m_spi);
    span<uint8_t const> tx{reinterpret_cast<uint8_t const *>(tx_buffer), tx_length};
    span<uint8_t> rx{reinterpret_cast<uint8_t *>(rx_buffer), rx_length};
    return obj->instance->m_spi->block_write(tx, rx, write_fill);    
}
int spi_master_write(spi_t *obj, int value)
{
    Expects(obj->instance);
    Expects(obj->instance->m_spi);
    switch (obj->instance->m_config.m_bits)
    {
    case DA1469x::Spi::Config::Bits::bits8:
        return obj->instance->m_spi->write(value & 0xFF);
    }
    Expects(false);
    return 0;
}
void spi_frequency(spi_t *obj, int hz)
{
    Expects(obj->instance);
    auto frequency = DA1469x::Spi::Config::int_to_frequency(hz);
    obj->instance->m_config.m_frequency = frequency;
    obj->instance->initialize();
}
void spi_format(spi_t *obj, int bits, int mode, int slave)
{
    Expects(obj->instance);
    Expects(slave == 0);
    auto &config = obj->instance->m_config;
    switch (bits)
    {
    case 8:
        config.m_bits = DA1469x::Spi::Config::Bits::bits8;
        break;
    default:
        Expects(false);
        break;
    }
    switch (mode)
    {
    case 0:
        config.m_mode = DA1469x::Spi::Config::Mode::mode_0;
        break;
    case 1:
        config.m_mode = DA1469x::Spi::Config::Mode::mode_1;
        break;
    case 2:
        config.m_mode = DA1469x::Spi::Config::Mode::mode_2;
        break;
    case 3:
        config.m_mode = DA1469x::Spi::Config::Mode::mode_3;
        break;
    default:
        Expects(false);
        break;
    }
    obj->instance->initialize();
}
void spi_free(spi_t *obj)
{
    Expects(obj->instance);
    delete obj->instance;
    obj->instance = 0;
}
void Spi_instance::initialize()
{
    if (m_config.m_frequency == DA1469x::Spi::Config::Frequency::unset)
    {
        return;
    }
    switch (m_config.m_frequency)
    {
    case DA1469x::Spi::Config::Frequency::unset:
        return;
    case DA1469x::Spi::Config::Frequency::freq_16m:
    case DA1469x::Spi::Config::Frequency::freq_8m:
    case DA1469x::Spi::Config::Frequency::freq_4m:
    case DA1469x::Spi::Config::Frequency::freq_2_28m:
        m_spi = make_unique<DA1469x::Spi_plus>(m_config);
        break;
    }
    // m_spi->initialize();
}

DA1469x::Impl::Spi::Spi(Config &config, Spi_instance::Type type) : m_config(config), m_id(DA1469x::Spi_instance::get_instance().acquire(type))
{
}

DA1469x::Impl::Spi::~Spi()
{
    DA1469x::Spi_instance::get_instance().release(m_id);
}
