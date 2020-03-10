#include "CordioHCITransportDriver.h"
#include "configurable_MAC.h"
#include "gsl/gsl"
#include "mbed_critical.h"
#include "mbed_debug.h"
#include <algorithm>
// #include "application.h"
extern "C"
{
#include "default_config.h"
#include "hw_pdc.h"
#include "init_da1469x.h"
}
using namespace std;
using namespace gsl;

#define CMAC_SYM_CONFIG ((void *)(0x00818f20 + MEMCTRL->CMI_CODE_BASE_REG))
#define CMAC_SYM_CONFIG_DYN ((void *)(0x00821af8 + MEMCTRL->CMI_CODE_BASE_REG))
#define CMAC_SYM_MBOX_RX ((void *)(0x008216b0 + MEMCTRL->CMI_CODE_BASE_REG))
#define CMAC_SYM_MBOX_TX ((void *)(0x008218b0 + MEMCTRL->CMI_CODE_BASE_REG))

#define CMAC_MBOX_SIZE 504
#define CMAC_MBOX_F_RESET 0x0008
#define CMAC_MBOX_F_WRITEPENDING 0x0010

extern uint32_t cmi_fw_dst_addr;
extern uint32_t __cmi_section_end__;
extern uint32_t __cmi_fw_area_start[];
namespace
{
struct cmac_config
{
    uint8_t ble_bd_address[6];    // The BLE Device Address
    uint8_t rf_calibration_delay; // The maximum delay allowed for RF calibration (in multiples of 100 msec)
    uint8_t lp_clock_freq;        // 0 (32768Hz) and 1 (32000Hz) are supported settings, Default: 32768Hz
    uint16_t lp_clock_drift;      // Device SCA setting, Default: 500

    uint16_t ble_rx_buffer_size;         // BLE Rx data buffer size, Default: 262 bytes
    uint16_t ble_tx_buffer_size;         // BLE Tx data buffer size, Defalut: 262 bytes
    bool ble_length_exchange_needed;     // Flag to control Length Exchange, Default: true
    uint16_t ble_chnl_assess_timer;      // Channel Assessment Timer duration (5s -
                                         // Multiple of 10ms), Default: 500, Private
    uint8_t ble_chnl_reassess_timer;     // Channel Reassessment Timer duration (Multiple of
                                         // Channel Assessment Timer duration), Default: 8, Private
    int8_t ble_chnl_assess_min_rssi;     // BLE Chnl Assess alg, Min RSSI, Default: -60 (dBm)
    uint16_t ble_chnl_assess_nb_pkt;     // # of packets to receive for statistics, Default: 20, Private
    uint16_t ble_chnl_assess_nb_bad_pkt; // # of bad packets needed to remove a channel, Default: 10, Private
    uint8_t system_tcs_length;           // Number of valid entries in the table
    uint8_t synth_tcs_length;            // Number of valid entries in the table
    uint8_t rfcu_tcs_length;             // Number of valid entries in the table
    uint8_t initial_tx_power_lvl;        // The initial Tx power level used in the ADV and Data channels
    bool ble_dup_filter_found;           // Unknown devices are treated as "found" (be in the
                                         // duplicate filter buffer) when the buffer is full,
                                         // if true, Default: true, Private
    bool use_high_performace_1m;         // Enable 1M High Performance mode
    bool use_high_performace_2m;         // Enable 2M High Performance mode
};

struct cmac_mbox
{
    volatile uint16_t magic;
    volatile uint16_t flags;
    volatile uint16_t wr_off;
    volatile uint16_t rd_off;
    uint8_t data[CMAC_MBOX_SIZE];
};
struct cmac_config_dynamic
{
    bool enable_sleep; /* Enable sleep */

    /* More options here, don't care now */
};
struct cmac_mbox *cmac_mbox_rx;
struct cmac_mbox *cmac_mbox_tx;

int8_t pdc_timer_cmac_index;

// int8_t g_da1469x_pdc_cmac2sys;

int on_read(uint8_t *buff, size_t len)
{
    // debug("recv [%u]\n", len);

    ble::vendor::cordio::CordioHCITransportDriver::on_data_received(buff, len);

    return len;
}
void da1469x_cmac_pdc_signal(void)
{
    hw_pdc_set_pending(pdc_timer_cmac_index);
}
void cmac2sys_isr(void)
{
    uint16_t wr_off;
    uint16_t rd_off;
    uint16_t chunk;
    uint16_t len;

    /* Clear CMAC2SYS interrupt */
    *(volatile uint32_t *)0x40002000 = 2;

    if (*(volatile uint32_t *)0x40002000 & 0x1c00)
    {
        /* XXX CMAC is in error state, need to recover */
        Ensures(false);
    }

    if (cmac_mbox_rx->flags & CMAC_MBOX_F_RESET)
    {
        cmac_mbox_rx->flags &= ~CMAC_MBOX_F_RESET;
        goto done;
    }

    do
    {
        rd_off = cmac_mbox_rx->rd_off;
        wr_off = cmac_mbox_rx->wr_off;

        if (rd_off <= wr_off)
        {
            chunk = wr_off - rd_off;
        }
        else
        {
            chunk = CMAC_MBOX_SIZE - rd_off;
        }

        while (chunk)
        {
            len = on_read(&cmac_mbox_rx->data[rd_off], chunk);

            rd_off += len;
            chunk -= len;
        };

        cmac_mbox_rx->rd_off = rd_off % CMAC_MBOX_SIZE;
    } while (cmac_mbox_rx->rd_off != cmac_mbox_rx->wr_off);

done:

    if (cmac_mbox_rx->flags & CMAC_MBOX_F_WRITEPENDING)
    {
        da1469x_cmac_pdc_signal();
    }
}
} // namespace
class Configurable_MAC::Impl final : virtual public Nondestructive
{
  public:
    Impl();
    Configurable_MAC::Result initialize();
    Configurable_MAC::Result set_address(gsl::span<uint8_t, 6> address);
    Configurable_MAC::Result write(gsl::span<uint8_t const> data);

  private:
    bool initialized;
    array<uint8_t, 6> m_address;
};
Configurable_MAC::Impl::Impl() = default;

Configurable_MAC::Result Configurable_MAC::Impl::initialize()
{
    Expects(!initialized);
    if (initialized)
    {
        return Configurable_MAC::Result::initialized;
    }
    uint32_t cmac_addr_code = (uint32_t)&cmi_fw_dst_addr;
    uint32_t cmac_addr_data = cmac_addr_code & 0x0007fffc;
    uint32_t cmac_addr_end = (uint32_t)&__cmi_section_end__;
    uint32_t *cmac_fw_area = __cmi_fw_area_start;
    struct cmac_config *cmac_config;
    struct cmac_config_dynamic *cmac_config_dyn;

    /* Add PDC entry to wake up CMAC from M33 */
    pdc_timer_cmac_index = hw_pdc_add_entry(HW_PDC_LUT_ENTRY_VAL(HW_PDC_TRIG_SELECT_PERIPHERAL, HW_PDC_PERIPH_TRIG_ID_MAC_TIMER, HW_PDC_MASTER_CMAC, HW_PDC_LUT_ENTRY_EN_XTAL));
    hw_pdc_set_pending(pdc_timer_cmac_index);
    hw_pdc_acknowledge(pdc_timer_cmac_index);

    auto pdc_combo_m33_index = hw_find_pdc_entry(HW_PDC_TRIG_SELECT_PERIPHERAL, HW_PDC_PERIPH_TRIG_ID_COMBO, HW_PDC_MASTER_CM33,
                                                 (dg_configENABLE_XTAL32M_ON_WAKEUP ? HW_PDC_LUT_ENTRY_EN_XTAL : static_cast<HW_PDC_LUT_ENTRY_EN>(0)));
    hw_pdc_set_pending(pdc_combo_m33_index);
    hw_pdc_acknowledge(pdc_combo_m33_index);

    /* Enable Radio LDO */
    CRG_TOP->POWER_CTRL_REG |= CRG_TOP_POWER_CTRL_REG_LDO_RADIO_ENABLE_Msk;

    /* Enable CMAC, but keep it in reset */
    CRG_TOP->CLK_RADIO_REG = (1 << CRG_TOP_CLK_RADIO_REG_RFCU_ENABLE_Pos) | (1 << CRG_TOP_CLK_RADIO_REG_CMAC_SYNCH_RESET_Pos) | (0 << CRG_TOP_CLK_RADIO_REG_CMAC_CLK_SEL_Pos) |
                             (1 << CRG_TOP_CLK_RADIO_REG_CMAC_CLK_ENABLE_Pos) | (0 << CRG_TOP_CLK_RADIO_REG_CMAC_DIV_Pos);

    /* Setup CMAC memory base addresses */
    MEMCTRL->CMI_CODE_BASE_REG = cmac_addr_code;
    MEMCTRL->CMI_DATA_BASE_REG = cmac_addr_data;
    MEMCTRL->CMI_SHARED_BASE_REG = 0;
    MEMCTRL->CMI_END_REG = cmac_addr_end;

    /* Copy CMAC firmware to RAM (9 words of header data, 2nd word has FW size) */
    memcpy((void *)cmac_addr_code, &cmac_fw_area[9], cmac_fw_area[1]);

    /* Symbols below are in shared memory, can update them now */
    cmac_config = (struct cmac_config *)CMAC_SYM_CONFIG;
    cmac_config_dyn = (struct cmac_config_dynamic *)CMAC_SYM_CONFIG_DYN;
    cmac_mbox_rx = (struct cmac_mbox *)CMAC_SYM_MBOX_RX;
    cmac_mbox_tx = (struct cmac_mbox *)CMAC_SYM_MBOX_TX;
    memcpy(cmac_config->ble_bd_address, m_address.data(), m_address.size());

    /* Update CMAC configuration */
    cmac_config->lp_clock_freq = 0;
    cmac_config->lp_clock_drift = 500;

    cmac_config->ble_rx_buffer_size = 251 + 11;
    cmac_config->ble_tx_buffer_size = 251 + 11;
    cmac_config->ble_length_exchange_needed = true;

    cmac_config->ble_chnl_assess_timer = 500;

    cmac_config->ble_chnl_reassess_timer = 8;

    cmac_config->ble_chnl_assess_min_rssi = -60;
    cmac_config->ble_chnl_assess_nb_pkt = 20;
    cmac_config->ble_chnl_assess_nb_bad_pkt = 10;
    cmac_config->system_tcs_length = 0;
    cmac_config->synth_tcs_length = 0;
    cmac_config->rfcu_tcs_length = 0;
    // #if (dg_configBLE_INITIAL_TX_POWER == 6)
    //         cmac_config_table_ptr->initial_tx_power_lvl       = 0xF;
    // #elif (dg_configBLE_INITIAL_TX_POWER == 0)
    //         cmac_config_table_ptr->initial_tx_power_lvl       = 0x8;
    // #endif
    cmac_config->initial_tx_power_lvl = 0x8;
    cmac_config->ble_dup_filter_found = true;
    cmac_config->use_high_performace_1m = true;
    cmac_config->use_high_performace_2m = true;

    //debug("ble initialize\n");
    //for (int i = 0; i < 6; i++)
    //{
    //    debug("%02X ", cmac_config->ble_bd_address[i]);
    //}
    //debug("\n");

    cmac_config_dyn->enable_sleep = true;

    /* Release CMAC from reset */
    CRG_TOP->CLK_RADIO_REG &= ~CRG_TOP_CLK_RADIO_REG_CMAC_SYNCH_RESET_Msk;

    while (MEMCTRL->CMI_DATA_BASE_REG == cmac_addr_data)
    {
        /* Wait for CMAC to update registers */
    }
    while (MEMCTRL->CMI_SHARED_BASE_REG != (MEMCTRL->CMI_END_REG & 0xfffffc00))
    {
    }

    /* Initialize mailboxes and sync with CMAC */
    cmac_mbox_tx->flags = CMAC_MBOX_F_RESET;
    cmac_mbox_tx->wr_off = 0;
    cmac_mbox_tx->rd_off = 0;
    cmac_mbox_tx->magic = 0xa55a;
    while (cmac_mbox_rx->magic != 0xa55a)
    {
    }

    NVIC_SetVector(CMAC2SYS_IRQn, (uint32_t)cmac2sys_isr);
    NVIC_SetPriority(CMAC2SYS_IRQn, 0);
    NVIC_EnableIRQ(CMAC2SYS_IRQn);

    da1469x_cmac_pdc_signal();
    initialized = true;
    return Configurable_MAC::Result::success;
}
// __attribute__((optimize("O0")))
Configurable_MAC::Result Configurable_MAC::Impl::set_address(gsl::span<uint8_t, 6> address)
{
    Expects(!initialized);
    if (initialized)
    {
        return Configurable_MAC::Result::initialized;
    }
    struct cmac_config *cmac_config = (struct cmac_config *)CMAC_SYM_CONFIG;
    //debug("set_address address\n");
    //for (auto const value : address)
    //{
    //    debug("%02X ", value);
    //}
    //debug("\n");
    copy(address.begin(), address.end(), m_address.begin());
    return Configurable_MAC::Result::success;
}
Configurable_MAC::Result Configurable_MAC::Impl::write(gsl::span<uint8_t const> data)
{
    Expects(initialized);
    if (!initialized)
    {
        return Configurable_MAC::Result::uninitialized;
    }
    int wr_off{};
    int rd_off{};
    int chunk{};
    int len = data.size();
    uint8_t const *buf = data.data();

    // uint32_t primask;

    // __HAL_DISABLE_INTERRUPTS(primask);
    core_util_critical_section_enter();
    auto _ = finally([&]() { core_util_critical_section_exit(); });
    while (len)
    {
        rd_off = cmac_mbox_tx->rd_off;
        wr_off = cmac_mbox_tx->wr_off;

        if (rd_off > wr_off)
        {
            chunk = min(len, rd_off - wr_off);
        }
        else
        {
            chunk = min(len, CMAC_MBOX_SIZE - wr_off);
        }

        if (chunk == 0)
        {
            continue;
        }

        memcpy(&cmac_mbox_tx->data[wr_off], buf, chunk);

        wr_off += chunk;
        cmac_mbox_tx->wr_off = wr_off % CMAC_MBOX_SIZE;

        da1469x_cmac_pdc_signal();

        len -= chunk;
        buf += chunk;
    }

    return Configurable_MAC::Result::success;
}
Configurable_MAC::Configurable_MAC() : m_impl(make_unique<Configurable_MAC::Impl>())
{
}
Configurable_MAC::~Configurable_MAC() = default;

Configurable_MAC::Result Configurable_MAC::initialize()
{
    return m_impl->initialize();
}
Configurable_MAC::Result Configurable_MAC::set_address(gsl::span<uint8_t, 6> address)
{
    return m_impl->set_address(address);
}

Configurable_MAC::Result Configurable_MAC::write(gsl::span<uint8_t const> data)
{
    return m_impl->write(data);
}