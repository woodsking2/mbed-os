#include "CordioHCITransportDriver.h"
#include "configurable_MAC.h"
#include "gsl/gsl"
#include "mbed_critical.h"
#include "mbed_debug.h"
#include <algorithm>
extern "C"
{
#include "default_config.h"
#include "hw_pdc.h"
#include "init_da1469x.h"
}
using namespace std;
using namespace gsl;
// #ifndef min
// #define min(a, b) ((a) < (b) ? (a) : (b))
// #endif

// #ifndef max
// #define max(a, b) ((a) > (b) ? (a) : (b))
// #endif

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
    uint8_t bdaddr[6]; /* Device address */

    uint8_t rf_calibration_delay;

    uint8_t lp_clock_freq; /* Sleep clock frequency (0 = 32768Hz, 1 = 32000Hz) */
    uint16_t lp_clock_sca; /* Sleep clock accuracy [ppm] */

    uint16_t rx_buf_len; /* RX buffer size */
    uint16_t tx_buf_len; /* TX buffer size */
    bool initial_length_req;

    /* Channel assessment algorithm settings */
    uint16_t chan_assess_itvl;
    uint8_t chan_assess_itvl_mult;
    int8_t chan_assess_min_rssi;
    uint16_t chan_assess_pkt_num;
    uint16_t chan_assess_bad_pkt_num;

    /* Calibration settings */
    uint8_t system_tcs_length;
    uint8_t synth_tcs_length;
    uint8_t rfcu_tcs_length;

    uint8_t default_tx_power;   /* Default TX power for connection/advertising */
    bool filter_dup_ov_discard; /* Discard unknown devices when filter buffer is full */
    bool use_hp_1m;
    bool use_hp_2m;
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
class Configurable_MAC::Impl
{
  public:
    Impl();
    void initialize();
    void write(gsl::span<uint8_t const> data);
};
Configurable_MAC::Impl::Impl() = default;

void Configurable_MAC::Impl::initialize()
{
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
    // uint32_t fw_size = cmac_fw_area[1];
    // debug("fw_size:%u\n", fw_size);
    memcpy((void *)cmac_addr_code, &cmac_fw_area[9], cmac_fw_area[1]);

    /* Symbols below are in shared memory, can update them now */
    cmac_config = (struct cmac_config *)CMAC_SYM_CONFIG;
    cmac_config_dyn = (struct cmac_config_dynamic *)CMAC_SYM_CONFIG_DYN;
    cmac_mbox_rx = (struct cmac_mbox *)CMAC_SYM_MBOX_RX;
    cmac_mbox_tx = (struct cmac_mbox *)CMAC_SYM_MBOX_TX;

    /* Update CMAC configuration */
    cmac_config->lp_clock_freq = 0;
    cmac_config->lp_clock_sca = 50;
    cmac_config->rx_buf_len = 251 + 11;
    cmac_config->tx_buf_len = 251 + 11;
    cmac_config->initial_length_req = 0;
    cmac_config->system_tcs_length = 0;
    cmac_config->synth_tcs_length = 0;
    cmac_config->rfcu_tcs_length = 0;
    cmac_config->default_tx_power = 4;
    cmac_config_dyn->enable_sleep = true;

    /* Release CMAC from reset */
    CRG_TOP->CLK_RADIO_REG &= ~CRG_TOP_CLK_RADIO_REG_CMAC_SYNCH_RESET_Msk;

    /* Wait for CMAC to update registers */
    while (MEMCTRL->CMI_DATA_BASE_REG == cmac_addr_data)
        ;
    while (MEMCTRL->CMI_SHARED_BASE_REG != (MEMCTRL->CMI_END_REG & 0xfffffc00))
        ;

    /* Initialize mailboxes and sync with CMAC */
    cmac_mbox_tx->flags = CMAC_MBOX_F_RESET;
    cmac_mbox_tx->wr_off = 0;
    cmac_mbox_tx->rd_off = 0;
    cmac_mbox_tx->magic = 0xa55a;
    while (cmac_mbox_rx->magic != 0xa55a)
        ;

    NVIC_SetVector(CMAC2SYS_IRQn, (uint32_t)cmac2sys_isr);
    NVIC_SetPriority(CMAC2SYS_IRQn, 0);
    NVIC_EnableIRQ(CMAC2SYS_IRQn);

    da1469x_cmac_pdc_signal();
}
void Configurable_MAC::Impl::write(gsl::span<uint8_t const> data)
{
    int wr_off{};
    int rd_off{};
    int chunk{};
    int len = data.size();
    uint8_t const* buf = data.data();

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
}
Configurable_MAC::Configurable_MAC() : m_impl(make_unique<Configurable_MAC::Impl>())
{
}
Configurable_MAC::~Configurable_MAC() = default;
Configurable_MAC &Configurable_MAC::get_instance()
{
    static Configurable_MAC instance;
    return instance;
}
void Configurable_MAC::initialize()
{
    m_impl->initialize();
}
void Configurable_MAC::write(gsl::span<uint8_t const> data)
{
    m_impl->write(data);
}