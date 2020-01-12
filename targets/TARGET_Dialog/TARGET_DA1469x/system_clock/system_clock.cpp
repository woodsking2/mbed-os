#include "mbed.h"
extern "C"
{
#include "default_config.h"
#include "sdk_defs.h"
#include "hw_clk.h"
#include "hw_pdc.h"
#include "hw_pmu.h"
#include "hw_otpc.h"
#include "hw_qspi.h"
#include "qspi_automode.h"
}

#include "system_clock.h"
#include "gsl/gsl"
#include "mbed_debug.h"

using namespace gsl;
using namespace std;
using namespace mbed;
using namespace rtos;
namespace
{
__RETAINED uint32_t xtal32_pdc_entry;
volatile bool xtal32m_settled;
events::EventQueue *high_prio_event;
uint32_t get_pdc_xtal32m_entry(void)
{
    auto index = hw_find_pdc_entry(HW_PDC_TRIG_SELECT_PERIPHERAL, HW_PDC_PERIPH_TRIG_ID_TIMER2, HW_PDC_MASTER_CM33, HW_PDC_LUT_ENTRY_EN_XTAL);
    if (index != HW_PDC_INVALID_LUT_INDEX)
    {
        return index;
    }
    Expects(false);
    return HW_PDC_INVALID_LUT_INDEX;
}
void enable_xtalm()
{
    GLOBAL_INT_DISABLE();

#if dg_configUSE_HW_PDC
    if (xtal32_pdc_entry == HW_PDC_INVALID_LUT_INDEX)
    {
        // Find a PDC entry for enabling XTAL32M
        xtal32_pdc_entry = get_pdc_xtal32m_entry();

        if (xtal32_pdc_entry == HW_PDC_INVALID_LUT_INDEX)
        {
            Expects(false);
            // If no PDC entry exists, add a new entry for enabling the XTAL32M
            // xtal32_pdc_entry = hw_pdc_add_entry(HW_PDC_TRIGGER_FROM_MASTER(HW_PDC_MASTER_CM33, HW_PDC_LUT_ENTRY_EN_XTAL));
        }

        ASSERT_WARNING(xtal32_pdc_entry != HW_PDC_INVALID_LUT_INDEX);

        // XTAL32M may not been started. Use PDC to start it.
        hw_pdc_set_pending(xtal32_pdc_entry);
        hw_pdc_acknowledge(xtal32_pdc_entry);

        // Clear the XTAL32M_XTAL_ENABLE bit to allow PDC to disable XTAL32M when going to sleep.
        hw_clk_disable_sysclk(SYS_CLK_IS_XTAL32M);
    }
#endif

    xtal32m_settled = hw_clk_is_xtalm_started();

    if (xtal32m_settled == false)
    {
        if (hw_clk_is_enabled_sysclk(SYS_CLK_IS_XTAL32M) == false)
        {
            // XTAL32M has not been started. Use PDC to start it.
            hw_pdc_set_pending(xtal32_pdc_entry);
            hw_pdc_acknowledge(xtal32_pdc_entry);
        }
    }
    GLOBAL_INT_RESTORE();
}

/**
 * \brief Get the CPU clock frequency in MHz
 *
 * \param[in] clk The system clock
 * \param[in] div The HCLK divider
 *
 * \return The clock frequency
 */
uint32_t get_clk_freq(sys_clk_t clk, ahb_div_t div)
{
    sys_clk_t clock = clk;

    if (clock == sysclk_RC32)
    {
        clock = sysclk_XTAL32M;
    }

    return (16 >> div) * clock;
}
/**
 * \brief Adjust OTP access timings according to the AHB clock frequency.
 *
 * \warning In mirrored mode, the OTP access timings are left unchanged since the system is put to
 *          sleep using the RC32M clock and the AHB divider set to 1, which are the same settings
 *          that the system runs after a power-up or wake-up!
 */
__RETAINED_CODE void adjust_otp_access_timings(sys_clk_t clk, ahb_div_t div)
{
#if (dg_configUSE_HW_OTPC == 1)
    uint32_t clk_freq{};

    if (hw_otpc_is_active())
    {
        clk_freq = get_clk_freq(clk, div);

        hw_otpc_set_speed(static_cast<HW_OTPC_CLK_FREQ>(hw_otpc_convert_sys_clk_mhz(clk_freq)));
    }
#endif
}

} // namespace
class System_clock::Impl
{
  private:
    System_clock::Clock m_clock;
    volatile bool pll_locked;
    HW_PMU_1V2_VOLTAGE vdd_voltage;
    ahb_div_t m_hw_ahb_div;
    Semaphore m_pll_sem;

    void low_power_clock_initialize();
    void enable_pll();
    void wait_pll_lock();
    void switch_to_pll(void);
    sys_clk_t get_hw_clock();

  public:
    Impl();
    void initialize();
    void set(System_clock::Clock clock);
    void xtal_ready();
    void pll_ready();
};
sys_clk_t System_clock::Impl::get_hw_clock()
{
    switch (m_clock)
    {
    case System_clock::Clock::xtal:
        return sysclk_XTAL32M;
    case System_clock::Clock::pll:
        return sysclk_PLL96;
    }
    Ensures(false);
    return sysclk_XTAL32M;
}
void System_clock::Impl::switch_to_pll(void)
{
    if (hw_clk_get_sysclk() == SYS_CLK_IS_XTAL32M)
    {
        // Slow --> fast clock switch
        adjust_otp_access_timings(get_hw_clock(), m_hw_ahb_div); // Adjust OTP timings
        qspi_automode_sys_clock_cfg(sysclk_PLL96);

        /*
         * If ultra-fast wake-up mode is used, make sure that the startup state
         * machine is finished and all power regulation is in order.
         */
        while (REG_GETF(CRG_TOP, SYS_STAT_REG, POWER_IS_UP) == 0)
        {
            __NOP();
        }

        /*
         * Wait for LDO to be OK. Core voltage may have been changed from 0.9V to
         * 1.2V in order to switch system clock to PLL
         */
        while ((REG_GETF(CRG_TOP, ANA_STATUS_REG, LDO_CORE_OK) == 0) && (REG_GETF(DCDC, DCDC_STATUS1_REG, DCDC_VDD_AVAILABLE) == 0))
        {
            __NOP();
        }

        hw_clk_set_sysclk(SYS_CLK_IS_PLL); // Set PLL as sys_clk
    }
}
void System_clock::Impl::xtal_ready()
{
    debug("xtal_ready\n");
    //     if (sysclk != sysclk_LP)
    //     {
    //         // Restore system clocks. xtal32m_rdy_cnt is updated in  cm_sys_clk_sleep()
    //         cm_sys_clk_sleep(false);

    // #ifdef OS_FREERTOS
    //         if (xEventGroupCM_xtal != NULL)
    //         {
    //             OS_BASE_TYPE xHigherPriorityTaskWoken, xResult;

    //             xResult = xtal32m_is_ready(&xHigherPriorityTaskWoken);

    //             if (xResult != OS_FAIL)
    //             {
    //                 /*
    //                  * If xHigherPriorityTaskWoken is now set to pdTRUE then a context
    //                  * switch should be requested.
    //                  */
    //                 OS_EVENT_YIELD(xHigherPriorityTaskWoken);
    //             }
    //         }
    // #endif
    //     }
}
void System_clock::Impl::pll_ready()
{
    // debug("pll_ready\n");
    ASSERT_WARNING(hw_clk_is_pll_locked());

    pll_locked = true;

    Ensures(m_clock == Clock::pll);
    switch_to_pll();
    m_pll_sem.release();
    // if (xEventGroupCM_xtal != NULL)
    // {
    //     OS_BASE_TYPE xHigherPriorityTaskWoken, xResult;

    //     xResult = pll_is_locked(&xHigherPriorityTaskWoken);

    //     if (xResult != OS_FAIL)
    //     {
    //         /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
    //          * switch should be requested. */
    //         OS_EVENT_YIELD(xHigherPriorityTaskWoken);
    //     }
    // }
}
void System_clock::Impl::enable_pll()
{
    if (hw_clk_is_pll_locked())
    {
        pll_locked = true;
    }
    else if (hw_clk_is_enabled_sysclk(SYS_CLK_IS_PLL) == false)
    {
        ASSERT_WARNING(!pll_locked);

        HW_PMU_1V2_RAIL_CONFIG rail_config;
        hw_pmu_get_1v2_active_config(&rail_config);

        // PLL cannot be powered by retention LDO
        ASSERT_WARNING(rail_config.current == HW_PMU_1V2_MAX_LOAD_50 || rail_config.src_type == HW_PMU_SRC_TYPE_DCDC_HIGH_EFFICIENCY);

        vdd_voltage = rail_config.voltage;
        if (vdd_voltage != HW_PMU_1V2_VOLTAGE_1V2)
        {
            // VDD voltage must be set to 1.2V prior to switching clock to PLL
            HW_PMU_ERROR_CODE error_code;
            error_code = hw_pmu_1v2_onwakeup_set_voltage(HW_PMU_1V2_VOLTAGE_1V2);
            ASSERT_WARNING(error_code == HW_PMU_ERROR_NOERROR);
        }

        hw_clk_enable_sysclk(SYS_CLK_IS_PLL); // Turn on PLL
        DBG_SET_HIGH(CLK_MGR_USE_TIMING_DEBUG, CLKDBG_PLL_ON);
    }
}
void System_clock::Impl::wait_pll_lock()
{
    m_pll_sem.acquire();
}
System_clock::Impl::Impl() : m_clock(System_clock::Clock::xtal)
{
}
void System_clock::Impl::set(System_clock::Clock clock)
{
    if (clock == m_clock)
    {
        return;
    }
    m_clock = clock;
    if (clock == System_clock::Clock::pll)
    {
        // debug("enable pll\n");
        // pll 需要先开启32m
        enable_pll();
        wait_pll_lock();
        // debug("pll ok\n");
    }
}
void System_clock::Impl::low_power_clock_initialize()
{
    hw_clk_set_lpclk(LP_CLK_IS_XTAL32K);
}
void System_clock::Impl::initialize()
{
    // ahbclk = cm_ahb_get_clock_divider();
    // apbclk = cm_apb_get_clock_divider();
    m_hw_ahb_div = hw_clk_get_hclk_div();

    high_prio_event = mbed_highprio_event_queue();
    HW_PMU_1V2_RAIL_CONFIG rail_config;
    hw_pmu_get_1v2_active_config(&rail_config);
    vdd_voltage = rail_config.voltage;
    hw_clk_disable_sysclk(SYS_CLK_IS_RC32);
    auto result = mbed_highprio_event_queue()->call_in(dg_configXTAL32K_SETTLE_TIME, callback(this, &System_clock::Impl::low_power_clock_initialize));
    Ensures(result != 0);
}
System_clock::System_clock() : m_impl(make_unique<System_clock::Impl>())
{
}
System_clock::~System_clock() = default;

System_clock &System_clock::get_instance()
{
    static System_clock instance;
    return instance;
}
void System_clock::initialize()
{
    m_impl->initialize();
}
void low_level_clock_init_initialize()
{
    NVIC_ClearPendingIRQ(XTAL32M_RDY_IRQn);
    NVIC_EnableIRQ(XTAL32M_RDY_IRQn); // Activate XTAL32 Ready IRQ

    NVIC_ClearPendingIRQ(PLL_LOCK_IRQn);
    NVIC_EnableIRQ(PLL_LOCK_IRQn); // Activate PLL Lock IRQ

    /*
     * Low power clock
     */
    hw_clk_enable_lpclk(LP_CLK_IS_RC32K);
    hw_clk_set_lpclk(LP_CLK_IS_RC32K);

    ASSERT_WARNING(REG_GETF(CRG_TOP, SYS_STAT_REG, TIM_IS_UP));

    hw_clk_xtalm_configure();
    if (dg_configXTAL32M_SETTLE_TIME_IN_USEC != 0)
    {

        uint16_t rdy_cnt = XTAL32M_USEC_TO_256K_CYCLES(dg_configXTAL32M_SETTLE_TIME_IN_USEC);

        hw_clk_set_xtalm_settling_time(rdy_cnt / 8, false);
    }

#if (dg_configLP_CLK_SOURCE == LP_CLK_IS_DIGITAL)
    hw_clk_configure_ext32k_pins();          // Configure Ext32K pins
    hw_clk_disable_lpclk(LP_CLK_IS_XTAL32K); // Disable XTAL32K
    hw_clk_disable_lpclk(LP_CLK_IS_RCX);     // Disable RCX
    hw_clk_set_lpclk(LP_CLK_IS_EXTERNAL);    // Set EXTERNAL as the LP clock
#elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
    hw_clk_enable_lpclk(LP_CLK_IS_RCX);      // Enable RCX
    hw_clk_disable_lpclk(LP_CLK_IS_XTAL32K); // Disable XTAL32K
                                             // LP clock will be switched to RCX after RCX calibration
#elif ((dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768))
    // No need to configure XTAL32K pins. Pins are automatically configured
    // when LP_CLK_IS_XTAL32K is enabled.
    hw_clk_configure_lpclk(LP_CLK_IS_XTAL32K); // Configure XTAL32K
    hw_clk_enable_lpclk(LP_CLK_IS_XTAL32K);    // Enable XTAL32K
    hw_clk_disable_lpclk(LP_CLK_IS_RCX);       // Disable RCX
                                               // LP clock cannot be set to XTAL32K here. XTAL32K needs a few seconds to settle after power up.
#else
#error not here
#endif

#if dg_configUSE_HW_PDC
    xtal32_pdc_entry = HW_PDC_INVALID_LUT_INDEX;
#endif
}

void low_level_clock_init_set_up()
{
    // Always enable the XTAL32M
    enable_xtalm();
    while (!xtal32m_settled)
    {
        __NOP();
        // Wait for XTAL32M to settle
    }
    hw_clk_set_sysclk(SYS_CLK_IS_XTAL32M); // Set XTAL32M as sys_clk

#if ((dg_configLP_CLK_SOURCE == LP_CLK_IS_ANALOG) && (dg_configUSE_LP_CLK == LP_CLK_RCX))
    /*
     * Note: If the LP clock is the RCX then we have to wait for the XTAL32M to settle
     *       since we need to estimate the frequency of the RCX before continuing
     *       (calibration procedure).
     */
    cm_rcx_calibrate();
    hw_clk_set_lpclk(LP_CLK_IS_RCX); // Set RCX as the LP clock
#endif
}
void PLL_Lock_Handler()
{
    if (high_prio_event)
    {
        high_prio_event->call(callback(&System_clock::get_instance(), &System_clock::pll_ready));
    }
}
void XTAL32M_Ready_Handler()
{
    ASSERT_WARNING(hw_clk_is_xtalm_started());

    if (dg_configXTAL32M_SETTLE_TIME_IN_USEC == 0)
    {
        if (hw_sys_hw_bsr_try_lock(HW_BSR_MASTER_SYSCPU, HW_BSR_WAKEUP_CONFIG_POS))
        {
            hw_clk_xtalm_update_rdy_cnt();
            hw_sys_hw_bsr_unlock(HW_BSR_MASTER_SYSCPU, HW_BSR_WAKEUP_CONFIG_POS);
        }
        else
        {
            /*
             * CMAC has locked the BSR entry so CMAC will update the RDY counter.
             * No need to do anything.
             */
        }
    }
    xtal32m_settled = true;
    if (high_prio_event)
    {
        high_prio_event->call(callback(&System_clock::get_instance(), &System_clock::xtal_ready));
    }
}
void System_clock::set(Clock clock)
{
    m_impl->set(clock);
}
void System_clock::xtal_ready()
{
    m_impl->xtal_ready();
}
void System_clock::pll_ready()
{
    m_impl->pll_ready();
}