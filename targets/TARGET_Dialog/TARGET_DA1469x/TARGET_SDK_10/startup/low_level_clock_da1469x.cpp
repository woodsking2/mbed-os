#include "low_level_clock_da1469x.h"
#include "gsl/gsl"
extern "C"
{
#include "default_config.h"
#include "sdk_defs.h"
#include "hw_clk.h"
#include "hw_pdc.h"
}
namespace
{
__RETAINED uint32_t xtal32_pdc_entry;
volatile bool xtal32m_settled = false;
// __RETAINED_RW sys_clk_t sysclk = sysclk_LP;
// sys_clk_t sys_clk_next;
// ahb_div_t ahb_clk_next;

__RETAINED_CODE bool cm_poll_xtalm_ready(void)
{
    return xtal32m_settled;
}
static uint32_t get_pdc_xtal32m_entry(void)
{
    auto index = hw_find_pdc_entry(HW_PDC_TRIG_SELECT_PERIPHERAL, HW_PDC_PERIPH_TRIG_ID_TIMER2, HW_PDC_MASTER_CM33, HW_PDC_LUT_ENTRY_EN_XTAL);
    if (index != HW_PDC_INVALID_LUT_INDEX)
    {
        return index;
    }
    Expects(false);
    return HW_PDC_INVALID_LUT_INDEX;
    // Search for any entry that will wake-up M33 and start the XTAL32M
    // field = HW_PDC_LUT_ENTRY_VAL(0, 0, HW_PDC_MASTER_CM33, HW_PDC_LUT_ENTRY_EN_XTAL);

    // mask = HW_PDC_LUT_ENTRY_VAL(0, 0, 0x3, HW_PDC_LUT_ENTRY_EN_XTAL);

    // index = hw_find_pdc_entry(HW_PDC_TRIG_SELECT_P0_GPIO, fiHW_PDC_PERIPH_TRIG_ID_TIMEReld);
    // return index;
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
#if dg_configUSE_HW_PDC
            // XTAL32M has not been started. Use PDC to start it.
            hw_pdc_set_pending(xtal32_pdc_entry);
            hw_pdc_acknowledge(xtal32_pdc_entry);

#else
            // PDC is not used. Enable XTAL32M by setting XTAL32M_XTAL_ENABLE bit
            // in XTAL32M_CTRL1_REG
            hw_clk_enable_sysclk(SYS_CLK_IS_XTAL32M);
#endif
        }
#if (dg_configENABLE_DA1469x_AA_SUPPORT)
        /* Workaround for bug2522A_075: XTAL32M does not start if V14 is supplied by DCDC */
        if (hw_clk_is_enabled_sysclk(SYS_CLK_IS_XTAL32M) == false)
        {
            /*
             * If XAL32M is still in IDLE state then the 1V4 rail is powered by the
             * DC/DC converter. In this case XTAL32M cannot be enabled if system has been
             * woken-up by the PDC.
             *
             * Toggle 1V4 rail DC/DC converter supply to allow XTAL32M to start.
             */

            /*
             * If ultra-fast wake-up mode is used, make sure that the startup state
             * machine is finished and all power regulation is in order.
             */
            while (REG_GETF(CRG_TOP, SYS_STAT_REG, POWER_IS_UP) == 0)
                ;

            uint32_t value = DCDC->DCDC_V14_REG;

            REG_CLR_FIELD(DCDC, DCDC_V14_REG, DCDC_V14_ENABLE_HV, value);
            REG_CLR_FIELD(DCDC, DCDC_V14_REG, DCDC_V14_ENABLE_LV, value);

            // Disable 1V4 rail supply from DC/DC converter. LDO is enabled automatically
            DCDC->DCDC_V14_REG = value;

            REG_SET_FIELD(DCDC, DCDC_V14_REG, DCDC_V14_ENABLE_HV, value, 1);
            REG_SET_FIELD(DCDC, DCDC_V14_REG, DCDC_V14_ENABLE_LV, value, 1);

            // Wait for LDO to be OK
            while (REG_GETF(CRG_TOP, ANA_STATUS_REG, LDO_RADIO_OK) == 0)
                ;

            // Enable DC/DC converter
            DCDC->DCDC_V14_REG = value;

            // Check if XTAL32M is started
            ASSERT_WARNING(hw_clk_is_enabled_sysclk(SYS_CLK_IS_XTAL32M));
        }
#endif /*dg_configENABLE_DA1469x_AA_SUPPORT */
    }
    GLOBAL_INT_RESTORE();
}
} // namespace
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
    while (!cm_poll_xtalm_ready())
    {
        // Wait for XTAL32M to settle
    }
    hw_clk_set_sysclk(SYS_CLK_IS_XTAL32M); // Set XTAL32M as sys_clk

#if (dg_configENABLE_DA1469x_AA_SUPPORT)
    /* Workaround for bug2522A_050: SW needed to overrule the XTAL calibration state machine */
    hw_clk_perform_init_rcosc_calibration(); // Perform initial RCOSC calibration
#endif

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
    ASSERT_WARNING(REG_GETF(CRG_XTAL, PLL_SYS_STATUS_REG, PLL_LOCK_FINE));
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
