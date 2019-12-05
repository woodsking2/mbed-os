#include "DA1469x_clock.h"
#include "DA1469xAB.h"
#include "DA1469x_power_domain.h"
#include "newt_sdks.h"
#include "DA1469x_power_domain_controller.h"
namespace
{
void da1469x_clock_lp_rcx_disable(void)
{
    CRG_TOP->CLK_RCX_REG &= ~CRG_TOP_CLK_RCX_REG_RCX_ENABLE_Msk;
}
void da1469x_clock_sys_xtal32m_init(void)
{
    uint32_t reg;
    int xtalrdy_cnt;

    /* Number of lp_clk cycles (~30.5us) */
    xtalrdy_cnt = MYNEWT_VAL(MCU_CLOCK_XTAL32M_SETTLE_TIME_US) * 10 / 305;

    reg = CRG_XTAL->XTALRDY_CTRL_REG;
    reg &= ~(CRG_XTAL_XTALRDY_CTRL_REG_XTALRDY_CLK_SEL_Msk | CRG_XTAL_XTALRDY_CTRL_REG_XTALRDY_CNT_Msk);
    reg |= xtalrdy_cnt;
    CRG_XTAL->XTALRDY_CTRL_REG = reg;
}
void da1469x_clock_sys_xtal32m_enable(void)
{
    int idx;

    idx = da1469x_pdc_find(MCU_PDC_TRIGGER_SW_TRIGGER, MCU_PDC_MASTER_M33, MCU_PDC_EN_XTAL);
    if (idx < 0)
    {
        idx = da1469x_pdc_add(MCU_PDC_TRIGGER_SW_TRIGGER, MCU_PDC_MASTER_M33, MCU_PDC_EN_XTAL);
    }
    assert(idx >= 0);

    da1469x_pdc_set(idx);
    da1469x_pdc_ack(idx);
}
void da1469x_clock_sys_xtal32m_switch(void)
{
    if (CRG_TOP->CLK_CTRL_REG & CRG_TOP_CLK_CTRL_REG_RUNNING_AT_RC32M_Msk)
    {
        CRG_TOP->CLK_SWITCH2XTAL_REG = CRG_TOP_CLK_SWITCH2XTAL_REG_SWITCH2XTAL_Msk;
    }
    else
    {
        CRG_TOP->CLK_CTRL_REG &= ~CRG_TOP_CLK_CTRL_REG_SYS_CLK_SEL_Msk;
    }

    while (!(CRG_TOP->CLK_CTRL_REG & CRG_TOP_CLK_CTRL_REG_RUNNING_AT_XTAL32M_Msk))
        ;
}
static inline bool da1469x_clock_is_xtal32m_settled(void)
{
    return ((*(uint32_t *)0x5001001c & 0xff00) == 0) && ((*(uint32_t *)0x50010054 & 0x000f) != 0xb);
}
void da1469x_clock_sys_xtal32m_switch_safe(void)
{
    uint32_t primask;

    __HAL_DISABLE_INTERRUPTS(primask);

    NVIC_ClearPendingIRQ(XTAL32M_RDY_IRQn);

    if (!da1469x_clock_is_xtal32m_settled())
    {
        NVIC_EnableIRQ(XTAL32M_RDY_IRQn);
        while (!NVIC_GetPendingIRQ(XTAL32M_RDY_IRQn))
        {
            __WFI();
        }
        NVIC_DisableIRQ(XTAL32M_RDY_IRQn);
    }

    __HAL_ENABLE_INTERRUPTS(primask);

    da1469x_clock_sys_xtal32m_switch();
}

void da1469x_clock_sys_rc32m_disable(void)
{
    CRG_TOP->CLK_RC32M_REG &= ~CRG_TOP_CLK_RC32M_REG_RC32M_ENABLE_Msk;
}

} // namespace
namespace DA1469x
{
namespace clock
{
void enable_otp()
{
    CRG_TOP->CLK_AMBA_REG |= CRG_TOP_CLK_AMBA_REG_OTP_ENABLE_Msk;
}
void disable_otp()
{
    CRG_TOP->CLK_AMBA_REG &= ~CRG_TOP_CLK_AMBA_REG_OTP_ENABLE_Msk;
}
void initialize()
{
    /* Reset clock dividers to 0 */
    CRG_TOP->CLK_AMBA_REG &= ~(CRG_TOP_CLK_AMBA_REG_HCLK_DIV_Msk | CRG_TOP_CLK_AMBA_REG_PCLK_DIV_Msk);

    /*
     * We cannot switch lp_clk to XTAL32K here since it needs some time to
     * settle, so we just disable RCX (we don't need it) and then we'll handle
     * switch to XTAL32K from sysinit since we need os_cputime for this.
     */    

    /* Make sure PD_TIM is up since this is where XTAL32M state machine runs */
    da1469x_pd_acquire(MCU_PD_DOMAIN_TIM);

    /* Switch to XTAL32M and disable RC32M */
    da1469x_clock_sys_xtal32m_init();
    da1469x_clock_sys_xtal32m_enable();
    da1469x_clock_sys_xtal32m_switch_safe();
    da1469x_clock_sys_rc32m_disable();

    da1469x_clock_lp_rcx_disable();
}
} // namespace clock
} // namespace DA1469x
