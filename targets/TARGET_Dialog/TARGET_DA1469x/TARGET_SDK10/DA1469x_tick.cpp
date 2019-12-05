#include "da1469x_tick.h"
#include "lp_ticker_api.h"
#include "newt_sdks.h"
#include "mbed_debug.h"
void lp_ticker_init(void)
{
    uint32_t primask;

    // g_hal_os_tick.last_trigger_val = 0;
    // g_hal_os_tick.ticks_per_ostick = 32768 / os_ticks_per_sec;
    // g_hal_os_tick.max_idle_ticks = (1UL << 22) / g_hal_os_tick.ticks_per_ostick;

    TIMER2->TIMER2_CTRL_REG = 0;
    TIMER2->TIMER2_PRESCALER_REG = 0;
    // TIMER2->TIMER2_CTRL_REG |= TIMER2_TIMER2_CTRL_REG_TIM_CLK_EN_Msk;
    // TIMER2->TIMER2_CTRL_REG |= (TIMER2_TIMER2_CTRL_REG_TIM_FREE_RUN_MODE_EN_Msk | TIMER2_TIMER2_CTRL_REG_TIM_IRQ_EN_Msk | TIMER2_TIMER2_CTRL_REG_TIM_EN_Msk);

    __HAL_DISABLE_INTERRUPTS(primask);

    // NVIC_SetPriority(TIMER2_IRQn, 0);
    NVIC_SetVector(TIMER2_IRQn, (uint32_t)lp_ticker_irq_handler);
    NVIC_EnableIRQ(TIMER2_IRQn);

    __HAL_ENABLE_INTERRUPTS(primask);
}
void lp_ticker_free(void)
{
    TIMER2->TIMER2_CTRL_REG = 0;
    TIMER2->TIMER2_PRESCALER_REG = 0;
}

uint32_t lp_ticker_read(void)
{
    uint32_t count = 0;
    uint32_t last_count = 0;

    // Loop until the same tick is read twice since this
    // is ripple counter on a different clock domain.
    count = TIMER2->TIMER2_TIMER_VAL_REG;
    do
    {
        last_count = count;
        count = TIMER2->TIMER2_TIMER_VAL_REG;
    } while (last_count != count);

    return count;
}

void lp_ticker_set_interrupt(timestamp_t timestamp)
{
    TIMER2->TIMER2_CTRL_REG |= TIMER2_TIMER2_CTRL_REG_TIM_CLK_EN_Msk;
    TIMER2->TIMER2_CTRL_REG |= (TIMER2_TIMER2_CTRL_REG_TIM_FREE_RUN_MODE_EN_Msk | TIMER2_TIMER2_CTRL_REG_TIM_IRQ_EN_Msk | TIMER2_TIMER2_CTRL_REG_TIM_EN_Msk);
    TIMER2->TIMER2_RELOAD_REG = timestamp & 0xFFFFFFFF;
}

void lp_ticker_disable_interrupt(void)
{
    TIMER2->TIMER2_CTRL_REG = 0;
}
/** Clear the low power ticker interrupt
 *
 * Pseudo Code:
 * @code
 * void lp_ticker_clear_interrupt(void)
 * {
 *     // Write to the ICR (interrupt clear register) of the LPTMR
 *     LPTMR_ICR = LPTMR_ICR_COMPARE_Msk;
 * }
 * @endcode
 */
void lp_ticker_clear_interrupt(void)
{
    TIMER2->TIMER2_CLEAR_IRQ_REG = 1;
}

void lp_ticker_fire_interrupt(void)
{
    NVIC_SetPendingIRQ(TIMER2_IRQn);
}
const ticker_info_t *lp_ticker_get_info()
{
    static const ticker_info_t info = {
        32768, // 32KHz
        24     // 16 bit counter
    };
    return &info;
}