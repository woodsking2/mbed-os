#include "lp_ticker_api.h"
#include "gsl/gsl"
#include "mbed_debug.h"
extern "C"
{
#include "default_config.h"
#include "hw_timer.h"
}
#include "Main_thread.h"
using namespace gsl;

/* HAL lp ticker */

/** Initialize the low power ticker
 *
 * Initialize or re-initialize the ticker. This resets all the
 * clocking and prescaler registers, along with disabling
 * the compare interrupt.
 *
 * Pseudo Code:
 * @code
 * void lp_ticker_init()
 * {
 *     // Enable clock gate so processor can read LPTMR registers
 *     POWER_CTRL |= POWER_CTRL_LPTMR_Msk;
 *
 *     // Disable the timer and ensure it is powered down
 *     LPTMR_CTRL &= ~(LPTMR_CTRL_ENABLE_Msk | LPTMR_CTRL_COMPARE_ENABLE_Msk);
 *
 *     // Configure divisors - no division necessary
 *     LPTMR_PRESCALE = 0;
 *     LPTMR_CTRL |= LPTMR_CTRL_ENABLE_Msk;
 *
 *     // Install the interrupt handler
 *     NVIC_SetVector(LPTMR_IRQn, (uint32_t)lp_ticker_irq_handler);
 *     NVIC_EnableIRQ(LPTMR_IRQn);
 * }
 * @endcode
 */
void lp_ticker_init(void)
{
    timer_config timer_cfg = {
        .clk_src = HW_TIMER_CLK_SRC_INT,
        .prescaler = 0,
        .mode = HW_TIMER_MODE_TIMER,
        .timer =
            {
                .direction = HW_TIMER_DIR_UP,
                .reload_val = 0,
                .free_run = true,
            },
        .pwm = {.frequency = 0, .duty_cycle = 0},
    };
    hw_timer_init(HW_TIMER2, &timer_cfg);
    hw_timer_enable(HW_TIMER2);
}

/** Deinitialize the lower power ticker
 *
 * Powerdown the lp ticker in preparation for sleep, powerdown, or reset.
 *
 * After calling this function no other ticker functions should be called except
 * lp_ticker_init(). Calling any function other than init after freeing is
 * undefined.
 *
 * @note This function stops the ticker from counting.
 */
void lp_ticker_free(void)
{
    hw_timer_disable(HW_TIMER2);
}

/** Read the current tick
 *
 * If no rollover has occurred, the seconds passed since lp_ticker_init()
 * was called can be found by dividing the ticks returned by this function
 * by the frequency returned by ::lp_ticker_get_info.
 *
 * @return The current timer's counter value in ticks
 *
 * Pseudo Code:
 * @code
 * uint32_t lp_ticker_read()
 * {
 *     uint16_t count;
 *     uint16_t last_count;
 *
 *     // Loop until the same tick is read twice since this
 *     // is ripple counter on a different clock domain.
 *     count = LPTMR_COUNT;
 *     do {
 *         last_count = count;
 *         count = LPTMR_COUNT;
 *     } while (last_count != count);
 *
 *     return count;
 * }
 * @endcode
 */
uint32_t lp_ticker_read(void)
{
    uint32_t count = 0;
    uint32_t last_count = 0;
    count = hw_timer_get_count(HW_TIMER2);
    do
    {
        last_count = count;
        count = hw_timer_get_count(HW_TIMER2);
    } while (last_count != count);
    return count;
}

/** Set interrupt for specified timestamp
 *
 * @param timestamp The time in ticks to be set
 *
 * @note no special handling needs to be done for times in the past
 * as the common timer code will detect this and call
 * lp_ticker_fire_interrupt() if this is the case
 *
 * @note calling this function with timestamp of more than the supported
 * number of bits returned by ::lp_ticker_get_info results in undefined
 * behavior.
 *
 * Pseudo Code:
 * @code
 * void lp_ticker_set_interrupt(timestamp_t timestamp)
 * {
 *     LPTMR_COMPARE = timestamp;
 *     LPTMR_CTRL |= LPTMR_CTRL_COMPARE_ENABLE_Msk;
 * }
 * @endcode
 */
void lp_ticker_set_interrupt(timestamp_t timestamp)
{
    Expects(timestamp <= 0xffffff);
    hw_timer_set_reload(HW_TIMER2, timestamp);
    hw_timer_register_int(HW_TIMER2, lp_ticker_irq_handler);
}

/** Disable low power ticker interrupt
 *
 * Pseudo Code:
 * @code
 * void lp_ticker_disable_interrupt(void)
 * {
 *     // Disable the compare interrupt
 *     LPTMR_CTRL &= ~LPTMR_CTRL_COMPARE_ENABLE_Msk;
 * }
 * @endcode
 */
void lp_ticker_disable_interrupt(void)
{
    hw_timer_unregister_int(HW_TIMER2);
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
    hw_timer_clear_interrupt(HW_TIMER2);
}
/** Set pending interrupt that should be fired right away.
 *
 * Pseudo Code:
 * @code
 * void lp_ticker_fire_interrupt(void)
 * {
 *     NVIC_SetPendingIRQ(LPTMR_IRQn);
 * }
 * @endcode
 */
void lp_ticker_fire_interrupt(void)
{
    NVIC_SetPendingIRQ(TIMER2_IRQn);
}
/** Get frequency and counter bits of this ticker.
 *
 * Pseudo Code:
 * @code
 * const ticker_info_t* lp_ticker_get_info()
 * {
 *     static const ticker_info_t info = {
 *         32768,      // 32KHz
 *         16          // 16 bit counter
 *     };
 *     return &info;
 * }
 * @endcode
 */
const ticker_info_t *lp_ticker_get_info(void)
{
    static const ticker_info_t info = {
        32768, // 32KHz
        24     // 24 bit counter
    };
    return &info;
}