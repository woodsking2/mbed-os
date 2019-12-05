#include "rtc_api.h"
#include "mbed_debug.h"
#include "gsl/gsl"
#include "mbed.h"
#include "mbed_mktime.h"
extern "C"
{
#include "default_config.h"
#include "hw_rtc.h"
}
/** Initialize the RTC peripheral
 *
 * Powerup the RTC in perpetration for access. This function must be called
 * before any other RTC functions ares called. This does not change the state
 * of the RTC. It just enables access to it.
 *
 * @note This function is safe to call repeatedly - Tested by ::rtc_init_test
 *
 * Example Implementation Pseudo Code:
 * @code
 * void rtc_init()
 * {
 *     // Enable clock gate so processor can read RTC registers
 *     POWER_CTRL |= POWER_CTRL_RTC_Msk;
 *
 *     // See if the RTC is already setup
 *     if (!(RTC_STATUS & RTC_STATUS_COUNTING_Msk)) {
 *
 *         // Setup the RTC clock source
 *         RTC_CTRL |= RTC_CTRL_CLK32_Msk;
 *     }
 * }
 * @endcode
 */
void rtc_init(void)
{
    // if (rtc_isenabled)
    // {
    //     return;
    // }
    // debug("rtc_init\n");
    // rtc_config const config = {
    //     .hour_clk_mode = RTC_24H_CLK,
    //     .keep_rtc = true,
    //     .pdc_evt =
    //         {
    //             .pdc_evt_en = false,
    //             .pdc_evt_period = 0,
    //         },
    //     .motor_evt =
    //         {
    //             .motor_evt_en = false,
    //             .motor_evt_period = 0,
    //         },
    // };
    // hw_rtc_init(&config);
    // hw_rtc_clock_enable();
    // hw_rtc_time_start();
}
/** Deinitialize RTC
 *
 * Powerdown the RTC in preparation for sleep, powerdown or reset. That should only
 * affect the CPU domain and not the time keeping logic.
 * After this function is called no other RTC functions should be called
 * except for ::rtc_init.
 *
 * @note This function does not stop the RTC from counting - Tested by ::rtc_persist_test
 *
 * Example Implementation Pseudo Code:
 * @code
 * void rtc_free()
 * {
 *     // Disable clock gate since processor no longer needs to read RTC registers
 *     POWER_CTRL &= ~POWER_CTRL_RTC_Msk;
 * }
 * @endcode
 */
void rtc_free(void)
{
    // nothing
}

/** Check if the RTC has the time set and is counting
 *
 * @retval 0 The time reported by the RTC is not valid
 * @retval 1 The time has been set the RTC is counting
 *
 * Example Implementation Pseudo Code:
 * @code
 * int rtc_isenabled()
 * {
 *     if (RTC_STATUS & RTC_STATUS_COUNTING_Msk) {
 *         return 1;
 *     } else {
 *         return 0;
 *     }
 * }
 * @endcode
 */
int rtc_isenabled(void)
{
    auto clock_enabled = hw_rtc_clock_enable_state();
    auto rtc_enabled = hw_rtc_time_start_state();
    if (clock_enabled && rtc_enabled)
    {
        return true;
    }
    return false;
}
/** Get the current time from the RTC peripheral
 *
 * @return The current time in seconds
 *
 * @note Some RTCs are not synchronized with the main clock. If
 * this is the case with your RTC then you must read the RTC time
 * in a loop to prevent reading the wrong time due to a glitch.
 * The test ::rtc_glitch_test is intended to catch this bug.
 *
 * Example implementation for an unsynchronized ripple counter:
 * @code
 * time_t rtc_read()
 * {
 *     uint32_t val;
 *     uint32_t last_val;
 *
 *     // Loop until the same value is read twice
 *     val = RTC_SECONDS;
 *     do {
 *         last_val = val;
 *         val = RTC_SECONDS;
 *     } while (last_val != val);
 *
 *     return (time_t)val;
 * }
 * @endcode
 */
time_t rtc_read(void)
{
    rtc_time _time{};
    rtc_calendar _calendar{};
    hw_rtc_get_time_clndr(&_time, &_calendar);
    struct tm time_info
    {
        .tm_sec = _time.sec, .tm_min = _time.minute, .tm_hour = _time.hour, .tm_mday = _calendar.mday, .tm_mon = _calendar.month - 1, .tm_year = _calendar.year - 1900, .tm_wday = _calendar.wday - 1,
        .tm_yday = 0, .tm_isdst = -1,
    };
    time_t time_second{};
    auto const result = _rtc_maketime(&time_info, &time_second, RTC_FULL_LEAP_YEAR_SUPPORT);
    Ensures(result);
    return time_second;
}
/** Write the current time in seconds to the RTC peripheral
 *
 * @param t The current time to be set in seconds.
 *
 * Example Implementation Pseudo Code:
 * @code
 * void rtc_write(time_t t)
 * {
 *     RTC_SECONDS = t;
 * }
 * @endcode
 */
void rtc_write(time_t time_second)
{

    rtc_config const config = {
        .hour_clk_mode = RTC_24H_CLK,
        .keep_rtc = true,
        .pdc_evt =
            {
                .pdc_evt_en = false,
                .pdc_evt_period = 0,
            },
        .motor_evt =
            {
                .motor_evt_en = false,
                .motor_evt_period = 0,
            },
    };
    hw_rtc_init(&config);
    hw_rtc_clock_enable();

    struct tm time_info
    {
    };
    auto const result = _rtc_localtime(time_second, &time_info, RTC_FULL_LEAP_YEAR_SUPPORT);
    Expects(result);
    rtc_time const _time = {
        .hour_mode = 0,
        .pm_flag = false,
        .hour = time_info.tm_hour,
        .minute = time_info.tm_min,
        .sec = time_info.tm_sec,
        .hsec = 0,
    };
    rtc_calendar const _calendar = {
        .year = time_info.tm_year + 1900,
        .month = time_info.tm_mon + 1,
        .mday = time_info.tm_yday,
        .wday = time_info.tm_wday + 1,
    };
    hw_rtc_set_time_clndr(&_time, &_calendar);
}