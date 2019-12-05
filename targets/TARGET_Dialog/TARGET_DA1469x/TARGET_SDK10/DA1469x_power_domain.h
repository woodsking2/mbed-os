#pragma once
#include "newt_sdks.h"
namespace DA1469x
{
namespace power_domain
{
enum class Power_domain
{
    aon,
    system,
    communication,
    memory,
    timers,
    peripherals,
    radio,
    synth,
};
void reset();
void power_up(Power_domain);
void power_down(Power_domain);

} // namespace power_domain
} // namespace DA1469x

namespace newt
{
    /* Available (controllable) power domains */
    #define MCU_PD_DOMAIN_PER           0
    #define MCU_PD_DOMAIN_RAD           1
    #define MCU_PD_DOMAIN_TIM           2
    #define MCU_PD_DOMAIN_COM           3

    #define MCU_PD_DOMAIN_COUNT         4

    int da1469x_pd_acquire(uint8_t pd);
    int da1469x_pd_release(uint8_t pd);
    int da1469x_pd_release_nowait(uint8_t pd);
}