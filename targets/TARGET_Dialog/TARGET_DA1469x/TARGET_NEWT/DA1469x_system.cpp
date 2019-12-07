#include "DA1469xAB.h"
#include "system_DA1469x.h"
#include "mbed_toolchain.h"
#include "DA1469x_power_domain.h"
#include "DA1469x_power_domain_controller.h"
#include "DA1469x_configurable_MAC.h"
#include "DA1469x_otp.h"
#include "DA1469x_power_rails.h"
#include "DA1469x_clock.h"
#include "mbed_boot.h"
#include "DA1469x_system.h"
MBED_USED uint32_t SystemCoreClock = 32000000;

// #define PMU_ALL_SLEEP_MASK (CRG_TOP_PMU_CTRL_REG_TIM_SLEEP_Msk | CRG_TOP_PMU_CTRL_REG_PERIPH_SLEEP_Msk | CRG_TOP_PMU_CTRL_REG_COM_SLEEP_Msk | CRG_TOP_PMU_CTRL_REG_RADIO_SLEEP_Msk)
#define SYS_ALL_IS_DOWN_MASK (CRG_TOP_SYS_STAT_REG_TIM_IS_DOWN_Msk | CRG_TOP_SYS_STAT_REG_PER_IS_DOWN_Msk | CRG_TOP_SYS_STAT_REG_COM_IS_DOWN_Msk | CRG_TOP_SYS_STAT_REG_RAD_IS_DOWN_Msk)

namespace
{
void enable_fpu()
{
    SCB->CPACR |= (3UL << 20) | (3UL << 22);
    __DSB();
    __ISB();
}
} // namespace
namespace DA1469x
{
namespace system
{
power_domain_controller::Index combo_index;
void enable_cache_retainability()
{
    CRG_TOP->PMU_CTRL_REG |= CRG_TOP_PMU_CTRL_REG_RETAIN_CACHE_Msk;
}
} // namespace system
} // namespace DA1469x

void SystemInit(void)
{
    enable_fpu();
    DA1469x::power_domain::reset();
    DA1469x::power_domain::power_up(DA1469x::power_domain::Power_domain::system);
    DA1469x::power_domain_controller::reset();
    DA1469x::configurable_MAC::reset();
    NVIC_DisableIRQ(PDC_IRQn);
    NVIC_ClearPendingIRQ(PDC_IRQn);
    DA1469x::power_domain::power_up(DA1469x::power_domain::Power_domain::timers);
    {
        auto index =
            DA1469x::power_domain_controller::add(DA1469x::power_domain_controller::Trigger::timer_2, DA1469x::power_domain_controller::Master::cortex_M33, DA1469x::power_domain_controller::En::xtal);
        DA1469x::power_domain_controller::set(index);
        DA1469x::power_domain_controller::acknowledge(index);
    }
    {
        auto index =
            DA1469x::power_domain_controller::add(DA1469x::power_domain_controller::Trigger::combo, DA1469x::power_domain_controller::Master::cortex_M33, DA1469x::power_domain_controller::En::xtal);
        DA1469x::power_domain_controller::set(index);
        DA1469x::power_domain_controller::acknowledge(index);
        DA1469x::system::combo_index = index;
    }
    // if power down ,debugger will fal
    // DA1469x::power_domain::power_down(DA1469x::power_domain::Power_domain::system);
    DA1469x::system::enable_cache_retainability();
    DA1469x::otp::initialize();
    DA1469x::power_rails::initialize();
    DA1469x::power_rails::enable_dcdc();

    // /* Latch all pins. We will unlatch them when initialized to do something. */
    CRG_TOP->P0_RESET_PAD_LATCH_REG = CRG_TOP_P0_PAD_LATCH_REG_P0_LATCH_EN_Msk;
    CRG_TOP->P1_RESET_PAD_LATCH_REG = CRG_TOP_P1_PAD_LATCH_REG_P1_LATCH_EN_Msk;

    // hal_system_init
    // RESET_STAT_REG

    // stop watchdog
    GPREG->SET_FREEZE_REG = GPREG_SET_FREEZE_REG_FRZ_SYS_WDOG_Msk;
}

void mbed_sdk_init(void)
{
    DA1469x::clock::initialize();
}