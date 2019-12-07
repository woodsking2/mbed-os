#include "da1469x_power_domain.h"
#include <cstdint>
#include "DA1469xAB.h"
#include "gsl/gsl"

using namespace gsl;
namespace
{
bool check(DA1469x::power_domain::Power_domain power_domain)
{
    switch (power_domain)
    {
    case DA1469x::power_domain::Power_domain::aon:
    case DA1469x::power_domain::Power_domain::memory:
    case DA1469x::power_domain::Power_domain::synth:
        return false;
    case DA1469x::power_domain::Power_domain::system:
    case DA1469x::power_domain::Power_domain::communication:
    case DA1469x::power_domain::Power_domain::timers:
    case DA1469x::power_domain::Power_domain::peripherals:
    case DA1469x::power_domain::Power_domain::radio:
        return true;
    }
    return false;
}
uint8_t get_bit_pos(DA1469x::power_domain::Power_domain power_domain)
{
    switch (power_domain)
    {
    case DA1469x::power_domain::Power_domain::system:
        return CRG_TOP_PMU_CTRL_REG_SYS_SLEEP_Pos;
    case DA1469x::power_domain::Power_domain::communication:
        return CRG_TOP_PMU_CTRL_REG_COM_SLEEP_Pos;
    case DA1469x::power_domain::Power_domain::timers:
        return CRG_TOP_PMU_CTRL_REG_TIM_SLEEP_Pos;
    case DA1469x::power_domain::Power_domain::peripherals:
        return CRG_TOP_PMU_CTRL_REG_RADIO_SLEEP_Pos;
    case DA1469x::power_domain::Power_domain::radio:
        return CRG_TOP_PMU_CTRL_REG_PERIPH_SLEEP_Pos;
    case DA1469x::power_domain::Power_domain::aon:
    case DA1469x::power_domain::Power_domain::memory:
    case DA1469x::power_domain::Power_domain::synth:
        // nothing
        break;
    }
    Expects(false);
    return 0;
}
} // namespace
void DA1469x::power_domain::reset()
{
    CRG_TOP->PMU_CTRL_REG = CRG_TOP_PMU_CTRL_REG_TIM_SLEEP_Msk | CRG_TOP_PMU_CTRL_REG_PERIPH_SLEEP_Msk | CRG_TOP_PMU_CTRL_REG_COM_SLEEP_Msk | CRG_TOP_PMU_CTRL_REG_RADIO_SLEEP_Msk;
}
void DA1469x::power_domain::power_up(Power_domain power_domain)
{
    Expects(check(power_domain));
    auto bit_pos = get_bit_pos(power_domain);
    CRG_TOP->PMU_CTRL_REG &= ~(1u << bit_pos);
}
void DA1469x::power_domain::power_down(Power_domain power_domain)
{
    Expects(check(power_domain));
    auto bit_pos = get_bit_pos(power_domain);
    CRG_TOP->PMU_CTRL_REG |= (1u << bit_pos);
}

namespace newt
{
struct da1469x_pd_desc
{
    uint8_t pmu_sleep_bit;
    uint8_t stat_down_bit; /* up is +1 */
};

static const struct da1469x_pd_desc g_da1469x_pd_desc[] = {
    [MCU_PD_DOMAIN_PER] = {CRG_TOP_PMU_CTRL_REG_PERIPH_SLEEP_Pos, CRG_TOP_SYS_STAT_REG_PER_IS_DOWN_Pos},
    [MCU_PD_DOMAIN_RAD] = {CRG_TOP_PMU_CTRL_REG_RADIO_SLEEP_Pos, CRG_TOP_SYS_STAT_REG_RAD_IS_DOWN_Pos},
    [MCU_PD_DOMAIN_TIM] = {CRG_TOP_PMU_CTRL_REG_TIM_SLEEP_Pos, CRG_TOP_SYS_STAT_REG_TIM_IS_DOWN_Pos},
    [MCU_PD_DOMAIN_COM] = {CRG_TOP_PMU_CTRL_REG_COM_SLEEP_Pos, CRG_TOP_SYS_STAT_REG_COM_IS_DOWN_Pos},
};

static uint8_t g_da1469x_pd_refcnt[ARRAY_SIZE(g_da1469x_pd_desc)];

int da1469x_pd_acquire(uint8_t pd)
{
    uint8_t *refcnt;
    uint32_t primask;
    uint32_t bitmask;
    int ret = 0;

    assert(pd < ARRAY_SIZE(g_da1469x_pd_desc));
    refcnt = &g_da1469x_pd_refcnt[pd];

    __HAL_DISABLE_INTERRUPTS(primask);

    assert(*refcnt < UINT8_MAX);
    if ((*refcnt)++ == 0)
    {
        bitmask = 1 << g_da1469x_pd_desc[pd].pmu_sleep_bit;
        CRG_TOP->PMU_CTRL_REG &= ~bitmask;

        bitmask = 1 << (g_da1469x_pd_desc[pd].stat_down_bit + 1);
        while ((CRG_TOP->SYS_STAT_REG & bitmask) == 0)
            ;

        ret = 1;
    }

    __HAL_ENABLE_INTERRUPTS(primask);

    return ret;
}

int da1469x_pd_release(uint8_t pd)
{
    uint8_t *refcnt;
    uint32_t primask;
    uint32_t bitmask;
    int ret = 0;

    assert(pd < MCU_PD_DOMAIN_COUNT);
    refcnt = &g_da1469x_pd_refcnt[pd];

    __HAL_DISABLE_INTERRUPTS(primask);

    assert(*refcnt > 0);
    if (--(*refcnt) == 0)
    {
        bitmask = 1 << g_da1469x_pd_desc[pd].pmu_sleep_bit;
        CRG_TOP->PMU_CTRL_REG |= bitmask;

        bitmask = 1 << g_da1469x_pd_desc[pd].stat_down_bit;
        while ((CRG_TOP->SYS_STAT_REG & bitmask) == 0)
            ;

        ret = 1;
    }

    __HAL_ENABLE_INTERRUPTS(primask);

    return ret;
}

int da1469x_pd_release_nowait(uint8_t pd)
{
    uint8_t *refcnt;
    uint32_t primask;
    uint32_t bitmask;
    int ret = 0;

    assert(pd < MCU_PD_DOMAIN_COUNT);
    refcnt = &g_da1469x_pd_refcnt[pd];

    __HAL_DISABLE_INTERRUPTS(primask);

    assert(*refcnt > 0);
    if (--(*refcnt) == 0)
    {
        bitmask = 1 << g_da1469x_pd_desc[pd].pmu_sleep_bit;
        CRG_TOP->PMU_CTRL_REG |= bitmask;

        ret = 1;
    }

    __HAL_ENABLE_INTERRUPTS(primask);

    return ret;
}
}