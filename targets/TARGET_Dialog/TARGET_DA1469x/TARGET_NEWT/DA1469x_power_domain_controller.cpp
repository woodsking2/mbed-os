#include "DA1469xAB.h"
#include "gsl/gsl"
#include "DA1469x_power_domain_controller.h"
#define MCU_PDC_CTRL_REGS(_i) ((__IO uint32_t *)&PDC->PDC_CTRL0_REG)[_i]
namespace
{
constexpr auto look_up_table_size = 16;
constexpr auto look_up_table_unused = 0;
volatile uint32_t &get_regs(DA1469x::power_domain_controller::Index index)
{
    return reinterpret_cast<volatile uint32_t *>(&PDC->PDC_CTRL0_REG)[index];
}
uint32_t get_en_value(DA1469x::power_domain_controller::En en)
{
    auto value = 1 << static_cast<uint32_t>(en);
    return value << PDC_PDC_CTRL0_REG_EN_XTAL_Pos;
}
uint32_t get_master_value(DA1469x::power_domain_controller::Master master)
{
    auto value = static_cast<uint32_t>(master) + 1;
    return value << PDC_PDC_CTRL0_REG_PDC_MASTER_Pos;
}
uint32_t get_select_value(DA1469x::power_domain_controller::Trigger trigger)
{
    auto value = 2; // static_cast<uint32_t>(trigger);
    return value << PDC_PDC_CTRL0_REG_TRIG_SELECT_Pos;
}
uint32_t get_trigger_value(DA1469x::power_domain_controller::Trigger trigger)
{
    auto value = static_cast<uint32_t>(trigger);
    return value << PDC_PDC_CTRL0_REG_TRIG_ID_Pos;
}
void set(DA1469x::power_domain_controller::Index index, DA1469x::power_domain_controller::Trigger trigger, DA1469x::power_domain_controller::Master master, DA1469x::power_domain_controller::En en)
{
    uint32_t select_value = get_select_value(trigger);
    uint32_t trigger_value = get_trigger_value(trigger);
    uint32_t master_value = get_master_value(master);
    uint32_t en_value = get_en_value(en);
    // static_cast<uint32_t>(en) << PDC_PDC_CTRL0_REG_EN_XTAL_Pos;
    get_regs(index) = select_value | trigger_value | master_value | en_value;
}
} // namespace
namespace DA1469x
{
namespace power_domain_controller
{
void reset()
{
    for (auto index = 0; index < look_up_table_size; index++)
    {
        get_regs(index) = look_up_table_unused;
        acknowledge(index);
    }
}
void acknowledge(Index index)
{
    Expects(index < look_up_table_size);
    PDC->PDC_ACKNOWLEDGE_REG = index;
}
void set(Index index)
{
    Expects(index < look_up_table_size);
    PDC->PDC_SET_PENDING_REG = index;
}
Index add(Trigger trigger, Master master, En en)
{
    for (auto index = 0; index < look_up_table_size; index++)
    {
        if (!(get_regs(index) & PDC_PDC_CTRL0_REG_PDC_MASTER_Msk))
        {

            return index;
        }
    }
    Ensures(false);
    return -1;
}
} // namespace power_domain_controller
} // namespace DA1469x

namespace newt
{

int da1469x_pdc_find(int trigger, int master, uint8_t en)
{
    int idx;
    uint32_t mask;
    uint32_t value;

    mask = en << PDC_PDC_CTRL0_REG_EN_XTAL_Pos;
    value = en << PDC_PDC_CTRL0_REG_EN_XTAL_Pos;
    if (trigger >= 0)
    {
        mask |= PDC_PDC_CTRL0_REG_TRIG_SELECT_Msk | PDC_PDC_CTRL0_REG_TRIG_ID_Msk;
        value |= ((trigger >> 5) << PDC_PDC_CTRL0_REG_TRIG_SELECT_Pos) | ((trigger & 0x1f) << PDC_PDC_CTRL0_REG_TRIG_ID_Pos);
    }
    if (master > 0)
    {
        mask |= PDC_PDC_CTRL0_REG_PDC_MASTER_Msk;
        value |= master << PDC_PDC_CTRL0_REG_PDC_MASTER_Pos;
    }
    assert(mask);

    for (idx = 0; idx < MCU_PDC_CTRL_REGS_COUNT; idx++)
    {
        if ((MCU_PDC_CTRL_REGS(idx) & mask) == value)
        {
            return idx;
        }
    }

    return SYS_ENOENT;
}
int da1469x_pdc_add(uint8_t source, uint8_t master, uint8_t en)
{
    int idx;
    uint8_t select;

    select = source >> 5;
    source &= 0x1f;

    for (idx = 0; idx < MCU_PDC_CTRL_REGS_COUNT; idx++)
    {
        if (!(MCU_PDC_CTRL_REGS(idx) & PDC_PDC_CTRL0_REG_PDC_MASTER_Msk))
        {
            MCU_PDC_CTRL_REGS(idx) =
                (select << PDC_PDC_CTRL0_REG_TRIG_SELECT_Pos) | (source << PDC_PDC_CTRL0_REG_TRIG_ID_Pos) | (master << PDC_PDC_CTRL0_REG_PDC_MASTER_Pos) | (en << PDC_PDC_CTRL0_REG_EN_XTAL_Pos);
            return idx;
        }
    }

    assert(0);

    return SYS_ENOENT;
}
} // namespace newt