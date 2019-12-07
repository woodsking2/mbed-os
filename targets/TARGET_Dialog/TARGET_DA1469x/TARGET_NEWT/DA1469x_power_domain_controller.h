#pragma once
#include "newt_sdks.h"
namespace DA1469x
{
namespace power_domain_controller
{
enum class Trigger
{
    timer,
    timer_2,
    timer_3,
    timer_4,
    rtc_alarm,
    rtc_timer,
    configurable_MAC_timer,
    motor_controller,
    xtal_32m_ready,
    rfdiag,
    combo, // CMAC2SYS_IRQ O VBUS Present IRQ OR JTAG present OR Debounced IO
    snc,
    reserved_0,
    reserved_1,
    reserved_2,
    software,
};
enum class Master
{
    cortex_M33,
    configurable_MAC,
    sensor_node_controller,
};
enum class En
{
    none,
    xtal,
    timer,
    peripherals,
    communication,
};
using Index = int;
void reset();
void acknowledge(Index index);
Index add(Trigger trigger, Master master, En en);
void set(Index index);
} // namespace power_domain_controller
} // namespace DA1469x

namespace newt
{
/* PDC trigger can be either a GPIO number or one of following values */
#define MCU_PDC_TRIGGER_TIMER (0x40 | 0)
#define MCU_PDC_TRIGGER_TIMER2 (0x40 | 1)
#define MCU_PDC_TRIGGER_TIMER3 (0x40 | 2)
#define MCU_PDC_TRIGGER_TIMER4 (0x40 | 3)
#define MCU_PDC_TRIGGER_RTC_ALARM (0x40 | 4)
#define MCU_PDC_TRIGGER_RTC_TIMER (0x40 | 5)
#define MCU_PDC_TRIGGER_MAC_TIMER (0x40 | 6)
#define MCU_PDC_TRIGGER_MOTOR_CONTROLLER (0x40 | 7)
#define MCU_PDC_TRIGGER_XTAL32M_READY (0x40 | 8)
#define MCU_PDC_TRIGGER_RFDIAG (0x40 | 9)
#define MCU_PDC_TRIGGER_COMBO (0x40 | 10) /* VBUS, IO, JTAG, CMAC2SYS */
#define MCU_PDC_TRIGGER_SNC (0x40 | 11)
#define MCU_PDC_TRIGGER_SW_TRIGGER (0x40 | 15)

/* PDC master can be either of following values */
#define MCU_PDC_MASTER_M33 1
#define MCU_PDC_MASTER_CMAC 2
#define MCU_PDC_MASTER_SNC 3

/* PDC enable bitmask can consist of following values */
#define MCU_PDC_EN_NONE 0x00
#define MCU_PDC_EN_XTAL 0x01
#define MCU_PDC_EN_PD_TMR 0x02
#define MCU_PDC_EN_PD_PER 0x04
#define MCU_PDC_EN_PD_COM 0x08

#define MCU_PDC_CTRL_REGS(_i)       (((__IO uint32_t*)&PDC->PDC_CTRL0_REG)[_i])
#define MCU_PDC_CTRL_REGS_COUNT     16

int da1469x_pdc_find(int trigger, int master, uint8_t en);
int da1469x_pdc_add(uint8_t trigger, uint8_t master, uint8_t en);
static inline void da1469x_pdc_set(int idx)
{
    PDC->PDC_SET_PENDING_REG = idx;
}
static inline void da1469x_pdc_ack(int idx)
{
    PDC->PDC_ACKNOWLEDGE_REG = idx;
}

} // namespace newt
