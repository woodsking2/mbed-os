#include "da1469x_gpio.h"
#include "DA1469x_power_domain.h"
#include "newt_sdks.h"
#include "da1469x_retreg.h"
#include "da1469xab.h"
#include "gpio_api.h"
#include "mbed_debug.h"
#include "gsl/gsl"
using namespace gsl;

namespace newt
{

/* GPIO interrupts */
#define HAL_GPIO_MAX_IRQ (4)

#define GPIO_REG(name) ((__IO uint32_t *)(GPIO_BASE + offsetof(GPIO_Type, name)))
#define WAKEUP_REG(name) ((__IO uint32_t *)(WAKEUP_BASE + offsetof(WAKEUP_Type, name)))
#define CRG_TOP_REG(name) ((__IO uint32_t *)(CRG_TOP_BASE + offsetof(CRG_TOP_Type, name)))

#define GPIO_PORT(pin) (((unsigned)(pin)) >> 5)

#define GPIO_PIN_DATA_REG_ADDR(pin) (GPIO_REG(P0_DATA_REG) + GPIO_PORT(pin))
#define GPIO_PIN_DATA_REG(pin) *GPIO_PIN_DATA_REG_ADDR(pin)
#define GPIO_PIN_SET_DATA_REG_ADDR(pin) (GPIO_REG(P0_SET_DATA_REG) + GPIO_PORT(pin))
#define GPIO_PIN_SET_DATA_REG(pin) *GPIO_PIN_SET_DATA_REG_ADDR(pin)
#define GPIO_PIN_RESET_DATA_REG_ADDR(pin) (GPIO_REG(P0_RESET_DATA_REG) + GPIO_PORT(pin))
#define GPIO_PIN_RESET_DATA_REG(pin) *GPIO_PIN_RESET_DATA_REG_ADDR(pin)
#define GPIO_PIN_MODE_REG_ADDR(pin) (GPIO_REG(P0_00_MODE_REG) + (pin))
#define GPIO_PIN_MODE_REG(pin) *GPIO_PIN_MODE_REG_ADDR(pin)
#define GPIO_PIN_PADPWR_CTRL_REG_ADDR(pin) (GPIO_REG(P0_PADPWR_CTRL_REG) + GPIO_PORT(pin))
#define GPIO_PIN_PADPWR_CTRL_REG(pin) *GPIO_PIN_PADPWR_CTRL_REG_ADDR(pin)
#define GPIO_PIN_UNLATCH_ADDR(pin) (CRG_TOP_REG(P0_SET_PAD_LATCH_REG) + GPIO_PORT(pin) * 3)
#define GPIO_PIN_LATCH_ADDR(pin) (CRG_TOP_REG(P0_RESET_PAD_LATCH_REG) + GPIO_PORT(pin) * 3)
#define GPIO_PIN_BIT(pin) (1 << GPIO_PORT_PIN(pin))

// #define GPIO_PORT(pin) (((unsigned)(pin)) >> 5U)
#define GPIO_PORT_PIN(pin) (((unsigned)(pin)) & 31U)

#define WKUP_CTRL_REG_ADDR (WAKEUP_REG(WKUP_CTRL_REG))
#define WKUP_RESET_IRQ_REG_ADDR (WAKEUP_REG(WKUP_RESET_IRQ_REG))
#define WKUP_SELECT_PX_REG_ADDR(pin) (WAKEUP_REG(WKUP_SELECT_P0_REG) + GPIO_PORT(pin))
#define WKUP_SELECT_PX_REG(pin) *(WKUP_SELECT_PX_REG_ADDR(pin))
#define WKUP_POL_PX_REG_ADDR(pin) (WAKEUP_REG(WKUP_POL_P0_REG) + GPIO_PORT(pin))
#define WKUP_POL_PX_SET_FALLING(pin)                                                                                                                                                                   \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        *(WKUP_POL_PX_REG_ADDR(pin)) |= (1 << ((pin)&31));                                                                                                                                             \
    } while (0)
#define WKUP_POL_PX_SET_RISING(pin)                                                                                                                                                                    \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        *(WKUP_POL_PX_REG_ADDR(pin)) &= ~(1 << ((pin)&31));                                                                                                                                            \
    } while (0)
#define WKUP_STAT_PX_REG_ADDR(pin) (WAKEUP_REG(WKUP_STATUS_P0_REG) + GPIO_PORT(pin))
#define WKUP_STAT(pin) ((*(WKUP_STAT_PX_REG_ADDR(pin)) >> ((pin)&31)) & 1)
#define WKUP_CLEAR_PX_REG_ADDR(pin) (WAKEUP_REG(WKUP_CLEAR_P0_REG) + GPIO_PORT(pin))
#define WKUP_CLEAR_PX(pin)                                                                                                                                                                             \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        (*(WKUP_CLEAR_PX_REG_ADDR(pin)) = (1 << ((pin)&31)));                                                                                                                                          \
    } while (0)
#define WKUP_SEL_GPIO_PX_REG_ADDR(pin) (WAKEUP_REG(WKUP_SEL_GPIO_P0_REG) + GPIO_PORT(pin))
#define WKUP_SEL_GPIO_PX_REG(pin) *(WKUP_SEL_GPIO_PX_REG_ADDR(pin))

#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
static uint8_t g_mcu_gpio_retained_num;
static struct da1469x_retreg g_mcu_gpio_retained[MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM)];
#endif
static inline void mcu_gpio_unlatch_prepare(int pin)
{
    __HAL_ASSERT_CRITICAL();

    /* Acquire PD_COM if first pin will be unlatched */
    if ((CRG_TOP->P0_PAD_LATCH_REG | CRG_TOP->P1_PAD_LATCH_REG) == 0)
    {
        da1469x_pd_acquire(MCU_PD_DOMAIN_COM);
    }
}
#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
static void mcu_gpio_retained_add_port(uint32_t latch_val, volatile uint32_t *base_reg)
{
    struct da1469x_retreg *retreg;
    int pin;

    retreg = &g_mcu_gpio_retained[g_mcu_gpio_retained_num];

    while (latch_val)
    {
        assert(g_mcu_gpio_retained_num < MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM));

        pin = __builtin_ctz(latch_val);
        latch_val &= ~(1 << pin);

        da1469x_retreg_assign(retreg, &base_reg[pin]);

        g_mcu_gpio_retained_num++;
        retreg++;
    }
}
#endif
static void mcu_gpio_retained_refresh(void)
{
#if MYNEWT_VAL(MCU_GPIO_RETAINABLE_NUM) >= 0
    g_mcu_gpio_retained_num = 0;

    mcu_gpio_retained_add_port(CRG_TOP->P0_PAD_LATCH_REG, &GPIO->P0_00_MODE_REG);
    mcu_gpio_retained_add_port(CRG_TOP->P1_PAD_LATCH_REG, &GPIO->P1_00_MODE_REG);
#endif
}

static inline void mcu_gpio_unlatch(int pin)
{
    __HAL_ASSERT_CRITICAL();

    *GPIO_PIN_UNLATCH_ADDR(pin) = 1 << ((pin)&31);
    mcu_gpio_retained_refresh();
}
void mcu_gpio_set_pin_function(int pin, int mode, mcu_gpio_func func)
{
    uint32_t primask;

    __HAL_DISABLE_INTERRUPTS(primask);

    mcu_gpio_unlatch_prepare(pin);

    GPIO_PIN_MODE_REG(pin) = (func & GPIO_P0_00_MODE_REG_PID_Msk) | (mode & (GPIO_P0_00_MODE_REG_PUPD_Msk | GPIO_P0_00_MODE_REG_PPOD_Msk));

    mcu_gpio_unlatch(pin);

    __HAL_ENABLE_INTERRUPTS(primask);
}

} // namespace newt

void gpio_init(gpio_t *obj, PinName pin)
{
    obj->pin = pin;
    obj->direction = PIN_DIRECTION_UNSET;
}

void gpio_dir(gpio_t *obj, PinDirection direction)
{
    switch (direction)
    {
    case PinDirection::PIN_INPUT:
    {
        obj->direction = PIN_INPUT;
    }
    break;

    case PinDirection::PIN_OUTPUT:
    {
        obj->direction = PIN_OUTPUT;
        // debug("%d output\n", obj->pin);
        // debug("%p read %p, data[%p] %p\n", GPIO_PIN_MODE_REG_ADDR(obj->pin), GPIO_PIN_MODE_REG(obj->pin), GPIO_PIN_DATA_REG_ADDR(obj->pin), GPIO_PIN_DATA_REG(obj->pin));
        uint32_t primask;

        __HAL_DISABLE_INTERRUPTS(primask);

        mcu_gpio_unlatch_prepare(obj->pin);

        GPIO_PIN_MODE_REG(obj->pin) = MCU_GPIO_MODE_OUTPUT;
        if (obj->value)
        {
            GPIO_PIN_SET_DATA_REG(obj->pin) = GPIO_PIN_BIT(obj->pin);
        }
        else
        {
            GPIO_PIN_RESET_DATA_REG(obj->pin) = GPIO_PIN_BIT(obj->pin);
        }
        // GPIO_PIN_PADPWR_CTRL_REG(obj->pin) = GPIO_PIN_BIT(GPIO_PIN_BIT(obj->pin));
        mcu_gpio_unlatch(obj->pin);

        __HAL_ENABLE_INTERRUPTS(primask);
        // debug("%p write %p,power %p,data[%p] %p\n", GPIO_PIN_MODE_REG_ADDR(obj->pin), GPIO_PIN_MODE_REG(obj->pin), GPIO_PIN_PADPWR_CTRL_REG(obj->pin), GPIO_PIN_DATA_REG_ADDR(obj->pin),
        //       GPIO_PIN_DATA_REG(obj->pin));
    }
    break;
    }
}
void gpio_mode(gpio_t *obj, PinMode mode)
{
    if (obj->direction != PIN_INPUT)
    {
        return;
    }
    volatile uint32_t *px_xx_mod_reg = GPIO_PIN_MODE_REG_ADDR(obj->pin);
    uint32_t regval;
    uint32_t primask;

    switch (mode)
    {
    case PinMode::PullUp:
        regval = MCU_GPIO_FUNC_GPIO | MCU_GPIO_MODE_INPUT_PULLUP;
        break;
    case PinMode::PullDown:
        regval = MCU_GPIO_FUNC_GPIO | MCU_GPIO_MODE_INPUT_PULLDOWN;
        break;
    case PinMode::PullNone:
        regval = MCU_GPIO_FUNC_GPIO | MCU_GPIO_MODE_INPUT;
        break;
    }

    __HAL_DISABLE_INTERRUPTS(primask);

    mcu_gpio_unlatch_prepare(obj->pin);

    *px_xx_mod_reg = regval;

    mcu_gpio_unlatch(obj->pin);

    __HAL_ENABLE_INTERRUPTS(primask);
}
int gpio_is_connected(const gpio_t *obj)
{
    return obj->pin != (PinName)NC;
}
void gpio_write(gpio_t *obj, int val)
{
    obj->value = val;
    if (obj->direction != PIN_OUTPUT)
    {
        return;
    }
    // debug("%d write %d\n", obj->pin, val);
    // debug("%p set_data[%p] %p\n", GPIO_PIN_SET_DATA_REG_ADDR(obj->pin), GPIO_PIN_DATA_REG_ADDR(obj->pin), GPIO_PIN_DATA_REG(obj->pin));
    uint32_t primask;

    __HAL_DISABLE_INTERRUPTS(primask);

    mcu_gpio_unlatch_prepare(obj->pin);
    GPIO_PIN_MODE_REG(obj->pin) = MCU_GPIO_MODE_OUTPUT;
    if (val)
    {
        GPIO_PIN_SET_DATA_REG(obj->pin) = GPIO_PIN_BIT(obj->pin);
    }
    else
    {
        GPIO_PIN_RESET_DATA_REG(obj->pin) = GPIO_PIN_BIT(obj->pin);
    }
    mcu_gpio_unlatch(obj->pin);

    __HAL_ENABLE_INTERRUPTS(primask);
    // debug("%p set_data[%p] %p\n", GPIO_PIN_SET_DATA_REG_ADDR(obj->pin), GPIO_PIN_DATA_REG_ADDR(obj->pin), GPIO_PIN_DATA_REG(obj->pin));
}
int gpio_read(gpio_t *obj)
{
    auto value = (GPIO_PIN_DATA_REG(obj->pin) >> GPIO_PORT_PIN(obj->pin)) & 1;
    // debug("%d read %d\n", obj->pin, value);
    return value;
}