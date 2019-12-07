#include "DA1469x_serial.h"
#include "DA1469x_gpio.h"
#include "newt_sdks.h"
#include "DA1469x_power_domain.h"
#include "newt_sdks.h"
#include "gsl/gsl"
namespace
{
// auto stdio_IRQn = UART_IRQn;
// static void stdio_isr(void)
// {
//     // da1469x_uart_common_isr(&da1469x_uart_0);
// }
struct da1469x_uart_baudrate
{
    uint32_t baudrate;
    uint32_t cfg; /* DLH=cfg[23:16] DLL=cfg[15:8] DLF=cfg[7:0] */
};

const struct da1469x_uart_baudrate da1469x_uart_baudrates[] = {
    {1000000, 0x00000200}, {500000, 0x00000400}, {230400, 0x0000080b}, {115200, 0x00001106}, {57600, 0x0000220c}, {38400, 0x00003401},
    {28800, 0x00004507},   {19200, 0x00006803},  {14400, 0x00008a0e},  {9600, 0x0000d005},   {4800, 0x0001a00b},
};
uint32_t da1469x_uart_find_baudrate_cfg(uint32_t baudrate)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(da1469x_uart_baudrates); i++)
    {
        if (da1469x_uart_baudrates[i].baudrate == baudrate)
        {
            return da1469x_uart_baudrates[i].cfg;
        }
    }

    return 0;
}
void serial_config(serial_t *obj, int32_t baudrate, int data_bits, SerialParity parity, int stop_bits)
{
    CRG_COM->SET_CLK_COM_REG = CRG_COM_SET_CLK_COM_REG_UART_ENABLE_Msk;
    auto baudrate_cfg = da1469x_uart_find_baudrate_cfg(baudrate);
    Ensures(baudrate_cfg);
    UART->UART_LCR_REG |= UART_UART_LCR_REG_UART_DLAB_Msk;
    UART->UART_IER_DLH_REG = (baudrate_cfg >> 16) & 0xff;
    UART->UART_RBR_THR_DLL_REG = (baudrate_cfg >> 8) & 0xff;
    UART->UART_DLF_REG = baudrate_cfg & 0xff;
    UART->UART_LCR_REG &= ~UART_UART_LCR_REG_UART_DLAB_Msk;
    auto reg = 0;
    reg |= (stop_bits - 1) << UART_UART_LCR_REG_UART_STOP_Pos;
    reg |= (data_bits - 5) << UART_UART_LCR_REG_UART_DLS_Pos;
    UART->UART_LCR_REG = reg;

    /* Enable hardware FIFO */
    UART->UART_SFE_REG = UART_UART_SFE_REG_UART_SHADOW_FIFO_ENABLE_Msk;
    UART->UART_SRT_REG = 0;
    UART->UART_STET_REG = 0;

    UART->UART_IER_DLH_REG |= UART_UART_IER_DLH_REG_ERBFI_DLH0_Msk;
}

} // namespace
int stdio_uart_inited;
serial_t stdio_uart;

void serial_init(serial_t *obj, PinName tx, PinName rx)
{
    mcu_gpio_set_pin_function(tx, MCU_GPIO_MODE_OUTPUT, MCU_GPIO_FUNC_UART_TX);
    mcu_gpio_set_pin_function(rx, MCU_GPIO_MODE_INPUT, MCU_GPIO_FUNC_UART_RX);
    da1469x_pd_acquire(MCU_PD_DOMAIN_COM);
    serial_config(obj, 115200, 8, ParityNone, 1);
    // NVIC_DisableIRQ(stdio_IRQn);
    // NVIC_SetPriority(stdio_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    // NVIC_SetVector(stdio_IRQn, (uint32_t)stdio_isr);
}
void serial_free(serial_t *obj)
{
    UART->UART_IER_DLH_REG &= ~(UART_UART_IER_DLH_REG_PTIME_DLH7_Msk | UART_UART_IER_DLH_REG_ETBEI_DLH1_Msk);
    UART->UART_IER_DLH_REG &= ~UART_UART_IER_DLH_REG_ERBFI_DLH0_Msk;
    CRG_COM->RESET_CLK_COM_REG = CRG_COM_SET_CLK_COM_REG_UART_ENABLE_Msk;
}
void serial_baud(serial_t *obj, int baudrate)
{
    serial_config(obj, baudrate, 8, ParityNone, 1);
}
// void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
// {
// }
// void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
// {
// }
// void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
// {
// }
int serial_getc(serial_t *obj)
{
    while (!serial_readable(obj))
    {
    }
    return UART->UART_RBR_THR_DLL_REG;
}
void serial_putc(serial_t *obj, int c)
{
    while (!serial_writable(obj))
    {
    }
    UART->UART_RBR_THR_DLL_REG = c;
}
int serial_readable(serial_t *obj)
{
    return UART->UART_LSR_REG & UART_UART_LSR_REG_UART_DR_Msk;
}
int serial_writable(serial_t *obj)
{
    return UART->UART_USR_REG & UART_UART_USR_REG_UART_TFNF_Msk;
}
// void serial_clear(serial_t *obj)
// {
// }

// void serial_break_set(serial_t *obj)
// {
// }

// void serial_break_clear(serial_t *obj)
// {
// }

// void serial_pinout_tx(PinName tx)
// {
// }
