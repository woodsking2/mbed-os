#include "serial_api.h"
#include "memory"
#include "gsl/gsl"
#include "serial_instance.h"
using namespace std;
using namespace gsl;
serial_t stdio_uart;
int stdio_uart_inited = 0;

/** Initialize the serial peripheral. It sets the default parameters for serial
 *  peripheral, and configures its specifieds pins.
 *
 * @param obj The serial object
 * @param tx  The TX pin name
 * @param rx  The RX pin name
 */
void serial_init(serial_t *obj, PinName tx, PinName rx)
{
    Serial_instance *instance = Serial_instance::acquire();
    obj->instance = instance;
    instance->initialize(tx, rx);
    if (obj == &stdio_uart)
    {
        stdio_uart_inited = 1;
    }
}

/** Release the serial peripheral, not currently invoked. It requires further
 *  resource management.
 *
 * @param obj The serial object
 */
void serial_free(serial_t *obj)
{
    Expects(obj);
    Serial_instance *instance = (Serial_instance *)(obj->instance);
    instance->release();
    delete instance;
}

/** Configure the baud rate
 *
 * @param obj      The serial object
 * @param baudrate The baud rate to be configured
 */
void serial_baud(serial_t *obj, int baudrate)
{
    Expects(obj);
    Serial_instance *instance = (Serial_instance *)(obj->instance);
    instance->set_baudrate(baudrate);
}

/** Configure the format. Set the number of bits, parity and the number of stop bits
 *
 * @param obj       The serial object
 * @param data_bits The number of data bits
 * @param parity    The parity
 * @param stop_bits The number of stop bits
 */
void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
{
    Expects(obj);
    Serial_instance *instance = (Serial_instance *)(obj->instance);
    instance->set_format(data_bits, parity, stop_bits);
}

/** The serial interrupt handler registration
 *
 * @param obj     The serial object
 * @param handler The interrupt handler which will be invoked when the interrupt fires
 * @param id      The SerialBase object
 */
void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
{
    Expects(false);
    // Expects(obj);
    // Serial_instance *instance = (Serial_instance *)(obj->instance);
}

/** Configure serial interrupt. This function is used for word-approach
 *
 * @param obj    The serial object
 * @param irq    The serial IRQ type (RX or TX)
 * @param enable Set to non-zero to enable events, or zero to disable them
 */
void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
{
    Expects(false);
    // Expects(obj);
    // Serial_instance *instance = (Serial_instance *)(obj->instance);
}
/** Get character. This is a blocking call, waiting for a character
 *
 * @param obj The serial object
 */
int serial_getc(serial_t *obj)
{
    Expects(false);
    // Expects(obj);
    // Serial_instance *instance = (Serial_instance *)(obj->instance);
}

/** Send a character. This is a blocking call, waiting for a peripheral to be available
 *  for writing
 *
 * @param obj The serial object
 * @param c   The character to be sent
 */
void serial_putc(serial_t *obj, int c)
{
    Expects(obj);
    Serial_instance *instance = (Serial_instance *)(obj->instance);
    instance->write(c);
}

/** Check if the serial peripheral is readable
 *
 * @param obj The serial object
 * @return Non-zero value if a character can be read, 0 if nothing to read
 */
int serial_readable(serial_t *obj)
{
#if 1
#warning not finish
    return 0;
#else
    Expects(obj);
    Serial_instance *instance = (Serial_instance *)(obj->instance);
#endif
}

/** Check if the serial peripheral is writable
 *
 * @param obj The serial object
 * @return Non-zero value if a character can be written, 0 otherwise.
 */
int serial_writable(serial_t *obj)
{
    Expects(obj);
    Serial_instance *instance = (Serial_instance *)(obj->instance);
    return instance->write_able();
}

/** Clear the serial peripheral
 *
 * @param obj The serial object
 */
void serial_clear(serial_t *obj)
{
    // Expects(obj);
    // Serial_instance *instance = (Serial_instance *)(obj->instance);
}

/** Set the break
 *
 * @param obj The serial object
 */
void serial_break_set(serial_t *obj)
{
    Expects(false);
    // Expects(obj);
    // Serial_instance *instance = (Serial_instance *)(obj->instance);
}

/** Clear the break
 *
 * @param obj The serial object
 */
void serial_break_clear(serial_t *obj)
{
    Expects(false);
    // Expects(obj);
    // Serial_instance *instance = (Serial_instance *)(obj->instance);
}

/** Configure the TX pin for UART function.
 *
 * @param tx The pin name used for TX
 */
void serial_pinout_tx(PinName tx)
{
    Expects(false);
    // Expects(obj);
    // Serial_instance *instance = (Serial_instance *)(obj->instance);
}

#if DEVICE_SERIAL_FC
#error not implement
/** Configure the serial for the flow control. It sets flow control in the hardware
 *  if a serial peripheral supports it, otherwise software emulation is used.
 *
 * @param obj    The serial object
 * @param type   The type of the flow control. Look at the available FlowControl types.
 * @param rxflow The TX pin name
 * @param txflow The RX pin name
 */
void serial_set_flow_control(serial_t *obj, FlowControl type, PinName rxflow, PinName txflow)
{
}
#endif

/** Get the pins that support Serial TX
 *
 * Return a PinMap array of pins that support Serial TX. The
 * array is terminated with {NC, NC, 0}.
 *
 * @return PinMap array
 */
const PinMap *serial_tx_pinmap(void)
{
}

/** Get the pins that support Serial RX
 *
 * Return a PinMap array of pins that support Serial RX. The
 * array is terminated with {NC, NC, 0}.
 *
 * @return PinMap array
 */
const PinMap *serial_rx_pinmap(void)
{
}

#if DEVICE_SERIAL_FC
#error not implement
/** Get the pins that support Serial CTS
 *
 * Return a PinMap array of pins that support Serial CTS. The
 * array is terminated with {NC, NC, 0}.
 *
 * @return PinMap array
 */
const PinMap *serial_cts_pinmap(void)
{
}

/** Get the pins that support Serial RTS
 *
 * Return a PinMap array of pins that support Serial RTS. The
 * array is terminated with {NC, NC, 0}.
 *
 * @return PinMap array
 */
const PinMap *serial_rts_pinmap(void)
{
}
#endif

#if DEVICE_SERIAL_ASYNCH
#error not implement

/**@}*/

/**
 * \defgroup hal_AsynchSerial Asynchronous Serial Hardware Abstraction Layer
 * @{
 */

/** Begin asynchronous TX transfer. The used buffer is specified in the serial object,
 *  tx_buff
 *
 * @param obj       The serial object
 * @param tx        The transmit buffer
 * @param tx_length The number of bytes to transmit
 * @param tx_width  Deprecated argument
 * @param handler   The serial handler
 * @param event     The logical OR of events to be registered
 * @param hint      A suggestion for how to use DMA with this transfer
 * @return Returns number of data transfered, otherwise returns 0
 */
int serial_tx_asynch(serial_t *obj, const void *tx, size_t tx_length, uint8_t tx_width, uint32_t handler, uint32_t event, DMAUsage hint)
{
}

/** Begin asynchronous RX transfer (enable interrupt for data collecting)
 *  The used buffer is specified in the serial object - rx_buff
 *
 * @param obj        The serial object
 * @param rx         The receive buffer
 * @param rx_length  The number of bytes to receive
 * @param rx_width   Deprecated argument
 * @param handler    The serial handler
 * @param event      The logical OR of events to be registered
 * @param handler    The serial handler
 * @param char_match A character in range 0-254 to be matched
 * @param hint       A suggestion for how to use DMA with this transfer
 */
void serial_rx_asynch(serial_t *obj, void *rx, size_t rx_length, uint8_t rx_width, uint32_t handler, uint32_t event, uint8_t char_match, DMAUsage hint)
{
}

/** Attempts to determine if the serial peripheral is already in use for TX
 *
 * @param obj The serial object
 * @return Non-zero if the RX transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
}

/** Attempts to determine if the serial peripheral is already in use for RX
 *
 * @param obj The serial object
 * @return Non-zero if the RX transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
}

/** The asynchronous TX and RX handler.
 *
 * @param obj The serial object
 * @return Returns event flags if an RX transfer termination condition was met; otherwise returns 0
 */
int serial_irq_handler_asynch(serial_t *obj)
{
}

/** Abort the ongoing TX transaction. It disables the enabled interupt for TX and
 *  flushes the TX hardware buffer if TX FIFO is used
 *
 * @param obj The serial object
 */
void serial_tx_abort_asynch(serial_t *obj)
{
}

/** Abort the ongoing RX transaction. It disables the enabled interrupt for RX and
 *  flushes the RX hardware buffer if RX FIFO is used
 *
 * @param obj The serial object
 */
void serial_rx_abort_asynch(serial_t *obj)
{
}

/**@}*/

#endif
