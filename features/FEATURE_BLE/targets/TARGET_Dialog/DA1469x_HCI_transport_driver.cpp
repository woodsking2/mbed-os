#include "DA1469x_HCI_transport_driver.h"
#include "DA1469x_configurable_MAC.h"
#include "mbed_debug.h"
ble::vendor::DA1469x::DA1469x_HCI_transport_driver::DA1469x_HCI_transport_driver(/* specific constructor arguments*/)
{
}

ble::vendor::DA1469x::DA1469x_HCI_transport_driver::~DA1469x_HCI_transport_driver()
{
}

//初始化
void ble::vendor::DA1469x::DA1469x_HCI_transport_driver::initialize()
{
    ::DA1469x::configurable_MAC::initialize();
}

void ble::vendor::DA1469x::DA1469x_HCI_transport_driver::terminate()
{
}

uint16_t ble::vendor::DA1469x::DA1469x_HCI_transport_driver::write(uint8_t packet_type, uint16_t len, uint8_t *data)
{
    // debug("type:%d len:%d %02X %02X %02X\n", packet_type, len, data[0], data[1], data[2]);
    ::DA1469x::configurable_MAC::write(&packet_type, 1);
    ::DA1469x::configurable_MAC::write(data, len);
    return len;
}
