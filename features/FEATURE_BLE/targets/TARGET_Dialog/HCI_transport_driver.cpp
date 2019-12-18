#include "HCI_transport_driver.h"
#include "configurable_MAC.h"
#include "mbed_debug.h"
using namespace gsl;
ble::vendor::DA1469x::HCI_transport_driver::HCI_transport_driver(/* specific constructor arguments*/)
{
}

ble::vendor::DA1469x::HCI_transport_driver::~HCI_transport_driver()
{
}

//初始化
void ble::vendor::DA1469x::HCI_transport_driver::initialize()
{
    Configurable_MAC::get_instance().initialize();
}

void ble::vendor::DA1469x::HCI_transport_driver::terminate()
{
}

uint16_t ble::vendor::DA1469x::HCI_transport_driver::write(uint8_t packet_type, uint16_t len, uint8_t *data)
{
    // debug("type:%d len:%d %02X %02X %02X\n", packet_type, len, data[0], data[1], data[2]);
    auto &instance = Configurable_MAC::get_instance();
    instance.write(make_span(&packet_type, 1));
    instance.write(make_span(data, len));
    return len;
}
