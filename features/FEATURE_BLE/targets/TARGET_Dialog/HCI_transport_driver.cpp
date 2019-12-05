#include "HCI_transport_driver.h"
#include "Application.h"
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
    auto const result = Application::get_instance<Configurable_MAC>().initialize();
    Ensures(result == Configurable_MAC::Result::success);
}

void ble::vendor::DA1469x::HCI_transport_driver::terminate()
{
}

uint16_t ble::vendor::DA1469x::HCI_transport_driver::write(uint8_t packet_type, uint16_t len, uint8_t *data)
{
    // debug("type:%d len:%d %02X %02X %02X\n", packet_type, len, data[0], data[1], data[2]);
    auto &instance = Application::get_instance<Configurable_MAC>();
    auto const type_result = instance.write(make_span(&packet_type, 1));
    Ensures(type_result == Configurable_MAC::Result::success);
    auto const data_result = instance.write(make_span(data, len));
    Ensures(data_result == Configurable_MAC::Result::success);
    return len;
}
