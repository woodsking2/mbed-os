#include "DA1469x_HCI_driver.h"
#include "DA1469x_HCI_transport_driver.h"
#include "hci_defs.h"
ble::vendor::DA1469x::DA1469x_HCI_driver::DA1469x_HCI_driver(cordio::CordioHCITransportDriver &transport_driver
                                                             /* specific constructor arguments*/
                                                             )
    : cordio::CordioHCIDriver(transport_driver)
{
}

ble::vendor::DA1469x::DA1469x_HCI_driver::~DA1469x_HCI_driver()
{
}

void ble::vendor::DA1469x::DA1469x_HCI_driver::do_initialize()
{
}

void ble::vendor::DA1469x::DA1469x_HCI_driver::do_terminate()
{
}

ble::vendor::cordio::buf_pool_desc_t ble::vendor::DA1469x::DA1469x_HCI_driver::get_buffer_pool_description()
{
    return get_default_buffer_pool_description();
}
// void ble::vendor::DA1469x::DA1469x_HCI_driver::start_reset_sequence()
// {
// }

// void ble::vendor::DA1469x::DA1469x_HCI_driver::handle_reset_sequence(uint8_t *pMsg)
// {
// }

ble::vendor::cordio::CordioHCIDriver &ble_cordio_get_hci_driver()
{
    static ble::vendor::DA1469x::DA1469x_HCI_transport_driver transport_driver;

    static ble::vendor::DA1469x::DA1469x_HCI_driver hci_driver(transport_driver /* other hci driver parameters */
    );

    return hci_driver;
}