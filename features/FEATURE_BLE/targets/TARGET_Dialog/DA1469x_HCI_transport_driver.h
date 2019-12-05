#pragma once
#include "CordioHCITransportDriver.h"

namespace ble
{
namespace vendor
{
namespace DA1469x
{

/**
 * @brief on_data_received.接受
 * 
 */
class DA1469x_HCI_transport_driver : public cordio::CordioHCITransportDriver
{
  
  public:
    DA1469x_HCI_transport_driver(/* specific constructor arguments*/);

    virtual ~DA1469x_HCI_transport_driver();

    virtual void initialize();

    virtual void terminate();

    virtual uint16_t write(uint8_t packet_type, uint16_t len, uint8_t *data);

  private:
    // private driver declarations
};

} // namespace target_name
} // namespace vendor
} // namespace ble