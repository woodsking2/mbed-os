#pragma once
#include "CordioHCIDriver.h"

namespace ble
{
namespace vendor
{
namespace DA1469x
{

class HCI_driver : public cordio::CordioHCIDriver
{
  public:
    HCI_driver(cordio::CordioHCITransportDriver &transport_driver
                       /* specific constructor arguments*/
    );

    virtual ~HCI_driver();

    virtual void do_initialize();

    virtual void do_terminate();

    virtual ble::vendor::cordio::buf_pool_desc_t get_buffer_pool_description();

    // virtual void start_reset_sequence();

    // virtual void handle_reset_sequence(uint8_t *msg);

  private:
    // private driver declarations
};

} // namespace DA1469x
} // namespace vendor
} // namespace ble