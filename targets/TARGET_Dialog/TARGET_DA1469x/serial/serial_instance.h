#pragma once
#include "PlatformMutex.h"
#include "PinNames.h"
#include "serial_api.h"
#include <memory>
class Serial_instance
{
  private:
    class impl;
    std::unique_ptr<impl> m_impl;
    Serial_instance();    
  public:
    ~Serial_instance();
    static Serial_instance *acquire();
    void release();
    void initialize(PinName tx, PinName rx);
    void set_baudrate(int baudrate);
    void set_format(int data_bits, SerialParity parity, int stop_bits);
    bool write_able();
    void write(int c);
};
