#pragma once
#include "PinNames.h"
#include <memory>
class I2c_instance final
{
  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    I2c_instance(PinName sda, PinName scl);
    ~I2c_instance();
    void set_frequency(int hz);
    int start();
    int stop();
    int read(int address, char *data, int length, int stop);
    int write(int address, const char *data, int length, int stop);
    void reset();
    int byte_read(int last);
    int byte_write(int data);
};
