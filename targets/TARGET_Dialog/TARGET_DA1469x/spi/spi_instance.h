#pragma once
#include "PinNames.h"
#include <memory>
class Spi_instance final
{
  public:
    Spi_instance(PinName mosi, PinName miso, PinName sclk, PinName ssel);
    ~Spi_instance();
    void set_format(int bits, int mode, int slave);
    void set_frequency(int hz);
    int write(int value);
    int write(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill);
  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
