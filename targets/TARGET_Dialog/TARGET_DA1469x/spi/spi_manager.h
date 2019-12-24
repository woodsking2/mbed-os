#pragma once
#include "PinNames.h"
#include <memory>
#include "Spi_manager.h"
#include "PeripheralNames.h"
class Spi_manager final
{
  public:
    enum class Type
    {
        spi_1,
        spi_2,
    };
    static Spi_manager &get_instance();
    Type acquire(PinName mosi, PinName miso, PinName mclk);
    void release(Type type);
    static SPIName get_spi_name(Type type);

  private:
    Spi_manager();
    ~Spi_manager();
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
