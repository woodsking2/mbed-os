#pragma once
#include "PinNames.h"
#include "PeripheralNames.h"

#ifdef __cplusplus
class gpio_t
{
  public:
    PinName pin;
    PinDirection direction;
    int value;
    gpio_t();
    ~gpio_t();
};
#endif

struct serial_s
{
    void *instance;
};

struct Spi_instance;
struct spi_s
{
    struct Spi_instance *instance;
};
struct Flash_instance;
struct flash_s
{
    struct Flash_instance *instance;
};
