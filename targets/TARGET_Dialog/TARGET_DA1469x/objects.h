#pragma once
#include "PinNames.h"
#include "PeripheralNames.h"

typedef struct 
{
    PinName pin;
    PinDirection direction;
    int value;
}gpio_t;
struct serial_s
{
    void *instance;
};
struct gpio_irq_s
{
    void *unused;
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
