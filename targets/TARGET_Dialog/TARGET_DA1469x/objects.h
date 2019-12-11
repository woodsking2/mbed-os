#pragma once
#include "PinNames.h"
#include "PeripheralNames.h"

typedef struct
{
    PinName pin;
    PinDirection direction;
    int value;
} gpio_t;
struct serial_s
{
    void *instance;
};
struct gpio_irq_s
{
    void *instance;
};
struct Spi_instance;
struct spi_s
{
    void *instance;
};
struct Flash_instance;
struct flash_s
{
    void *instance;
};
