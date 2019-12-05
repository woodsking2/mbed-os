#pragma once
#include "PinNames.h"

struct serial_s
{
    int unused;
};
typedef struct
{
    PinName pin;
    PinDirection direction;
    int value;
} gpio_t;

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
