#include "mbed_wait_api.h"
extern "C"
{
#include "default_config.h"
#include "hw_clk.h"
}
void wait_ns(unsigned int ns)
{
    auto us = ns / 1000;
    hw_clk_delay_usec(us);
}

void wait_us(int us)
{
    hw_clk_delay_usec(us);
}
