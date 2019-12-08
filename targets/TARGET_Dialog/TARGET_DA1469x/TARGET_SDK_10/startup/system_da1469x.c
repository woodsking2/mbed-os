#include "stdint.h"
#include "mbed_toolchain.h"
#include "init_da1469x.h"
#define __RETAINED_RW MBED_SECTION("retention_mem_init")

#define __SYSTEM_CLOCK 32000000
__RETAINED_RW uint32_t SystemCoreClock = __SYSTEM_CLOCK;

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = __SYSTEM_CLOCK;
}

// extern void da1469x_SystemInit(void);

void SystemInit(void)
{

    SystemCoreClock = __SYSTEM_CLOCK;

    da1469x_SystemInit();
}