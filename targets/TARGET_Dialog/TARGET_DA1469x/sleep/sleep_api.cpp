#include "sleep_api.h"
extern "C"
{
#include "default_config.h"
}
void hal_sleep()
{
     __WFI();
}
void hal_deepsleep()
{
    __WFI();
}