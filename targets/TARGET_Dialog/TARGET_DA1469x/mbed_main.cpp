

#include "mbed_debug.h"
#include "system_clock.h"
extern "C"
{
    void mbed_main();
}
void mbed_main()
{
    // debug("mbed_main\n");
    System_clock::get_instance().initialize();
    // mbed_highprio_event_queue().call()
}