#include "system_clock.h"
#include "gsl/gsl"
#include "mbed.h"
#include "mbed_debug.h"
extern "C"
{
#include "default_config.h"
#include "sdk_defs.h"
#include "hw_clk.h"
}
using namespace gsl;
using namespace std;
using namespace mbed;
class System_clock::Impl
{
  private:
    void low_power_clock_initialize();

  public:
    void initialize();
};

void System_clock::Impl::low_power_clock_initialize()
{
    // debug("low_power_clock_initialize\n");
    hw_clk_set_lpclk(LP_CLK_IS_XTAL32K);
}
void System_clock::Impl::initialize()
{
    // debug("System_clock initialize\n");
    hw_clk_disable_sysclk(SYS_CLK_IS_RC32);
    auto result = mbed_highprio_event_queue()->call_in(dg_configXTAL32K_SETTLE_TIME, callback(this, &System_clock::Impl::low_power_clock_initialize));
    Ensures(result != 0);
}
System_clock::System_clock() : m_impl(make_unique<System_clock::Impl>())
{
}
System_clock::~System_clock() = default;

System_clock &System_clock::get_instance()
{
    static System_clock instance;
    return instance;
}
void System_clock::initialize()
{
    m_impl->initialize();
}