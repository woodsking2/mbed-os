#include "DA1469x_otp.h"
#include "DA1469xAB.h"
#include "DA1469x_clock.h"
namespace
{
enum class Mode
{
    power_down,
    deep_stand_by,
    stand_by,
    read,
    programming,
    programming_verification,
    initial_read,
    reserved,
};
void set_mode(Mode mode)
{
    OTPC->OTPC_MODE_REG = (OTPC->OTPC_MODE_REG & ~OTPC_OTPC_MODE_REG_OTPC_MODE_MODE_Msk) | (static_cast<uint32_t>(mode) << OTPC_OTPC_MODE_REG_OTPC_MODE_MODE_Pos);
    while (!(OTPC->OTPC_STAT_REG & OTPC_OTPC_STAT_REG_OTPC_STAT_MRDY_Msk))
    {
        __NOP();
    }
}
} // namespace
namespace DA1469x
{
namespace otp
{
void initialize()
{
    clock::enable_otp();
    set_mode(Mode::stand_by);

    // /* set clk timing */
    OTPC->OTPC_TIM1_REG = 0x0999101f; /* 32 MHz default */
    OTPC->OTPC_TIM2_REG = 0xa4040409;

    clock::disable_otp();
}
} // namespace otp
} // namespace DA1469x
