#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
    void low_level_clock_init_initialize(void);
    void low_level_clock_init_set_up(void);
    void XTAL32M_Ready_Handler(void);
    void PLL_Lock_Handler(void);
#ifdef __cplusplus
}
#endif
