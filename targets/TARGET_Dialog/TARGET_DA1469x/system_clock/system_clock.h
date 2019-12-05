#pragma once
#ifdef __cplusplus
#include <memory>
extern "C"
{
#endif
    void low_level_clock_init_initialize(void);
    void low_level_clock_init_set_up(void);
    void XTAL32M_Ready_Handler(void);
    void PLL_Lock_Handler(void);
#ifdef __cplusplus
}
class System_clock final
{
  private:
    System_clock();
    ~System_clock();
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    enum class Clock
    {
        xtal,
        pll,
    };
    static System_clock &get_instance();
    void initialize();
    void set(Clock clock);
    void xtal_ready();
    void pll_ready();
};
#endif
