#include "PinNames.h"
#include <memory>
class Pwmout_instance final
{
  public:
    Pwmout_instance(PinName pin);
    ~Pwmout_instance();
    void write(float percent);
    float read();
    void period(float seconds);
    void period_ms(int ms);
    void period_us(int us);
    void pulse_width(float seconds);
    void pulse_width_ms(int ms);
    void pulse_width_us(int us);

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
