#pragma once
#include "PinNames.h"
#include <memory>
class Pwmout_manager final
{
  public:
    enum class Type
    {
        timer_1,
        timer_3,
        timer_4,
    };
    static Pwmout_manager &get_instance();
    Type acquire(PinName pin);
    void release(Type type);

  private:
    Pwmout_manager();
    ~Pwmout_manager();
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
