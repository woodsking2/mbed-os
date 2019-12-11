#pragma once
#include "PinNames.h"
#include <memory>
#include "gpio_irq_api.h"

class Interrupt_instance
{
  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    Interrupt_instance(PinName pin, gpio_irq_handler handler, uint32_t id);
    ~Interrupt_instance();
    void set_event(gpio_irq_event event);
    void set_enable(bool enable);
    /**
     * @brief 中断触发
     * 
     */
    void triggered();
};