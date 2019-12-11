#pragma once
#include <memory>
#include "pinnames.h"
#include "Interrupt_instance.h"
class Interrupt_manager final
{
  private:
    Interrupt_manager();
    ~Interrupt_manager();
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    static Interrupt_manager &get_instance();
    void add(PinName pin, Interrupt_instance *);
    void remove(PinName pin);
    void interrupt_from_isr();
};
