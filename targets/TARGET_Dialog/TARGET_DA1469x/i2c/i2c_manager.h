#pragma once
#include "PinNames.h"
#include <memory>
#include "i2c_instance.h"
class I2c_manager final
{
  public:
    enum class Type
    {
        i2c_1,
        i2c_2,
    };

  private:
    I2c_manager();
    ~I2c_manager();
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    static I2c_manager &get_instance();
    Type acquire(PinName sda, PinName scl);
    void release(Type type);
};
