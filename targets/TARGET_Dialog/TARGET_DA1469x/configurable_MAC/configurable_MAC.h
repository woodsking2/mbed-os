#pragma once
#include <memory>
#include "gsl/gsl"
class Configurable_MAC
{
  private:
    Configurable_MAC();
    ~Configurable_MAC();
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    static Configurable_MAC &get_instance();    
    void initialize();
    void write(gsl::span<uint8_t const> data);
};
