#pragma once
#include <memory>
class Configurable_MAC
{
  private:
    Configurable_MAC();
    ~Configurable_MAC();
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    static Configurable_MAC &get_instance();
    void reset();
    void initialize();
    void write(uint8_t const *buff, size_t len);
};
