#pragma once
#include <memory>
class Spi_manager final
{
  public:
    enum class Type
    {
        spi_1,
        spi_2,
    };

  private:
    Spi_manager();
    ~Spi_manager();
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    static Spi_manager &get_instance();
    Type acquire();
    void release(Type type);
};
