#pragma once
#include <memory>
class System_clock final
{
  private:
    System_clock();
    ~System_clock();
    class Impl;
    std::unique_ptr<Impl> m_impl;

  public:
    static System_clock &get_instance();
    void initialize();
};
