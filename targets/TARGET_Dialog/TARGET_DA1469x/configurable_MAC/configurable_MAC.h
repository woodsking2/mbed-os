#pragma once
#include <memory>
#include "gsl/gsl"
#include "Nondestructive.h"
class Configurable_MAC final : virtual public Nondestructive
{
  public:
    enum class Result
    {
        success,
        uninitialized,
        initialized,
    };
    Configurable_MAC();
    ~Configurable_MAC();
    /**
     * @brief only call once
     *
     * @return Result
     */
    Result initialize();

    /**
     * @brief only can call before initialized
     *
     * @param ble address
     * @return
     *
     */
    Result set_address(gsl::span<uint8_t, 6> address);

    /**
     * @brief only can call after initialized
     *
     * @param data
     * @return Result
     */
    Result write(gsl::span<uint8_t const> data);

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
