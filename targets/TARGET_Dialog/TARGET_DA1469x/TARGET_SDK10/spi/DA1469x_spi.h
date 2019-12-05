#pragma once
#include "PinNames.h"
#include <memory>
#include <cstdint>
#include "gsl/gsl"
#include "newt_sdks.h"
#include "DA1469x_spi_instance.h"

namespace DA1469x
{
class Spi
{
  public:
    Spi()
    {
    }
    virtual ~Spi()
    {
    }
    // virtual void initialize() = 0;
    virtual uint8_t write(uint8_t value) = 0;
    virtual int block_write(gsl::span<uint8_t const> tx, gsl::span<uint8_t> rx, uint8_t write_fill) = 0;

  public:
    class Config
    {
      public:
        enum class Bits
        {
            bits8,
        };
        enum class Mode
        {
            mode_0,
            mode_1,
            mode_2,
            mode_3,
        };
        enum class Frequency
        {
            unset,
            freq_16m,
            freq_8m,
            freq_4m,
            freq_2_28m, // 2.28 m
        };
        PinName m_mosi_pin;
        PinName m_miso_pin;
        PinName m_clock_pin;
        PinName m_cs_pin;
        Bits m_bits;
        Mode m_mode;
        Frequency m_frequency;
        static constexpr int frequency_to_int(Frequency frequency)
        {
            switch (frequency)
            {
            case Frequency::unset:
                Expects(false);
                break;
            case Frequency::freq_16m:
                return 16000000;
            case Frequency::freq_8m:
                return 8000000;
            case Frequency::freq_4m:
                return 4000000;
            case Frequency::freq_2_28m:
                return 2285715;
            }
            Expects(false);
        }
        static Frequency int_to_frequency(int hz)
        {
            switch (hz)
            {
            case frequency_to_int(Frequency::freq_16m):
                return Frequency::freq_16m;
            case frequency_to_int(Frequency::freq_8m):
                return Frequency::freq_8m;
            case frequency_to_int(Frequency::freq_4m):
                return Frequency::freq_4m;
            case frequency_to_int(Frequency::freq_2_28m):
                return Frequency::freq_2_28m;
            }
            return Frequency::unset;
        }
    };
};

} // namespace DA1469x

struct Spi_instance
{
    DA1469x::Spi::Config m_config;
    std::unique_ptr<DA1469x::Spi> m_spi;
    void initialize();
};

namespace DA1469x
{
namespace Impl
{
class Spi : public virtual DA1469x::Spi
{
  protected:
    Config &m_config;
    Spi_instance::Id m_id;
    Spi(Config &config, Spi_instance::Type type);

  public:
    ~Spi() override;
};
} // namespace Impl
} // namespace DA1469x
