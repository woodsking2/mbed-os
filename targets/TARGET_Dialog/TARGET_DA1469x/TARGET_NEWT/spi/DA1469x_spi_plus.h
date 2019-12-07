#pragma once
#include "DA1469x_spi.h"
#include "DigitalOut.h"
namespace DA1469x
{
class Spi_plus : public DA1469x::Impl::Spi
{
  private:
    mbed::DigitalOut m_cs;
    SPI_Type *m_regs;
    void set_pin_func();
    void clear();
    void enable();
    void set_frequency();
    void set_bits();
    void set_mode();

  public:
    Spi_plus(DA1469x::Spi::Config &config);
    ~Spi_plus() override;

    // void initialize();
    uint8_t write(uint8_t value) override;
    int block_write(gsl::span<uint8_t const> tx, gsl::span<uint8_t> rx, uint8_t write_fill) override;
};

} // namespace DA1469x