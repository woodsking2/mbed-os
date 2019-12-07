#pragma once
#include <cstddef>
#include <cstdint>
namespace DA1469x
{
namespace configurable_MAC
{
void reset();
void initialize();
void write(uint8_t const *buff, size_t len);
} // namespace configurable_MAC
} // namespace DA1469x