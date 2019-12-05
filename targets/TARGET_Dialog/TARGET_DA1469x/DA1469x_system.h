#pragma once
#include "DA1469x_power_domain_controller.h"
namespace DA1469x
{
namespace system
{
extern power_domain_controller::Index combo_index;
void enable_cache_retainability();
} // namespace system
} // namespace DA1469x