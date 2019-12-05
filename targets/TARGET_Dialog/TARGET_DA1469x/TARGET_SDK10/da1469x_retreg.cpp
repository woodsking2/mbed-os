#include "da1469x_retreg.h"
namespace newt
{
void da1469x_retreg_init(struct da1469x_retreg *retregs, uint8_t count)
{
    while (count--)
    {
        da1469x_retreg_invalidate(retregs++);
    }
}

void da1469x_retreg_update(struct da1469x_retreg *retregs, uint8_t count)
{
    while (count--)
    {
        retregs->value = *retregs->reg;
        retregs++;
    }
}

void da1469x_retreg_restore(struct da1469x_retreg *retregs, uint8_t count)
{
    while (count--)
    {
        *retregs->reg = retregs->value;
        retregs++;
    }
}
} // namespace newt