/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup PDC
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file hw_pdc.c
 *
 * @brief Implementation of the Power Domains Controller Low Level Driver.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#if dg_configUSE_HW_PDC


#include <hw_pdc.h>

__RETAINED_CODE uint32_t hw_pdc_add_entry(uint32_t lut_entry)
{
        uint32_t idx = HW_PDC_INVALID_LUT_INDEX;

        for (int i = 0; i < HW_PDC_LUT_SIZE; ++i) {
                if (*(&PDC->PDC_CTRL0_REG + i) == HW_PDC_UNUSED_LUT_ENTRY_VALUE) {
                        idx = i;
                        *(&PDC->PDC_CTRL0_REG + i) = lut_entry;
                        break;
                }
        }

        return idx;
}

void hw_pdc_write_entry(uint32_t idx, uint32_t value)
{
        ASSERT_ERROR(idx < HW_PDC_LUT_SIZE);

        // In case of invalid value check if LUT idx is pending. If it is, acknowledge it.
        if ((value & HW_PDC_LUT_ENTRY_FIELD_MASK(PDC_MASTER)) == 0 &&
            ((hw_pdc_get_pending() & (1 << idx)) != 0)) {
                hw_pdc_acknowledge(idx);
                // Check if it is the only pending idx. If it is clean pending PDC IRQ.
                if (!hw_pdc_get_pending() && NVIC_GetPendingIRQ(PDC_IRQn)) {
                        NVIC_ClearPendingIRQ(PDC_IRQn);
                }
        }

        *(&PDC->PDC_CTRL0_REG + idx) = value;
}

uint32_t hw_pdc_remove_entry(uint32_t idx)
{
        uint32_t old_value = hw_pdc_read_entry(idx);

        hw_pdc_write_entry(idx, HW_PDC_UNUSED_LUT_ENTRY_VALUE);

        return old_value;
}

void hw_pdc_ack_all_pending_cm33(void)
{
        uint32_t pending = hw_pdc_get_pending_cm33();

        for (int i = 0; i < HW_PDC_LUT_SIZE; ++i) {
                if (pending & (1 << i) ) {
                        hw_pdc_acknowledge(i);
                }
        }
}

void hw_pdc_lut_reset(void)
{
        for (int i = 0; i < HW_PDC_LUT_SIZE; ++i) {
                *(&PDC->PDC_CTRL0_REG + i) = HW_PDC_UNUSED_LUT_ENTRY_VALUE;
                hw_pdc_acknowledge(i);
        }
}

#endif /* dg_configUSE_HW_PDC */
/**
 * \}
 * \}
 * \}
 */
