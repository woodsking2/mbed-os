/**
 * \addtogroup PLA_DRI_PER_ANALOG
 * \{
 * \addtogroup HW_HAPTIC Haptic LRA/ERM Driver
 * \{
 * \brief Haptic Controller
 */

/**
 ****************************************************************************************
 *
 * @file hw_haptic.h
 *
 * @brief Definition of API for the Haptic Low Level Driver.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef HW_HAPTIC_H_
#define HW_HAPTIC_H_


#include <sdk_defs.h>

#if dg_configUSE_HW_ERM || dg_configUSE_HW_LRA

#define DREF_MAX_VAL 950
#define DREF_MIN_VAL 50

#if dg_configUSE_HW_ERM
/**
 * \brief Haptic driver output
 *
 */
typedef enum {
        HW_ERM_OUTPUT_HDRVM = 0,        /**< Negative haptic driver output is enabled*/
        HW_ERM_OUTPUT_HDRVP = 1,        /**< Positive haptic driver output is enabled*/
        HW_ERM_OUTPUT_HDRVM_HDRVP = 2,  /**< Negative and Positive haptic driver outputs are enabled */
} HW_ERM_OUTPUT;
#endif

/**
 * \brief Haptic configuration
 *
 */
typedef struct {
        uint16_t duty_cycle;                    /**< duty cycle reference value (permille)*/
        uint16_t resonant_frequency;            /**< ERM's/LRA's resonant frequency (Hz) */
#if dg_configUSE_HW_ERM
        HW_ERM_OUTPUT signal_out;               /**< haptic driver output */
#endif
#if dg_configUSE_HW_LRA
        uint16_t resonant_frequency_min;        /**< LRA's minimum resonant frequency (Hz) */
        uint16_t resonant_frequency_max;        /**< LRA's maximum resonant frequency (Hz) */
        uint8_t trim_gain;                      /**< pre-amplifier gain */
#endif
} haptic_config_t;

#if dg_configUSE_HW_ERM

/**
 * \brief Initialize ERM
 *
 * \p cfg should not be NULL
 *
 * \param [in] cfg ERM configuration
 *
 */
void hw_haptic_erm_init(const haptic_config_t *cfg);

/**
 * \brief Enable or disable ERM feedback
 *
 * \param [in] state true to enable ERM feedback, or false to disable it
 *
 */
__STATIC_INLINE void hw_haptic_erm_enable(bool state)
{
        REG_SETF(LRA, LRA_CTRL1_REG, LRA_EN, state);
}
#endif /* dg_configUSE_HW_ERM */

#if dg_configUSE_HW_LRA
/**
 * \brief Callback that is fired on LRA event
 *
 */
typedef void (*hw_haptic_lra_interrupt_cb_t)(void);

/**
 * \brief Initialize LRA
 *
 * \p cfg should not be NULL
 *
 * \param [in] cfg LRA configuration
 *
 */
void hw_haptic_lra_init(const haptic_config_t *cfg);

/**
 * \brief Start LRA feedback
 *
 */
__STATIC_INLINE void hw_haptic_lra_start(void)
{
        NVIC_ClearPendingIRQ(LRA_IRQn);
        REG_SET_BIT(LRA, LRA_CTRL1_REG, LRA_EN);
        NVIC_EnableIRQ(LRA_IRQn);
}

/**
 * \brief Stop LRA feedback
 *
 */
__STATIC_INLINE void hw_haptic_lra_stop(void)
{
        REG_CLR_BIT(LRA, LRA_CTRL1_REG, LRA_EN);
        NVIC_DisableIRQ(LRA_IRQn);
}
#endif /* dg_configUSE_HW_LRA */

/**
 * \brief Set duty cycle reference
 *
 * Valid settings 50 through 950 (permille)
 *
 * \param [in] value duty cycle
 *
 */
__STATIC_INLINE void hw_haptic_set_duty_cycle(uint16_t value)
{
        ASSERT_WARNING((value <= DREF_MAX_VAL) && (value >= DREF_MIN_VAL)); // out of range
        REG_SETF(LRA, LRA_CTRL3_REG, DREF, (uint16_t)(1 << 15) * value / 1000);
}

#endif /* dg_configUSE_HW_ERM || dg_configUSE_HW_LRA */


#endif /* HW_HAPTIC_H_ */

/**
 * \}
 * \}
 * \}
 */
