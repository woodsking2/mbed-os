/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup GPADC
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file hw_gpadc.c
 *
 * @brief Implementation of the GPADC Low Level Driver.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "default_config.h"
#if dg_configUSE_HW_GPADC


#include <stdio.h>
#include <string.h>
#include <hw_gpadc.h>

#if (dg_configSYSTEMVIEW)
#  include "SEGGER_SYSVIEW_FreeRTOS.h"
#else
#  define SEGGER_SYSTEMVIEW_ISR_ENTER()
#  define SEGGER_SYSTEMVIEW_ISR_EXIT()
#endif

static hw_gpadc_interrupt_cb intr_cb = NULL;
static uint16_t hw_gpadc_apply_correction(uint16_t input);

__RETAINED static int16_t hw_gpadc_differential_gain_error;
__RETAINED static int16_t hw_gpadc_single_ended_gain_error;
__RETAINED static int16_t hw_gpadc_differential_offset_error;
__RETAINED static int16_t hw_gpadc_single_ended_offset_error;

#define ADC_IRQ         GPADC_IRQn


/**
 * \brief Calibration Data-Point
 * (Temperature, ADC value) & slope (x 1000)
 */
typedef struct {
        int32_t temp;
        uint32_t adc;
        int32_t millislope;
} hw_gpadc_calibration_values_t;

/**
 * \brief Default temperature calibration data.
 *
 * The calibration point coordinates and the temperature functions slope are empirical values
 * calculated via a range of test measurements between -40 and 80 degrees Celsius.
 * The arithmetic values are 32-bit left-aligned, following the format of the GP_ADC_RESULT_REG
 */
__RETAINED_CONST_INIT static const hw_gpadc_calibration_values_t temperature_calibration_data[HW_GPADC_TEMPSENSOR_MAX] = {
        {0, 0, 0},                      /* GND - no sensor */
        {0, 0, 0},                      /* No calibration function - Open circuit value Z */
        {0, 0, 0},                      /* No calibration function - V(ntc) */
        {0, 712 << 6, 2440 << 6},       /* temperature sensor near charger */
        {0, 0, 0},                      /* no sensor */
        {0, 641 << 6, -(1282 << 6)},    /* diode near radio */
        {0, 637 << 6, -(1300 << 6)},    /* diode near charger */
        {0, 638 << 6, -(1291 << 6)},    /* diode near bandgap */
};

void hw_gpadc_init(const gpadc_config *cfg)
{
        GPADC->GP_ADC_CTRL_REG = 0;
        GPADC->GP_ADC_CTRL2_REG = 0;
        GPADC->GP_ADC_CTRL3_REG = 0x40;      // default value for GP_ADC_EN_DEL

        NVIC_DisableIRQ(ADC_IRQ);

        hw_gpadc_configure(cfg);
}

void hw_gpadc_reset(void)
{
        GPADC->GP_ADC_CTRL_REG = REG_MSK(GPADC, GP_ADC_CTRL_REG, GP_ADC_EN);
        GPADC->GP_ADC_CTRL2_REG = 0;
        GPADC->GP_ADC_CTRL3_REG = 0x40;      // default value for GP_ADC_EN_DEL

        NVIC_DisableIRQ(ADC_IRQ);
}

void hw_gpadc_configure(const gpadc_config *cfg)
{
        if (cfg) {
                hw_gpadc_set_clock(cfg->clock);
                hw_gpadc_set_input_mode(cfg->input_mode);
                hw_gpadc_set_input(cfg->input);
                hw_gpadc_set_sample_time(cfg->sample_time);
                hw_gpadc_set_continuous(cfg->continous);
                hw_gpadc_set_interval(cfg->interval);
                hw_gpadc_set_input_attenuator_state(cfg->input_attenuator);
                hw_gpadc_set_chopping(cfg->chopping);
                hw_gpadc_set_oversampling(cfg->oversampling);
                if (hw_gpadc_get_input() == HW_GPADC_INPUT_SE_TEMPSENS) {
                        ASSERT_ERROR(cfg->temp_sensor < HW_GPADC_TEMPSENSOR_MAX);
                        ASSERT_WARNING(cfg->temp_sensor != HW_GPADC_NO_TEMP_SENSOR);
                        ASSERT_WARNING(cfg->temp_sensor != HW_GPADC_CHARGER_TEMPSENS_GND);
                        /* Switches on/off the GP_ADC_DIFF_TEMP_EN bit, according to cfg->temp_sensor value.
                         * This field drives the TEMPSENS input circuit (diodes or charger tempsens).
                         */
                        hw_gpadc_set_diff_temp_sensors(cfg->temp_sensor > HW_GPADC_CHARGER_TEMPSENS_VTEMP);
                        hw_gpadc_select_diff_temp_sensor(cfg->temp_sensor);
                }
        }
}

void hw_gpadc_register_interrupt(hw_gpadc_interrupt_cb cb)
{
        intr_cb = cb;

        REG_SETF(GPADC, GP_ADC_CTRL_REG, GP_ADC_MINT, 1);

        NVIC_EnableIRQ(ADC_IRQ);
}

void hw_gpadc_unregister_interrupt(void)
{
        NVIC_DisableIRQ(ADC_IRQ);

        REG_SETF(GPADC, GP_ADC_CTRL_REG, GP_ADC_MINT, 0);

        intr_cb = NULL;
}

void hw_gpadc_adc_measure(void)
{
        hw_gpadc_start();
        while (hw_gpadc_in_progress());
        hw_gpadc_clear_interrupt();
}

void hw_gpadc_offset_calibrate(void)
{
        uint16_t adc_off_p, adc_off_n;
        uint32_t diff;
        int i;

        for (i = 0; i < 5; i++) {               // Try up to five times to calibrate correctly.
                hw_gpadc_set_offset_positive(0x200);
                hw_gpadc_set_offset_negative(0x200);
                hw_gpadc_set_mute(true);
                if (dg_configUSE_SOC == 1) {
                        hw_gpadc_set_oversampling(4);
                }

                hw_gpadc_set_sign_change(false);
                hw_gpadc_adc_measure();
                if (dg_configUSE_SOC == 1) {
                        adc_off_p = ((hw_gpadc_get_raw_value() >> 6) - 0x200);
                } else {
                        adc_off_p = hw_gpadc_get_value() - 0x200;
                }

                hw_gpadc_set_sign_change(true);
                hw_gpadc_adc_measure();
                if (dg_configUSE_SOC == 1) {
                        adc_off_n = ((hw_gpadc_get_raw_value() >> 6) - 0x200);
                } else {
                        adc_off_n = hw_gpadc_get_value() - 0x200;
                }

                if (hw_gpadc_get_input_mode() == HW_GPADC_INPUT_MODE_SINGLE_ENDED) {
                        hw_gpadc_set_offset_positive(0x200 - 2 * adc_off_p);
                        hw_gpadc_set_offset_negative(0x200 - 2 * adc_off_n);
                } else {
                        hw_gpadc_set_offset_positive(0x200 - adc_off_p);
                        hw_gpadc_set_offset_negative(0x200 - adc_off_n);
                }

                hw_gpadc_set_sign_change(false);

                // Verify calibration result
                hw_gpadc_adc_measure();
                if (dg_configUSE_SOC == 1) {
                        adc_off_n = (hw_gpadc_get_raw_value() >> 6);
                } else {
                        adc_off_n = hw_gpadc_get_value();
                }

                if (adc_off_n > 0x200) {
                        diff = adc_off_n - 0x200;
                }
                else {
                        diff = 0x200 - adc_off_n;
                }

                if (diff < 0x8) {
                        break;
                }
                else if (i == 4) {
                        ASSERT_WARNING(0);      // Calibration does not converge.
                }
        }

        hw_gpadc_set_mute(false);
}

void ADC_Handler(void)
{
        SEGGER_SYSTEMVIEW_ISR_ENTER();

        if (intr_cb) {
                intr_cb();
        } else {
                hw_gpadc_clear_interrupt();
        }

        SEGGER_SYSTEMVIEW_ISR_EXIT();
}

void hw_gpadc_test_measurements(void)
{
        uint32_t val[16];
        uint32_t mid = 0, diff_n = 0, diff_p = 0;
        volatile bool loop = true;

        if (hw_gpadc_get_input_mode() != HW_GPADC_INPUT_MODE_SINGLE_ENDED) {
                hw_gpadc_set_input_mode(HW_GPADC_INPUT_MODE_SINGLE_ENDED);
                hw_gpadc_set_ldo_constant_current(true);
                hw_gpadc_set_ldo_dynamic_current(true);
                hw_gpadc_offset_calibrate();
        }
        hw_gpadc_reset();
        hw_gpadc_set_input_mode(HW_GPADC_INPUT_MODE_SINGLE_ENDED);
        hw_gpadc_set_ldo_constant_current(true);
        hw_gpadc_set_ldo_dynamic_current(true);
        hw_gpadc_set_sample_time(15);
        hw_gpadc_set_chopping(true);
        hw_gpadc_set_input(HW_GPADC_INPUT_SE_VBAT);
//        hw_gpadc_set_oversampling(1); // 2 samples
        hw_gpadc_set_oversampling(2); // 4 samples

        while (loop) {
                for (volatile int i = 0; i < 4; i++); // wait ~1.5usec
                for (int j = 0; j < 16; j++) {
                        hw_gpadc_adc_measure();

//                        val[j] = hw_gpadc_get_raw_value() >> 4; // 2 samples
                        val[j] = hw_gpadc_get_raw_value() >> 3; // 4 samples

                        if (j == 0) {
                                mid = 0;
                                diff_p = 0;
                                diff_n = 0;
                        }

                        mid += val[j];

                        if (j == 15) {
                                mid /= 16;
                                for (int k = 0; k < 16; k++) {
                                        if ((mid > val[k]) && (diff_p < mid - val[k])) {
                                                diff_p = mid - val[k];
                                        }

                                        if ((mid < val[k]) && (diff_n < val[k] - mid)) {
                                                diff_n = val[k] - mid;
                                        }
                                }
                        }
                }
        }
}

bool hw_gpadc_pre_check_for_gain_error(void)
{
        if (dg_configUSE_ADC_GAIN_ERROR_CORRECTION == 1) {
                return (hw_gpadc_single_ended_gain_error && hw_gpadc_differential_gain_error);
        }

        return false;
}


void hw_gpadc_store_se_gain_error(int16_t single)
{
        hw_gpadc_single_ended_gain_error = single;
}

void hw_gpadc_store_diff_gain_error(int16_t diff)
{
        hw_gpadc_differential_gain_error = diff;
}


void hw_gpadc_store_se_offset_error(int16_t single)
{
        hw_gpadc_single_ended_offset_error = single;
}

void hw_gpadc_store_diff_offset_error(int16_t diff)
{
        hw_gpadc_differential_offset_error = diff;
}

static uint16_t hw_gpadc_apply_correction(uint16_t input)
{
        int32_t res = input;

        if (dg_configUSE_ADC_GAIN_ERROR_CORRECTION == 1) {
                if (hw_gpadc_pre_check_for_gain_error()) {
                        int16_t gain_trim;
                        int16_t offset;
                        const HW_GPADC_INPUT_MODE input_mode = hw_gpadc_get_input_mode();

                        if (input_mode == HW_GPADC_INPUT_MODE_SINGLE_ENDED) {
                                gain_trim = hw_gpadc_single_ended_gain_error;
                                /* Gain correction */
                                res = UINT16_MAX * (uint32_t)input / (UINT16_MAX + gain_trim);
                                offset = hw_gpadc_single_ended_offset_error;
                                res = (uint32_t)res - UINT16_MAX * offset / (UINT16_MAX + gain_trim);
                                /* Boundary check */
                                if ((uint32_t)res > UINT16_MAX) {
                                        return UINT16_MAX;
                                }
                        } else if (input_mode == HW_GPADC_INPUT_MODE_DIFFERENTIAL) {
                                gain_trim =  hw_gpadc_differential_gain_error;
                                res = (int16_t)(input ^ 0x8000);
                                /* Gain correction */
                                res = UINT16_MAX * res / (UINT16_MAX + gain_trim);
                                offset = hw_gpadc_differential_offset_error;
                                res = res - UINT16_MAX * offset / (UINT16_MAX + gain_trim);
                                /* Boundary check for lower limit */
                                if (res < INT16_MIN) {
                                        return 0;               /* INT16_MIN ^ 0x8000 */
                                }
                                /* Boundary check for upper limit */
                                if (res > INT16_MAX) {
                                        return UINT16_MAX;      /* INT16_MAX ^ 0x8000 */
                                }
                                res ^= 0x8000;
                        }
                }
        }

        return res;
}

int16_t hw_gpadc_convert_to_temperature(const gpadc_config *cfg, uint16_t val)
{
        int32_t calib_temp;
        uint32_t calib_adc;
        int32_t slope_x_1000;
        uint32_t val32 = val << (6 - MIN(6, cfg->oversampling));

        /* Check if shifting due to oversampling is correct */
        ASSERT_ERROR((val32 & 0xFFFF0000) == 0);

        ASSERT_ERROR(cfg->temp_sensor < HW_GPADC_TEMPSENSOR_MAX);
        calib_temp = temperature_calibration_data[cfg->temp_sensor].temp;
        calib_adc = temperature_calibration_data[cfg->temp_sensor].adc;
        slope_x_1000 = temperature_calibration_data[cfg->temp_sensor].millislope;
        return calib_temp + ((int32_t)(val32 - calib_adc) * 1000) / slope_x_1000;
}

uint16_t hw_gpadc_convert_temperature_to_raw_val(const gpadc_config *cfg, int16_t temp)
{
        int32_t calib_temp;
        uint32_t calib_adc;
        int32_t slope_x_1000;
        uint32_t result32;

        ASSERT_ERROR(cfg->temp_sensor < HW_GPADC_TEMPSENSOR_MAX);
        calib_temp = temperature_calibration_data[cfg->temp_sensor].temp;
        calib_adc = temperature_calibration_data[cfg->temp_sensor].adc;
        slope_x_1000 = temperature_calibration_data[cfg->temp_sensor].millislope;
        result32 = calib_adc + ((int32_t)temp - calib_temp) * slope_x_1000 / 1000;
        ASSERT_ERROR((result32 & 0xFFFF0000) == 0);
        return (uint16_t) result32;
}


uint16_t hw_gpadc_get_value(void)
{
        uint16_t adc_raw_res = hw_gpadc_get_raw_value();
        return hw_gpadc_apply_correction(adc_raw_res) >> (6 - MIN(6, hw_gpadc_get_oversampling()));
}
#endif /* dg_configUSE_HW_GPADC */
/**
 * \}
 * \}
 * \}
 */
