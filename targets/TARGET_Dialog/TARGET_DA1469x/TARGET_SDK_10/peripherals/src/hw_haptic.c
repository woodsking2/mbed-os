/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup Haptic
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file hw_haptic.c
 *
 * @brief Implementation of the Haptic Low Level Driver.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "hw_haptic.h"
#include "hw_clk.h"

#define FREQ2HALFPERIOD(f) (uint16_t)(1000000 / ((f) << 3))

#if dg_configUSE_HW_ERM

void hw_haptic_erm_init(const haptic_config_t *cfg)
{
        ASSERT_WARNING(cfg);
        ASSERT_WARNING((cfg->duty_cycle <= DREF_MAX_VAL) && (cfg->duty_cycle >= DREF_MIN_VAL)); // out of range
        //Enable the LRA digital clk
        REG_SETF(CRG_PER, CLK_PER_REG, LRA_CLK_EN, 1);
        REG_SETF(LRA, LRA_CTRL1_REG, LDO_EN, 1);
        REG_SETF(LRA, LRA_CTRL1_REG, HBRIDGE_EN, 1);
        REG_SETF(LRA, LRA_CTRL1_REG, ADC_EN, 1);
        LRA->LRA_CTRL3_REG = 0;
        LRA->LRA_DFT_REG = 0;
        // Bridge settings
        LRA->LRA_BRD_LS_REG = 0x077F;
        LRA->LRA_BRD_HS_REG = 0x1407;

        REG_SETF(LRA, LRA_CTRL3_REG, DREF, (uint16_t)(1 << 15) * (cfg->duty_cycle) / 1000);

        if (cfg->signal_out == HW_ERM_OUTPUT_HDRVM_HDRVP) {
                REG_SETF(LRA, LRA_CTRL2_REG, HALF_PERIOD, FREQ2HALFPERIOD(cfg->resonant_frequency));
        } else {
                REG_SETF(LRA, LRA_DFT_REG, SWM_SEL, 1); //Polarity not controlled in the loop
                REG_SETF(LRA, LRA_DFT_REG, SWM_MAN, cfg->signal_out);
        }

        LRA->LRA_FLT_COEF1_REG = 0; //Loop filter not used
        LRA->LRA_FLT_COEF2_REG = 0; //Loop filter not used
        LRA->LRA_FLT_COEF3_REG = 0; //Loop filter not used

        /*
         * At system startup keeping LOOP_EN = 0 ensures clock synchronization in the FSM
         * that controls the H-bridge. We need to wait approximately 2 PWM cycles before
         * enabling the FSM.
         */
        hw_clk_delay_usec(8);
        REG_SETF(LRA, LRA_CTRL1_REG, LOOP_EN, 1); //Enabling FSM
}

#endif /* dg_configUSE_HW_ERM */

#if dg_configUSE_HW_LRA

#define FIX(a) (int)(16384 * (a) + 0.5)

__RETAINED static hw_haptic_lra_interrupt_cb_t lra_interrupt_cb;

typedef struct {
        uint16_t min_period;
        uint16_t max_period;
} lra_config_env_t;

__RETAINED static lra_config_env_t lra_cfg;

static void hw_lra_register_int(hw_haptic_lra_interrupt_cb_t handler)
{
        lra_interrupt_cb = handler;
        REG_SET_BIT(LRA, LRA_CTRL1_REG, IRQ_CTRL_EN);
}

static const uint16_t phi[] = {16384,9672,5110,2594,1302,652,326,163,84,41,20,10,5,3,1};

static int32_t cordic(int32_t y, int32_t x)
{
        int32_t dx,dy,dz,z,s;
        uint8_t i;
        z = 0;

        for (i = 0; i < sizeof(phi) / sizeof(phi[0]) ; i++) {
                if (y < 0) {
                        s = -1;
                } else {
                        s = 1;
                }
                // compute steps
                dx = (s * y) >> i;
                dy = (s * x) >> i;
                dz = s * phi[i];
                // take steps
                x += dx;
                y -= dy;
                z += dz;
        }
        return z;
}

static const int table[3][6] =
 {
     { FIX( 0.4031), FIX( 0.2693), FIX(  0.0946), FIX(  -0.0946), FIX( -0.2693), FIX(  -0.4031)},
     { FIX(-1.2636), FIX( 0.2281), FIX(  1.0354), FIX(   1.0354), FIX(  0.2281), FIX(  -1.2636)},
     { FIX( 1.1576), FIX(-0.0122), FIX( -0.6453), FIX(  -0.6453), FIX( -0.0122), FIX(   1.1576)}
 };

static void lra_frequency_control_cb(void)
{

        uint8_t smp_idx, j;
        int32_t beta[3] = {0,0,0};
        int32_t angle;
        int16_t half_period;
        int32_t sample_regs[4] = {0};
        int16_t sample;
        int32_t *sample_regs_p;


        // check the current sample index
        smp_idx = REG_GETF(LRA, LRA_CTRL1_REG, SMP_IDX);
        if ((smp_idx & 7) != 7) {
                // we've started too late
                return;
        }

        /* Read samples 1-8 or 9-16, depending on smp_idx */
        sample_regs_p = (int32_t *) ((smp_idx < 8) ? &LRA->LRA_FLT_SMP1_REG : &LRA->LRA_FLT_SMP5_REG);
        for (j = 0; j < 4; j++) {
                sample_regs[j] = *sample_regs_p++;
        }

        /* Compute beta parameters based on the 6 middle samples */
        for (j = 0; j < 6; j++) {
                sample = *(((int16_t *) sample_regs) + j + 1);
                beta[0] += table[0][j] * sample;
                beta[1] += table[1][j] * sample;
                beta[2] += table[2][j] * sample;
        }
        if (beta[1] < 0) {
                beta[0] = -beta[0];
                beta[1] = -beta[1];
        }

        angle = cordic(beta[0], beta[1]);

        half_period = REG_GETF(LRA, LRA_CTRL2_REG, HALF_PERIOD);

        half_period -= ((half_period*angle) >> 17);

        if (half_period > lra_cfg.min_period) {
                half_period = lra_cfg.min_period;
        } else if (half_period < lra_cfg.max_period) {
                half_period = lra_cfg.max_period;
        }
        REG_SETF(LRA, LRA_CTRL2_REG, HALF_PERIOD, half_period);
}

void hw_haptic_lra_init(const haptic_config_t *cfg)
{
        ASSERT_WARNING(cfg);
        ASSERT_WARNING((cfg->duty_cycle <= DREF_MAX_VAL) && (cfg->duty_cycle >= DREF_MIN_VAL)); // out of range
        ASSERT_WARNING((cfg->trim_gain == 0x1) || (cfg->trim_gain == 0x2)
                || (cfg->trim_gain == 0x4) || (cfg->trim_gain == 0x8)); // invalid trim-gain settings
        //Enable the LRA digital clk
        REG_SETF(CRG_PER, CLK_PER_REG, LRA_CLK_EN, 1);
        REG_SETF(LRA, LRA_CTRL1_REG, LDO_EN, 1);
        REG_SETF(LRA, LRA_CTRL1_REG, HBRIDGE_EN, 1);
        REG_SETF(LRA, LRA_CTRL1_REG, ADC_EN, 1);
        LRA->LRA_CTRL2_REG = 0;
        LRA->LRA_CTRL3_REG = 0;
        // Bridge settings
        LRA->LRA_BRD_LS_REG = 0x077F;
        LRA->LRA_BRD_HS_REG = 0x1407;
        // LRA ADC trim settings
        *((uint32_t *) 0x50030A48) = 0x00020100;
        *((uint32_t *) 0x50030A4C) = 0x00020100;

        REG_SETF(LRA, LRA_CTRL1_REG, IRQ_IDX, 6);
        REG_SETF(LRA, LRA_CTRL1_REG, IRQ_DIV, 0);
        REG_SETF(LRA, LRA_CTRL1_REG, SMP_SEL, 0); // Select raw down-sampled data from the ADC for the resonance control algorithm
        REG_SETF(LRA, LRA_CTRL3_REG, DREF, (uint16_t)(1 << 15) * (cfg->duty_cycle) / 1000);
        REG_SETF(LRA, LRA_BRD_HS_REG, TRIM_GAIN, cfg->trim_gain);
        REG_SETF(LRA, LRA_CTRL2_REG, HALF_PERIOD, FREQ2HALFPERIOD(cfg->resonant_frequency));

        LRA->LRA_FLT_COEF1_REG = 0;
        LRA->LRA_FLT_COEF2_REG = 0;
        LRA->LRA_FLT_COEF3_REG = 0;

        lra_cfg.min_period = FREQ2HALFPERIOD(cfg->resonant_frequency_min);
        lra_cfg.max_period = FREQ2HALFPERIOD(cfg->resonant_frequency_max);

        /*
         * At system startup keeping LOOP_EN = 0 ensures clock synchronization in the FSM
         * that controls the H-bridge. We need to wait approximately 2 PWM cycles before
         * enabling the FSM.
         */
        hw_clk_delay_usec(8);
        REG_SETF(LRA, LRA_CTRL1_REG, LOOP_EN, 1);
        hw_lra_register_int(lra_frequency_control_cb);
}

void LRA_Handler(void)
{
        if (lra_interrupt_cb != NULL) {
                lra_interrupt_cb();
        }
}
#endif /* dg_configUSE_HW_LRA */
/**
 * \}
 * \}
 * \}
 */
