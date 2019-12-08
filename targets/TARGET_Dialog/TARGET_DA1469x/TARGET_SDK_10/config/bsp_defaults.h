/**
 * \addtogroup PLA_BSP_CONFIG
 * \{
 * \addtogroup BSP_CONFIG_DEFAULTS BSP Default Configuration Values
 *
 * \brief Board support package default configuration values
 *
 * The following tags are used to describe the type of each configuration option.
 *
 * - **\bsp_config_option_build**       : To be changed only in the build configuration
 *                                        of the project ("Defined symbols -D" in the
 *                                        preprocessor options).
 *
 * - **\bsp_config_option_app**         : To be changed only in the custom_config*.h
 *                                        project files.
 *
 * - **\bsp_config_option_expert_only** : To be changed only by an expert user.
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file bsp_defaults.h
 *
 * @brief Board Support Package. System Configuration file default values.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef BSP_DEFAULTS_H_
#define BSP_DEFAULTS_H_


#if defined(dg_configUSE_HW_ECC)
#if (dg_configUSE_HW_ECC == 1)
#error "(dg_configUSE_HW_ECC == 1) not supported in DA1469x!"
#endif
#else
#define dg_configUSE_HW_ECC                     (0)
#endif

#if defined(dg_configUSE_HW_TIMER1)
#if (dg_configUSE_HW_TIMER1 == 1)
#error "(dg_configUSE_HW_TIMER1 == 1) currently not supported!"
#endif
#else
#define dg_configUSE_HW_TIMER1                  (0)
#endif

/**
 * \brief Workarounds for AA silicon h/w issues - Disabled by default.
 */
#ifndef dg_configENABLE_DA1469x_AA_SUPPORT
#define dg_configENABLE_DA1469x_AA_SUPPORT  (0)
#endif




/**
 * \brief System Clock definitions
 */
#               ifndef dg_configXTAL32M_FREQ
#                      define dg_configXTAL32M_FREQ       (32000000)
#               endif /* dg_configXTAL32M_FREQ */
#               ifndef dg_configRC32M_FREQ
#                      define dg_configRC32M_FREQ         (32000000)
#               endif /*dg_configRC32M_FREQ */
#               define dg_configRC32M_FREQ_MIN            (27000000)
#               ifndef dg_configDIVN_FREQ
#                      define dg_configDIVN_FREQ          (32000000)
#               endif /* dg_configDIVN_FREQ */
#               ifndef dg_configPLL96M_FREQ
#                      define dg_configPLL96M_FREQ        (96000000)
#               endif /* dg_configPLL96M_FREQ */

#               if dg_configUSE_LP_CLK == LP_CLK_32768
#                       undef dg_configXTAL32K_FREQ
#                       define dg_configXTAL32K_FREQ      (32768)
#               elif dg_configUSE_LP_CLK == LP_CLK_32000
#                       undef dg_configXTAL32K_FREQ
#                       define dg_configXTAL32K_FREQ      (32000)
#               elif dg_configUSE_LP_CLK == LP_CLK_RCX
#                       undef dg_configXTAL32K_FREQ
#                       define dg_configXTAL32K_FREQ      (0)
#               elif dg_configUSE_LP_CLK == LP_CLK_ANY
#                       ifndef dg_configXTAL32K_FREQ
#                               error "dg_configXTAL32K_FREQ must be defined when dg_configUSE_LP_CLK is LP_CLK_ANY"
#                       endif
#               endif

#               define dg_configRC32K_FREQ                (32000)
#               define dg_configRCX_FREQ                  (10000)
#               ifndef dg_configLP_CLK_DRIFT
#                       if dg_configUSE_LP_CLK == LP_CLK_RCX
#                               define dg_configLP_CLK_DRIFT    (500) //ppm
#                       else
#                               define dg_configLP_CLK_DRIFT    (50) //ppm
#                       endif
#               endif





/**
 * \addtogroup POWER_SETTINGS
 *
 * \brief Power configuration settings
 * \{
 */
/* -------------------------------------- Power settings ---------------------------------------- */

/*
 * This is a deprecated configuration hence not to be defined by an application.
 * There's a single power configuration setup.
 */
#ifdef dg_configPOWER_CONFIG
#error "Configuration option dg_configPOWER_CONFIG is no longer supported."
#endif

/**
 * \brief When set to 1, the system will go to sleep and never exit allowing for the sleep current to be
 *        measured.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configTESTMODE_MEASURE_SLEEP_CURRENT
#define dg_configTESTMODE_MEASURE_SLEEP_CURRENT (0)
#endif
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup IMAGE_CONFIGURATION_SETTINGS
 *
 * \brief Image configuration settings.
 * \{
 */
/* ------------------------------- Image configuration settings --------------------------------- */

/**
 * \brief The chip revision that we compile for.
 *
 * \note There is no default value defined for the target chip revision. The application must\n
 *       neither explicitly set dg_configBLACK_ORCA_IC_REV, nor set\n
 *       dg_configUSE_AUTO_CHIP_DETECTION to 1.
 *
 * \bsp_default_note{\bsp_config_option_build,}
 */
#ifndef dg_configBLACK_ORCA_IC_REV
#       if dg_configUSE_AUTO_CHIP_DETECTION != 1
#               error "You must either define dg_configBLACK_ORCA_IC_REV in the build configuration or set dg_configUSE_AUTO_CHIP_DETECTION to 1"
#       endif
#endif

/**
 * \brief The chip stepping that we compile for.
 *
 * \note There is no default value defined for the target chip revision. The application must\n
 *       either explicitly set dg_configBLACK_ORCA_IC_STEP nor set\n
 *       dg_configUSE_AUTO_CHIP_DETECTION to 1.
 *
 * \bsp_default_note{\bsp_config_option_build,}
 */
#ifndef dg_configBLACK_ORCA_IC_STEP
#       if dg_configUSE_AUTO_CHIP_DETECTION != 1
#               error "You must either define dg_configBLACK_ORCA_IC_STEP in the build configuration or set dg_configUSE_AUTO_CHIP_DETECTION to 1"
#       endif
#endif

/**
 * \brief The motherboard revision we compile for.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configBLACK_ORCA_MB_REV
#define dg_configBLACK_ORCA_MB_REV      BLACK_ORCA_MB_REV_D
#endif

/*
 * This is a deprecated configuration hence not to be defined by an application.
 * The values of the trim registers are not anymore taken from the Flash.
 *
 */
#ifdef  dg_configCONFIG_HEADER_IN_FLASH
#error "Configuration option dg_configCONFIG_HEADER_IN_FLASH is no longer supported."
#endif

/**
 * \brief Controls how the image is built.
 *  - DEVELOPMENT_MODE: various debugging options are included.
 *  - PRODUCTION_MODE: all code used for debugging is removed.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configIMAGE_SETUP
#define dg_configIMAGE_SETUP            DEVELOPMENT_MODE
#endif

/**
 * \brief When set to 1, the delay at the start of execution of the Reset_Handler is skipped.
 *
 * \details This delay is added in order to facilitate proper programming of the Flash or QSPI\n
 *        launcher invocation. Without it, there is no guarantee that the execution of the image\n
 *        will not proceed, altering the default configuration of the system from the one that the\n
 *        bootloader leaves it.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configSKIP_MAGIC_CHECK_AT_START
#define dg_configSKIP_MAGIC_CHECK_AT_START      (0)
#endif

/**
 * \brief When set to 1, the chip version (DA14680/1-01 or DA14682/3-00, DA15XXX-00 ) will be detected\n
 *        automatically.
 * \note It cannot be used in BLE applications because of the different linker scripts\n
 *       that are used.
 * \warning Not to be used by a generic project, applicable for uartboot only.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configUSE_AUTO_CHIP_DETECTION
#define dg_configUSE_AUTO_CHIP_DETECTION        (0)
#else
# if (dg_configUSE_AUTO_CHIP_DETECTION == 1)
        #undef dg_configBLACK_ORCA_IC_REV
        #define dg_configBLACK_ORCA_IC_REV      BLACK_ORCA_IC_REV_AUTO
        #undef dg_configBLACK_ORCA_IC_STEP
        #define dg_configBLACK_ORCA_IC_STEP     BLACK_ORCA_IC_STEP_AUTO
# endif
#endif

/**
 * \brief When set to 1, the OTP copy will be emulated when in DEVELOPMENT_MODE.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configEMULATE_OTP_COPY
#define dg_configEMULATE_OTP_COPY       (0)
#endif

/**
 * \brief When set to 1, the QSPI copy will be emulated when in DEVELOPMENT_MODE (Not Applicable!).
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configEMULATE_QSPI_COPY
#define dg_configEMULATE_QSPI_COPY      (0)
#endif
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup LOW_POWER_CLOCK_SETTINGS
 *
 * \brief Doxygen documentation is not yet available for this module.
 *        Please check the source code file(s)
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 * \{
 */
/* --------------------------------- Low-Power clock settings ----------------------------------- */

/*
 * Maximum sleep time the sleep timer supports
 */


#if (dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768) || (dg_configUSE_LP_CLK == LP_CLK_RCX)
#ifdef dg_configTim1Prescaler
#error "Timer1 prescaler is not supported in DA1469x"
#undef dg_configTim1Prescaler
#endif /* dg_configTim1Prescaler */
#elif (dg_configUSE_LP_CLK == LP_CLK_ANY)

#else
#error "dg_configUSE_LP_CLK has invalid setting"
#endif

#ifdef dg_configTim1PrescalerBitRange
#error "Timer1 prescaler is not supported in DA1469x"
#undef dg_configTim1PrescalerBitRange
#endif

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup SYSTEM_CONFIGURATION_SETTINGS
 *
 * \brief System configuration settings
 *
 * \{
 */
/* ------------------------------- System configuration settings -------------------------------- */

/**
 * \brief Source of Low Power clock used (LP_CLK_IS_ANALOG, LP_CLK_IS_DIGITAL)
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configLP_CLK_SOURCE
#define dg_configLP_CLK_SOURCE          LP_CLK_IS_ANALOG
#endif

# if ((dg_configLP_CLK_SOURCE == LP_CLK_IS_ANALOG) && (dg_configUSE_LP_CLK == LP_CLK_ANY))
# error "When the LP source is analog (XTAL), the option LP_CLK_ANY is invalid!"
# endif

# if ((dg_configLP_CLK_SOURCE == LP_CLK_IS_DIGITAL) && (dg_configUSE_LP_CLK == LP_CLK_RCX))
# error "When the LP source is digital (External), the option LP_CLK_RCX is invalid!"
# endif

/**
 * \brief Low Power clock used (LP_CLK_32000, LP_CLK_32768, LP_CLK_RCX, LP_CLK_ANY)
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
# ifndef dg_configUSE_LP_CLK
# define dg_configUSE_LP_CLK            LP_CLK_RCX
# endif

/**
 * \brief Code execution mode
 *
 *  - MODE_IS_RAM
 *  - MODE_IS_MIRRORED
 *  - MODE_IS_CACHED
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
# ifndef dg_configEXEC_MODE
# define dg_configEXEC_MODE             MODE_IS_RAM
# endif

/**
 * \brief Code location
 *
 * - NON_VOLATILE_IS_OTP
 * - NON_VOLATILE_IS_FLASH
 * - NON_VOLATILE_IS_NONE (RAM)
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
# ifndef dg_configCODE_LOCATION
# define dg_configCODE_LOCATION         NON_VOLATILE_IS_NONE
# endif


#ifdef dg_configEXT_CRYSTAL_FREQ
#       error "Configuration option dg_configEXT_CRYSTAL_FREQ is no longer supported."
#endif


/**
 * \brief External LP type
 *
 * - 0: a crystal is connected to XTAL32Kp and XTALK32Km
 * - 1: a digital clock is provided.
 *
 * \note the frequency of the digital clock must be 32KHz or 32.768KHz and be always running.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
# ifndef dg_configEXT_LP_IS_DIGITAL
# define dg_configEXT_LP_IS_DIGITAL     (0)
# endif

/*
 * This is a deprecated configuration hence not to be defined by an application.
 * Forcing the system to enter into clockless sleep during deep sleep is no longer supported.
  */
#ifdef dg_configFORCE_DEEP_SLEEP
#error "Configuration option dg_configFORCE_DEEP_SLEEP is no longer supported."
#endif

/**
 * \brief Timer 1 usage
 *
 * When set to 0, Timer1 is reserved for the OS tick.
 *
 * \note A setting to 1 is meaningful only for non-RTOS projects.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configUSER_CAN_USE_TIMER1
#define dg_configUSER_CAN_USE_TIMER1    (0)
#endif

/**
 * \brief Time needed for the settling of the LP clock, in msec.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configXTAL32K_SETTLE_TIME
#if dg_configLP_CLK_SOURCE == LP_CLK_IS_ANALOG
#define dg_configXTAL32K_SETTLE_TIME    (8000)
#else
#define dg_configXTAL32K_SETTLE_TIME    (1000)
#endif
#endif

#ifdef dg_configINITIAL_SLEEP_DELAY_TIME
#pragma message "dg_configINITIAL_SLEEP_DELAY_TIME is deprecated. At startup, the system will stay active \
          for at least dg_configXTAL32K_SETTLE_TIME before it is allowed to go to sleep"
#endif

/**
 * \brief XTAL32M trimming default settings
 */
#ifndef dg_configDEFAULT_CLK_FREQ_TRIM_REG__XTAL32M_TRIM__VALUE
#define dg_configDEFAULT_CLK_FREQ_TRIM_REG__XTAL32M_TRIM__VALUE    (0x120)
#endif

#ifndef dg_configDEFAULT_XTAL32M_CTRL0_REG__XTAL32M_CXCOMP_ENABLE__VALUE
#define dg_configDEFAULT_XTAL32M_CTRL0_REG__XTAL32M_CXCOMP_ENABLE__VALUE      (0x0)
#endif

/**
 * \brief XTAL32M settle time
 *
 * Time needed for the settling of the XTAL32M, in usec.
 *
 * \note If set to zero, the settling time will be automatically adjusted.
 */
#ifndef dg_configXTAL32M_SETTLE_TIME_IN_USEC
#define dg_configXTAL32M_SETTLE_TIME_IN_USEC    (0x0)
#endif

/**
 * \brief Enable XTAL32M upon system wake-up
 *
 * If set to 1 the PDC will enable XTAL32M when it wakes-up M33
 *
 */
#ifndef dg_configENABLE_XTAL32M_ON_WAKEUP
#define dg_configENABLE_XTAL32M_ON_WAKEUP       (1)
#endif

/**
 * \brief The number of LP clock cycles required to check the voltage of an LDO during startup
 */
#ifndef dg_configVOLTAGE_CHECK_LP_CYCLES
#define dg_configVOLTAGE_CHECK_LP_CYCLES        (5)
#endif

/**
 * \brief The number of LP clock cycles required for the HW FSM to detect the wake-up signal
 */
#ifndef dg_configHW_FSM_WAKEUP_CYCLES
#define dg_configHW_FSM_WAKEUP_CYCLES           (3)
#endif

/**
 * \brief The number of LP clock cycles required for the system to start-up
 */
#ifndef dg_configSYSTEM_STARTUP_CYCLES
#define dg_configSYSTEM_STARTUP_CYCLES          (4)
#endif


/**
 * \brief RC32 wake-up time in slow wake-up mode
 *
 * This is the maximum time, in LP cycles, needed to wake-up the chip and start executing code
 * using RC32 in slow wake-up mode.
 *
 * \note Wake-up time calculation:
 *              dg_configHW_FSM_WAKEUP_CYCLES cycles for wake-up
 *              1 additional cycle for slow wake-up
 *              2 cycles for V30 (worst case Vclamp -> 3V)
 *              1 cycle for BG
 *              dg_configVOLTAGE_CHECK_LP_CYCLES cycles for V12 and V14
 *              dg_configVOLTAGE_CHECK_LP_CYCLES cycles for V18P
 *              dg_configVOLTAGE_CHECK_LP_CYCLES cycles for V18
 *              dg_configSYSTEM_STARTUP_CYCLES cycles for system start-up
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#define dg_configWAKEUP_RC32_TIME_SLOW       (dg_configHW_FSM_WAKEUP_CYCLES + 4 + \
                                              3 * dg_configVOLTAGE_CHECK_LP_CYCLES + \
                                              dg_configSYSTEM_STARTUP_CYCLES)

/**
 * \brief RC32 wake-up time in fast wake-up mode
 *
 * This is the maximum time, in LP cycles, needed to wake-up the chip and start executing code
 * using RC32 in fast wake-up mode.
 *
 * \note Wake-up time calculation:
 *              dg_configHW_FSM_WAKEUP_CYCLES cycles for wake-up
 *              dg_configVOLTAGE_CHECK_LP_CYCLES cycles for V12 (worst case 0.75V -> 0.9V)
 *              dg_configSYSTEM_STARTUP_CYCLES cycles for system start-up
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#define dg_configWAKEUP_RC32_TIME_FAST       (dg_configHW_FSM_WAKEUP_CYCLES + \
                                              dg_configVOLTAGE_CHECK_LP_CYCLES + \
                                              dg_configSYSTEM_STARTUP_CYCLES)

/**
 * \brief RC32 wake-up time
 *
 * This is the maximum time, in LP cycles, needed to wake-up the chip and start executing code
 * using RC32 in ultra-fast wake-up mode.
 *
 * \note Wake-up time calculation:
 *              dg_configHW_FSM_WAKEUP_CYCLES cycles for wake-up
 *              dg_configSYSTEM_STARTUP_CYCLES cycles for system start-up
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#define dg_configWAKEUP_RC32_TIME_ULTRA_FAST (dg_configHW_FSM_WAKEUP_CYCLES + \
                                              dg_configSYSTEM_STARTUP_CYCLES)

#ifndef dg_configUSE_CLOCK_MGR
#       ifdef OS_BAREMETAL
#               define dg_configUSE_CLOCK_MGR (0)
#       elif defined(OS_FREERTOS)
#               define dg_configUSE_CLOCK_MGR (1)
#       endif
#endif



/**
 * \brief OS tick restore time
 *
 * This is the time, in LP cycles, required from the OS to restore the tick timer.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configOS_TICK_RESTORE_TIME

#endif // dg_configOS_TICK_RESTORE_TIME

/**
 * \brief Image copy time
 *
 * The number of LP cycles needed for the application's image data to be copied from the OTP
 * (or QSPI) to the RAM in mirrored mode.
 *
 * \warning MUST BE SMALLER THAN #dg_configMIN_SLEEP_TIME !!!
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#if (dg_configEXEC_MODE != MODE_IS_MIRRORED)
        #undef dg_configIMAGE_COPY_TIME
        #define dg_configIMAGE_COPY_TIME        (0)
#elif !defined(dg_configIMAGE_COPY_TIME)
        #if ((dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768))
                #define dg_configIMAGE_COPY_TIME        (64)
        #elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
                #define dg_configIMAGE_COPY_TIME        cm_rcx_us_2_lpcycles(1950)
        #else /* LP_CLK_ANY */
                // Must be defined in the custom_config_<>.h file.
        #endif
#endif

    /**
 * \brief Retention memory configuration.
 *
 * 16 bits field; each couple of bits controls whether the relevant memory block will be retained (0) or not (1).
 * -  bits 0-1   : SYSRAM1
 * -  bits 2-3   : SYSRAM2
 * -  bits 4-5   : SYSRAM3
 * -  bits 6-7   : SYSRAM4
 * -  bits 8-9   : SYSRAM5
 * -  bits 9-10  : SYSRAM6
 * -  bits 11-12 : SYSRAM7
 * -  bits 13-14 : SYSRAM8
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 *
 */
# ifndef dg_configMEM_RETENTION_MODE
#  define dg_configMEM_RETENTION_MODE     (0)
# endif

/*
 * This is a deprecated configuration hence not to be defined by an application.
 */
#ifdef dg_configMEM_RETENTION_MODE_PRESERVE_IMAGE
#error "dg_configMEM_RETENTION_MODE_PRESERVE_IMAGE is no longer supported."
#endif

/**
 * \brief Memory Shuffling mode.
 *
 * See SYS_CTRL_REG:REMAP_RAMS field.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configSHUFFLING_MODE
#define dg_configSHUFFLING_MODE         (0)
#endif

/**
 * \brief ECC microcode RAM retainment
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configECC_UCODE_RAM_RETAINED
#define dg_configECC_UCODE_RAM_RETAINED (0)
#endif

/**
 * \brief Watchdog Service
 *
 * - 1: enabled
 * - 0: disabled
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUSE_WDOG
#define dg_configUSE_WDOG               (0)
#endif

/**
 * \brief Brown-out Detection
 *
 * - 1: used
 * - 0: not used
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUSE_BOD
#define dg_configUSE_BOD                (1)
#endif

/**
 * \brief Reset value for Watchdog.
 *
 * See WATCHDOG_REG:WDOG_VAL field.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configWDOG_RESET_VALUE
#define dg_configWDOG_RESET_VALUE       (0xFF)
#endif


#if (dg_configUSE_WDOG == 0) && defined(dg_configWDOG_IDLE_RESET_VALUE)
#pragma message "dg_configWDOG_IDLE_RESET_VALUE is ignored. Maximum watchdog value will be used."
#undef dg_configWDOG_IDLE_RESET_VALUE
#endif

/**
 * \brief Reset value for Watchdog when system is idle.
 */
#ifndef dg_configWDOG_IDLE_RESET_VALUE
#define dg_configWDOG_IDLE_RESET_VALUE (SYS_WDOG_WATCHDOG_REG_WDOG_VAL_Msk >> SYS_WDOG_WATCHDOG_REG_WDOG_VAL_Pos)
#endif


/**
 * \brief Maximum watchdog tasks
 *
 * Maximum number of tasks that the Watchdog Service can monitor. It can be larger (up to 32) than
 * needed, at the expense of increased Retention Memory requirement.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configWDOG_MAX_TASKS_CNT
#               define dg_configWDOG_MAX_TASKS_CNT      (6)
#endif

/**
 * \brief Watchdog notify interval
 *
 * Interval (in miliseconds) for common timer which can be used to trigger tasks in order to notify
 * watchdog. Can be set to 0 in order to disable timer code entirely.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configWDOG_NOTIFY_TRIGGER_TMO
#define dg_configWDOG_NOTIFY_TRIGGER_TMO        (0)
#endif

/**
 * \brief Abort a clock modification if it will cause an error to the SysTick counter
 *
 * - 1: on
 * - 0: off
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configABORT_IF_SYSTICK_CLK_ERR
#define dg_configABORT_IF_SYSTICK_CLK_ERR       (0)
#endif

/**
 * \brief Maximum adapters count
 *
 * Should be equal to the number of Adapters used by the Application. It can be larger (up to 254)
 * than needed, at the expense of increased Retention Memory requirements. It cannot be 0.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configPM_MAX_ADAPTERS_CNT
#define dg_configPM_MAX_ADAPTERS_CNT    (16)
#endif

/**
 * \brief Maximum sleep defer time
 *
 * The maximum time sleep can be deferred via a call to pm_defer_sleep_for(). It is in clock cycles
 * in the case of the XTAL32K and in usec in the case of RCX.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configPM_MAX_ADAPTER_DEFER_TIME
#if ((dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768))
        #define dg_configPM_MAX_ADAPTER_DEFER_TIME      (128)
#elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
        #define dg_configPM_MAX_ADAPTER_DEFER_TIME      cm_rcx_us_2_lpcycles(4000)
#else /* LP_CLK_ANY */
        // Must be defined in the custom_config_<>.h file. It should be > 3.5msec.
#endif
#endif

/**
 * \brief Minimum sleep time
 *
 *  No power savings if we enter sleep when the sleep time is less than N LP cycles
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configMIN_SLEEP_TIME
#if ((dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768))
        #define dg_configMIN_SLEEP_TIME (33*3)  // 3 msec
#elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
        #define dg_configMIN_SLEEP_TIME cm_rcx_us_2_lpcycles_low_acc(3000)  // 3 msec
#else /* LP_CLK_ANY */
        // Must be defined in the custom_config_<>.h file. It should be ~3msec but this may vary.
#endif
#endif


/**
 * \brief When set to 1, the DCDC is used.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configUSE_DCDC
#define dg_configUSE_DCDC               (1)
#endif

/*
 * This is a deprecated configuration hence not to be defined by an application.
 * The semantics of dg_configUSE_ADC replaced by dg_configUSE_HW_GPADC.
 */
#ifdef dg_configUSE_ADC
#error "Configuration option dg_configUSE_ADC is no longer supported. Use dg_configUSE_HW_GPADC instead."
#endif

/**
 * \brief Apply ADC Gain Error correction.
 * - 1: used
 * - 0: not used
 *
 * The default setting is: 1.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configUSE_ADC_GAIN_ERROR_CORRECTION
#define dg_configUSE_ADC_GAIN_ERROR_CORRECTION  (1)
#endif


/*
 * \brief AES will be able to access keys stored in OTP User Data Encryption Key section.
 * - 1: used
 * - 0: not used
 */
#ifndef dg_configAES_USE_OTP_KEYS
#define dg_configAES_USE_OTP_KEYS        (0)
#endif

/*
 * \brief AES will use secure DMA channel for keys stored in OTP User Data Encryption Key section.
 *
 * Enabling this feature also enables dg_configAES_USE_OTP_KEYS.
 *
 * - 1: used
 * - 0: not used
 */
#ifndef dg_configAES_USE_SECURE_DMA_CHANNEL
#define dg_configAES_USE_SECURE_DMA_CHANNEL        (0)
#elif (dg_configAES_USE_SECURE_DMA_CHANNEL == 1)
        #undef dg_configAES_USE_OTP_KEYS
        #define dg_configAES_USE_OTP_KEYS          (1)
#endif

/**
 * \addtogroup CHARGER_SETTINGS
 *
 * \brief Charger configuration settings
 * \{
 */
/* -------------------------------------- Charger settings -------------------------------------- */


/**
 * \brief Controls how the system will behave when the USB i/f is suspended.
 *
 * \details When the USB Node is suspended by the USB Host, the application may have to act in
 *          order to comply with the USB specification (consume less than 2.5mA). The available
 *          options are:
 *          0: do nothing
 *          1: pause system clock => the system clock is stopped and only VBUS and USB irqs are handled
 *          2: pause application => The system is not paused but the application must stop all
 *             timers and make sure all tasks are blocked.
 *
 *          Both in modes 1 and 2, the application must make sure that all external peripherals are
 *          either powered off or placed in the lowest power consumption mode.
 */
#ifndef dg_configUSB_SUSPEND_MODE
#define dg_configUSB_SUSPEND_MODE       USB_SUSPEND_MODE_NONE
#endif

/* ---------------------------------------------------------------------------------------------- */

/**
 * \} CHARGER_SETTINGS
 */


/**
 * \brief The Rsense of the SOC in multiples of 0.1Ohm. The default value is (1 x 0.1Ohm).
 */
#ifndef dg_configSOC_RSENSE
#define dg_configSOC_RSENSE             (1)     // N x 0.1Ohm
#endif

/**
 * \brief Set the board that is used.
 */
#ifndef dg_configUSE_Board
#         define dg_configUSE_Board       "boards/brd_prodk_da1469x.h"
#endif /* dg_configUSE_Board */

/**
 * \brief When set to 1, State of Charge function is enabled.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configUSE_SOC
#define dg_configUSE_SOC                (0)
#else
#   if (dg_configUSE_SOC == 1)
#   undef dg_configUSE_HW_SOC
#   define dg_configUSE_HW_SOC          (1)
#   endif
#endif

/**
 * \addtogroup FLASH_SETTINGS
 *
 * \brief Flash configuration settings
 *
 * \{
 */
/* -------------------------------------- Flash settings ---------------------------------------- */

/**
 * \brief The rail from which the Flash is powered, if a Flash is used.
 *
 * - FLASH_IS_NOT_CONNECTED
 * - FLASH_CONNECTED_TO_1V8
 * - FLASH_CONNECTED_TO_1V8P
 * - FLASH_CONNECTED_TO_1V8F
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configFLASH_CONNECTED_TO
#error "dg_configFLASH_CONNECTED_TO is not defined!"
#endif

/**
 * \brief When set to 1, the QSPI FLASH is put to power-down state during sleep.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configFLASH_POWER_DOWN
#define dg_configFLASH_POWER_DOWN       (0)
#endif

/* Backward compatibility checks */
#if defined(dg_configPOWER_FLASH) || defined(dg_configFLASH_POWER_OFF)
#define PRINT_POWER_RAIL_SETUP
#endif

#ifdef dg_configPOWER_EXT_1V8_PERIPHERALS
#error "dg_configPOWER_EXT_1V8_PERIPHERALS is deprecated! Please use dg_configPOWER_1V8P instead!"
#endif


#ifdef dg_configPOWER_FLASH
#error "dg_configPOWER_FLASH is deprecated! Use dg_configPOWER_1V8P_ACTIVE instead!"
#endif

#if (dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8P || dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8F) \
     && defined(dg_configFLASH_POWER_OFF) && defined(dg_configPOWER_1V8P_SLEEP) && (dg_configPOWER_1V8P_SLEEP == 0)
#error "Both dg_configFLASH_POWER_OFF and dg_configPOWER_1V8P_SLEEP are defined! Please use dg_configPOWER_1V8P_SLEEP only!"
#endif

#if (dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8) && defined(dg_configFLASH_POWER_OFF) && defined(dg_configPOWER_1V8_SLEEP)
#error "Both dg_configFLASH_POWER_OFF and dg_configPOWER_1V8_SLEEP are defined! Please use dg_configPOWER_1V8_SLEEP only!"
#endif


/* End of backward compatibility checks */

/**
 * \brief When set to 1, the 1V8 rail is powered, when the system is in active state.
 * When set to 2 the rail configuration will be defined by the application
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configPOWER_1V8_ACTIVE
 #define dg_configPOWER_1V8_ACTIVE                      (2)
#else
 #if (dg_configPOWER_1V8_ACTIVE == 0) && (dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8)
  #error "Flash is connected to the 1V8 rail but the rail is turned off. Please do not set dg_configPOWER_1V8_ACTIVE to 0."
 #endif
#endif


/**
 * \brief When set to 1, the 1V8 is powered during sleep.
 * When set to 2 the rail configuration will be defined by the application
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configPOWER_1V8_SLEEP
#define dg_configPOWER_1V8_SLEEP                        (2)
#endif



 /**
  * \brief When set to 1, the 1V8P rail is powered.
  *
  * \bsp_default_note{\bsp_config_option_app,}
  */
#ifdef dg_configPOWER_1V8P
 #if defined(dg_configPOWER_1V8P_ACTIVE) || defined(dg_configPOWER_1V8P_SLEEP)
  #error "dg_configPOWER_1V8P cannot be used when dg_configPOWER_1V8P_ACTIVE or dg_configPOWER_1V8P_SLEEP is used"
 #else
  #if dg_configPOWER_1V8P == 1
   #define dg_configPOWER_1V8P_ACTIVE      (1)
   #define dg_configPOWER_1V8P_SLEEP       (1)
  #else
   #define dg_configPOWER_1V8P_ACTIVE      (0)
   #define dg_configPOWER_1V8P_SLEEP       (0)
  #endif
 #endif
#endif

/**
 * \brief When set to 1, the 1V8P rail is powered, when the system is in active state.
 * When set to 2 the rail configuration will be defined by the application
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configPOWER_1V8P_ACTIVE
#define dg_configPOWER_1V8P_ACTIVE         (2)
#endif

#if defined(dg_configPOWER_1V8P_ACTIVE) && (dg_configPOWER_1V8P_ACTIVE == 0) && \
     (dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8P || dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8F)
 #error "Flash is connected to the 1V8P rail but the rail is turned off. Please do not set dg_configPOWER_1V8P_ACTIVE to 0."
#endif

/**
 * \brief When set to 1, the 1V8P is powered during sleep.
 * When set to 2 the rail configuration will be defined by the application
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configPOWER_1V8P_SLEEP
#define dg_configPOWER_1V8P_SLEEP          (2)
#endif


/**
 * \brief When set to 1, the Flash is powered off during sleep.
 */

#ifndef dg_configFLASH_POWER_OFF
// dg_configFLASH_POWER_OFF will only be allowed to be defined to 1 if dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8P or FLASH_CONNECTED_TO_1V8F
// and dg_configPOWER_1V8P_SLEEP == 1
#if (((dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8) && defined(dg_configPOWER_1V8_SLEEP) && (dg_configPOWER_1V8_SLEEP == 0)) || \
     ((dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8P || dg_configFLASH_CONNECTED_TO == FLASH_CONNECTED_TO_1V8F) \
       && defined(dg_configPOWER_1V8P_SLEEP) && (dg_configPOWER_1V8P_SLEEP == 0)))
#define dg_configFLASH_POWER_OFF    (1)
#else
#define dg_configFLASH_POWER_OFF    (0)
#endif

#endif /* dg_configFLASH_POWER_OFF */


/**
 * \brief Enable the Flash Auto-detection mode for QSPIC
 *
 * \warning THIS WILL GREATLY INCREASE THE CODE SIZE AND RETRAM USAGE!!! MAKE SURE YOUR PROJECT
 *          CAN SUPPORT THIS.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configFLASH_AUTODETECT
#define dg_configFLASH_AUTODETECT       (0)
#endif


/**
 * \brief Enable the Auto-detection mode for QSPIC2 device
 *
 * \warning THIS WILL GREATLY INCREASE THE CODE SIZE AND RETRAM USAGE!!! MAKE SURE YOUR PROJECT
 *          CAN SUPPORT THIS.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configQSPIC2_DEV_AUTODETECT
#define dg_configQSPIC2_DEV_AUTODETECT       (0)
#endif

#if dg_configQSPIC2_DEV_AUTODETECT == 0

/**
 * \brief The QSPI  2 Driver header file to include
 *
 * The header file must be in the include path of the compiler
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configQSPIC2_DEV_HEADER_FILE
#define dg_configQSPIC2_DEV_HEADER_FILE      "psram_aps6404jsq.h"
#endif

/**
 * \brief The QSPI 2 Driver configuration structure
 *
 * The configuration structure must be in the include path of the compiler
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configQSPIC2_DEV_CONFIG
#define dg_configQSPIC2_DEV_CONFIG psram_aps6404jsq_config
#endif

#endif /* dg_configQSPIC2_DEV_AUTODETECT  == 0 */

#if dg_configFLASH_AUTODETECT == 0

/**
 * \brief The Flash Driver header file to include
 *
 * The header file must be in the include path of the compiler
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configFLASH_HEADER_FILE
#if   (dg_configENABLE_DA1469x_AA_SUPPORT == 1)
#define dg_configFLASH_HEADER_FILE      "qspi_w25q80ew.h"
#else
#define dg_configFLASH_HEADER_FILE      "qspi_mx25u3235.h"
#endif /* dg_configUSE_FPGA, dg_configENABLE_DA1469x_AA_SUPPORT */
#endif /* dg_configFLASH_HEADER_FILE */

/**
 * \brief The Flash Driver configuration structure
 *
 * The configuration structure must be in the include path of the compiler
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configFLASH_CONFIG
#if   (dg_configENABLE_DA1469x_AA_SUPPORT == 1)
#define dg_configFLASH_CONFIG           flash_w25q80ew_config
#else
#define dg_configFLASH_CONFIG           flash_mx25u3235_config
#endif /* dg_configUSE_FPGA, dg_configENABLE_DA1469x_AA_SUPPORT */
#endif /* dg_configFLASH_CONFIG */

#endif /* dg_configFLASH_AUTODETECT == 0 */

/**
 * \brief Offset of the image if not placed at the beginning of QSPI Flash.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configIMAGE_FLASH_OFFSET
#define dg_configIMAGE_FLASH_OFFSET             (0)
#endif

/**
 * \brief Set the flash page size.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configFLASH_MAX_WRITE_SIZE
#define dg_configFLASH_MAX_WRITE_SIZE           (128)
#endif

/**
 * \brief Disable background operations.
 *
 * When enabled, outstanding QSPI operations will take place during sleep time
 * increasing the efficiency.
 *
 * - 1 : Disabled
 * - 0 : Enabled
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configDISABLE_BACKGROUND_FLASH_OPS
#define dg_configDISABLE_BACKGROUND_FLASH_OPS   (0)
#endif

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup CACHE_SETTINGS
 *
 * \brief Cache configuration settings
 * \{
 */
/* -------------------------------------- Cache settings ---------------------------------------- */

/**
 * \brief Set the size (in bytes) of the QSPI flash cacheable area.
 *
 * All reads from offset 0 up to (not including) offset dg_configCACHEABLE_QSPI_AREA_LEN
 * will be cached. In addition, any writes to this area will trigger cache flushing, to
 * avoid any cache incoherence.
 *
 * The size must be 64KB-aligned, due to the granularity of CACHE_CTRL2_REG[CACHE_LEN].
 *
 * Special values:
 *  *  0 : Turn off cache.
 *  * -1 : Don't configure cacheable area size (i.e. leave as set by booter).
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configCACHEABLE_QSPI_AREA_LEN
#define dg_configCACHEABLE_QSPI_AREA_LEN        (-1)
#endif

/**
 * \brief Set the associativity of the cache.
 *
 * Available values:
 *  0   /// direct-mapped
 *  1   /// 2-way set associative
 *  2   /// 4-way set associative
 *  3   /// leave as set by the ROM booter
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configCACHE_ASSOCIATIVITY
#define dg_configCACHE_ASSOCIATIVITY    (2)
#endif

/**
 * \brief Set the line size of the cache.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 *
 * Available values:
 *  0   /// 8 bytes
 *  1   /// 16 bytes
 *  2   /// 32 byte
 *  3   /// leave as set by the ROM booter
 */
#ifndef dg_configCACHE_LINESZ
#define dg_configCACHE_LINESZ           (0)
#endif
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup ARBITER_SETTINGS
 *
 * \brief Arbiter configuration settings
 * \{
 */
/* ------------------------------- Arbiter configuration settings ------------------------------- */

/**
 * \brief Custom arbiter configuration support
 * When defined, coex is configurable and priorities can be set:
 * -either manually, per mac, using coex api
 * -or automatically, by the PTIs provided by each mac
 *
 * When not defined, coex operates with the default/fixed priority scheme:
 * BLE traffic has always higher priority than FTDF.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configCOEX_ENABLE_CONFIG
#define dg_configCOEX_ENABLE_CONFIG     (0)
#endif

/**
 * \brief Arbiter statistics
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configCOEX_ENABLE_STATS
#define dg_configCOEX_ENABLE_STATS      (0)
#endif

/**
 * \brief Arbiter diagnostics enable
 *
 * This automatically enables arbiter diagnostic signals (when RF PD is on)
 *
 * See hw_coex.h for more information.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configCOEX_ENABLE_DIAGS
#define dg_configCOEX_ENABLE_DIAGS      (0)
#endif

/**
 * \brief Arbiter diagnostics mode
 *
 * This is the default mode for arbiter diagnostics.
 *
 * See hw_coex.h for more information.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configCOEX_DIAGS_MODE
#define dg_configCOEX_DIAGS_MODE        (HW_COEX_DIAG_MODE_3)
#endif
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup PERIPHERAL_SELECTION
 *
 * \brief Peripheral selection
 *
 * When enabled the specific low level driver is included in the compilation of the SDK.
 * - 0 : Disabled
 * - 1 : Enabled
 *
 * The default option can be overridden in the application configuration file.
 *
 * \{
   Driver                         | Setting                                | Default option
   ------------------------------ | -------------------------------------- | :------------------:
   AES HASH                       | dg_configUSE_HW_AES_HASH               | 0
   Radio MAC Arbiter              | dg_configUSE_HW_COEX                   | 0
   Clock and Power Manager        | dg_configUSE_HW_CPM                    | 1
   System                         | dg_configUSE_HW_SYS                    | 1
   Clock driver                   | dg_configUSE_HW_CLK                    | 1
   Power Domains driver           | dg_configUSE_HW_PD                     | 1
   Power Manager                  | dg_configUSE_HW_PMU                    | 1
   Direct Memory Access           | dg_configUSE_HW_DMA                    | 1
   Elliptic Curve Controller      | dg_configUSE_HW_ECC                    | 1
   Analog to Digital Converter    | dg_configUSE_HW_GPADC                  | 1
   General Purpose I/O            | dg_configUSE_HW_GPIO                   | 1
   Inter-Integrated Circuit       | dg_configUSE_HW_I2C                    | 0
   Infra Red Generator            | dg_configUSE_HW_IRGEN                  | 0
   ISO7816                        | dg_configUSE_HW_ISO7816                | 0
   Keyboard scanner               | dg_configUSE_HW_KEYBOARD_SCANNER       | 0
   LCD controller                 | dg_configUSE_HW_LCDC                   | 0
   OTP controller                 | dg_configUSE_HW_OTPC                   | 1
   QSPI controller                | dg_configUSE_HW_QSPI                   | 1
   QSPI2 controller               | dg_configUSE_HW_QSPI2                  | 0
   Quadrature decoder             | dg_configUSE_HW_QUAD                   | 0
   Radio module                   | dg_configUSE_HW_RF                     | 1
   State of charge module         | dg_configUSE_HW_SOC                    | 0
   Timer 0                        | dg_configUSE_HW_TIMER0                 | 0
   Timer 1                        | dg_configUSE_HW_TIMER1                 | 1
   Timer 2                        | dg_configUSE_HW_TIMER2                 | 0
   True Random Generator          | dg_configUSE_HW_TRNG                   | 1
   UART                           | dg_configUSE_HW_UART                   | 1
   USB charger                    | dg_configUSE_HW_USB_CHARGER            | 1
   HW charger                     | dg_configUSE_HW_CHARGER                | 0
   Wakeup timer                   | dg_configUSE_HW_WKUP                   | 1
   PDM interface                  | dg_configUSE_IF_PDM                    | 0
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */

/* -------------------------------- Peripherals (hw_*) selection -------------------------------- */

#ifndef dg_configUSE_HW_AES_HASH
#define dg_configUSE_HW_AES_HASH        (0)
#endif

#ifndef dg_configUSE_HW_COEX
#define dg_configUSE_HW_COEX            (0)
#endif

#ifndef dg_configUSE_HW_CPM
#define dg_configUSE_HW_CPM             (1)
#endif

#ifndef dg_configUSE_HW_SYS
#define dg_configUSE_HW_SYS             (1)
#endif

#ifndef dg_configUSE_HW_CLK
#define dg_configUSE_HW_CLK             (1)
#endif

#ifndef dg_configUSE_HW_PD
#define dg_configUSE_HW_PD              (1)
#endif

#ifndef dg_configUSE_HW_PMU
#define dg_configUSE_HW_PMU             (1)
#endif

#ifndef dg_configUSE_HW_DMA
#define dg_configUSE_HW_DMA             (1)
#endif

#ifndef dg_configUSE_HW_ECC
#define dg_configUSE_HW_ECC             (1)
#endif

#ifndef dg_configUSE_HW_GPADC
#define dg_configUSE_HW_GPADC           (1)
#endif

#ifndef dg_configUSE_HW_SDADC
#define dg_configUSE_HW_SDADC           (1)
#endif

#ifndef dg_configUSE_HW_GPIO
#define dg_configUSE_HW_GPIO            (1)
#endif

#ifndef dg_configUSE_HW_I2C
#define dg_configUSE_HW_I2C             (0)
#endif

#ifndef dg_configUSE_HW_IRGEN
#define dg_configUSE_HW_IRGEN           (0)
#endif

#ifndef dg_configUSE_HW_ISO7816
#define dg_configUSE_HW_ISO7816         (0)
#endif

#ifndef dg_configUSE_HW_KEYBOARD_SCANNER
#define dg_configUSE_HW_KEYBOARD_SCANNER        (0)
#endif

#ifndef dg_configUSE_HW_LCDC
#define dg_configUSE_HW_LCDC            (0)
#endif

#ifndef dg_configUSE_HW_OTPC
#define dg_configUSE_HW_OTPC            (1)
#endif

#ifndef dg_configUSE_HW_QSPI
#define dg_configUSE_HW_QSPI            (1)
#endif

#ifndef dg_configUSE_HW_QSPI2
#define dg_configUSE_HW_QSPI2           (0)
#endif

#if dg_configUSE_HW_QSPI2
/**
 * \brief The base address for accessing the Flash memory connected to QSPIC2
 *
 * The base address is used in qspi_automode. Automode is using a single zero-based address region
 * for accessing the QSPI Flash devices connected to both QSPI controllers (QSPIC1 and QSPIC2).
 * Two address sub-regions are defined:
 *    Address region 1: 0..dg_configQSPI2_FLASH_BASE_ADDR-1
 *    Address region 2: starting at dg_configQSPI2_FLASH_BASE_ADDR
 * When QSPI Flash address is in region 1 then the device connected to QSPIC1 is accessed.
 * When QSPI Flash address is in region 2 then the device connected to QSPIC2 is accessed.
 * The maximum region size handled by each QSPI controller in automode is 32MBytes. The
 * default value of dg_configQSPI2_FLASH_BASE_ADDR is 0x2000000 allowing 32MBytes region
 * for each controller. The default value can be overridden and set to the size of the QSPI
 * Flash device connected to QSPIC1 to allow a continuous address space for both devices.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
#ifndef dg_configQSPI2_FLASH_BASE_ADDR
#define dg_configQSPI2_FLASH_BASE_ADDR 0x2000000
#endif

#endif /* dg_configUSE_HW_QSPI2 */

#ifndef dg_configUSE_HW_QUAD
#define dg_configUSE_HW_QUAD            (0)
#endif


#if defined(dg_configUSE_HW_RF)
#error "dg_configUSE_HW_RF is not supported in DA14690 devices"
#endif

#ifndef dg_configUSE_HW_SENSOR_NODE
#define dg_configUSE_HW_SENSOR_NODE     (0)
#endif

#ifndef dg_configUSE_HW_SOC
#define dg_configUSE_HW_SOC             (0)
#endif

#ifndef dg_configUSE_HW_SPI
#define dg_configUSE_HW_SPI             (0)
#endif

#ifdef dg_configUSE_HW_TEMPSENS
#error "This is a deprecated configuration hence not to be defined by an application."
#endif

#ifndef dg_configUSE_HW_TIMER
#define dg_configUSE_HW_TIMER           (1)
#endif

#ifndef dg_configUSE_HW_RTC
#define dg_configUSE_HW_RTC           (1)
#endif


#ifndef dg_configUSE_HW_TRNG
#define dg_configUSE_HW_TRNG            (1)
#endif

#ifndef dg_configUSE_HW_UART
#define dg_configUSE_HW_UART            (1)
#endif

#ifndef dg_configUSE_HW_USB_CHARGER
#define dg_configUSE_HW_USB_CHARGER     (1)
#endif

#ifndef dg_configUSE_HW_CHARGER
#define dg_configUSE_HW_CHARGER         (0)
#endif

#ifndef dg_configUSE_HW_WKUP
#define dg_configUSE_HW_WKUP            (1)
#endif

#ifndef dg_configUSE_HW_ERM
#define dg_configUSE_HW_ERM             (0)
#endif
#ifndef dg_configUSE_HW_LRA
#define dg_configUSE_HW_LRA             (0)
#endif

#ifndef dg_configUSE_HW_USB
#define dg_configUSE_HW_USB             (1)
#endif

#ifndef dg_configUSE_IF_PDM
#define dg_configUSE_IF_PDM             (0)
#endif

#ifndef dg_configUSE_HW_CACHE
#define dg_configUSE_HW_CACHE           (1)
#endif

#ifndef dg_configUSE_HW_PDC
#define dg_configUSE_HW_PDC             (1)
#endif

#ifndef dg_configUSE_HW_MPU
#define dg_configUSE_HW_MPU             (0)
#endif

#if (dg_configUSE_HW_GPADC == 0)
#error "dg_configUSE_HW_GPADC should be 1 in order to properly trim the PLL clock"
#endif
/* ---------------------------------------------------------------------------------------------- */


/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/* ------------------------------- USB settings ------------------------------------------------ */

/**
 * \addtogroup USB_SETTINGS
 *
 * \brief USB DMA enable configuration settings
 *
 * The macros in this section are used to enable the DMA with USB and to define the two possible end point to use the DMA for data transfers.
 * \{
 */

/**
 * \brief Enable the DMA for reading/writing data to USB EP.\n
 * By default the USB DMA is not enabled.\n
 * To enable the DMA for the USB, set this the macro to value (1) in the custom_config_xxx.h file.\n
 * When the USB DMA is enabled, the default end points with DMA are EP1 and EP2. \n
 * It is possible only one TX and one RX end point to use DMA.\n
 * User can choose a different pair of end points to use the DMA as needed according to app requirements.\n
 * To change the end points using DMA, set in the the custom_config_xxx.h file the desired values for the macros:
 * \par \c dg_configUSB_TX_DMA_EP
 * valid values: 1,3,5\n
 * default value: 1
 * \par \c dg_configUSB_RX_DMA_EP
 * valid values: 2,4,6\n
 * default value: 2
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUSB_DMA_SUPPORT
#define dg_configUSB_DMA_SUPPORT                (0)
#endif


/**
 * \brief The USB TX end point (D-->H) to enable the DMA.\n
 * User can choose a different pair of end points to use the DMA as needed according to app requirements.\n
 * To change the TX end point using DMA, set in the the custom_config_xxx.h file the desired value for this macros.
 * \par \c dg_configUSB_TX_DMA_EP
 * valid values: 1,3,5\n
 * default value: 1
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUSB_TX_DMA_EP
#define  dg_configUSB_TX_DMA_EP                 (1)
#endif

/**
 * \brief The USB RX end point (D-->H) to enable the DMA.\n
 * User can choose a different pair of end points to use the DMA as needed according to app requirements.\n
 * To change the RX end point using DMA, set in the the custom_config_xxx.h file the desired value for this macros.
 * \par \c dg_configUSB_RX_DMA_EP
 * valid values: 2,4,6\n
 * default value: 2
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUSB_RX_DMA_EP
#define  dg_configUSB_RX_DMA_EP                 (2)
#endif

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */



/* ------------------------------- WKUP settings ------------------------------------------------ */

/**
 * \addtogroup WKUP_SETTINGS
 *
 * \brief WKUP configuration settings
 * \{
 */

/* ------------------------------- LATCH settings ------------------------------------------------ */

/**
 * \addtogroup WKUP_LATCH_SETTINGS
 *
 * \brief WKUP LATCH configuration settings
 * \{
 */

/**
 * \brief WKUP latch wakeup (io) source support
 *
 * \note In chip revision DA14680/1-01, this feature is implemented in s/w
 * \note In chip revision DA14682/3-00, DA15XXX-00, this feature is implemented in h/w
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#if (!defined(dg_configLATCH_WKUP_SOURCE)) || (dg_configUSE_HW_WKUP == 0)
        #undef dg_configLATCH_WKUP_SOURCE
        #define dg_configLATCH_WKUP_SOURCE     (0)
#endif

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/* ------------------------------- UART settings ------------------------------------------------ */

/**
 * \addtogroup UART_SETTINGS
 *
 * \brief UART FIFO configuration settings
 * \{
 */

/* ------------------------------- FIFO settings ------------------------------------------------ */

/**
 * \addtogroup UART_FIFO_SETTINGS
 *
 * \brief UART FIFO configuration settings
 * \{
 */

/**
 * \brief Software FIFO support
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART_SOFTWARE_FIFO
#define dg_configUART_SOFTWARE_FIFO     (0)
#endif

/**
 * \brief UART1's software FIFO size
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART1_SOFTWARE_FIFO_SIZE
#   define dg_configUART1_SOFTWARE_FIFO_SIZE    (0)
#endif

/**
 * \brief UART2's software FIFO size
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART2_SOFTWARE_FIFO_SIZE
#   define dg_configUART2_SOFTWARE_FIFO_SIZE    (0)
#endif

/**
 * \brief UART3's software FIFO size
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART3_SOFTWARE_FIFO_SIZE
#   define dg_configUART3_SOFTWARE_FIFO_SIZE    (0)
#endif
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/* ------------------------------- Circular DMA for RX settings --------------------------------- */

/**
 * \addtogroup UART_CIRCULAR_DMA_FOR_RX_SETTINGS
 *
 * \brief UART FIFO configuration settings
 * \{
 */

/**
 * \brief Circular DMA support for RX
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART_RX_CIRCULAR_DMA
#define dg_configUART_RX_CIRCULAR_DMA   (0)
#endif

/**
 * \brief UART1's Circular DMA buffer size for RX
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART1_RX_CIRCULAR_DMA_BUF_SIZE
#define dg_configUART1_RX_CIRCULAR_DMA_BUF_SIZE (0)
#endif

/**
 * \brief UART2's Circular DMA buffer size for RX
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART2_RX_CIRCULAR_DMA_BUF_SIZE
#define dg_configUART2_RX_CIRCULAR_DMA_BUF_SIZE (0)
#endif

/**
 * \brief UART3's Circular DMA buffer size for RX
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configUART3_RX_CIRCULAR_DMA_BUF_SIZE
#define dg_configUART3_RX_CIRCULAR_DMA_BUF_SIZE (0)
#endif

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup RF_DRIVER_SETTINGS
 *
 * \brief Doxygen documentation is not yet available for this module.
 *        Please check the source code file(s)
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 * \{
 */
/* ------------------------------- RF driver configuration settings ----------------------------- */

/* Set to 1 to enable the recalibration procedure */
#if   ( (dg_configDEVICE == DEVICE_DA1469x) && defined(CONFIG_USE_BLE) )
#ifndef dg_configRF_ENABLE_RECALIBRATION
#define dg_configRF_ENABLE_RECALIBRATION        (1)
#endif
#endif

/* Minimum time before performing an RF recalibration, in FreeRTOS scheduler ticks */
#ifndef dg_configRF_MIN_RECALIBRATION_TIMEOUT
#if (dg_configUSE_LP_CLK == LP_CLK_32000)
#define dg_configRF_MIN_RECALIBRATION_TIMEOUT   (1000) // ~2 secs
#elif (dg_configUSE_LP_CLK == LP_CLK_32768)
#define dg_configRF_MIN_RECALIBRATION_TIMEOUT   (1024) // ~2 secs
#elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
#define dg_configRF_MIN_RECALIBRATION_TIMEOUT   (1000)
#else /* LP_CLK_ANY */
// Must be defined in the custom_config_<>.h file. Should result in ~2sec.
#endif
#endif

/* Maximum time before performing an RF recalibration, in FreeRTOS scheduler ticks
 * If this time has elapsed, and RF is about to be powered off, recalibration will
 * be done unconditionally. Set to 0 to disable this functionality */
#ifndef dg_configRF_MAX_RECALIBRATION_TIMEOUT
#define dg_configRF_MAX_RECALIBRATION_TIMEOUT   (0) // Disabled
#endif

/* Timeout value (in FreeRTOS scheduler ticks) for timer to initiate RF recalibration.
 * This will happen at ANY TIME RF is ON and CONFIGURED, EVEN IF A MAC IS RX/TXing DURING
 * THIS TIME, in contrary to dg_configRF_MAX_RECALIBRATION_TIMEOUT, that will be
 * performed ONLY when powering off RF.
 * This is intended for applications where RF is always on, so there is no
 * opportunity to be recalibrated the normal way (i.e. during poweroff)
 *
 * Set to 0 to disable this functionality */
#ifndef dg_configRF_RECALIBRATION_TIMER_TIMEOUT
#define dg_configRF_RECALIBRATION_TIMER_TIMEOUT (0) // Disabled
#endif


/* Minimum temp difference before performing an RF recalibration, in oC*/
#ifndef dg_configRF_MIN_RECALIBRATION_TEMP_DIFF
#ifdef CONFIG_USE_FTDF
        #define dg_configRF_MIN_RECALIBRATION_TEMP_DIFF (5)
#else
        #define dg_configRF_MIN_RECALIBRATION_TEMP_DIFF (10)
#endif
#endif

/* Duration of recalibration procedure, in lp clk cycles */
#ifndef dg_configRF_RECALIBRATION_DURATION
# if defined(CONFIG_USE_FTDF) && defined(CONFIG_USE_BLE)
#  if ((dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768))
#   define dg_configRF_RECALIBRATION_DURATION (230)
#  elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
#   define dg_configRF_RECALIBRATION_DURATION cm_rcx_us_2_lpcycles((uint32_t)(230 * 30.5))
#  else /* LP_CLK_ANY */
    // Must be defined in the custom_config_<>.h file. It should be ~7msec.
#  endif
# else
#  if ((dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768))
#   define dg_configRF_RECALIBRATION_DURATION (131)
#  elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
#   define dg_configRF_RECALIBRATION_DURATION cm_rcx_us_2_lpcycles((uint32_t)(131 * 30.5))
#  else /* LP_CLK_ANY */
    // Must be defined in the custom_config_<>.h file. It should be ~4msec.
#  endif
# endif
#endif

#ifndef dg_configRF_IFF_CALIBRATION_TIMEOUT
#if ((dg_configUSE_LP_CLK == LP_CLK_32000) || (dg_configUSE_LP_CLK == LP_CLK_32768))
        #define dg_configRF_IFF_CALIBRATION_TIMEOUT     (40)
#elif (dg_configUSE_LP_CLK == LP_CLK_RCX)
        #define dg_configRF_IFF_CALIBRATION_TIMEOUT     cm_rcx_us_2_lpcycles((uint32_t)(40 * 30.5))
#else /* LP_CLK_ANY */
// Must be defined in the custom_config_<>.h file. It should be ~1.2msec.
#  endif
#endif

#ifndef dg_configUSE_SYS_TCS
#define dg_configUSE_SYS_TCS (1)
#endif
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/* ----------------------------------- BLE hooks configuration ---------------------------------- */

/* Name of the function that is called to block BLE from sleeping under certain conditions
 *
 * The function must be declared as:
 *      unsigned char my_block_sleep(void);
 * The return value of the function controls whether the BLE will be allowed to go to sleep or not,
 *      0: the BLE may go to sleep, if possible
 *      1: the BLE is not allowed to go to sleep. The caller (BLE Adapter) may block or not,
 *         depending on the BLE stack status.
 *
 * dg_configBLE_HOOK_BLOCK_SLEEP should be set to my_block_sleep in this example.
 */
#ifndef dg_configBLE_HOOK_BLOCK_SLEEP
/* Already undefined - Nothing to do. */
#endif

/* Name of the function that is called to modify the PTI value (Payload Type Indication) when
 * arbitration is used
 *
 * Arbitration is needed only when another external radio is present. The function must be declared
 * as:
 *      unsigned char my_pti_modify(void);
 * Details for the implementation of such a function will be provided when the external radio
 * arbitration functionality is integrated.
 *
 * dg_configBLE_HOOK_PTI_MODIFY should be set to my_pti_modify in this example.
 *
 * See also the comment about the <BLE code hooks> in ble_config.h for more info.
 */
#ifndef dg_configBLE_HOOK_PTI_MODIFY
/* Already undefined - Nothing to do. */
#endif

/*------------------------------------------------------------------------------------------------*/

/*
 * \brief When set to 1, the LEDs are used from M33.
 *        When set to 0, the LEDs are used from SNC.
 */
#ifndef dg_configM33_USES_LEDS
#define dg_configM33_USES_LEDS         (1)
#endif

/*
 * \brief When set to 1, the GPIO configuration becomes static, i.e. it does not change during runtime.
 *        When set to 0, the GPIO configuration can change during runtime.
 *
 * \note If SNC is enabled (i.e. dg_configUSE_HW_SENSOR_NODE == 1), the GPIO configuration must be
 *       static, thus dg_configUSE_STATIC_IO_CONFIG is set to 1
 */
#if dg_configUSE_HW_SENSOR_NODE
#undef dg_configUSE_STATIC_IO_CONFIG
#define dg_configUSE_STATIC_IO_CONFIG (1)
#else
#ifndef dg_configUSE_STATIC_IO_CONFIG
#define dg_configUSE_STATIC_IO_CONFIG (0)
#endif /* dg_configUSE_STATIC_IO_CONFIG */
#endif /* dg_configUSE_HW_SENSOR_NODE */

/*------------------------------------ BOARDS DEFINITIONS ----------------------------------------*/

#include dg_configUSE_Board

/* ---------------------------------------------------------------------------------------------- */

/* ----------------------------------- RF FEM CONFIGURATION ------------------------------------- */

#include "bsp_fem.h"

/* ---------------------------------------------------------------------------------------------- */


/* ----------------------------------- DEBUG CONFIGURATION -------------------------------------- */

#include "bsp_debug.h"

/* ---------------------------------------------------------------------------------------------- */

/* ---------------------------------- MEMORY LAYOUT CONFIGURATION ------------------------------- */

#include "bsp_memory_defaults.h"

/* ---------------------------------------------------------------------------------------------- */


/**
 * \}
 */

#endif /* BSP_DEFAULTS_H_ */

/**
\}
\}
*/
