/**
 * \addtogroup PLA_BSP_CONFIG
 * \{
 * \addtogroup BSP_MEMORY_DEFAULTS Memory Default Configuration Values
 *
 * \brief BSP memory default configuration values
 *
 * The following tags are used to describe the type of each configuration option.
 *
 * - **\bsp_config_option_build**        : To be changed only in the build configuration
 *                                                of the project ("Defined symbols -D" in the
 *                                                preprocessor options).
 *
 * - **\bsp_config_option_app**          : To be changed only in the custom_config*.h
 *                                                project files.
 *
 * - **\bsp_config_option_expert_only**  : To be changed only by an expert user.
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file bsp_memory_defaults.h
 *
 * @brief Board Support Package. Memory Configuration file default values.
 *
 * Copyright (C) 2018-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef BSP_MEMORY_DEFAULTS_H_
#define BSP_MEMORY_DEFAULTS_H_

#define PARTITION2(...)
#include "partition_table.h"
#undef PARTITION2

/* ---------------------------------- Heap size configuration ----------------------------------- */

/**
 * \brief Heap size for used libc malloc()
 *
 * Specifies the amount of RAM that will be used as heap for libc malloc() function.
 * It can be configured in bare metal projects to match application's requirements.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
// #ifndef __HEAP_SIZE
// # if defined(CONFIG_RETARGET) || defined(CONFIG_RTT)
// #  define __HEAP_SIZE  0x0600
// # else
// #  define __HEAP_SIZE  0x0100
// # endif
// #endif

/* flag used by linker scripts */
// #if (dg_configBLACK_ORCA_IC_REV != BLACK_ORCA_IC_REV_A)
// #define __HEAP_IS_LESS_THAN_0x200       (__HEAP_SIZE < 0x200)
// #endif

// #if dg_configUSE_FPGA_AD9361_RADIO
// #undef __HEAP_SIZE
// #define __HEAP_SIZE 0x0c00
// #endif

/* ---------------------------------------------------------------------------------------------- */

/* --------------------------------- Stack size configuration ----------------------------------- */

/**
 * \brief Stack size for main() function and interrupt handlers.
 *
 * Specifies the amount of RAM that will be used as stack for the main() function and the interrupt
 * handlers.
 *
 * \bsp_default_note{\bsp_config_option_app, \bsp_config_option_expert_only}
 */
// #ifndef __STACK_SIZE
// #define __STACK_SIZE  0x0200
// #endif

// #if dg_configUSE_FPGA_AD9361_RADIO
// #undef __STACK_SIZE
// #define __STACK_SIZE 0x0400
// #endif

/* ---------------------------------------------------------------------------------------------- */

/* ---------------------------------- MEMORY LAYOUT CONFIGURATION ------------------------------- */

/**
 * \addtogroup MEMORY_LAYOUT_SETTINGS
 *
 * \brief Memory layout configuration settings.
 * \{
 */

/**
 * \brief Size of the RETAINED_RAM_UNINIT section, in bytes.
 *
 * This section is not initialized during startup by either the bootloader or
 * the application. It can be therefore used to maintain debug or other relevant
 * information that will no be lost after reset. It should be guaranteed that
 * both the bootloader (if any) and the application are using the same value for
 * this option (or otherwise the booloader can corrupt the contents of the section).
 * To use this section for a specific variable, use the __RETAINED_UNINIT attribute.
 */
#ifndef dg_config_RETAINED_UNINIT_SECTION_SIZE
#define dg_config_RETAINED_UNINIT_SECTION_SIZE                  (128)
#endif

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup QSPI_PROJECT_MEMORY_LAYOUT_SETTINGS
 *
 * \brief Memory layout configuration settings for a QSPI project
 * \{
 */

/**
 * \brief Code size in QSPI projects for DA1469x.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configQSPI_CODE_SIZE_AA
#define dg_configQSPI_CODE_SIZE_AA                              (370 * 1024) /* Take into account CMI firmware size */
#endif

/**
 * \brief Maximum size (in bytes) of image in the QSPI flash.
 *
 * The image in the QSPI flash contains the text (code + const data) and any other initialized data.
 *
 * \note This size should not be larger than the flash partition where the image is stored.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configQSPI_MAX_IMAGE_SIZE
#define dg_configQSPI_MAX_IMAGE_SIZE                            ( IMAGE_PARTITION_SIZE )
#endif

#if dg_configQSPI_MAX_IMAGE_SIZE < dg_configQSPI_CODE_SIZE_AA
#error "dg_configQSPI_MAX_IMAGE_SIZE cannot be smaller than dg_configQSPI_CODE_SIZE_AA"
#endif

/**
 * \brief RAM-block size in cached mode for DA1469x.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configQSPI_CACHED_RAM_SIZE_AA
#ifdef dg_configENABLE_MTB
/* reserve last 8KiB of RAM for MTB */
#define dg_configQSPI_CACHED_RAM_SIZE_AA                        (504 * 1024)
#else
#define dg_configQSPI_CACHED_RAM_SIZE_AA                        (512 * 1024)
#endif
#endif
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/**
 * \addtogroup RAM_PROJECT_MEMORY_LAYOUT_SETTINGS
 *
 * \brief Memory layout configuration settings for a RAM project
 * \{
 */
/**
 * \brief Code and RAM size in RAM projects for DA1469xAA.
 *
 * \bsp_default_note{\bsp_config_option_app,}
 */
#ifndef dg_configRAM_CODE_SIZE_AA
/* CODE and RAM are merged into a single RAM section */
#define dg_configRAM_CODE_SIZE_AA                      (512 * 1024)
#endif /* dg_configRAM_CODE_SIZE_AA */
/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

/* ---------------------------------------------------------------------------------------------- */

/**
 * \}
 */

#endif /* BSP_MEMORY_DEFAULTS_H_ */


/**
\}
\}
*/
