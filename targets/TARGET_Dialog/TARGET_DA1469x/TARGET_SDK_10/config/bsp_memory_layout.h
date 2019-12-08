/**
 ****************************************************************************************
 *
 * @file bsp_memory_layout.h
 *
 * @brief Board Support Package. System Configuration file default values.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef BSP_MEMORY_LAYOUT_H_
#define BSP_MEMORY_LAYOUT_H_

/*************************************************************************************************\
 * Default configuration for retention RAM
 */

#if !defined(RELEASE_BUILD) && (dg_configOPTIMAL_RETRAM == 1)
        /* WARNING: retRAM optimizations are disabled in DEBUG builds! */
        #undef dg_configOPTIMAL_RETRAM
        #define dg_configOPTIMAL_RETRAM         (0)
#elif (dg_configEXEC_MODE != MODE_IS_CACHED)
        /* WARNING: retRAM optimizations are not applicable in MIRRORED mode! */
        #undef dg_configOPTIMAL_RETRAM
        #define dg_configOPTIMAL_RETRAM         (0)
#endif

#if (dg_configOPTIMAL_RETRAM == 0)
        #undef  dg_configMEM_RETENTION_MODE
        #define dg_configMEM_RETENTION_MODE     (0)


#endif

/*************************************************************************************************\
 * Memory layout configuration
 */
#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
        #if (dg_configEXEC_MODE == MODE_IS_CACHED)
                #define CODE_SIZE       dg_configQSPI_CODE_SIZE_AA
                #define RAM_SIZE        dg_configQSPI_CACHED_RAM_SIZE_AA
        #else // MIRRORED
                #error "QSPI mirrored mode is not supported!"
        #endif
#elif (dg_configCODE_LOCATION == NON_VOLATILE_IS_NONE)
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
        #pragma message "RAM cached mode is not supported! Resetting to RAM (mirrored) mode!"
        #undef dg_configEXEC_MODE
        #define dg_configEXEC_MODE      MODE_IS_RAM
#endif

        #define CODE_SIZE       dg_configRAM_CODE_SIZE_AA

#if (CODE_SZ > 512)
#error "The used CODE_SZ value exceed the total amount of RAM!"
#endif
#else
        #error "Unknown configuration..."
#endif




#endif /* BSP_MEMORY_LAYOUT_H_ */
