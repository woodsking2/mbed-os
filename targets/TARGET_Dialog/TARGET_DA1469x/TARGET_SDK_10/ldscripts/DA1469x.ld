/*
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define NON_VOLATILE_IS_OTP             0       // Code is in OTP
#define NON_VOLATILE_IS_FLASH           1       // Code is in QSPI Flash
#define NON_VOLATILE_IS_NONE            2       // Debug mode! Code is in RAM!

#define MODE_IS_MIRRORED                0
#define MODE_IS_CACHED                  1
#define MODE_IS_RAM                     MODE_IS_MIRRORED

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

#define IVT_AREA_OVERHEAD               0x200

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_NONE)
        #define CODE_BASE_ADDRESS               (0x0)   /* Remapped address will be 0x800000 */
        #define CODE_SZ                         (CODE_SIZE)

        MEMORY
        {
                /* CODE and RAM are merged into a single RAM section */
                RAM (rx) : ORIGIN = CODE_BASE_ADDRESS, LENGTH = CODE_SZ
        }
#elif (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
        #define CODE_BASE_ADDRESS               (0x16002400)   /* Remapped address will be at any offset in
                                                                QSPI (0x16000000) according to the image header  */
        #define CODE_SZ                         (CODE_SIZE)
        #define RAM_BASE_ADDRESS                (0x20000000 + IVT_AREA_OVERHEAD)
        #define RAM_SZ                          (RAM_SIZE - IVT_AREA_OVERHEAD)

        MEMORY
        {
                ROM (rx) : ORIGIN = CODE_BASE_ADDRESS, LENGTH = CODE_SZ
                RAM (rw) : ORIGIN = RAM_BASE_ADDRESS,  LENGTH = RAM_SZ
        }
#else
        #error "Unknown code location type..."
#endif
/* Library configurations */
GROUP(libgcc.a libc.a libm.a libnosys.a)

#if (dg_configEXEC_MODE == MODE_IS_CACHED)
#define SNC_SECTION_SIZE                (__snc_section_end__ - __snc_section_start__)
#define RETENTION_RAM_INIT_SIZE         (__retention_ram_init_end__ - __retention_ram_init_start__)
#define NON_RETENTION_RAM_INIT_SIZE     (__non_retention_ram_init_end__ - __non_retention_ram_init_start__)
#else
/* CODE and RAM are merged into a single RAM section */
#define ROM                             RAM
#endif

#if ( dg_configUSE_SEGGER_FLASH_LOADER == 1 )
#define QSPI_FLASH_ADDRESS 0x0
#define QSPI_FW_BASE_OFFSET 0x2000
#define QSPI_FW_IVT_OFFSET  0x400
#define QSPI_FW_BASE_ADDRESS (QSPI_FLASH_ADDRESS + QSPI_FW_BASE_OFFSET)
#define QSPI_FW_IVT_BASE_ADDRESS (QSPI_FW_BASE_ADDRESS + QSPI_FW_IVT_OFFSET)
#else
#define QSPI_FW_IVT_BASE_ADDRESS 0x0
#endif /* dg_configUSE_SEGGER_FLASH_LOADER */

ENTRY(Reset_Handler)

SECTIONS
{
        .init_text :
#if ( dg_configUSE_SEGGER_FLASH_LOADER == 1 )
        AT ( QSPI_FW_IVT_BASE_ADDRESS)
#endif /* dg_configUSE_SEGGER_FLASH_LOADER */
        {
                KEEP(*(.isr_vector))
                /* Interrupt vector remmaping overhead */
                . = 0x200;
                __Vectors_End = .;
                __Vectors_Size = __Vectors_End - __isr_vector;
                *(text_reset*)
        } > ROM

        .text :
        {
                /* Optimize the code of specific libgcc files by executing them
                 * from the .retention_ram_init section. */
                *(EXCLUDE_FILE(*libnosys.a:sbrk.o
                               *libgcc.a:_aeabi_uldivmod.o
                               *libgcc.a:_muldi3.o
                               *libgcc.a:_dvmd_tls.o
                               *libgcc.a:bpabi.o
                               *libgcc.a:_udivdi3.o
                               *libgcc.a:_clzdi2.o
                               *libgcc.a:_clzsi2.o) .text*)

                . = ALIGN(4);

#ifdef CONFIG_USE_BLE
#if (dg_configEXEC_MODE != MODE_IS_CACHED)
                . = ALIGN(0x10000); /* Code region should start at 1Kb boundary and
                                     * should use different RAM cell than SYSCPU/SNC */
                cmi_fw_dst_addr = .;
#endif

                /*
                 * Section used to store the CMAC FW.
                 * Code should copy this FW to address 'cmi_fw_dst_addr' and
                 * configure the memory controller accordingly.
                 */
                __cmi_fw_area_start = .;
                KEEP(*(.cmi_fw_area*))
                __cmi_fw_area_end = .;

#if (dg_configEXEC_MODE != MODE_IS_CACHED)
                . = ALIGN(0x400); /* CMI end region ends at 1Kb boundary */
                __cmi_section_end__ = . - 1;
#endif
#endif /* CONFIG_USE_BLE */
            KEEP(*(.init))
            KEEP(*(.fini))

            /* .ctors */
            *crtbegin.o(.ctors)
            *crtbegin?.o(.ctors)
            *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
            *(SORT(.ctors.*))
            *(.ctors)

            /* .dtors */
            *crtbegin.o(.dtors)
            *crtbegin?.o(.dtors)
            *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
            *(SORT(.dtors.*))
            *(.dtors)

            . = ALIGN(4);
            /* preinit data */
            PROVIDE_HIDDEN (__preinit_array_start = .);
            KEEP(*(.preinit_array))
            PROVIDE_HIDDEN (__preinit_array_end = .);

            . = ALIGN(4);
            /* init data */
            PROVIDE_HIDDEN (__init_array_start = .);
            KEEP(*(SORT(.init_array.*)))
            KEEP(*(.init_array))
            PROVIDE_HIDDEN (__init_array_end = .);

            . = ALIGN(4);
            /* finit data */
            PROVIDE_HIDDEN (__fini_array_start = .);
            KEEP(*(SORT(.fini_array.*)))
            KEEP(*(.fini_array))
            PROVIDE_HIDDEN (__fini_array_end = .);

            *(.rodata*)

            KEEP(*(.eh_frame*))            
    } > ROM
   
    .ARM.extab :
    {
            *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > ROM

    __exidx_start = .;
    .ARM.exidx :
    {
            *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > ROM
    __exidx_end = .;

    /* To copy multiple ROM to RAM sections,
        * uncomment .copy.table section and,
        * define __STARTUP_COPY_MULTIPLE in startup_ARMCMx.S */

    .copy.table :
    {
            . = ALIGN(4);
            __copy_table_start__ = .;
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
            LONG (__etext)
            LONG (__retention_ram_init_start__)
            LONG (RETENTION_RAM_INIT_SIZE)

            LONG (__etext + (RETENTION_RAM_INIT_SIZE))
            LONG (__snc_section_start__)
            LONG (SNC_SECTION_SIZE)

            LONG (__etext + (RETENTION_RAM_INIT_SIZE) + (SNC_SECTION_SIZE))
            LONG (__non_retention_ram_init_start__)
            LONG (NON_RETENTION_RAM_INIT_SIZE)
#endif
            __copy_table_end__ = .;
    } > ROM

    /* To clear multiple BSS sections,
        * uncomment .zero.table section and,
        * define __STARTUP_CLEAR_BSS_MULTIPLE in startup_ARMCMx.S */

    .zero.table :
    {
            . = ALIGN(4);
            __zero_table_start__ = .;
            LONG (__bss_start__)
            LONG (__bss_end__ - __bss_start__)
            LONG (__retention_ram_zi_start__)
            LONG (__retention_ram_zi_end__ - __retention_ram_zi_start__)
            LONG (__HeapBase)
            LONG (__HeapLimit - __HeapBase)
            __zero_table_end__ = .;
    } > ROM

    __etext = .;
    __data_start__ = __etext;
    __data_end__ = __data_start__ + (RETENTION_RAM_INIT_SIZE) + (SNC_SECTION_SIZE) + (NON_RETENTION_RAM_INIT_SIZE);          

    /*
    * Retention ram that should not be initialized during startup.
    * On QSPI cached images, it should be at a fixed RAM address for both
    * the bootloader and the application, so that the bootloader will not alter
    * those data due to conflicts between its .data/.bss sections with application's
    * .retention_ram_uninit section.
    * - On QSPI images it is relocated to the first RAM address after IVT_AREA_OVERHEAD
    *       with fixed size of dg_config_RETAINED_UNINIT_SECTION_SIZE bytes.
    * - On RAM images the section is not located at a fixed location.
    */
    .retention_ram_uninit (NOLOAD) :
    {
            __retention_ram_uninit_start__ = .;
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
            ASSERT( . == ORIGIN(RAM), ".retention_ram_uninit section moved!");
#endif /* (dg_configEXEC_MODE == MODE_IS_CACHED) */
            KEEP(*(nmi_info))
            KEEP(*(hard_fault_info))
            KEEP(*(retention_mem_uninit))

            ASSERT( . <= __retention_ram_uninit_start__ + dg_config_RETAINED_UNINIT_SECTION_SIZE,
                    "retention_ram_uninit section overflowed! Increase dg_config_RETAINED_UNINIT_SECTION_SIZE.");

            . = __retention_ram_uninit_start__ + dg_config_RETAINED_UNINIT_SECTION_SIZE;
            __retention_ram_uninit_end__ = .;
    } > RAM
    /*
        * Initialized retention RAM
        */
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
        .retention_ram_init : AT (QSPI_FW_IVT_BASE_ADDRESS +__etext)
#else
        /*
         * No need to add this to the copy table,
         * copy will be done by the debugger.
         */
        .retention_ram_init :
#endif
        {
                __retention_ram_init_start__ = .;
                . = ALIGN(4); /* Required by copy table */

            /*
                * Retained .text sections moved to RAM that need to be initialized
                */
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
            /* Retained code exists only in QSPI projects */
            *(text_retained)
#endif
            /* Make the '.text' section of specific libgcc files retained, to
                * optimize perfomance */
            *libnosys.a:sbrk.o (.text*)
            *libgcc.a:_aeabi_uldivmod.o (.text*)
            *libgcc.a:_muldi3.o (.text*)
            *libgcc.a:_dvmd_tls.o (.text*)
            *libgcc.a:bpabi.o (.text*)
            *libgcc.a:_udivdi3.o (.text*)
            *libgcc.a:_clzdi2.o (.text*)
            *libgcc.a:_clzsi2.o (.text*)

            /*
                * Retained .data sections that need to be initialized
                */

            /* Retained data */
            *(privileged_data_init)
            *(.retention)

            *(vtable)

            *(retention_mem_init)
            *(retention_mem_const)

            *libg_nano.a:* (.data*)
            *libnosys.a:* (.data*)
            *libgcc.a:* (.data*)
            *libble_stack_da1469x.a:* (.data*)
            *crtbegin.o (.data*)

            KEEP(*(.jcr*))
            . = ALIGN(4); /* Required by copy table */
            /* All data end */
            __retention_ram_init_end__ = .;
    } > RAM

    /*
        * Zero-initialized retention RAM
        */
    .retention_ram_zi (NOLOAD) :
    {
            __retention_ram_zi_start__ = .;

            *(privileged_data_zi)
            *(retention_mem_zi)

            *libg_nano.a:* (.bss*)
            *libnosys.a:* (.bss*)
            *libgcc.a:* (.bss*)
            *libble_stack_da1469x.a:* (.bss*)
            *crtbegin.o (.bss*)            
            
            __retention_ram_zi_end__ = .;            
    } > RAM

    /*
        * Sensor node controller section, used for SNC code and possibly for SNC data.
        */
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
    .snc_section : AT (QSPI_FW_IVT_BASE_ADDRESS + __etext + (RETENTION_RAM_INIT_SIZE))
#else
        /*
         * No need to add this to the copy table,
         * copy will be done by the debugger.
         */
        .snc_section :
#endif
    {
            __snc_section_start__ = .;
            . = ALIGN(4); /* Required by copy table */
            KEEP(*(.snc_region*))
            . = ALIGN(4); /* Required by copy table */
            __snc_section_end__ = .;
    } > RAM
    
#ifdef CONFIG_USE_BLE
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
        /*
         * CMAC interface section
         */
        .cmi_section (NOLOAD) :
        {
                __cmi_section_start__ = .;

            . = ALIGN(0x400); /* Code region should start at 1Kb boundary */

            /*
                * The actual CMAC code (copied from '.cmi_fw_area')
                * will be running here.
                */
            cmi_fw_dst_addr = .;

            /*
                * Create space to copy/expand the CMAC image to.
                */
            . += (__cmi_fw_area_end - __cmi_fw_area_start);

            . = ALIGN(0x400); /* CMI end region ends at 1Kb boundary */

            __cmi_section_end__ = . - 1;
    } > RAM
#endif /* (dg_configEXEC_MODE == MODE_IS_CACHED) */
#endif /* CONFIG_USE_BLE */
    
    __non_retention_ram_start__ = .;

    /*
        * Initialized RAM area that does not need to be retained during sleep.
        * On RAM projects, they are located in the .retention_ram_init section
        * for better memory handling.
        */
#if (dg_configEXEC_MODE == MODE_IS_CACHED)
        .non_retention_ram_init :  AT (QSPI_FW_IVT_BASE_ADDRESS + __etext + (RETENTION_RAM_INIT_SIZE) + (SNC_SECTION_SIZE))
#else
        /*
         * No need to add this to the copy table,
         * copy will be done by the debugger.
         */
        .non_retention_ram_init :
#endif
    {
            __non_retention_ram_init_start__ = .;
            . = ALIGN(4); /* Required by copy table */
            *(EXCLUDE_FILE(*libg_nano.a:* *libnosys.a:* *libgcc.a:* *libble_stack_da1469x.a:* *crtbegin.o) .data*)

            . = ALIGN(4); /* Required by copy table */
            __non_retention_ram_init_end__ = .;
    } > RAM
    /*
        * Note that region [__bss_start__, __bss_end__] will be also zeroed by newlib nano,
        * during execution of __START.
        */
    .bss :
    {
            . = ALIGN(4);
            __bss_start__ = .;

            *(EXCLUDE_FILE(*libg_nano.a:* *libnosys.a:* *libgcc.a:* *libble_stack_da1469x.a:* *crtbegin.o) .bss*)

            *(COMMON)
            . = ALIGN(4);
            __bss_end__ = .;
    } > RAM

    __non_retention_ram_end__ = .;

    . = ALIGN(8);
    __HeapBase = .;
    __end__ = __HeapBase;
    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > RAM
    _ram_start = ORIGIN(RAM);
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Top of head is the bottom of the stack */
    __HeapLimit = __StackLimit;
    
    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__HeapBase <= __HeapLimit, "region RAM overflowed with stack")
#if ( dg_configUSE_SEGGER_FLASH_LOADER == 1 )
        .prod_head :
        AT ( QSPI_FLASH_ADDRESS)
        {
                SHORT(0x7050)                   // 'Pp' flag
                LONG(QSPI_FW_BASE_OFFSET)       // active image pointer
                LONG(QSPI_FW_BASE_OFFSET)       // update image pointer
                LONG(0xA8A500EB)                // busrtcmdA
                LONG(0x66)                      // busrtcmdB
                SHORT(0x11AA)                   // Flash config section
                SHORT(0x03)                     // Flash config length
                BYTE(0x01)                      // Flash config sequence
                BYTE(0x40)                      // Flash config sequence
                BYTE(0x07)                      // Flash config sequence
                SHORT(0x4EC8)                   // CRC

        } > ROM

        .prod_head_backup :
        AT ( QSPI_FLASH_ADDRESS + 0x1000)
        {
                SHORT(0x7050)                   // 'Pp' flag
                LONG(QSPI_FW_BASE_OFFSET)       // active image pointer
                LONG(QSPI_FW_BASE_OFFSET)       // update image pointer
                LONG(0xA8A500EB)                // busrtcmdA
                LONG(0x66)                      // busrtcmdB
                SHORT(0x11AA)                   // Flash config section
                SHORT(0x03)                     // Flash config length
                BYTE(0x01)                      // Flash config sequence
                BYTE(0x40)                      // Flash config sequence
                BYTE(0x07)                      // Flash config sequence
                SHORT(0x4EC8)                   // CRC

        } > ROM

        .img_head :
        AT (QSPI_FW_BASE_ADDRESS)
        {
                SHORT(0x7151)                   // 'Pp' flag
                LONG(SIZEOF(.text))
                LONG(0x0)                       // crc, doesn't matter
                LONG(0x0)                       // version, doesn't matter
                LONG(0x0)                       // version, doesn't matter
                LONG(0x0)                       // version, doesn't matter
                LONG(0x0)                       // version, doesn't matter
                LONG(0x0)                       // timestamp, doesn't matter
                LONG(QSPI_FW_IVT_OFFSET)        // IVT pointer
                SHORT(0x22AA)                   // Security section type
                SHORT(0x0)                      //Security section length
                SHORT(0x44AA)                   // Device admin type
                SHORT(0x0)                      // Device admin length

        } > ROM
#endif /* dg_configUSE_SEGGER_FLASH_LOADER */
     
}
