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

/* Linker script to configure memory regions. */
#define DEBUG_FLAG_SIZE 16
MEMORY
{
    FLASH (rx)  : ORIGIN = (0x0), LENGTH = (380 * 1024)
    RAM (rw)    : ORIGIN = (0x20000000), LENGTH = (512 * 1024)
}

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")

ENTRY(Reset_Handler)

SECTIONS
{
     __text = .;

    .text :
    {
        __isr_vector_start = .;
        KEEP(*(.isr_vector))
        /* ISR vector shall have exactly 512 bytes */
        . = __isr_vector_start + 0x200;
        __isr_vector_end = .;
        
        *(.text)
        *(.text.*)

        __cmac_fw_area_start = .;
        KEEP(*(.cmi_fw_area*))
        __cmac_fw_area_end = .;
        . = ALIGN(4);

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

        *(.rodata*)

        KEEP(*(.eh_frame*))
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        . = ALIGN(4);
    } > FLASH
    __exidx_end = .;

    .intvect :
    {
        . = ALIGN(4);
        __intvect_start__ = .;
        . = . + (__isr_vector_end - __isr_vector_start);
        . = ALIGN(4);
    } > RAM
    .sleep_state (NOLOAD) :
    {
        . = ALIGN(4);
        *(sleep_state)
    } > RAM

    __text_ram_addr = LOADADDR(.text_ram);

    .text_ram :
    {
        . = ALIGN(4);
        __text_ram_start__ = .;
        *(.text_ram*)
        . = ALIGN(4);
        __text_ram_end__ = .;
    } > RAM AT > FLASH

    __etext = LOADADDR(.data);

    .data :
    {
        __data_start__ = .;
        *(vtable)
        *(.data*)

        . = ALIGN(8);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(8);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);


        . = ALIGN(8);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.jcr)
        . = ALIGN(8);
        /* All data end */
        __data_end__ = .;
    } > RAM AT > FLASH

    .bssnz :
    {
        . = ALIGN(4);
        __bssnz_start__ = .;
        *(.bss.core.nz*)
        . = ALIGN(4);
        __bssnz_end__ = .;
    } > RAM

    .bss :
    {
        . = ALIGN(8);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        __bss_end__ = .;
    } > RAM

    .cmac_ram (NOLOAD) :
    {
        . = ALIGN(0x400);
        __cmac_ram_section_start__ = .;
        cmac_fw_addr = .;
        . += (__cmac_fw_area_end - __cmac_fw_area_start);
        . = ALIGN(0x400);
        __cmac_ram_section_end__ = . - 1;
    } > RAM

    __debug_flag_start = .;
    .debug_flag (NOLOAD):
    {
            . = 16;
    } > RAM
    __debug_flag_end = .;

    /* Heap starts after BSS */
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

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Top of head is the bottom of the stack */
    __HeapLimit = __StackLimit;
    
    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__HeapBase <= __HeapLimit, "region RAM overflowed with stack")

    /* Check that intvect is at the beginning of RAM */
    ASSERT(__intvect_start__ == ORIGIN(RAM), "intvect is not at beginning of RAM")
}
