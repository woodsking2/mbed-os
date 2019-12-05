/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */


#define SYS_CTRL_REG_DEBUGGER_ENABLE_MSK        0x80
    .syntax unified
    .arch   armv8-m.main

    .section .stack
    .align  3
    .equ    Stack_Size, MBED_CONF_TARGET_BOOT_STACK_SIZE
    .equ    SYS_CTRL_REG,       0x50000024
    .equ    CACHE_FLASH_REG,    0x100C0040
    .equ    RESET_STAT_REG,     0x500000BC    

    .globl  __StackTop
    .globl  __StackLimit
__StackLimit:
    .space  Stack_Size
    .size   __StackLimit, . - __StackLimit
__StackTop:
    .size   __StackTop, . - __StackTop

    .section .heap
    .align  3
    .equ    Heap_Size, 0
    .globl  __HeapBase
    .globl  __HeapLimit
__HeapBase:
    .if     Heap_Size
    .space  Heap_Size
    .endif
    .size   __HeapBase, . - __HeapBase
__HeapLimit:
    .size   __HeapLimit, . - __HeapLimit

    .section .isr_vector
    .align 2
    .globl  __isr_vector
__isr_vector:
    .long   __StackTop
    .long   Reset_Handler
    /* Cortex-M33 interrupts */
    .long   NMI_Handler
    .long   HardFault_Handler
    .long   MemoryManagement_Handler
    .long   BusFault_Handler
    .long   UsageFault_Handler
    .long   SecureFault_Handler
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   SVC_Handler
    .long   DebugMonitor_Handler
    .long   0                       /* Reserved */
    .long   PendSV_Handler
    .long   SysTick_Handler
    /* DA1469x interrupts */
    .long   SENSOR_NODE_IRQHandler
    .long   DMA_IRQHandler
    .long   CHARGER_STATE_IRQHandler
    .long   CHARGER_ERROR_IRQHandler
    .long   CMAC2SYS_IRQHandler
    .long   UART_IRQHandler
    .long   UART2_IRQHandler
    .long   UART3_IRQHandler
    .long   I2C_IRQHandler
    .long   I2C2_IRQHandler
    .long   SPI_IRQHandler
    .long   SPI2_IRQHandler
    .long   PCM_IRQHandler
    .long   SRC_IN_IRQHandler
    .long   SRC_OUT_IRQHandler
    .long   USB_IRQHandler
    .long   TIMER_IRQHandler
    .long   TIMER2_IRQHandler
    .long   RTC_IRQHandler
    .long   KEY_WKUP_GPIO_IRQHandler
    .long   PDC_IRQHandler
    .long   VBUS_IRQHandler
    .long   MRM_IRQHandler
    .long   MOTOR_CONTROLLER_IRQHandler
    .long   TRNG_IRQHandler
    .long   DCDC_IRQHandler
    .long   XTAL32M_RDY_IRQHandler
    .long   ADC_IRQHandler
    .long   ADC2_IRQHandler
    .long   CRYPTO_IRQHandler
    .long   CAPTIMER1_IRQHandler
    .long   RFDIAG_IRQHandler
    .long   LCD_CONTROLLER_IRQHandler
    .long   PLL_LOCK_IRQHandler
    .long   TIMER3_IRQHandler
    .long   TIMER4_IRQHandler
    .long   LRA_IRQHandler
    .long   RTC_EVENT_IRQHandler
    .long   GPIO_P0_IRQHandler
    .long   GPIO_P1_IRQHandler
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .long   0                       /* Reserved */
    .size   __isr_vector, . - __isr_vector

    .section .text_ram
    .thumb
    .thumb_func
    .align 2
    .globl wakeup_handler
    .type  wakeup_handler, %function
wakeup_handler:
    ldr     r0, =RESET_STAT_REG
    ldr     r0, [r0, #0]
    teq     r0, #0
    ite     eq
    ldreq   r3, =da1469x_m33_wakeup
    ldrne   r3, =Reset_Handler
    bx      r3

    .size   wakeup_handler, . - wakeup_handler

    .text
    .thumb
    .thumb_func
    .align 2
    .globl Reset_Handler
    .type  Reset_Handler, %function
    
Reset_Handler:
/* If the Magic Word {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEAD10CC} is found at 0x20026c00 then the
 * execution will block for a while in order to give time to a debugger to attach. */
    ldr     r4, =0x2007fff0
    ldmia   r4!, {r0-r3}
    ldr     r4, =0xDEADBEEF
    cmp     r0, r4
    bne     .code_starts
    cmp     r1, r4
    bne     .code_starts
    cmp     r2, r4
    bne     .code_starts
    ldr     r4, =0xDEAD10CC
    cmp     r3, r4
    bne     .code_starts
    /* Enable debugger */
    ldr     r4, =SYS_CTRL_REG
    ldrh    r5, [r4]
    mov     r6, r5
    movs    r1, #SYS_CTRL_REG_DEBUGGER_ENABLE_MSK
    orrs    r5, r5, r1
    strh    r5, [r4]
    /* Block for 2'' */
    ldr     r0, =0x150000

.delay_loop:
    subs    r0, #1
    bgt     .delay_loop
    /* Make sure that this will happen only once! */
    ldr     r4, =0x2007fff0
    movs    r0, #0
    str     r0, [r4]
    /* Restore debugger setting */
    ldr     r4, =SYS_CTRL_REG
    strh    r6, [r4]

.code_starts:
 /* Make sure interrupt vector is remapped at 0x0 */
    ldr     r1, =SYS_CTRL_REG
    ldrh    r2, [r1, #0]
    orrs    r2, r2, #8
    strh    r2, [r1, #0]

/*
 * Flash is remapped at 0x0 with an offset, i.e. 0x0 does not correspond to
 * 0x16000000 but to start of an image on flash. This is calculated from product
 * header by 1st state bootloader and configured in CACHE_FLASH_REG. We need to
 * retrieve proper offset value for calculations later.
 */
    ldr     r1, =CACHE_FLASH_REG
    ldr     r4, [r1, #0]
    mov     r2, r4
    mov     r3, #0xFFFF
    bic     r4, r4, r3      /* CACHE_FLASH_REG[FLASH_REGION_BASE] */
    mov     r3, #0xFFF0
    and     r2, r2, r3      /* CACHE_FLASH_REG[FLASH_REGION_OFFSET] */
    lsr     r2, r2, #2
    orr     r4, r4, r2

/* Copy ISR vector from flash to RAM */
    ldr     r1, =__isr_vector_start     /* src ptr */
    ldr     r2, =__isr_vector_end       /* src end */
    ldr     r3, =__intvect_start__      /* dst ptr */
/* Make sure we copy from QSPIC address range, not from remapped range */
    cmp     r1, r4
    itt     lt
    addlt   r1, r1, r4
    addlt   r2, r2, r4
.loop_isr_copy:
    cmp     r1, r2
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r3], #4
    blt     .loop_isr_copy

/* Copy QSPI code from flash to RAM */
    ldr     r1, =__text_ram_addr        /* src ptr */
    ldr     r2, =__text_ram_start__     /* ptr */
    ldr     r3, =__text_ram_end__       /* dst end */
.loop_code_text_ram_copy:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt     .loop_code_text_ram_copy

/* Copy data from flash to RAM */
    ldr     r1, =__etext                /* src ptr */
    ldr     r2, =__data_start__         /* dst ptr */
    ldr     r3, =__data_end__           /* dst end */
.loop_data_copy:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt     .loop_data_copy


/* Clear BSS */
    movs    r0, 0
    ldr     r1, =__bss_start__
    ldr     r2, =__bss_end__
.loop_bss_clear:
    cmp     r1, r2
    itt     lt
    strlt   r0, [r1], #4
    blt     .loop_bss_clear

/* Set reset handler to wakeup handler so we can handle wake up from deep sleep */
    ldr     r0, =__intvect_start__
    ldr     r1, =wakeup_handler
    str     r1, [r0, #4]

    bl      SystemInit    
    bl      _start

    .pool
    .size   Reset_Handler, . - Reset_Handler

/* Default interrupt handler */
    .type   Default_Handler, %function
Default_Handler:
    ldr     r1, =SYS_CTRL_REG
    ldrh    r2, [r1, #0]
    orrs    r2, r2, #0x80   /* DEBUGGER_ENABLE */
    strh    r2, [r1, #0]
    b       .

    .size   Default_Handler, . - Default_Handler

/* Default handlers for all interrupts */
    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    /* Cortex-M33 interrupts */
    IRQ  NMI_Handler
    IRQ  HardFault_Handler
    IRQ  MemoryManagement_Handler
    IRQ  BusFault_Handler
    IRQ  UsageFault_Handler
    IRQ  SecureFault_Handler
    IRQ  SVC_Handler
    IRQ  DebugMonitor_Handler
    IRQ  PendSV_Handler
    IRQ  SysTick_Handler
    /* DA1469x interrupts */
    IRQ  SENSOR_NODE_IRQHandler
    IRQ  DMA_IRQHandler
    IRQ  CHARGER_STATE_IRQHandler
    IRQ  CHARGER_ERROR_IRQHandler
    IRQ  CMAC2SYS_IRQHandler
    IRQ  UART_IRQHandler
    IRQ  UART2_IRQHandler
    IRQ  UART3_IRQHandler
    IRQ  I2C_IRQHandler
    IRQ  I2C2_IRQHandler
    IRQ  SPI_IRQHandler
    IRQ  SPI2_IRQHandler
    IRQ  PCM_IRQHandler
    IRQ  SRC_IN_IRQHandler
    IRQ  SRC_OUT_IRQHandler
    IRQ  USB_IRQHandler
    IRQ  TIMER_IRQHandler
    IRQ  TIMER2_IRQHandler
    IRQ  RTC_IRQHandler
    IRQ  KEY_WKUP_GPIO_IRQHandler
    IRQ  PDC_IRQHandler
    IRQ  VBUS_IRQHandler
    IRQ  MRM_IRQHandler
    IRQ  MOTOR_CONTROLLER_IRQHandler
    IRQ  TRNG_IRQHandler
    IRQ  DCDC_IRQHandler
    IRQ  XTAL32M_RDY_IRQHandler
    IRQ  ADC_IRQHandler
    IRQ  ADC2_IRQHandler
    IRQ  CRYPTO_IRQHandler
    IRQ  CAPTIMER1_IRQHandler
    IRQ  RFDIAG_IRQHandler
    IRQ  LCD_CONTROLLER_IRQHandler
    IRQ  PLL_LOCK_IRQHandler
    IRQ  TIMER3_IRQHandler
    IRQ  TIMER4_IRQHandler
    IRQ  LRA_IRQHandler
    IRQ  RTC_EVENT_IRQHandler
    IRQ  GPIO_P0_IRQHandler
    IRQ  GPIO_P1_IRQHandler
    IRQ  RESERVED40_IRQHandler
    IRQ  RESERVED41_IRQHandler
    IRQ  RESERVED42_IRQHandler
    IRQ  RESERVED43_IRQHandler
    IRQ  RESERVED44_IRQHandler
    IRQ  RESERVED45_IRQHandler
    IRQ  RESERVED46_IRQHandler
    IRQ  RESERVED47_IRQHandler

.end
