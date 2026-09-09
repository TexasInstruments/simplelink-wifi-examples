/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== startup_ticlang.c ========
 *
 *  CC35xx M33 vector table and reset handler for BL3.
 *
 *  Based on kernel/freertos/startup/startup_cc35xx_ticlang.c with all
 *  FreeRTOS and SysConfig dependencies removed.  BL3 does not use the
 *  RTOS tick, SVC, or PendSV — all peripheral IRQs default to faultSpin.
 *  Vector table: 16 core (M33) + 53 peripheral = 69 entries total.
 */
#if !(defined(__clang__))
    #error "startup_ticlang.c: TI-CLANG only"
#endif

#include <stdint.h>

extern void _c_int00(void);
void resetISR(void);

extern void *__stack;
extern unsigned long __STACK_END;

/* Spin on any unhandled fault or interrupt */
static void faultSpin(void)
{
    while (1) {}
}

/* Vector table — must match CC35xx IRQ count (16 core + 53 peripheral = 69 entries) */
__attribute__((section(".resetVecs"), retain))
void (*const resetVectors[])(void) = {
    /* Initial stack pointer */
    (void (*)(void))((unsigned long)&__STACK_END),
    /* Reset handler */
    resetISR,
    /* NMI */
    faultSpin,
    /* HardFault */
    faultSpin,
    /* MemManage */
    faultSpin,
    /* BusFault */
    faultSpin,
    /* UsageFault */
    faultSpin,
    /* SecureFault */
    faultSpin,
    /* Reserved */
    faultSpin,
    /* Reserved */
    faultSpin,
    /* Reserved */
    faultSpin,
    /* SVCall */
    faultSpin,
    /* DebugMonitor */
    faultSpin,
    /* Reserved */
    faultSpin,
    /* PendSV */
    faultSpin,
    /* SysTick */
    faultSpin,
    /* IRQ0..52 — 53 peripheral interrupts */
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin, faultSpin, faultSpin,
    faultSpin, faultSpin, faultSpin,
};

void localProgramStart(void)
{
    unsigned long *vtor = (unsigned long *)0xE000ED08;

    /* BL2 called SetupTrimDevice before handing off — no need to repeat it */
    *vtor = (unsigned long)&resetVectors[0];

    __asm(" .global _c_int00\n"
          " b.w     _c_int00");
}

void resetISR(void)
{
    __asm__ __volatile__(
        " movw r0, #:lower16:resetVectors\n"
        " movt r0, #:upper16:resetVectors\n"
        " ldr r0, [r0]\n"
        " mov sp, r0\n"
        " b localProgramStart");
}
