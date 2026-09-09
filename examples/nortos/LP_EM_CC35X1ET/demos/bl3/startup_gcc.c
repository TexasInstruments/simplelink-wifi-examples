/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== startup_gcc.c ========
 *
 *  CC35xx M33 vector table and reset handler for BL3 — GCC build.
 *
 *  Mirrors startup_ticlang.c (TI Clang) with the following GCC-specific changes:
 *    - No _c_int00: GCC does not use TI RTS, so this file performs all
 *      C runtime initialisation (.bss zero, .data/.tcm_data/.TI.ramfunc
 *      copy) before calling main().
 *    - resetISR() is __attribute__((naked)); .ltorg avoids literal-pool
 *      errors under LTO.
 *    - Stack top is _stack_end (GCC linker convention vs __STACK_END).
 *    - FPU is enabled before any C code executes.
 *    - SetupTrimDevice() is NOT called: BL2 already performed device trim
 *      before handing off to BL3.
 *    - No FreeRTOS config dependencies (BL3 is nortos).
 *    - No PSRAM section copies (BL3 has no .psram_data / .psram_bss).
 *    - No C++ constructor loop (BL3 is pure C).
 */
#if defined(__clang__)
    #error "startup_gcc.c: GCC only — use startup_ticlang.c for TI Clang"
#endif

#include <stdint.h>

void resetISR(void);

/* Stack top exported by the linker script (_stack_end = high address = initial SP). */
extern unsigned long _stack_end;

/* Section-copy / zero-init symbols exported by linker_bl3_GNU.lds. */
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __data_load__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __tcm_data_load__;
extern uint32_t __tcm_data_start__;
extern uint32_t __tcm_data_end__;
extern uint32_t __ramfunc_load__;
extern uint32_t __ramfunc_start__;
extern uint32_t __ramfunc_end__;


extern int main(void);

static void faultSpin(void)
{
    while (1) {}
}

/* Vector table — 16 core (M33) + 53 peripheral = 69 entries.
 * Identical layout to startup_ticlang.c (TI Clang). */
__attribute__((section(".resetVecs"), used))
void (*const resetVectors[])(void) = {
    /* Initial stack pointer */
    (void (*)(void))((unsigned long)&_stack_end),
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

__attribute__((used)) void localProgramStart(void)
{
    volatile uint32_t *cpacr = (volatile uint32_t *)0xE000ED88;
    volatile uint32_t *vtor  = (volatile uint32_t *)0xE000ED08;
    volatile uint32_t *bs, *be;
    volatile uint32_t *dl, *ds, *de;
    volatile uint32_t *ramFuncLoad, *ramFuncStart, *ramFuncEnd;

    /* Enable FPU — required before any FP instructions; mbedTLS bignum
     * operations may trigger FP register saves on context switch paths. */
    *cpacr |= (0xF0u << 16);

    /* Zero-initialise .bss (DRAM_NON_SECURE). */
    bs = (volatile uint32_t *)&__bss_start__;
    be = (volatile uint32_t *)&__bss_end__;
    while (bs < be)
    {
        *bs++ = 0u;
    }

    /* Relocate .data (initialised globals) from flash to DRAM. */
    dl = (volatile uint32_t *)&__data_load__;
    ds = (volatile uint32_t *)&__data_start__;
    de = (volatile uint32_t *)&__data_end__;
    if (dl != ds)
    {
        while (ds < de)
        {
            *ds++ = *dl++;
        }
    }

    /* Relocate .tcm_data (.internalRAM sections) from flash to TCM_DRAM.
     * DMA engine cannot reach flash or PS_RAM on CC35xx. */
    dl = (volatile uint32_t *)&__tcm_data_load__;
    ds = (volatile uint32_t *)&__tcm_data_start__;
    de = (volatile uint32_t *)&__tcm_data_end__;
    if (dl != ds)
    {
        while (ds < de)
        {
            *ds++ = *dl++;
        }
    }

    /* Copy .TI.ramfunc (flash erase/program routines) from flash to
     * TCM_CRAM.  These must execute from RAM when OTFDE is disabled. */
    ramFuncLoad  = (volatile uint32_t *)&__ramfunc_load__;
    ramFuncStart = (volatile uint32_t *)&__ramfunc_start__;
    ramFuncEnd   = (volatile uint32_t *)&__ramfunc_end__;
    if (ramFuncLoad != ramFuncStart)
    {
        while (ramFuncStart < ramFuncEnd)
        {
            *ramFuncStart++ = *ramFuncLoad++;
        }
    }

    /* Point VTOR at the flash vector table. */
    *vtor = (uint32_t)(uintptr_t)&resetVectors[0];

    main();
}

void __attribute__((naked)) resetISR(void)
{
    /* Load initial SP from vector table word 0, then enter C runtime init.
     * .ltorg forces literal pool emission here to avoid LTO pool errors. */
    __asm__ __volatile__(
        " movw r0, #:lower16:resetVectors\n"
        " movt r0, #:upper16:resetVectors\n"
        " ldr  r0, [r0]\n"
        " mov  sp, r0\n"
        " b    localProgramStart\n"
        " .ltorg\n");
}

/* Empty stub required by newlib when exit() support is linked in. */
void _fini(void) {}
