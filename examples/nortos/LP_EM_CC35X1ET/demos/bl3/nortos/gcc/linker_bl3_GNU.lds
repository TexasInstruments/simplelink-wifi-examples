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
 *    distribution.
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
 *  ======== linker_bl3_GNU.lds ========
 *
 *  BL3 GNU LD linker script for CC35xx (8 MB flash platform).
 *
 *  Derived from linker_bl3.cmd with the following GCC-specific changes:
 *    - INCLUDE replaces #include; ternary operator replaces TI arithmetic
 *      for conditional region sizes.
 *    - STACKSIZE variable replaces -stack flag.  No -heap or -l directives:
 *      the driverlib archive must be added to makefile LFLAGS as
 *      ti/devices/cc35xx/driverlib/lib/gcc/driverlib.a.
 *    - AT> + LOADADDR() symbols replace table(BINIT) for section copies.
 *      startup_gcc.c performs the copies; symbols must match exactly.
 *    - .ARM.exidx / .ARM.extab added (GCC requires these even for C code).
 *    - *(COMMON) placed in .bss (TI Clang merges COMMON into .bss
 *      automatically; GCC does not).
 *    - Stack top exported as _stack_end / __stack_end (GCC convention).
 *
 *  Platform constraint: BL3 container size must not exceed 50 KB.
 */

INCLUDE "ti_build_linker.lds.toolbox"

STACKSIZE          = 0x1000;   /* 4 KB — BL3 has shallow call depth, no RTOS */
HEAPSIZE           = 0x8000;   /* 32 KB — mbedTLS bignum/ECDSA P-256 verify needs significant
                                *          transient MPI allocations (point read + scalar mul). */
FLASH_INT_VEC_SIZE = 0x2400;   /* Including padding */

MEMORY
{
    FLASH_INT_VEC           (RX)  : ORIGIN = build_linker_toolbox_VENDOR_BL3_LOGICAL_START_ADDRESS,
                                    LENGTH = FLASH_INT_VEC_SIZE
    FLASH_NON_SECURE        (RX)  : ORIGIN = build_linker_toolbox_VENDOR_BL3_LOGICAL_START_ADDRESS + FLASH_INT_VEC_SIZE,
                                    LENGTH = build_linker_toolbox_VENDOR_BL3_SIZE - FLASH_INT_VEC_SIZE
    TCM_CRAM_NON_SECURE     (RWX) : ORIGIN = 0x00000000, LENGTH = 0x00007FFF
    CRAM_NON_SECURE         (RWX) : ORIGIN = 0x08000000, LENGTH = 0x0000FFFF
    TCM_DRAM_NON_SECURE     (RW)  : ORIGIN = 0x20000000,
                                    LENGTH = (build_linker_toolbox_PSRAM_SIZE == 0 ? 0x1FFFF : 0x0FFFF)
    CONNECTIVITY_SHARED_MEM (RWX) : ORIGIN = 0x28000000, LENGTH = 0x000000FF
    BOOT_REPORT_SHARED_MEM  (RWX) : ORIGIN = 0x28000100, LENGTH = 0x00000CAF
    DRAM_NON_SECURE         (RWX) : ORIGIN = 0x28000DB0, LENGTH = 0x0007F24F
    PS_RAM                  (RW)  : ORIGIN = 0x60000000,
                                    LENGTH = (build_linker_toolbox_PSRAM_SIZE > 0 ? build_linker_toolbox_PSRAM_SIZE : 0x1)
}

SECTIONS
{
    /* Reserved for bootloader metadata — BL3 image starts at +0x2000. */
    .reserved (NOLOAD) : {
        . += 0x2000;
    } > FLASH_INT_VEC

    /* Vector table in flash. */
    .resetVecs : {
        KEEP (*(.resetVecs))
    } > FLASH_INT_VEC

    /* Code sections. */
    .text : {
        CREATE_OBJECT_SYMBOLS
        *(.text)
        *(.text.*)
        . = ALIGN(0x4);
        KEEP (*(.ctors))
        . = ALIGN(0x4);
        KEEP (*(.dtors))
        . = ALIGN(0x4);
        __init_array_start = .;
        KEEP (*(.init_array*))
        __init_array_end = .;
        *(.init)
        *(.fini*)
    } > FLASH_NON_SECURE

    PROVIDE (__etext = .);
    PROVIDE (_etext  = .);
    PROVIDE (etext   = .);

    .rodata : {
        *(.rodata)
        *(.rodata.*)
        *(.rodata_*)
        . = ALIGN(4);
    } > FLASH_NON_SECURE

    .cinit : {
        *(.cinit)
    } > FLASH_NON_SECURE

    .binit : {
        *(.binit)
    } > FLASH_NON_SECURE

    /* .TI.ramfunc: flash erase/program routines that must execute from CRAM.
     * startup_gcc.c copies __ramfunc_load__ → [__ramfunc_start__, __ramfunc_end__). */
    .TI.ramfunc : ALIGN(4) {
        __ramfunc_load__  = LOADADDR(.TI.ramfunc);
        __ramfunc_start__ = .;
        *(.TI.ramfunc)
        __ramfunc_end__   = .;
    } > TCM_CRAM_NON_SECURE AT> FLASH_NON_SECURE

    /* C++ / GCC exception handling tables (required even for C-only builds). */
    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH_NON_SECURE

    .ARM.extab : {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH_NON_SECURE

    /* .stack: 4 KB at the BOTTOM of TCM_DRAM for M33 call stack.
     * Placed first so a stack overflow grows into unmapped space below
     * 0x20000000, not into .tcm_data (internalRAM driver state).
     * This matches the ticlang layout where .stack precedes .internalRAM.
     * _stack_end is the high address — used as the initial SP in the
     * vector table by startup_gcc.c (stack grows downward). */
    .stack (NOLOAD) : ALIGN(8) {
        __stack     = .;
        KEEP (*(.stack))
        . += STACKSIZE;
        _stack_end  = .;
        __stack_end = .;
    } > TCM_DRAM_NON_SECURE

    /* .tcm_data: .internalRAM sections copied from flash to TCM_DRAM.
     * Placed ABOVE .stack so stack overflow cannot corrupt flash driver state.
     * DMA engine cannot reach flash or PS_RAM on CC35xx.
     * startup_gcc.c copies __tcm_data_load__ → [__tcm_data_start__, __tcm_data_end__). */
    .tcm_data : ALIGN(4) {
        __tcm_data_load__  = LOADADDR(.tcm_data);
        __tcm_data_start__ = .;
        *(.internalRAM)
        *(.internalRAM.*)
        . = ALIGN(4);
        __tcm_data_end__   = .;
    } > TCM_DRAM_NON_SECURE AT> FLASH_NON_SECURE

    /* .data: initialised globals copied from flash to DRAM.
     * startup_gcc.c copies __data_load__ → [__data_start__, __data_end__). */
    .data : ALIGN(4) {
        __data_load__  = LOADADDR(.data);
        __data_start__ = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        __data_end__   = .;
    } > DRAM_NON_SECURE AT> FLASH_NON_SECURE

    /* .heap: malloc heap for mbedTLS bignum/ECDSA P-256 transient allocations.
     * end/__end/_end satisfy libnosys _sbrk (same pattern as cc35xx_nortos.lds). */
    .heap (NOLOAD) : {
        __heap_start__ = .;
        end  = __heap_start__;
        _end = end;
        __end = end;
        . += HEAPSIZE;
        __heap_end__ = .;
    } > DRAM_NON_SECURE

    /* .bss: zero-initialised globals and COMMON symbols.
     * No FreeRTOS subsections — BL3 is nortos.
     * startup_gcc.c zeroes [__bss_start__, __bss_end__). */
    .bss : {
        __bss_start__ = .;
        *(.shbss)
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > DRAM_NON_SECURE

    /* .ramVecs: RAM copy of the vector table, populated by driverlib. */
    .ramVecs (NOLOAD) : ALIGN(512) {
        KEEP (*(.ramVecs))
    } > TCM_CRAM_NON_SECURE

    /* Connectivity shared status (owned by firmware, not BL3). */
    .connectivity_shared_status_section (NOLOAD) : ALIGN(4) {
        KEEP (*(.connectivity_shared_status_section))
    } > CONNECTIVITY_SHARED_MEM

    /* Boot Report region: written by BL2, read by BL3.
     * NOLOAD — BL3 does not initialise this region. */
    .boot_report_shared_section (NOLOAD) : ALIGN(4) {
        _Boot_report_address = .;
    } > BOOT_REPORT_SHARED_MEM
}

ENTRY(resetISR)
