/*****************************************************************************

  Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

   Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the
   distribution.

   Neither the name of Texas Instruments Incorporated nor the names of
   its contributors may be used to endorse or reproduce products derived
   from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/
/*
 *  ======== linker_bl3.cmd ========
 *
 *  BL3 linker script for CC35xx (8 MB flash platform).
 *
 *  Derived from CC35xx_common/linker.cmd with the following changes:
 *    - Stack reduced to 4 KB (BL3 has minimal stack depth compared to an app)
 *    - FreeRTOS BSS subsections removed (BL3 is nortos, no RTOS heap)
 *    - Off-target logging sections removed (no LOG_DATA / LOG_PTR)
 *    - BOOT_REPORT_SHARED_MEM kept as NOLOAD: BL3 reads it but does not own it
 *
 *  Platform constraint: BL3 container size must not exceed 50 KB.
 */
#include "ti_build_linker.cmd.toolbox"

/* CC35xx driverlib — relative to ${COM_TI_SIMPLELINK_WIFI_SDK_INSTALL_DIR}/source
 * (already in the linker library search path). Required because XMEMWFF3.c /
 * FlashWFF3.c reference OSPI/XIP register-access functions defined here. */
-l ti/devices/cc35xx/driverlib/lib/ticlang/driverlib.a

--retain=".resetVecs"

-stack 0x1000   /* 4 KB — BL3 has shallow call depth, no RTOS */
-heap  0x8000   /* 32 KB — mbedTLS bignum/ECDSA P-256 verify needs significant
                 *         transient MPI allocations (point read + scalar mul). */

#define FLASH_INT_VEC_SIZE (0x2400) /* Including padding */

MEMORY
{
    FLASH_INT_VEC           (RWX)  : origin = build_linker_toolbox_VENDOR_BL3_LOGICAL_START_ADDRESS,         length = FLASH_INT_VEC_SIZE
    FLASH_NON_SECURE        (RX)   : origin = end(FLASH_INT_VEC), length = build_linker_toolbox_VENDOR_BL3_SIZE - FLASH_INT_VEC_SIZE
    TCM_CRAM_NON_SECURE     (RWX)  : origin = 0x00000000,         length = 0x00007FFF
    CRAM_NON_SECURE         (RWX)  : origin = 0x08000000,         length = 0x0000FFFF
    TCM_DRAM_NON_SECURE     (RW)   : origin = 0x20000000,         length = ((build_linker_toolbox_PSRAM_SIZE == 0) * 0x10000 + 0xFFFF)
    CONNECTIVITY_SHARED_MEM (RW)   : origin = 0x28000000,         length = 0x000000FF
    BOOT_REPORT_SHARED_MEM  (RW)   : origin = 0x28000100,         length = 0x00000CAF
    DRAM_NON_SECURE         (RW)   : origin = 0x28000DB0,         length = 0x0007F24F
    PS_RAM                  (RW)   : origin = 0x60000000,         length = build_linker_toolbox_PSRAM_SIZE + (build_linker_toolbox_PSRAM_SIZE == 0)
}

SECTIONS
{
    GROUP {
        .reserved:                   { . += 0x2000; } (NOLOAD)
        .resetVecs:   {} palign(4)
    } > FLASH_INT_VEC

    GROUP {
        .cram:   {} palign(4)
        .text:   {} palign(4)
        .rodata: {} palign(4)
    } > FLASH_NON_SECURE

    GROUP {
        .binit:  {} palign(4)
        .cinit:  {} palign(4)
    } > FLASH_NON_SECURE

    /* Code that must run from RAM (e.g., flash erase/program routines) */
    .TI.ramfunc : {} load=FLASH_NON_SECURE, run=TCM_CRAM_NON_SECURE, table(BINIT)


    /* .internalRAM is in TCM_DRAM.
     * DMA engine cannot reach Flash or PS_RAM on CC35xx, so DMA buffers must be in TCM. */
    GROUP {
        .internalRAM:      {} palign(4)
    } > TCM_DRAM_NON_SECURE

    /* .internalRAM.bss is in TCM_DRAM for fast DMA access. */
    GROUP {
        .internalRAM.bss:  {} palign(4)
    } > TCM_DRAM_NON_SECURE

    /* .internalRAM.data is in TCM_DRAM. Required for Flash operations. */
    GROUP {
        .internalRAM.data: {} palign(4)
    } > TCM_DRAM_NON_SECURE

    /* Initialized globals and statics */
    GROUP {
        .data: {} palign(4)
    } > DRAM_NON_SECURE

    GROUP {
        .sysmem: {} palign(4)
    } > DRAM_NON_SECURE

    /* Uninitialized globals (no FreeRTOS subsections in BL3) */
    GROUP {
        .bss: {} palign(4)
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
    } > DRAM_NON_SECURE

    /* Stack in fast TCM for M33 */
    GROUP {
        .stack: {} palign(4)
    } > TCM_DRAM_NON_SECURE

    GROUP {
        .ramVecs: {} palign(512) (NOLOAD)
    } > TCM_CRAM_NON_SECURE

    GROUP {
        .connectivity_shared_status_section: {} palign(4) (NOLOAD)
    } > CONNECTIVITY_SHARED_MEM

    /* Boot Report region: written by BL2, read by BL3. NOLOAD — BL3 does not own it. */
    GROUP {
        .boot_report_shared_section: { _Boot_report_address = start(BOOT_REPORT_SHARED_MEM); } palign(4) (NOLOAD)
    } > BOOT_REPORT_SHARED_MEM
}
