/*****************************************************************************

  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/

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
   its contributors may be used to endorse or promote products derived
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
#include "ti_build_linker.cmd.toolbox"

--retain=".resetVecs"
-stack 0x2FF0
-heap  0x0

#define FLASH_INT_VEC_SIZE (0x2400) // Including padding

MEMORY
{
    FLASH_INT_VEC           (RWX)  : origin = 0x14000000,          length = FLASH_INT_VEC_SIZE              //0x10000000-0x10001300  0x4kbyte
    FLASH_NON_SECURE        (RX)   : origin = end(FLASH_INT_VEC),  length = build_linker_toolbox_FLASH_SIZE-FLASH_INT_VEC_SIZE
    //INT_VEC                 (RWX)  : origin = 0x00000000,          length = 0x000002FF                      //0x00000000-0x000002FF  0x300kbyte
    TCM_CRAM_NON_SECURE     (RWX)  : origin = 0x00000000,          length = 0x00007FFF                      //0x00000000-0x000007FF  32kbyte
    //TCM_CRAM_SECURE         (RWX)  : origin = 0x04000000,          length = 0x03FFFFFF                      //0x04000000-0x07FFFFFF
    CRAM_NON_SECURE         (RWX)  : origin = 0x08000000,          length = 0x0000FFFF                      //0x08000000-0x0800FFFF  64Kbyte
    //CRAM_SECURE             (RWX)  : origin = 0x0C000000,          length = 0x03FFFFFF                      //0x0C000000-0x0FFFFFFF
    TCM_DRAM_NON_SECURE     (RW)   : origin = 0x20000000,          length = ((build_linker_toolbox_PSRAM_SIZE == 0) * 0x10000 + 0xFFFF)  //0x20000000-0x2000FFFF  64Kbyte for PSRAM / //0x20000000-0x2001FFFF  128Kbyte for NO-PSRAM 
    //TCM_DRAM_SECURE         (RW)   : origin = 0x24000000,          length = 0x03FFFFFF                      //0x24000000-0x27FFFFFF
    CONNECTIVITY_SHARED_MEM (RW)   : origin = 0x28000000,          length = 0x000000FF                      //0x28030000 - 0x280000FF
    BOOT_REPORT_SHARED_MEM  (RW)   : origin = 0x28000100,          length = 0x00000CAF                      //0x28000100-0x28000DAF 
    DRAM_NON_SECURE         (RW)   : origin = 0x28000DB0,          length = 0x0007F24F                      //0x28000DB0-0x2807FFFF 
    //DRAM_SECURE             (RW)   : origin = 0x2C000000,          length = 0x03FFFFFF                      //0x2C000000-0x2FFFFFFF
    PS_RAM                  (RW)   : origin = 0x60000000,          length = build_linker_toolbox_PSRAM_SIZE + (build_linker_toolbox_PSRAM_SIZE == 0)  //0x60000000-0x60800000 //0x60000000-0x60200000 Configure by sysconfig

    /* Explicitly placed off target for the storage of logging data.
     * The ARM memory map allocates 1 GB of external memory from 0x60000000 - 0x9FFFFFFF.
     * Unlikely that all of this will be used, so we are using the upper parts of the region.
     * ARM memory map: https://developer.arm.com/documentation/ddi0337/e/memory-map/about-the-memory-map*/
    LOG_DATA (R) : origin = 0x90000000,                          length = 0x40000        /* 256 KB */
    LOG_PTR  (R) : origin = 0x94000008,                          length = 0x40000        /* 256 KB */

}

SECTIONS
{
    GROUP {
        .reserved:                   { . += 0x2000; } (NOLOAD)
        .resetVecs:   {} palign(4)   /* This is where code resides */
    } > FLASH_INT_VEC

    /* This is rest of code */
    GROUP {
        .cram:   {} palign(4)
        .text:   {} palign(4)   /* This is where code resides */
        .rodata: {} palign(4)   /* This is where const's go */
        .binit:  {} palign(4)
        .cinit:  {} palign(4)
    } > FLASH_NON_SECURE

    /* Code RAM */
    .TI.ramfunc     : {} load=FLASH_NON_SECURE, run=TCM_CRAM_NON_SECURE, table(BINIT)

    GROUP {
        .sysmem: {} palign(4)   /* This is where the malloc heap goes */
        .bss:    {} palign(4)   /* This is where uninitialized globals go */
        .data:   {} palign(4)   /* This is where initialized globals and static go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
        .stack:  {} palign(4)   /* This is where the main() stack goes */
    } > DRAM_NON_SECURE | TCM_DRAM_NON_SECURE

    GROUP {
        .connectivity_shared_status_section: {} palign(4) (NOLOAD)
    } > CONNECTIVITY_SHARED_MEM

    GROUP {
        .boot_report_shared_section: { _Boot_report_address = start(BOOT_REPORT_SHARED_MEM);} palign(4) (NOLOAD)
    } > BOOT_REPORT_SHARED_MEM

    .log_data       :   > LOG_DATA, type = COPY
    .log_ptr        : { *(.log_ptr*) } > LOG_PTR align 4, type = COPY
}

