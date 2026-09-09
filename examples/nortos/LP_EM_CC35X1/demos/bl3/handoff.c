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
 *    its contributors may be used to endorse or reproduce products derived
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
 *  ======== handoff.c ========
 *
 *  BL3 handoff: HOST_BOOT_DONE and M33 self-reset.
 *  See bl3_handoff.h for a detailed explanation of each function.
 */

#include <stdint.h>

#include "handoff.h"
#include "config.h"
#include "uart.h"

/* ---------------------------------------------------------------------------
 * BL3_HANDOFF_setHostBootDone
 * ---------------------------------------------------------------------------
 */
void BL3_HANDOFF_setHostBootDone(void)
{
    volatile uint32_t *hostBootDoneReg =
        (volatile uint32_t *)(BL3_SECGP_BASE_ADDR + BL3_SECGP_HOST_BOOT_DONE_OFFSET);

    *hostBootDoneReg = 1U;

    BL3_DBG_print("[BL3] HOST_BOOT_DONE asserted\r\n");
}

/* ---------------------------------------------------------------------------
 * BL3_HANDOFF_launchApp
 * ---------------------------------------------------------------------------
 */
__attribute__((noreturn))
void BL3_HANDOFF_launchApp(uint32_t appBase)
{
    /* Read initial SP and reset handler from the vendor app vector table. */
    const uint32_t initSp   = ((const uint32_t *)appBase)[0];
    const uint32_t resetVec = ((const uint32_t *)appBase)[1];

    __asm volatile (
        "CPSID I        \n"  /* disable interrupts */
        "MSR    MSP, %0 \n"  /* load vendor app initial stack pointer */
        "BX     %1      \n"  /* branch to vendor app reset handler */
        :
        : "r" (initSp), "r" (resetVec)
        : "memory"
    );
    while (1) {}
}
