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
 *  ======== boot_report.c ========
 *
 *  BL3 Boot Report module.
 *
 *  The Boot Report is a shared memory structure at 0x28000100 (M33 view),
 *  populated by BL2 before releasing M33. BL3 reads it to:
 *    - Detect BL2 errors (ti_bootloader_bl2_status != 0 → halt)
 *    - Select the boot flow (normal vs DSSM debug)
 *    - Pass the report to the vendor application
 *
 *  The report pointer is passed to BL3_HOOK_issueReport() before handoff
 *  so the vendor application can receive additional BL3 status data.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "boot_report.h"
#include "config.h"
#include "uart.h"

/* Cached pointer to Boot Report in shared DRAM — set once during init */
static bl3_bootReport_t *s_bootReport = NULL;

/* ---------------------------------------------------------------------------
 * BL3_bootReport_init
 * ---------------------------------------------------------------------------
 */
int32_t BL3_bootReport_init(void)
{
    s_bootReport = (bl3_bootReport_t *)BL3_BOOT_REPORT_ADDR;

    BL3_DBG_print("[BL3] Boot Report at 0x%08X, BL2 status = 0x%02X\r\n",
                  BL3_BOOT_REPORT_ADDR,
                  (unsigned)s_bootReport->ti_bootloader_bl2_status);

    if (s_bootReport->ti_bootloader_bl2_status != BL3_BL2_STATUS_OK)
    {
        BL3_DBG_print("[BL3] ERROR: BL2 reported failure (status=0x%02X)\r\n",
                      (unsigned)s_bootReport->ti_bootloader_bl2_status);
        return -1;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * BL3_bootReport_isDssmRequested
 * ---------------------------------------------------------------------------
 */
bool BL3_bootReport_isDssmRequested(void)
{
    if (s_bootReport == NULL)
    {
        return false;
    }

    /* BL2 sets debug_open_status = 1 when a DSSM debug flow is requested. */
    return (s_bootReport->debug_open_status == 1U);
}

/* ---------------------------------------------------------------------------
 * BL3_bootReport_getActiveVendorSlot
 * ---------------------------------------------------------------------------
 */
uint8_t BL3_bootReport_getActiveVendorSlot(void)
{
    if (s_bootReport == NULL)
    {
        return BL3_VENDOR_SLOT_INVALID;
    }

    OtaUpdate_ImageSelectReport_u sel = s_bootReport->otaReport.vendorAppImageReport.imageSelectReport;

    if (sel.slot1Ok && !sel.slot2Ok)
    {
        return BL3_VENDOR_SLOT_1;
    }
    if (!sel.slot1Ok && sel.slot2Ok)
    {
        return BL3_VENDOR_SLOT_2;
    }
    return BL3_VENDOR_SLOT_INVALID;
}

/* ---------------------------------------------------------------------------
 * BL3_bootReport_get
 * ---------------------------------------------------------------------------
 */
const bl3_bootReport_t *BL3_bootReport_get(void)
{
    return s_bootReport;
}

/* ---------------------------------------------------------------------------
 * BL3_bootReport_getPubkeyHash
 * ---------------------------------------------------------------------------
 */
const uint8_t *BL3_bootReport_getPubkeyHash(void)
{
    return (const uint8_t *)(BL3_BOOT_REPORT_ADDR + BL3_BOOT_REPORT_PUBKEY_HASH_OFFSET);
}

