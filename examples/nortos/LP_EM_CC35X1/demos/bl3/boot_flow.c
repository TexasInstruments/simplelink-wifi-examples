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
 *  ======== boot_flow.c ========
 *
 *  BL3 boot sequence.
 *
 *  Boot flow:
 *    1. Pre-BL3 hook
 *    2. Read and validate BL2 Boot Report
 *    3. Platform security configuration hook
 *    4. DSSM debug flow (only if requested by Boot Report)
 *    5. Authenticate vendor application image (SHA-256 + ECDSA)
 *    6. OTA hook
 *    7. Security lock hook (asserts HOST_BOOT_DONE by default)
 *    8. Post-BL3 hook, issue report hook, launch vendor app
 *
 *  To customize BL3 for your product, override the hook functions declared
 *  in hooks.h. You do not need to modify this file.
 */

#include "config.h"
#include "hooks.h"
#include "boot_report.h"
#include "image_auth.h"
#include "handoff.h"
#include "uart.h"
#include "gpio.h"
#include "boot_flow.h"
#include "bootutil/image.h"
#include "flash_map_backend/flash_map_backend.h"


static void BL3_errorHandler(void)
{
    BL3_DBG_print("[BL3] FATAL: errorHandler called — halting\r\n");
    while (1) {}
}

void BL3_runBootFlow(void)
{
    /* Call driver init functions */    
    /* init the gpio settings if enabled at the beginning of the BL3*/
    BL3_GPIO_init();
    
    /* Debug UART: initialized here */
    BL3_UART_init();

    /* Pre-BL3 hook — runs before any driver initialization. */
    BL3_HOOK_preBL3Hook();
    
    BL3_DBG_print("[BL3] Starting\r\n");

    /* GPIO loopback test: BP.25 (GPIO4, input) <-> BP.26 (GPIO3, output) */
    BL3_GPIO_loopbackTest();

    /* Read the BL2 Boot Report. Halt if BL2 reported an error. */
    if (BL3_bootReport_init() != 0)
    {
        BL3_errorHandler();
    }

    /* Platform security configuration hook. */
    BL3_HOOK_securityConfig();

    /* DSSM debug flow — skipped on normal boot. */
    if (BL3_bootReport_isDssmRequested())
    {
        BL3_DBG_print("[BL3] DSSM flow requested\r\n");
        BL3_HOOK_dssmHook();
    }

    /* Authenticate the vendor application image. Halt if authentication fails. */
    uint8_t activeSlot = BL3_bootReport_getActiveVendorSlot();
    if (activeSlot == BL3_VENDOR_SLOT_INVALID)
    {
        BL3_DBG_print("[BL3] No valid vendor slot reported by BL2\r\n");
        BL3_errorHandler();
    }
    BL3_DBG_print("[BL3] Active vendor slot = %u\r\n", BL3_SLOT_DISPLAY(activeSlot));

    const struct flash_area *vendorFap = NULL;
    if (flash_area_open(activeSlot, &vendorFap) != 0)
    {
        BL3_DBG_print("[BL3] flash_area_open failed (slot=%u)\r\n",
                      BL3_SLOT_DISPLAY(activeSlot));
        BL3_errorHandler();
    }

    struct image_header vendorHdr;
    if (BL3_AUTH_verifyImage(vendorFap, &vendorHdr) != BL3_AUTH_STATUS_OK)
    {
        flash_area_close(vendorFap);
        BL3_errorHandler();
    }

    uint32_t vendorAppBase = BL3_VENDOR_APP_BASE;

    flash_area_close(vendorFap);
    BL3_DBG_print("[BL3] Image authentication passed\r\n");

    /* OTA hook. */
    BL3_HOOK_otaHook();

    /* Security lock hook — applies write-once locks and asserts HOST_BOOT_DONE. */
    BL3_HOOK_securityLock();

    /* Post-BL3 vendor hook. */
    BL3_HOOK_postBL3Hook();

    /* Issue report hook — vendor writes BL3 status to the Boot Report. */
    BL3_HOOK_issueReport((bl3_bootReport_t *)BL3_bootReport_get());

    BL3_DBG_print("[BL3] Launching vendor app at 0x%08X\r\n",
                  (unsigned)vendorAppBase);
    /* Restore GPIO3/GPIO4 and UART pin mux before handing off. */    
    BL3_UART_deinit();

    /* Launch vendor application. Does not return. */
    BL3_HANDOFF_launchApp(vendorAppBase);

    /* Unreachable */
    while (1) {}
}
