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
 *  ======== boot_report.h ========
 *
 *  BL3 Boot Report module.
 *
 *  Responsibilities:
 *    - Read and validate the Boot Report written by BL2 at 0x28000100
 *      (M33 view). Detect BL2 errors and DSSM requests.
 *    - Append BL3's own status before handing off to the vendor application.
 *
 *  The Boot Report region is owned by BL2. BL3 does not allocate or
 *  initialize it — it is present at address BL3_BOOT_REPORT_ADDR when
 *  M33 starts executing BL3.
 */

#ifndef BL3_BOOT_REPORT_H_
#define BL3_BOOT_REPORT_H_

#include <stdbool.h>
#include <stdint.h>
#include "boot_report_types.h"

/*
 * Initialize the boot report module and validate BL2's status.
 *
 * Caches a pointer to the Boot Report at BL3_BOOT_REPORT_ADDR.
 * Checks ti_bootloader_bl2_status — if non-zero, BL2 encountered an error
 * and BL3 must halt.
 *
 * Returns  0  BL2 status is OK; boot can proceed
 *         -1  BL2 reported an error; caller must invoke BL3_errorHandler()
 */
int32_t BL3_bootReport_init(void);

/*
 * Returns true if the Boot Report indicates a DSSM debug flow is requested.
 * Based on debug_open_status written by BL2.
 * Must be called after BL3_bootReport_init().
 */
bool BL3_bootReport_isDssmRequested(void);

/*
 * Returns the active vendor application slot index based on
 * vendorAppImageReport.imageSelectReport written by BL2:
 *   slot1Ok = 1, slot2Ok = 0  →  BL3_VENDOR_SLOT_1 (0)
 *   slot1Ok = 0, slot2Ok = 1  →  BL3_VENDOR_SLOT_2 (1)
 *   else                      →  BL3_VENDOR_SLOT_INVALID (0xFF)
 *
 * The returned value is the id passed to flash_area_open(); the slot table
 * in flash_map_backend.c maps it to the corresponding SysConfig slot
 * (slot 0 → vendor_image_*_slot_1_*, slot 1 → vendor_image_*_slot_2_*).
 *
 * Must be called after BL3_bootReport_init().
 */
uint8_t BL3_bootReport_getActiveVendorSlot(void);

/*
 * Returns a read-only pointer to the Boot Report for inspection or
 * pass-through to the vendor application. Must be called after init.
 */
const bl3_bootReport_t *BL3_bootReport_get(void);

/*
 * Returns a pointer to the 32-byte vendor public key SHA-256 hash written
 * by BL2 at Boot Report offset 0x1f (absolute address 0x2800011f).
 * Used by bootutil_find_key() to validate the IMAGE_TLV_PUBKEY against
 * the ROT hash BL2 read from eFUSE rows 5-12.
 * Must be called after BL3_bootReport_init().
 */
const uint8_t *BL3_bootReport_getPubkeyHash(void);


#endif /* BL3_BOOT_REPORT_H_ */
