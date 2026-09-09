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
 *  ======== hooks.h ========
 *
 *  VENDOR CUSTOMIZATION INTERFACE
 *  ================================
 *  All functions declared here are defined as __attribute__((weak)) stubs
 *  in hooks.c. To customize BL3 for your product, override any subset
 *  of these functions by providing your own non-weak definition — in your
 *  own source file, with the same signature. The linker will prefer your
 *  definition over the default no-op stub.
 *
 *  You do NOT need to modify hooks.c or any other BL3 source file.
 *  The hook mechanism is the only customization interface provided.
 *
 *  Hook call order in boot_flow.c:
 *
 *    BL3_HOOK_preBL3Hook()           first call, before driver init
 *    BL3_HOOK_securityConfig()       platform security configuration (no-op by default)
 *    BL3_HOOK_dssmHook()             only if DSSM flow is requested by Boot Report
 *    BL3_HOOK_rollbackProtection()   after image SHA/ECDSA validation succeeds
 *    BL3_HOOK_otaHook()              after image authentication succeeds
 *    BL3_HOOK_securityLock()         apply security locks, assert HOST_BOOT_DONE at end
 *    BL3_HOOK_postBL3Hook()          after HOST_BOOT_DONE
 *    BL3_HOOK_issueReport()          write BL3 status to Boot Report before handoff
 */

#ifndef BL3_HOOKS_H_
#define BL3_HOOKS_H_

#include "image_auth.h"
#include "boot_report_types.h"

struct flash_area;
struct image_header;

/* ---------------------------------------------------------------------------
 * Pre-BL3 hook
 *
 * First call in main(), before any driver initialization.
 *
 * Override to add any initialization that must run before drivers start.
 * ---------------------------------------------------------------------------
 */
void BL3_HOOK_preBL3Hook(void);

/* ---------------------------------------------------------------------------
 * Platform security configuration
 *
 * Called after Boot Report validation.
 *
 * No-op by default. Override to configure any of the following topics:
 *
 *   Firewalls       — access rights settings for host memory regions
 *                     and peripherals (M33 Secure, M33 Non-Secure)
 *   DMA             — channel ownership and access settings
 *   Device memory   — device memory partition mode (MEMSS)
 *   M33 memory      — Secure/Non-Secure partition of internal M33 memories
 *   iCache          — instruction cache size and address range
 *   SAU             — Secure Attribution Unit region configuration
 *   MPU             — Memory Protection Unit region configuration (Secure and Non-Secure)
 *   IDAU            — Implementation Defined Attribution Unit (hardware-defined)
 *   VTOR            — vector table base address (Secure and Non-Secure)
 *   M3 events       — M3 subsystem event routing
 *   OTFDE           — XIP region parameters (address ranges)
 * ---------------------------------------------------------------------------
 */
void BL3_HOOK_securityConfig(void);

/* ---------------------------------------------------------------------------
 * DSSM debug flow
 *
 * Called only when BL3_bootReport_isDssmRequested() returns true. Indicates
 * that BL2 detected a debug boot request.
 *
 * No-op by default. Override to implement BL3 debug operations.
 * ---------------------------------------------------------------------------
 */
void BL3_HOOK_dssmHook(void);

/* ---------------------------------------------------------------------------
 * Vendor rollback protection
 *
 * Called after image SHA-256 and ECDSA signature validation succeeds.
 * This hook is the entire vendor rollback policy — if you implement it,
 * you implement everything:
 *
 *   1. Extract the image security counter from IMAGE_TLV_SEC_CNT (TLV type
 *      0x50) using fap + hdr.  Helper: bootutil_get_img_security_cnt().
 *   2. Read the vendor application rollback fuse status from BL2's boot
 *      report (report->fuse_value_reporting[]).  M33 cannot read fuses
 *      directly — it must rely on what BL2 reported.
 *   3. Compare and decide.  Return BL3_AUTH_STATUS_OK to accept,
 *      BL3_AUTH_STATUS_ROLLBACK_FAIL to reject.
 *
 * The default no-op stub returns BL3_AUTH_STATUS_OK, accepting any image
 * that passed signature verification regardless of rollback counter.
 * ---------------------------------------------------------------------------
 */
BL3_authStatus_t BL3_HOOK_rollbackProtection(const struct flash_area    *fap,
                                              const struct image_header  *hdr,
                                              const bl3_bootReport_t     *report);

/* ---------------------------------------------------------------------------
 * OTA handling
 *
 * Called after image authentication succeeds. Override to add vendor OTA
 * logic such as checking for a pending update, swapping image slots, or
 * marking a slot as confirmed.
 *
 * Atomic staging of BL3 + vendor image is enforced by the FWU driver.
 * ---------------------------------------------------------------------------
 */
void BL3_HOOK_otaHook(void);

/* ---------------------------------------------------------------------------
 * Security lock
 *
 * Apply write-once security locks and assert HOST_BOOT_DONE here.
 *
 * The default implementation asserts HOST_BOOT_DONE (real work, not a no-op).
 * Override to apply vendor locks before asserting HOST_BOOT_DONE at the end:
 *
 *   iCache              — instruction cache configuration lock
 *   M33 memory          — Secure/Non-Secure partition lock
 *   Device memory       — MEMSS mode and firewall lock
 *   DMA channel owner   — DMA ownership lock
 *   OTFDE region params — OTFDE region parameters lock
 *   M3 events           — M3 event routing lock
 *   Peripherals         — peripherals firewall lock
 *   VTOR                — vector table lock
 *   SAU                 — Secure Attribution Unit lock
 *   MPU                 — Memory Protection Unit lock (Secure and Non-Secure)
 *
 * Always assert HOST_BOOT_DONE at the end of your override.
 * ---------------------------------------------------------------------------
 */
void BL3_HOOK_securityLock(void);

/* ---------------------------------------------------------------------------
 * Post-BL3 vendor hook
 *
 * Called after HOST_BOOT_DONE is asserted.
 * ---------------------------------------------------------------------------
 */
void BL3_HOOK_postBL3Hook(void);

/* ---------------------------------------------------------------------------
 * Issue report
 *
 * Called after BL3_HOOK_postBL3Hook(), before launching the vendor application.
 * The Boot Report pointer is passed directly — write any BL3 status or
 * additional data for the vendor application into the available fields:
 *
 *   bl3_report[512]   — 512 B, BL3 status (written by BL3)
 *
 * No-op by default.
 * ---------------------------------------------------------------------------
 */
void BL3_HOOK_issueReport(bl3_bootReport_t *report);

#endif /* BL3_HOOKS_H_ */
