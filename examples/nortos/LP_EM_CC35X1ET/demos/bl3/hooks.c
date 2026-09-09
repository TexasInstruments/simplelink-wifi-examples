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
 *  ======== hooks.c ========
 *
 *  Default implementations of all BL3 vendor customization hooks.
 *
 *  All functions are declared __attribute__((weak)) so that vendors can
 *  override any subset by providing their own definition with the same
 *  signature. See hooks.h for documentation on each hook.
 *
 *  BL3_HOOK_securityLock() is the only hook with a non-empty default:
 *  it asserts HOST_BOOT_DONE. Override it to apply your own locks first,
 *  then assert HOST_BOOT_DONE at the end.
 */

#include "hooks.h"
#include "handoff.h"

__attribute__((weak)) void BL3_HOOK_preBL3Hook(void)
{
    /* No-op. Override to add initialization that must run before drivers start. */
}

__attribute__((weak)) void BL3_HOOK_securityConfig(void)
{
    /* No-op. Override to configure any of the following topics:
     *   - Firewalls (host memory regions, peripherals — M33 Secure / Non-Secure)
     *   - DMA channel ownership and access settings
     *   - Device memory partition (MEMSS)
     *   - M33 memory Secure/Non-Secure partition
     *   - iCache size and address range
     *   - SAU region boundaries
     *   - MPU regions (Secure and Non-Secure)
     *   - IDAU (hardware-defined)
     *   - VTOR (Secure and Non-Secure vector table base)
     *   - M3 event routing
     *   - OTFDE XIP region parameters (address ranges) */
}

__attribute__((weak)) void BL3_HOOK_dssmHook(void)
{
    /* No-op. Override to implement BL3 debug operations. */
}

__attribute__((weak)) BL3_authStatus_t BL3_HOOK_rollbackProtection(
    const struct flash_area    *fap,
    const struct image_header  *hdr,
    const bl3_bootReport_t     *report)
{
    /* No-op. Default accepts any signature-valid image regardless of rollback
     * counter. Override to enforce vendor rollback policy using
     * bootutil_get_img_security_cnt(hdr, fap, ...) and the fuse status in
     * report->fuse_value_reporting[]. */
    (void)fap;
    (void)hdr;
    (void)report;
    return BL3_AUTH_STATUS_OK;
}

__attribute__((weak)) void BL3_HOOK_otaHook(void)
{
    /* No-op. Override to add OTA slot swap or confirmation logic. */
}

__attribute__((weak)) void BL3_HOOK_securityLock(void)
{
    /* Default: assert HOST_BOOT_DONE. Override to apply write-once security
     * locks before asserting HOST_BOOT_DONE at the end. Topics that can be
     * locked here:
     *   - iCache configuration
     *   - M33 memory Secure/Non-Secure partition
     *   - Device memory (MEMSS mode and firewall)
     *   - DMA channel ownership
     *   - OTFDE region parameters (permanent write-once lock for XIP regions)
     *   - M3 event routing
     *   - Peripherals firewall
     *   - VTOR (Secure and Non-Secure)
     *   - SAU
     *   - MPU (Secure and Non-Secure) */
    BL3_HANDOFF_setHostBootDone(); /* asserts HOST_BOOT_DONE, locking host security config */
}

__attribute__((weak)) void BL3_HOOK_postBL3Hook(void)
{
    /* No-op. Override to add vendor logic after HOST_BOOT_DONE. */
}

__attribute__((weak)) void BL3_HOOK_issueReport(bl3_bootReport_t *report)
{
    /* No-op. Override to write BL3 status or vendor data into the Boot Report
     * before handoff. Use report->bl3_report[]. */
    (void)report;
}
