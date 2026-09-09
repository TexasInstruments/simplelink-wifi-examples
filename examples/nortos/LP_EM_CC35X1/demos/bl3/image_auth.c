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
 *  ======== image_auth.c ========
 *
 *  Vendor application image authentication using mbedTLS (SW, no HSM).
 *
 *  Flow:
 *    1. Read 32-byte image_header at fa_off (DMA + STIG substitution for
 *       ih_magic). Verify ih_magic == IMAGE_MAGIC.
 *    2. bootutil_img_validate:
 *         - bootutil_img_hash: SHA-256 over (header + body + protected TLVs)
 *         - bootutil_tlv_iter_*: locate TLVs
 *         - IMAGE_TLV_SHA256 compare
 *         - IMAGE_TLV_PUBKEY -> bootutil_find_key (ROT fuse compare; STUB)
 *         - IMAGE_TLV_ECDSA_SIG -> bootutil_verify_sig (mbedTLS ECDSA-P256)
 *    3. BL3_HOOK_rollbackProtection: vendor-overridable rollback policy
 *       (default no-op accepts the image).
 *
 *  All crypto is SW-only via mbedTLS (mcuboot/mbedtls/). No HSM involved.
 *  The caller (boot_flow.c) owns flash_area_open / flash_area_close.
 */

#include <stdint.h>
#include <string.h>

#include "bootutil/image.h"
#include "bootutil/fault_injection_hardening.h"
#include "flash_map_backend/flash_map_backend.h"
#include "image_auth.h"
#include "boot_report.h"
#include "config.h"
#include "hooks.h"
#include "uart.h"

/* BOOT_TMPBUF_SZ (256) from bootutil_priv.h */
#ifndef BOOT_TMPBUF_SZ
#define BOOT_TMPBUF_SZ 256
#endif

/* IMAGE_MAGIC is gated on DEVICE_TYPE_MX1 in bootutil/image.h.  This project
 * builds with -DCC35XX, not -DDEVICE_TYPE_MX1, so define the MX1 value
 * explicitly here. */
#ifndef IMAGE_MAGIC
#define IMAGE_MAGIC 0x690c47c2U
#endif

/* ---------------------------------------------------------------------------
 * BL3_AUTH_verifyImage
 * ---------------------------------------------------------------------------
 */
BL3_authStatus_t BL3_AUTH_verifyImage(const struct flash_area *fap,
                                      struct image_header     *hdrOut)
{
    struct image_header   hdr;
    struct image_tlv_info tlvInfo;
    struct flash_area     authFap;
    uint8_t tmp_buf[BOOT_TMPBUF_SZ];
    fih_ret fih_rc = FIH_FAILURE;
    BL3_authStatus_t hookStatus;
    uint32_t tlvOff;
    int rc;

    if (fap == NULL || hdrOut == NULL) {
        return BL3_AUTH_STATUS_DRIVER_ERR;
    }

    BL3_DBG_print("[BL3] Image auth: fa_off=0x%08X, slot=%u\r\n",
                  (unsigned)fap->fa_off, BL3_SLOT_DISPLAY(fap->fa_id));

    /* Header read. flash_area_read transparently substitutes the first 4 bytes
     * (ih_magic) with a STIG read so the 32-byte image_header is correct as
     * a contiguous block. */
    rc = flash_area_read(fap, 0, &hdr, sizeof(hdr));
    if (rc != 0) {
        BL3_DBG_print("[BL3] Image auth: header read failed\r\n");
        return BL3_AUTH_STATUS_DRIVER_ERR;
    }

    if (hdr.ih_magic != IMAGE_MAGIC) {
        BL3_DBG_print("[BL3] Image auth: bad magic 0x%08X (expected 0x%08X)\r\n",
                      (unsigned)hdr.ih_magic, (unsigned)IMAGE_MAGIC);
        return BL3_AUTH_STATUS_INVALID_HDR;
    }

    /* ih_hdr_size must be at least sizeof(hdr) — a smaller value would place
     * tlvOff inside the header itself.  ih_img_size is not bounded by slot
     * size: GPE images store the encrypted body size which can exceed the
     * nominal slot window. */
    if (hdr.ih_hdr_size < sizeof(hdr))
    {
        BL3_DBG_print("[BL3] Image auth: ih_hdr_size too small (0x%X)\r\n",
                      (unsigned)hdr.ih_hdr_size);
        return BL3_AUTH_STATUS_INVALID_HDR;
    }

    /* Compute actual image size (header + body + protected TLVs + unprotected
     * TLVs + 0xFF safety margin) and pass a corrected flash_area to
     * bootutil_img_validate. fa_size from flash_area_open reflects the slot
     * size, not the image; flash_area_read has no bounds check so reading
     * past fa_size is safe here. */
    tlvOff = hdr.ih_hdr_size + hdr.ih_img_size + hdr.ih_protect_tlv_size;
    rc = flash_area_read(fap, tlvOff, &tlvInfo, sizeof(tlvInfo));
    if (rc != 0) {
        return BL3_AUTH_STATUS_DRIVER_ERR;
    }

    /* Guard against uint32_t overflow in fa_size before signature check. */
    if ((uint64_t)tlvOff + tlvInfo.it_tlv_tot + 0xFFU > UINT32_MAX)
    {
        BL3_DBG_print("[BL3] Image auth: fa_size overflow\r\n");
        return BL3_AUTH_STATUS_INVALID_HDR;
    }
    authFap         = *fap;
    authFap.fa_size = tlvOff + tlvInfo.it_tlv_tot + 0xFFU;

    /* SHA-256 + ECDSA-P256 signature verify via mbedTLS. */
    FIH_CALL(bootutil_img_validate, fih_rc,
             NULL,          /* boot_loader_state — not used in DIRECT_XIP */
             &hdr,
             &authFap,
             tmp_buf, sizeof(tmp_buf),
             NULL, 0,       /* seed / seed_len — not used */
             NULL);         /* out_hash — not needed by BL3 */


    if (FIH_NOT_EQ(fih_rc, FIH_SUCCESS)) {
        BL3_DBG_print("[BL3] Image auth: signature FAILED (fih_rc=%d)\r\n",
                      (int)fih_rc);
        return BL3_AUTH_STATUS_SIG_FAIL;
    }

    /* Vendor rollback policy hook. Default no-op accepts the image. */
    hookStatus = BL3_HOOK_rollbackProtection(fap, &hdr, BL3_bootReport_get());
    if (hookStatus != BL3_AUTH_STATUS_OK) {
        BL3_DBG_print("[BL3] Image auth: rollback hook rejected (status=%d)\r\n",
                      (int)hookStatus);
        return hookStatus;
    }

    *hdrOut = hdr;
    BL3_DBG_print("[BL3] Image auth: OK\r\n");
    return BL3_AUTH_STATUS_OK;
}

