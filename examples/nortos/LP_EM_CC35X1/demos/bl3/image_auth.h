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
 *  ======== image_auth.h ========
 *
 *  BL3 vendor application image authentication — SW only (mbedTLS, no HSM).
 *
 *  Full authentication flow (spec: bl3_learning_guide.html, Image Auth):
 *    1. Extract Public Key TLV from the non-protected VA-TLV
 *    2. SHA-256 the public key; compare against ROT fuses (eFUSE rows 5-12)
 *    4. Use ROT public key as the authentication key
 *    5a. Read signing algorithm from Public Key TLV (default: ECDSA SECP256R1)
 *    5b. Verify Signature TLV over (header + payload + protected TLVs) via mbedTLS
 *    6. Validate rollback counter against vendor app fuses (rows 25-26)
 *    7. Report result
 *
 *  Image layout (MCUBoot format, GPE-wrapped):
 *
 *    [ GPE manifest (0x1000 bytes) ]
 *    [ MCUBoot image header (32 bytes, struct image_header) ]
 *    [ Image body (ih_img_size bytes) ]
 *    [ Protected TLV area (ih_protect_tlv_size bytes) ]
 *    [ TLV info magic + TLV entries ]
 *      IMAGE_TLV_SHA256 (0x10)       : SHA-256 hash (protected)
 *      IMAGE_TLV_ECDSA_SIG (0x22)    : ECDSA signature (protected)
 *      Security counter TLV (0x50)   : rollback counter
 *    [ VA-TLV (non-protected, appended after TLV area) ]
 *      Public Key TLV                : vendor ECC public key (92 bytes)
 *      Signature TLV                 : ECDSA signature over image
 */

#ifndef BL3_IMAGE_AUTH_H_
#define BL3_IMAGE_AUTH_H_

#include <stdint.h>

/* Return codes from BL3_AUTH_verifyImage() */
typedef enum
{
    BL3_AUTH_STATUS_OK            =  0,  /* Image authenticated successfully */
    BL3_AUTH_STATUS_SIG_FAIL      = -1,  /* ECDSA signature verification failed */
    BL3_AUTH_STATUS_ROT_FAIL      = -2,  /* Public key does not match ROT fuses */
    BL3_AUTH_STATUS_ROLLBACK_FAIL = -3,  /* Image version below rollback minimum */
    BL3_AUTH_STATUS_TLV_NOT_FOUND = -4,  /* Required TLV not present in image */
    BL3_AUTH_STATUS_INVALID_HDR   = -5,  /* Image header magic mismatch */
    BL3_AUTH_STATUS_DRIVER_ERR    = -6,  /* Flash/xmem driver error */
} BL3_authStatus_t;

/*
 * Authenticate the vendor application image in the active flash slot.
 *
 * fap     Pointer to the flash_area returned by flash_area_open(), already
 *         pointing at the GPE manifest (slot+0xFFC) within the active slot.
 *         Caller owns the open/close lifecycle.
 * hdrOut  [out] On success, receives a copy of the validated 32-byte
 *         image_header.  Caller uses hdrOut->ih_hdr_size to compute the
 *         vendor app entry point.
 *
 * Validation phases run in order:
 *   - Magic check (ih_magic == IMAGE_MAGIC)
 *   - bootutil_img_validate (SHA-256, ECDSA-P256, key/ROT lookup)
 *   - BL3_HOOK_rollbackProtection (vendor-overridable, default no-op)
 *
 * Returns BL3_AUTH_STATUS_OK on success, or a negative error code.
 * On failure, boot_flow.c invokes BL3_errorHandler() — the system halts.
 */
struct flash_area;
struct image_header;
BL3_authStatus_t BL3_AUTH_verifyImage(const struct flash_area *fap,
                                      struct image_header     *hdrOut);

#endif /* BL3_IMAGE_AUTH_H_ */
