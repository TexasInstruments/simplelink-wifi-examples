/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== bootutil_verify_sig.c ========
 *
 *  CC35xx adapter for MCUBoot's ECDSA signature verification.
 *
 *  Replaces the upstream definition in mcuboot/src/image_ecdsa.c (which must
 *  be excluded from the build).  The only functional difference from upstream
 *  is the trailing-zero trim on the signature buffer before it is handed to
 *  mbedTLS — the CC35xx signing tool pads each TLV to a 4-byte boundary, so
 *  the IMAGE_TLV_ECDSA_SIG buffer may contain up to 3 trailing 0x00 bytes
 *  after the DER signature ends.  Upstream mbedTLS's strict ASN.1 parser
 *  rejects those trailing bytes with MBEDTLS_ERR_ASN1_LENGTH_MISMATCH.
 *
 *  Trimming here keeps both mcuboot and mbedTLS unmodified — all platform
 *  quirks stay inside the CC35xx adaptation layer (mcuboot/ti/).
 */

#include <string.h>

#include "mcuboot_config/mcuboot_config.h"
#include "bootutil/bootutil_log.h"

BOOT_LOG_MODULE_DECLARE(mcuboot);

#if defined(MCUBOOT_SIGN_EC256) || defined(MCUBOOT_SIGN_EC384)

#include "bootutil_priv.h"
#include "bootutil/fault_injection_hardening.h"
#include "bootutil/crypto/ecdsa.h"

fih_ret
bootutil_verify_sig(uint8_t *hash, uint32_t hlen, uint8_t *sig, size_t slen,
                    uint8_t key_id)
{
    int rc;
    bootutil_ecdsa_context ctx;
    FIH_DECLARE(fih_rc, FIH_FAILURE);
    uint8_t *pubkey;
    uint8_t *end;

    BOOT_LOG_DBG("bootutil_verify_sig: ECDSA key %d", key_id);

    pubkey = (uint8_t *)bootutil_keys[key_id].key;
    end = pubkey + *bootutil_keys[key_id].len;
    bootutil_ecdsa_init(&ctx);

    rc = bootutil_ecdsa_parse_public_key(&ctx, &pubkey, end);
    if (rc) {
        goto out;
    }

    /* CC35xx adaptation: strip trailing alignment-padding zero bytes from
     * the signature length.  TLVs are 4-byte aligned by the signing tool. */
    while (slen > 0U && sig[slen - 1U] == 0x00U) {
        slen--;
    }

    rc = bootutil_ecdsa_verify(&ctx, pubkey, end - pubkey,
                               hash, hlen, sig, slen);
    fih_rc = fih_ret_encode_zero_equality(rc);
    if (FIH_NOT_EQ(fih_rc, FIH_SUCCESS)) {
        FIH_SET(fih_rc, FIH_FAILURE);
    }

out:
    bootutil_ecdsa_drop(&ctx);

    FIH_RET(fih_rc);
}

#endif /* MCUBOOT_SIGN_EC256 || MCUBOOT_SIGN_EC384 */
