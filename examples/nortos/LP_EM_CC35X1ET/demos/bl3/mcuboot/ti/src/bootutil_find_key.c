/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== bootutil_find_key.c ========
 *
 *  MCUBoot MCUBOOT_HW_KEY implementation for CC35xx BL3.
 *
 *  Called by image_validate.c when IMAGE_TLV_PUBKEY is found in the image TLV
 *  area.  This function:
 *
 *    1. Computes SHA-256 of the DER-encoded public key from the TLV.
 *    2. Compares the hash against the ROT pubkey hash BL2 read from eFUSE
 *       rows 5-12 and wrote to Boot Report offset 0x1f (0x2800011f).
 *       Returns -1 on mismatch to abort authentication.
 *    3. Stores a pointer to the key in bootutil_keys[0] so that
 *       bootutil_verify_sig() (image_ecdsa.c) can access it for ECDSA
 *       verification.
 *
 *  boot_retrieve_public_key_hash() is declared in sign_key.h for the
 *  MCUBOOT_HW_KEY path; it is provided here as a companion stub (not used
 *  by this implementation — the comparison is done inside bootutil_find_key).
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "bootutil/sign_key.h"
#include "bootutil/crypto/sha.h"
#include "bootutil/fault_injection_hardening.h"
#include "bootutil_priv.h"
#include "boot_report.h"
#include "config.h"

/* Storage for the key length — bootutil_keys[0].len points here */
static unsigned int s_key_len;

/* bootutil_keys[] and bootutil_key_cnt are defined in bootutil_keys.c */
extern struct bootutil_key bootutil_keys[];

/*
 * bootutil_find_key — called by image_validate.c with MCUBOOT_HW_KEY.
 *
 * Parameters:
 *   image_index  image slot index (0 for single-image BL3)
 *   key          pointer to the DER SubjectPublicKeyInfo from IMAGE_TLV_PUBKEY
 *   len          byte length of key
 *
 * Returns:
 *   0   key accepted; bootutil_keys[0] populated for verify_sig
 *  -1   key rejected (hash mismatch against ROT hash from Boot Report)
 */
int bootutil_find_key(int image_index, uint8_t *key, uint16_t len)
{
    (void)image_index;

    /* Trim TLV alignment padding to the actual DER SPKI length. */
    if (len >= 2 && key[0] == 0x30U && key[1] < 0x80U) {
        len = (uint16_t)(2U + key[1]);
    }

    /* Hash the TLV public key and compare against the ROT hash from Boot Report.
     * BL2 reads the ROT pubkey hash from eFUSE rows 5-12 and writes it at
     * Boot Report offset 0x1f (address 0x2800011f). */
    uint8_t key_hash[IMAGE_HASH_SIZE];
    bootutil_sha_context sha;
    bootutil_sha_init(&sha);
    bootutil_sha_update(&sha, key, len);
    bootutil_sha_finish(&sha, key_hash);
    bootutil_sha_drop(&sha);

    FIH_DECLARE(fih_rc, FIH_FAILURE);
    FIH_CALL(boot_fih_memequal, fih_rc, key_hash, BL3_bootReport_getPubkeyHash(), BL3_BOOT_REPORT_PUBKEY_HASH_SIZE);
    if (FIH_NOT_EQ(fih_rc, FIH_SUCCESS)) {
        return -1;
    }

    /* Wire the TLV key into bootutil_keys[0] for bootutil_verify_sig */
    s_key_len = (unsigned int)len;
    bootutil_keys[0].key = key;
    bootutil_keys[0].len = &s_key_len;

    return 0;
}

/*
 * boot_retrieve_public_key_hash — declared in sign_key.h for MCUBOOT_HW_KEY.
 *
 * Not used by this implementation (comparison is done in bootutil_find_key).
 * Provided to satisfy the linker; returns the ROT hash from the Boot Report.
 */
int boot_retrieve_public_key_hash(uint8_t image_index,
                                  uint8_t *public_key_hash,
                                  size_t *key_hash_size)
{
    (void)image_index;
    memcpy(public_key_hash, BL3_bootReport_getPubkeyHash(), IMAGE_HASH_SIZE);
    *key_hash_size = IMAGE_HASH_SIZE;
    return 0;
}
