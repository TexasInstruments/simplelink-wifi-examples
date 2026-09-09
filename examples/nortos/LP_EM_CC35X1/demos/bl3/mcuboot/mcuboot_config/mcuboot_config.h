/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== mcuboot_config.h ========
 *
 *  MCUBoot compile-time configuration for BL3 image authentication.
 *  SW-only path: SHA-256 + ECDSA SECP256R1 via mbedTLS. No HSM.
 *
 *  To switch to the PSA/HSM path later: replace MCUBOOT_USE_MBED_TLS
 *  with MCUBOOT_USE_PSA_CRYPTO. No other changes needed.
 */

#ifndef MCUBOOT_CONFIG_H
#define MCUBOOT_CONFIG_H

/* Crypto backend: mbedTLS SW-only */
#define MCUBOOT_USE_MBED_TLS

/* Signing algorithm: ECDSA SECP256R1 (vendor-configurable per spec) */
#define MCUBOOT_SIGN_EC256

/*
 * Key source: IMAGE_TLV_PUBKEY in the image TLV area.
 * The public key is carried in the image itself; BL3 hashes it and
 * compares against vendor ROT fuses (eFUSE rows 5-12).
 */
#define MCUBOOT_HW_KEY

/* Execution model: XIP from flash, no slot swapping */
#define MCUBOOT_DIRECT_XIP

/* Single image (vendor application) */
#define BOOT_IMAGE_NUMBER       1

/* Maximum number of flash sectors per image slot */
#define MCUBOOT_MAX_IMG_SECTORS 128

/* Fault injection hardening: MEDIUM matches BL2's hardening level */
#define MCUBOOT_FIH_PROFILE_MEDIUM

/* Logging: not used in bare-metal BL3 */
/* Do NOT define MCUBOOT_HAVE_LOGGING */

#endif /* MCUBOOT_CONFIG_H */
