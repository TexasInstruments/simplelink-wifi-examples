/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== bootutil_keys.c ========
 *
 *  MCUBoot key table for BL3 MCUBOOT_HW_KEY mode.
 *
 *  With MCUBOOT_HW_KEY the public key is carried in the image TLV area
 *  (IMAGE_TLV_PUBKEY) rather than compiled into the bootloader.
 *  bootutil_find_key() (bootutil_find_key.c) populates bootutil_keys[0]
 *  at runtime when the TLV key is found.
 *
 *  bootutil_key_cnt must be >= 1 so that image_validate.c accepts key_id = 0
 *  returned by bootutil_find_key().
 */

#include "bootutil/sign_key.h"

struct bootutil_key bootutil_keys[1] = {
    { .key = NULL, .len = NULL },
};

const int bootutil_key_cnt = 1;
