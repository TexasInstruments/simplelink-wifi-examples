/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== security_cnt.c ========
 *
 *  BL3 stub: rollback counter is always 0.
 *  Replace with eFUSE-backed implementation when anti-rollback is required.
 *
 *  Signature matches bootutil/image.h (BL3 uses the image_header/flash_area
 *  API, not the boot_loader_state API from vanilla MCUBoot).
 */

#include <stdint.h>
#include "bootutil/image.h"
#include "flash_map_backend/flash_map_backend.h"

int32_t bootutil_get_img_security_cnt(struct image_header *hdr,
                                      const struct flash_area *fap,
                                      uint32_t *security_cnt)
{
    (void)hdr;
    (void)fap;
    *security_cnt = 0;
    return 0;
}
