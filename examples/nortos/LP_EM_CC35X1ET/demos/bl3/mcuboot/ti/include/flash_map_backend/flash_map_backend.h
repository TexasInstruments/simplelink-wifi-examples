/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== flash_map_backend.h ========
 *
 *  MCUBoot flash abstraction layer for CC35xx BL3.
 *
 *  flash_area_open(id, &fap) opens the XMEM driver for the given vendor
 *  slot (lazy XMEMWFF3_init on first call) and returns a flash_area pointing
 *  at the MCUBoot header within that slot.  flash_area_close releases it.
 *
 *  Slot ID values are defined in boot_report_types.h:
 *      BL3_VENDOR_SLOT_1 (0) → SysConfig vendor_image_slot_1
 *      BL3_VENDOR_SLOT_2 (1) → SysConfig vendor_image_slot_2
 *
 *  flash_area_read() goes through OTFDE. Must be called between open and close.
 *
 *  Only the subset of the flash_map API needed by image_validate.c, tlv.c,
 *  and bootutil_img_hash.c is implemented.
 */

#ifndef FLASH_MAP_BACKEND_H
#define FLASH_MAP_BACKEND_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct flash_area {
    uint8_t  fa_id;
    uint8_t  fa_device_id;
    uint16_t pad16;
    uint32_t fa_off;   /* XIP logical address of MCUBoot header in active slot */
    uint32_t fa_size;  /* usable byte size of the flash area */
};

struct flash_sector {
    uint32_t fs_off;
    uint32_t fs_size;
};

int  flash_area_open(uint8_t id, const struct flash_area **fap);
void flash_area_close(const struct flash_area *fap);
int  flash_area_read(const struct flash_area *fap, uint32_t off,
                     void *dst, uint32_t len);
int  flash_area_read_is_erased(const struct flash_area *fap, uint32_t off,
                               uint32_t len, uint8_t *out);
uint32_t flash_area_get_off(const struct flash_area *fap);
uint32_t flash_area_get_size(const struct flash_area *fap);
uint8_t  flash_area_get_id(const struct flash_area *fap);
uint8_t  flash_area_get_device_id(const struct flash_area *fap);
uint16_t flash_area_get_align(const struct flash_area *fap);

struct boot_loader_state;
uint32_t bootutil_max_image_size(struct boot_loader_state *state,
                                 const struct flash_area *fap);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_MAP_BACKEND_H */
