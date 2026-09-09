/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 *  ======== flash_map_backend.c ========
 *
 *  MCUBoot flash abstraction — CC35xx XMEMWFF3 adaptation.
 *
 *  flash_area_open(id, &fap) does the heavy lifting:
 *    - lazy XMEMWFF3_init_nortos() on first call
 *    - XMEMWFF3_open_nortos() with regionBase / regionStartAddr / regionSize
 *      selected from the slot table (id 0 → SysConfig vendor_image_slot_1,
 *      id 1 → vendor_image_slot_2)
 *    - returns a const pointer to an internal flash_area populated with
 *      the MCUBoot header offset and usable size for that slot
 *
 *  flash_area_close() releases the XMEM handle.
 *
 *  XMEMWFF3_read_nortos() takes an OFFSET from the slot's regionStartAddr (logical
 *  base), so flash_area_read() computes:
 *
 *    xmem_offset = (xip_addr - active_logical_base)
 *
 *  The driver uses regionBase internally for STIG (physical flash) and
 *  regionStartAddr for XIP/DMA reads.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "xmem/XMEMWFF3_nortos.h"
#include "ti_flash_map_config.h"
#include "config.h"
#include "boot_report_types.h"          /* BL3_VENDOR_SLOT_1 / _SLOT_2 ids */
#include "flash_map_backend/flash_map_backend.h"

/* ---------------------------------------------------------------------------
 * Slot table — populated lazily from SysConfig externs.
 *
 * Index meaning matches BL3_VENDOR_SLOT_1 / BL3_VENDOR_SLOT_2 (0 / 1).
 * SysConfig names slots 1-indexed; BL2 reports them 0-indexed.
 * ---------------------------------------------------------------------------
 */
typedef struct {
    uint32_t physical;    /* flash device physical base — used for STIG */
    uint32_t logical;     /* XIP logical base — used for DMA + offset math */
    uint32_t size;        /* slot size in bytes */
} bl3_slot_entry_t;

#define BL3_NUM_SLOTS 2

static bl3_slot_entry_t s_slots[BL3_NUM_SLOTS];

/* Per-open state — a single open is supported at a time (one active vendor slot) */
static struct flash_area s_fap;
static XMEM_Handle_nortos  s_xmemHandle       = NULL;
static uint32_t            s_activeLogical    = 0U;
static bool                s_xmemInitialized  = false;

/* ---------------------------------------------------------------------------
 * initSlotTable — copy SysConfig extern values into the slot table once.
 * ---------------------------------------------------------------------------
 */
static void initSlotTable(void)
{
    s_slots[BL3_VENDOR_SLOT_1].physical = vendor_image_physical_slot_1_address;
    s_slots[BL3_VENDOR_SLOT_1].logical  = vendor_image_logical_slot_1_address;
    s_slots[BL3_VENDOR_SLOT_1].size     = vendor_image_slot_1_region_size;

    s_slots[BL3_VENDOR_SLOT_2].physical = vendor_image_physical_slot_2_address;
    s_slots[BL3_VENDOR_SLOT_2].logical  = vendor_image_logical_slot_2_address;
    s_slots[BL3_VENDOR_SLOT_2].size     = vendor_image_slot_2_region_size;
}

/* ---------------------------------------------------------------------------
 * flash_area_open — open XMEM for the given slot id and return a flash_area
 * pointing at the MCUBoot header within that slot.
 *
 *   id      0 = SysConfig vendor_image_slot_1; 1 = vendor_image_slot_2
 *   fap_out [out] receives a pointer to the internal flash_area on success
 *
 * Returns 0 on success, -1 on bad id or XMEM open failure.
 * ---------------------------------------------------------------------------
 */
int flash_area_open(uint8_t id, const struct flash_area **fap_out)
{
    if (id >= BL3_NUM_SLOTS || fap_out == NULL) {
        return -1;
    }

    if (!s_xmemInitialized) {
        XMEMWFF3_init_nortos();
        initSlotTable();
        s_xmemInitialized = true;
    }

    /* BL2's SetVendorExeRegion() swaps OTFDE region 0 <-> region 2 when slot 2
     * is selected so the active slot is ALWAYS XIP-mapped at
     * vendor_image_logical_slot_1_address (0x14000000) by the time BL3 runs.
     * DMA/XIP reads therefore always target slot 1's logical base regardless
     * of which physical slot is active.  STIG bypasses OTFDE and goes
     * straight to physical flash — that's the only path that needs the
     * active slot's physical address. */
    XMEM_Params_nortos params;
    params.deviceNum       = XMEM_FLASH_NORTOS;
    params.regionBase      = (size_t)s_slots[id].physical;                 /* id-dependent (STIG)      */
    params.regionStartAddr = (size_t)vendor_image_logical_slot_1_address;  /* always 0x14000000 (DMA)  */
    params.regionSize      = (size_t)s_slots[id].size;

    s_xmemHandle = XMEMWFF3_open_nortos(&params);
    if (s_xmemHandle == NULL) {
        return -1;
    }

    s_activeLogical = vendor_image_logical_slot_1_address;

    /* fa_off points at the GPE manifest's ih_magic position (slot+0xFFC).
     * The image_header is a contiguous 32 bytes from fa_off to fa_off+0x1F.
     * The first 4 bytes (magic) are STIG-written by BL2; flash_area_read
     * substitutes them with a STIG read transparently when off == 0. */
    s_fap.fa_id        = id;
    s_fap.fa_device_id = 0;
    s_fap.pad16        = 0;
    s_fap.fa_off       = vendor_image_logical_slot_1_address + BL3_GPE_HEADER_OFFSET;
    s_fap.fa_size      = BL3_VENDOR_SLOT_SIZE - BL3_GPE_HEADER_OFFSET;

    *fap_out = &s_fap;
    return 0;
}

/* ---------------------------------------------------------------------------
 * flash_area_close — release the XMEM handle.
 * ---------------------------------------------------------------------------
 */
void flash_area_close(const struct flash_area *fap)
{
    (void)fap;
    if (s_xmemHandle != NULL) {
        (void)XMEMWFF3_close_nortos(s_xmemHandle);
        s_xmemHandle    = NULL;
        s_activeLogical = 0U;
    }
}

/* ---------------------------------------------------------------------------
 * flash_area_read — DMA/XIP read through OTFDE.
 *
 * CC35xx GPE image-format detail: BL2 writes the 4-byte ih_magic to flash
 * via STIG (raw, not OTFDE-encrypted), while the rest of the image_header
 * and body live in OTFDE-encrypted space.  At off==0 we therefore split the
 * read: STIG the first 4 bytes (the magic), then DMA the remaining bytes
 * through OTFDE.  bootutil_img_hash sees a contiguous, correct image_header
 * and the SHA matches what BL2's signing tool produced.
 *
 * Vendors with a different image format should override this function.
 * ---------------------------------------------------------------------------
 */
int flash_area_read(const struct flash_area *fap, uint32_t off,
                    void *dst, uint32_t len)
{
    if (s_xmemHandle == NULL) {
        return -1;
    }

    uint32_t xip_addr = fap->fa_off + off;
    size_t xmem_offset = (size_t)xip_addr - s_activeLogical;

    if (off == 0U && len >= sizeof(uint32_t)) {
        /* STIG-read the magic (first 4 bytes), then DMA the rest. */
        int_fast16_t ret = XMEMWFF3_read_nortos(s_xmemHandle, xmem_offset, dst,
                                         sizeof(uint32_t), XMEM_READ_STIG_NORTOS);
        if (ret != XMEM_STATUS_SUCCESS_NORTOS) {
            return -1;
        }
        if (len > sizeof(uint32_t)) {
            ret = XMEMWFF3_read_nortos(s_xmemHandle,
                                xmem_offset + sizeof(uint32_t),
                                (uint8_t *)dst + sizeof(uint32_t),
                                (size_t)(len - sizeof(uint32_t)), 0);
            if (ret != XMEM_STATUS_SUCCESS_NORTOS) {
                return -1;
            }
        }
        return 0;
    }

    /* Pure DMA path for all other reads. */
    int_fast16_t ret = XMEMWFF3_read_nortos(s_xmemHandle, xmem_offset, dst,
                                     (size_t)len, 0);
    return (ret == XMEM_STATUS_SUCCESS_NORTOS) ? 0 : -1;
}

/* ---------------------------------------------------------------------------
 * flash_area accessor helpers
 * ---------------------------------------------------------------------------
 */
uint32_t flash_area_get_off(const struct flash_area *fap)        { return fap->fa_off; }
uint32_t flash_area_get_size(const struct flash_area *fap)       { return fap->fa_size; }
uint8_t  flash_area_get_id(const struct flash_area *fap)         { return fap->fa_id; }
uint8_t  flash_area_get_device_id(const struct flash_area *fap)  { return fap->fa_device_id; }
uint16_t flash_area_get_align(const struct flash_area *fap)      { (void)fap; return 1; }

int flash_area_read_is_erased(const struct flash_area *fap, uint32_t off,
                               uint32_t len, uint8_t *out)
{
    (void)fap; (void)off; (void)len;
    *out = 0;
    return 0;
}

/* ---------------------------------------------------------------------------
 * bootutil_max_image_size — DIRECT_XIP has no swap trailer, the entire flash
 * area is available for the image.
 * ---------------------------------------------------------------------------
 */
uint32_t bootutil_max_image_size(struct boot_loader_state *state,
                                 const struct flash_area *fap)
{
    (void)state;
    return fap->fa_size;
}
