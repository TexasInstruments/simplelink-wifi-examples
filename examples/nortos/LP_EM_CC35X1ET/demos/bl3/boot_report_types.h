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
 *  ======== boot_report_types.h ========
 *
 *  Boot Report type definitions for BL3.
 *
 *  Field layout MUST match the BL2-internal bootReport_t exactly so that
 *  every offset BL3 reads matches the byte BL2 wrote.  Sub-types that BL3
 *  uses are typed properly (otaReport, extended_status); fields BL3 does
 *  not yet parse are kept as raw byte arrays with the correct sizes —
 *  add typedefs as needed when those fields are accessed.
 *
 *  Source of truth: bootloader/TI_Bootloader/mcuboot/ti/src/Services/
 *      boot_report.h  +  ota_update_infra.h
 *
 *  The Boot Report is written by BL2 at address 0x28000100 (M33 view)
 *  before releasing M33. BL3 reads it on startup and appends its own
 *  status before handoff.
 */

#ifndef BOOT_REPORT_TYPES_H_
#define BOOT_REPORT_TYPES_H_

#include <stdint.h>

#ifndef __packed
#define __packed __attribute__((__packed__))
#endif

/* ---------------------------------------------------------------------------
 * OTA report — per-image-type slot selection result, written by BL2.
 *
 * Each image type (BL2, WSOC, vendor app, vendor SBL) has its own 2-byte
 * report:
 *   reqReport          OTA request type + status
 *   imageSelectReport  Which slot was selected, plus error flags
 *
 * For vendor app authentication BL3 reads vendorAppImageReport.imageSelectReport
 * to determine the active physical slot (slot1Ok=1 → slot 0; slot2Ok=1 → slot 1).
 * ---------------------------------------------------------------------------
 */

typedef union OtaUpdate_ImageSelectReport_u
{
    struct __packed {
        uint8_t slot1Ok          : 1;  /* slot 1 passed validation */
        uint8_t slot2Ok          : 1;  /* slot 2 passed validation */
        uint8_t otaVerConflict   : 1;
        uint8_t alternateOta     : 1;
        uint8_t noValidMain      : 1;
        uint8_t otaDontCare      : 1;
        uint8_t bothSlotsInvalid : 1;
        uint8_t authError        : 1;
    };
    uint8_t byte;
} __packed OtaUpdate_ImageSelectReport_u;

typedef union OtaUpdate_ReqReport_u
{
    struct __packed {
        uint8_t reqType   : 2;
        uint8_t reqStatus : 6;
    };
    uint8_t byte;
} __packed OtaUpdate_ReqReport_u;

typedef struct otaUpdate_ImageReport_t
{
    OtaUpdate_ReqReport_u         reqReport;
    OtaUpdate_ImageSelectReport_u imageSelectReport;
} __packed otaUpdate_ImageReport_t;

typedef struct otaUpdate_OtaReport_t
{
    otaUpdate_ImageReport_t bl2ImageReport;        /* +0  BL2 self-report           */
    otaUpdate_ImageReport_t wsocImageReport;       /* +2  WSOC firmware             */
    otaUpdate_ImageReport_t vendorAppImageReport;  /* +4  vendor application slot   */
    otaUpdate_ImageReport_t sblImageReport;        /* +6  vendor SBL                */
} __packed otaUpdate_OtaReport_t;                  /* total: 8 bytes                */

/* ---------------------------------------------------------------------------
 * BL2 extended status — lifecycle state change + reserved bits.
 * ---------------------------------------------------------------------------
 */
typedef union bl3_bl2ExtendedStatus_u
{
    struct __packed {
        uint16_t lifeCycleStateChange : 2;
        uint16_t reserved             : 14;
    };
    uint8_t bytes[2];
} __packed bl3_bl2ExtendedStatus_u;

/* ---------------------------------------------------------------------------
 * Boot Report structure — fields written by BL2, read by BL3.
 *
 * Layout matches BL2's bootReport_t exactly so byte offsets line up.  Fields
 * BL3 does not use yet are kept as opaque byte arrays with BL2's sizes — type
 * them as needed when BL3 starts to parse them.
 * ---------------------------------------------------------------------------
 */
typedef struct bl3_bootReport_t
{
    otaUpdate_OtaReport_t      otaReport;                       /* OTA slot selection (BL2)        */
    uint8_t                    debugOpenStatus_legacy;          /* legacy field                    */
    uint8_t                    flashAndCalibInfo[12];           /* flash/PSRAM calibration (BL2)   */
    uint16_t                   reset_cause;                     /* reset cause bits                */
    uint32_t                   scratch_pads[2];                 /* PRCM scratch pads               */
    uint8_t                    fuse_value_reporting[136];       /* fuse reporting (BL2)            */
    uint8_t                    m33_fault_count_indication;      /* M33 fault counter               */
    uint8_t                    ti_bootloader_bl2_status;        /* 0 = BL2 success; non-zero = err */
    bl3_bl2ExtendedStatus_u    extended_status;                 /* lifecycle state + DSSM bits     */
    uint8_t                    ota_reporting[8];                /* OTA error indication            */
    uint8_t                    debug_open_status;               /* debug state                     */
    uint8_t                    general_vendor_image_info[1024]; /* 1 KB vendor image info          */
    uint8_t                    vendor_soc_configuration_status; /* SOC config result (BL2)         */
    uint8_t                    vendor_host_configuration_status;/* HOST config result (BL3 writes) */
    uint8_t                    configuration_errors_list[1024]; /* 1 KB error list                 */
    uint8_t                    mcu_boot[512];                   /* MCUBoot area                    */
    uint8_t                    bl3_report[512];                 /* BL3 status — written by BL3     */
} __packed bl3_bootReport_t;

/* ---------------------------------------------------------------------------
 * Status value constants
 * ---------------------------------------------------------------------------
 */

/* BL2 status values for ti_bootloader_bl2_status */
#define BL3_BL2_STATUS_OK             (0U)  /* BL2 completed successfully */

/* Active vendor slot indices returned by BL3_bootReport_getActiveVendorSlot().
 * BL2 reports 0 (= SysConfig vendor_image_slot_1) or 1 (= SysConfig _slot_2). */
#define BL3_VENDOR_SLOT_INVALID       ((uint8_t)0xFFU)
#define BL3_VENDOR_SLOT_1             ((uint8_t)0U)
#define BL3_VENDOR_SLOT_2             ((uint8_t)1U)

/* Convert internal slot value (0, 1) to user-facing display number (1, 2). */
#define BL3_SLOT_DISPLAY(s)           ((unsigned)(s) + 1U)

#endif /* BOOT_REPORT_TYPES_H_ */
