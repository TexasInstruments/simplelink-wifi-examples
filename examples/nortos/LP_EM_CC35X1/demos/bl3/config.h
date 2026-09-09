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
 *  ======== config.h ========
 *
 *  Central configuration for the BL3 example.
 *  All addresses, register offsets, and tunable parameters are defined here.
 *
 *  Target: CC35xx — supports all flash configurations (8 MB and 4 MB).
 *  Platform constraint: 50 KB maximum BL3 container size.
 */

#ifndef BL3_CONFIG_H_
#define BL3_CONFIG_H_

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Debug UART
 *
 * Uncomment to enable BL3 debug output on UART1 (XDS110 back-channel, 115200 8N1).
 *
 * ---------------------------------------------------------------------------
 */
#define BL3_DEBUG_ENABLE

/* ---------------------------------------------------------------------------
 * GPIO loopback test
 *
 * Requires BL3_DEBUG_ENABLE. Drives GPIO3 (BP.26) HIGH and reads back on
 * GPIO4 (BP.25). Connect a jumper between BP.25 and BP.26 before enabling.
 * Off by default — without the jumper the test prints FAIL.
 * ---------------------------------------------------------------------------
 */
#define BL3_GPIO_LOOPBACK_ENABLE

/* ---------------------------------------------------------------------------
 * Vendor image flash layout
 *
 * BL2 selects the active vendor image slot (OTA primary or secondary) and
 * places its flash base address in the Boot Report. The constant below is the
 * default for the primary slot on the 8 MB flash platform.
 * ---------------------------------------------------------------------------
 */

/*
 * Slot offset of the GPE manifest's ih_magic — written by BL2 via STIG.
 * flash_area_open() sets fa_off = slot_logical_base + this offset.
 */
#define BL3_GPE_HEADER_OFFSET           (0xFFCU)

/*
 * XIP address of the vendor application ARM vector table.
 * Derived as: slot XIP base (0x14000000)
 *           + GPE magic offset  (0xFFC)
 *           + MCUBoot header    (0x020)
 *           + manifest padding  (0xFE4)
 *           = 0x14002000
 */
#define BL3_VENDOR_APP_BASE             (0x14002000U)

/*
 * Size of the vendor image slot in flash.
 * 1 MB is the default allocation for the primary slot on the 8 MB platform.
 * Adjust for your platform's partition table.
 */
#define BL3_VENDOR_SLOT_SIZE            (0x100000U)

/* ---------------------------------------------------------------------------
 * Boot Report
 *
 * BL2 writes the Boot Report to BOOT_REPORT_SHARED_MEM before releasing M33.
 * BL3 reads it on startup and appends its own status before handoff.
 * The region is marked NOLOAD in the BL3 linker script — BL3 does not own it.
 * ---------------------------------------------------------------------------
 */

/* M33 virtual address of the BL2 Boot Report (BOOT_REPORT_SHARED_MEM base) */
#define BL3_BOOT_REPORT_ADDR            (0x28000100U)

/* ---------------------------------------------------------------------------
 * Security registers
 *
 * HOST_BOOT_DONE (SECGP_HOST_BOOT_DONE, offset 0xb0) is intentionally left
 * unset by BL2 when BL3 is present. BL3 asserts it after completing security
 * configuration and locks. Setting this register locks those configurations
 * and signals the hardware that M33 boot is done.
 * ---------------------------------------------------------------------------
 */

/* SOC_AON_REGS base address (SOC_AON_BASE = 0x41100000) */
#define BL3_SECGP_BASE_ADDR             (0x41100000U)

/* Offset of the 32-byte vendor public key SHA-256 hash in the Boot Report.
 * BL2 writes the ROT pubkey hash (read from eFUSE rows 5-12) at this offset.
 * Absolute address: BL3_BOOT_REPORT_ADDR + offset = 0x2800011f */
#define BL3_BOOT_REPORT_PUBKEY_HASH_OFFSET  (0x1fU)
#define BL3_BOOT_REPORT_PUBKEY_HASH_SIZE    (32U)

/* Offset within SOC_AON_REGS for the HOST_BOOT_DONE register */
#define BL3_SECGP_HOST_BOOT_DONE_OFFSET (0xb0U)

#endif /* BL3_CONFIG_H_ */
