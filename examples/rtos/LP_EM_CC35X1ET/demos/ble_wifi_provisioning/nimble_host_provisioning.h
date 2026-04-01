/*
 * Copyright (c) 2024, Texas Instruments Incorporated
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
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
                      API Calls
******************************************************************************/

typedef struct ExtAdvCfg_t
{
    uint8 instance;
    uint8 legacy;
    uint32 interval_ms;
    uint8 prim_phy;
    uint8 sec_phy;
}ExtAdvCfg_t;

typedef struct ExtAdvEnable_t
{
    uint8 enable;
    uint8 instance;
    int duration;
    int max_events;
}ExtAdvEnable_t;

typedef struct ExtScanCfg_t
{
    uint16 scan_interval_ms;
    uint16 scan_window_ms;
    uint8 own_address_type;
    uint8 filter_policy;
    uint8 scan_type;
    uint8 scan_phy;

}ExtScanCfg_t;

typedef struct ExtScanEnable_t
{
    uint16 duration;
    uint16 period;
    uint8 enable;
    uint8 filter_duplicate;

}ExtScanEnable_t;


int nimble_host_start(void);

int nimble_host_stop(void);

int nimble_host_is_enabled(void);

int nimble_host_ext_adv_cfg(ExtAdvCfg_t *pAdvCfg);

int nimble_host_ext_adv_enable(ExtAdvEnable_t *pAdvEnb);

int nimble_host_ext_scan_cfg(ExtScanCfg_t *pScanCfg);

int nimble_host_ext_scan_enable(ExtScanEnable_t *pScanEnb);

int nimble_host_ext_connect(uint8_t* bd_addr, uint8_t addr_type);

int nimble_host_ext_connect_params(uint8_t* bd_addr, uint8_t addr_type, uint8 phy_mask, uint32 interval_min_us, uint32 interval_max_us, uint32 supervision_timeout_us);

int nimble_host_gap_update_params(uint16 connHandle, uint32 interval_min_us, uint32 interval_max_us, uint32 supervision_timeout_us, uint32 latency, uint16 min_ce_len, uint16 max_cle_len);

int nimble_host_ext_disconnect(uint8_t* bd_addr, uint8_t addr_type);

int nimble_host_connected_peers(void);

int nimble_host_delete_all_bond_info(void);

int nimble_host_get_bd_address(uint8_t addr_type);

int nimble_host_set_bd_address(uint8_t addr_type);

/* application specific APIs */
int nimble_host_gatt_svr_chr_notify_wlan_connection(uint8_t status);

