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
#include "cmd_parser.h"
#include "host/ble_hs.h"

/******************************************************************************
                      Macros
******************************************************************************/

#define BLE_DEVICE_LOCAL_NAME_LEN  MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME_MAX_LENGTH)

#define BLE_COPY_BD_ADDRESS( dstBd, srcBd )                                  \
  (dstBd)[0] = (srcBd)[5];                                                   \
  (dstBd)[1] = (srcBd)[4];                                                   \
  (dstBd)[2] = (srcBd)[3];                                                   \
  (dstBd)[3] = (srcBd)[2];                                                   \
  (dstBd)[4] = (srcBd)[1];                                                   \
  (dstBd)[5] = (srcBd)[0];


/******************************************************************************
                      Structs and Enums
******************************************************************************/

typedef enum 
{
    BLE_ADV_TYPE_IND                 = 0,
    BLE_ADV_TYPE_DIRECT_IND_HIGH     = 1,
    BLE_ADV_TYPE_SCAN_IND            = 2,
    BLE_ADV_TYPE_NONCONN_IND         = 3,
    BLE_ADV_TYPE_DIRECT_IND_LOW      = 4,
    BLE_ADV_TYPE_EXT_NOSCANNABLE_IND = 5,
    BLE_ADV_TYPE_EXT_CONNECTABLE_IND = 6,
    BLE_ADV_TYPE_EXT_SCANNABLE_IND   = 7,
    BLE_ADV_TYPE_MAX
} AdvType_e;

typedef struct
{
    ble_addr_t addr;
    int8_t rssi;
    uint8_t prim_phy;
    char local_name[BLE_DEVICE_LOCAL_NAME_LEN];
} scanResEntry_t;

typedef struct
{
    uint8_t instance;
    /* Minimum advertising interval in 0.625ms units, if 0 default is used */
    uint32_t intervalMin;
    /* Maximum advertising interval in 0.625ms units, if 0 default is used */
    uint32_t intervalMax;
    /* Advertise type:
     * 
     * 0: IND
     * 1: DIRECT IND HIGH
     * 2: SCAN IND
     * 3: NONCONN IND
     * 4: DIRECT IND LOW
     * 5: EXT NONSCANNABLE IND
     * 6: EXT CONNECTABLE IND
     * 7: EXT SCANNABLE IND
     */
    AdvType_e advType;
    /* Own address type to be used by advertising instance */
    uint8_t ownAddrType;
    /* Advertising channel map, if 0 default is used */
    uint8_t channelMap;
    /* Advertising Filter policy */
    uint8_t filterPolicy;
    /* Peer address for directed advertising, valid only if directed is set */
    ble_addr_t peerAddr;
    /* Primary advertising PHY to use */
    uint8_t primPhy;
    /* Secondary advertising PHY to use */
    uint8_t secPhy;
} AtCmdExtAdvCfg_t;

typedef struct {
    uint16_t scan_interval;
    uint16_t scan_window;
    uint8_t own_address_type;
    uint8_t filter_policy;
    uint8_t scan_type;
    uint8_t scan_phy;
} AtCmdExtScanCfg_t;

/******************************************************************************
                      API Calls
******************************************************************************/
int nimble_host_start(void);

int nimble_host_stop(void);

int nimble_host_is_enabled(void);

int nimble_host_ext_adv_cfg(AtCmdExtAdvCfg_t *pAdvCfg);

void nimble_host_get_ext_adv_cfg(AtCmdExtAdvCfg_t *pAdvCfg);

int nimble_host_ext_adv_enable(ExtAdvEnable_t *pAdvEnb);

int nimble_host_ext_scan_cfg(AtCmdExtScanCfg_t *pScanCfg);

int nimble_host_ext_scan_cfg_get(AtCmdExtScanCfg_t *pScanCfg);

int nimble_host_ext_scan_enable(ExtScanEnable_t *pScanEnb);

int nimble_host_ext_connect(uint8_t* bd_addr, uint8_t addr_type);

int nimble_host_ext_connect_params(uint8_t* bd_addr, uint8_t addr_type, uint8 phy_mask, uint32 interval_min_us, uint32 interval_max_us, uint32 supervision_timeout_us, int32_t duration_ms);

int nimble_host_gap_update_params(uint16 connHandle, uint32 interval_min_us, uint32 interval_max_us, uint32 supervision_timeout_us, uint32 latency, uint16 min_ce_len, uint16 max_cle_len);

int nimble_host_ext_disconnect(uint8_t* bd_addr, uint8_t addr_type);

int nimble_host_connected_peers(uint8_t *peerAddress, uint8_t *peerAddressType);

int nimble_host_delete_all_bond_info(void);

int nimble_host_set_mac_addr(uint8_t addrType);

int nimble_host_get_mac_addr(ble_addr_t *addr);

void register_extra_gap_event_cb(void (*cb)(void *, void *));
