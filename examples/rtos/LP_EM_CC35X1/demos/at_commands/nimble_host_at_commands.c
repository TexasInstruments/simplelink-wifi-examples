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
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "uart_term.h"
#include "console/console.h"
#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/dis/ble_svc_dis.h"
#include "../host/src/ble_hs_priv.h"

#include "ble_if.h"
#include "nimble_host_at_commands.h"

#include "atcmd_defs.h"
#include "atcmd_event.h"
#include "atcmd_gen.h"

/******************************************************************************
                      Defines
******************************************************************************/
#ifdef CC33XX
#define NIMBLE_THRD_PRIORITY       (3)
#else
#define NIMBLE_THRD_PRIORITY       (8)
#endif

#define NIMBLE_THRD_DEFAULT_STACK  NULL
#define NIMBLE_THRD_STACK_SIZE     (3072)

#define BLE_MGF_DATA_LEN           (3)
#define BLE_MAX_SCAN_RESULTS       (8)
#define BLE_MAX_SUPPORTED_CONNS    MYNEWT_VAL(BLE_MAX_CONNECTIONS)
#define BLE_DEVICE_LOCAL_NAME      MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME)
#define BLE_STORE_CONFIG_PERSIST   MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST)

#define BLE_DEFAULT_PASSKEY        123456

/*******************************************************************************
                      Macros
******************************************************************************/


/******************************************************************************
                      Typedef
******************************************************************************/

typedef struct
{
    AtCmdExtScanCfg_t   extScanParams;
    scanResEntry_t      extScanResults[BLE_MAX_SCAN_RESULTS];
    uint8               extScanResultsCount;
    uint8               extScanResultsTotal;
}extScanCB_t;

typedef struct
{
    uint16 connHandles[BLE_MAX_SUPPORTED_CONNS];
    uint8  connCount;
}extConnCB_t;

/******************************************************************************
                      Globals
******************************************************************************/
static struct ble_npl_task s_task_host;
static const char gap_name[] = BLE_DEVICE_LOCAL_NAME;
static const char mgf_data[BLE_MGF_DATA_LEN] = {0x0D, 0x00, 0xFF};
static uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint8_t empty_addr[BLE_DEV_ADDR_LEN] = {0};
static extScanCB_t extScanCB;
static extConnCB_t extConnCB;
static AtCmdExtAdvCfg_t extAdvCfg;
static void (*extra_gap_event_cb)(void *, void *) = NULL;
static OsiSyncObj_t hostInitEventSyncObj;

/******************************************************************************
                      Function Prototypes
******************************************************************************/
void nimble_host_task(void *param);
#if BLE_STORE_CONFIG_PERSIST
void ble_store_config_init(void);
#else
void ble_store_ram_init(void);
#endif

/******************************************************************************
                      Function Implementations
******************************************************************************/
static void ble_example_init()
{
    // Reset scan results table data
    os_memset(extScanCB.extScanResults,0,(BLE_MAX_SCAN_RESULTS * sizeof(scanResEntry_t)));
    extScanCB.extScanResultsCount = 0;
    extScanCB.extScanResultsTotal = 0;

    // Reset scan results table data
    os_memset(extConnCB.connHandles, BLE_HS_CONN_HANDLE_NONE, sizeof(extConnCB.connHandles));
    extConnCB.connCount = 0;

    // Reset extended advertise configurations
    os_memset(&extAdvCfg, 0x0, sizeof(extAdvCfg));

    // Set IO Capabilities
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;

    // Create Sync Object for host init
    if (osi_SyncObjCreate(&hostInitEventSyncObj) != OSI_OK)
    {
        Report("\n\rERROR: Failed to create sync object for BLE host init");
        ASSERT_GENERAL(0);
    }
}

static void print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    console_printf("%02X:%02X:%02X:%02X:%02X:%02X",
                   u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

static void add_to_connected_peers(uint16 conn_handle)
{
    uint8 i;

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        for (i=0; i<BLE_MAX_SUPPORTED_CONNS; i++)
        {
            if (extConnCB.connHandles[i] == BLE_HS_CONN_HANDLE_NONE)
            {
                extConnCB.connHandles[i] = conn_handle;
                extConnCB.connCount++;
                break;
            }
        }
    }
}

static void remove_from_connected_peers(uint16 conn_handle)
{
    uint8 i;

    if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        for (i=0; i<BLE_MAX_SUPPORTED_CONNS; i++)
        {
            if (extConnCB.connHandles[i] == conn_handle)
            {
                extConnCB.connHandles[i] = BLE_HS_CONN_HANDLE_NONE;
                extConnCB.connCount--;
                break;
            }
        }
    }
}

/******************************************************************************
                      GATT SERVER
******************************************************************************/
#define BLE_SVC_TI_PERIPHERAL_UUID16        0XFFF0
#define BLE_SVC_TI_PERIPHERAL_CHR_UUID16_1  0XFFF1
#define BLE_SVC_TI_PERIPHERAL_CHR_UUID16_2  0XFFF2
#define BLE_SVC_TI_PERIPHERAL_CHR_UUID16_3  0XFFF3
#define BLE_SVC_TI_PERIPHERAL_CHR_UUID16_4  0XFFF4
#define BLE_SVC_TI_PERIPHERAL_CHR_UUID16_5  0XFFF5

uint16_t ti_peripheral_handle = 0;

/**
 * Structure holding data for the main characteristics
 */
struct ble_svc_ti_peripheral_data {
    uint8_t char1;
    uint8_t char2;
    uint8_t char3;
    uint8_t char4;
    uint8_t char5[5];
};

struct ble_svc_ti_peripheral_data ble_svc_ti_peripheral_data = {
    .char1 = 0x01,
    .char2 = 0x02,
    .char3 = 0x03,
    .char4 = 0x04,
    .char5 = {0x01,0x02,0x03,0x04,0x05},
};

static int gatt_svr_chr_access_ti_peripheral(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: TI Simple Peripheral */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_TI_PERIPHERAL_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /* Characteristic: READ/WRITE */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_TI_PERIPHERAL_CHR_UUID16_1),
            .access_cb = gatt_svr_chr_access_ti_peripheral,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: READ */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_TI_PERIPHERAL_CHR_UUID16_2),
            .access_cb = gatt_svr_chr_access_ti_peripheral,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            /* Characteristic: WRITE */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_TI_PERIPHERAL_CHR_UUID16_3),
            .access_cb = gatt_svr_chr_access_ti_peripheral,
            .flags = BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: NOTIFY */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_TI_PERIPHERAL_CHR_UUID16_4),
            .access_cb = gatt_svr_chr_access_ti_peripheral,
            .val_handle = &ti_peripheral_handle,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            /* Characteristic: READ */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_TI_PERIPHERAL_CHR_UUID16_5),
            .access_cb = gatt_svr_chr_access_ti_peripheral,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            0, /* No more characteristics in this service */
        }, }
    },
    {
        0, /* No more services */
    },
};

static int gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int gatt_svr_chr_notify(uint16_t conn_handle, uint8_t value)
{
    int rc;
    struct os_mbuf *om;
    uint8_t notify_value[1];

    notify_value[0] = value;

    om = ble_hs_mbuf_from_flat(&notify_value, sizeof(notify_value));

    rc = ble_gatts_notify_custom(conn_handle, ti_peripheral_handle, om);

    return rc;
}

static int gatt_svr_chr_access_ti_peripheral(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16;
    int rc;

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    ASSERT_GENERAL(uuid16 != 0);

    switch(uuid16) {
        case BLE_SVC_TI_PERIPHERAL_CHR_UUID16_1:
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, &ble_svc_ti_peripheral_data.char1,
                                    sizeof(ble_svc_ti_peripheral_data.char1));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                rc = gatt_svr_chr_write(ctxt->om, 0,
                                        sizeof(ble_svc_ti_peripheral_data.char1),
                                        &ble_svc_ti_peripheral_data.char1, NULL);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;

        case BLE_SVC_TI_PERIPHERAL_CHR_UUID16_2:
            ASSERT_GENERAL(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            rc = os_mbuf_append(ctxt->om, &ble_svc_ti_peripheral_data.char2,
                                sizeof(ble_svc_ti_peripheral_data.char2));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_SVC_TI_PERIPHERAL_CHR_UUID16_3:
            ASSERT_GENERAL(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);
            rc = gatt_svr_chr_write(ctxt->om, 0,
                                    sizeof(ble_svc_ti_peripheral_data.char3),
                                    &ble_svc_ti_peripheral_data.char3, NULL);
            if (rc == 0)
            {
                rc = gatt_svr_chr_notify(conn_handle, ble_svc_ti_peripheral_data.char3);
            }
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_SVC_TI_PERIPHERAL_CHR_UUID16_4:
            rc = os_mbuf_append(ctxt->om, &ble_svc_ti_peripheral_data.char4,
                                sizeof(ble_svc_ti_peripheral_data.char4));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_SVC_TI_PERIPHERAL_CHR_UUID16_5:
            ASSERT_GENERAL(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            rc = os_mbuf_append(ctxt->om, &ble_svc_ti_peripheral_data.char5,
                                sizeof(ble_svc_ti_peripheral_data.char5));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        default:
            console_printf("\n\rERROR: unexpected UUID 0x%x\n\r", uuid16);
            return BLE_ATT_ERR_UNLIKELY;
    }

    /* Unknown characteristic; the nimble stack should not have called this
     * function.
     */
    console_printf("\n\rERROR: Unknown characteristic\n\r");
    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_init(void)
{
    int rc;

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

/******************************************************************************
                      GAP EVENT CALLBACK
******************************************************************************/
static int gap_event_passkey_action(struct ble_gap_event *passkey_event)
{
    uint16_t conn_handle;
    uint8_t action;
    uint32_t numcmp;
    struct ble_sm_io pk;

    if (passkey_event == NULL)
    {
       return (-1);
    }

    // Extract parameters
    conn_handle = passkey_event->passkey.conn_handle;
    action = passkey_event->passkey.params.action;
    numcmp = passkey_event->passkey.params.numcmp;

    switch (action)
    {
        case BLE_SM_IOACT_NONE:
            break;
        case BLE_SM_IOACT_OOB:
            break;
        case BLE_SM_IOACT_INPUT:
            pk.action = BLE_SM_IOACT_INPUT;
            pk.passkey = BLE_DEFAULT_PASSKEY;
            ble_sm_inject_io(conn_handle, &pk);
            break;
        case BLE_SM_IOACT_DISP:
            pk.action = BLE_SM_IOACT_DISP;
            pk.passkey = BLE_DEFAULT_PASSKEY;
            Report("\n\rBLE_SM_IOACT_DISP passkey=[%d] \n\r", pk.passkey);
            ble_sm_inject_io(conn_handle, &pk);
            break;
        case BLE_SM_IOACT_NUMCMP:
            pk.action = BLE_SM_IOACT_NUMCMP;
            pk.numcmp_accept = 1; //always confirm
            Report("\n\rBLE_SM_IOACT_NUMCMP passkey=[%d] \n\r", numcmp);
            ble_sm_inject_io(conn_handle, &pk);
            break;
        case BLE_SM_IOACT_OOB_SC:
            break;
        default:
            ASSERT_GENERAL(0);
    }

    return (0);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    int rc = 0;
    struct ble_hs_adv_fields adv_fields;
    char no_local_name[7] = "Unnamed";
    ATCmd_BleEvent_t ble_event;
    
    os_memset(&ble_event, 0x0, sizeof(ble_event));

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
        {
            ATCmd_BleConnectEvent_t *ble_connect_event = NULL;
            struct ble_gap_conn_desc conn_desc;

            ble_connect_event = os_malloc(sizeof(ATCmd_BleConnectEvent_t));
            if (ble_connect_event == NULL)
            {
                ATCmd_errorResult(ATCmd_errorAllocStr, 0);
                return 0;
            }
            os_memset(ble_connect_event, 0x0, sizeof(ATCmd_BleConnectEvent_t));

            ble_connect_event->status = event->connect.status;

            if (event->connect.status == 0)
            {
                ble_connect_event->connectionHandle = event->connect.conn_handle;

                rc = ble_gap_conn_find(event->connect.conn_handle, &conn_desc);
                if (rc != 0)
                {
                    os_free(ble_connect_event);
                    ATCmd_errorResult(ATCmd_errorCmdStr, -1);
                    return 0;
                }

                BLE_COPY_BD_ADDRESS(ble_connect_event->peerAddress,
                                    conn_desc.peer_ota_addr.val);
                ble_connect_event->peerAddressType = conn_desc.peer_ota_addr.type;
                add_to_connected_peers(event->connect.conn_handle);
            }

            ble_event.eventId = ATCMD_BLE_EVENT_CONNECT;
            ble_event.eventArgs = ble_connect_event;
            ATCmdEvent_compose(&ble_event, sizeof(ATCmd_BleEvent_t), ATCmdEvent_bleCallback, 0);
        }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
        {
            ATCmd_BleDisconnectEvent_t *ble_disconnect_event = NULL;

            remove_from_connected_peers(event->disconnect.conn.conn_handle);

            ble_disconnect_event = os_malloc(sizeof(ATCmd_BleDisconnectEvent_t));
            if (ble_disconnect_event == NULL)
            {
                ATCmd_errorResult(ATCmd_errorAllocStr, 0);
                return 0;
            }
            os_memset(ble_disconnect_event, 0x0, sizeof(ATCmd_BleDisconnectEvent_t));

            BLE_COPY_BD_ADDRESS(ble_disconnect_event->peerAddress,
                                event->disconnect.conn.peer_ota_addr.val);
            ble_disconnect_event->peerAddressType = event->disconnect.conn.peer_ota_addr.type;

            ble_event.eventId = ATCMD_BLE_EVENT_DISCONNECT;
            ble_event.eventArgs = ble_disconnect_event;
            ATCmdEvent_compose(&ble_event, sizeof(ATCmd_BleEvent_t), ATCmdEvent_bleCallback, 0);
        }
            break;

        case BLE_GAP_EVENT_CONN_UPDATE:
            break;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ble_event.eventId = ATCMD_BLE_EVENT_SCAN_COMPLETE;
            ble_event.eventArgs = NULL;
            ATCmdEvent_compose(&ble_event, sizeof(ATCmd_BleEvent_t), ATCmdEvent_bleCallback, 0);
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_event.eventId = ATCMD_BLE_EVENT_ADV_COMPLETE;
            ble_event.eventArgs = NULL;

            ATCmdEvent_compose(&ble_event, sizeof(ATCmd_BleEvent_t),
                               ATCmdEvent_bleCallback,
                               event->adv_complete.reason);
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            console_printf("\n\rEncryption change event with status=%x\n\r",
                        event->enc_change.status);
            break;

        case BLE_GAP_EVENT_PASSKEY_ACTION:
            gap_event_passkey_action(event);
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            break;

        case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
            // console_printf("\n\rPHY update complete; status=%d, conn_handle=%d, tx_phy=%d, rx_phy=%d\n\r",
            //                event->phy_updated.status,
            //                event->phy_updated.conn_handle,
            //                event->phy_updated.tx_phy,
            //                event->phy_updated.rx_phy);
            break;

        case BLE_GAP_EVENT_EXT_DISC:
        {
            scanResEntry_t *scan_res_entry = NULL;

            rc = ble_hs_adv_parse_fields(&adv_fields, event->ext_disc.data, event->ext_disc.length_data);
            if (rc != 0)
            {
                console_printf("Discard adv report with invalid data fields. error code: %d\n\r",rc);
                return 0;
            }

            scan_res_entry = os_malloc(sizeof(scanResEntry_t));
            if (scan_res_entry == NULL)
            {
                ATCmd_errorResult(ATCmd_errorAllocStr, 0);
                return 0;
            }
            os_memset(scan_res_entry, 0x0, sizeof(scanResEntry_t));

            BLE_COPY_BD_ADDRESS(scan_res_entry->addr.val, event->ext_disc.addr.val)
            scan_res_entry->addr.type = event->ext_disc.addr.type;
            scan_res_entry->prim_phy = event->ext_disc.prim_phy;
            scan_res_entry->rssi = event->ext_disc.rssi;
            os_memcpy(scan_res_entry->local_name, no_local_name, 7);

            if (adv_fields.name != NULL) {
                if (adv_fields.name_len < (BLE_DEVICE_LOCAL_NAME_LEN-1)) {
                    os_memcpy(scan_res_entry->local_name, adv_fields.name, adv_fields.name_len);
                    scan_res_entry->local_name[adv_fields.name_len] = '\0';
                }
            }

            ble_event.eventId = ATCMD_BLE_EVENT_SCAN_RESULT;
            ble_event.eventArgs = scan_res_entry;
            ATCmdEvent_compose(&ble_event, sizeof(ATCmd_BleEvent_t), ATCmdEvent_bleCallback, 0);

            break;
        }
        case BLE_GAP_EVENT_PAIRING_COMPLETE:
            console_printf("\n\rPairing/Encryption %s with status=%d",
                        event->pairing_complete.status == 0 ? "completed successfully\n\r" : "failed\n\r",
                        event->pairing_complete.status);
            break;

        default:
            break;
    }

    if (extra_gap_event_cb)
    {
        extra_gap_event_cb(event, arg);
    }

    return 0;
}

/******************************************************************************
                      NIMBLE HOST CB
******************************************************************************/
static void ble_reset_cb(int reason)
{
    console_printf("\n\rRESET due to 0x%x\n\r",reason);
    ASSERT_GENERAL(0);
}

static void ble_sync_cb(void)
{
    int rc;
    ble_addr_t addr;
    const char *name;

    rc = ble_hs_util_ensure_addr(0);

    if ( rc != 0 )
    {
        console_printf("\n\rError: failed with ensure BLE address\n\r");
    }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);

    if ( rc != 0 )
    {
        console_printf("\n\rError: failed with inferred BLE address type\n\r");
    }

    if (nimble_host_is_enabled())
    {
        console_printf("\n\rHost and controller become synced");
        addr.type = own_addr_type;
        rc = ble_hs_id_copy_addr(addr.type, addr.val, NULL);
        if ( rc != 0 )
        {
            console_printf("\n\rError: failed to retrieve one of the device's identity addresses\n\r");
        }
        else
        {
            console_printf("\n\rBD address: ");
            print_addr(addr.val);
        }
        name = ble_svc_gap_device_name();
        console_printf("\n\rName: %s \n\r", name);
    }
    else
    {
        console_printf("\n\rHost and controller failed to sync\n\r");
    }

    osi_SyncObjSignal(&hostInitEventSyncObj);
}

static void ble_stop_cb(int status, void *arg)
{
    if (status == 0) {
        console_printf("\n\rBLE stopped");
    } else {
        console_printf("\n\rBLE already stopped");
    }
}

void nimble_host_task(void *param)
{
    ble_hs_cfg.reset_cb = ble_reset_cb;
    ble_hs_cfg.sync_cb = ble_sync_cb;

    ble_svc_gap_device_name_set(gap_name);

    nimble_port_run();
}

/******************************************************************************
                      API Calls
******************************************************************************/
int nimble_host_ext_adv_cfg(AtCmdExtAdvCfg_t *pAdvCfg)
{
    struct ble_gap_ext_adv_params params = {0};
    struct ble_hs_adv_fields adv_fields;
    struct os_mbuf *adv_data;
    ble_addr_t addr;
    const char *device_name = ble_svc_gap_device_name();
    int rc = 0;

    if ( !pAdvCfg )
    {
        console_printf("\n\rError: failed to configure advertisement parameters\n\r");
        return (-1);
    }

    /* Verify the adv instance */
    if (pAdvCfg->instance >= BLE_ADV_INSTANCES) {
        console_printf("\n\rERROR: invalid instance\n\r");
        return (-1);
    }

    /* Init the params */
    os_memset(&params, 0, sizeof(params));

    /* Defaults */
    params.include_tx_power = 0;
    params.anonymous = 0;
    params.scan_req_notif = 0;
    params.tx_power = 127;
    params.sid = 0;

    /* User set params */
    params.itvl_min = pAdvCfg->intervalMin;
    params.itvl_max = pAdvCfg->intervalMax;
    params.own_addr_type = pAdvCfg->ownAddrType;
    params.channel_map = pAdvCfg->channelMap;
    params.filter_policy = pAdvCfg->filterPolicy;
    params.peer.type = pAdvCfg->peerAddr.type;
    os_memcpy(params.peer.val, pAdvCfg->peerAddr.val, BLE_DEV_ADDR_LEN);
    params.primary_phy = pAdvCfg->primPhy;
    params.secondary_phy = pAdvCfg->secPhy;

    /* Match input advertise type to legacy_pdu, connectable
     * scannable, directed and high_duty_directed flags.
     */
    switch (pAdvCfg->advType)
    {
        case BLE_ADV_TYPE_IND:
            params.legacy_pdu = 1;
            params.connectable = 1;
            params.scannable = 1;
            break;

        case BLE_ADV_TYPE_DIRECT_IND_HIGH:
            params.legacy_pdu = 1;
            params.connectable = 1;
            params.directed = 1;
            params.high_duty_directed = 1;
            break;

        case BLE_ADV_TYPE_SCAN_IND:
            params.legacy_pdu = 1;
            params.scannable = 1;
            break;

        case BLE_ADV_TYPE_NONCONN_IND:
            params.legacy_pdu = 1;
            break;

        case BLE_ADV_TYPE_DIRECT_IND_LOW:
            params.legacy_pdu = 1;
            params.directed = 1;
            params.connectable = 1;
            break;

        case BLE_ADV_TYPE_EXT_NOSCANNABLE_IND:
            /* All flags set to 0 */
            break;

        case BLE_ADV_TYPE_EXT_CONNECTABLE_IND:
            params.connectable = 1;
            break;

        case BLE_ADV_TYPE_EXT_SCANNABLE_IND:
            params.scannable = 1;
            break;

        default:
            return -1;
    }

    /* Remove existing adv set with same instance in case it exists */
    rc = ble_gap_ext_adv_remove(pAdvCfg->instance);
    if ( (rc != 0) && (rc != BLE_HS_EALREADY) )
    {
        console_printf("\n\rERROR: failed to remove existing adv set with error code: %d\n\r",rc);
        return (rc);
    }

    /* Configure the adv set */
    rc = ble_gap_ext_adv_configure(pAdvCfg->instance, &params, NULL,
                                   gap_event_cb, NULL);
    if ( rc != 0 ) {
        console_printf("\n\rERROR: failed to configure advertising instance with error code: %d\n\r",rc);
        return (rc);
    }

    /* Set adv Address */
    if (own_addr_type == BLE_OWN_ADDR_PUBLIC) {
        addr.type = BLE_ADDR_PUBLIC;
        rc = ble_hs_id_copy_addr(addr.type, addr.val, NULL);
        if ( rc != 0 )
        {
            console_printf("\n\rERROR: fail to retrieves device's PUBLIC identity addresses, with error code: %d\n\r",rc);
            return (rc);
        }

    } else {
        addr.type = BLE_ADDR_RANDOM;
        rc = ble_hs_id_copy_addr(addr.type, addr.val, NULL);
        if ( rc != 0 )
        {
            console_printf("\n\rERROR: fail to retrieves device's RANDOM identity addresses, with error code: %d\n\r",rc);
            return (rc);
        }

        /* Set random address for advertising instance */
        rc = ble_gap_ext_adv_set_addr(pAdvCfg->instance, &addr);
        if ( rc != 0 )
        {
            console_printf("\n\rERROR: fail to set random address for configured advertising instance, with error code: %d\n\r",rc);
            return (rc);
        }
    }

    /* Set ADV fields */
    memset(&adv_fields, 0, sizeof(adv_fields));
    /* General Discoverable and BrEdrNotSupported */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.tx_pwr_lvl_is_present = 0;
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    /* Set device name as instance name */
    adv_fields.name = (uint8_t *) device_name;
    adv_fields.name_len = strlen(device_name);
    adv_fields.name_is_complete = 1;
    /* Manufacturer specific data */
    adv_fields.mfg_data = (uint8_t*)mgf_data;
    adv_fields.mfg_data_len = BLE_MGF_DATA_LEN;

    /* Get Packet Header */
    /* Default to legacy PDUs size, mbuf chain will be increased if needed */
    adv_data = os_msys_get_pkthdr(BLE_HCI_MAX_ADV_DATA_LEN, 0);
    if ( adv_data == NULL )
    {
        console_printf("\n\rERROR: failed to allocate memory for adv data\n\r");
        return (-1);
    }

    /* Set ADV fields */
    rc = ble_hs_adv_set_fields_mbuf(&adv_fields, adv_data);
    if ( rc != 0 )
    {
        console_printf("\n\rERROR: failed to set adv set fields, with error code: %d\n\r", rc);
        os_mbuf_free_chain(adv_data);
        return (rc);
    }

    /* Set ADV Data */
    // rc = ble_gap_ext_adv_set_data(pAdvCfg->instance, adv_data);
    // if ( rc != 0 )
    // {
    //     console_printf("\n\rERROR: failed to configure the data to include in advertisements packets, with error code: %d\n\r", rc);
    //     return (rc);
    // }

    console_printf("\n\rInstance %u configured successfully with device name: '%s', BD address: ",
                   pAdvCfg->instance, device_name);
    print_addr(addr.val);
    console_printf("\n\r");

    /* Store the adv config if operation was successful */
    os_memcpy(&extAdvCfg, pAdvCfg, sizeof(AtCmdExtAdvCfg_t));

    return 0;
}

void nimble_host_get_ext_adv_cfg(AtCmdExtAdvCfg_t *pAdvCfg)
{
    os_memcpy(pAdvCfg, &extAdvCfg, sizeof(AtCmdExtAdvCfg_t));
}

int nimble_host_ext_adv_enable(ExtAdvEnable_t *pAdvEnb)
{
    int rc = 0;

    // Sanity Check
    if ( !pAdvEnb )
    {
        return -1;
    }

    // Validate the instance value
    if (pAdvEnb->instance >= BLE_ADV_INSTANCES) {
        console_printf("\n\rERROR: invalid instance\n\r");
        return (-1);
    }

    // Check the enable parameter
    if (pAdvEnb->enable)
    {
        // Duration must be limited for high duty cycle advertise
        if (extAdvCfg.advType == BLE_ADV_TYPE_DIRECT_IND_HIGH)
        {
            pAdvEnb->duration = 128;
        }

        rc = ble_gap_ext_adv_start(pAdvEnb->instance, pAdvEnb->duration, pAdvEnb->max_events);
        if ( rc != 0 )
        {
            console_printf("\n\rERROR: failed to enable advertising instance with error code: %d\n\r",rc);
            return rc;
        }
        console_printf("\n\rInstance %u enabled successfully with duration: %d, max events: %d\n\r",
                       pAdvEnb->instance, pAdvEnb->duration, pAdvEnb->max_events);
    }
    else //disable
    {
        rc = ble_gap_ext_adv_stop(pAdvEnb->instance);
        if ( rc != 0 )
        {
            console_printf("\n\rERROR: failed to disable advertising instance with error code: %d\n\r",rc);
            return rc;
        }
        console_printf("\n\rInstance %u disabled successfully\n\r", pAdvEnb->instance);
    }

    return rc;
}

int nimble_host_ext_scan_cfg(AtCmdExtScanCfg_t *pScanCfg)
{
    int rc = 0;

    // Sanity Check
    if (pScanCfg)
    {
        /* Store scan parameters to be used later when scan will be enabled */
        os_memcpy(&extScanCB.extScanParams, pScanCfg, sizeof(AtCmdExtScanCfg_t));

        console_printf("\n\rScan parameters configured successfully with the following parameters:");
        console_printf("\n\rfilter_policy: %d, own_address_type: %d, scan_interval (0.625ms units): %d",
                       extScanCB.extScanParams.filter_policy, extScanCB.extScanParams.own_address_type,
                       extScanCB.extScanParams.scan_interval);
        console_printf("\n\rscan_phy: %d, scan_type: %d, scan_window (0.625ms units): %d\n\r",
                       extScanCB.extScanParams.scan_phy, extScanCB.extScanParams.scan_type,
                       extScanCB.extScanParams.scan_window);
    }
    else
    {
        console_printf("\n\rERROR: failed to configure scan parameters\n\r");
        rc = -1;
    }

    return rc;
}

int nimble_host_ext_scan_cfg_get(AtCmdExtScanCfg_t *pScanCfg)
{
    int rc = 0;

    // Sanity Check
    if (pScanCfg)
    {
        os_memcpy(pScanCfg, &extScanCB.extScanParams, sizeof(AtCmdExtScanCfg_t));
    }
    else
    {
        console_printf("\n\rERROR: failed to get scan parameters\n\r");
        rc = -1;
    }

    return rc;
}

int nimble_host_ext_scan_enable(ExtScanEnable_t *pScanEnb)
{
    int rc = 0;
    struct ble_gap_ext_disc_params uncoded_params = {0};
    struct ble_gap_ext_disc_params coded_params = {0};
    struct ble_gap_ext_disc_params* pUncodedParams = NULL;
    struct ble_gap_ext_disc_params* pCodedParams = NULL;

    // Sanity Check
    if ( !pScanEnb )
    {
        return -1;
    }

    // Reset scan results table data
    os_memset(extScanCB.extScanResults,0,(BLE_MAX_SCAN_RESULTS * sizeof(scanResEntry_t)));
    extScanCB.extScanResultsCount = 0;
    extScanCB.extScanResultsTotal = 0;

    // Disable scan
    if (pScanEnb->enable == 0)
    {
        rc = ble_gap_disc_cancel();
        if (rc != 0)
        {
            console_printf("\n\rERROR: failed to disable scan with error code: %d\n\r",rc);
            return rc;
        }
        //console_printf("\n\rScan disabled successfully \n\r");
    }
    else // Enable scan
    {
        // Init uncodedParam if set scan phy to 1M
        if (extScanCB.extScanParams.scan_phy & BLE_GAP_LE_PHY_1M_MASK)
        {
            uncoded_params.itvl = extScanCB.extScanParams.scan_interval;
            uncoded_params.window = extScanCB.extScanParams.scan_window;
            uncoded_params.passive = extScanCB.extScanParams.scan_type;
            pUncodedParams = &uncoded_params;
        }

        // Init codedParam if set scan phy to coded
        if (extScanCB.extScanParams.scan_phy & BLE_GAP_LE_PHY_CODED_MASK)
        {
            coded_params.itvl = extScanCB.extScanParams.scan_interval;
            coded_params.window = extScanCB.extScanParams.scan_window;
            coded_params.passive = extScanCB.extScanParams.scan_type;
            pCodedParams = &coded_params;
        }

        rc = ble_gap_ext_disc(own_addr_type, pScanEnb->duration, pScanEnb->period, pScanEnb->filter_duplicate,
                              extScanCB.extScanParams.filter_policy, 0, pUncodedParams, pCodedParams,
                              gap_event_cb, NULL);

        if ( rc != 0)
        {
            console_printf("\n\rERROR: failed to enable scan with error code: %d\n\r",rc);
            return rc;
        }
        console_printf("\n\rScan enabled successfully with scan duration of %d ms", (10*pScanEnb->duration));
        console_printf("\n\rScanning for peers...\n\r");
    }

    return rc;
}

int nimble_host_ext_connect(uint8_t* bd_addr, uint8_t addr_type)
{
    int rc = 0;
    ble_addr_t peer_addr;

    //Set the address to connect to
    peer_addr.type = addr_type;
    BLE_COPY_BD_ADDRESS(peer_addr.val, bd_addr);

    console_printf("\n\rConnection attempt to peer ");
    print_addr(peer_addr.val);
    console_printf("\n\r");

    rc = ble_gap_ext_connect(own_addr_type, &peer_addr, 0,
                             BLE_GAP_LE_PHY_1M_MASK | BLE_GAP_LE_PHY_2M_MASK,
                             NULL, NULL, NULL, gap_event_cb, NULL);

    if (rc != 0)
    {
        console_printf("\n\rERROR: Connection attempt failed with error code: %d\n\r",rc);
        return rc;
    }

    return rc;
}

int nimble_host_ext_connect_params(uint8_t* bd_addr, uint8_t addr_type, uint8 phy_mask, uint32 interval_min_us, uint32 interval_max_us, uint32 supervision_timeout_us, int32_t duration_ms)
{
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x0010,
        .scan_window = 0x0010,
        .itvl_min = (interval_min_us == 0) ? BLE_GAP_INITIAL_CONN_ITVL_MIN : BLE_GAP_CONN_ITVL_MS((double) interval_min_us / 1000),
        .itvl_max = (interval_max_us == 0) ? BLE_GAP_INITIAL_CONN_ITVL_MAX : BLE_GAP_CONN_ITVL_MS((double) interval_max_us / 1000),
        .latency = BLE_GAP_INITIAL_CONN_LATENCY,
        .supervision_timeout = (supervision_timeout_us == 0) ? BLE_GAP_INITIAL_SUPERVISION_TIMEOUT : (double) supervision_timeout_us / 1000 / 10,
        .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
        .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
    };

    int rc = 0;
    ble_addr_t peer_addr;

    //Set the address to connect to
    peer_addr.type = addr_type;
    BLE_COPY_BD_ADDRESS(peer_addr.val, bd_addr);

    console_printf("\n\rConnection attempt to peer ");
    print_addr(peer_addr.val);
    console_printf("\n\rduration = %d\n\r", duration_ms);

    rc = ble_gap_ext_connect(own_addr_type, &peer_addr, duration_ms,
                             phy_mask,
                             &conn_params, &conn_params, &conn_params, gap_event_cb, NULL);

    if (rc != 0)
    {
        console_printf("\n\rERROR: Connection attempt failed with error code: %d\n\r",rc);
        return rc;
    }

    return rc;
}

int nimble_host_gap_update_params(uint16 connHandle, uint32 interval_min_us, uint32 interval_max_us, uint32 supervision_timeout_us, uint32 latency, uint16 min_ce_len, uint16 max_cle_len)
{
    struct ble_gap_upd_params upd_params = {
        .itvl_min = (interval_min_us == 0) ? BLE_GAP_INITIAL_CONN_ITVL_MIN : BLE_GAP_CONN_ITVL_MS((double) interval_min_us / 1000),
        .itvl_max = (interval_max_us == 0) ? BLE_GAP_INITIAL_CONN_ITVL_MAX : BLE_GAP_CONN_ITVL_MS((double) interval_max_us / 1000),
        .latency = BLE_GAP_INITIAL_CONN_LATENCY,
        .supervision_timeout = (supervision_timeout_us == 0) ? BLE_GAP_INITIAL_SUPERVISION_TIMEOUT : (double) supervision_timeout_us / 1000 / 10,
        .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
        .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
    };

    return ble_gap_update_params(connHandle, &upd_params);
}

int nimble_host_ext_disconnect(uint8_t* bd_addr, uint8_t addr_type)
{
    int rc = 0;
    uint8 i = 0;
    uint16 connHandle;
    ble_addr_t peer_addr;
    struct ble_gap_conn_desc disc_desc;

    //Set the address to disconnect from
    peer_addr.type = addr_type;
    BLE_COPY_BD_ADDRESS(peer_addr.val, bd_addr);

    //Disconnect all connections
    if (os_memcmp(bd_addr,empty_addr,BLE_DEV_ADDR_LEN) == 0)
    {
        console_printf("\n\rDisconnecting all peers...");
        for (i=0; i<BLE_MAX_SUPPORTED_CONNS; i++)
        {
            connHandle = extConnCB.connHandles[i];
            if (connHandle != BLE_HS_CONN_HANDLE_NONE)
            {
                rc = ble_gap_conn_find(connHandle, NULL);
                if (rc == 0)
                {
                    ble_gap_terminate(connHandle, BLE_ERR_REM_USER_CONN_TERM);
                }
                remove_from_connected_peers(connHandle);
            }
        }
        console_printf("\n\rDisconnected all peers completed");
    }
    else //Disconnect from requested connection
    {
        //Find the connection handle of this address
        rc = ble_gap_conn_find_by_addr(&peer_addr, &disc_desc);
        if (rc != 0)
        {
            console_printf("\n\rERROR: No connection exists with address: ");
            print_addr(peer_addr.val);
            return rc;
        }

        rc = ble_gap_terminate(disc_desc.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0)
        {
            console_printf("\n\rERROR: Connection not exists with address ");
            print_addr(peer_addr.val);
            return rc;
        }
        console_printf("\n\rDisconnected from peer ");
        print_addr(peer_addr.val);
        console_printf("\n\r");
    }

    return rc;
}

/* Return number of connected peers */
int nimble_host_connected_peers(uint8_t *peerAddress, uint8_t *peerAddressType)
{
    int rc = 0;
    struct ble_gap_conn_desc connDesc;

    if (peerAddress == NULL)
    {
        return (-1);
    }

    if (extConnCB.connCount == 0)
    {
        return (0);
    }

    rc = ble_gap_conn_find(extConnCB.connHandles[0], &connDesc);
    if (rc != 0)
    {
        return -1;
    }

    if (peerAddress != NULL)
    {
        BLE_COPY_BD_ADDRESS(peerAddress, connDesc.peer_ota_addr.val);
    }

    if (peerAddressType != NULL)
    {
        *peerAddressType = connDesc.peer_ota_addr.type;
    }

    return (1);
}

int nimble_host_delete_all_bond_info(void)
{
    int rc;

    if (!nimble_host_is_enabled())
    {
        Report("\n\rBLE is stopped, run ble_start.\n\r");
        return 0;
    }

    //Disconnect all links
    nimble_host_ext_disconnect(empty_addr,0);

    //Clear all bond information
    rc = ble_store_clear();

    if (rc != 0)
    {
        Report("\n\rBLE bonding information failed to be deleted with status 0x%x",rc);
        return rc;
    }

    Report("\n\rBLE bonding information was successfully deleted");
    return 0;
}

int nimble_host_set_mac_addr(uint8_t addressType)
{
    ble_addr_t addr;
    int rc = 0;

    if (addressType == BLE_OWN_ADDR_PUBLIC)
    {
        addr.type = BLE_OWN_ADDR_PUBLIC;
        rc = ble_hs_id_copy_addr(addr.type, addr.val, NULL);
        if (rc != 0)
        {
            return -1;
        }
        ble_hs_id_set_pub(addr.val);
    }
    else if (addressType == BLE_OWN_ADDR_RANDOM)
    {
        rc = ble_hs_id_gen_rnd(0, &addr);
        if (rc != 0)
        {
            return -1;
        }

        rc = ble_hs_id_set_rnd(addr.val);
        if (rc != 0)
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }

    own_addr_type = addressType;

    return 0;
}

int nimble_host_get_mac_addr(ble_addr_t *addr)
{
    if (addr == NULL)
    {
        return -1;
    }

    addr->type = own_addr_type;

    return ble_hs_id_copy_addr(addr->type, addr->val, NULL);
}

int nimble_host_start(void)
{
    int rc = 0;

    BleIf_EnableBLE();

    ble_example_init();

    nimble_port_init();

    ble_transport_ll_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_dis_init();

    gatt_svr_init();

#if BLE_STORE_CONFIG_PERSIST
    ble_store_config_init();
#else
    ble_store_ram_init();
#endif

    /* Create task which handles default event queue for host stack. */
    rc = ble_npl_task_init(&s_task_host, "nimble_host", nimble_host_task,
                      NULL, NIMBLE_THRD_PRIORITY, BLE_NPL_TIME_FOREVER,
                      NIMBLE_THRD_DEFAULT_STACK, NIMBLE_THRD_STACK_SIZE);

    Report("\n\r-------------- Wait for BLE Host\n\r");

    rc = osi_SyncObjWait(&hostInitEventSyncObj, OSI_WAIT_FOR_SECOND);
    if(OSI_OK != rc)
    {
        Report("\n\rBleIf_EnableBLE: Failed to receive Host Init Done. error number %d", rc);
        ASSERT_GENERAL(0);
    }

    rc = osi_SyncObjDelete(&hostInitEventSyncObj);
    if(OSI_OK != rc)
    {
        Report("\n\rBleIf_EnableBLE: Failed to delete sync object. error number %d", rc);
        ASSERT_GENERAL(0);
    }

    return rc;
}

int nimble_host_stop(void)
{
    static struct ble_hs_stop_listener listener;
    int rc;

    rc = ble_hs_stop(&listener, ble_stop_cb, NULL);
    if (rc == BLE_HS_EALREADY) {
        console_printf("\n\rBLE host is already stopped\n\r");
        return 0;
    }

    ble_gatts_reset();

    ble_npl_eventq_delete(nimble_port_get_dflt_eventq());

    rc = ble_npl_task_remove(&s_task_host);
    if(OSI_OK != rc)
    {
        Report("\n\rERROR: FAILED to remove host task\n\r");
        return rc;
    }

    return 0;
}

int nimble_host_is_enabled(void)
{
    return ble_hs_is_enabled();
}

void register_extra_gap_event_cb(void (*cb)(void *, void *))
{
    extra_gap_event_cb = cb;
}
