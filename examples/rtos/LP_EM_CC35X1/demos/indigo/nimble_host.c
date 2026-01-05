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
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/dis/ble_svc_dis.h"

#include "ble_if.h"
#include "nimble_host.h"

/******************************************************************************
                      Defines
******************************************************************************/
#ifdef CC33XX
#define NIMBLE_THRD_PRIORITY       (3)
#else
#define NIMBLE_THRD_PRIORITY       (8)
#endif

#define NIMBLE_THRD_DEFAULT_STACK  NULL
#define NIMBLE_THRD_STACK_SIZE     (4096)

#define BLE_MGF_DATA_LEN           (3)
#define BLE_MAX_SCAN_RESULTS       (8)
#define BLE_MAX_SUPPORTED_CONNS    MYNEWT_VAL(BLE_MAX_CONNECTIONS)
#define BLE_DEVICE_LOCAL_NAME_LEN  MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME_MAX_LENGTH)
#define BLE_DEVICE_LOCAL_NAME      MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME)
#define BLE_STORE_CONFIG_PERSIST   MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST)

#define BLE_DEFAULT_PASSKEY        123456

/*******************************************************************************
                      Macros
******************************************************************************/
#define BLE_COPY_BD_ADDRESS( dstBd, srcBd )                                  \
  (dstBd)[0] = (srcBd)[5];                                                   \
  (dstBd)[1] = (srcBd)[4];                                                   \
  (dstBd)[2] = (srcBd)[3];                                                   \
  (dstBd)[3] = (srcBd)[2];                                                   \
  (dstBd)[4] = (srcBd)[1];                                                   \
  (dstBd)[5] = (srcBd)[0];

/******************************************************************************
                      Typedef
******************************************************************************/
typedef struct
{
    ble_addr_t addr;
    int8_t rssi;
    uint8_t prim_phy;
    char local_name[BLE_DEVICE_LOCAL_NAME_LEN];
}scanResEntry_t;

typedef struct
{
    ExtScanCfg_t   extScanParams;
    scanResEntry_t extScanResults[BLE_MAX_SCAN_RESULTS];
    uint8          extScanResultsCount;
    uint8          extScanResultsTotal;
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

static void print_scan_results()
{
    uint8 i;

    if ( extScanCB.extScanResultsCount == 0 )
    {
        console_printf("\n\rNo scan results found\n\r");
    }
    else
    {
        console_printf("\n\rFound total of %d scan results. First %d results:",
                       extScanCB.extScanResultsTotal, BLE_MAX_SCAN_RESULTS);
        UART_PRINT(lineBreak);
        printBorder('-', 65);
        UART_PRINT(lineBreak);
        console_printf("| ID |         NAME         |      ADDRESS      |  TYPE  | RSSI |");
        UART_PRINT(lineBreak);
        printBorder('-', 65);
        UART_PRINT(lineBreak);
        for (i=0; i<extScanCB.extScanResultsCount; i++)
        {
            console_printf("| %02d ",i);
            console_printf("| %-20s ", extScanCB.extScanResults[i].local_name);
            console_printf("| ");
            print_addr(extScanCB.extScanResults[i].addr.val);
            console_printf(" | %-6s", extScanCB.extScanResults[i].addr.type == 0 ? "PUBLIC" : "RANDOM");
            console_printf(" | %04d |", extScanCB.extScanResults[i].rssi);
            UART_PRINT(lineBreak);
        }
        printBorder('-', 65);
        UART_PRINT(lineBreak);
    }
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

static void print_connected_peers()
{
    uint8 i;
    struct ble_gap_conn_desc connDesc;

    if ( extConnCB.connCount == 0 )
    {
        console_printf("\n\rNo connected peers found\n\r");
    }
    else
    {
        console_printf("\n\rConnected devices:");
        UART_PRINT(lineBreak);
        printBorder('-', 63);
        UART_PRINT(lineBreak);
        console_printf("| Handle |   PEER ADDRESS    |  TYPE  |  OWN ROLE  | BONDED? |");
        UART_PRINT(lineBreak);
        printBorder('-', 63);
        UART_PRINT(lineBreak);
        for (i=0; i<BLE_MAX_SUPPORTED_CONNS; i++)
        {
            if (extConnCB.connHandles[i] != BLE_HS_CONN_HANDLE_NONE)
            {
                ble_gap_conn_find(extConnCB.connHandles[i], &connDesc);
                console_printf("| %d      ", connDesc.conn_handle);
                console_printf("| ");
                print_addr(connDesc.peer_ota_addr.val);
                console_printf(" | %-6s", connDesc.peer_ota_addr.type == 0 ? "PUBLIC" : "RANDOM");
                console_printf(" | %-10s", connDesc.role == 0 ? "CENTRAL" : "PERIPHERAL");
                console_printf(" | %-4s    |", connDesc.sec_state.bonded == 0 ? "NO" : "YES");
                UART_PRINT(lineBreak);

            }
        }
        printBorder('-', 63);
        UART_PRINT(lineBreak);
    }
}

static void adv_enable(void)
{
    ExtAdvEnable_t extAdvEnable;

    extAdvEnable.enable = 1;
    extAdvEnable.instance = 0;
    extAdvEnable.duration = 0;
    extAdvEnable.max_events = 0;

    nimble_host_ext_adv_enable(&extAdvEnable);
}

static int is_peer_bonded(uint16 conn_handle)
{
    struct ble_store_key_sec key_sec;
    struct ble_store_value_sec out_bond;
    struct ble_gap_conn_desc desc;
    int rc;

    rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        return FALSE;
    }

    memset(&key_sec, 0, sizeof key_sec);
    key_sec.peer_addr = desc.peer_id_addr;

    rc = ble_store_read_peer_sec(&key_sec, &out_bond);
    if (rc != 0) {
        return FALSE;
    }

    return TRUE;
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

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            console_printf("\n\rConnection %s with status=%d",
                        event->connect.status == 0 ? "established successfully" : "failed",
                        event->connect.status);

            if (event->connect.status == 0)
            {
                add_to_connected_peers(event->connect.conn_handle);
                if (is_peer_bonded(event->connect.conn_handle))
                {
                    console_printf("\n\rEncryption started");
                }
                else
                {
                    console_printf("\n\rPairing started");
                }
                rc = ble_gap_security_initiate(event->connect.conn_handle);
                if (rc != 0) {
                   console_printf("\n\rPairing failed with status %d", rc);
                   return 0;
                }
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            console_printf("\n\rDisconnected from ");
            print_addr(event->disconnect.conn.peer_ota_addr.val);
            console_printf(" with reason 0x%x\n\r",event->disconnect.reason);
            remove_from_connected_peers(event->disconnect.conn.conn_handle);
            break;

        case BLE_GAP_EVENT_CONN_UPDATE:
            console_printf("\n\rConnection parameters update %s with status=%d",
                           event->conn_update.status == 0 ? "completed successfully" : "failed",
                           event->conn_update.status);
            break;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            print_scan_results();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            console_printf("\n\rAdvertising instance %u completed with termination code: 0x%x",
                           event->adv_complete.instance,
                           event->adv_complete.reason);
            if (extConnCB.connCount < BLE_MAX_SUPPORTED_CONNS)
            {
                adv_enable();
            }
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            console_printf("\n\rEncryption change event with status=0x%x",
                        event->enc_change.status);
            break;

        case BLE_GAP_EVENT_PASSKEY_ACTION:
            gap_event_passkey_action(event);
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            console_printf("\n\rsubscribe event attr_handle=%d\n",
                        event->subscribe.attr_handle);
            break;

        case BLE_GAP_EVENT_MTU:
            console_printf("\n\rmtu update event; conn_handle=%d cid=%d mtu=%d\n",
                        event->mtu.conn_handle,
                        event->mtu.channel_id,
                        event->mtu.value);
            break;

        case BLE_GAP_EVENT_IDENTITY_RESOLVED:
            console_printf("\n\ridentity resolved; peer identity address: ");
            print_addr(event->identity_resolved.peer_id_addr.val);
            break;

        case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
            console_printf("\n\rPHY update complete; status=%d, conn_handle=%d, tx_phy=%d, rx_phy=%d\n",
                           event->phy_updated.status,
                           event->phy_updated.conn_handle,
                           event->phy_updated.tx_phy,
                           event->phy_updated.rx_phy);
            break;

        case BLE_GAP_EVENT_EXT_DISC:
            if (extScanCB.extScanResultsCount < BLE_MAX_SCAN_RESULTS)
            {
                rc = ble_hs_adv_parse_fields(&adv_fields, event->ext_disc.data, event->ext_disc.length_data);

                if (rc != 0)
                {
                    console_printf("Discard adv report with invalid data fields. error code: %d\n\r",rc);
                    return 0;
                }

                // Store partial adv report data
                os_memcpy(&extScanCB.extScanResults[extScanCB.extScanResultsCount].addr,&event->ext_disc.addr,sizeof(ble_addr_t));
                extScanCB.extScanResults[extScanCB.extScanResultsCount].prim_phy = event->ext_disc.prim_phy;
                extScanCB.extScanResults[extScanCB.extScanResultsCount].rssi = event->ext_disc.rssi;
                os_memcpy(extScanCB.extScanResults[extScanCB.extScanResultsCount].local_name,no_local_name,7);

                if (adv_fields.name != NULL) {
                    if (adv_fields.name_len < (BLE_DEVICE_LOCAL_NAME_LEN-1)) {
                        os_memcpy(extScanCB.extScanResults[extScanCB.extScanResultsCount].local_name, adv_fields.name, adv_fields.name_len);
                        extScanCB.extScanResults[extScanCB.extScanResultsCount].local_name[adv_fields.name_len] = '\0';
                    }
                }

                extScanCB.extScanResultsCount++;
            }
            extScanCB.extScanResultsTotal++;
            break;

        case BLE_GAP_EVENT_PAIRING_COMPLETE:
            console_printf("\n\rPairing/Encryption %s with status=%d",
                        event->pairing_complete.status == 0 ? "completed successfully" : "failed",
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
int nimble_host_ext_adv_cfg(ExtAdvCfg_t *pAdvCfg)
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

    //Init the params
    os_memset(&params, 0, sizeof(params));

    /* Adv Properties */
    params.legacy_pdu = pAdvCfg->legacy;
    if (params.legacy_pdu) {
        params.connectable = 1;
        params.scannable = 1;
        params.include_tx_power = 0;
    }
    else
    {
        params.connectable = 1;
        params.scannable = 0;
        params.include_tx_power = 0;
    }
    params.directed = 0;
    params.high_duty_directed = 0;
    params.anonymous = 0;
    params.scan_req_notif = 0;
    if (params.directed && params.legacy_pdu) {
        params.scannable = 0;
    }

    /* Other adv parameters */
    params.itvl_min = BLE_GAP_ADV_ITVL_MS(pAdvCfg->interval_ms);
    params.itvl_max = BLE_GAP_ADV_ITVL_MS(pAdvCfg->interval_ms);
    params.channel_map = 0x7;
    params.peer.type = BLE_OWN_ADDR_PUBLIC;
    os_memset(params.peer.val,0x00,BLE_DEV_ADDR_LEN);
    params.own_addr_type = own_addr_type;
    params.filter_policy = 0x0;
    params.tx_power = 127;
    params.primary_phy = pAdvCfg->prim_phy;
    params.secondary_phy = pAdvCfg->sec_phy;
    params.sid = 0;

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
    rc = ble_gap_ext_adv_set_data(pAdvCfg->instance, adv_data);
    if ( rc != 0 )
    {
        console_printf("\n\rERROR: failed to configure the data to include in advertisements packets, with error code: %d\n\r", rc);
        return (rc);
    }

    console_printf("\n\rInstance %u configured successfully with device name: '%s', BD address: ",
                   pAdvCfg->instance, device_name);
    print_addr(addr.val);

    return 0;
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

int nimble_host_ext_scan_cfg(ExtScanCfg_t *pScanCfg)
{
    int rc = 0;

    // Sanity Check
    if (pScanCfg)
    {
        /* Store scan parameters to be used later when scan will be enabled */
        os_memcpy(&extScanCB.extScanParams, pScanCfg, sizeof(ExtScanCfg_t));

        console_printf("\n\rScan parameters configured successfully with the following parameters:");
        console_printf("\n\rfilter_policy: %d, own_address_type: %d, scan_interval (ms): %d",
                       extScanCB.extScanParams.filter_policy, extScanCB.extScanParams.own_address_type,
                       extScanCB.extScanParams.scan_interval_ms);
        console_printf("\n\rscan_phy: %d, scan_type: %d, scan_window (ms): %d\n\r",
                       extScanCB.extScanParams.scan_phy, extScanCB.extScanParams.scan_type,
                       extScanCB.extScanParams.scan_window_ms);
    }
    else
    {
        console_printf("\n\rERROR: failed to configure scan parameters\n\r");
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
        if ( rc != 0)
        {
            console_printf("\n\rERROR: failed to disable scan with error code: %d\n\r",rc);
            return rc;
        }
        console_printf("\n\rScan disabled successfully \n\r");
    }
    else // Enable scan
    {
        // Init uncodedParam if set scan phy to 1M
        if (extScanCB.extScanParams.scan_phy & BLE_GAP_LE_PHY_1M_MASK)
        {
            uncoded_params.itvl = BLE_GAP_SCAN_ITVL_MS(extScanCB.extScanParams.scan_interval_ms);
            uncoded_params.window = BLE_GAP_SCAN_ITVL_MS(extScanCB.extScanParams.scan_window_ms);
            uncoded_params.passive = extScanCB.extScanParams.scan_type;
            pUncodedParams = &uncoded_params;
        }

        // Init codedParam if set scan phy to coded
        if (extScanCB.extScanParams.scan_phy & BLE_GAP_LE_PHY_CODED_MASK)
        {
            coded_params.itvl = BLE_GAP_SCAN_ITVL_MS(extScanCB.extScanParams.scan_interval_ms);
            coded_params.window = BLE_GAP_SCAN_ITVL_MS(extScanCB.extScanParams.scan_window_ms);
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

int nimble_host_ext_connect_params(uint8_t* bd_addr, uint8_t addr_type, uint8 phy_mask, uint32 interval_min_us, uint32 interval_max_us, uint32 supervision_timeout_us)
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

    rc = ble_gap_ext_connect(BLE_OWN_ADDR_PUBLIC, &peer_addr, 0,
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
    }

    return rc;
}

int nimble_host_connected_peers(void)
{
    print_connected_peers();
    return (0);
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

int nimble_host_get_bd_address(uint8_t addr_type)
{
    int rc;
    ble_addr_t addr;

    if (!nimble_host_is_enabled())
    {
        Report("\n\rBLE is stopped, run ble_start.\n\r");
        return 0;
    }

    addr.type = addr_type;

    rc = ble_hs_id_copy_addr(addr.type, addr.val, NULL);
    if ( rc != 0 )
    {
        if (rc == BLE_HS_ENOADDR)
        {
            console_printf("\n\rDevice does not have an identity address of the requested type\n\r");
        }
        else
        {
            console_printf("\n\rError: failed to retrieve device's address\n\r");
        }
    }
    else
    {
        console_printf("\n\rBD address: ");
        print_addr(addr.val);
    }

    return rc;
}

int nimble_host_set_bd_address(uint8_t addr_type)
{
    ble_addr_t addr;
    int rc = 0;

    if (!nimble_host_is_enabled())
    {
        Report("\n\rBLE is stopped, run ble_start.\n\r");
        return 0;
    }

    if (addr_type == BLE_OWN_ADDR_PUBLIC)
    {
        /*
         * There is no standard way to set the local public address.
         * Just update our own address type.
         */
        addr.type = BLE_OWN_ADDR_PUBLIC;
        console_printf("\n\rSet identity address type to PUBLIC successfully\n\r");
    }
    else if (addr_type == BLE_OWN_ADDR_RANDOM)
    {
        addr.type = BLE_OWN_ADDR_RANDOM;
        rc = ble_hs_id_gen_rnd(0, &addr);
        if (rc != 0)
        {
            console_printf("\n\rError: Failed to generate random address\n\r");
            return -1;
        }

        rc = ble_hs_id_set_rnd(addr.val);
        if (rc != 0)
        {
            console_printf("\n\rError: Failed to set random address\n\r");
            return -1;
        }
        console_printf("\n\rSet identity address type to RANDOM successfully");
        console_printf("\n\rRANDOM BD address: ");
        print_addr(addr.val);
    }
    else
    {
        console_printf("\n\rError: Received invalid address type\n\r");
        return -1;
    }

    own_addr_type = addr.type;

    return 0;
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
