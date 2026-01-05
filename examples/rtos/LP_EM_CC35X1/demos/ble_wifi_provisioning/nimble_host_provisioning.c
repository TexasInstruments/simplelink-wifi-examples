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

#include "ble_cmd.h"
#include "ble_if.h"
#include "ble_wifi_provisioning.h"
#include "nimble_host_provisioning.h"

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
    return;
}

static void add_to_connected_peers(uint16 handle)
{
    uint8 i;

    if (handle != BLE_HS_CONN_HANDLE_NONE)
    {
        for (i=0; i<BLE_MAX_SUPPORTED_CONNS; i++)
        {
            if (extConnCB.connHandles[i] == BLE_HS_CONN_HANDLE_NONE)
            {
                extConnCB.connHandles[i] = handle;
                extConnCB.connCount++;
                break;
            }
        }
    }
}

static void remove_from_connected_peers(uint16 handle)
{
    uint8 i;

    if (handle != BLE_HS_CONN_HANDLE_NONE)
    {
        for (i=0; i<BLE_MAX_SUPPORTED_CONNS; i++)
        {
            if (extConnCB.connHandles[i] == handle)
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
    return;
}

/******************************************************************************
                      GATT SERVER
******************************************************************************/
#define BLE_SVC_PROVISIONING_UUID16                 0XCC00
#define BLE_SVC_PROVISIONING_CHR_UUID16_SSID        0XCC01
#define BLE_SVC_PROVISIONING_CHR_UUID16_PASSWORD    0XCC02
#define BLE_SVC_PROVISIONING_CHR_UUID16_CONNECTION  0XCC03
#define BLE_SVC_PROVISIONING_CHR_UUID16_SECURITY    0XCC04

uint16_t wlan_ssid_handle = 0;
uint16_t wlan_password_handle = 0;
uint16_t wlan_conn_notify_handle = 0;
uint16_t wlan_security_handle = 0;

static int gatt_svr_chr_access_SSID(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svr_chr_access_Password(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svr_chr_access_WLAN_Connect(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svr_chr_access_Security(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Provisioning */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROVISIONING_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /* Characteristic: SSID Write */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROVISIONING_CHR_UUID16_SSID),
            .access_cb = gatt_svr_chr_access_SSID,
            .val_handle = &wlan_ssid_handle,
            .flags = BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: PASS Write */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROVISIONING_CHR_UUID16_PASSWORD),
            .access_cb = gatt_svr_chr_access_Password,
            .val_handle = &wlan_password_handle,
            .flags = BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: Read send WLAN Connection */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROVISIONING_CHR_UUID16_CONNECTION),
            .access_cb = gatt_svr_chr_access_WLAN_Connect,
            .val_handle = &wlan_conn_notify_handle,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        }, {
            /* Characteristic: SECURITY Write */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROVISIONING_CHR_UUID16_SECURITY),
            .access_cb = gatt_svr_chr_access_Security,
            .val_handle = &wlan_security_handle,
            .flags = BLE_GATT_CHR_F_WRITE,
        }, {
            0, /* No more characteristics in this service */
        }, }
    },
    {
        0, /* No more services */
    },
};

static int gatt_svr_chr_access_SSID(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *SSIDctxt, void *arg)
{
    int ret = 0;
    provSaveSSID(SSIDctxt->om->om_data,SSIDctxt->om->om_len);
    return ret;
}

static int gatt_svr_chr_access_Password(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *Passctxt, void *arg)
{
    int ret = 0;
    provSavePassword(Passctxt->om->om_data,Passctxt->om->om_len);
    return ret;
}

static int gatt_svr_chr_access_WLAN_Connect(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int ret = 0;
    static const uint16_t wlan_connect = WLAN_CONNECT_START;
    assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
    ret = os_mbuf_append(ctxt->om, &wlan_connect, sizeof(wlan_connect));
    Report("\r\n[BLE GATT]: WLAN CONNECT Requested");
    osi_SyncObjSignal(&prov_CB.StartConnectEventSyncObj);
    return (ret == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int gatt_svr_chr_access_Security(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *Passctxt, void *arg)
{
    int ret = 0;
    provSaveSecurity(Passctxt->om->om_data,Passctxt->om->om_len);
    return ret;
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
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            console_printf("\n\r[BLE]: Connection %s with status=%d",
                        event->connect.status == 0 ? "established successfully" : "failed",
                        event->connect.status);
            if (event->connect.status == 0)
            {
                add_to_connected_peers(event->connect.conn_handle);
            }
            else
            {
                advConfigure();
                advEnable();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            console_printf("\n\r[BLE]: Disconnected from ");
            print_addr(event->disconnect.conn.peer_ota_addr.val);
            console_printf(" with reason %d\n\r",event->disconnect.reason);
            remove_from_connected_peers(event->disconnect.conn.conn_handle);
            break;

        case BLE_GAP_EVENT_CONN_UPDATE:
            console_printf("\n\r[BLE]: Connection parameters update %s with status=%d",
                           event->conn_update.status == 0 ? "completed successfully" : "failed",
                           event->conn_update.status);
            break;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            console_printf("\n\rAdvertising instance %u completed with termination code: 0x%x",
                           event->adv_complete.instance,
                           event->adv_complete.reason);
            if (extConnCB.connCount < BLE_MAX_SUPPORTED_CONNS)
            {
                advConfigure();
                advEnable();
            }
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            console_printf("\n\rsubscribe event; conn_handle=%d attr_handle=%d"
                           "\n\rcur_notify=%d wlan_conn_notify_handle=%d\n",
                           event->subscribe.conn_handle,
                           event->subscribe.attr_handle,
                           event->subscribe.cur_notify,
                           wlan_conn_notify_handle);
            break;

        case BLE_GAP_EVENT_EXT_DISC:
            break;

        case BLE_GAP_EVENT_PAIRING_COMPLETE:
            console_printf("\n\r[BLE]: Pairing %s with status=%d",
                        event->pairing_complete.status == 0 ? "completed successfully" : "failed",
                        event->pairing_complete.status);
            break;

        default:
            break;
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
int nimble_host_gatt_svr_chr_notify_wlan_connection(uint8_t status)
{
    int rc;
    struct os_mbuf *om;
    uint8_t wlan_connection_status[1];

    wlan_connection_status[0] = status;

    om = ble_hs_mbuf_from_flat(&wlan_connection_status, sizeof(wlan_connection_status));

    rc = ble_gatts_notify_custom(extConnCB.connHandles[0], wlan_conn_notify_handle, om);

    return rc;
}

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
