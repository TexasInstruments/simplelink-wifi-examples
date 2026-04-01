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

/*!
    \file   ota_wifi.c
    \brief  WiFi connection handling for the OTA example.

    This module manages WiFi initialization, scanning, and interactive
    connection for the OTA example application. It encapsulates all WiFi
    driver interaction, TCP/IP stack setup, and connection state.

    Dependencies: wlan_if.h, tcpip_if.h, dns_if.h, osi_kernel.h, uart_term.h,
                  psa_fwu.h (for FW slot query only)
*/

/* Standard includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  /* atoi */

/* Application headers */
#include "ota_wifi.h"
#include "ota_example.h"
#include "uart_term.h"

/* Direct WiFi driver API */
#include "wlan_if.h"

/* TCP/IP and DNS */
#include "tcpip_if.h"
#include "dns_if.h"

/* OS abstraction */
#include "osi_kernel.h"

/* PSA Firmware Update API (for FW slot query) */
#include "ti/utils/FWU/psa_fwu.h"

/*****************************************************************************/
/*                           Defines                                         */
/*****************************************************************************/

#define OTA_SCAN_TIMEOUT_MS     10000
#define OTA_CONNECT_TIMEOUT_MS  15000
#define OTA_IP_TIMEOUT_MS       15000

/* Security type bitmap values (from rsn.h) */
#define SECURITY_TYPE_BITMAP_OPEN            (0)
#define SECURITY_TYPE_BITMAP_WPA             (1 << 1)
#define SECURITY_TYPE_BITMAP_WPA2            (1 << 2)
#define SECURITY_TYPE_BITMAP_WPA3            (1 << 3)
#define SECURITY_TYPE_BITMAP_PMF_CAPABLE     (1 << 4)
#define SECURITY_TYPE_BITMAP_PMF_REQUIRED    (1 << 5)
#define SECURITY_TYPE_MASK                   (SECURITY_TYPE_BITMAP_OPEN | SECURITY_TYPE_BITMAP_WPA | SECURITY_TYPE_BITMAP_WPA2 | SECURITY_TYPE_BITMAP_WPA3)

/*****************************************************************************/
/*                      WiFi Global State                                    */
/*****************************************************************************/

static void            *gStaNetif = NULL;
static OsiSyncObj_t     gScanSyncObj;
static OsiSyncObj_t     gConnectSyncObj;
static OsiSyncObj_t     gIpAcquiredSyncObj;
static WlanNetworkEntry_t gScanEntries[WLAN_MAX_SCAN_COUNT];
static uint32_t         gScanEntryCount = 0;
static volatile int     gConnected = 0;

/*****************************************************************************/
/*                      TCP/IP Callbacks                                     */
/*****************************************************************************/

/*!
    \brief  Send callback for TCP/IP stack - wraps Wlan_EtherPacketSend.
*/
static int otaTcpipSend(void *hNetif, uint8_t *pPayload, int16_t payloadLen,
                        uint32_t flags)
{
    Wlan_EtherPacketSend(WLAN_ROLE_STA, pPayload, payloadLen, (int16_t)flags);
    return 0;
}

/*!
    \brief  WiFi receive callback - passes incoming packets to TCP/IP stack.
*/
static void otaWifiReceive(WlanRole_e role, uint8_t *inBuf, uint32_t inLen)
{
    if (role == WLAN_ROLE_STA && gStaNetif != NULL)
    {
        TCPIP_IF_receive(gStaNetif, inBuf, inLen);
    }
}

/*!
    \brief  TCP/IP event callback - handles IP acquired/lost events.
*/
static void otaTcpipEvent(void *pNetif, TcpipStatue_e status, void *pParams)
{
    switch (status)
    {
    case E_TCPIP_STATUS_IP_ACQUIRED:
        UART_PRINT("[OTA] IP acquired\n\r");
        osi_SyncObjSignal(&gIpAcquiredSyncObj);
        break;
    case E_TCPIP_STATUS_IP_LOST:
        UART_PRINT("[OTA] IP lost\n\r");
        break;
    default:
        break;
    }
}

/*****************************************************************************/
/*                      WLAN Event Handler                                   */
/*****************************************************************************/

/*!
    \brief  WLAN event handler - processes WiFi driver events directly.

    Modeled after WlanStackEventHandler in network_terminal.c.
*/
static void otaWlanEventHandler(WlanEvent_t *pWlanEvent)
{
    if (pWlanEvent == NULL)
    {
        return;
    }

    switch (pWlanEvent->Id)
    {
    case WLAN_EVENT_CONNECT:
    {
        if (pWlanEvent->Data.Connect.Status < 0)
        {
            UART_PRINT("[OTA] Connection failed (status %d)\n\r",
                        pWlanEvent->Data.Connect.Status);
            osi_SyncObjSignal(&gConnectSyncObj);
            break;
        }

        UART_PRINT("[OTA] Connected to AP: %.*s, BSSID: %x:%x:%x:%x:%x:%x\n\r",
                    pWlanEvent->Data.Connect.SsidLen,
                    pWlanEvent->Data.Connect.SsidName,
                    pWlanEvent->Data.Connect.Bssid[0],
                    pWlanEvent->Data.Connect.Bssid[1],
                    pWlanEvent->Data.Connect.Bssid[2],
                    pWlanEvent->Data.Connect.Bssid[3],
                    pWlanEvent->Data.Connect.Bssid[4],
                    pWlanEvent->Data.Connect.Bssid[5]);

        /* Notify TCP/IP stack of link up and register RX callback */
        TCPIP_IF_notifyLinkChange(gStaNetif, E_TCPIP_LINK_UP);
        Wlan_EtherPacketRecvRegisterCallback(WLAN_ROLE_STA, otaWifiReceive);
        gConnected = 1;
        osi_SyncObjSignal(&gConnectSyncObj);
        break;
    }

    case WLAN_EVENT_DISCONNECT:
    {
        WlanEventDisconnect_t *pEventData = &pWlanEvent->Data.Disconnect;

        UART_PRINT("[OTA] Disconnected from AP (reason %d)\n\r",
                    pEventData->ReasonCode);

        /* Notify TCP/IP stack of link down and unregister RX callback */
        if (gStaNetif != NULL)
        {
            TCPIP_IF_notifyLinkChange(gStaNetif, E_TCPIP_LINK_DOWN);
        }
        Wlan_EtherPacketRecvRegisterCallback(WLAN_ROLE_STA, NULL);
        gConnected = 0;
        osi_SyncObjSignal(&gConnectSyncObj);
        break;
    }

    case WLAN_EVENT_SCAN_RESULT:
    {
        uint32_t i;
        uint32_t count = pWlanEvent->Data.ScanResult.NetworkListResultLen;

        if (count > WLAN_MAX_SCAN_COUNT)
        {
            count = WLAN_MAX_SCAN_COUNT;
        }
        gScanEntryCount = count;

        for (i = 0; i < count; i++)
        {
            memset(gScanEntries[i].Ssid, 0, WLAN_SSID_MAX_LENGTH);
            memcpy(gScanEntries[i].Ssid,
                   pWlanEvent->Data.ScanResult.NetworkListResult[i].Ssid,
                   pWlanEvent->Data.ScanResult.NetworkListResult[i].SsidLen);
            memcpy(gScanEntries[i].Bssid,
                   pWlanEvent->Data.ScanResult.NetworkListResult[i].Bssid,
                   WLAN_BSSID_LENGTH);
            gScanEntries[i].SsidLen =
                pWlanEvent->Data.ScanResult.NetworkListResult[i].SsidLen;
            gScanEntries[i].Rssi =
                (int8_t)pWlanEvent->Data.ScanResult.NetworkListResult[i].Rssi;
            gScanEntries[i].SecurityInfo =
                pWlanEvent->Data.ScanResult.NetworkListResult[i].SecurityInfo;
            gScanEntries[i].Channel =
                pWlanEvent->Data.ScanResult.NetworkListResult[i].Channel;
        }

        /* Print scan results table */
        UART_PRINT("\n\r %-3s %-33s %-6s %-4s %-10s\n\r",
                    "#", "SSID", "RSSI", "Ch", "Security");
        UART_PRINT(
            "--------------------------------------------------------------\n\r");
        for (i = 0; i < count; i++)
        {
            uint32_t secType = WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(
                                    gScanEntries[i].SecurityInfo) &
                               SECURITY_TYPE_MASK;
            const char *secStr;

            switch (secType)
            {
            case SECURITY_TYPE_BITMAP_OPEN:
                secStr = "Open";
                break;
            case SECURITY_TYPE_BITMAP_WPA:
                secStr = "WPA";
                break;
            case SECURITY_TYPE_BITMAP_WPA2:
                secStr = "WPA2";
                break;
            case SECURITY_TYPE_BITMAP_WPA3:
                secStr = "WPA3";
                break;
            case (SECURITY_TYPE_BITMAP_WPA | SECURITY_TYPE_BITMAP_WPA2):
                secStr = "WPA/WPA2";
                break;
            case (SECURITY_TYPE_BITMAP_WPA2 | SECURITY_TYPE_BITMAP_WPA3):
                secStr = "WPA2/WPA3";
                break;
            default:
                secStr = "Unknown";
                break;
            }

            UART_PRINT(" %-3d %-33.*s %-6d %-4d %s\n\r",
                        i + 1,
                        gScanEntries[i].SsidLen,
                        gScanEntries[i].Ssid,
                        gScanEntries[i].Rssi,
                        gScanEntries[i].Channel,
                        secStr);
        }

        osi_SyncObjSignal(&gScanSyncObj);
        break;
    }

    default:
        UART_PRINT("[OTA] WLAN event: 0x%x\n\r", pWlanEvent->Id);
        break;
    }
}

/*****************************************************************************/
/*                      Security Type Helper                                 */
/*****************************************************************************/

/*!
    \brief  Map scan result SecurityInfo to Wlan_Connect security type.
*/
static uint8_t getSecurityType(uint16_t securityInfo)
{
    uint32_t secType = WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(securityInfo) &
                       SECURITY_TYPE_MASK;

    switch (secType)
    {
    case SECURITY_TYPE_BITMAP_OPEN:
        return WLAN_SEC_TYPE_OPEN;
    case SECURITY_TYPE_BITMAP_WPA:
    case SECURITY_TYPE_BITMAP_WPA2:
    case (SECURITY_TYPE_BITMAP_WPA | SECURITY_TYPE_BITMAP_WPA2):
        return WLAN_SEC_TYPE_WPA_WPA2;
    case SECURITY_TYPE_BITMAP_WPA3:
        return WLAN_SEC_TYPE_WPA3;
    case (SECURITY_TYPE_BITMAP_WPA2 | SECURITY_TYPE_BITMAP_WPA3):
        return WLAN_SEC_TYPE_WPA2_PLUS;
    default:
        return WLAN_SEC_TYPE_WPA_WPA2;
    }
}

/*****************************************************************************/
/*                           API Functions                                   */
/*****************************************************************************/

int OTA_WIFI_initStack(void)
{
    int ret;

    /* Create synchronization objects */
    ret = osi_SyncObjCreate(&gScanSyncObj);
    if (ret != 0)
    {
        UART_PRINT("[OTA] Failed to create scan sync obj\n\r");
        return -1;
    }
    ret = osi_SyncObjCreate(&gConnectSyncObj);
    if (ret != 0)
    {
        UART_PRINT("[OTA] Failed to create connect sync obj\n\r");
        return -1;
    }
    ret = osi_SyncObjCreate(&gIpAcquiredSyncObj);
    if (ret != 0)
    {
        UART_PRINT("[OTA] Failed to create IP sync obj\n\r");
        return -1;
    }

    /* Initialize TCP/IP stack */
    ret = TCPIP_IF_init(otaTcpipEvent, 1);
    if (ret != 0)
    {
        UART_PRINT("[OTA] TCPIP_IF_init failed: %d\n\r", ret);
        return ret;
    }

    /* Initialize DNS */
    DNS_IF_init();

    return 0;
}

int OTA_WIFI_connect(void)
{
    char cmdBuf[CMD_BUFFER_LEN];
    int ret;
    uint8_t netIdx;
    uint8_t secType;
    WlanMacAddress_t macAddr;
    WlanFWVersions_t wlanVer = {0};
    RoleUpStaCmd_t roleParams;

    /* Drain stale signals from sync objects (e.g. disconnect event
       leaves a pending signal on gConnectSyncObj) */
    osi_SyncObjWait(&gScanSyncObj, 0);
    osi_SyncObjWait(&gConnectSyncObj, 0);
    osi_SyncObjWait(&gIpAcquiredSyncObj, 0);

    /* Set the active Wireless FW slot before Wlan_Start().
       PSA FWU determines which slot is primary after boot; tell the WiFi
       driver to use that slot.  Default is slot 1 (component ID 2). */
    {
        psa_fwu_component_info_t wfwInfo;
        WlanConnectivityFWSlot_t fwSlot;

        fwSlot.Connectivityslot = 1; /* default */
        ret = psa_fwu_query(WSOC_OR_RFTool_Slot_2, &wfwInfo);
        if (ret == PSA_SUCCESS && wfwInfo.impl.Primary)
        {
            fwSlot.Connectivityslot = 2;
        }
        Wlan_Set(WLAN_SET_PRIMARY_CONNECTIVITY_FW, &fwSlot);
    }

    /* Start WiFi driver with our event handler */
    UART_PRINT("[OTA] Starting WiFi driver...\n\r");
    ret = Wlan_Start(otaWlanEventHandler);
    if (ret != 0)
    {
        UART_PRINT("[OTA] Wlan_Start failed: %d\n\r", ret);
        return ret;
    }

    /* Register STA network interface (first time only — reuse on reconnect) */
    if (gStaNetif == NULL)
    {
        macAddr.roleType = WLAN_ROLE_STA;
        Wlan_Get(WLAN_GET_MACADDRESS, &macAddr);
        gStaNetif = TCPIP_IF_addInterface("st", macAddr.pMacAddress,
                                           otaTcpipSend,
                                           TCPIP_IF_FLAGS_DHCPC |
                                           TCPIP_IF_FLAGS_DEFAULT);
        if (gStaNetif == NULL)
        {
            UART_PRINT("[OTA] Failed to add network interface\n\r");
            goto wifi_fail;
        }
    }

    /* Print firmware version */
    os_sleep(1, 0);
    Wlan_Get(WLAN_GET_FWVERSION, &wlanVer);
    UART_PRINT("Firmware version:%d.%d.%d.%d\r\n",
                wlanVer.major_version,
                wlanVer.minor_version,
                wlanVer.api_version,
                wlanVer.build_version);
    UART_PRINT("Phy version:%d.%d.%d.%d.%d.%d\r\n",
                wlanVer.phy_version[5],
                wlanVer.phy_version[4],
                wlanVer.phy_version[3],
                wlanVer.phy_version[2],
                wlanVer.phy_version[1],
                wlanVer.phy_version[0]);

    /* Bring up STA interface and role */
    TCPIP_IF_setInterfaceState(gStaNetif, E_TCPIP_IF_UP);

    memset(&roleParams, 0, sizeof(roleParams));
    strncpy((char *)roleParams.countryDomain, "00", 2);

    UART_PRINT("[OTA] Bringing up STA role...\n\r");
    ret = Wlan_RoleUp(WLAN_ROLE_STA, &roleParams, WLAN_WAIT_FOREVER);
    if (ret < 0)
    {
        UART_PRINT("[OTA] Wlan_RoleUp failed: %d\n\r", ret);
        goto wifi_fail;
    }
    UART_PRINT("[OTA] STA role up\n\r");

    /* Scan for networks — loop allows re-scan if AP not found */
    while (1)
    {
        UART_PRINT("\n\r[OTA] Scanning for WiFi networks...\n\r");
        ret = Wlan_Scan(WLAN_ROLE_STA, NULL, WLAN_MAX_SCAN_COUNT);
        if (ret != 0)
        {
            UART_PRINT("[OTA] Wlan_Scan failed: %d\n\r", ret);
            goto wifi_fail;
        }

        /* Wait for scan results (signaled by WLAN_EVENT_SCAN_RESULT handler) */
        ret = osi_SyncObjWait(&gScanSyncObj, OTA_SCAN_TIMEOUT_MS);
        if (ret != 0 || gScanEntryCount == 0)
        {
            UART_PRINT("[OTA] No WiFi networks found\n\r");
            goto wifi_fail;
        }

        /* Prompt user to select a network, or 0 to re-scan */
        while (1)
        {
            {
                char promptBuf[64];
                snprintf(promptBuf, sizeof(promptBuf),
                         "Enter network number (1-%d), or 0 to re-scan: ",
                         (int)gScanEntryCount);
                ret = GetCmd(cmdBuf, CMD_BUFFER_LEN, promptBuf);
            }
            UART_PRINT("\n\r");
            if (ret <= 0)
            {
                UART_PRINT("[OTA] Invalid input, try again\n\r");
                continue;
            }

            netIdx = (uint8_t)atoi(cmdBuf);
            if (netIdx == 0)
            {
                break; /* break inner loop, outer loop re-scans */
            }
            if (netIdx >= 1 && netIdx <= gScanEntryCount)
            {
                break; /* valid selection */
            }
            UART_PRINT("[OTA] Invalid network number, try again\n\r");
        }
        if (netIdx == 0)
        {
            UART_PRINT("[OTA] Re-scanning...\n\r");
            continue; /* re-scan */
        }
        break; /* valid network selected */
    }

    /* Determine security type and prompt for password if needed */
    secType = getSecurityType(gScanEntries[netIdx - 1].SecurityInfo);
    if (secType != WLAN_SEC_TYPE_OPEN)
    {
        const char *secName;

        switch (secType)
        {
        case WLAN_SEC_TYPE_WPA_WPA2:
            secName = "WPA/WPA2";
            break;
        case WLAN_SEC_TYPE_WPA3:
            secName = "WPA3";
            break;
        case WLAN_SEC_TYPE_WPA2_PLUS:
            secName = "WPA2+";
            break;
        default:
            secName = "secured";
            break;
        }
        UART_PRINT("[OTA] Network is %s — password required\n\r", secName);
        ret = GetCmd(cmdBuf, CMD_BUFFER_LEN, "Enter WiFi password: ");
        UART_PRINT("\n\r");
        if (ret <= 0)
        {
            UART_PRINT("[OTA] Invalid password\n\r");
            goto wifi_fail;
        }
    }
    else
    {
        UART_PRINT("[OTA] Network is open — no password required\n\r");
        cmdBuf[0] = '\0';
    }

    /* Connect to the selected network */
    UART_PRINT("\n\r[OTA] Connecting to %.*s...\n\r",
                gScanEntries[netIdx - 1].SsidLen,
                gScanEntries[netIdx - 1].Ssid);

    gConnected = 0;
    ret = Wlan_Connect(
        (const signed char *)gScanEntries[netIdx - 1].Ssid,
        gScanEntries[netIdx - 1].SsidLen,
        NULL,
        secType,
        cmdBuf,
        strlen(cmdBuf),
        0);
    if (ret < 0)
    {
        UART_PRINT("[OTA] Wlan_Connect failed: %d\n\r", ret);
        goto wifi_fail;
    }

    /* Wait for WLAN_EVENT_CONNECT */
    ret = osi_SyncObjWait(&gConnectSyncObj, OTA_CONNECT_TIMEOUT_MS);
    if (ret != 0 || !gConnected)
    {
        UART_PRINT("[OTA] WiFi connection timeout\n\r");
        goto wifi_fail;
    }

    /* Wait for IP address (DHCP) */
    UART_PRINT("[OTA] Waiting for IP address...\n\r");
    ret = osi_SyncObjWait(&gIpAcquiredSyncObj, OTA_IP_TIMEOUT_MS);
    if (ret != 0)
    {
        UART_PRINT("[OTA] IP acquisition timeout\n\r");
        goto wifi_fail;
    }

    UART_PRINT("[OTA] WiFi connected with IP\n\r");
    return 0;

wifi_fail:
    Wlan_Disconnect(WLAN_ROLE_STA, NULL);
    osi_SyncObjWait(&gConnectSyncObj, OTA_CONNECT_TIMEOUT_MS);
    if (gStaNetif != NULL)
    {
        TCPIP_IF_setInterfaceState(gStaNetif, E_TCPIP_IF_DOWN);
    }
    os_sleep(1, 0);
    Wlan_Stop(FALSE);
    gConnected = 0;
    return -1;
}

int OTA_WIFI_isConnected(void)
{
    return gConnected ? 1 : 0;
}

void OTA_WIFI_disconnectAndStop(void)
{
    if (gConnected)
    {
        UART_PRINT("[OTA] Disconnecting WiFi...\n\r");
        Wlan_Disconnect(WLAN_ROLE_STA, NULL);
        osi_SyncObjWait(&gConnectSyncObj, OTA_CONNECT_TIMEOUT_MS);
    }

    /* Set interface down so lwIP stops processing events for it */
    if (gStaNetif != NULL)
    {
        TCPIP_IF_setInterfaceState(gStaNetif, E_TCPIP_IF_DOWN);
    }

    /* Let the lwIP tcpip thread finish processing link-down and
       interface-down callbacks before tearing down the driver */
    os_sleep(1, 0);

    Wlan_Stop(FALSE);
    gConnected = 0;
}
