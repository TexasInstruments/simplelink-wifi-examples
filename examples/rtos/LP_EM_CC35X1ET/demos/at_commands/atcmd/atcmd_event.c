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

//*****************************************************************************
// includes
//*****************************************************************************
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <unistd.h>
#include <mqueue.h>

#include "wlan_if.h"
#include "lwip/netif.h"

#include "str_mpl.h"

/* AT Header files */
#include "atcmd_defs.h"
#include "atcmd_event.h"
#include "atcmd.h"
#include "atcmd_gen.h"

#include "network_terminal.h"
#include "nimble_host_at_commands.h"
#include "host/ble_hs.h"
#include "lwip_ping.h"

//*****************************************************************************
// defines
//*****************************************************************************

//*****************************************************************************
// typedefs
//*****************************************************************************

//*****************************************************************************
// externs
//*****************************************************************************
extern appControlBlock app_CB;

/******************************************************************************
// AT Events Routines
******************************************************************************/

/*!
    \brief          Compose event.

    \param          event      -   Points to event struct dat
    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdEvent_compose(void *event, uint32_t size, int32_t (*callback)(void *, int32_t, char *), int32_t num)
{
    void *params;
    ATCmd_Event_t eventMsg;

    params = malloc(size);
    if (params == NULL)
    {
        return -1;
    }
    memcpy(params, event, size);

    eventMsg.callback = callback;
    eventMsg.args = params;
    eventMsg.num = num;
    ATCmd_signalEvent(&eventMsg);  

    return 0;

}

/*!
    \brief          Fatal Error Event.

    This routine send Fatal Error event to host as AT event.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdEvent_fatalErrorCallback(void *args, int32_t num, char *buff)
{
    int32_t ret = 0;
#if 0
    SlDeviceFatal_t  *params = (SlDeviceFatal_t *)(args);

    StrMpl_setStr(ATCmd_eventFatalErrorStr,&buff,ATCMD_DELIM_EVENT);

    StrMpl_setListStr(ATCmd_eventDeviceId, sizeof(ATCmd_eventDeviceId)/sizeof(StrMpl_List_t), &params->Id,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_32|STRMPL_FLAG_PARAM_UNSIGNED);
    switch (params->Id)
    {
        case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
            /* code */
            StrMpl_setVal(&params->Data.DeviceAssert.Code,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* value */
            StrMpl_setVal(&params->Data.DeviceAssert.Value,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
	    case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
            break;
	    case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
            /* code */
            StrMpl_setVal(&params->Data.NoCmdAck.Code,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
	    case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
            /* code */
            StrMpl_setVal(&params->Data.CmdTimeout.Code,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        default:
            ret = -1;
            break;
    }

    free (params);
#endif
    return ret;
}

/*!
    \brief          Device General Event.

    This routine send device general event to host as AT event.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdEvent_deviceGeneralCallback(void *args, int32_t num, char *buff)
{
    int32_t ret = 0;
#if 0
    SlDeviceEvent_t  *params = (SlDeviceEvent_t *)(args);

    if ((params->Id == SL_DEVICE_EVENT_RESET_REQUEST) || (params->Id == SL_DEVICE_EVENT_ERROR))
    {
        StrMpl_setStr(ATCmd_eventGeneralStr,&buff,ATCMD_DELIM_EVENT);
        StrMpl_setListStr(ATCmd_eventDeviceId, sizeof(ATCmd_eventDeviceId)/sizeof(StrMpl_List_t), &params->Id,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_32|STRMPL_FLAG_PARAM_UNSIGNED);
        StrMpl_setVal(&params->Data.Error.Code,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_16 | STRMPL_FLAG_PARAM_SIGNED);
        StrMpl_setListStr(ATCmd_eventDeviceSender, sizeof(ATCmd_eventDeviceSender)/sizeof(StrMpl_List_t), &params->Data.Error.Source,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_8|STRMPL_FLAG_PARAM_UNSIGNED);
    }
    else
    {
        ret = -1;
    }
    free (params);
#endif
    return ret;
}

int32_t ATCmdEvent_pingStopCallback(void *args, int32_t num, char *buff)
{
    int32_t ret = 0;
    ping_results_report_t *results = (ping_results_report_t *)args;
    uint8_t ip[4];
    uint32_t packetLossPercent = 0;

    StrMpl_setStr(ATCmd_eventPingReportStr, &buff, ATCMD_DELIM_EVENT);

    /* Target IP */
    ATCmd_valToIPv4(htonl(results->target_ip.addr), ip);
    StrMpl_setArrayVal(ip, &buff, IPV4_ADDR_LEN,
                       ATCMD_DELIM_ARG, ATCMD_DELIM_INTER,
                       STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 |
                       STRMPL_FLAG_PARAM_UNSIGNED);


    /* Session ID */
    StrMpl_setVal(&results->session_id, &buff,
                  ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_16 |
                  STRMPL_FLAG_PARAM_DEC |
                  STRMPL_FLAG_PARAM_UNSIGNED);

    /* Packets sent */
    StrMpl_setVal(&results->packets_sent, &buff,
                  ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_32 |
                  STRMPL_FLAG_PARAM_DEC |
                  STRMPL_FLAG_PARAM_UNSIGNED);

    /* Packets received */
    StrMpl_setVal(&results->packets_received, &buff,
                  ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_32 |
                  STRMPL_FLAG_PARAM_DEC |
                  STRMPL_FLAG_PARAM_UNSIGNED);

    /* Packets loss % */
    packetLossPercent = results->packets_sent > 0 ?
                        (results->packets_sent - results->packets_received) * 100 / results->packets_sent : 0;
    StrMpl_setVal(&packetLossPercent, &buff,
                  ATCMD_DELIM_TRM,
                  STRMPL_FLAG_PARAM_SIZE_32 |
                  STRMPL_FLAG_PARAM_DEC |
                  STRMPL_FLAG_PARAM_UNSIGNED);

    return ret;
}

/*!
    \brief          lwip async Event.

    This routine send lwip event to host as AT event.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdEvent_networkCallback(void *args, int32_t num, char *buff)
{
    struct netif *pNetIf = (struct netif *)args;
    int32_t ret = 0;
    uint8_t ip[4];

    StrMpl_setStr(ATCmd_eventNetworkStr, &buff, ATCMD_DELIM_EVENT);

    StrMpl_setStr("ipv4_acquired", &buff, ATCMD_DELIM_ARG);

    /* Address */
    ATCmd_valToIPv4(htonl(pNetIf->ip_addr.addr), ip);
    StrMpl_setArrayVal(ip, &buff, IPV4_ADDR_LEN,
                       ATCMD_DELIM_ARG, ATCMD_DELIM_INTER,
                       STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | \
                       STRMPL_FLAG_PARAM_UNSIGNED);

    /* Netmask */
    ATCmd_valToIPv4(htonl(pNetIf->netmask.addr), ip);
    StrMpl_setArrayVal(ip, &buff, IPV4_ADDR_LEN,
                       ATCMD_DELIM_ARG, ATCMD_DELIM_INTER,
                       STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | \
                       STRMPL_FLAG_PARAM_UNSIGNED);

    /* Gateway */
    ATCmd_valToIPv4(htonl(pNetIf->gw.addr), ip);
    StrMpl_setArrayVal(ip, &buff, IPV4_ADDR_LEN, 
                       ATCMD_DELIM_TRM, ATCMD_DELIM_INTER,
                       STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | \
                       STRMPL_FLAG_PARAM_UNSIGNED);


    return ret;
}


/*!
    \brief          wlan async Event.

    This routine send wlan event to host as AT event.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdEvent_wlanCallback(void *args, int32_t num, char *buff)
{
    int32_t ret = 0;
    WlanEvent_t  *pWlanEvent = (WlanEvent_t *)(args);

    StrMpl_setStr(ATCmd_eventWlanStr, &buff, ATCMD_DELIM_EVENT);

    StrMpl_setListStr(ATCmd_eventWlanId, sizeof(ATCmd_eventWlanId) / sizeof(StrMpl_List_t),
                      &pWlanEvent->Id, &buff, 
                      ATCMD_DELIM_ARG, STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);

    switch (pWlanEvent->Id)
    {
        case WLAN_EVENT_CONNECT:
        {
            WlanEventConnect_t *pWlanEventConnect = &pWlanEvent->Data.Connect;

            char ssid[WLAN_SSID_MAX_LENGTH + 1];
            char bssid[WLAN_BSSID_LENGTH + 1];

            if (pWlanEventConnect->Status < 0)
            {
                /* Set error status */
                int8_t status = -1;
                StrMpl_setVal(&status, &buff, ATCMD_DELIM_TRM,
                              STRMPL_FLAG_PARAM_SIZE_8 |
                              STRMPL_FLAG_PARAM_DEC |
                              STRMPL_FLAG_PARAM_SIGNED);
                break;
            }

            os_memset(ssid, 0, sizeof(ssid));
            os_memcpy(ssid, pWlanEventConnect->SsidName, pWlanEventConnect->SsidLen);
            os_memset(bssid, 0, sizeof(bssid));
            os_memcpy(bssid, pWlanEventConnect->Bssid, WLAN_BSSID_LENGTH);

            /* ssid */
            StrMpl_setStr((char *)ssid, &buff, ATCMD_DELIM_ARG);

            /* bssid */
            StrMpl_setArrayVal(bssid, &buff, WLAN_BSSID_LENGTH, ATCMD_DELIM_TRM, ATCMD_DELIM_ARRAY,
                               STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 |
                               STRMPL_FLAG_PARAM_UNSIGNED | STRMPL_FLAG_PARAM_NO_HEX_PREFIX);

            break;
        }

        case WLAN_EVENT_DISCONNECT:
        {
            WlanEventDisconnect_t *pWlanEventDisconnect = &pWlanEvent->Data.Disconnect;

            /* ssid */
            StrMpl_setStr((char *)app_CB.CON_CB.ConnectionSSID, &buff, ATCMD_DELIM_ARG);

            /* bssid */
            StrMpl_setArrayVal(app_CB.CON_CB.ConnectionBSSID, &buff, WLAN_BSSID_LENGTH,
                               ATCMD_DELIM_ARG,ATCMD_DELIM_ARRAY,
                               STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 |
                               STRMPL_FLAG_PARAM_UNSIGNED | STRMPL_FLAG_PARAM_NO_HEX_PREFIX);
            
            /* Reason */
            StrMpl_setVal(&pWlanEventDisconnect->ReasonCode, &buff, ATCMD_DELIM_TRM,
                          STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_DEC |
                          STRMPL_FLAG_PARAM_UNSIGNED);

            memset(&(app_CB.CON_CB.ConnectionSSID), 0x0,
                   sizeof(app_CB.CON_CB.ConnectionSSID));
            memset(&(app_CB.CON_CB.ConnectionBSSID), 0x0,
                   sizeof(app_CB.CON_CB.ConnectionBSSID));

            break;
        }

        case WLAN_EVENT_SCAN_RESULT:
        {
            WlanEventScanResult_t *pWlanEventScan = &pWlanEvent->Data.ScanResult;
            uint32_t index = num;
            uint8_t param;

            char ssid[WLAN_SSID_MAX_LENGTH + 1];
            char bssid[WLAN_BSSID_LENGTH + 1];

            WlanNetworkEntry_t *pNetworkListResultEntry;

            /* +wlanscan: */
            //StrMpl_setStr(ATCmd_wlanScanStr, &buff, ATCMD_DELIM_EVENT);

            pNetworkListResultEntry = &(pWlanEventScan->NetworkListResult[index]);
            os_memset(ssid, 0, sizeof(ssid));
            os_memcpy(ssid, pNetworkListResultEntry->Ssid, pNetworkListResultEntry->SsidLen);
            os_memset(bssid, 0, sizeof(bssid));
            os_memcpy(bssid, pNetworkListResultEntry->Bssid, WLAN_BSSID_LENGTH);

            /* SSID */
            StrMpl_setStr(ssid, &buff, ATCMD_DELIM_ARG);

            /* BSSID */
            StrMpl_setArrayVal(bssid ,&buff, WLAN_BSSID_LENGTH, ATCMD_DELIM_ARG, ATCMD_DELIM_ARRAY,
                               STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 |
                               STRMPL_FLAG_PARAM_UNSIGNED | STRMPL_FLAG_PARAM_NO_HEX_PREFIX);

            /* RSSI */
            StrMpl_setVal(&(pNetworkListResultEntry->Rssi), &buff, ATCMD_DELIM_ARG,
                          STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIGNED);

            /* Channel */
            StrMpl_setVal(&(pNetworkListResultEntry->Channel), &buff, ATCMD_DELIM_ARG, 
                          sizeof(pNetworkListResultEntry->Channel) | STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_UNSIGNED);

            /* Security_Type */
            param = WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(pNetworkListResultEntry->SecurityInfo);
            param &= SECURITY_TYPE_MASK;
            StrMpl_setListStr(ATCmd_wlanScanSecurity,
                              sizeof(ATCmd_wlanScanSecurity) / sizeof(StrMpl_List_t),
                              (void *)&param, &buff, ATCMD_DELIM_ARG,
                              STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);

            /* PMF */
            param = WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(pNetworkListResultEntry->SecurityInfo);
            if (param & SECURITY_TYPE_BITMAP_PMF_REQUIRED)
            {
                StrMpl_setStr("required", &buff, ATCMD_DELIM_ARG);
            }
            else if (param & SECURITY_TYPE_BITMAP_PMF_CAPABLE)
            {
                StrMpl_setStr("capable", &buff, ATCMD_DELIM_ARG);
            }
            else
            {
                StrMpl_setStr("disabled", &buff, ATCMD_DELIM_ARG);
            }

            /* Hidden_SSID */
            param = ((0 == pNetworkListResultEntry->SsidLen) ||
                     (0 == pNetworkListResultEntry->Ssid[0])) ?
                     1 : 0;
            StrMpl_setVal(&param, &buff, ATCMD_DELIM_LIST,
                          STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);

            /* Null terminate the atcmd event scan entry response */
            buff++;
            *buff = ATCMD_DELIM_TRM;

            break;
        }
        case WLAN_EVENT_ADD_PEER:
        {
            uint8_t *macAddr = pWlanEvent->Data.AddPeer.Mac;

            StrMpl_setArrayVal(macAddr ,&buff, WLAN_BSSID_LENGTH, ATCMD_DELIM_TRM, ATCMD_DELIM_ARRAY,
                               STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 |
                               STRMPL_FLAG_PARAM_UNSIGNED | STRMPL_FLAG_PARAM_NO_HEX_PREFIX);
        }
            break;
        case WLAN_EVENT_REMOVE_PEER:
        {
            uint8_t *macAddr = pWlanEvent->Data.RemovePeer.Mac;

            StrMpl_setArrayVal(macAddr ,&buff, WLAN_BSSID_LENGTH, ATCMD_DELIM_TRM, ATCMD_DELIM_ARRAY,
                               STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 |
                               STRMPL_FLAG_PARAM_UNSIGNED | STRMPL_FLAG_PARAM_NO_HEX_PREFIX);
        }
            break;
        default:
            ret = -1;
            break;
    }

    if (args)
    {
        /* args pointer was malloced when the event was sent */
        free(args);
    }

    return ret;
}

#if 0
/*!
    \brief          socket async Event.

    This routine send socket event to host as AT event.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdEvent_sockCallback(void *args, int32_t num, char *buff)
{
    int32_t ret = 0;
    SlSockEvent_t  *params = (SlSockEvent_t *)(args);

    StrMpl_setStr(ATCmd_eventSockStr,&buff,ATCMD_DELIM_EVENT);

    StrMpl_setListStr(ATCmd_eventSockId, sizeof(ATCmd_eventSockId)/sizeof(StrMpl_List_t), &params->Event,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_32|STRMPL_FLAG_PARAM_UNSIGNED);
    switch (params->Event)
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            /* sd */
            StrMpl_setVal(&params->SocketAsyncEvent.SockTxFailData.Sd,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_8|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_UNSIGNED);
            /* status */
            StrMpl_setVal(&params->SocketAsyncEvent.SockTxFailData.Status,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_16|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_SIGNED);
            break;
        case SL_SOCKET_ASYNC_EVENT:
            /* sd */
            StrMpl_setVal(&params->SocketAsyncEvent.SockAsyncData.Sd,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_8|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_UNSIGNED);
            /* type */
            StrMpl_setListStr(ATCmd_eventSockType, sizeof(ATCmd_eventSockType)/sizeof(StrMpl_List_t), &params->SocketAsyncEvent.SockAsyncData.Type,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_8|STRMPL_FLAG_PARAM_UNSIGNED);
            /* val */
            StrMpl_setVal(&params->SocketAsyncEvent.SockAsyncData.Val,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_16|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_SIGNED);
            break;
        default:
            ret = -1;
            break;
    }

    free (params);
    return ret;
}

/*!
    \brief          netapp async Event.

    This routine send netapp event to host as AT event.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdEvent_netappCallback(void *args, int32_t num, char *buff)
{
    int32_t ret = 0;
    SlNetAppEvent_t  *params = (SlNetAppEvent_t *)(args);
    uint8_t ip[4];

    StrMpl_setStr(ATCmd_eventNetappStr,&buff,ATCMD_DELIM_EVENT);

    StrMpl_setListStr(ATCmd_eventNetappId, sizeof(ATCmd_eventNetappId)/sizeof(StrMpl_List_t), &params->Id,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_32|STRMPL_FLAG_PARAM_UNSIGNED);
    switch (params->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
            /* address */
            ATCmd_valToIPv4(params->Data.IpAcquiredV4.Ip, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Gateway */
            ATCmd_valToIPv4(params->Data.IpAcquiredV4.Gateway, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* DNS */
            ATCmd_valToIPv4(params->Data.IpAcquiredV4.Dns, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_TRM,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        case SL_NETAPP_EVENT_IPV6_ACQUIRED:
            /* address */
            StrMpl_setArrayVal(params->Data.IpAcquiredV6.Ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* dns */
            StrMpl_setArrayVal(params->Data.IpAcquiredV6.Dns,&buff,4,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        case SL_NETAPP_EVENT_IP_COLLISION:
            /* address */
            ATCmd_valToIPv4(params->Data.IpCollision.IpAddress, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* DHCP mac */
            StrMpl_setArrayVal(params->Data.IpCollision.DhcpMac,&buff,SL_WLAN_BSSID_LENGTH,ATCMD_DELIM_ARG,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Conflict mac */
            StrMpl_setArrayVal(params->Data.IpCollision.ConflictMac,&buff,SL_WLAN_BSSID_LENGTH,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
    	case SL_NETAPP_EVENT_DHCPV4_LEASED:
            /* address */
            ATCmd_valToIPv4(params->Data.IpLeased.IpAddress, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* lease time */
            StrMpl_setVal(&params->Data.IpLeased.LeaseTime,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_32|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_UNSIGNED);
            /* bssid */
            StrMpl_setArrayVal(params->Data.IpLeased.Mac,&buff,SL_WLAN_BSSID_LENGTH,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
    	case SL_NETAPP_EVENT_DHCPV4_RELEASED:
            /* address */
            ATCmd_valToIPv4(params->Data.IpReleased.IpAddress, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* bssid */
            StrMpl_setArrayVal(params->Data.IpReleased.Mac,&buff,SL_WLAN_BSSID_LENGTH,ATCMD_DELIM_ARG,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Reason */
            StrMpl_setVal(&params->Data.IpReleased.Reason,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_16|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_UNSIGNED);
            break;
    	case SL_NETAPP_EVENT_IPV4_LOST:
            /* status */
            StrMpl_setVal(&params->Data.IpV4Lost.Status,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_16|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_SIGNED);
            break;
    	case SL_NETAPP_EVENT_DHCP_IPV4_ACQUIRE_TIMEOUT:
            /* status */
            StrMpl_setVal(&params->Data.DhcpIpAcquireTimeout.Status,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_16|STRMPL_FLAG_PARAM_DEC|STRMPL_FLAG_PARAM_SIGNED);
            break;
    	case SL_NETAPP_EVENT_IPV6_LOST:
            /* ip lost */
            StrMpl_setArrayVal(params->Data.IpV6Lost.IpLost,&buff,4,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        default:
            ret = -1;
            break;
    }

    free (params);
    return ret;
}
#endif


/*!
    \brief          BLE async Event.

    This routine send BLE event to host as AT event.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdEvent_bleCallback(void *args, int32_t num, char *buff)
{
    int32_t ret = 0;
    ATCmd_BleEvent_t *bleEvent = (ATCmd_BleEvent_t *)args;
    uint8_t eventTypeDelim = ATCMD_DELIM_ARG;

    /* For events with no arguments, use the terminator delimiter */
    if (bleEvent->eventId == ATCMD_BLE_EVENT_SCAN_COMPLETE)
    {
        eventTypeDelim = ATCMD_DELIM_TRM;
    }

    StrMpl_setStr(ATCmd_eventBleStr, &buff, ATCMD_DELIM_EVENT);

    StrMpl_setListStr(ATCmd_eventBleId,
                      sizeof(ATCmd_eventBleId) / sizeof(StrMpl_List_t),
                      &bleEvent->eventId, &buff, eventTypeDelim,
                      STRMPL_FLAG_PARAM_SIZE_16 | STRMPL_FLAG_PARAM_UNSIGNED);

    switch (bleEvent->eventId)
    {
        case ATCMD_BLE_EVENT_SCAN_RESULT:
        {
            scanResEntry_t *pEntry = (scanResEntry_t *)bleEvent->eventArgs;

            /* MAC address */
            StrMpl_setArrayVal(pEntry->addr.val, &buff, BLE_DEV_ADDR_LEN,
                               ATCMD_DELIM_ARG, ATCMD_DELIM_ARRAY,
                               STRMPL_FLAG_PARAM_HEX |
                               STRMPL_FLAG_PARAM_SIZE_8 |
                               STRMPL_FLAG_PARAM_UNSIGNED | 
                               STRMPL_FLAG_PARAM_NO_HEX_PREFIX);

            /* RSSI */
            StrMpl_setVal(&pEntry->rssi, &buff, ATCMD_DELIM_ARG,
                          STRMPL_FLAG_PARAM_SIZE_8 |
                          STRMPL_FLAG_PARAM_DEC |
                          STRMPL_FLAG_PARAM_SIGNED);
            
            /* own address type */
            if (pEntry->addr.type == BLE_OWN_ADDR_PUBLIC)
            {
                StrMpl_setStr("public", &buff, ATCMD_DELIM_ARG);
            }
            else if (pEntry->addr.type == BLE_OWN_ADDR_RANDOM)
            {
                StrMpl_setStr("random", &buff, ATCMD_DELIM_ARG);
            }
            else
            {
                StrMpl_setStr("unknown", &buff, ATCMD_DELIM_ARG);
            }
            
            /* Name */
            StrMpl_setStr(pEntry->local_name, &buff, ATCMD_DELIM_TRM);

            os_free(bleEvent->eventArgs);
        }
            break;
        case ATCMD_BLE_EVENT_SCAN_COMPLETE:
            break;
        case ATCMD_BLE_EVENT_CONNECT:
        {
            ATCmd_BleConnectEvent_t *pEvent = (ATCmd_BleConnectEvent_t *)bleEvent->eventArgs;

            if (pEvent->status == 0)
            {
                /* Connection handle */
                StrMpl_setVal(&pEvent->connectionHandle, &buff,
                              ATCMD_DELIM_ARG,
                              STRMPL_FLAG_PARAM_SIZE_16 |
                              STRMPL_FLAG_PARAM_DEC |
                              STRMPL_FLAG_PARAM_UNSIGNED);

                /* MAC address */
                StrMpl_setArrayVal(pEvent->peerAddress, &buff,
                                   BLE_DEV_ADDR_LEN,
                                   ATCMD_DELIM_ARG, ATCMD_DELIM_ARRAY,
                                   STRMPL_FLAG_PARAM_HEX |
                                   STRMPL_FLAG_PARAM_SIZE_8 |
                                   STRMPL_FLAG_PARAM_UNSIGNED |
                                   STRMPL_FLAG_PARAM_NO_HEX_PREFIX);
                
                /* Peer address type */
                if (pEvent->peerAddressType == BLE_ADDR_PUBLIC)
                {
                    StrMpl_setStr("public", &buff, ATCMD_DELIM_TRM);
                }
                else if (pEvent->peerAddressType == BLE_ADDR_RANDOM)
                {
                    StrMpl_setStr("random", &buff, ATCMD_DELIM_TRM);
                }
                else
                {
                    StrMpl_setStr("unknown", &buff, ATCMD_DELIM_TRM);
                }
            }
            else
            {
                /* Set error status */
                int8_t status = -1;
                StrMpl_setVal(&status, &buff, ATCMD_DELIM_TRM,
                              STRMPL_FLAG_PARAM_SIZE_8 |
                              STRMPL_FLAG_PARAM_DEC |
                              STRMPL_FLAG_PARAM_SIGNED);
            }

            os_free(bleEvent->eventArgs);
        }
            break;
        case ATCMD_BLE_EVENT_DISCONNECT:
        {
            ATCmd_BleDisconnectEvent_t *pEvent = (ATCmd_BleDisconnectEvent_t *)bleEvent->eventArgs;

            /* Connection handle */
            StrMpl_setVal(&pEvent->connectionHandle, &buff,
                          ATCMD_DELIM_ARG,
                          STRMPL_FLAG_PARAM_SIZE_16 |
                          STRMPL_FLAG_PARAM_DEC |
                          STRMPL_FLAG_PARAM_UNSIGNED);

            /* MAC address */
            StrMpl_setArrayVal(pEvent->peerAddress, &buff,
                               BLE_DEV_ADDR_LEN,
                               ATCMD_DELIM_ARG, ATCMD_DELIM_ARRAY,
                               STRMPL_FLAG_PARAM_HEX |
                               STRMPL_FLAG_PARAM_SIZE_8 |
                               STRMPL_FLAG_PARAM_UNSIGNED |
                               STRMPL_FLAG_PARAM_NO_HEX_PREFIX);
            
            /* Peer address type */
            if (pEvent->peerAddressType == BLE_ADDR_PUBLIC)
            {
                StrMpl_setStr("public", &buff, ATCMD_DELIM_TRM);
            }
            else if (pEvent->peerAddressType == BLE_ADDR_RANDOM)
            {
                StrMpl_setStr("random", &buff, ATCMD_DELIM_TRM);
            }
            else
            {
                StrMpl_setStr("unknown", &buff, ATCMD_DELIM_TRM);
            }

            os_free(bleEvent->eventArgs);
        }
            break;
        case ATCMD_BLE_EVENT_ADV_COMPLETE:
            /* Set termination code. 0 for success */
            StrMpl_setVal(&num, &buff,
                          ATCMD_DELIM_TRM,
                          STRMPL_FLAG_PARAM_SIZE_32 |
                          STRMPL_FLAG_PARAM_DEC |
                          STRMPL_FLAG_PARAM_SIGNED);
            break;
        default:
            ret = -1;
            break;
    }

    /* Allocated in ATCmdEvent_compose() */
    os_free(bleEvent);
    return ret;
}
