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
/* This example demonstrates the McSPI RX and TX operation configured
 * in blocking, interrupt mode of operation.
 *
 * This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
 * and then receives the same in RX mode. Internal pad level loopback mode
 * is enabled to receive data.
 * To enable internal pad level loopback mode, D0 pin is configured to both
 * TX Enable as well as RX input pin in the SYSCFG.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */

#include <stdlib.h>
#include "FreeRTOS.h"

//SOCKET
#include "socket_examples.h"
#include "lwip_iperf_examples.h"

/* Board Header files */
#include "ti_drivers_config.h"
#ifdef CC33XX
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#endif // CC33XX

/* Example Header files */
#include "cmd_parser.h"
#include "wlan_cmd.h"
#include "ble_cmd.h"
#include "wlan_if.h"
#include "uart_term.h"

//LWIP
#include "network_lwip.h"

//ERRORS
#include "errors.h"

#include "osi_kernel.h"
#include "network_terminal.h"

#include "date_time_service.h"
#ifdef SNTP_SUPPORT
#include "sntp_wrapper.h"
#endif
// Debug for total allocation
#ifdef PRINT_DBG_TOTAL_MALLOC_FREE
extern volatile UINT32 totalloc;
#endif

#ifdef CC35XX
#define WLAN_REASON_DEAUTH_LEAVING 3
#define WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY 4
extern OsiSyncObj_t p2p_find_stopped_syncObj;
extern Bool_e g_wait_p2p_scan_complete;

void initialize_mbedtls_threading(); //define prototype

#endif // CC35XX

/* Application defines */

#define APP_MCSPI_MSGSIZE       (100U)
#define CC33XX_MAX_FW_LOGS_BUFFER_SIZE    (4096U)

#define APPLICATION_NAME        ("Quick Track")
/****************************************************************************
                      LOCAL FUNCTION PROTOTYPES
****************************************************************************/
int32_t showAvailableCmd();
int32_t cmd_prompt(void *arg);
int32_t cmdClearcallback(void *arg);
int32_t printClearUsage(void *arg);
int32_t cmdHelpCallback(void *arg);
int32_t printHelpUsage(void *arg);
int32_t initAppVariables();
#ifdef SNTP_SUPPORT
int32_t cmdSntpConfigServers(void *arg);
int32_t printSntpConfigServersUsage(void *arg);
int32_t cmdSntpUpdateDateTime(void *arg);
int32_t printSntpUpdateDateTimeUsage(void *arg);
int32_t ParseSntpConfigServersCmd(void *arg, uint32_t *pNumOfServers,char* serverIp[]);
#endif
int32_t cmdSetDateTime(void *arg);
int32_t cmdGetDateTime(void *arg);
int32_t printSetDateTimeUsage(void *arg);
int32_t printGetDateTimeUsage(void *arg);
int32_t ParseSetDateTimeCmd(void *arg, uint32_t* pYear, uint32_t* pMonth,
        uint32_t* pDay, uint32_t* pHour, uint32_t* pMinute,
        uint32_t* pSecond);

extern uint32_t ActiveNetIfBitMap;

/****************************************************************************
                      GLOBAL VARIABLES
****************************************************************************/

/*    Command List :
 *
 *    Upon calling 'cmd_prompt()', for every command line the user enters,
 *    This Table gets checked for the appropriate command column,
 *    If command was found, cmd_prompt would dispatch the command callback.
 */

cmdAction_t gCmdList[] =
{
/* command */         /* Command callback */        /* Print Usage */

/* Show help */
{ helpStr,              cmdHelpCallback,            printHelpUsage              },

/* Clear term */
{ clearStr,             cmdClearcallback,           printClearUsage             },

/* Wlan RoleUpAp */
{ wlanRoleUpApStr,      cmdWlanRoleUpApCallback,    printWlanRoleUpApUsage      },

/* Wlan RoleDwonAp */
{ wlanRoleDownApStr,    cmdWlanRoleDownApCallback,  printWlanRoleDownApUsage    },

/* Wlan RoleUpSta */
{ wlanRoleUpStaStr,     cmdWlanRoleUpStaCallback,   printWlanRoleUpStaUsage     },

/* Wlan RoleDwonSta */
{ wlanRoleDownStaStr,   cmdWlanRoleDownStaCallback, printWlanRoleDownStaUsage   },

/* Wlan connect */
{ wlanConnectStr,       cmdWlanConnectCallback,     printWlanConnectUsage       },

///* Wlan set early termination */
//{ wlanSetEarlyTermStr,       cmdWlanSetScanEarlyTerminationCallback,     printSetEarlyTermUsage       },
//
///* Wlan get early termination */
//{ wlanGetEarlyTermStr,       cmdWlanGetScanEarlyTerminationCallback,     printGetEarlyTermUsage       },

/* Wlan disconnect */
{ wlanDisconnectStr,    cmdWlanDisconnectCallback,  printWlanDisconnectUsage    },

/* Scan */
{ scanStr,              cmdScanCallback,            printScanUsage              },

/* Get MacAddress */
{ GetMacAddressStr,     cmdGetMacAddressCallback,   printGetMacAddressUsage     },

/* Set MacAddress */
{ SetMacAddressStr,     cmdSetMacAddressCallback,   printSetMacAddressUsage     },

// /* Get Power save mode */
// { GetPsModeStr,         cmdGetPsModeCallback,       printGetPsModeUsage         },

// /* Set Power save mode */
// { SetPsModeStr,         cmdSetPsModeCallback,       printSetPsModeUsage         },

/* Set Power management  mode     */
{ SetPmModeStr,         cmdSetPmModeCallback,       printSetPmModeUsage         },

/* Set Interface IP mode */
{ SetInterfaceIpStr,     cmdSetInterfaceIpCallback, printSetInterfaceIpUsage    },

/* Get Interface IP mode */
{ GetInterfaceIpStr,     cmdGetInterfaceIpCallback, printGetInterfaceIpUsage    },

/* Set DHCP Server mode */
{ SetDhcpServerStr,     cmdSetDhcpServerCallback,   printSetDhcpServerUsage    },

/* Get DHCP Server mode */
{ GetDhcpServerStr,     cmdGetDhcpServerCallback,   printGetDhcpServerUsage    },

// #ifdef CC35XX
// /* Sleep Test */
// { SleepTestStr,         cmdSleepTestCallback,       printSleepTestUsage         },
// #endif

// /* Set Long sleep interval     */
// { SetLsiStr,                 cmdSetLsiCallback,      printSetLsiUsage      },

/* Get FW version */
{ GetFwVerStr,          cmdGetFwVerCallback,            printGetFwVerUsage     },

/* wlan Start*/
{ wlanStartStr,         cmdWlanStartCallback,       printWlanStartUsage         },

/* wlan Stop */
{ wlanStopStr,          cmdWlanStopCallback,        printWlanStopUsage          },

#ifndef CC35XX

/* Send */
{ sendStr,              cmdSendCallback,               printSendUsage          },

/* Recv */
{ recvStr,              cmdRecvCallback,               printRecvUsage          },

/* Show available running thread */
{ showStr,              cmdShowCallback,            printShowUsage              },

/* Kill available running thread */
{ killStr,              cmdKillCallback,            printKillUsage              },

#endif
// #ifdef CC35XX

// { TestIperf,             cmdTestIperfCallback,           printTestIperfUsage      },

// { StopTestIperf,         cmdStopTestIperfCallback,       printStopTestIperfUsage  },

// #endif


// /* ble AdvCfg */
// { bleAdvCfgStr,         cmdBleAdvCfgCallback,       printBleAdvCfgUsage         },

// /* ble AdvEnable */
// { bleAdvEnableStr,      cmdBleAdvEnableCallback,    printBleAdvEnableUsage      },

// /* ble ScanCfg */
// { bleScanCfgStr,        cmdBleScanCfgCallback,      printBleScanCfgUsage        },

// /* ble ScanEnable */
// { bleScanEnableStr,     cmdBleScanEnableCallback,   printBleScanEnableUsage     },

// /* ble Connect */
// { bleConnectStr,        cmdBleConnectCallback,      printBleConnectUsage        },

// /* ble Disconnect */
// { bleDisconnectStr,     cmdBleDisconnectCallback,   printBleDisconnectUsage     },

// /* ble Peers */
// { blePeersStr,          cmdBlePeersCallback,        printBlePeersUsage          },

// /* ble Delete Bonds */
// { bleDeleteBondsStr,    cmdBleDeleteBondsCallback,  printBleDeleteBondsUsage    },

///* ble Get BD address */
//{ bleGetBdAddrStr,      cmdBleGetBdAddrCallback,    printBleGetBdAddrUsage      },
//
///* ble Set BD address */
//{ bleSetBdAddrStr,      cmdBleSetBdAddrCallback,    printBleSetBdAddrUsage      },

///* ble Start*/
//{ bleStartStr,          cmdBleStartCallback,        printBleStartUsage          },

// /* ble Stop*/
// { bleStopStr,           cmdBleStopCallback,         printBleStopUsage           },

// /* Set Ble Test Mode*/
// { bleTestModeStr,       cmdBleTestModeCallback,     printBleTestModeUsage       },

// #ifdef CC35XX
// /*------------------ CSI ---------------------------*/
// /* csi enable */
// { csiEnableStr,            cmdCsiEnableCallback,       printCsiEnableUsage      },

// /* csi stop */
// {csiStopStr,            cmdCsiStopCallback,      printCsiStopUsage     },

// /* csi disable */
// { csiDisableStr,            cmdCsiDisableCallback,       printCsiDisableUsage      },

// /* csi get results */
// {csiGetResultsStr,         cmdCsiGetResultsCallback,   printCsiGetResultsUsage   },
// #endif

// /* Calibrator */
// { calibratorStr,        cmdCalibratorCallback,         printCalibratorUsage        },


#ifdef CC35XX
// #ifdef SNTP_SUPPORT
// { SntpConfigNTPServers,       cmdSntpConfigServers,        printSntpConfigServersUsage              },
// { SntpUpdateDateTime,         cmdSntpUpdateDateTime,       printSntpUpdateDateTimeUsage              },
// #endif
{ SetDateTime,                cmdSetDateTime,           printSetDateTimeUsage              },
{ GetDateTime,                cmdGetDateTime,           printGetDateTimeUsage              },
/*------------------ vendor IE ---------------------------*/
///* Add vendor iE to the list */
// {wlanAddVendorIEStr, cmdAddVendorIECallback, printaddVendoIEUsage },

///* remove vendor iE from the list */
// {wlanRemoveVendorIEStr, cmdDeleteVendorIECallback, printDeleteVendoIEUsage },

/*------------------ Configure STA aging event ---------------------------*/
{wlanConfigPeerAgingStr, cmdConfigStaAgingEventCallback, printConfiPeerAgingUsage },

/*------------------ p2p ---------------------------*/
/* P2P Device role up */
{wlanRoleUpP2PStr, cmdWlanRoleUpP2PCallback, printWlanRoleUpP2PUsage },

{wlanRoleDownP2PStr, cmdWlanRoleDownP2PCallback, printWlanRoleDownP2PUsage },

{wlanP2PFindStr,    cmdWlanP2PFindCallback,    printWlanP2PFindUsage },

{wlanP2PConnectStr, cmdWlanP2PConnectCallback , printWlan2PConnectUsage },

{wlanP2PStopFindStr, cmdWlanP2PFindStopCallback , printWlanP2PFindStopUsage },

{wlanP2PGrpRemoveStr, cmdWlanP2PGrpRemoveCallback , printWlanP2PGrpRemoveUsage },

{wlanP2PSetchannelStr, cmdWlanP2PSetChannelCallback , printWlanP2PSetChannelUsage },

{wlanP2PGetchannelStr, cmdWlanP2PGetChannelCallback , printWlanP2PGetChannelUsage },

{wlanP2PListenStr, cmdWlanP2PListenCallback, printWlanP2PListenUsage },

{wlanP2PCancelStr, cmdWlanP2PCancelCallback, printWlanP2PCancelUsage },

/*------------------ p2p end ---------------------------*/
/* Start AP WPS */
{startApWpsStr, cmdStartApWpsCallback, printStartApWpsUsage },
/*------------------ WSOC ---------------------------*/
// {SetWsocPrimaryStr, cmdSetWsocPrimaryCallback, printSetWsocPrimaryUsage },
/*------------------ WSOC end ---------------------------*/

/*------------ connection policy and profiles ----------------*/

/* Connection Policy Set */
{ wlanSetConnPolicyStr,   cmdWlanSetConnPolicyCallback,   printWlanSetConnPolicyUsage  },

/* Connection Policy Get */
{ wlanGetConnPolicyStr,   cmdWlanGetConnPolicyCallback,   printWlanGetConnPolicyUsage  },

/* Add Profile */
{ wlanAddProfileStr,      cmdWlanAddProfileCallback,      printAddProfileUsage         },

/* Remove Profile */
{ wlanDelProfileStr,      cmdWlanDeleteProfileCallback,   printDeleteProfileUsage      },

/* Get Profile */
{ wlanGetProfileStr,      cmdWlanGetProfileCallback,      printGetProfileUsage         },

///* Set Scan dwell time */
//{wlanSetScanDwellTimeStr,    cmdWlanSetScanDwellTimeCallback, printWlanSetScanDwellTimeUsage },

/* Wlan Profile Connect */
{ wlanProfileConnectStr,  cmdWlanProfileConnectCallback,  printWlanProfileConnectUsage },

/*----------- end connection policy and profiles -------------*/

{ pingStartStr,           cmdPingStartCallback,           printPingStartUsage },

{ pingStopStr,            cmdPingStopCallback,            printPingStopUsage },

//{ wlanSetRegDomEntryStr,  cmdWlanSetRegDomainEntryCallback, printWlanSetRegDomainEntryUsage },
//
//{ wlanGetRegDomEntryStr,  cmdWlanGetRegDomainEntryCallback, printWlanGetRegDomainEntryUsage },

#endif // CC35XX
{ loadCertificate,              cmdloadCartificateCallback,            printloadCartificateUsage      },

#ifdef TEST_CMD
/* test */
{ testStr,              cmdtestCallback,            printWlanStopUsage      }
#endif
};

uint32_t            gMaxCmd = (sizeof(gCmdList)/sizeof(cmdAction_t));
appControlBlock     app_CB;

/*!
    \brief          WlanStackEventHandler

    This handler gets called whenever a WLAN event is reported
    by the host driver / NWP. Here user can implement he's own logic
    for any of these events. This handler is used by 'network_terminal'
    application to show case the following scenarios:

    1. Handling connection / Disconnection.
    2. Handling scan

    \param         pWlanEvent       -   pointer to Wlan event data.

    \return        void

    \note          For more information, please refer to: user.h in the porting
                   folder of the host driver and the
                   CC31xx/CC32xx NWP programmer's guide (SWRU455)

    \sa            cmdWlanConnectCallback,cmdWlanDisconnectCallback.

*/
void WlanStackEventHandler(WlanEvent_t *pWlanEvent)
{
    void *staif = NULL;

    if(!pWlanEvent)
    {
        return;
    }
    switch(pWlanEvent->Id)
    {
        case WLAN_EVENT_CONNECT:
        {

        if( pWlanEvent->Data.Connect.Status <0 )
        {
            UART_PRINT(
                        "\n\r\n\r[WLAN EVENT HANDLER] connection failed(probably due to FW crash), reovery action required");
            osi_SyncObjSignal(&app_CB.CON_CB.connectEventSyncObj);
            break;
        }
        SET_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);

        /* Copy new connection SSID and BSSID to global parameters */
        os_memcpy(app_CB.CON_CB.ConnectionSSID, pWlanEvent->Data.Connect.SsidName,
               pWlanEvent->Data.Connect.SsidLen);
        os_memcpy(app_CB.CON_CB.ConnectionBSSID, pWlanEvent->Data.Connect.Bssid,
               WLAN_BSSID_LENGTH);

        UART_PRINT(
            "\n\r\n\r[WLAN EVENT HANDLER] STA Connected to the AP: %s , "
            "BSSID: %x:%x:%x:%x:%x:%x\n\r",
            app_CB.CON_CB.ConnectionSSID,
            app_CB.CON_CB.ConnectionBSSID[0],
            app_CB.CON_CB.ConnectionBSSID[1],
            app_CB.CON_CB.ConnectionBSSID[2],
            app_CB.CON_CB.ConnectionBSSID[3],
            app_CB.CON_CB.ConnectionBSSID[4],
            app_CB.CON_CB.ConnectionBSSID[5]);
        staif = network_get_sta_if();
        if(staif != NULL)
        {
            network_set_up(staif);
        }

        if(osi_SyncObjSignal(&(app_CB.CON_CB.connectEventSyncObj)) != 0)
        {
            UART_PRINT(
            "\n\r\n\r[WLAN EVENT HANDLER] STA Connected  sync object signal failure 0x%x",(uint32_t) app_CB.CON_CB.connectEventSyncObj);
        }

    }
    break;

    case WLAN_EVENT_DISCONNECT:
    {
        WlanEventDisconnect_t  *pEventData = NULL;

        Report("\n\r\n\r[WLAN EVENT HANDLER] WLAN_EVENT_DISCONNECT ");
        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);
        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);
        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IPV6_ACQUIRED);
#ifndef CC35XX
        killAllProcess();
#endif
        staif = network_get_sta_if();

        network_set_down(staif);
        /* If ping operation is running, release it. */
        if(IS_PING_RUNNING(app_CB.Status))
        {
            UART_PRINT(
                "\n\rPing failed, since device is no longer connected.\n\r");
        }

        pEventData = &pWlanEvent->Data.Disconnect;

        /* If the user has initiated 'Disconnect' request,
           'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED */
#ifdef CC35XX		   
        if(WLAN_REASON_DEAUTH_LEAVING == pEventData->ReasonCode)
#else
        if(WLAN_DISCONNECT_USER_INITIATED == pEventData->ReasonCode)
#endif
        {
            UART_PRINT(
                "\n\r[WLAN EVENT HANDLER] Device disconnected from the AP: %s,\n\r"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                app_CB.CON_CB.ConnectionSSID,
                app_CB.CON_CB.ConnectionBSSID[0],
                app_CB.CON_CB.ConnectionBSSID[1],
                app_CB.CON_CB.ConnectionBSSID[2],
                app_CB.CON_CB.ConnectionBSSID[3],
                app_CB.CON_CB.ConnectionBSSID[4],
                app_CB.CON_CB.ConnectionBSSID[5]);
        }
        else
        {
            UART_PRINT(
                "\n\r[WLAN EVENT HANDLER] Device disconnected from the AP: %s,\n\r"
                "BSSID: %x:%x:%x:%x:%x:%x, reason code :%d\n\r",
                app_CB.CON_CB.ConnectionSSID,
                app_CB.CON_CB.ConnectionBSSID[0],
                app_CB.CON_CB.ConnectionBSSID[1],
                app_CB.CON_CB.ConnectionBSSID[2],
                app_CB.CON_CB.ConnectionBSSID[3],
                app_CB.CON_CB.ConnectionBSSID[4],
                app_CB.CON_CB.ConnectionBSSID[5],
                pEventData->ReasonCode);
        }
        memset(&(app_CB.CON_CB.ConnectionSSID), 0x0,
               sizeof(app_CB.CON_CB.ConnectionSSID));
        memset(&(app_CB.CON_CB.ConnectionBSSID), 0x0,
               sizeof(app_CB.CON_CB.ConnectionBSSID));

        osi_SyncObjSignal(&(app_CB.CON_CB.disconnectEventSyncObj));
        osi_SyncObjSignal(&(app_CB.CON_CB.connectEventSyncObj));//if on the middle of connection, stop wait

    }
    break;

    case WLAN_EVENT_PEER_AGING:
    {
       struct os_time t;
       os_get_time(&t);
       Report("\n\r\n\r[WLAN EVENT HANDLER] %d.%d WLAN_EVENT_PEER_AGING, mac:0x%02x:%02x:%02x:%02x:%02x:%02x inactive time:%d Ms",
                t.sec,
                t.usec/1000,
                pWlanEvent->Data.peerAging.peer_mac[0],
                pWlanEvent->Data.peerAging.peer_mac[1],
                pWlanEvent->Data.peerAging.peer_mac[2],
                pWlanEvent->Data.peerAging.peer_mac[3],
                pWlanEvent->Data.peerAging.peer_mac[4],
                pWlanEvent->Data.peerAging.peer_mac[5],
                pWlanEvent->Data.peerAging.inactiveTime);
    }
    break;
    case WLAN_EVENT_SCAN_RESULT:
    {
        uint32_t    index;
        uint32_t scan_result_len = pWlanEvent->Data.ScanResult.NetworkListResultLen;
        UART_PRINT("\n\r[WLAN EVENT HANDLER] scan result received: %d \n\r", scan_result_len);
        scan_result_len = MIN(scan_result_len,WLAN_MAX_SCAN_COUNT);
        for(index = 0; index < scan_result_len ; index++)
        {

            /* Copy SSID and BSSID to global parameters */
            //strncpy - can handle strings with null-terminated characters
            //os_memcpy(app_CB.gDataBuffer.netEntries[index].Ssid, pWlanEvent->Data.ScanResult.NetworkListResult[index].Ssid,
            //   pWlanEvent->Data.ScanResult.NetworkListResult[index].SsidLen);
            //clear the buffer before copying
            os_memset(app_CB.gDataBuffer.netEntries[index].Ssid, 0x0, WLAN_SSID_MAX_LENGTH);
            os_strncpy((char *)app_CB.gDataBuffer.netEntries[index].Ssid, (char *)pWlanEvent->Data.ScanResult.NetworkListResult[index].Ssid,
               pWlanEvent->Data.ScanResult.NetworkListResult[index].SsidLen);
            os_memcpy(app_CB.gDataBuffer.netEntries[index].Bssid, pWlanEvent->Data.ScanResult.NetworkListResult[index].Bssid,
                WLAN_BSSID_LENGTH);

            app_CB.gDataBuffer.netEntries[index].SsidLen = pWlanEvent->Data.ScanResult.NetworkListResult[index].SsidLen;
            app_CB.gDataBuffer.netEntries[index].Rssi = (int8_t)pWlanEvent->Data.ScanResult.NetworkListResult[index].Rssi;
            app_CB.gDataBuffer.netEntries[index].SecurityInfo = pWlanEvent->Data.ScanResult.NetworkListResult[index].SecurityInfo;
            app_CB.gDataBuffer.netEntries[index].Channel = pWlanEvent->Data.ScanResult.NetworkListResult[index].Channel;

        }

        printScanResults(scan_result_len);
        osi_SyncObjSignal(&app_CB.eventCompletedScanObj);
    }
    break;
    case WLAN_EVENT_CONNECT_PERIODIC_SCAN_COMPLETE:
        Report("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_CONNECT_PERIODIC_SCAN_COMPLETE");
        break;
    case WLAN_EVENT_ADD_PEER:
    {
        uint8_t *macAddr = pWlanEvent->Data.AddPeer.Mac;
        UART_PRINT(
            "\n\r[WLAN EVENT HANDLER] WLAN_EVENT_ADD_PEER Device Mac: %x:%x:%x:%x:%x:%x Connected to AP \n\r",
            macAddr[0],
            macAddr[1],
            macAddr[2],
            macAddr[3],
            macAddr[4],
            macAddr[5]);
        app_CB.ConnectedStations++;
        SET_STATUS_BIT(app_CB.Status, STATUS_BIT_PEER_CONNECTED);
    }
    break;
    case WLAN_EVENT_REMOVE_PEER:
    {
        uint8_t *macAddr = pWlanEvent->Data.RemovePeer.Mac;
        UART_PRINT(
            "\n\r[WLAN EVENT HANDLER] WLAN_EVENT_REMOVE_PEER Device Mac: %x:%x:%x:%x:%x:%x Disconnected from AP \n\r",
            macAddr[0],
            macAddr[1],
            macAddr[2],
            macAddr[3],
            macAddr[4],
            macAddr[5]);
        app_CB.ConnectedStations--;
        if (app_CB.ConnectedStations == 0 )
        {
            CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_PEER_CONNECTED);
        }
    }
    break;
    case WLAN_EVENT_CONNECTING:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_CONNECTING, STA is connecting \n\r");
    }
    break;
    case WLAN_EVENT_ASSOCIATED:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_ASSOCIATED, STA associated \n\r");
    }
    break;
#ifdef CC35XX
    case WLAN_EVENT_WPS_INVALID_PIN:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_WPS_INVALID_PIN \n\r");
    }
    break;
    case WLAN_EVENT_AP_WPS_START_FAILED:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_AP_WPS_START_FAILED with error code: %d \n\r", pWlanEvent->Data.wpsFailCode);
    }
    break;
    case WLAN_EVENT_AUTHENTICATION_REJECTED:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_AUTHENTICATION_REJECTED \n\r");
    }
    break;
    case WLAN_EVENT_COMMAND_TIMEOUT:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_COMMAND_TIMEOUT \n\r");
    }
    break;
    case WLAN_EVENT_ASSOCIATION_REJECTED:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_ASSOCIATION_REJECTED \n\r");
    }
    break;
    case WLAN_EVENT_GENERAL_ERROR:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_GENERAL_ERROR \n\r");
    }
    break;
#endif
    case WLAN_EVENT_BLE_ENABLED:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] BLE Enabled \n\r");
    }
    break;
    case WLAN_EVENT_CS_FINISH:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_CS_FINISH \n\r");
    }
    break;
    case WLAN_EVENT_ERROR:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_ERROR error: 0x%X \n\r", pWlanEvent->Data.error.error_num);
    }
    break;
    case WLAN_EVENT_FW_CRASH:
    {
        uint32_t    index;
        uint32_t    log_size = pWlanEvent->Data.FwCrashLog.log_buffer_len;

        UART_PRINT("\n\r[HANDLER] WLAN_EVENT_FW_CRASH FW is crashed \n\r");
        if (pWlanEvent->Data.FwCrashLog.log_buffer == NULL)
        {
            UART_PRINT("Error: log_buffer is NULL");
            return;
        }
        UART_PRINT("log_buffer_len = %d\n\r", log_size);
        if (log_size > CC33XX_MAX_FW_LOGS_BUFFER_SIZE)
            log_size = CC33XX_MAX_FW_LOGS_BUFFER_SIZE;
        for(index = 0; index < log_size ; index++)
        {
            UART_PRINT("%02x ", pWlanEvent->Data.FwCrashLog.log_buffer[index]);
        }

    }
    break;
#ifdef CC35XX
    case WLAN_EVENT_P2P_SCAN_COMPLETED:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER]P2P Scan completed event arrived\n");

        //only if we are waiting for this syncObj
        if (g_wait_p2p_scan_complete)
        {
            g_wait_p2p_scan_complete = FALSE;
            osi_SyncObjSignal(&p2p_find_stopped_syncObj);
        }

    }
    break;
    case WLAN_EVENT_P2P_GROUP_STARTED:
    {
        uint32_t    ret;

        Report("[WLAN EVENT HANDLER] P2P GROUP STARTED \n\r");

        SET_STATUS_BIT(app_CB.Status, STATUS_BIT_P2P_GROUP_STARTED);

        if (pWlanEvent->Data.FormationComplete.P2pRoleGo == TRUE)
        {
            void* apif;
            SET_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_AP_BIT);
            network_stack_add_if_ap();
            apif = network_get_ap_if();
            if(apif != NULL)
            {
                network_set_up(apif);
            }
            else
            {
                Report("\n\r[WLAN EVENT HANDLER][ERROR]WLAN_EVENT_P2P_GROUP_STARTED failed to load lwip interface for AP-GO\n\r");
            }

        }
        else
        {
            void* staif;
            network_stack_add_if_sta();//send callback to tcp
            SET_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_STA_BIT);

            staif = network_get_sta_if();
            if(staif != NULL)
            {
                network_set_up(staif);
            }

            //Wait for getting wlan role up response
            ret = osi_SyncObjWait(&(app_CB.CON_CB.staRoleupSyncObj), OSI_WAIT_FOR_SECOND * 2);
            if(OSI_OK != ret)
            {
                Report("\n\r[WLAN EVENT HANDLER][ERROR] WLAN_EVENT_P2P_GROUP_STARTED: Failed waiting sync object\n\r");
                ASSERT_GENERAL(0);
            }
        }

        Report("[WLAN EVENT HANDLER] WLAN_EVENT_P2P_GROUP_STARTED \r\n P2P ,ROLE %s connected. GO Bssid: %02x:%02x:%02x:%02x:%02x:%02x on Channel :%d\n\r",
                ((pWlanEvent->Data.FormationComplete.P2pRoleGo == TRUE)?"GO":"CLIENT"),
                pWlanEvent->Data.FormationComplete.Bssid[0],
                pWlanEvent->Data.FormationComplete.Bssid[1],
                pWlanEvent->Data.FormationComplete.Bssid[2],
                pWlanEvent->Data.FormationComplete.Bssid[3],
                pWlanEvent->Data.FormationComplete.Bssid[4],
                pWlanEvent->Data.FormationComplete.Bssid[5],
                pWlanEvent->Data.FormationComplete.Channel);
                
        osi_SyncObjSignal(&(app_CB.CON_CB.connectEventSyncObj));

    }
    break;
    case WLAN_EVENT_P2P_GROUP_REMOVED:
    {
        void* netif;
        Report("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_P2P_GROUP_REMOVED! RoleType=%d\n\r",
            pWlanEvent->Data.GroupRemoved.RoleType);

        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_P2P_GROUP_STARTED);

        if (pWlanEvent->Data.GroupRemoved.RoleType == WLAN_ROLE_P2P_GO)
        {
            netif = network_get_ap_if();
            if (netif != NULL)
            {
                network_set_down(netif);
                network_stack_remove_if_ap();
            }
            else
            {
                Report("\n\r[WLAN EVENT HANDLER][ERROR] WLAN_EVENT_P2P_GROUP_REMOVED failed to unload if_ap lwip interface\n\r");
            }
            CLEAR_BIT_IN_BITMAP(ActiveNetIfBitMap, NET_IF_AP_BIT);
            
        }
        else if (pWlanEvent->Data.GroupRemoved.RoleType == WLAN_ROLE_P2P_CL)
        {
            CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);
            CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);
            CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IPV6_ACQUIRED);
            netif = network_get_sta_if();
            if (netif != NULL)
            {
                network_set_down(netif);

                network_stack_remove_if_sta();
            }
            else
            {
                Report("\n\r[WLAN EVENT HANDLER][ERROR] WLAN_EVENT_P2P_GROUP_REMOVED failed to unload sta_if lwip interface\n\r");
            }

            CLEAR_BIT_IN_BITMAP(ActiveNetIfBitMap, NET_IF_STA_BIT);
        }

    }
    break;
    case WLAN_EVENT_P2P_GROUP_FORMATION_FAILED:
    {
        Report("\n\r[WLAN EVENT HANDLER][ERROR] WLAN_EVENT_P2P_GROUP_FORMATION_FAILED\n\r");
        osi_SyncObjSignal(&(app_CB.CON_CB.connectEventSyncObj));

        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_P2P_GROUP_STARTED);
    }
    break;

#endif

    default:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] Unexpected event [%d]\n\r",
                   pWlanEvent->Id);
    }
    break;
    }
    //UART_PRINT(cmdPromptStr);
}



/*!
    \brief          Command Prompt.

    This routine reads characters from UART interface,
    and serves as command line for application's user input.
    it reads a string representing the command followed by parameters.
    First - it searches for the command in the global command table
    (gCmdList) and if found, dispatches the handler.
    If no command is found, appropriate error would be printed to screen.
    This function isn't expected to return.

    \param          arg       -   Points to command line buffer.

    \return

    \sa             GetCmd

*/
int32_t cmd_prompt(void *arg)
{
    int32_t     lRetVal = 1;
    uint32_t    i = 0;
    char        cmdBuffer[(MAX_CMD_NAME_LEN+5)];
    char        *token = NULL;

    while(!app_CB.Exit)
    {

        /* Poll UART terminal to receive user command terminated by '/r' */
        lRetVal = GetCmd((char *)app_CB.CmdBuffer, CMD_BUFFER_LEN, (char *)cmdPromptStr);
        if(0 == lRetVal)
        {
            UART_PRINT(lineBreak);
            continue;
        }
        else
        {
            os_memcpy(cmdBuffer, &app_CB.CmdBuffer, (MAX_CMD_NAME_LEN+4));
            cmdBuffer[MAX_CMD_NAME_LEN+4] = '\0';
            token = strtok(cmdBuffer, " ");
            if(token)
            {
                for(i = 0; i < gMaxCmd; i++)
                {
                    /* Search for typed command in
                       the application command list. */
                    if(!strcmp((char*)token, gCmdList[i].cmd))
                    {
                        /* Dispatch command callback */
                        lRetVal =
                            gCmdList[i].callback((void *)(app_CB.CmdBuffer +
                                                          strlen(token)));
                        UART_PRINT(lineBreak);
                        break;
                    }
                }
                if(i >= gMaxCmd)
                {
                    UART_PRINT(lineBreak);
                    UART_PRINT("No such command\n\r");
                    /* Display Network Terminal API commands */
                    showAvailableCmd();
#ifdef PRINT_DBG_TOTAL_MALLOC_FREE
                    UART_PRINT("\n\r+++++Total alloc is %d+++++\n\r", totalloc);
#endif
                }
            }
        }



    }
    return(0);
}

/*!
    \brief          Show help callback.

    This routine searches for the command in the global command table
    (gCmdList) and if found, dispatches the appropriate callback, to
    display it's menu options. If no command is found, appropriate error
    would be printed to screen and list of available commands.

    \param          arg       -   Points to command name.

    \return         This function shall return 0.

    \sa             GetCmd

*/
int32_t cmdHelpCallback(void *arg)
{
    uint32_t    i = 0;
    char        *param[2] = {NULL, NULL};
    char        *token = NULL;
    char        cmdStr[CMD_BUFFER_LEN+1];

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    /* Get command options/params */
    token = strtok(cmdStr, space_str);
    while(token && (i < 2))
    {
        param[i] = token;
        token = strtok(NULL, space_str);
        i++;
    }

    /* Lookup for help display callback for command */
    if(param[0])
    {
        for(i = 0; i < gMaxCmd; i++)
        {
            if(!strcmp(param[0], gCmdList[i].cmd))
            {
                gCmdList[i].printusagecallback((void *) arg);
                break;
            }
        }
        if(i >= gMaxCmd)
        {
            UART_PRINT("Help doesn't exist for command %s\n\r", param[0]);
        }
    }
    else
    {
        printHelpUsage(arg);
        showAvailableCmd();
    }

    return(0);
}


/*!
    \brief          Prints 'help' command help menu.

    \param          arg       -   Points to command line buffer.

    \return         This function shall return 0.

    \sa             cmdHelpCallback

*/
int32_t printHelpUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(helpStr);
    UART_PRINT(helpUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(helpDetailsStr);
    UART_PRINT(lineBreak);
    return(0);
}


/*!
    \brief          clear command

    This routine clears the command line console screen.

    \param          arg       -   Points to command line buffer.

    \return         This function shall return 0.

    \sa             printClearUsage

 */
int32_t cmdClearcallback(void *arg)
{
#if CC35XX // Was ifdef 0 in mx
    ClearTerm();
#endif 
    return(0);
}

/*!
    \brief          Prints 'clear' command help menu.

    \param          arg       -   Points to command line buffer.

    \return         This function shall return 0.

    \sa             cmdClearcallback

 */
int32_t printClearUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(clearStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(clearDetailsStr);
    UART_PRINT(lineBreak);
    return(0);
}
/*!
    \brief          Prints IP address.

    This routine prints IP addresses in a dotted decimal
    notation (IPv4) or colon : notation (IPv6)

    \param          ip         -   Points to command line buffer.

    \param          ipv6       -   Flag that sets
                                   if the address is IPv4 or IPv6.

    \return         void

 */
void PrintIPAddress(unsigned char ipv6,
                    void *ip)
{
    uint32_t        *pIPv4;
    uint8_t         *pIPv6;
    int32_t          i=0;

    if(!ip)
    {
        return;
    }

    if(ipv6)
    {
        pIPv6 = (uint8_t*) ip;

        for(i = 0; i < 14; i+=2)
        {
            UART_PRINT("%02x%02x:", pIPv6[i], pIPv6[i+1]);
        }

        UART_PRINT("%02x%02x", pIPv6[i], pIPv6[i+1]);
    }
    else
    {
        pIPv4 = (uint32_t*)ip;
        UART_PRINT("%d.%d.%d.%d",
                    WLAN_IPV4_BYTE(*pIPv4,3),
                    WLAN_IPV4_BYTE(*pIPv4,2),
                    WLAN_IPV4_BYTE(*pIPv4,1),
                    WLAN_IPV4_BYTE(*pIPv4,0));
    }
    return;
}

/*!
    \brief          Prints list of available commands.

    \return         This function shall return 0.

    \sa             cmd_prompt

 */
int32_t showAvailableCmd()
{
    uint8_t i = 0;

    printBorder('=', 80);
    UART_PRINT("\n\rAvailable commands:\n\r");

    for(i = 0; i < gMaxCmd; i++)
    {
        if(!(i % 4))
        {
            UART_PRINT(lineBreak);
        }
        UART_PRINT("%-20s", gCmdList[i].cmd);
    }

    UART_PRINT(lineBreak);
    printBorder('=', 80);
    UART_PRINT(lineBreak);

    return(0);
}

/*!
    \brief          Display application banner

    This routine shows how to get device information form the NWP.
    Also, it prints the PHY, MAC, NWP and Driver versions.

    \param          appName    -   points to a string representing
                                   application name.

    \param          appVersion -   points to a string representing
                                   application version number.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,
                    this function would return negative value.

    \sa             sl_DeviceGet, sl_NetCfgGet

*/
int32_t DisplayAppBanner(char* appName, char* appVersion)
{
    Report("******************************************************************\r\n");
    Report("***************** %-28s *******************\r\n", APPLICATION_NAME);
    Report("***************** %-28s *******************\r\n", APPLICATION_VERSION);
    Report("******************************************************************\r\n");
#ifdef __clang__
    Report("Compiled with Clang\r\n");
#elif defined(__GNUC__)
    Report("Compiled with GCC\r\n");
#else
    printf("Compiled with an unknown compiler\r\n");
#endif
    return 0;
    return(0);
}

/*!
    \brief          initialize Application's Variables

    This routine initialize the application control block.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1.

    \sa             MainThread

*/
int32_t    initAppVariables(void)
{
    int32_t ret = 0;

    app_CB.Status = 0 ;
    app_CB.Role = WLAN_ROLE_RESERVED;
    app_CB.Exit = FALSE;

    memset(&app_CB.CmdBuffer, 0x0, CMD_BUFFER_LEN);
    memset(&app_CB.gDataBuffer, 0x0, sizeof(app_CB.gDataBuffer));
    memset(&app_CB.CON_CB, 0x0, sizeof(app_CB.CON_CB));

    ret = osi_SyncObjCreate(&(app_CB.CON_CB.disconnectEventSyncObj));
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }

    ret = osi_SyncObjCreate(&(app_CB.CON_CB.connectEventSyncObj));
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }

    ret = osi_SyncObjCreate(&(app_CB.CON_CB.eventCompletedSyncObj));
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }
    ret = osi_SyncObjCreate(&app_CB.eventCompletedScanObj);
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }
    ret = osi_SyncObjCreate(&(app_CB.CON_CB.dhcpIprecvSyncObj));
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }
    ret = osi_SyncObjCreate(&(app_CB.CON_CB.staRoleupSyncObj));
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }
    ret = osi_SyncObjCreate(&(app_CB.CON_CB.staRoledownSyncObj));
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }

    return(ret);
}

void *mycalloc(uint32_t memb,size_t len)
{
    return 0;
}


void *myzalloc(size_t len)
{
    return 0;
}


#ifdef TERMINAL_TAB_COMPLETION
void initCompletions()
{
    uint8_t len = gMaxCmd;
    
    char **strings = calloc(len, sizeof(char *));
    for (uint8_t i = 0; i < len; i += 1)
    {
        const char *cmd = gCmdList[i].cmd;
        uint8_t cmdLen = strlen(cmd);
        strings[i] = (char *)malloc(cmdLen + 1);
        os_strlcpy(strings[i], cmd, cmdLen + 1);
    }

    initCompletionArray(strings, len);
}
#endif


//OSPREY_MX-38
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x))) //TODO temporary need to be removed
#define ICACHE_BASE 0x41902000  //TODO temporary need to be removed, only for M3, M$ has different address


void *network_terminal_entry(void *args)
{
#ifdef CC33XX
    int32_t             RetVal = -1;

    /* init drivers and services */
    Drivers_open();
    Board_driversOpen();

    InitTerm();

#ifdef TERMINAL_TAB_COMPLETION
    initCompletions();
#endif

    /* Init Application variables */
    initAppVariables();
    network_stack_init();

    // Turn on WiFi chip and set SPI interface
    extern void wlan_TurnOffWlan();
    wlan_TurnOffWlan();
#elif defined(CC35XX)
    int32_t             RetVal = -1;
    HWREG(ICACHE_BASE + 0x84) |= 0x00000001  ;//OSPREY_MX-38
    HWREG(ICACHE_BASE + 0x4) |= 0xc0000000  ;//OSPREY_MX-38
    //HWREG(ICACHE_BASE + 0x4) |= 0x80000000  ;//OSPREY_MX-38, this is for 64M cache, instead CRAM

    Board_init();

#ifdef TERMINAL_TAB_COMPLETION
    initCompletions();
#endif

    /* Init Application variables */
    initAppVariables();
    network_stack_init(); //initialize lwip
    // init the terminal
    InitTerm();
    datetime_init();

    initialize_mbedtls_threading();
#endif // CC35XX


    /* Output device information to the UART terminal */
    RetVal = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    /* Display Quick Track API commands */
    // showAvailableCmd();

    if(RetVal < 0)
    {
        /* Handle Error */
        UART_PRINT(
            "Indigo - Unable to retrieve device information \n");
        return(NULL);
    }

    /*
     * Calling UART handling method which serves as the application main loop.
     * Note that this function doesn't return.
     */
    RetVal = cmd_prompt(NULL);
#ifdef CC33XX
#ifdef TERMINAL_TAB_COMPLETION
    freeCompletionArray();
#endif
    Board_driversClose();
    Drivers_close();
#endif // CC33XX
	return NULL;
}


#ifdef CC35XX
void *mainThread(void *args)
{
    network_terminal_entry(NULL);
    return NULL;
}
#endif // CC35XX

#ifdef SNTP_SUPPORT
int32_t cmdSntpConfigServers(void *arg)
{
    int32_t ret  = 0;
    char  *serverIp[3];
    uint32_t numOfServers;
    int i;
    ret = ParseSntpConfigServersCmd(arg, &numOfServers,serverIp);
    if(ret < 0)
    {
        printSntpConfigServersUsage(arg);
        return(0);
    }
    sntpWrapper_store_servers(numOfServers,serverIp[0],serverIp[1], serverIp[2]);
    for(i=0; i<numOfServers; i++)
    {
        os_free(serverIp[i]);
    }
    return ret;
}
int32_t cmdSntpUpdateDateTime(void *arg)
{
    int32_t ret  = 0;

    if (((!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT)) || (!IS_STA_CONNECTED(app_CB.Status)))
            && (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT)))
    {
        Report("\n\rSTA/AP role is not up or connected.\n\r");
        return -1;
    }

    ret = sntpWrapper_updateDateTime();
    return ret;
}
#endif
int32_t cmdSetDateTime(void *arg)
{
    int32_t ret  = 0;
    uint32_t epochTime;
    uint32_t year,month,day,hour, minute, second;

    ret = ParseSetDateTimeCmd(arg, &year,&month,&day,&hour, &minute, &second);
    if(ret < 0)
    {
        printSetDateTimeUsage(arg);
        return(0);
    }
    epochTime =  datetime_to_epoch(year,month,day,hour, minute, second);
    datetime_SecondsSet( epochTime);
    return ret;
}
int32_t cmdGetDateTime(void *arg)
{
    int32_t ret  = 0;
    datetime_printCurTime();
    return ret;
}
