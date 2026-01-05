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
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <pthread.h>

#include "atcmd.h"
#include "uart_term.h"
#include "atcmd_defs.h"
#include "atcmd_event.h"

// Network terminal dependencies
#include "network_terminal.h"
#include "network_lwip.h"
#include "wlan_if.h"
#include "osi_kernel.h"
#include "lwip/netif.h"


//OSPREY_MX-38
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x))) //TODO temporary need to be removed
#define ICACHE_BASE 0x41902000  //TODO temporary need to be removed, only for M3, M$ has different address

//*****************************************************************************
// defines
//*****************************************************************************
#ifdef APPLICATION_NAME
    #undef APPLICATION_NAME
    #define APPLICATION_NAME        "AT Commands"
#endif

#ifdef APPLICATION_VERSION
    #undef APPLICATION_VERSION
    #define APPLICATION_VERSION     "0.0.0.0"
#endif

#define ATCOMMANDS_TASK_STACK_SIZE   (4096)

#define SLNET_IF_WIFI_PRIO           (5)

#define AT_COMMANDS_TERMINAL_TAB_COMPLETION

#define WLAN_REASON_DEAUTH_LEAVING 3

#define USER_PROMPT_STR "user: "
#define DATA_PROMPT_STR "continue supplying data (%d bytes left): "

//****************************************************************************
// globals
//****************************************************************************
pthread_t ATCommands_eventThread = (pthread_t)NULL;
char ATCommands_cmdBuffer[ATCMD_CMD_BUFFER_SIZE];
char ATCommands_eventBuffer[ATCMD_EVENT_BUFFER_SIZE];

appControlBlock app_CB;

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************

//****************************************************************************
//                         EXTERNAL FUNTIONS
//****************************************************************************

//*****************************************************************************
//
//! \brief Display Application Banner
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
int8_t ATCommands_displayBanner(void)
{
    //WlanMacAddress_t macAddressParams = {0};
    //WlanFWVersions_t wlanFwVersions = {0};
    int8_t status = -1;

    /* Print device Mac address */
    // status = Wlan_Get(WLAN_GET_MACADDRESS, (void *)&macAddressParams);
    // if (status < 0)
    // {
    //     return -1;
    // }

    // status = Wlan_Get(WLAN_GET_FWVERSION, (void *)&wlanFwVersions);
    // if (status < 0)
    // {
    //     return -1;
    // }

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\t    %s Example    \n\r",
               APPLICATION_NAME,
               APPLICATION_VERSION);
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");

    // The commented out prints requires wlan_start().
    // UART_PRINT("\t MAC:  %d.%d.%d.%d",
    //            wlanFwVersions.major_version, wlanFwVersions.minor_version,
    //            wlanFwVersions.api_version, wlanFwVersions.build_version);
    // UART_PRINT("\n\r");
    // UART_PRINT("\t PHY:  %d.%d.%d.%d",
    //            wlanFwVersions.phy_version[0], wlanFwVersions.phy_version[1],
    //            wlanFwVersions.phy_version[2], wlanFwVersions.phy_version[3],
    //            wlanFwVersions.phy_version[4], wlanFwVersions.phy_version[5],
    //            wlanFwVersions.phy_version[6], wlanFwVersions.phy_version[7]);
    // UART_PRINT("\n\r");

    UART_PRINT("\t HOST version: %s", version_upper_mac);
    UART_PRINT("\n\r\n\r");

    // UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
    //            macAddressParams.pMacAddress[0], macAddressParams.pMacAddress[1],
    //            macAddressParams.pMacAddress[2], macAddressParams.pMacAddress[3],
    //            macAddressParams.pMacAddress[4], macAddressParams.pMacAddress[5]);
    // UART_PRINT("\n\r\n\r");

    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r\n\r");

    return status;
}

#ifdef AT_COMMANDS_TERMINAL_TAB_COMPLETION
void initCompletions()
{
    uint32_t len = ATCmd_maxCmd;
    
    char **strings = calloc(len, sizeof(char *));
    for (uint8_t i = 0; i < len; i += 1)
    {
        const char *cmd = ATCmd_list[i].cmd;
        uint8_t cmdLen = strlen(cmd);
        /* One extra byte for '\0' and 2 more for "at" */
        strings[i] = (char *)malloc(cmdLen + strlen(ATCMD_CMD_PREFIX) + 1);
        os_strlcpy(strings[i], ATCMD_CMD_PREFIX, strlen(ATCMD_CMD_PREFIX) + 1);
        os_strlcpy(strings[i] + strlen(ATCMD_CMD_PREFIX), cmd, cmdLen + 1);
    }

    initCompletionArray(strings, len);
}
#endif // AT_COMMANDS_TERMINAL_TAB_COMPLETION

//*****************************************************************************
//
//! ATCommands_readCmd
//!
//!  \return none
//!
//!  \brief Handler function to handle the AT commands input
//
//*****************************************************************************
int32_t ATCommands_readCmd(void)
{
    uint32_t i = 1;
    uint16_t length, dataReadLen, offset;
    uint16_t cmdBufferResidue;
    int16_t readRes, dataResidue, retVal;
    uint8_t format = ATCMD_DATA_FORMAT_BASE64, validCmd = 0;
    char *pbuf, tmpBuf[16], *pTmpBuf;

    Report("Enter AT Command:\n\r");

    while (i)
    {
        pbuf = NULL;
        usleep(1000);

        readRes = GetCmd((char *)ATCommands_cmdBuffer,
                         ATCMD_CMD_BUFFER_SIZE,
                         USER_PROMPT_STR);
        if (readRes == 0 || readRes == COMMAND_END_OF_TEXT)
        {
            /* Ctrl+C or Enter pressed */
            Report("\n\r");
            continue;
        }
        else if (readRes == COMMAND_TOO_LONG)
        {
            Report("\n\rERROR: AT command too long, max is %d characters\n\r",
                   ATCMD_CMD_BUFFER_SIZE - 1);
            continue;
        }
        validCmd = 1;

        /* send */
        /* sendto */
        if ((strstr(ATCommands_cmdBuffer, ATCmd_sockSendStr)) ||
            (strstr(ATCommands_cmdBuffer, ATCmd_sockSendToStr)))
        {
            /* Skip send(to) [sd] argument */
            pbuf = strchr(ATCommands_cmdBuffer, ATCMD_DELIM_ARG) + 1;
            if (strstr(ATCommands_cmdBuffer, ATCmd_sockSendToStr))
            {
                /* Skip sendto [family] argument */
                pbuf = strchr(pbuf, ATCMD_DELIM_ARG) + 1;
                /* Skip sendto [port] argument */
                pbuf = strchr(pbuf, ATCMD_DELIM_ARG) + 1;
                /* Skip sendto [address] argument */
                pbuf = strchr(pbuf, ATCMD_DELIM_ARG) + 1;
            }
        }

        if (pbuf != NULL)
        {
            pTmpBuf = tmpBuf;
            strncpy(pTmpBuf, pbuf, sizeof(tmpBuf) - 1);
            pTmpBuf[sizeof(tmpBuf) - 1] = '\0';

            /* format */
            retVal = StrMpl_getVal(&pTmpBuf, &format , ATCMD_DELIM_ARG,
                                   STRMPL_FLAG_PARAM_SIZE_8);
            if (retVal == STRMPL_ERROR_DELIM_MISSING)
            {
                Report("\n\rERROR: Missing delimiter after format\n\r");
                continue;
            } 
            else if (retVal < 0)
            {
                Report("\n\rERROR: Invalid format argument\n\r");
                continue;
            }

            /* data length */
            retVal = StrMpl_getVal(&pTmpBuf, &length, ATCMD_DELIM_ARG,
                                   STRMPL_FLAG_PARAM_SIZE_16);
            if (retVal == STRMPL_ERROR_DELIM_MISSING)
            {
                Report("\n\rERROR: Missing delimiter after length\n\r");
                continue;
            } 
            else if (retVal < 0)
            {
                Report("\n\rERROR: Invalid length argument\n\r");
                continue;
            }
            
            /* Compute space left in cmd buffer, 
             * account for terminating null */
            cmdBufferResidue = ATCMD_CMD_BUFFER_SIZE - readRes - 1;
            
            /* Compute the length of the data we already read */
            /* pTmpBuf now points to the beginning of the data portion */
            dataReadLen = readRes - \
                          (pbuf - ATCommands_cmdBuffer) - \
                          (pTmpBuf - tmpBuf);

            if (cmdBufferResidue < length - dataReadLen)
            {
                Report("\n\rERROR: Data length %d too long for "
                       "buffer limitation (%d bytes)\n\r",
                       length, ATCMD_CMD_BUFFER_SIZE - 1);
                continue;
            }

            if (format == ATCMD_DATA_FORMAT_BINARY)
            {
                dataResidue = length - dataReadLen;
                offset = readRes;

                /* Means that the requested length is
                 * smaller than the actual payload */
                if (dataResidue < 0)
                {
                    Report("\n\rERROR: Length smaller than payload\n\r");
                    continue;
                }
                while (dataResidue > 0)
                {
                    Report("\n\r");
                    Report(DATA_PROMPT_STR, dataResidue);

                    /* Read the data residue */
                    readRes = GetCmd((char *)&ATCommands_cmdBuffer[offset],
                                     dataResidue + 1, NULL);
                    if (readRes == 0)
                    {
                        //Report("\n\r");
                        continue;
                    }
                    else if (readRes == COMMAND_TOO_LONG)
                    {
                        Report("\n\rERROR: Command buffer overflow\n\r");
                        validCmd = 0;
                        break;
                    }
                    else if (readRes == COMMAND_END_OF_TEXT)
                    {
                        // Report("\n\rCommand send(to) cancelled\n\r");
                        validCmd = 0;
                        break;
                    }

                    dataResidue -= readRes;
                    offset += readRes;
                }
            }
        }

        Report("\n\r");

        if (validCmd)
        {
            ATCmd_send(ATCommands_cmdBuffer);
        }
    }

    return 0;
}

//*****************************************************************************
//
//! \brief determine event type and send it to the UART
//!
//! \param  event buffer
//!
//! \return
//!
//*****************************************************************************
int32_t ATCommands_sendDataToUart(char *buffer)
{
    uint32_t i;
    char *pbuf = NULL;
    uint8_t format = ATCMD_DATA_FORMAT_BASE64;
    uint16_t length;
    char tmpBuf[8], *pTmpBuf;

    /* recv */
    /* recv from */
    if ((strstr(buffer, ATCmd_sockRecvStr)) ||
#if 0
        (strstr(buffer, ATCmd_sockRecvFromStr)) ||
        (strstr(buffer, ATCmd_httpGetHeaderStr)))
#endif
        (strstr(buffer, ATCmd_sockRecvFromStr)))
    {
        /* Skip sendto [sd] argument */
        pbuf = strchr(buffer, ATCMD_DELIM_ARG) + 1;
    }

#if 0
    /* file read */
    else if(strstr(buffer, ATCmd_fileReadStr))
    {
        pbuf = strchr(buffer,ATCMD_DELIM_EVENT) + 1;
    }

    /* netapp recv */
    else if((strstr(buffer, ATCmd_netappRecvStr)) ||
       (strstr(buffer, ATCmd_httpReadResBodyStr)))
    {
        pbuf = buffer;
        for(i = 0; i < 2; i++)
        {
            pbuf = strchr(pbuf,ATCMD_DELIM_ARG) + 1;
            if(pbuf == NULL)
            {
                break;
            }
        }
    }

    /* netutil cmd */
    else if((strstr(buffer, ATCmd_netUtilCmdStr)) ||
       (strstr(buffer, ATCmd_netUtilGetStr)))
    {
        /* only netutil cmd sign_msg and netutil
        get public key return more then 1 parameter */
        /* therefore look for argument delimiter */
        pbuf = strchr(buffer,ATCMD_DELIM_ARG);
        if(pbuf != NULL)
        {
            pbuf = strchr(buffer,ATCMD_DELIM_EVENT) + 1;
        }
    }

    /* mqtt recv event */
    else if((strstr(buffer, ATCmd_eventMqttStr)) &&
       (strstr(buffer, ATCmd_mqttEventId[1].str)))
    {
        pbuf = buffer;
        for(i = 0; i < 5; i++)
        {
            pbuf = strchr(pbuf,ATCMD_DELIM_ARG) + 1;
            if(pbuf == NULL)
            {
                break;
            }
        }
    }
#endif

    if (pbuf != NULL)
    {
        pTmpBuf = tmpBuf;
        strncpy(pTmpBuf, pbuf, sizeof(tmpBuf) - 1);
        pTmpBuf[sizeof(tmpBuf) - 1] = '\0';

        /* format */
        StrMpl_getVal(&pTmpBuf, &format, ATCMD_DELIM_ARG, STRMPL_FLAG_PARAM_SIZE_8);
        if (format == ATCMD_DATA_FORMAT_BINARY)
        {
            /* data length */
            StrMpl_getVal(&pTmpBuf, &length, ATCMD_DELIM_ARG, STRMPL_FLAG_PARAM_SIZE_16);
            length += ((pbuf - buffer) + (pTmpBuf - tmpBuf));

            /* send binary data to the uart */
            for (i = 0; i < length; i++)
            {
                putch(buffer[i]);
            }
            UART_PRINT("\n\r");
            return(0);
        }
    }
    writeStr(buffer);
    writeStr("\r\n");

    return(0);
}

//*****************************************************************************
//
//! ATCommands_eventTask
//!
//!  \param  pvParameters
//!
//!  \return none
//!
//!  \brief   AT event Task handler function to receive inputs
//
//*****************************************************************************
void *ATCommands_eventTask(void *pvParameters)
{
    int status;

    while (1)
    {
        status = ATCmd_recv(ATCommands_eventBuffer, 0);
        if (status >= 0)
        {
            ATCommands_sendDataToUart(ATCommands_eventBuffer);
        }
    }
}

//*****************************************************************************
//
//! \brief Callback from the Network Stack to deliver status to the application.
//!        Called in this example when IP is acquired.
//!
//! \param[in]  roleId    - Role for which the network status callback is called.
//! \param[in]  ipAddress - Network i/f IP address.
//!
//! \return None
//!
//*****************************************************************************
void ATCommands_NetworkStatusCallback(WlanRole_e roleId, uint32_t ipAddress)
{
    struct netif *stateNetif = NULL;
    const ip4_addr_t *temp;

    if (roleId == WLAN_ROLE_STA)
    {
        stateNetif = (struct netif *)network_get_sta_if();
    }
    else if (roleId == WLAN_ROLE_AP)
    {
        stateNetif = (struct netif *)network_get_ap_if();
    }

    if (netif_is_up(stateNetif))
    {
        temp = netif_ip4_addr(stateNetif);
        if (temp->addr)
        {
            ATCmdEvent_compose((void *)stateNetif,
                               sizeof(struct netif),
                               ATCmdEvent_networkCallback, 0);
        }
    }
}

//*****************************************************************************
//
//! \brief Callback from the Wi-Fi Stack to deliver events to the application.
//!        This is registered via Wlan_Start(WlanStackEventHandler)
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void WlanStackEventHandler(WlanEvent_t *pWlanEvent)
{
    void *staif = NULL;

    if (!pWlanEvent)
    {
        return;
    }

    Report("\n\r--> WlanStackEventHandler Id = %d\n\r", pWlanEvent->Id);

    switch (pWlanEvent->Id)
    {
        case WLAN_EVENT_CONNECT:
        {
            WlanEventConnect_t *pWlanEventConnect = &pWlanEvent->Data.Connect;

            if (pWlanEventConnect->Status >= 0)
            {
                SET_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);
    
                 /* Copy new connection SSID and BSSID to global parameters */
                os_memcpy(app_CB.CON_CB.ConnectionSSID,
                          pWlanEventConnect->SsidName,
                          pWlanEventConnect->SsidLen);
                os_memcpy(app_CB.CON_CB.ConnectionBSSID,
                          pWlanEventConnect->Bssid,
                          WLAN_BSSID_LENGTH);
    
                staif = network_get_sta_if();
                if (staif != NULL)
                {
                    network_set_up(staif);
                }
            }

            osi_SyncObjSignal(&app_CB.CON_CB.connectEventSyncObj);

            /* Send ATCmd event notifying that the connection attempt has succeded */
            ATCmdEvent_compose(pWlanEvent, sizeof(WlanEvent_t), ATCmdEvent_wlanCallback, 0);
        }
        break;

        case WLAN_EVENT_DISCONNECT:
        {
            CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);
            CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);
            CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IPV6_ACQUIRED);

            staif = network_get_sta_if();
            if (staif != NULL)
            {
                network_set_down(staif);
            }

            osi_SyncObjSignal(&app_CB.CON_CB.disconnectEventSyncObj);
            
            /* Send ATCmd event notifying about wlan disconnection */
            ATCmdEvent_compose(pWlanEvent, sizeof(WlanEvent_t), ATCmdEvent_wlanCallback, 0);
        }
        break;
        case WLAN_EVENT_SCAN_RESULT:
        {
            WlanEventScanResult_t *pEventScanResult = &pWlanEvent->Data.ScanResult;
            uint32_t numResults = pEventScanResult->NetworkListResultLen;

            for (int index = 0; index < numResults ; index++)
            {
                /* Compose an event for each scan results */
                ATCmdEvent_compose(pWlanEvent, sizeof(WlanEvent_t), ATCmdEvent_wlanCallback, index);
            }
        }
        break;
        case WLAN_EVENT_ADD_PEER:
        {
            SET_STATUS_BIT(app_CB.Status, STATUS_BIT_PEER_CONNECTED);

            ATCmdEvent_compose(pWlanEvent, sizeof(WlanEvent_t), ATCmdEvent_wlanCallback, 0);
        }
        break;
        case WLAN_EVENT_REMOVE_PEER:
        {
            app_CB.ConnectedStations--;
            if (app_CB.ConnectedStations == 0)
            {
                CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_PEER_CONNECTED);
            }

            ATCmdEvent_compose(pWlanEvent, sizeof(WlanEvent_t), ATCmdEvent_wlanCallback, 0);
        }
        break;
        case WLAN_EVENT_CONNECTING:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_CONNECTING\n\r");
        }
        break;
        case WLAN_EVENT_ACTION_FRAME_RX:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_ACTION_FRAME_RX\n\r");
        }
        break;
        case WLAN_EVENT_ASSOCIATED:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_ASSOCIATED\n\r");
        }
        break;
        case WLAN_EVENT_AP_EXT_WPS_SETTING_FAILED:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_AP_EXT_WPS_SETTING_FAILED\n\r");
        }
        break;
        case WLAN_EVENT_BLE_ENABLED:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_BLE_ENABLED\n\r");
        }
        break;
        case WLAN_EVENT_CS_FINISH:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_CS_FINISH\n\r");
        }
        break;
        case WLAN_EVENT_ROC_DONE:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_ROC_DONE\n\r");
        }
        break;
        case WLAN_EVENT_CROC_DONE:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_CROC_DONE\n\r");
        }
        break;
        case WLAN_EVENT_SEND_ACTION_DONE:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_SEND_ACTION_DONE\n\r");
        }
        break;
        case WLAN_EVENT_CONNECT_PERIODIC_SCAN_COMPLETE:
        {
            Report("\n\r--> WlanStackEventHandler WLAN_EVENT_CONNECT_PERIODIC_SCAN_COMPLETE\n\r");
        }
        break;
        default:
        {
            Report("\n\r--> WlanStackEventHandler !! UNEXPECTED EVENT!!\n\r");
        }
        break;
    }
}

//*****************************************************************************
//
//! mainThread
//!
//!  \param  none
//!
//!  \return Upon successful completion, the function shall return 0.
//!          In case of failure, this function would return -1.
//!
//!  \brief  Initialize Application's Variables
//
//*****************************************************************************
int32_t initAppVariables(void)
{
    int32_t ret = 0;

    app_CB.Status = 0 ;
    app_CB.Role = WLAN_ROLE_RESERVED;
    app_CB.Exit = FALSE;

    memset(&app_CB.CmdBuffer, 0x0, CMD_BUFFER_LEN);
    memset(&app_CB.gDataBuffer, 0x0, sizeof(app_CB.gDataBuffer));
    memset(&app_CB.CON_CB, 0x0, sizeof(app_CB.CON_CB));

    ret = osi_SyncObjCreate(&app_CB.CON_CB.disconnectEventSyncObj);
    if (ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return -1;
    }

    ret = osi_SyncObjCreate(&app_CB.CON_CB.connectEventSyncObj);
    if (ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return -1;
    }

    ret = osi_SyncObjCreate(&app_CB.CON_CB.eventCompletedSyncObj);
    if (ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return -1;
    }
    ret = osi_SyncObjCreate(&app_CB.eventCompletedScanObj);
    if (ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return -1;
    }
    ret = osi_SyncObjCreate(&app_CB.CON_CB.dhcpIprecvSyncObj);
    if (ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return -1;
    }
    ret = osi_SyncObjCreate(&app_CB.CON_CB.staRoleupSyncObj);
    if (ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return -1;
    }

    return ret;
}

//*****************************************************************************
//
//! mainThread
//!
//!  \param  pvParameters
//!
//!  \return none
//!
//!  \brief Task handler
//
//*****************************************************************************
void *mainThread(void *pvParameters)
{
    int32_t status = 0;
    pthread_attr_t pAttrs;
    struct sched_param priParam;

    //uint32_t ticksToSleep;
    HWREG(ICACHE_BASE + 0x84) |= 0x00000001  ;//OSPREY_MX-38
    HWREG(ICACHE_BASE + 0x4) |= 0xc0000000  ;//OSPREY_MX-38
    //HWREG(ICACHE_BASE + 0x4) |= 0x80000000  ;//OSPREY_MX-38, this is for 64M cache, instead CRAM

#ifdef AT_COMMANDS_TERMINAL_TAB_COMPLETION
    initCompletions();
#endif

    status = initAppVariables();
    if (status != 0)
    {
        return 0;
    }
    network_stack_init();
    network_stack_register_extra_status_callback(ATCommands_NetworkStatusCallback);

    InitTerm();
    UART_PRINT("\r\n");

    /* Create AT Command module */
    ATCmd_create();

    ATCommands_displayBanner();

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 5;
    status = pthread_attr_setschedparam(&pAttrs, &priParam);
    status |= pthread_attr_setstacksize(&pAttrs, ATCOMMANDS_TASK_STACK_SIZE);

    status = pthread_create(&ATCommands_eventThread,
                            &pAttrs,
                            ATCommands_eventTask,
                            NULL);
    if (status != 0)
    {
        UART_PRINT("ATCommands_eventThread creation failed\n\r");
        /* error handling */
        while (1)
        {
            ;
        }
    }

    ATCommands_readCmd();

#ifdef AT_COMMANDS_TERMINAL_TAB_COMPLETION
    freeCompletionArray();
#endif

    return (0);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
