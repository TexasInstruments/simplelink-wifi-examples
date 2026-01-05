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
#include "ble_wifi_provisioning.h"
#include <stdlib.h>
#include "FreeRTOS.h"

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
#include "wlan_cmd.h"
#include "ble_cmd.h"
#include "wlan_if.h"
#include "osi_kernel.h"
#include "uart_term.h"

//NIMBLE
#include "nimble_host_provisioning.h"

//LWIP
#include "network_lwip.h"

//SOCKET
#include "socket_examples.h"

//ERRORS
#include "errors.h"

// Debug for total allocation
#ifdef PRINT_DBG_TOTAL_MALLOC_FREE
extern volatile UINT32 totalloc;
#endif

#ifdef CC35XX
#define WLAN_REASON_DEAUTH_LEAVING 3
#define WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY 4
#endif // CC35XX

/* Application defines */

#define APP_MCSPI_MSGSIZE       (100U)
#define WLAN_SEC_TYPE_UNDEFINED 0xFF

/****************************************************************************
                      LOCAL FUNCTION PROTOTYPES
****************************************************************************/
int32_t initAppVariables();
int32_t initProvVariables(void);
int32_t ble_wifi_provisioning_app(void);

/****************************************************************************
                      GLOBAL VARIABLES
****************************************************************************/
appControlBlock app_CB;
provControlBlock prov_CB;

/*!
    \brief          WlanStackEventHandler

    This handler gets called whenever a WLAN event is reported
    by the host driver / NWP. Here user can implement he's own logic
    for any of these events. This handler is used by 'ble_wifi_provisioning'
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
                        "\n\r\n\r[WLAN EVENT HANDLER] connection failed (probably due to Timeout)");
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

        //Notify WLAN is connected
        nimble_host_gatt_svr_chr_notify_wlan_connection(WLAN_CONNECT_SUCCESS);

        //Clear WLAN Credentials
        provClearCredentials();

        osi_SyncObjSignal(&app_CB.CON_CB.connectEventSyncObj);

    }
    break;

    case WLAN_EVENT_DISCONNECT:
    {
        WlanEventDisconnect_t  *pEventData = NULL;

        Report("\n\r\n\r[WLAN EVENT HANDLER] WLAN_EVENT_DISCONNECT ");
        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);
        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);
        CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IPV6_ACQUIRED);

        killAllProcess();
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

        //Notify WLAN is idle
        nimble_host_gatt_svr_chr_notify_wlan_connection(WLAN_CONNECT_IDLE);

        //Clear WLAN Credentials
        provClearCredentials();

        osi_SyncObjSignal(&app_CB.CON_CB.disconnectEventSyncObj);
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

    case WLAN_EVENT_BLE_ENABLED:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] BLE Enabled \n\r");
    }
    break;

    case WLAN_EVENT_ERROR:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_ERROR error: 0x%X \n\r", pWlanEvent->Data.error.error_num);
    }
    break;

    case WLAN_EVENT_FW_CRASH:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] WLAN_EVENT_FW_CRASH FW is crashed \n\r");
    }
    break;

    default:
    {
        UART_PRINT("\n\r[WLAN EVENT HANDLER] Unexpected event [%d]\n\r",
                   pWlanEvent->Id);
    }
    break;
    }
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
    UART_PRINT("******************************************************************\r\n");
    UART_PRINT("***************** %-28s *******************\r\n", APPLICATION_NAME);
    UART_PRINT("***************** %-28s *******************\r\n", APPLICATION_VERSION);
    UART_PRINT("******************************************************************\r\n");
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

    ret = osi_SyncObjCreate(&app_CB.CON_CB.disconnectEventSyncObj);
     if(ret != 0)
     {
         SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
         return(-1);
     }

     ret = osi_SyncObjCreate(&app_CB.CON_CB.connectEventSyncObj);
     if(ret != 0)
     {
         SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
         return(-1);
     }

     ret = osi_SyncObjCreate(&app_CB.CON_CB.eventCompletedSyncObj);
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
     ret = osi_SyncObjCreate(&app_CB.CON_CB.dhcpIprecvSyncObj);
     if(ret != 0)
     {
         SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
         return(-1);
     }
     ret = osi_SyncObjCreate(&app_CB.CON_CB.staRoleupSyncObj);
     if(ret != 0)
     {
         SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
         return(-1);
     }
     ret = osi_SyncObjCreate(&app_CB.CON_CB.staRoledownSyncObj);
     if(ret != 0)
     {
         SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
         return(-1);
     }

     return(ret);
}


int32_t    initProvVariables(void)
{
    int32_t ret = 0;

    provClearCredentials();

    ret = osi_SyncObjCreate(&prov_CB.StartConnectEventSyncObj);
    if(ret != 0)
    {
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }

    return(ret);
}

void provSaveSSID(uint8_t *data, uint8_t length)
{
    os_memset(prov_CB.wlanSSID,0,WLAN_SSID_MAX_LENGTH); //Clear buffer

    if (length > WLAN_SSID_MAX_LENGTH)
    {
        length = WLAN_SSID_MAX_LENGTH;
    }

    //Copy the SSID
    os_memcpy(prov_CB.wlanSSID,data,length);

    //Init the password flag
    prov_CB.wlanPasswordSet = FALSE;

    Report("\r\n[BLE GATT]: WLAN SSID Received");
}

void provSavePassword(uint8_t *data, uint8_t length)
{

    os_memset(prov_CB.wlanPASSWORD,0,PASSWD_LEN_MAX); //Clear buffer

    if (length > PASSWD_LEN_MAX)
    {
        length = PASSWD_LEN_MAX;
    }

    //Copy the PASSWORD
    os_memcpy(prov_CB.wlanPASSWORD,data,length);

    //Set the PASSWORD length
    prov_CB.wlanPasswordLength = length;

    //Set the PASSWORD flag
    prov_CB.wlanPasswordSet = TRUE;

    Report("\r\n[BLE GATT]: WLAN Password Received");
}

void provSaveSecurity(uint8_t *data, uint8_t length)
{
    //Copy the SECURITY type
    prov_CB.wlanSecurity = data[0];

    Report("\r\n[BLE GATT]: WLAN Security Type Received");
}

void provClearCredentials(void)
{
    os_memset(prov_CB.wlanSSID,0,WLAN_SSID_MAX_LENGTH);
    os_memset(prov_CB.wlanPASSWORD,0,PASSWD_LEN_MAX);
    prov_CB.wlanPasswordLength = 0;
    prov_CB.wlanPasswordSet = FALSE;
    prov_CB.wlanSecurity = WLAN_SEC_TYPE_UNDEFINED;
}

void wlanStart(void)
{
    cmdWlanStartCallback("");
}

void wlanRoleUpSta(void)
{
    cmdWlanRoleUpStaCallback("");
}

int32_t wlanConnect(void)
{
    int rc;
    char *wlanConnTemplateOPEN = " -s \"%s\" -t OPEN";
    char *wlanConnTemplateWPA2 = " -s \"%s\" -t WPA2 -p %s";
    char *wlanConnTemplateWPA3 = " -s \"%s\" -t WPA3 -p %s";
    char wlanConnStr[CMD_BUFFER_LEN];

    //Check if security type was set
    if (prov_CB.wlanSecurity == WLAN_SEC_TYPE_UNDEFINED)
    {
        Report("\r\n[WLAN][ERROR] Security type was not set\r\n");
        return(-1);
    }

    //Check if security type requires password
    if (prov_CB.wlanSecurity != WLAN_SEC_TYPE_OPEN)
    {
        //Check if password is required and was not set
        if (prov_CB.wlanPasswordSet == FALSE)
        {
            Report("\r\n[WLAN][ERROR] No password was set. Security type %d requires password\r\n", prov_CB.wlanSecurity);
            return(-1);
        }
        else //Check password length
        {
            if ((prov_CB.wlanPasswordLength > PASSWD_LEN_MAX) ||
                (prov_CB.wlanPasswordLength < PASSWD_LEN_MIN))
            {
                Report("\r\n[WLAN][ERROR] Invalid password length %d. Security type %d requires password length to be in range [%d,%d]\r\n",
                       prov_CB.wlanPasswordLength,prov_CB.wlanSecurity,PASSWD_LEN_MIN,PASSWD_LEN_MAX);
                return(-1);
            }
        }
    }

    //Check the security type
    switch(prov_CB.wlanSecurity)
    {
        case WLAN_SEC_TYPE_OPEN:
        {
            sprintf(wlanConnStr, wlanConnTemplateOPEN, prov_CB.wlanSSID);
            break;
        }
        case WLAN_SEC_TYPE_WPA_WPA2:
        {
            sprintf(wlanConnStr, wlanConnTemplateWPA2, prov_CB.wlanSSID, prov_CB.wlanPASSWORD);
            break;
        }
        case WLAN_SEC_TYPE_WPA3:
        {
            sprintf(wlanConnStr, wlanConnTemplateWPA3, prov_CB.wlanSSID, prov_CB.wlanPASSWORD);
            break;
        }
        default:
        {
            Report("\r\n[WLAN][ERROR] Unsupported security type %d\r\n", prov_CB.wlanSecurity);
            return(-1);
        }
    }

    //Notify WLAN is connecting
    nimble_host_gatt_svr_chr_notify_wlan_connection(WLAN_CONNECT_START);

    Report("\r\n[WLAN] Trying to connect...\r\n");
    rc = cmdWlanConnectCallback(wlanConnStr);

    return rc;
}

void bleStart(void)
{
    cmdBleStartCallback("");
}

void advConfigure(void)
{
    ExtAdvCfg_t extAdvParams;
    
    extAdvParams.instance = 0;
    extAdvParams.legacy = 1;
    extAdvParams.interval_ms = 100;
    extAdvParams.prim_phy = 0x1;
    extAdvParams.sec_phy = 0x1;

    nimble_host_ext_adv_cfg(&extAdvParams);
}

void advEnable(void)
{
    ExtAdvEnable_t extAdvEnable;

    extAdvEnable.enable = 1;
    extAdvEnable.instance = 0;
    extAdvEnable.duration = 0;
    extAdvEnable.max_events = 0;

    nimble_host_ext_adv_enable(&extAdvEnable);
}


//OSPREY_MX-38
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x))) //TODO temporary need to be removed
#define ICACHE_BASE 0x41902000  //TODO temporary need to be removed, only for M3, M$ has different address


void *ble_wifi_provisioning_entry(void *args)
{
#ifdef CC33XX
    int32_t RetVal = -1;

    /* init drivers and services */
    Drivers_open();
    Board_driversOpen();

    // init the terminal
    InitTerm();

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

    /* Init Application variables */
    initAppVariables();
    network_stack_init(); //initialize lwip

    // init the terminal
    InitTerm();
#endif // CC35XX


    /* Output device information to the UART terminal */
    RetVal = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    if(RetVal < 0)
    {
        /* Handle Error */
        UART_PRINT(
            "Network Terminal - Unable to retrieve device information \n");
        return(NULL);
    }

    /* Turn on WiFi chip and set SPI interface */
    extern void wlan_TurnOffWlan();
    wlan_TurnOffWlan();

    /* Start WLAN */
    wlanStart();

    os_sleep(1, 0);

    /* Enable WLAN STA Role */
    wlanRoleUpSta();

    os_sleep(1, 0);

    /* Start BLE */
    bleStart();

    os_sleep(1, 0);

    /*
     * Calling method which serves as the application main loop.
     * Note that this function doesn't return.
     */
    RetVal = ble_wifi_provisioning_app();
#ifdef CC33XX
    Board_driversClose();
    Drivers_close();
#endif // CC33XX
    return NULL;
}


#ifdef CC35XX
void *mainThread(void *args)
{
    ble_wifi_provisioning_entry(NULL);
    return NULL;
}
#endif // CC35XX
/***
 *     \brief          ble_wifi_provisioning_app.

    This routine start ble advertisement to establish
    ble connection in order to get WLAN credentials.

    \param          arg.

    \return

    \sa             GetCmd
 */

int32_t ble_wifi_provisioning_app(void)
{
    int32_t RetVal = -1;

    /* Init Provisioning Variables */
    initProvVariables();

    /* Configure BLE ADV */
    advConfigure();

    /* Enable BLE ADV */
    advEnable();

    /* Wait for the WLAN credentials to be sent over BLE */
    UART_PRINT("\r\nUse the mobile application in order to set the WLAN AP credentials which the device will connect to.");

    while (1)
    {
        //Wait for WLAN credentials
        UART_PRINT("\r\nWaiting for WLAN credentials...\r\n");
        if (osi_SyncObjWait(&prov_CB.StartConnectEventSyncObj, OSI_WAIT_FOREVER) == OSI_OK)
        {
            UART_PRINT("\r\nWLAN connection request sent successfully\r\n");

            //WLAN connection attempt
            RetVal = wlanConnect();
            if (RetVal != 0)
            {
                nimble_host_gatt_svr_chr_notify_wlan_connection(WLAN_CONNECT_FAILURE);
                UART_PRINT("\r\nWLAN connection failed. Please re-send the WLAN credentials.\r\n");
            }
        }
        else
        {
            UART_PRINT("\r\nWaiting for WLAN credentials timed out. Please re-send the WLAN credentials.\r\n");
        }
    }
}

