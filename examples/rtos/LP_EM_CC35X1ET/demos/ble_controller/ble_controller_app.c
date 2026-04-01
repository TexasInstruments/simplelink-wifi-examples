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

#include "ble_controller_app.h"
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
#include "uart_npi.h"

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
#define SIX_BYTES_SIZE_MAC_ADDRESS  (17)
#define APP_MCSPI_MSGSIZE       (100U)

/****************************************************************************
                      LOCAL FUNCTION PROTOTYPES
****************************************************************************/
int32_t initAppVariables();
int32_t ble_controller_app(void);

/****************************************************************************
                      GLOBAL VARIABLES
****************************************************************************/
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
    if(!pWlanEvent)
    {
        return;
    }
    switch(pWlanEvent->Id)
    {
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
            UART_PRINT("\n\r[HANDLER] WLAN_EVENT_FW_CRASH FW is crashed \n\r");
        }
        break;
        default:
        {
            UART_PRINT("\n\r[WLAN EVENT HANDLER] Unexpected event [0x%x]\n\r",
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


void *mycalloc(uint32_t memb,size_t len)
{
    return 0;
}


void *myzalloc(size_t len)
{
    return 0;
}


//OSPREY_MX-38
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x))) //TODO temporary need to be removed
#define ICACHE_BASE 0x41902000  //TODO temporary need to be removed, only for M3, M$ has different address


void *ble_controller_entry(void *args)
{
#ifdef CC33XX
    int32_t             RetVal = -1;

    /* init drivers and services */
    Drivers_open();
    Board_driversOpen();

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

#ifdef TERMINAL_TAB_COMPLETION
    initCompletions();
#endif

    /* Init Application variables */
    initAppVariables();
    network_stack_init();
    // init the terminal
    InitTerm();
#endif // CC35XX


    /* Output device information to the UART terminal */
    RetVal = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    if(RetVal < 0)
    {
        /* Handle Error */
        UART_PRINT(
            "BLE Controller - Unable to retrieve device information \n");
        return(NULL);
    }

    // Start WLAN
    RetVal = cmdWlanStartCallback("");
    if(RetVal != 0)
    {
        /* Handle Error */
        UART_PRINT("ERROR: error with start WLAN.\n");
        return(NULL);
    }
    os_sleep(1, 0);

    // Set Always Active Power Mode
    RetVal = cmdSetPmModeCallback("-m 0");
    if(RetVal != 0)
    {
        /* Handle Error */
        UART_PRINT("ERROR: error with Set PM Mode.\n");
        return(NULL);
    }
    os_sleep(1, 0);

    // Read FW Version
    RetVal = cmdGetFwVerCallback("");
    if(RetVal != 0)
    {
        /* Handle Error */
        UART_PRINT("ERROR: error with reading FW version.\n");
        return(NULL);
    }
    os_sleep(1, 0);

    // Start BLE Controller
    RetVal = BleIf_EnableBLE();
    if(RetVal != 0)
    {
        /* Handle Error */
        UART_PRINT("ERROR: error with start BLE controller.\n");
        return(NULL);
    }

    os_sleep(1, 0);
    /*
     * Calling UART handling method which serves as the application main loop.
     * Note that this function doesn't return.
     */
    RetVal = ble_controller_app();
    return NULL;
}


#ifdef CC35XX
void *mainThread(void *args)
{
    ble_controller_entry(NULL);
    return NULL;
}
#endif // CC35XX
/***
 *     \brief          ble_controller_app.

    This routine read hci packets from UART interface,
    and send the packet to host adaptation layer.
    This function isn't expected to return.

    \param          arg.

    \return

    \sa             GetCmd
 */

int32_t ble_controller_app(void)
{
    Report("\n\rBLE Controller Example Start\n\r");

    //Open BLE transport
    BleIf_OpenTransport();

    //Open the UART for hci packets from the NPI.
    NpiIf_OpenNpi();

    //Register the callback for writing received packets from the NPI uart
    NpiIf_EventCbRegister(BleIf_SendCommand);

    //Register the callback for writing received packets from the controller
    BleIf_EventCbRegister(NpiIf_SendCommand);

    return(0);
}
