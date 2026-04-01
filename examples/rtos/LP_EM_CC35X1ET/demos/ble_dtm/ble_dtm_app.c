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

#include "ble_dtm_app.h"
#include <stdlib.h>
#include <string.h>
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
#include "wlan_if.h"
#include "osi_kernel.h"
#include "uart_term.h"

/* DTM module */
#include "dtm_cmd.h"

/* LWIP */
#include "network_lwip.h"

/* ERRORS */
#include "errors.h"

/****************************************************************************
                      LOCAL FUNCTION PROTOTYPES
****************************************************************************/
int32_t initAppVariables(void);
int32_t ble_dtm_app(void);
int32_t cmd_prompt(void *arg);
int32_t showAvailableCmd(void);

/****************************************************************************
                      STRING DEFINITIONS
****************************************************************************/
static const char lineBreak[]           = "\n\r";
static const char cmdPromptStr[]        = "> ";
static const char helpStr[]             = "help";
static const char clearStr[]            = "clear";
static const char dtmRxStr[]            = "dtm_rx_test";
static const char dtmTxStr[]            = "dtm_tx_test";
static const char dtmEndStr[]           = "dtm_test_end";
static const char dtmSetMaxTxPwrStr[]   = "dtm_max_tx_pwr";
static const char usageStr[]            = "Usage: ";
static const char descriptionStr[]      = "Description: ";

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
{ helpStr,              cmdHelpCallback,            printHelpUsage      },
{ clearStr,             cmdClearcallback,           printClearUsage     },
{ dtmRxStr,             cmdDtmRxCallback,           printDtmRxUsage     },
{ dtmTxStr,             cmdDtmTxCallback,           printDtmTxUsage     },
{ dtmEndStr,            cmdDtmEndCallback,          printDtmEndUsage    },
{ dtmSetMaxTxPwrStr,    cmdDtmSetMaxTxPowerCallback, printDtmSetMaxTxPowerUsage },
};

uint32_t            gMaxCmd = (sizeof(gCmdList)/sizeof(cmdAction_t));
appControlBlock     app_CB;

//OSPREY_MX-38
#define HWREG(x)    (*((volatile unsigned long *)(x)))
#define ICACHE_BASE 0x41902000

/****************************************************************************
                      WLAN EVENT HANDLER
****************************************************************************/
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
            UART_PRINT("\n\r[WLAN EVENT] BLE Enabled \n\r");
            SET_STATUS_BIT(app_CB.Status, STATUS_BIT_BLE_ENABLED);
        }
        break;
        case WLAN_EVENT_ERROR:
        {
            UART_PRINT("\n\r[WLAN EVENT] Error: 0x%X \n\r", pWlanEvent->Data.error.error_num);
        }
        break;
        case WLAN_EVENT_FW_CRASH:
        {
            UART_PRINT("\n\r[WLAN EVENT] FW Crash \n\r");
        }
        break;
        default:
        {
            UART_PRINT("\n\r[WLAN EVENT] Unexpected event [0x%x]\n\r", pWlanEvent->Id);
        }
        break;
    }
}

/****************************************************************************
                      UTILITY FUNCTIONS
****************************************************************************/
void PrintIPAddress(unsigned char ipv6, void *ip)
{
    uint32_t *pIPv4;
    if(!ip) return;

    if(!ipv6)
    {
        pIPv4 = (uint32_t*)ip;
        UART_PRINT("%d.%d.%d.%d",
                    WLAN_IPV4_BYTE(*pIPv4,3),
                    WLAN_IPV4_BYTE(*pIPv4,2),
                    WLAN_IPV4_BYTE(*pIPv4,1),
                    WLAN_IPV4_BYTE(*pIPv4,0));
    }
}

int32_t DisplayAppBanner(char* appName, char* appVersion)
{
    UART_PRINT("******************************************************************\r\n");
    UART_PRINT("***************** %-28s *******************\r\n", APPLICATION_NAME);
    UART_PRINT("***************** %-28s *******************\r\n", APPLICATION_VERSION);
    UART_PRINT("******************************************************************\r\n");
#ifdef __clang__
    Report("Compiled with Clang\r\n");
#elif defined(__GNUC__)
    Report("Compiled with GCC\r\n");
#else
    printf("Compiled with an unknown compiler\r\n");
#endif
    return 0;
}

int32_t initAppVariables(void)
{
    int32_t ret = 0;

    app_CB.Status = 0;
    app_CB.Exit = FALSE;

    memset(&app_CB.CmdBuffer, 0x0, CMD_BUFFER_LEN);
    memset(&app_CB.CON_CB, 0x0, sizeof(app_CB.CON_CB));

    ret = osi_SyncObjCreate(&app_CB.CON_CB.eventCompletedSyncObj);
    if(ret != 0)
    {
        SHOW_WARNING(DTM_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return(-1);
    }

    /* Initialize DTM module */
    ret = DtmInit();
    if(ret != 0)
    {
        UART_PRINT("ERROR: Failed to initialize DTM module\n\r");
        return(-1);
    }

    return(ret);
}

/****************************************************************************
                      COMMAND CALLBACKS
****************************************************************************/
int32_t cmdHelpCallback(void *arg)
{
    uint32_t i = 0;
    char *param[2] = {NULL, NULL};
    char *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, " ");
    while(token && (i < 2))
    {
        param[i] = token;
        token = strtok(NULL, " ");
        i++;
    }

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

    return 0;
}

int32_t cmdClearcallback(void *arg)
{
#ifdef CC35XX
    ClearTerm();
#endif
    return 0;
}

/****************************************************************************
                      USAGE PRINT FUNCTIONS
****************************************************************************/
int32_t printHelpUsage(void *arg)
{
    UART_PRINT("\n\r%s\n\r", lineBreak);
    UART_PRINT("%shelp [command]\n\r", usageStr);
    UART_PRINT("%sDisplay help for commands\n\r", descriptionStr);
    UART_PRINT("%s\n\r", lineBreak);
    return 0;
}

int32_t printClearUsage(void *arg)
{
    UART_PRINT("\n\r%s\n\r", lineBreak);
    UART_PRINT("%sclear\n\r", usageStr);
    UART_PRINT("%sClear the terminal screen\n\r", descriptionStr);
    UART_PRINT("%s\n\r", lineBreak);
    return 0;
}

/****************************************************************************
                      COMMAND PROMPT
****************************************************************************/
int32_t showAvailableCmd(void)
{
    uint32_t i;

    UART_PRINT("\n\r");
    UART_PRINT("================================================================================\n\r");
    UART_PRINT("Available commands:\n\r");
    UART_PRINT("================================================================================\n\r");

    for(i = 0; i < gMaxCmd; i++)
    {
        if(!(i % 4)) UART_PRINT("\n\r");
        UART_PRINT("%-20s", gCmdList[i].cmd);
    }

    UART_PRINT("\n\r");
    UART_PRINT("================================================================================\n\r");
    return 0;
}

int32_t cmd_prompt(void *arg)
{
    int32_t lRetVal = 1;
    uint32_t i = 0;
    char cmdBuffer[(MAX_CMD_NAME_LEN + 5)];
    char *token = NULL;

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
            memcpy(cmdBuffer, &app_CB.CmdBuffer, (MAX_CMD_NAME_LEN + 4));
            cmdBuffer[MAX_CMD_NAME_LEN + 4] = '\0';
            token = strtok(cmdBuffer, " ");
            if(token)
            {
                for(i = 0; i < gMaxCmd; i++)
                {
                    if(!strcmp((char*)token, gCmdList[i].cmd))
                    {
                        lRetVal = gCmdList[i].callback((void *)(app_CB.CmdBuffer + strlen(token)));
                        UART_PRINT(lineBreak);
                        break;
                    }
                }
                if(i >= gMaxCmd)
                {
                    UART_PRINT(lineBreak);
                    UART_PRINT("No such command\n\r");
                    showAvailableCmd();
                }
            }
        }
    }
    return 0;
}

/****************************************************************************
                      APPLICATION ENTRY
****************************************************************************/
void *ble_dtm_entry(void *args)
{
#ifdef CC33XX
    int32_t RetVal = -1;

    Drivers_open();
    Board_driversOpen();

    InitTerm();
    initAppVariables();
    network_stack_init();

    extern void wlan_TurnOffWlan();
    wlan_TurnOffWlan();
#elif defined(CC35XX)
    int32_t RetVal = -1;
    HWREG(ICACHE_BASE + 0x84) |= 0x00000001;
    HWREG(ICACHE_BASE + 0x4) |= 0xc0000000;

    Board_init();

    initAppVariables();
    network_stack_init();
    InitTerm();
#endif

    RetVal = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);
    if(RetVal < 0)
    {
        UART_PRINT("BLE DTM - Unable to retrieve device information \n");
        return(NULL);
    }

    /* Start WLAN */
    UART_PRINT("\n\rStarting WLAN...\n\r");
    RetVal = cmdWlanStartCallback("");
    if(RetVal != 0)
    {
        UART_PRINT("ERROR: Failed to start WLAN.\n");
        return(NULL);
    }
    os_sleep(1, 0);

    /* Set Always Active Power Mode */
    RetVal = cmdSetPmModeCallback("-m 0");
    if(RetVal != 0)
    {
        UART_PRINT("ERROR: Failed to set PM Mode.\n");
        return(NULL);
    }
    os_sleep(1, 0);

    /* Read FW Version */
    RetVal = cmdGetFwVerCallback("");
    if(RetVal != 0)
    {
        UART_PRINT("ERROR: Failed to read FW version.\n");
        return(NULL);
    }
    os_sleep(1, 0);

    /* Start BLE Controller */
    UART_PRINT("\n\rEnabling BLE...\n\r");
    RetVal = BleIf_EnableBLE();
    if(RetVal != 0)
    {
        UART_PRINT("ERROR: Failed to enable BLE controller.\n");
        return(NULL);
    }
    os_sleep(1, 0);

    /*
     * Calling event handling method which serves as the application main loop.
     * Note that this function doesn't return.
     */
    RetVal = ble_dtm_app();
    return NULL;
}


#ifdef CC35XX
void *mainThread(void *args)
{
    ble_dtm_entry(NULL);
    return NULL;
}
#endif // CC35XX
/***
 *     \brief          ble_dtm_app.

    This routine read hci packets from interface,
    and handle the packet in the app layer.
    This function isn't expected to return.

    \param          arg.

    \return

    \sa             GetCmd
 */

int32_t ble_dtm_app(void)
{
    Report("\n\rBLE DTM Example Start\n\r");

    /* Open BLE transport */
    BleIf_OpenTransport();

    /* Register DTM event handler */
    DtmRegisterEventHandler();

    /* Show DTM User menu */
    showAvailableCmd();

    /* Enter command prompt loop */
    cmd_prompt(NULL);

    return(0);
}
