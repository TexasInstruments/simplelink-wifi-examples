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

#ifndef __BLE_DTM_APP_H__
#define __BLE_DTM_APP_H__

#include <stdint.h>
#include <wlan_if.h>
#include "osi_kernel.h"
#include "upper_mac_versions.h"
#include "ble_if.h"

#ifdef CC35XX
#define MAX_BUF_SIZE            (1470)
#endif // CC35XX
#ifdef CC33XX
#define MAX_BUF_SIZE            (1200)
#endif // CC33XX
#define CMD_BUFFER_LEN          (256)
#define MAX_CMD_NAME_LEN        (32)
#define APPLICATION_NAME        ("BLE DTM")
#define APPLICATION_VERSION     (version_upper_mac)
#define TASK_STACK_SIZE         (2048)
#define SPAWN_TASK_PRIORITY     (9)

/* BLE DTM Errors */
#define DTM_OSI_ERROR_BASE                     (-1000L)
#define DTM_OPERATION_FAILED                   (-1001L)
#define DTM_TIMEOUT                            (-1005L)

#define OS_ERROR_MSG            ( \
        "OS error, please refer \"OSI ERRORS CODES\" section in errno.h")

/* Command action structure */
typedef struct cmdAction
{
    const char        *cmd;
    int32_t    (*callback)(void *);
    int32_t    (*printusagecallback)(void *);
} cmdAction_t;

/* Connection control block */
typedef struct connectionControlBlock_t
{
    OsiSyncObj_t    eventCompletedSyncObj;
} connection_CB;

/* Application control block */
typedef struct appControlBlock_t
{
    /* Status Variables */
    uint32_t Status;
    /* This flag lets the application to exit */
    uint32_t Exit;
    /* Cmd Prompt buffer */
    uint8_t CmdBuffer[CMD_BUFFER_LEN];
    /* STA/AP mode CB */
    connection_CB CON_CB;
} appControlBlock;

extern appControlBlock app_CB;

/* Status keeping MACROS */
#define SET_STATUS_BIT(status_variable, bit) status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit) status_variable &= ~(1<<(bit))
#define GET_STATUS_BIT(status_variable, bit) (0 != (status_variable & (1<<(bit))))

typedef enum
{
    STATUS_BIT_NWP_INIT = 0,
    STATUS_BIT_BLE_ENABLED
} e_StatusBits;

#define IS_NW_PROCSR_ON(status_variable)     \
                GET_STATUS_BIT(status_variable, STATUS_BIT_NWP_INIT)

#define IS_BLE_ENABLED(status_variable)      \
                GET_STATUS_BIT(status_variable, STATUS_BIT_BLE_ENABLED)

#define SHOW_WARNING(ret, errortype)        UART_PRINT( \
        "\n\r[line:%d, error code:%d] %s\n\r", __LINE__, ret, errortype);

#define ASSERT_ON_ERROR(ret, errortype)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                return -1;\
            }\
        }

/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/

#define WLAN_MAC_ADDR_LEN                              (6)
#define WLAN_IPV4_BYTE(val,index)                      ( (val >> (index*8)) & 0xFF )

/*****************************************************************************/
/* Function declarations                                                     */
/*****************************************************************************/

void PrintIPAddress(unsigned char ipv6, void *ip);

/* App command callbacks */
int32_t cmdHelpCallback(void *arg);
int32_t cmdClearcallback(void *arg);

/* App usage print functions */
int32_t printHelpUsage(void *arg);
int32_t printClearUsage(void *arg);

#endif /* __BLE_DTM_APP_H__ */
