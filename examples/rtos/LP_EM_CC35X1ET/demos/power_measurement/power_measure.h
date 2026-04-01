/*
 * Copyright (c) 2016-2025, Texas Instruments Incorporated
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

#ifndef POWER_MEASURE_H_
#define POWER_MEASURE_H_

#include <stdint.h>
#include "wlan_if.h"
#include "network_terminal.h"

//*****************************************************************************
// Defines
//*****************************************************************************

/* Compatibility macros for status bits */
#define STATUS_BIT_CONNECTION       STATUS_BIT_STA_CONNECTION
#define IS_CONNECTED(status)        GET_STATUS_BIT(status, STATUS_BIT_STA_CONNECTION)

//*****************************************************************************
// Application Defines
//*****************************************************************************
#ifdef APPLICATION_NAME
    #undef APPLICATION_NAME
    #define APPLICATION_NAME        ("Power Measurement")
#endif

#ifdef APPLICATION_VERSION
    #undef APPLICATION_VERSION
    #define APPLICATION_VERSION     (version_upper_mac)
#endif

#define FRAME_LENGTH                (1000)
#define OPEN_SOCK_ONCE              (-2)
#define ALWAYS_OPEN_SOCK            (-1)

/* IP Address Configuration */
#define STATIC_IP                   (0)
#define DHCP_MODE                   (1)

/* USER's defines */
/* options-> UseCase_SLEEP, UseCase_AlwaysConnected */
#define PM_USECASE                  UseCase_AlwaysConnected

/* options-> UseCase_Normal, UseCase_CustomDTIM */
#define AC_USECASE                  UseCase_Normal

/* options -> SocketType_UDP , SocketType_TCP */
#define SOCKET_TYPE                 SocketType_UDP
#define PORT                        (5001)
#define DEST_IP_ADDR                "192.168.1.100"

/* relevant for Static IP mode */
#define SRC_IP_ADDR                 "192.168.1.10"
#define GATEWAY_IP_ADDR             "192.168.1.1"
#define SUBNET_MASK                 "255.255.255.0"

#define NUM_OF_PKT                  (1)

/* options -> STATIC_IP, DHCP_MODE */
#define IP_ADDR_ALLOC_MODE          DHCP_MODE

#define NOT_ACTIVE_DURATION_MSEC    (5000)  /* 5 seconds */
#define SLEEP_IDLE_TIME_MSEC        (5000)  /* 5 seconds */
#define LSI_MIN_DURATION_IN_MSEC    (100)
#define LSI_MAX_DURATION_IN_MSEC    (2000)
#define ELP_MIN_DURATION_IN_MSEC    (100)
#define ELP_MAX_DURATION_IN_MSEC    (255000)

/* Tag setting defines */
#define CCA_BYPASS                  (1)
#define TAG_FRAME_TRANSMIT_RATE     (6)
#define TAG_FRAME_TRANSMIT_POWER    (7)
#define TAG_CHANNEL                 (1)

/* Stack size in bytes */
#define TASKSTACKSIZE               (4096)
#define SPAWN_TASK_PRIORITY         (9)

//*****************************************************************************
// Typedefs
//*****************************************************************************

typedef enum
{
    UseCase_SLEEP,                      /* WiFi + host low power sleep mode */
    UseCase_AlwaysConnected,            /* Maintain connection with power save */
    UseCase_BleAdvertisement,           /* BLE legacy 1M advertisement */
    UseCase_BleConnection               /* BLE connection to peer device */
}UseCases;

typedef enum
{
    AlwaysConnected_HostAlwaysOn = 0,      /* Sub-mode 2.1: Host Always On - LWIP active, CPU active */
    AlwaysConnected_HostSleepWithLWIP = 1, /* Sub-mode 2.2: Host Sleep with LWIP - LWIP active, CPU sleeps */
    AlwaysConnected_HostAlwaysSleep = 2    /* Sub-mode 2.3: Host Always Sleep - LWIP disabled, Layer 2 only */
}AlwaysConnectedUseCases;

typedef enum
{
    SocketType_UDP,
    SocketType_TCP
}SocketTypes;

typedef struct _PowerMeasure_AppData_t_
{
    UseCases                       useCase;
    AlwaysConnectedUseCases        alwaysConnectedUseCase;
    uint8_t                        customDtimInterval; /* Custom DTIM interval (2-10 recommended) */
    uint32_t                       pktsToDo;
    int32_t                        sockID;
    SocketTypes                    socketType;
    char                           ipAddr[16];
    uint32_t                       port;
    char                           ssid[33];           /* WiFi SSID (max 32 chars + null) */
    char                           password[64];       /* WiFi password (max 63 chars + null) */
    uint16_t                       securityType;       /* WiFi security type (WLAN_SEC_TYPE_*) */
    /* BLE parameters */
    uint32_t                       bleAdvInterval_us;  /* BLE advertisement interval in us */
}PowerMeasure_AppData;

//*****************************************************************************
// Function prototypes
//*****************************************************************************

void *mainThread(void *arg0);
void NetworkStatusCallback(WlanRole_e roleid, uint32_t address);
void WlanStackEventHandler(WlanEvent_t *pWlanEvent);
int32_t displayBanner(void);
void startMeasureBanner(void);
void getWiFiCredentials(void);
int32_t wlanConnect(void);
int32_t configSimplelinkToUseCase(void);
void sleepMode(void);
void alwaysConnected(void);
void bleAdvertisementMode(void);
void bleConnectionMode(void);

#endif /* POWER_MEASURE_H_ */
