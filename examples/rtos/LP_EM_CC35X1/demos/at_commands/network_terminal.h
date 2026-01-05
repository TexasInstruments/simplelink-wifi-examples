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
#ifndef __NETWORK_TERMINAL_H__
#define __NETWORK_TERMINAL_H__

#include <stdint.h>
#include "str.h"
#include "wlan_if.h"
#include "osi_kernel.h"
#include "upper_mac_versions.h"

#ifdef CC35XX
#define MAX_BUF_SIZE            (1470)
#endif // CC35XX
#ifdef CC33XX
#define MAX_BUF_SIZE            (1200)
#endif // CC33XX
#define CMD_BUFFER_LEN          (256)
#define MAX_CMD_NAME_LEN        (32)
#define APPLICATION_NAME        ("Network Terminal")
#define APPLICATION_VERSION     (version_upper_mac)
#define TASK_STACK_SIZE         (2048)
#define SPAWN_TASK_PRIORITY     (9)
#define PASSWD_LEN_MAX          (64)
#define PASSWD_LEN_MIN          (8)
#define MAX_FILE_NAME_LEN       (32)
#define MAX_TEXT_PAD_SIZE       (256)
#define MAX_FILE_LIST           (20)
#define SL_STOP_TIMEOUT         (200)
#define DEV_TYPE_LEN            (17)
#define IPV6_ADDR_LEN           (16)
#define IPV4_ADDR_LEN           (4)

/* Network Terminal Errors */
#define WLAN_OSI_ERROR_BASE                     (-1000L)
#define WLAN_OPERATION_FAILED                   (-1001L)
#define WLAN_ABORTED                            (-1002L)
#define WLAN_OSI_INVALID_PARAMS                 (-1003L)
#define WLAN_ALLOCATION_FAILURE                 (-1004L)
#define WLAN_TIMEOUT                            (-1005L)
#define WLAN_EVENTS_IN_USE                      (-1006L)
#define WLAN_EVENT_OPEARTION_FAILURE            (-1007L)
#define WLAN_NO_SEMAPHORE_INSTANCE_AVAILABLE    (-1008L)
#define WLAN_EXCEEDED_SEMAPHORE_COUNT           (-1009L)
#define WLAN_DELETED                            (-1010L)
#define WLAN_FILESYS_FAILURE                    (-1011L)

#define DEVICE_ERROR_MSG        ( \
        "Device error, please refer \"DEVICE ERRORS CODES\" section in errors.h")
#define WLAN_ERROR_MSG          ( \
        "WLAN error, please refer \"WLAN ERRORS CODES\" section in errors.h")
#define BSD_SOCKET_ERROR_MSG    ( \
        "BSD Socket error, please refer \"BSD SOCKET ERRORS CODES\" section in errors.h")
#define SOCKET_ERROR_MSG        ( \
        "Socket error, please refer \"SOCKET ERRORS CODES\" section in errors.h")
#define NETAPP_ERROR_MSG        ( \
        "Netapp error, please refer \"NETAPP ERRORS CODES\" section in errors.h")
#define OS_ERROR_MSG            ( \
        "OS error, please refer \"OSI ERRORS CODES\" section in errno.h")
#define TRNSPORT_CMD_ERROR_MSG  ( \
        "TRNSPORT CMD error, please refer \"TRNSPORT CMD ERRORS CODES\" section in errno.h")
#define CMD_ENT_ERROR           ("Invalid option we do not support enterprise user name.")
#define CMD_ERROR               ("Invalid option/command.")
#define RECEIVE_TIMEOUT         (20)

typedef union
{
    WlanNetworkEntry_t         netEntries[WLAN_MAX_SCAN_COUNT];
}gDataBuffer_t;

typedef struct cmdAction
{
    const char        *cmd;
    int32_t    (*callback)(void *);
    int32_t    (*printusagecallback)(void *);
}cmdAction_t;


typedef struct connectionControlBlock_t
{
    OsiSyncObj_t    disconnectEventSyncObj;
    OsiSyncObj_t    connectEventSyncObj;
    OsiSyncObj_t    eventCompletedSyncObj;
    OsiSyncObj_t    dhcpIprecvSyncObj;
    OsiSyncObj_t    staRoleupSyncObj;
    OsiSyncObj_t    staRoledownSyncObj;
    uint32_t GatewayIP;
    uint8_t  ConnectionSSID[WLAN_SSID_MAX_LENGTH +1];
    uint8_t  ConnectionBSSID[WLAN_BSSID_LENGTH];
    uint32_t DestinationIp;
    uint32_t IpAddr;
    uint32_t StaIp;
    uint32_t Ipv6Addr[4];
}connection_CB;

typedef struct appControlBlock_t
{
    /* Status Variables */
    /* This bit-wise status variable shows the state of the NWP */
    uint32_t Status;
     /* This field keeps the device's role (STA, P2P or AP) */
    uint32_t Role;
     /* This field keeps the device's role (STA, P2P or AP) */
    uint32_t ConnectedStations;
    /* This flag lets the application to exit */
    uint32_t Exit;
    /* Sets the number of Ping attempts to send */
    uint32_t PingAttempts;
    /* Data & Network entry Union */
    gDataBuffer_t gDataBuffer;
    /* Cmd Prompt buffer */
    uint8_t CmdBuffer[CMD_BUFFER_LEN];
    /* STA/AP mode CB */
    connection_CB CON_CB;

    OsiSyncObj_t    eventCompletedScanObj;
}appControlBlock;

extern appControlBlock app_CB;

typedef enum
{
/* This bit is set: Network Processor is powered up */
    STATUS_BIT_NWP_INIT = 0,          
/* This bit is set: the device is connected to the AP (STA) */
    STATUS_BIT_STA_CONNECTION,
/* This bit is set: the device is configured as an AP 
   and is up(AP) */
    STATUS_BIT_AP_CONNECTION,
/* This bit is set: One or more clients are connected to device (AP) */
    STATUS_BIT_PEER_CONNECTED,
/* This bit is set: the device has leased IP to any connected client */
    STATUS_BIT_IP_LEASED,
/* This bit is set: the device has acquired an IP */
    STATUS_BIT_IP_ACQUIRED,               
/* If this bit is set: the device (P2P mode)
   found any p2p-device in scan */
    STATUS_BIT_P2P_DEV_FOUND,
/* If this bit is set: the device (P2P mode) 
   found any p2p-negotiation request */
    STATUS_BIT_P2P_REQ_RECEIVED,
/* If this bit is set: the device(P2P mode)
   connection to client(or reverse way) is failed */
    STATUS_BIT_CONNECTION_FAILED,
/* If this bit is set: the device (P2P mode)
    p2p group is started */
    STATUS_BIT_P2P_GROUP_STARTED,
/* This bit is set: device is undergoing ping operation */
    STATUS_BIT_PING_STARTED,              
/* This bit is set: Scan is running is background */
    STATUS_BIT_SCAN_RUNNING,              
/* If this bit is set: the device 
   has acquired an IPv6 address */
    STATUS_BIT_IPV6_ACQUIRED,             
/* If this bit is set: the device has acquired 
   an IPv6 address */
    STATUS_BIT_IPV6_GLOBAL_ACQUIRED,
/* If this bit is set: the device has acquired
   an IPv6 address */
    STATUS_BIT_IPV6_LOCAL_ACQUIRED,   
                                         
/* If this bit is set: Authentication with ENT AP failed. */
    STATUS_BIT_AUTHENTICATION_FAILED, 
    

    STATUS_BIT_RESET_REQUIRED,

    STATUS_BIT_TX_STARED

}e_StatusBits;


/* Status keeping MACROS */

#define SET_STATUS_BIT(status_variable, bit) status_variable |= (1<<(bit))

#define CLR_STATUS_BIT(status_variable, bit) status_variable &= ~(1<<(bit))

#define GET_STATUS_BIT(status_variable, bit) \
                                (0 != (status_variable & (1<<(bit))))

#define IS_NW_PROCSR_ON(status_variable)     \
                GET_STATUS_BIT(status_variable, STATUS_BIT_NWP_INIT)

#define IS_STA_CONNECTED(status_variable)        \
                GET_STATUS_BIT(status_variable, STATUS_BIT_STA_CONNECTION)

#define IS_AP_CONNECTED(status_variable)        \
                GET_STATUS_BIT(status_variable, STATUS_BIT_AP_CONNECTION)

#define IS_PEER_CONNECTED(status_variable)        \
                GET_STATUS_BIT(status_variable, STATUS_BIT_PEER_CONNECTED)


#define IS_IP_LEASED(status_variable)        \
                GET_STATUS_BIT(status_variable, STATUS_BIT_IP_LEASED)

#define IS_IP_ACQUIRED(status_variable)      \
                GET_STATUS_BIT(status_variable, STATUS_BIT_IP_ACQUIRED)

#define IS_IP6_ACQUIRED(status_variable)     \
    GET_STATUS_BIT(status_variable, \
                   (STATUS_BIT_IPV6_LOCAL_ACQUIRED | \
                    STATUS_BIT_IPV6_GLOBAL_ACQUIRED))

#define IS_IPV6L_ACQUIRED(status_variable)   \
                GET_STATUS_BIT(status_variable, STATUS_BIT_IPV6_LOCAL_ACQUIRED)

#define IS_IPV6G_ACQUIRED(status_variable)   \
                GET_STATUS_BIT(status_variable, STATUS_BIT_IPV6_GLOBAL_ACQUIRED)

#define IS_PING_RUNNING(status_variable)     \
                GET_STATUS_BIT(status_variable, STATUS_BIT_PING_STARTED)

#define IS_TX_ON(status_variable)            \
                GET_STATUS_BIT(status_variable, STATUS_BIT_TX_STARED)

#define ASSERT_ON_ERROR(ret, errortype)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                return -1;\
            }\
        }

#define ASSERT_AND_CLEAN_CONNECT(ret, errortype, ConnectParams)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                FreeConnectCmd(ConnectParams);\
                return -1;\
            }\
        }

#define ASSERT_AND_CLEAN_PROFILE(ret, errortype, ProfileParams)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                FreeProfileCmd(ProfileParams);\
                return -1;\
            }\
        }

#define ASSERT_AND_CLEAN_STARTAP(ret, errortype, StartApParams)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                FreeStartApCmd(StartApParams);\
                return -1;\
            }\
        }

#define ASSERT_AND_CLEAN_PING(ret, errortype, pingParams)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                FreePingCmd(pingParams);\
                return -1;\
            }\
        }

#define ASSERT_AND_CLEAN_MDNS_ADV(ret, errortype, mDNSAdvertiseParams)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                FreemDNSAdvertiseCmd(mDNSAdvertiseParams);\
                return -1;\
            }\
        }

#define ASSERT_AND_CLEAN_RECV(ret, errortype, RecvCmdParams)\
        {\
            if(ret < 0)\
            {\
                SHOW_WARNING(ret, errortype);\
                return -1;\
            }\
        }

#define SHOW_WARNING(ret, errortype)        UART_PRINT( \
        "\n\r[line:%d, error code:%d] %s\n\r", __LINE__, ret, errortype);


/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/

#define WLAN_MAC_ADDR_LEN                              (6)
#define WLAN_IPV6_ADDR_LEN                             (16)
#define WLAN_IPV4_VAL(add_3,add_2,add_1,add_0)         ((((uint32_t)add_3 << 24) & 0xFF000000) | \
                                                        (((uint32_t)add_2 << 16) & 0xFF0000) | \
                                                        (((uint32_t)add_1 << 8) & 0xFF00) | \
                                                        ((uint32_t)add_0 & 0xFF))
#define WLAN_IPV6_VAL(add_1,add_2)                     ((((_u32)add_1 << 16) & 0xFFFF0000) | (((_u32)add_2 ) & 0x0000FFFF) )
#define WLAN_IPV4_BYTE(val,index)                      ( (val >> (index*8)) & 0xFF )

void gpioButtonFxn1(uint8_t index);

void PrintIPAddress(unsigned char ipv6,
                    void *ip);

#endif /* __NETWORK_TERMINAL_H__ */
