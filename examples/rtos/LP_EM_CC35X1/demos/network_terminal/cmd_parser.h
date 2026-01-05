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
#ifndef __CMD_PARSER_H__
#define __CMD_PARSER_H__

/* Application includes */
#include "network_terminal.h"



/* Application defines */
#define MAX_SERVICE_NAME_LENGTH     (63)
#define CHANNEL_MASK_ALL            (0x1FFF)
/* without channel 140 */
#define CHANNEL_MASK_ALL_5G         (0x1F7FFFF)
#define RSSI_TH_MAX                 (-95)
#define BSSID_ADDR                  (6)
#define MAX_SSID_ENTRIES            (30)
#define MAX_SSID_ENTRIES_FOR_P2P    (10)
#define DEFAULT_MAX_SSID_ENTRIES    (18)


/* Command structures */

typedef struct ScanCmd
{
    /* Number of Scan Entries to retrieve from the NWP */
    uint8_t         numOfentries;
    /* The netEntries array position to start write results from */
    uint8_t         index;

}ScanCmd_t;

typedef struct StopCmd
{
    /*Is wlan stop recovery */
    uint8_t         isRecovery;

}StopCmd_t;


typedef struct ConnectCmd
{
    uint8_t                 bssid[BSSID_ADDR];
    /* Ap's SSID */
    uint8_t                 *ssid;
    /* Security parameters - Security Type and Password */
    WlanSecParams_t secParams;
}ConnectCmd_t;

typedef struct ProfileCmd
{
    /* Ap's SSID */
    uint8_t                 *ssid;
    /* MAC Address */
    uint8_t                 *mac;
    /* Priority */
    uint32_t                *priority;
    /* Scan SSID */
    uint32_t                *hidden;
    /* Enterprise user name */
    uint8_t                 *entUserName;
    /* Security parameters - Security Type and Password */
    WlanSecParams_t       secParams;
}ProfileCmd_t;

typedef struct mDnsQueryCmd
{   /* The service name to query */
    uint8_t ServiceName[MAX_SERVICE_NAME_LENGTH];
    /* Sets the query type: One shot or continuous */
    uint8_t OneShotFlag;

}mDnsQueryCmd_t;


typedef struct NetworkAssistedRoamingCmd
{
    uint8_t  Enable;          /* Enable App bit - 1, Disable 0 */
    int16_t  rssiThreshold;   /* Reserved For future use       */
    uint8_t  Reserved;        /* Padding                       */
} NetworkAssistedRoamingCmd_t;


typedef union
{
    uint32_t ipv4;          /* Ipv4 Address */
    uint8_t ipv6[16];       /* Ipv6 Address */
}ip_t;

typedef struct
{
    /* Ping interval time in miliseconds */
    uint32_t interval_time_ms;
    /* Ping payload size
     * Default is 64, combined with icmp header (8 bytes)
     */
    uint16_t payload_size;
    /* Ping count - 0 for infinite */
    uint32_t count;
    /* Target IP in binary format (host representation) */
    ip_t target_ip;
    /* Source IP in binary format (host representation) */
    ip_t source_ip;
    /* User configurations flags */
    int32_t flags;
    /* Ping stop callback function */
    void (*ping_deinit_callback)(void *args);
}PingParams_t;

typedef struct SendCmd
{   /* Number of packets to send */
    int32_t numberOfPackets;
    /* Server's port address*/
    int16_t destOrLocalPortNumber;
    /* Decides the type of transport protocol */
    uint8_t udpFlag;
    /* Send as server or client flag */
    uint8_t server;
    /* Blocking or non-blocking on socket */
    uint8_t nb;
    /* IPV4 or IPv6 flag. By default IPV4 enable */
    uint8_t ipv6;
    /* IP in binary format */
    ip_t ipAddr;
    /* rate in Mbps */
    uint32_t bandwidth;

}SendCmd_t;

typedef struct RecvCmd
{/* Number of packets to receive */
    int32_t numberOfPackets;
    /* Server's port address*/
    uint32_t destOrLocalPortNumber;
    /* Decides the type of transport protocol */
    uint8_t udpFlag;
    /* Send as server or client flag */
    uint8_t server;
    /* Blocking or non-blocking on socket */
    uint8_t nb;
    /* IPV4 or IPv6 flag. By default IPV4 enable */
    uint8_t ipv6;
    /* IP in binary format */
    ip_t ipAddr;
    /* write the TP  to the terminal */
    uint8_t period;
    /* limit operation in time */
    uint32_t timeout;
    /* limit udp client  bandwidth */
    uint32_t bandwidth; //Mbps
}RecvCmd_t;

typedef struct stopCmd
{
    uint32_t processNum;
}stopCmd_t;

typedef struct ExtAdvCfg_t
{
    uint8 instance;
    uint8 legacy;
    uint32 interval_ms;
    uint8 prim_phy;
    uint8 sec_phy;
}ExtAdvCfg_t;

typedef struct ExtAdvEnable_t
{
    uint8 enable;
    uint8 instance;
    int duration;
    int max_events;
}ExtAdvEnable_t;

typedef struct ExtScanCfg_t
{
    uint16 scan_interval_ms;
    uint16 scan_window_ms;
    uint8 own_address_type;
    uint8 filter_policy;
    uint8 scan_type;
    uint8 scan_phy;

}ExtScanCfg_t;

typedef struct ExtScanEnable_t
{
    uint16 duration;
    uint16 period;
    uint8 enable;
    uint8 filter_duplicate;

}ExtScanEnable_t;

typedef struct SetInterfaceIpParams_t {
    WlanRole_e  roleType;
    uint32_t    ipMode;
    uint32_t    ipAddress;
    uint32_t    netmask;
    uint32_t    gateway;
    uint32_t    dhcp;
    Bool_e      setDhcpServerAddress;
} SetInterfaceIpParams_t;

typedef struct DhcpParams_t {
    uint32_t    leaseTime;
    uint32_t    startAddress;
    uint32_t    endAddress;
} DhcpParams_t;

typedef struct LoadCertiCmd
{
    uint8_t fileType;
    uint32_t size;
    char* certi;

}LoadCertiCmd_t;


/* Function prototypes */
int32_t ParseScanCmd
    (void *arg, ScanCmd_t *scanParams);

int32_t ParseGetMacAddressCmd(void *arg,uint32_t *RoleId);
int32_t ParseConnectCmd (void *arg, ConnectCmd_t *ConnectParams,
        WlanEapConnectParams_t* eapConnectParams, uint8_t* isEnt);
int32_t ParseRoleUpApCmd
    (void *arg, RoleUpApCmd_t *RoleUpApParams);
int32_t ParseRoleUpStaCmd
    (void *arg, RoleUpStaCmd_t *RoleUpStaParams);
int32_t ParseSendCmd
    (void *arg,  SendCmd_t *SendCmdParams);
int32_t ParseRecvCmd
    (void *arg,  RecvCmd_t *RecvCmdParams);
int32_t ParseStopCmd
    (void *arg, StopCmd_t *stopParams);


int32_t ParseTestIperfCmd(void *arg,  RecvCmd_t *IperfCmdParams);
int32_t ParseStopTestIperfCmd(void *arg, stopCmd_t *stopCmd);


int32_t ParseDisconnectCmd(void *arg, uint32_t *RoleId);
int32_t ParseCmd(void *arg);
int32_t ParseSetMacAddressCmd(void *arg, uint8_t *pMacAddress, uint32_t*  RoleId);
int32_t ParseSetWsocPrimaryCmd(void *arg, WlanConnectivityFWSlot_t *WsocSlot);
int32_t parseSetEarlyTerminationArgs(void *arg, uint8_t *pEnable);

int32_t ParseBleAdvCfgCmd(void *arg, ExtAdvCfg_t *advParams);
int32_t ParseBleAdvEnableCmd(void *arg, ExtAdvEnable_t *advEnable);
int32_t ParseBleScanCfgCmd(void *arg, ExtScanCfg_t *scanParams);
int32_t ParseBleScanEnableCmd(void *arg, ExtScanEnable_t *scanEnable);
int32_t ParseBleConnectCmd(void *arg, uint8_t *bd_addr, uint8_t* addr_type);
int32_t ParseBleDisconnectCmd(void *arg, uint8_t *bd_addr, uint8_t* addr_type);
int32_t ParseBleGetBdAddressCmd(void *arg, uint8_t* addr_type);
int32_t ParseBleSetBdAddressCmd(void *arg, uint8_t* addr_type);
int32_t ParseBleConnectedPeersCmd();
int32_t ParseBleTestModeCmd(void *arg, uint8_t *enable);
int32_t ParseSetInterfaceIpCmd(void *arg, SetInterfaceIpParams_t *params);
int32_t ParseGetInterfaceIpCmd(void *arg, WlanRole_e *RoleId);
int32_t ParseSetDhcpServerCmd(char *arg,uint32_t *leaseTime,uint32_t *startAddress,uint32_t *endAddress);

#ifdef CC35XX
int32_t ParseRoleUpP2PCmd(void *arg, RoleUpStaCmd_t *RoleUpDeviceParams);
int32_t ParseSetChannelCmd(void *arg, WlanP2pCmd_t *P2PParams);

int32_t ParseP2PConnectCmd(void *arg,uint8_t* peer_mac, uint32_t* wps_method,char* pin);

int32_t ParseConnPolicySetCmd(void *arg, WlanPolicySetGet_t *ConnPolicySetParams);
int32_t ParseProfileCmd(void *arg, ProfileCmd_t *ProfileParams);
int32_t ParseDelProfileCmd(void *arg);
int32_t ParseGetProfileCmd(void *arg);
int32_t ParseProfileConnectCmd(void *arg);
#endif

void FreeConnectCmd(ConnectCmd_t *ConnectParams);
void FreeProfileCmd(ProfileCmd_t *ProfileParams);
void FreeRoleUpApCmd(RoleUpApCmd_t *RoleUpApParams);

int32_t ipv6AddressParse(char *str,
                         uint8_t *ipv6ip);

int32_t ipv4AddressParse(char *str,
                         uint32_t *ipv4ip);

int32_t hexbyteStrtoASCII(char *str,
                          uint8_t *ascii);

int32_t macAddressParse(char *str,
                        uint8_t *mac);

#ifdef CC35XX
int32_t ParseStartApWpsSessionCmd(void *arg, wlanWpsSession_t *wpsSession);
int32_t ParsePingCmd(void *arg,
                     PingParams_t *pingParams);
int32_t ParsePingStopCmd(void *arg, int8_t *session_id);
int32_t ParseRegDomEntrySetCmd(void *arg, WlanSetRegDomainCustomEntry_t *entryParams);
int32_t ParseRegDomEntryGetCmd(void *arg, WlanSetRegDomainCustomEntry_t *entryParams);
#endif

//Indigo 
int32_t ParseLoadCartificateCmd(void *arg, LoadCertiCmd_t *loadCertificate);

#endif /* __CMD_PARSER_H__ */
