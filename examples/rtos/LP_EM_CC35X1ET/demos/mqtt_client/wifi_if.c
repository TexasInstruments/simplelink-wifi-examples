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
/*****************************************************************************

   Application Name     - Provisioning application
   Application Overview - This application demonstrates how to use 
                          the provisioning method
                        in order to establish connection to the AP.
                        The application
                        connects to an AP and ping's the gateway to
                        verify the connection.

   Application Details  - Refer to 'Provisioning' README.html

 *****************************************************************************/
//****************************************************************************
//
//! \addtogroup
//! @{
//
//****************************************************************************

/* Standard Include */
#include <stdint.h>
#include <unistd.h>
#include <stdarg.h>
/* TI-DRIVERS Header files */

//#include <ti/drivers/SPI.h>
//#include <ti/drivers/GPIO.h>


#include "wifi_if.h"
#include "tcpip_if.h"

#include "osi_kernel.h"

#include "ti_drivers_config.h"
#include "wifi_settings.h"

#ifdef WIFI_LED_INDEX
#include "led_if.h"
#endif

#include "uart_term.h"



#define WLAN_RET_CODE_DEV_NOT_STARTED       (-2018L)
#define WLAN_RET_CODE_DEV_ALREADY_STARTED   (-2012L)


#define TIMEOUT_SEM 1

typedef enum { LOG_RX, LOG_TX } logType_e;

#define NTOS(ptr) (((uint16_t)(ptr)[0])*256+(ptr)[1])

/* frequency (# packet received/sent) of heap log, 0 to disable logs */
#define ARP 0x806
#define IP 0x800
#define IP6 0x86DD
#define ICMP    1
#define IGMP    2
#define TCP     6
#define UDP     17
#define ICMP6   58

//#define WIFI_LOG_ENABLE
#ifdef WIFI_LOG_ENABLE
#define LOG_WIFI NET_logEthFrame
#else
#define LOG_WIFI(...)
#endif


#define ETH_LOG    (1)
#define IP4_LOG    (1)
#define IP6_LOG    (1)
#define TCP_LOG    (1)
#define UDP_LOG    (1)
#define ARP_LOG    (1)
#define ICMP4_LOG  (1)
#define ICMP6_LOG  (1)
#define IGMP_LOG   (1)

static uint32_t IPv6_LOG(char *buff, uint32_t offset, uint8_t *iphdr, int ipOffset)
{
    int i;
    bool bFoundZero = false;
    for(i=0; i<8; i++)
    {
        uint16_t value = iphdr[ipOffset+i*2]*256+iphdr[ipOffset+1+i*2];
        if(value)
        {
            offset += sprintf(&buff[offset], "%04x:", value);
            bFoundZero = false;
        }
        else
        {
            if(!bFoundZero)
            {
                offset += sprintf(&buff[offset], ":");
            }
            bFoundZero = true;
        }
    }
    buff[offset-1] = ' ';
    return offset-1;
}

/*!

    \brief     Ethernet Packet Trace (Log)

    \param[in] type - LOG_RX or LOG_TX
    \param[in] pIfName - string name of the interface
    \param[in] nBytes - length of packet (headers + payload)
    \param[in] p - pointer to start of packet (ethernet header)

    \return    module id (>=0) upon success, or negative error code

*/
static char buff[256];
static char strIP[128];
void NET_logEthFrame(logType_e type, char *pIfName, uint32_t nBytes, uint8_t * p)
{
    uint16_t prot;
    uint8_t *ethhdr = p;
    char *pTypeStr = (type==LOG_RX)?"RCVD":"SENT";
    uint32_t offset = 0;
    uint32_t msecs;

    msecs = 2*osi_GetTimeMS(); //tick factor 2 correction

    offset = sprintf(&buff[0], "%d.%d sec | ", msecs/1000, msecs%1000);
    offset += sprintf(&buff[offset], "%s (%4lu) | ", pTypeStr, (unsigned long)nBytes);

#if ETH_LOG
    offset += sprintf(&buff[offset], "ETH: %02x:%02x:%02x:%02x:%02x:%02x -> %02x:%02x:%02x:%02x:%02x:%02x",
              ethhdr[6], ethhdr[7], ethhdr[8], ethhdr[9], ethhdr[10], ethhdr[11],
              ethhdr[0], ethhdr[1], ethhdr[2], ethhdr[3], ethhdr[4], ethhdr[5]);
#endif

    prot = NTOS(&ethhdr[12]);
    switch (prot)
    {
    case ARP: {
        uint8_t *arphdr = &ethhdr[14];
        uint16_t op = NTOS(&arphdr[6]);
#if ARP_LOG
        offset += sprintf(&buff[offset], " | ARP: OP=%d sender=%d.%d.%d.%d target=%d.%d.%d.%d", op,
                              arphdr[14], arphdr[15], arphdr[16], arphdr[17],
                              arphdr[24], arphdr[25], arphdr[26], arphdr[27]);
#endif
        break;
    }
    case IP:
    case IP6: {
        uint8_t *iphdr = &ethhdr[14];
        uint8_t ip_prot = 0;
        uint8_t ip_prot_offset = 0;
        uint32_t strIPOffset = 0;
        // 14 bytes from ethernet header + 9 bytes from IP header
        if((prot==IP) && ((iphdr[0]&0xf0) == 0x40))
        {
#if IP4_LOG
             sprintf(strIP, " | IP4: %d.%d.%d.%d -> %d.%d.%d.%d", iphdr[12], iphdr[13], iphdr[14], iphdr[15], iphdr[16], iphdr[17], iphdr[18], iphdr[19]);
             ip_prot = iphdr[9];
             ip_prot_offset = (iphdr[0]&0xf)*4;
#endif
        }
        else if((prot==IP6) && ((iphdr[0]&0xf0) == 0x60))
        {
#if IP6_LOG
             strIPOffset += sprintf(&strIP[strIPOffset], " | IP6: ");
             strIPOffset = IPv6_LOG(strIP, strIPOffset, iphdr, 8);
             strIPOffset += sprintf(&strIP[strIPOffset], " -> ");
             strIPOffset = IPv6_LOG(strIP, strIPOffset, iphdr, 24);
             ip_prot = iphdr[6];
             ip_prot_offset = 40;

             if(ip_prot == 0)
             {
                 ip_prot = iphdr[40];
                 ip_prot_offset = 48+iphdr[41];
             }

#endif
        }
        offset += sprintf(&buff[offset], "%-64s", strIP);
        if (ip_prot == UDP)
        {
#if UDP_LOG
            uint8_t *udphdr = &iphdr[ip_prot_offset];
        uint16_t src_port, dst_port;
            src_port = NTOS(&udphdr[0]);
            dst_port = NTOS(&udphdr[2]);
            if(dst_port == 5353)
            {
                uint8_t *mdnshdr = &udphdr[8];
                uint16_t mdnsflags = NTOS(&mdnshdr[2]);
                offset += sprintf(&buff[offset], " | MDNS: %s (%04x)", (mdnsflags&0x8000)?"RESPONSE":"QUERY", mdnsflags);
            }
            else
            {
                offset += sprintf(&buff[offset], " | UDP: %d -> %d ", src_port, dst_port);
            }
#endif
            break;
        }
        else if (ip_prot == ICMP)
        {
#if ICMP4_LOG
            offset += sprintf(&buff[offset], " ICMP:");
#endif
        }
        else if (ip_prot == ICMP6)
        {
#if ICMP6_LOG
            char strType[4] = {0};
            char *icmp6Type = strType;
            uint8_t *icmp6hdr = &iphdr[ip_prot_offset];
            sprintf(strType, "%d", icmp6hdr[0]);
            switch(icmp6hdr[0]) {
            case 128:  icmp6Type="Echo Request"; break;
            case 129:  icmp6Type="Echo Reply"; break;
            case 130:  icmp6Type="Multicast Listener Query"; break;
            case 131:  icmp6Type="Multicast Listener Report"; break;
            case 133:  icmp6Type="Router Solicitation"; break;
            case 134:  icmp6Type="Router Advertisement "; break;
            case 135:  icmp6Type="Neighbor Solicitation"; break;
            case 136:  icmp6Type="Neighbor Advertisement "; break;
            case 137:  icmp6Type="Redirect"; break;
            }
            offset += sprintf(&buff[offset], " | ICMP6: %s", icmp6Type);
#endif
        }
        else if (ip_prot == IGMP)
        {
#if IGMP4_LOG
             offset += sprintf(&buff[offset], " | IGMP:");
#endif
        }
        else if (ip_prot == TCP)
        {
#if TCP_LOG
            offset += sprintf(&buff[offset], " | TCP:");
        }
        else
        {
            offset += sprintf(&buff[offset], " | IP-PROT (%d):", ip_prot);
#endif
        }
        break;
    }
    default: {
        offset += sprintf(&buff[offset], " | prot: %d", prot);
    }
    }
    if(type==LOG_RX)
    {
        UART_PRINT("\033[0;94m%s\033[0m\n\r", buff);
    }
    else
    {
        UART_PRINT("\033[0;34m%s\033[0m\n\r", buff);
    }
}

/*** TBD - multi-thread protection ***/

static void WiFiCB_receive(WlanRole_e role, uint8_t *inBuf, uint32_t inLen);

/****************************************************************************
              TYPES AND STRUCTURES DEFINITIN
 ****************************************************************************/
typedef enum
{
/* This bit is set: Network Processor is powered up */
    STATUS_BIT_NWP_INIT = 0,          
/* This bit is set: the device is connected to the AP (STA) */
    STATUS_BIT_STA_CONNECTION,
/* This bit is set: the device is configureWIFI_SERVICE_LVL_MACd as an AP
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
} statusBits_e;

typedef struct connectionControlBlock_t
{
    OsiSyncObj_t       disconnectEventSyncObj;
    uint8_t            SSID[WLAN_SSID_MAX_LENGTH +1];
    uint8_t            BSSID[WLAN_BSSID_LENGTH];
} wifiConn_t;

typedef struct _connRequest
{
    WifiEventHandler_f    handler;
    WifiConnStatus_e      reqStatus;
    OsiSyncObj_t          syncObj;    
    struct _connRequest  *pNext;
} wifiConnRequest_t;

typedef struct appControlBlock_t
{
    bool                  bIsStarted;
    void                 *hStaNetif;
        /* This bit-wise status variable shows the state of the NWP */
    uint32_t             Status;

     /* This field keeps the device's role (STA, P2P or AP) */
    uint32_t             Role;

    WlanNetworkEntry_t   netEntries[WLAN_MAX_SCAN_COUNT];

    uint8_t              netEntriesSize;

    wifiConnRequest_t    *pConnRequests;

    uint32_t             ConnectedStations;
    WifiServiceLevel_e   maxReqConnLevel;
    WifiConnStatus_e     currConnStatus;
    /* STA/AP mode CB */
    wifiConn_t           currentConn;
 } WifiCtx_t;


/****************************************************************************
              MODULES (STATIC) VARIABLES
 ****************************************************************************/
WifiCtx_t             m_wifiCtx = {0};
/****************************************************************************
              GLOBAL VARIABLES
 ****************************************************************************/

//*****************************************************************************
//            LOCAL FUNCTIONS
//*****************************************************************************
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


static void Notify(WifiConnStatus_e netStatus, void *params)
{
    wifiConnRequest_t *pConnReq = m_wifiCtx.pConnRequests;
    while(pConnReq)
    {
        if(pConnReq->handler)
        {
            pConnReq->handler(netStatus, params);
        }
        if(pConnReq->reqStatus == netStatus)
    {
            osi_SyncObjSignal(&pConnReq->syncObj);
    }
        pConnReq = pConnReq->pNext;
    }
}    

static void *AddConnRequest(WifiEventHandler_f handler, WifiServiceLevel_e level, bool *pIsFirst)
{
    wifiConnRequest_t *pConnReq = (wifiConnRequest_t *)malloc(sizeof(wifiConnRequest_t));
    if(pConnReq)
    {
        if(pIsFirst)
        {
            *pIsFirst = (m_wifiCtx.pConnRequests == NULL);
        }
        int rc = osi_SyncObjCreate(&pConnReq->syncObj);
        if(rc != 0)
        {
            UART_PRINT("osi_SyncObjCreate (%d)\r\n", rc);
            free(pConnReq);
            return(NULL);
        }

        pConnReq->handler = handler;
        pConnReq->reqStatus = (WifiConnStatus_e)(WIFI_STATUS_CONNECTED + level);
        if(m_wifiCtx.maxReqConnLevel > level)
        {
            m_wifiCtx.maxReqConnLevel = level;
        }
        pConnReq->pNext = m_wifiCtx.pConnRequests;
        m_wifiCtx.pConnRequests = pConnReq;
    }
    return (void*)pConnReq;
}    

static int WaitForConn(void *hConnReq, unsigned long timeout_ms)
{
    int rc;
    wifiConnRequest_t *pConnReq = (wifiConnRequest_t *)hConnReq;
    rc = osi_SyncObjWait(&pConnReq->syncObj, timeout_ms);
    if(rc != OSI_OK)
    {
        UART_PRINT("[wlanconnect] : Timeout expired connecting to Network: %s\r\n", AP_SSID);
        Wlan_Disconnect(WLAN_ROLE_STA,NULL);
    }
    return rc;
}

static void DelConnRequest(void *hConnReq, bool *pIsLast)
{
    wifiConnRequest_t *pConnReq = m_wifiCtx.pConnRequests, *pPrev = NULL;
    while(pConnReq)
    {
        if(pConnReq == hConnReq)
        {
            if(pPrev)
            {
                pPrev->pNext = pConnReq->pNext;
            }
            else
            {
                m_wifiCtx.pConnRequests = pConnReq->pNext;
            }
            free(pConnReq);
            break;
        }
        pPrev = pConnReq;    
        pConnReq = pConnReq->pNext;    
    }
    *pIsLast = (m_wifiCtx.pConnRequests == NULL);
}



//*****************************************************************************
//
//! \brief  Create and init the WIFI_IF context
//!
//*****************************************************************************


static int32_t  ContextInit(void)
{
    int32_t ret = OSI_OK;

    memset(&m_wifiCtx, 0x0, sizeof(m_wifiCtx));
    m_wifiCtx.Role = WLAN_ROLE_RESERVED;
    m_wifiCtx.maxReqConnLevel = WIFI_SERVICE_LVL_MAX;
    m_wifiCtx.hStaNetif = NULL; 
    m_wifiCtx.currConnStatus = WIFI_STATUS_DISCONNECTED;
    m_wifiCtx.bIsStarted = true;

    ret = osi_SyncObjCreate(&m_wifiCtx.currentConn.disconnectEventSyncObj);
    if(ret != OSI_OK)
    {
        UART_PRINT("osi_SyncObjCreate (%d)\r\n", ret);
        return(-1);
    }
    return(ret);
}

/*!
    \brief          Print scan results.

    This function print the scan results neatly in a table.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdScancallback

*/
static void PrintScanResults(uint32_t res_num)
{

    uint32_t    index;

    /* Print table column headers */
    UART_PRINT("----------------------------------------------------------------------------\r\n");
    UART_PRINT("    |               SSID               |       BSSID       | RSSI  | Ch | Hidden | Security \r\n");
    UART_PRINT("----------------------------------------------------------------------------\r\n");


    /* Print the table */
    for(index = 0; index < res_num; index++)
    {
        UART_PRINT(" %-2d | %-32s | %02x:%02x:%02x:%02x:%02x:%02x | %-5d | %-2d | %s | %d \r\n",
            index+1, m_wifiCtx.netEntries[index].Ssid,
            m_wifiCtx.netEntries[index].Bssid[0],
            m_wifiCtx.netEntries[index].Bssid[1],
            m_wifiCtx.netEntries[index].Bssid[2],
            m_wifiCtx.netEntries[index].Bssid[3],
            m_wifiCtx.netEntries[index].Bssid[4],
            m_wifiCtx.netEntries[index].Bssid[5],
            m_wifiCtx.netEntries[index].Rssi,
            m_wifiCtx.netEntries[index].Channel,
            WLAN_SCAN_RESULT_HIDDEN_SSID(m_wifiCtx.netEntries[index].SecurityInfo) == 0 ? "NO " : "YES",
            WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(m_wifiCtx.netEntries[index].SecurityInfo)
            );

    }

    UART_PRINT("----------------------------------------------------------------------------\r\n");

    return;
}

/*!
    \brief          WlanEventHandler

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
void OnWifiDriverEvent(WlanEvent_t *pWlanEvent)
{
    if(!pWlanEvent)
    {
        return;
    }
    switch(pWlanEvent->Id)
    {
        case WLAN_EVENT_CONNECT:
        {
            SET_STATUS_BIT(m_wifiCtx.Status, STATUS_BIT_STA_CONNECTION);
            m_wifiCtx.currConnStatus = WIFI_STATUS_CONNECTED_MAC;

            /* Copy new connection SSID and BSSID to global parameters */
            os_memcpy(m_wifiCtx.currentConn.SSID, pWlanEvent->Data.Connect.SsidName,
                pWlanEvent->Data.Connect.SsidLen);
            os_memcpy(m_wifiCtx.currentConn.BSSID, pWlanEvent->Data.Connect.Bssid,
                WLAN_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , BSSID: %x:%x:%x:%x:%x:%x\r\n",
                m_wifiCtx.currentConn.SSID,
                m_wifiCtx.currentConn.BSSID[0], m_wifiCtx.currentConn.BSSID[1], m_wifiCtx.currentConn.BSSID[2],
                m_wifiCtx.currentConn.BSSID[3], m_wifiCtx.currentConn.BSSID[4], m_wifiCtx.currentConn.BSSID[5]);

            TCPIP_IF_notifyLinkChange(m_wifiCtx.hStaNetif, E_TCPIP_LINK_UP);
            Wlan_EtherPacketRecvRegisterCallback(WLAN_ROLE_STA, WiFiCB_receive);

            Notify(WIFI_STATUS_CONNECTED, NULL);
    }
    break;

    case WLAN_EVENT_DISCONNECT:
    {
        WlanEventDisconnect_t  *pEventData = NULL;
#ifdef WIFI_LED_INDEX
       LED_IF_set(WIFI_LED_INDEX, 0);
#endif

        CLR_STATUS_BIT(m_wifiCtx.Status, STATUS_BIT_STA_CONNECTION);
        CLR_STATUS_BIT(m_wifiCtx.Status, STATUS_BIT_IP_ACQUIRED);
        CLR_STATUS_BIT(m_wifiCtx.Status, STATUS_BIT_IPV6_ACQUIRED);
#if 0
        killAllProcess();
        staif = network_get_sta_if();

        network_set_down(staif);
#endif
        /* If ping operation is running, release it. */
        if(IS_PING_RUNNING(m_wifiCtx.Status))
        {
            UART_PRINT(
                "\n\rPing failed, since device is no longer connected.\n\r");
        }

        pEventData = &pWlanEvent->Data.Disconnect;

        /* If the user has initiated 'Disconnect' request,
           'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED */
        if(WLAN_DISCONNECT_USER_INITIATED == pEventData->ReasonCode)
        {
            UART_PRINT(
                "\n\r[WLAN EVENT] Device disconnected from the AP: %s, "
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                m_wifiCtx.currentConn.SSID,
                m_wifiCtx.currentConn.BSSID[0],
                m_wifiCtx.currentConn.BSSID[1],
                m_wifiCtx.currentConn.BSSID[2],
                m_wifiCtx.currentConn.BSSID[3],
                m_wifiCtx.currentConn.BSSID[4],
                m_wifiCtx.currentConn.BSSID[5]);
                osi_SyncObjSignal(&m_wifiCtx.currentConn.disconnectEventSyncObj);
        }
        else
        {
            UART_PRINT(
                "\n\r[WLAN ERROR] Device disconnected from the AP: %s,\n\r"
                "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                m_wifiCtx.currentConn.SSID,
                m_wifiCtx.currentConn.BSSID[0],
                m_wifiCtx.currentConn.BSSID[1],
                m_wifiCtx.currentConn.BSSID[2],
                m_wifiCtx.currentConn.BSSID[3],
                m_wifiCtx.currentConn.BSSID[4],
                m_wifiCtx.currentConn.BSSID[5]);
        }
        memset(&(m_wifiCtx.currentConn.SSID), 0x0,
               sizeof(m_wifiCtx.currentConn.SSID));
        memset(&(m_wifiCtx.currentConn.BSSID), 0x0,
               sizeof(m_wifiCtx.currentConn.BSSID));
        TCPIP_IF_notifyLinkChange(m_wifiCtx.hStaNetif, E_TCPIP_LINK_DOWN);
        Wlan_EtherPacketRecvRegisterCallback(WLAN_ROLE_STA, NULL);

        Notify(WIFI_STATUS_DISCONNECTED, NULL);
    }
    break;
    case WLAN_EVENT_SCAN_RESULT:
    {
        uint32_t    index;
        m_wifiCtx.netEntriesSize = pWlanEvent->Data.ScanResult.NetworkListResultLen;
        os_memset(m_wifiCtx.netEntries, 0x0, sizeof(m_wifiCtx.netEntries));

        for(index = 0; index < m_wifiCtx.netEntriesSize; index++)
        {

             /* Copy SSID and BSSID to global parameters */
            os_memcpy(m_wifiCtx.netEntries[index].Ssid, pWlanEvent->Data.ScanResult.NetworkListResult[index].Ssid,
               pWlanEvent->Data.ScanResult.NetworkListResult[index].SsidLen);
            os_memcpy(m_wifiCtx.netEntries[index].Bssid, pWlanEvent->Data.ScanResult.NetworkListResult[index].Bssid,
                WLAN_BSSID_LENGTH);

            m_wifiCtx.netEntries[index].SsidLen = pWlanEvent->Data.ScanResult.NetworkListResult[index].SsidLen;
            m_wifiCtx.netEntries[index].Rssi = (int8)pWlanEvent->Data.ScanResult.NetworkListResult[index].Rssi;
            m_wifiCtx.netEntries[index].SecurityInfo = pWlanEvent->Data.ScanResult.NetworkListResult[index].SecurityInfo;
            m_wifiCtx.netEntries[index].Channel = pWlanEvent->Data.ScanResult.NetworkListResult[index].Channel;

        }
        PrintScanResults(m_wifiCtx.netEntriesSize);
    }
    break;
    case WLAN_EVENT_ADD_PEER:
    {
        uint8_t *macAddr = pWlanEvent->Data.AddPeer.Mac;
        UART_PRINT(
            "\n\r[WLAN EVENT] Device Mac: %x:%x:%x:%x:%x:%x Connected to AP \n\r",
            macAddr[0],
            macAddr[1],
            macAddr[2],
            macAddr[3],
            macAddr[4],
            macAddr[5]);
        m_wifiCtx.ConnectedStations++;
        SET_STATUS_BIT(m_wifiCtx.Status, STATUS_BIT_PEER_CONNECTED);
    }
    break;
    case WLAN_EVENT_REMOVE_PEER:
    {
        uint8_t *macAddr = pWlanEvent->Data.RemovePeer.Mac;
        UART_PRINT(
            "\n\r[WLAN EVENT] Device Mac: %x:%x:%x:%x:%x:%x Disconnected from AP \n\r",
            macAddr[0],
            macAddr[1],
            macAddr[2],
            macAddr[3],
            macAddr[4],
            macAddr[5]);
        m_wifiCtx.ConnectedStations--;
        if (m_wifiCtx.ConnectedStations == 0 )
        {
            CLR_STATUS_BIT(m_wifiCtx.Status, STATUS_BIT_PEER_CONNECTED);
        }
    }
    break;
    default:
    {
        UART_PRINT("\n\r[WLAN EVENT] Unexpected event [0x%x]\n\r",
                   pWlanEvent->Id);
    }
    break;
    }
}

static void OnTcpipEvent(void *pNetif, TcpipStatue_e status, void * pParams)
{
    UART_PRINT("OnTcpipEvent(%d)\n\r", status);
    switch (status)
    {
    case E_TCPIP_STATUS_IP_ACQUIRED:
        Notify(WIFI_STATUS_CONNECTED_IP, m_wifiCtx.hStaNetif);
#ifdef WIFI_LED_INDEX
       LED_IF_set(WIFI_LED_INDEX, 100);
#endif
        break;
    case E_TCPIP_STATUS_IP_LOST:
        Notify(WIFI_STATUS_CONNECTED, NULL);
#ifdef WIFI_LED_INDEX
       LED_IF_set(WIFI_LED_INDEX, 0);
#endif
        break;
    default:
        break;
    }
}

void myErr(char *err, char *file, unsigned long line)
{
    //LOG_ERROR("**************************** Error %s in %s (%d) **************************\n\r", err, file, line);
}


static int TcpipCB_send(void *hNetif, uint8_t *pPayload, int16_t payloadLen, uint32_t flags)
{
    int rc = Wlan_EtherPacketSend(WLAN_ROLE_STA, pPayload, payloadLen, (int16_t)flags);
    if (rc >= 0)
    {
        //UART_PRINT("Wlan_EtherPacketSend (len = %d) :: payload[0:3] = 0x%x:0x%x:0x%x:0x%x\r\n", payloadLen, *pPayload, *(pPayload+1), *(pPayload+2), *(pPayload+3));
        LOG_WIFI(LOG_TX, "WIFI", payloadLen, pPayload);
    }
    else
    {
        UART_PRINT("Wlan_EtherPacketSend (len = %d) :: Error = %d\r\n", payloadLen, rc);
    }
    return OSI_OK;
}

#define WIFI_BUFF_SIZE 1544

//static unsigned char rxbuff[WIFI_BUFF_SIZE];
static void WiFiCB_receive(WlanRole_e role, uint8_t *inBuf, uint32_t inLen)
{
    void *hNetIf;

    if(role == WLAN_ROLE_STA)
    {
        hNetIf = m_wifiCtx.hStaNetif;
    }
    else
    {
        UART_PRINT("WiFiCB_receive:: Unknown Role = %d\r\n", role);
        return;
    }

#ifdef TCPIP_IF_ZERO_COPY
#error
#else
    int rc = OSI_OK;
    //UART_PRINT("IP Receive (len = %d) :: payload[0:3] = 0x%x:0x%x:0x%x:0x%x\r\n", inLen, *inBuf, *(inBuf+1), *(inBuf+2), *(inBuf+3));
    LOG_WIFI(LOG_RX, "WIFI", inLen, inBuf);
      rc = TCPIP_IF_receive(hNetIf, inBuf, inLen);
     if (rc != OSI_OK)
     {
         // If an error occurred, make sure the pbuf gets freed.
         UART_PRINT("LWIP Receive Error (%d)\r\n", rc);
     }
#endif
}


//*****************************************************************************
//
//! \brief  WiFi (SimpleLink driver and SlWifiConn) initialization
//!
//*****************************************************************************
int WIFI_IF_init(unsigned char bNetworkStackInit)
{
    int rc = WLAN_RET_CODE_DEV_ALREADY_STARTED;
    WlanMacAddress_t wlanMacAddress;

    if(!m_wifiCtx.bIsStarted)
    {
        ContextInit();
        rc = Wlan_Start(OnWifiDriverEvent);

#ifdef WIFI_LED_INDEX
       LED_IF_set(WIFI_LED_INDEX, 0);
#endif

        if(rc == OSI_OK)
        {
            uint32_t powerManagement = (uint32_t)POWER_MANAGEMENT_ELP_MODE;

            rc = TCPIP_IF_init(OnTcpipEvent, bNetworkStackInit);
            if (rc == OSI_OK)
            {
                if(!m_wifiCtx.hStaNetif)
                {
                    wlanMacAddress.roleType = WLAN_ROLE_STA;
                    Wlan_Get(WLAN_GET_MACADDRESS, &wlanMacAddress);
                    m_wifiCtx.hStaNetif = TCPIP_IF_addInterface("st", wlanMacAddress.pMacAddress, TcpipCB_send, TCPIP_IF_FLAGS_DHCPC|TCPIP_IF_FLAGS_DEFAULT);
                }
            }
            if (m_wifiCtx.hStaNetif == NULL)
            {
                UART_PRINT("WIFI Interface not started, TCPIP_IF_addInterface error = %d\r\n", rc);
            }

            Wlan_Set(WLAN_SET_POWER_MANAGEMENT, &powerManagement);

        }
        else
        {
            UART_PRINT("WIFI Interface not started, Wlan_Start error = %d\r\n", rc);
        }
    }
    return rc;
}


//*****************************************************************************
//
//! \brief  try to connect to an AP (based on provisioning, stored profiles
//!           or hard-coded setting). If timeout is provided, the function will
//!          block until connection is established or timeout occurs.         
//!
//*****************************************************************************
int WIFI_IF_start(WifiEventHandler_f handler, WifiServiceLevel_e level, unsigned long timeout_ms, void **hConnReq)
{
    int rc = OSI_MEMORY_ALLOCATION_FAILURE;
    bool bFirst;
    void *hConnRequest = AddConnRequest(handler, level, &bFirst);
    if(hConnRequest)
    {
        if(bFirst)
        {
            RoleUpStaCmd_t RoleUpStaParams;
            memset(&RoleUpStaParams, 0, sizeof(RoleUpStaCmd_t));
            strncpy((char *)RoleUpStaParams.countryDomain, (char *)"00", 2);
            UART_PRINT("\n\tChosen domain is WW\n");

            TCPIP_IF_setInterfaceState(m_wifiCtx.hStaNetif, E_TCPIP_IF_UP);

            Wlan_RoleUp(WLAN_ROLE_STA, &RoleUpStaParams, WLAN_WAIT_FOREVER);
#if WIFI_IF_STATIC_PROFILE_SUPPORT
#if (AP_SEC_TYPE == WLAN_SEC_TYPE_OPEN)
            rc = Wlan_Connect((const signed char *)AP_SSID, strlen(AP_SSID), NULL, WLAN_SEC_TYPE_OPEN, NULL , 0, 0);
#else
            rc = Wlan_Connect((const signed char *)AP_SSID, strlen(AP_SSID), NULL, AP_SEC_TYPE, AP_PASSWORD , strlen(AP_PASSWORD), 0);
#endif
#else
    #warning "Please enable WIFI_IF_STATIC_PROFILE_SUPPORT and set the AP credentials (AP_SSID, AP_SEC_TYPE, AP_PASSWORD) in wifi_settings.h!!!"
#endif

            if(rc < 0)
            {
                UART_PRINT("[wlanconnect] : error connecting to AP: %d\r\n", rc);
            }
        }
        if(m_wifiCtx.currConnStatus == WIFI_STATUS_DISCONNECTED)
        {
            m_wifiCtx.currConnStatus = WIFI_STATUS_WAITING_FOR_CONNECTION;
        }
        if(m_wifiCtx.currConnStatus < (WifiConnStatus_e)(WIFI_STATUS_CONNECTED + level))
        {
            if(WaitForConn(hConnRequest, timeout_ms) != OSI_OK)
            {
                rc = OSI_OPERATION_FAILED;
            }
            else
            {
                rc = OSI_OK;
            }
            if(hConnReq)
            {
                *hConnReq = hConnRequest;
            }
        }
    }
    return rc;
}

//*****************************************************************************
//
//! \brief  try to connect to an AP (based on provisioning, stored profiles
//!           or hard-coded setting). If timeout is provided, the function will
//!          block until connection is established or timeout occurs.         
//!
//*****************************************************************************
int WIFI_IF_stop(void *hConnReq)
{
    int rc = OSI_OK;
    bool bLast;

    DelConnRequest(hConnReq, &bLast);

    if(bLast)
    {
        unsigned long timeout_ms = 0xffff;
        TCPIP_IF_setInterfaceState(m_wifiCtx.hStaNetif, E_TCPIP_IF_DOWN);
        rc = Wlan_Disconnect(WLAN_ROLE_STA,NULL);

       /* Wait for connection events:
         * In order to verify that connection was successful,
         * we pend on two incoming events: Connected and Ip acquired.
         * The semaphores below are pend by this (Main) context.
         * They will be signaled once an asynchronous event
         * Indicating that the NWP has connected and acquired IP address is raised.
         * For further information, see this application read me file.
         */
        if(timeout_ms && !IS_STA_CONNECTED(m_wifiCtx.Status))
        {
            rc = osi_SyncObjWait(&m_wifiCtx.currentConn.disconnectEventSyncObj, timeout_ms);
        }
    }

    return rc;
}



//*****************************************************************************
//
//! \brief  A request for NWP reset
//!
//*****************************************************************************
int WIFI_IF_reset()
{
    return OSI_OK;
}

//*****************************************************************************
//
//! \brief  Free SlWifiConn resources (including the module's thread)
//!
//*****************************************************************************
int WIFI_IF_deinit()
{
    int rc = WLAN_RET_CODE_DEV_NOT_STARTED;
    /*** TBD - delete all pending connRequests ***/

    if(m_wifiCtx.bIsStarted)
    {
        m_wifiCtx.bIsStarted = false;
#if 0
        TCPIP_IF_deleteInterface(m_wifiCtx.hStaNetif);
        m_wifiCtx.hStaNetif = 0;
#endif

        rc = Wlan_Stop(TRUE);
    }
    return rc;
}

//*****************************************************************************
//
//! \brief  scan for surrounding APs
//!
//*****************************************************************************
int WIFI_IF_scan(signed char RoleID)
{
    int ret;

    ret = Wlan_Scan(RoleID,
                    NULL,
                    WLAN_MAX_SCAN_COUNT);

    return ret;
}

//*****************************************************************************
//
//! \brief  get index from scanning list
//!
//*****************************************************************************
int WIFI_IF_getNetEntry(uint8_t netIdx, WlanNetworkEntry_t   *netEntry)
{
    if (netIdx <= m_wifiCtx.netEntriesSize)
    {
        os_memcpy(netEntry, &m_wifiCtx.netEntries[netIdx-1], sizeof(WlanNetworkEntry_t));
        return OSI_OK;
    }
    else
    {
        return OSI_INVALID_PARAMS;
    }
}

//*****************************************************************************
//
//! \brief  get size of scanning list
//!
//*****************************************************************************
uint8_t WIFI_IF_getNetEntrySize()
{
    return m_wifiCtx.netEntriesSize;
}

//*****************************************************************************
//
//! \brief  connect to AP
//!
//*****************************************************************************
int WIFI_IF_connect(void *hConnReq, uint8_t netIdx, int8_t *password, uint8_t passLen, WifiServiceLevel_e level, unsigned long timeout_ms)
{
    uint8_t secType;
    int rc;

    m_wifiCtx.currConnStatus = WIFI_STATUS_WAITING_FOR_CONNECTION;

    switch (WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(m_wifiCtx.netEntries[netIdx-1].SecurityInfo))
    {
        case 0:
            secType = WLAN_SEC_TYPE_OPEN;
            break;
        case 2:
        case 4:
        case 6:
            secType = WLAN_SEC_TYPE_WPA_WPA2;
            break;
        case 5:
            secType = WLAN_SEC_TYPE_WPA3;
            break;
        case 7:
            secType = WLAN_SEC_TYPE_WPA2_PLUS;
            break;
    }

    rc = Wlan_Connect((const signed char *)m_wifiCtx.netEntries[netIdx-1].Ssid, strlen(m_wifiCtx.netEntries[netIdx-1].Ssid), NULL, secType, (int8_t *)password, passLen, 0);

    if(m_wifiCtx.currConnStatus < (WifiConnStatus_e)(WIFI_STATUS_CONNECTED + level))
    {
        if(WaitForConn(hConnReq, timeout_ms) != 0)
        {
            rc = OSI_OPERATION_FAILED;
        }
        else
        {
            rc = OSI_OK;
        }
    }

    return rc;
}
