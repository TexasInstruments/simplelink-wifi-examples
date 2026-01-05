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

   Application Name     - Connection manager application
   Application Overview - This application demonstrates how to use
                          the provisioning method
                        in order to establish connection to the AP.
                        The application
                        connects to an AP and ping's the gateway to
                        verify the connection.

   Application Details  - Refer to 'Connection manager' README.html

 *****************************************************************************/
//****************************************************************************
//
//! \addtogroup
//! @{
//
//****************************************************************************

/* Standard Include */

/* TI-DRIVERS Header files */

#include "lwip/autoip.h"
#include "lwip/dhcp.h"
#include "lwip/err.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#if LWIP_IPV6
#include "lwip/ethip6.h"
#endif
#include "lwip/tcpip.h"
#include "osi_kernel.h"
#include "tcpip_if.h"

#include "uart_term.h"


#define ETH_MAX_PAYLOAD 1514
#define VLAN_TAG_SIZE (4U)
#define ETHHDR_SIZE 14
#define ETH_FRAME_SIZE (ETH_MAX_PAYLOAD + VLAN_TAG_SIZE)

/****************************************************************************
              TYPE DEFINITIONS
 ****************************************************************************/
#ifndef bool
typedef enum { false, true } bool;
#endif

/****************************************************************************
              GLOBAL VARIABLES
 ****************************************************************************/
/* dhcp struct for the ethernet netif */
static struct dhcp   netif_dhcp;

/* autoip struct for the ethernet netif */
static struct autoip netif_autoip;

/* User Callback */
static EvtCallback_f  m_fEvtCallback = NULL;
static bool           m_bStackInit = false;
static bool           m_bLinkUp = false;
static OsiSyncObj_t   m_initSem;
static SendCallback_f m_fSendCallback = NULL;

//*****************************************************************************
//                 Local Functions
//*****************************************************************************

/* LWIP Callbacks */
static void  cb_initCompleteInd(void * arg);
static err_t cb_addInterfaceCompleteInd(struct netif *pNetif);
static void  cb_linkStatusInd(struct netif *pNetif);
static void  cb_interfaceStatusInd(struct netif *pNetif);
static void  cb_addInterfaceDeferred(void *ctx);
static void  cb_linkUpDeferred(void *hNewif);
static void  cb_linkDownDeferred(void *hNewif);
static err_t cb_wifiSend(struct netif *pNetif, struct pbuf * pktPBuf);


/* This function initializes this lwIP test. When NO_SYS=1, this is done in
 * the main_loop context (there is no other one), when NO_SYS=0, this is done
 * in the tcpip_thread context */
static void cb_initCompleteInd(void * arg)
{ 
    UART_PRINT("%s\r\n", __func__);
    OsiSyncObj_t *init_sem = (OsiSyncObj_t*)arg;
    LWIP_ASSERT("arg != NULL", arg != NULL);
    osi_SyncObjSignal(init_sem);
}

static err_t cb_addInterfaceCompleteInd(struct netif *pNetif)
{
    UART_PRINT("%s\r\n", __func__);
    netif_set_status_callback(pNetif, cb_interfaceStatusInd);
    netif_set_link_callback(pNetif, cb_linkStatusInd);
    autoip_set_struct(pNetif, &netif_autoip);
    dhcp_set_struct(pNetif, &netif_dhcp);

    pNetif->output          = etharp_output;
#if LWIP_IPV6
    pNetif->output_ip6      = ethip6_output;
#endif
    pNetif->linkoutput      = cb_wifiSend;
    pNetif->flags          |= (NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP | NETIF_FLAG_BROADCAST);
    pNetif->mtu             = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;

    netif_set_default(pNetif);
    osi_SyncObjSignal(&m_initSem);

    return ERR_OK;
}

static void cb_addInterfaceDeferred(void *hNewif)
{
    UART_PRINT("%s\r\n", __func__);
    struct netif *pNetif = hNewif;
    ip4_addr_t ipaddr = {0}, netmask = {0}, gw = {0};

    if(!netif_is_up(pNetif)){
        netif_add(pNetif, &ipaddr, &netmask, &gw, NULL, cb_addInterfaceCompleteInd, tcpip_input);
        netif_set_default(pNetif);
        UART_PRINT("Default Interface Added\r\n");
    }
}
static void cb_deleteInterfaceDeferred(void *hNewif)
{
    UART_PRINT("%s\r\n", __func__);
    struct netif *pNetif = hNewif;

    netif_remove(pNetif);
    free(pNetif);
}


static void cb_linkStatusInd(struct netif *pNetif)
{
    UART_PRINT("%s\r\n", __func__);
    err_t err;

    if (netif_is_link_up(pNetif))
    {
        if(strncmp(pNetif->name, "st", 2) == 0) 
        {
            const ip4_addr_t *ipv4 = netif_ip4_addr(pNetif);
            if(!ipv4->addr)
            {
                UART_PRINT("Link (%.2s) is UP starting DHCPS\r\n", pNetif->name);
#ifdef STATIC_IP
                err = dhcp_start(pNetif);
                dhcp_inform(pNetif);
                etharp_gratuitous(pNetif);
#else
                err = dhcp_start(pNetif);
#endif
#if LWIP_IPV6
                netif_create_ip6_linklocal_address(pNetif, 1);
                netif_set_ip6_autoconfig_enabled(pNetif, 1);
#endif
                UART_PRINT("DHCP is %d\r\n", err);
            }
            else
            {
                dhcp_inform(pNetif);
                etharp_gratuitous(pNetif);
            }
        }
        else if(strncmp(pNetif->name, "ap", 2) == 0) 
        {
            UART_PRINT("DHCPS is not not needed for MQTT client\r\n");
#if 0
            const ip4_addr_t *ipv4 = netif_ip4_addr(pNetif);
            if(ipv4->addr)
            {
                UART_PRINT("Link (%.2s) is UP starting DHCPS\r\n", pNetif->name);
                DHCPS_IF_start(ipv4->addr, pNetif);
                dhcp_inform(pNetif);
                etharp_gratuitous(pNetif);
            }
#endif
        }
    }
    else
    {
        if(strncmp(pNetif->name, "st", 2) == 0)
        {
            dhcp_stop(pNetif);
            UART_PRINT("DHCP stopped\r\n");
        }
        else if(strncmp(pNetif->name, "ap", 2) == 0) 
        {
            UART_PRINT("DHCPS is not not needed for MQTT client\r\n");
#if 0
            DHCPS_IF_stop();
            UART_PRINT("DHCP Server stopped\r\n");
#endif
        }
        UART_PRINT("[LWIP-EVENT:LINK] Link (%.2s) is DOWN\r\n", pNetif->name);
    }

}



static void cb_interfaceStatusInd(struct netif *pNetif)
{
    UART_PRINT("%s\r\n", __func__);
    if (netif_is_up(pNetif))
    {
        const ip4_addr_t *pIpV4;
        unsigned long status = E_TCPIP_STATUS_IP_LOST;
        pIpV4 = netif_ip4_addr(pNetif);

        if(pIpV4->addr)
        {
            char strAddr[16], strGW[16], strNetmask[16];

            ip4addr_ntoa_r(netif_ip4_addr(pNetif), strAddr, sizeof(strAddr));
            ip4addr_ntoa_r(netif_ip4_gw(pNetif), strGW, sizeof(strGW));
            ip4addr_ntoa_r(netif_ip4_netmask(pNetif), strNetmask, sizeof(strNetmask));
            UART_PRINT("STATUS:: IPv4: UP (ip %s, gw %s, mask %s)\r\n", strAddr, strGW, strNetmask);
            status = E_TCPIP_STATUS_IP_ACQUIRED;
        }
        else
        {
            UART_PRINT("STATUS:: IPv4: DOWN\r\n");
        }
#if LWIP_IPV6
        for(int i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
            if (netif_ip6_addr_state(pNetif, i) != 0) {
                const ip6_addr_t *pIpV6;
                uint8_t state = netif_ip6_addr_state(pNetif, i);
                pIpV6 = netif_ip6_addr(pNetif, i);
                UART_PRINT("STATUS:: IPv6: %s state=%02x\r\n", ip6addr_ntoa(pIpV6), state);
                if(state == IP6_ADDR_PREFERRED)
                {
                    status = E_TCPIP_STATUS_IP_ACQUIRED;
                }
            }
        }
#endif
        if(m_fEvtCallback && status < E_TCPIP_STATUS_MAX)
            m_fEvtCallback (pNetif, status, NULL);
    }

}

static void cb_interfaceUpDeferred(void *hNewif)
{
    struct netif *pNetif = hNewif;
    netif_set_up(pNetif);
    UART_PRINT("%s:: %x\r\n", __func__, pNetif->flags);
}


static void cb_interfaceDownDeferred(void *hNewif)
{
    struct netif *pNetif = hNewif;
    netif_set_down(pNetif);
    UART_PRINT("%s:: %x\r\n", __func__, pNetif->flags);
}

static void cb_linkUpDeferred(void *hNewif)
{
    struct netif *pNetif = hNewif;
    netif_set_link_up(pNetif);
    m_bLinkUp = true;
    UART_PRINT("%s:: %x\r\n", __func__, pNetif->flags);
}

static void cb_linkDownDeferred(void *hNewif)
{
    struct netif *pNetif = hNewif;
    netif_set_link_down(pNetif);
    m_bLinkUp = false;
    UART_PRINT("%s:: %x\r\n", __func__, pNetif->flags);
}

static err_t cb_wifiSend(struct netif *pNetif, struct pbuf * p)
{
    if(netif_is_up(pNetif))
    {
        ASSERT_GENERAL((unsigned int)m_fSendCallback);
        m_fSendCallback(pNetif, p->payload, p->len, 0);
    }
    return (err_t) ERR_OK;
}

int TCPIP_IF_init(EvtCallback_f fEvtCallback, unsigned char bStackInit)
{
    UART_PRINT("%s::\r\n",  __func__);
    int rc = OSI_OK;

    if(bStackInit && !m_bStackInit)
    {
        OsiSyncObj_t init_sem;
        osi_SyncObjCreate(&init_sem);
        tcpip_init(cb_initCompleteInd, &init_sem);
        osi_SyncObjWait(&init_sem, OSI_WAIT_FOREVER);
        osi_SyncObjDelete(&init_sem); 
        osi_SyncObjCreate(&m_initSem);
        m_bStackInit = true;
    }
    m_fEvtCallback = fEvtCallback;

    return rc;
}


void * TCPIP_IF_addInterface(char *pName, uint8_t *pMacAddr, SendCallback_f fSendCallback, unsigned long flags)
{
    UART_PRINT("%s::\r\n",  __func__);
    struct netif *pNetif = NULL;

    if(m_fEvtCallback)
    {
        pNetif = (struct netif*)malloc(sizeof(struct netif));
        if(pNetif) 
        {
            memset(pNetif, 0, sizeof(struct netif));
            pNetif->name[0] = (pName)?pName[0]:'i';
            pNetif->name[1] = (pName)?pName[1]:'f';
            memcpy(pNetif->hwaddr, pMacAddr, 6);
            pNetif->hwaddr_len = 6;
            tcpip_callback(cb_addInterfaceDeferred, pNetif);
            osi_SyncObjWait(&m_initSem, OSI_WAIT_FOREVER);
            m_fSendCallback = fSendCallback;
        }
    }
    UART_PRINT("addInterface(%s): %02x:%02x:%02x:%02x:%02x:%02x\r\n", pName, pMacAddr[0], pMacAddr[1], pMacAddr[2], pMacAddr[3], pMacAddr[4], pMacAddr[5]);
    return pNetif;
}

int TCPIP_IF_setIp4Addr(void *hNetIf, ip4addr_t _ipaddr, ip4addr_t _netmask, ip4addr_t _gw)
{
    struct netif *pNetIf = (struct netif *)hNetIf;
    ip4_addr_t ipaddr, netmask, gw;
    ipaddr.addr = _ipaddr;
    netmask.addr = _netmask;
    gw.addr = _gw;

    if(!netif_is_up(pNetIf)){
        netif_set_addr(pNetIf, &ipaddr, &netmask, &gw);
        netif_set_ipaddr(pNetIf, &ipaddr);
    }
    return OSI_OK;
}

int TCPIP_IF_getIp4Addr(void *hNetIf, ip4addr_t *pIp4Addr, ip4addr_t *pIp4Mask, ip4addr_t *pIp4Gateway)
{
    int rc = OSI_INVALID_PARAMS;
    const ip4_addr_t *pLwipIp4Addr;
    struct netif *pNetIf = (struct netif *)hNetIf;


    if(pIp4Addr && pIp4Mask && pIp4Gateway)
    {
        pLwipIp4Addr = netif_ip4_addr(pNetIf);
        if(pLwipIp4Addr)
        {
            *pIp4Addr = pLwipIp4Addr->addr;
            *pIp4Mask = (netif_ip4_netmask(pNetIf))->addr;
            *pIp4Gateway = (netif_ip4_gw(pNetIf))->addr;
            rc = OSI_OK;
        }
        else
        {
            *pIp4Addr = 0;
            *pIp4Mask = 0;
            *pIp4Gateway = 0;
        }
    }
    return rc;
}


void TCPIP_IF_deleteInterface(void *hNetIf)
{
    tcpip_callback(cb_deleteInterfaceDeferred, hNetIf);
}

int TCPIP_IF_setInterfaceState(void *hNetIf, TcpipInterfaceState_e state)
{
    if (state == E_TCPIP_IF_UP)
    {
        UART_PRINT("[TCPIP_IF_setInterfaceState] I/F - UP!\r\n");
        tcpip_callback(cb_interfaceUpDeferred, hNetIf);
    }
    else
    {
        UART_PRINT("[TCPIP_IF_setInterfaceState] I/F - DOWN!\r\n");
        tcpip_callback(cb_interfaceDownDeferred, hNetIf);
    }
    return OSI_OK;
}

int TCPIP_IF_notifyLinkChange(void *hNetIf, TcpipLinkState_e state)
{
    if (state == E_TCPIP_LINK_UP)
    {
        UART_PRINT("[TCPIP_IF_notifyLinkChange] I/F - CONNECTED (MAC LEVEL)!\r\n");
        tcpip_callback(cb_linkUpDeferred, hNetIf);
    }
    else
    {
        UART_PRINT("[TCPIP_IF_notifyLinkChange] I/F - DISCONNECTED!\r\n");
        tcpip_callback(cb_linkDownDeferred, hNetIf);
    }
    return OSI_OK;
}

#ifdef TCPIP_IF_ZERO_COPY
void *TCPIP_IF_pktAlloc(uint16_t pktSize)
{
    return pbuf_alloc(PBUF_LINK, pktSize, PBUF_POOL);
}

void  TCPIP_IF_pktFree(void *hPkt)
{
    pbuf_free(hPkt);
}

void *TCPIP_IF_pktPayload(void *hPkt)
{
    return ((struct pbuf *)hPkt)->payload; 
}

int   TCPIP_IF_pktLength(void *hPkt)
{
    return ((struct pbuf *)hPkt)->len;
}
#endif

int   TCPIP_IF_receive(void *hNetif, void *hPkt, uint16_t pktLen)
{
    struct netif *pNetif = (struct netif*)hNetif;
#ifdef TCPIP_IF_ZERO_COPY
    struct pbuf *pPkt = (struct pbuf *)hPkt;
#else
    struct pbuf *pPkt =  pbuf_alloc(PBUF_LINK, pktLen, PBUF_POOL);
    memcpy(pPkt->payload, hPkt ,pktLen);
#endif
    pPkt->len = pktLen;
    pPkt->tot_len = pktLen;
    return pNetif->input(pPkt, hNetif);
}

