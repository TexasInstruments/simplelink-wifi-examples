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
/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* lwIP core includes */
#include "lwip/opt.h"

#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/init.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/api.h"

#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/dhcp.h"
#include "lwip/autoip.h"
#include "lwip/pbuf.h"
#include "lwip/sockets.h"
#include "lwip/nd6.h"
#include "lwip/ip6_addr.h"
NETIF_DECLARE_EXT_CALLBACK(netif_ipv6_callback)

/* lwIP netif includes */
#include "lwip/etharp.h"
#include "lwip/ethip6.h"
#include "netif/ethernet.h"

#include "lwip/dhcp.h"

/* applications includes */
#include "lwip/apps/netbiosns.h"
#include "lwip/apps/httpd.h"
#include "apps/httpserver/httpserver-netconn.h"
#include "apps/netio/netio.h"
#include "apps/ping/ping.h"
#include "apps/rtp/rtp.h"
#include "apps/chargen/chargen.h"
#include "apps/shell/shell.h"
#include "apps/tcpecho/tcpecho.h"
#include "apps/udpecho/udpecho.h"
#include "apps/tcpecho_raw/tcpecho_raw.h"
#include "apps/socket_examples/socket_examples.h"

#include "examples/lwiperf/lwiperf_example.h"
#include "examples/mdns/mdns_example.h"
#include "examples/snmp/snmp_example.h"
#include "examples/tftp/tftp_example.h"
#include "examples/sntp/sntp_example.h"
#include "examples/mqtt/mqtt_example.h"


#include "examples/httpd/cgi_example/cgi_example.h"
#include "examples/httpd/fs_example/fs_example.h"
#include "examples/httpd/ssi_example/ssi_example.h"

#include "default_netif.h"
#include "wlan_if.h"
#include "network_lwip.h"
#include "lwipopts.h"


#if NO_SYS
/* ... then we need information about the timer intervals: */
#include "lwip/ip4_frag.h"
#include "lwip/igmp.h"
#endif /* NO_SYS */

#include "netif/ppp/ppp_opts.h"
#if PPP_SUPPORT
/* PPP includes */
#include "lwip/sio.h"
#include "netif/ppp/pppapi.h"
#include "netif/ppp/pppos.h"
#include "netif/ppp/pppoe.h"
#if !NO_SYS && !LWIP_PPP_API
#error With NO_SYS==0, LWIP_PPP_API==1 is required.
#endif
#endif /* PPP_SUPPORT */

/* include the port-dependent configuration */
//#include "lwipcfg.h"

//#include "test_enet_lwip.h"

#include "uart_term.h"
#include "wlan_cmd.h"
#include "wlan_if.h"
#include "osi_kernel.h"
#include "dhcpserver.h"
#include "network_terminal.h"

#ifndef LWIP_EXAMPLE_APP_ABORT
#define LWIP_EXAMPLE_APP_ABORT() 0
#endif

/** Define this to 1 to enable a port-specific ethernet interface as default interface. */
#ifndef USE_DEFAULT_ETH_NETIF
#define USE_DEFAULT_ETH_NETIF 1
#endif

/** Define this to 1 to enable a PPP interface. */
#ifndef USE_PPP
#define USE_PPP 0
#endif

/** Define this to 1 or 2 to support 1 or 2 SLIP interfaces. */
#ifndef USE_SLIPIF
#define USE_SLIPIF 0
#endif

/** Use an ethernet adapter? Default to enabled if port-specific ethernet netif or PPPoE are used. */
#ifndef USE_ETHERNET
#define USE_ETHERNET  (USE_DEFAULT_ETH_NETIF || PPPOE_SUPPORT)
#endif

/** Use an ethernet adapter for TCP/IP? By default only if port-specific ethernet netif is used. */
#ifndef USE_ETHERNET_TCPIP
#define USE_ETHERNET_TCPIP  (USE_DEFAULT_ETH_NETIF)
#endif

#if USE_SLIPIF
#include <netif/slipif.h>
#endif /* USE_SLIPIF */

#ifndef USE_DHCP
#define USE_DHCP    LWIP_DHCP
#endif
#ifndef USE_AUTOIP
#define USE_AUTOIP  LWIP_AUTOIP
#endif

#define ETH_MAX_PAYLOAD  1514
#define VLAN_TAG_SIZE         (4U)
#define ETHHDR_SIZE     14
#define ETH_FRAME_SIZE        (ETH_MAX_PAYLOAD + VLAN_TAG_SIZE)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* globales variables for netifs */
#if USE_ETHERNET
#if LWIP_DHCP
/* dhcp struct for the ethernet netif */
struct dhcp netif_dhcp;
#endif /* LWIP_DHCP */
#if LWIP_AUTOIP
/* autoip struct for the ethernet netif */
struct autoip netif_autoip;
#endif /* LWIP_AUTOIP */
#endif /* USE_ETHERNET */
#if USE_PPP
/* THE PPP PCB */
ppp_pcb *ppp;
/* THE PPP interface */
struct netif ppp_netif;
/* THE PPP descriptor */
u8_t sio_idx = 0;
sio_fd_t ppp_sio;
#endif /* USE_PPP */
#if USE_SLIPIF
struct netif slipif1;
#if USE_SLIPIF > 1
struct netif slipif2;
#endif /* USE_SLIPIF > 1 */
#endif /* USE_SLIPIF */


//#define STATIC_IP
#ifdef STATIC_IP
#ifdef CC35XX
const char static_ip[4]     = {10,0,0,103};
const char static_gw[4]     = {10,0,0,1};
#endif
#ifdef CC33XX
const char static_ip[4]     = {10,123,45,12};
const char static_gw[4]     = {10,123,45,1};
#endif 
const char static_mask[4]   = {255,255,255,0};

#endif


struct netif staif = {0};
struct netif apif = { .ip_addr.u_addr.ip4.addr = PP_HTONL(LWIP_MAKEU32(10, 0, 0, 3)),
                      .netmask.u_addr.ip4.addr = PP_HTONL(LWIP_MAKEU32(255, 255, 255, 0)),
                      .gw.u_addr.ip4.addr      = PP_HTONL(LWIP_MAKEU32(10, 0, 0, 3)) };
extern appControlBlock app_CB;

/* Tracks whether STA has acquired an IP address. For STATIC mode: set to 1 when IP configured
 * via menu to prevent erroneous DHCP restart on link-up. For DHCP: set to 0 on true disconnect
 * (link_callback DOWN) to allow DHCP restart on reconnect. WPA3 EAP rekey link flaps occur at
 * driver level and do NOT trigger link_callback, so isIpAcquired remains stable during rekey. */
static uint32_t isIpAcquired;
static uint8_t sta_ip_mode = IP_DHCP;
static uint32_t last_reported_ipv4;  /* Track last reported IPv4 to suppress duplicate status prints */
static uint8_t last_dhcp_waiting_printed;  /* Suppress repetitive "Waiting for DHCP" messages */
static uint8_t ap_ip_mode = IP_DHCP;
static void (*extra_status_callback)(WlanRole_e roleid, uint32_t address, uint32_t local_ipv6[4], uint32_t global_ipv6[4]);
static uint8_t ipv6_callback_registered = 0;

static sys_sem_t sta_ip_config_done;

/* Deferred gratuitous ARP callback: fires after EAPOL HW blocks are freed.
 * Used when static IP is already set before connection, or as a reliable
 * fallback on link-up. The delay ensures EAPOL TX completes first. */
#define GRAT_ARP_DEFER_MS         200
#define STA_IP_CONFIG_TIMEOUT_MS  15000  /* max wait for IP config signal before returning to caller */
static void deferred_gratuitous_arp_cb(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    if (netif && netif_is_up(netif) && netif_is_link_up(netif) &&
        !ip4_addr_isany(netif_ip4_addr(netif)))
    {
        etharp_gratuitous(netif);
    }
}
static uint8_t sta_ip_config_sem_initialized = 0;
static uint8_t waiting_for_sta_ip_config = 0;
static sys_mutex_t sta_ip_config_mutex;
static uint8_t sta_ip_config_mutex_initialized = 0;

struct netif *network_netif_find_by_ip6(const uint8_t *ip6_bytes)
{
    /* Link-local addresses (fe80::/10) are scoped to a single interface.
     * Search all netifs for the one that owns the given IPv6 address.
     * Returns that netif so the caller can use it for zone assignment
     * (ip6_addr_assign_zone) or scope_id (sin6_scope_id = netif->num + 1).
     * Falls back to the STA netif if no match is found. */
    struct netif *n;
    int idx;
    for (n = netif_list; n != NULL; n = n->next) {
        for (idx = 0; idx < LWIP_IPV6_NUM_ADDRESSES; idx++) {
            if (memcmp(n->ip6_addr[idx].u_addr.ip6.addr, ip6_bytes, 16) == 0) {
                return n;
            }
        }
    }
    return (struct netif *)network_get_sta_if();
}

int update_arp(void* ip_addr)
{
    err_t result = ERR_CONN;
    ssize_t arp_find;
    struct netif *pNetIf = &staif;
    struct eth_addr* eth_ret = NULL;
    ip4_addr_t* ip4_ret = NULL;
    ip4_addr_t* ip4_addr = (ip4_addr_t*)ip_addr;

    if(netif_is_up(pNetIf))
    {
        LOCK_TCPIP_CORE();
        result = etharp_query(pNetIf, (const ip4_addr_t *) ip4_addr, NULL);
        UNLOCK_TCPIP_CORE();
        if(result == ERR_OK)
        {
            result = ERR_CONN;
            //wait here some time for reply to be received
            uint32_t timeout_ms = 3000; // 3 seconds timeout
            uint32_t elapsed_ms = 0;
            uint32_t sleep_interval_ms = 10; // Check every 10ms
            while(elapsed_ms < timeout_ms)
            {
                osi_Sleep(sleep_interval_ms);//set time to other thread to get the reply
                eth_ret = NULL;
                ip4_ret = NULL;
                arp_find = etharp_find_addr(pNetIf,(ip4_addr_t *)ip4_addr, &eth_ret,(const ip4_addr_t **)&ip4_ret);
                if(arp_find <0 )
                {
                    result = ERR_CONN;
                }
                else
                {
                    if (ip4_ret && eth_ret && ip4_addr_cmp(ip4_ret, ip4_addr) )
                    {
                       result = ERR_OK;
                       break;//found
                    }
                }
                elapsed_ms += sleep_interval_ms;
            }
        }
    }
    return result;
}

#define IPV6_RA_TIMEOUT_MS  10000

static void ipv6_ra_timeout(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    int i;
    for (i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
        if ((netif->ip6_addr_state[i] & IP6_ADDR_VALID) &&
            !ip6_addr_islinklocal(netif_ip6_addr(netif, i))) {
            return; /* global address already assigned - no need to warn */
        }
    }
    Report("\r\nIPv6: No Router Advertisement received - router may not support IPv6\r\n");
    Report("IPv6: Only link-local address available, no global IPv6 address will be assigned\r\n");

    /* Signal completion for DHCP mode when RA timeout (no global address) */
    if (netif == network_get_sta_if() && waiting_for_sta_ip_config &&
        sta_ip_config_sem_initialized && sta_ip_mode == IP_DHCP) {
        sys_sem_signal(&sta_ip_config_done);
        waiting_for_sta_ip_config = 0;
    }
}

static void ipv6_addr_state_callback(struct netif *netif,
                                     netif_nsc_reason_t reason,
                                     const netif_ext_callback_args_t *args)
{
    if (reason & LWIP_NSC_IPV6_ADDR_STATE_CHANGED) {
        s8_t idx = args->ipv6_addr_state_changed.addr_index;
        if (netif->ip6_addr_state[idx] & IP6_ADDR_VALID) {
            char ipv6_str[INET6_ADDRSTRLEN];
            const char *if_name = (netif == network_get_ap_if()) ? "AP" : "STA";
            inet_ntop(AF_INET6, (struct in6_addr*)&netif->ip6_addr[idx], ipv6_str, INET6_ADDRSTRLEN);
            if (ip6_addr_islinklocal(netif_ip6_addr(netif, idx))) {
                /* On static IP reconnect, status_callback prints IPv4 before link_callback
                 * initializes IPv6. Reprint the full block with IPv6 now that it is ready. */
                if (netif == network_get_sta_if() && last_reported_ipv4 != 0) {
                    Report("\r\n========== IP Configuration ==========\r\n");
                    Report("Status: CONNECTED (role: STA)\r\n");
                    Report("IPv4: %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
                    Report("IPv6: %s (link-local)\r\n", ipv6_str);
                    Report("=======================================\r\n");
                } else {
                    Report("\r\n%s IPv6 Link-local Address: %s\r\n", if_name, ipv6_str);
                }
                /* AP is the router - it will never receive an RA from itself, no need to warn */
                if (netif != network_get_ap_if()) {
                    /* cancel any pending RA timer before scheduling a new one - state callbacks
                     * can fire multiple times (tentative->deprecated->preferred) and duplicate
                     * timers exhaust the MEMP_SYS_TIMEOUT pool */
                    sys_untimeout(ipv6_ra_timeout, netif);
                    sys_timeout(IPV6_RA_TIMEOUT_MS, ipv6_ra_timeout, netif);
                }
            } else {
                sys_untimeout(ipv6_ra_timeout, netif);
                Report("\r\n%s IPv6 Global Address (SLAAC): %s\r\n", if_name, ipv6_str);
            }

            /* Signal completion when IPv6 global SLAAC address arrives (IPv4 is in DHCP mode) */
            if (netif == network_get_sta_if() && sta_ip_config_sem_initialized && waiting_for_sta_ip_config &&
                sta_ip_mode == IP_DHCP && !ip6_addr_islinklocal(netif_ip6_addr(netif, idx))) {
                sys_sem_signal(&sta_ip_config_done);
                waiting_for_sta_ip_config = 0;
            }
        }
    }
}

void status_callback(struct netif *state_netif)
{
    WlanRole_e roleid = WLAN_ROLE_NONE;
    if(network_get_sta_if() == state_netif)
    {
       //Note: there is no support for STA and P2P CL at the same time.
       if (GET_STATUS_BIT(app_CB.Status, STATUS_BIT_P2P_GROUP_STARTED) && app_CB.P2pGroupType == P2P_GROUP_TYPE_CLIENT)
        {
            roleid = WLAN_ROLE_AP;
        }
        else
        {
            roleid = WLAN_ROLE_STA;
        }
    }
    else if (network_get_ap_if() == state_netif)
    {
        roleid = WLAN_ROLE_AP;
    }
    if (netif_is_up(state_netif))
    {
        const ip4_addr_t *temp;
        ip6_addr_t local_ipv6 = {0};
        ip6_addr_t global_ipv6 = {0};

        WlanMacAddress_t macAddressParams;
        memset(&macAddressParams, 0, sizeof(WlanMacAddress_t));

        macAddressParams.roleType = roleid;

        Wlan_Get(WLAN_GET_MACADDRESS, (void *)&macAddressParams);

        memcpy(state_netif->hwaddr, macAddressParams.pMacAddress, sizeof (macAddressParams.pMacAddress));

        state_netif->hwaddr_len = 6;

        /* Remember whether IPv6 link-local was already configured before this callback.
         * Used below to signal the DHCP semaphore on re-DHCP cycles where the RA timer
         * is not re-armed (IPv6 state never transitions through INVALID again). */
        int ipv6_was_valid = (roleid == WLAN_ROLE_STA) &&
                             !ip6_addr_isinvalid(netif_ip6_addr_state(state_netif, 0));

        if (roleid == WLAN_ROLE_STA &&
            ip6_addr_isinvalid(netif_ip6_addr_state(state_netif, 0))) {
            /* Check address STATE (not value) to catch invalid IPv6 after rekey, where the
             * address bytes (fe80::) may persist but state is marked INVALID by driver.
             * EUI-64 is derived from unique MAC so collision is impossible; skip DAD. */
            netif_create_ip6_linklocal_address(state_netif, 1);
            netif_ip6_addr_set_state(state_netif, 0, IP6_ADDR_PREFERRED);
        }
        temp = netif_ip4_addr(state_netif);
        

        if (temp->addr)
        {
            isIpAcquired = 1;
            /* Print IP config only on IP change. Since link-down always resets last_reported_ipv4
             * (never triggered by brief rekey flaps), any reconnect with same static IP will still
             * print because last_reported_ipv4 will be 0 after the disconnect. */
            if (temp->addr != last_reported_ipv4) {
                last_reported_ipv4 = temp->addr;
                last_dhcp_waiting_printed = 0;
                Report("\r\n========== IP Configuration ==========\r\n");
                Report("Status: CONNECTED (role: %s)\r\n", roleid == WLAN_ROLE_STA ? "STA" : "AP");
                Report("IPv4: %s\r\n", ip4addr_ntoa(netif_ip4_addr(state_netif)));
                if (roleid == WLAN_ROLE_STA) {
                    int idx, found = 0;
                    for (idx = 0; idx < LWIP_IPV6_NUM_ADDRESSES; idx++) {
                        if (netif_ip6_addr_state(state_netif, idx) & IP6_ADDR_VALID) {
                            char ipv6_str[INET6_ADDRSTRLEN];
                            inet_ntop(AF_INET6, (struct in6_addr*)&state_netif->ip6_addr[idx], ipv6_str, INET6_ADDRSTRLEN);
                            if (ip6_addr_islinklocal(netif_ip6_addr(state_netif, idx))) {
                                os_memcpy(local_ipv6.addr, state_netif->ip6_addr[idx].u_addr.ip6.addr, sizeof(ip6_addr_t));
                                if (!found) Report("IPv6: ");
                                Report("%s (link-local)", ipv6_str);
                            } else {
                                os_memcpy(global_ipv6.addr, state_netif->ip6_addr[idx].u_addr.ip6.addr, sizeof(ip6_addr_t));
                                if (!found) Report("IPv6: ");
                                Report("%s (global)", ipv6_str);
                            }
                            if (idx < LWIP_IPV6_NUM_ADDRESSES - 1) Report(" | ");
                            found = 1;
                        }
                    }
                    if (found) Report("\r\n");

                    /* For STATIC mode, signal when IPv6 is also configured (created in status_callback) */
                    if (waiting_for_sta_ip_config && sta_ip_config_sem_initialized && sta_ip_mode == IP_STATIC) {
                        sys_sem_signal(&sta_ip_config_done);
                        waiting_for_sta_ip_config = 0;
                    }
                    /* For DHCP mode on re-DHCP cycles: IPv6 was already valid so ipv6_addr_state_callback
                     * will not fire for link-local and the RA timer will not be re-armed. Signal here
                     * directly once the new DHCP address is acquired - IPv6 setup is already complete. */
                    else if (waiting_for_sta_ip_config && sta_ip_config_sem_initialized &&
                             sta_ip_mode == IP_DHCP && ipv6_was_valid) {
                        sys_sem_signal(&sta_ip_config_done);
                        waiting_for_sta_ip_config = 0;
                    }
                }
                Report("=======================================\r\n");
            }
            if (roleid == WLAN_ROLE_STA)
            {
                netif_set_default(state_netif);
            }
            if (app_CB.CON_CB.dhcpIprecvSyncObj)
            {
                osi_SyncObjSignal(&app_CB.CON_CB.dhcpIprecvSyncObj);
            }

        }
        else
        {
            if (!last_dhcp_waiting_printed && sta_ip_mode != IP_STATIC) {
                Report("\n\rWaiting for DHCP IP address...\r\n");
                last_dhcp_waiting_printed = 1;
            }
            isIpAcquired = 0;
            last_reported_ipv4 = 0;
        }

        if (extra_status_callback)
        {
            extra_status_callback(roleid, temp->addr, local_ipv6.addr, global_ipv6.addr);
        }
    }
    else
    {
        Report("\n\rLink DOWN - Interface disconnected (role: %s)\r\n", roleid == WLAN_ROLE_STA ? "STA" : roleid == WLAN_ROLE_AP ? "AP" : "NONE");

        /* Signal completion for STA when going DOWN in STATIC mode (e.g., invalid IP 0.0.0.0) */
        if (roleid == WLAN_ROLE_STA && waiting_for_sta_ip_config &&
            sta_ip_config_sem_initialized && sta_ip_mode == IP_STATIC) {
            sys_sem_signal(&sta_ip_config_done);
            waiting_for_sta_ip_config = 0;
        }
    }
}


void printBuffer(uint8_t *in,uint32_t len)
{
    int j, total;
    total  = 0;

    Report("\n\r");

    while(total < 8)
    {
        for(j = 0; j < 8 ; j++)
        {
            Report("%x ",in[total]);
            total++;
        }
        Report("\n\r");
    }
}

void network_recv(WlanRole_e roleId, uint8_t *inBuf, uint32_t inLen)
{
    struct netif *pIf = NULL;
    struct pbuf *packet;    
    err_t err;


    if(roleId == WLAN_ROLE_STA)
    {
        pIf = (struct netif *)network_get_sta_if();
    }
    else if(roleId == WLAN_ROLE_AP)
    {
        pIf = (struct netif *)network_get_ap_if();
    }
    else
    {
        Report("\n\r network_recv,  no role available,packet DROP, roleId:%d",roleId);
        return;
    }

    packet = pbuf_alloc(PBUF_RAW , inLen, PBUF_POOL);
    if(!packet){
        return;
    }
    memcpy(packet->payload,inBuf,inLen);
    packet->len = inLen;
    packet->tot_len = inLen;

    if ((err = tcpip_input(packet, pIf)) != 0) {
        pbuf_free(packet);
        return;
    }
}

void link_callback(struct netif *state_netif)
{
    struct netif *newif = state_netif;
    WlanRole_e roleid = WLAN_ROLE_NONE;
    err_t err = ERR_OK;
    if (network_get_sta_if() == newif)
    {
        roleid = WLAN_ROLE_STA;
    }
    else if (network_get_ap_if() == newif)
    {
        roleid = WLAN_ROLE_AP;
    }
    if (netif_is_link_up(newif))
    {
        Wlan_EtherPacketRecvRegisterCallback(roleid, network_recv);
        if (roleid == WLAN_ROLE_STA)
        {
        	/* Only restart DHCP if IP was not acquired. This prevents redundant DHCP requests
        	 * on spurious link events while connected, but allows recovery after transient
        	 * link drops (e.g., WPA3 EAP rekeying) where isIpAcquired is cleared on link-down. */
        	if (!isIpAcquired)
        	{
            	Report("\n\r link_callback==UP starting DHCP");
#ifdef STATIC_IP
            	err = dhcp_start(state_netif);
            	dhcp_inform(state_netif);
            	etharp_gratuitous(state_netif);
#else
                if (sta_ip_mode == IP_DHCP)
            	{
            	    err = dhcp_start(state_netif);
            	    Report("\n\r DHCP is %d\n\n\r", err);
            	}
                else if (sta_ip_mode == IP_STATIC)
                {
                    /* Static IP not yet assigned via set_if_ip (called after connection).
                     * Schedule deferred ARP anyway - deferred_gratuitous_arp_cb() guards
                     * against zero IP at fire time, so it only fires after the IP is set.
                     * This covers the case where network_stack_set_static_ip_if_sta()
                     * post-config ARP arrives too late relative to the ping window. */
                    sys_untimeout(deferred_gratuitous_arp_cb, state_netif);
                    sys_timeout(GRAT_ARP_DEFER_MS, deferred_gratuitous_arp_cb, state_netif);
                }
#endif
        	}
         	else
         	{
         	    /* IP already configured (set_if_ip called before connection or on reconnect).
         	     * isIpAcquired=1 path: but note status_callback often clears isIpAcquired=0
         	     * before link_callback fires, so in practice this branch may rarely run.
         	     * The !isIpAcquired+IP_STATIC branch above is the reliable path. */
         	    if (sta_ip_mode == IP_STATIC &&
         	        !ip4_addr_isany(netif_ip4_addr(state_netif)))
         	    {
         	        sys_untimeout(deferred_gratuitous_arp_cb, state_netif);
         	        sys_timeout(GRAT_ARP_DEFER_MS, deferred_gratuitous_arp_cb, state_netif);
         	    }
         	}

        	/* Reinitialize IPv6 link-local if invalid. Runs for both DHCP and STATIC modes.
        	 * Checks address STATE (not value) so it correctly fires after rekey, where
        	 * link-down invalidated the state but left the old fe80:: address value intact. */
        	if (ip6_addr_isinvalid(netif_ip6_addr_state(state_netif, 0))) {
        	    netif_create_ip6_linklocal_address(state_netif, 1);
        	    netif_ip6_addr_set_state(state_netif, 0, IP6_ADDR_PREFERRED);
        	}
        	/* Enable IPv6 autoconfiguration now that link is up */
        	netif_set_ip6_autoconfig_enabled(state_netif, 1);


        }
        else if (roleid == WLAN_ROLE_AP)
        {
            Report("\n\rlink_callback==UP starting DHCP Server\n\r");
            dhcps_start(newif->ip_addr.u_addr.ip4.addr, newif);
            dhcp_inform(newif);
            etharp_gratuitous(newif);

            if (ip6_addr_isany(netif_ip6_addr(newif, 0))) {
                /* Link is now up - safe to send MLD reports. EUI-64 is derived from
                 * the AP's own MAC so collision is impossible; skip DAD. */
                netif_create_ip6_linklocal_address(newif, 1);
                netif_ip6_addr_set_state(newif, 0, IP6_ADDR_PREFERRED);
            }

            char ap_ipv4_str[16];
            inet_ntop(AF_INET, &state_netif->ip_addr.u_addr.ip4, ap_ipv4_str, sizeof(ap_ipv4_str));
            Report("\n\r=== Network Configuration (AP) ===\n\r");
            Report("AP IPv4 Address: %s\n\r", ap_ipv4_str);
            if (netif_ip6_addr_state(state_netif, 0) & IP6_ADDR_VALID) {
                char ap_ipv6_str[INET6_ADDRSTRLEN];
                inet_ntop(AF_INET6, (struct in6_addr*)&state_netif->ip6_addr[0], ap_ipv6_str, sizeof(ap_ipv6_str));
                Report("AP IPv6 Address: %s\n\r", ap_ipv6_str);
            }
            Report("Use this address to connect to device servers\n\r");
            Report("===================================\n\r");
        }

       // err = autoip_start(state_netif);
       // Report("autoip is %d\r\n", err);
    }
    else
    {
        sys_untimeout(deferred_gratuitous_arp_cb, state_netif);
        Wlan_EtherPacketRecvRegisterCallback(roleid, NULL);
        if (roleid == WLAN_ROLE_STA)
        {
            dhcp_stop(state_netif);
            /* Clear isIpAcquired only for DHCP mode to allow DHCP restart on link-up after
             * transient drops (e.g., WPA3 EAP rekeying). For STATIC mode, keep it set so
             * link-up takes the else branch (etharp_gratuitous) instead of restarting DHCP. */
            if (sta_ip_mode == IP_DHCP)
            {
                isIpAcquired = 0;
            }
            /* Always reset IP tracking on true disconnect. link_callback DOWN is never triggered
             * by brief rekey flaps (those are handled at higher layers), so any DOWN here is a
             * real disconnect. Clearing tracking ensures IP config reprints on reconnect, even
             * if reconnecting with the same static IP address. */
            last_reported_ipv4 = 0;
            last_dhcp_waiting_printed = 0;
            Report("DHCP stopped\r\n");

            /* Clear all IPv6 addresses on link-down to prevent stale state after transient
             * link flaps. Invalidates pre-rekeying IPv6 addresses and cancels pending RA
             * timeouts to ensure clean IPv6 initialization on link-up. */
            sys_untimeout(ipv6_ra_timeout, newif);
            {
                int idx;
                for (idx = 0; idx < LWIP_IPV6_NUM_ADDRESSES; idx++) {
                    netif_ip6_addr_set_state(newif, idx, IP6_ADDR_INVALID);
                }
            }
        }
        else if (roleid == WLAN_ROLE_AP)
        {
            dhcps_stop();
            Report("DHCP Server stopped\r\n");
        }

        Report("link_callback==DOWN\r\n");
    }
}



err_t network_send(struct netif *netif, struct pbuf *p)
{
    WlanRole_e role;
    uint16_t total_len;
    struct pbuf *currentPacket = p;
    uint8_t *buff;
    uint16_t offset = 0;

    total_len = currentPacket->tot_len;

    if(osi_GetFreeHeapSize() < HEAP_THRESHOLD_FOR_TX)
    {
        return ERR_MEM;//no room for tx send
    }


    if(total_len != currentPacket->len)
    {
        buff = os_malloc(total_len);
        if(buff == NULL )
        {
            Report("\r\n allocation of TX buffer failed");
            return ERR_MEM;
        }
    //aggregate all packets
        do
        {
            os_memcpy(buff + offset,
                      currentPacket->payload,
                      currentPacket->len);
            offset += currentPacket->len;
            currentPacket = currentPacket->next;
        }
        while (currentPacket);
    }
    else
    {
        buff = currentPacket->payload;
    }


    if (netif_is_up(netif))
    {
        if (netif == &staif)
        {
            role = WLAN_ROLE_STA;
        }
        else
        {
            role = WLAN_ROLE_AP;
        }

        Wlan_EtherPacketSend(role, buff, total_len, 0);
#ifdef CC33XX
        osi_uSleep(10);
#endif // CC33XX
    }
    if (buff != currentPacket->payload)
    {
        os_free(buff);
    }
    //done review
    return ERR_OK;
}


void tcpinternal_network_set_up(void *newif)
{
    struct netif *nif = newif;
#ifdef STATIC_IP
    WlanRole_e role;
    if(nif == &staif)
    {
        role = WLAN_ROLE_STA;
    }
    else
    {
        role = WLAN_ROLE_AP;
    }

    if(WLAN_ROLE_AP != role)
    {
        struct ip4_addr ipaddr;
        struct ip4_addr netmask;
        struct ip4_addr gw;

        IP4_ADDR(&ipaddr, static_ip[0], static_ip[1], static_ip[2], static_ip[3]);
        IP4_ADDR(&netmask, static_mask[0], static_mask[1], static_mask[2], static_mask[3]);
        IP4_ADDR(&gw, static_gw[0], static_gw[1], static_gw[2], static_gw[3]);

        dhcp_release(nif);

        dhcp_stop(nif);

        netif_set_addr(nif, &ipaddr, &netmask, &gw);
    }
#endif
    netif_set_up(nif);
    nif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;
    netif_set_link_up(nif);
}

void tcpinternal_network_set_down(void *newif)
{
    struct netif *nif = newif;
    netif_set_down(nif);
    nif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;
    netif_set_link_down(nif);

}

signed char _role_sta_up(struct netif *newif)
{
    netif_set_status_callback(newif, status_callback);
    netif_set_link_callback(newif, link_callback);
    autoip_set_struct(newif, &netif_autoip);
    dhcp_set_struct(newif, &netif_dhcp);

    newif->name[0] = 's';
    newif->name[1] = 't';

    newif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;

    /* Populate the Driver Interface Functions. */
    newif->remove_callback      = NULL;
    newif->output               = etharp_output;
    newif->linkoutput           = network_send;
    newif->flags               |=  NETIF_FLAG_BROADCAST |
                                   NETIF_FLAG_ETHARP |
                                   NETIF_FLAG_IGMP;
    newif->output_ip6           = ethip6_output;
    newif->flags               |=  NETIF_FLAG_MLD6;
    if (!ipv6_callback_registered) {
        netif_add_ext_callback(&netif_ipv6_callback, ipv6_addr_state_callback);
        ipv6_callback_registered = 1;
    }

    if (app_CB.CON_CB.staRoleupSyncObj != NULL)
    {
        //Signal to application that Wlan_RoleUp done
        osi_SyncObjSignal(&app_CB.CON_CB.staRoleupSyncObj);
    }
    return 0;
}

signed char _role_ap_up(struct netif *newif)
{
    netif_set_status_callback(newif, status_callback);
    netif_set_link_callback(newif, link_callback);
    //autoip_set_struct(newif, &netif_autoip);
    //dhcp_set_struct(newif, &netif_dhcp);

    newif->name[0] = 'a';
    newif->name[1] = 'p';

    newif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;

    /* Populate the Driver Interface Functions. */
    newif->remove_callback      = NULL;
    newif->output               = etharp_output;
    newif->linkoutput           = network_send;
    newif->flags               |=  NETIF_FLAG_BROADCAST |
                                   NETIF_FLAG_ETHARP |
                                   NETIF_FLAG_IGMP;
    newif->output_ip6           = ethip6_output;
    newif->flags               |=  NETIF_FLAG_MLD6;
    if (!ipv6_callback_registered) {
        netif_add_ext_callback(&netif_ipv6_callback, ipv6_addr_state_callback);
        ipv6_callback_registered = 1;
    }

    return 0;
}

void tcpip_network_stack_add_if_sta(void *ctx)
{
    ip4_addr_t ipaddr, netmask, gw;

    struct netif *pNetIf = &staif;
    if (!netif_is_up(pNetIf))
    {
        memset(pNetIf, 0, sizeof(struct netif));

        ip4_addr_set_zero(&gw);
        ip4_addr_set_zero(&ipaddr);
        ip4_addr_set_zero(&netmask);

        netif_add(pNetIf, &ipaddr, &netmask, &gw, NULL, _role_sta_up, tcpip_input);
        netif_set_default(pNetIf);
    }
}

void tcpip_network_stack_remove_if_sta(void *ctx)
{
    struct netif *pNetIf = &staif;
    tcpinternal_network_set_down(pNetIf);

    /* Clear IP from lwIP after the netif is down so the address change does not trigger
     * status_callback (lwIP only fires it when netif is UP). Covers both static and DHCP. */
    dhcp_release_and_stop(pNetIf);
    netif_set_addr(pNetIf, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4);

    sta_ip_mode = IP_DHCP;
    isIpAcquired = 0;
    last_reported_ipv4 = 0;
    last_dhcp_waiting_printed = 0;
    waiting_for_sta_ip_config = 0;

    sys_untimeout(ipv6_ra_timeout, pNetIf);
    if (!netif_is_up(&apif)) {
        netif_remove_ext_callback(&netif_ipv6_callback);
        ipv6_callback_registered = 0;
    }
    etharp_cleanup_netif(pNetIf);
    netif_remove(pNetIf);
    osi_SyncObjSignal(&app_CB.CON_CB.staRoledownSyncObj);

    if (netif_is_up(&apif))
    {
        netif_set_default(&apif);
    }

}

void tcpip_network_stack_add_if_ap(void *ctx)
{
    ip4_addr_t ipaddr, netmask, gw;

    struct netif *pNetIf = network_get_ap_if();
    if (!netif_is_up(pNetIf))
    {
        ipaddr.addr = pNetIf->ip_addr.u_addr.ip4.addr;
        netmask.addr = pNetIf->netmask.u_addr.ip4.addr;
        gw.addr = pNetIf->gw.u_addr.ip4.addr;

        os_memset(pNetIf, 0, sizeof(struct netif));

        netif_add(pNetIf, &ipaddr, &netmask, &gw, NULL, _role_ap_up, tcpip_input);
        netif_set_addr(pNetIf, &ipaddr, &netmask, &gw);
        netif_set_ipaddr(pNetIf, &ipaddr);
        netif_set_up(pNetIf);

        if(!netif_is_up(&staif))
        {
            netif_set_default(pNetIf);
        }

    }
}

void tcpip_network_stack_remove_if_ap(void *ctx)
{
    struct netif *pNetIf = &apif;
    tcpinternal_network_set_down(pNetIf);

    if (!netif_is_up(&staif)) {
        netif_remove_ext_callback(&netif_ipv6_callback);
        ipv6_callback_registered = 0;
    }
    netif_remove(pNetIf);
}

/* This function initializes this lwIP test. When NO_SYS=1, this is done in
 * the main_loop context (there is no other one), when NO_SYS=0, this is done
 * in the tcpip_thread context */
void tcpip_network_internal_init(void * arg)
{ /* remove compiler warning */
  sys_sem_t *init_sem;
  LWIP_ASSERT("arg != NULL", arg != NULL);
  init_sem = (sys_sem_t*)arg;
  sys_sem_signal(init_sem);
}

/*************************** network API *****************************/

void * network_get_sta_if()
{
    return (void *)&staif;
}

void * network_get_ap_if()
{
    return (void *)&apif;
}


void network_set_up(void *newif)
{
    //tcpip_callback(tcpinternal_network_set_up, newif);
    LOCK_TCPIP_CORE();

    tcpinternal_network_set_up(newif);

    UNLOCK_TCPIP_CORE();

}


void network_set_down(void *newif)
{    /* suspect need to move to when there is wlan connect event */
    if(netif_is_up((struct netif *)newif))
    {
        //tcpip_callback(tcpinternal_network_set_down, newif);

        LOCK_TCPIP_CORE();

        tcpinternal_network_set_down(newif);

        UNLOCK_TCPIP_CORE();
    }
}

void network_stack_add_if_sta()
{
    LOCK_TCPIP_CORE();

    tcpip_network_stack_add_if_sta(NULL);

    UNLOCK_TCPIP_CORE();
}

void network_stack_remove_if_sta()
{
    //tcpip_callback(tcpip_network_stack_remove_if_sta, NULL);
    LOCK_TCPIP_CORE();

    tcpip_network_stack_remove_if_sta(NULL);

    UNLOCK_TCPIP_CORE();
}

void network_stack_add_if_ap()
{
    //tcpip_callback(tcpip_network_stack_add_if_ap, NULL);
    LOCK_TCPIP_CORE();

    tcpip_network_stack_add_if_ap(NULL);

    UNLOCK_TCPIP_CORE();
}

void network_stack_remove_if_ap()
{
    //tcpip_callback(tcpip_network_stack_remove_if_ap, NULL);
    LOCK_TCPIP_CORE();

    tcpip_network_stack_remove_if_ap(NULL);

    UNLOCK_TCPIP_CORE();

}

void network_stack_init()
{
    sys_sem_t init_sem;
    sys_sem_new(&init_sem,0);
    tcpip_init(tcpip_network_internal_init,&init_sem);
    sys_sem_wait(&init_sem);
    sys_sem_free(&init_sem);
}

void network_stack_set_ap_ip_mode(uint32_t mode)
{
    ap_ip_mode = mode;
}

void network_stack_set_sta_ip_mode(uint32_t mode)
{
    sta_ip_mode = mode;
}

/* Set static IP address for STA interface with synchronous handshake.
 * Blocks until lwIP completes the IP configuration (netif UP) to ensure
 * address is stable before caller proceeds (e.g., wlan_set_if_ip command).
 * This prevents ping failures where the address wasn't yet applied to the netif. */
void network_stack_set_static_ip_if_sta(uint32_t ip, uint32_t netmask, uint32_t gw)
{
    ip4_addr_t ip_addr = { .addr = ip };
    ip4_addr_t netmask_addr = { .addr = netmask };
    ip4_addr_t gw_addr = { .addr = gw };
    struct netif *pNetIf = network_get_sta_if();

    if (pNetIf)
    {
        /* Initialize semaphore and mutex for synchronizing static IP configuration handshake */
        if (!sta_ip_config_sem_initialized) {
            sys_sem_new(&sta_ip_config_done, 0);
            sys_mutex_new(&sta_ip_config_mutex);
            sta_ip_config_sem_initialized = 1;
            sta_ip_config_mutex_initialized = 1;
        }

        /* Serialize concurrent callers - only one IP-config handshake in flight at a time */
        sys_mutex_lock(&sta_ip_config_mutex);

        LOCK_TCPIP_CORE();

        /* Use atomic release_and_stop to prevent race between clearing DHCP state and IP application */
        dhcp_release_and_stop(pNetIf);

        /* When ip=0 (clearing the address between connections) restore DHCP mode so that
         * the next link_callback UP calls dhcp_start, sending DHCP DISCOVER broadcasts.
         * Those broadcasts are DATA frames that go through the AP's bridge and trigger
         * source-learning, refreshing the FDB entry to the current 802.11 association.
         * This restores the 3.0.10.25 behaviour where DHCP mode was always active at
         * link-up time because sta_ip_mode tracking did not exist yet.
         * When ip!=0 (assigning a real address) we switch to STATIC as before. */
        network_stack_set_sta_ip_mode(ip != 0 ? IP_STATIC : IP_DHCP);

        /* isIpAcquired=1 prevents link-up from restarting DHCP on reconnect with static IP.
         * isIpAcquired=0 allows link-up to start DHCP (sends DISCOVERs that refresh AP FDB). */
        isIpAcquired = (ip != 0) ? 1 : 0;

        /* Signal status_callback to expect IP config notification for this call */
        if (ip != 0) {
            waiting_for_sta_ip_config = 1;
        }

        /* dhcp_release_and_stop() returns early (no-op) when DHCP state is already OFF, which
         * is the case after a previous static IP call. If the desired IP is the same as the
         * current one, lwIP will skip the status_callback because it detects no address change,
         * and the semaphore will never be signaled. Clear the address first to force a real
         * address transition that fires the callback. */
        if (ip != 0 && netif_ip4_addr(pNetIf)->addr == ip) {
            netif_set_addr(pNetIf, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4);
        }

        netif_set_addr(pNetIf, &ip_addr, &netmask_addr, &gw_addr);

        UNLOCK_TCPIP_CORE();

        /* Synchronous wait: block until status_callback signals static IP is applied (netif UP).
         * This ensures netif address is stable before returning to caller (e.g., ping after connect).
         * Skip for ip=0 (clearing IP during potential link-down) to prevent deadlock. */
        if (ip != 0) {
            if (sys_arch_sem_wait(&sta_ip_config_done, STA_IP_CONFIG_TIMEOUT_MS) == SYS_ARCH_TIMEOUT) {
                Report("\r\n[ERROR] set_static_ip: timed out waiting for IP config signal\r\n");
                waiting_for_sta_ip_config = 0;
            }
        }

        sys_mutex_unlock(&sta_ip_config_mutex);
    }
}

void network_stack_set_dynamic_ip_if_sta()
{
    struct netif *pNetIf = network_get_sta_if();
    if (pNetIf)
    {
        /* Initialize semaphore and mutex if not already done */
        if (!sta_ip_config_sem_initialized) {
            sys_sem_new(&sta_ip_config_done, 0);
            sys_mutex_new(&sta_ip_config_mutex);
            sta_ip_config_sem_initialized = 1;
            sta_ip_config_mutex_initialized = 1;
        }

        /* Serialize concurrent callers - only one IP-config handshake in flight at a time */
        sys_mutex_lock(&sta_ip_config_mutex);

        LOCK_TCPIP_CORE();

        dhcp_stop(pNetIf);
        dhcp_release(pNetIf);

        netif_set_addr(pNetIf, NULL, NULL, NULL);

        network_stack_set_sta_ip_mode(IP_DHCP);
        waiting_for_sta_ip_config = 1;

        dhcp_start(pNetIf);

        UNLOCK_TCPIP_CORE();

        if (!netif_is_link_up(pNetIf))
        {
            Report("\n\rDHCP mode configured. Device is not connected to an AP yet.\n\r");
            Report("DHCP will start automatically when you connect to an AP.\n\r");
            sys_mutex_unlock(&sta_ip_config_mutex);
            return;
        }
        /* Wait for IPv4 DHCP to acquire an address. IPv6 uses SLAAC (RA-based), not DHCP. */
        if (sys_arch_sem_wait(&sta_ip_config_done, STA_IP_CONFIG_TIMEOUT_MS) == SYS_ARCH_TIMEOUT) {
            Report("\r\n[ERROR] set_dynamic_ip: timed out waiting for IPv4 DHCP config signal\r\n");
            waiting_for_sta_ip_config = 0;
        }

        /* Send gratuitous ARP only after STA is connected and DHCP address is stable.
         * By the time sys_sem_wait() returns here, the 4-way handshake is complete and
         * EAPOL HW blocks are free - DHCP acquisition takes several seconds post-connect. */
        if (netif_is_up(pNetIf) && netif_is_link_up(pNetIf) &&
            !ip4_addr_isany(netif_ip4_addr(pNetIf)))
        {
            LOCK_TCPIP_CORE();
            etharp_gratuitous(pNetIf);
            UNLOCK_TCPIP_CORE();
        }

        sys_mutex_unlock(&sta_ip_config_mutex);
    }
}

void network_stack_set_static_ip_if_ap(uint32_t ip, uint32_t netmask, uint32_t gw)
{
    ip4_addr_t ip_addr = { .addr = ip };
    ip4_addr_t netmask_addr = { .addr = netmask };
    ip4_addr_t gw_addr = { .addr = gw };
    struct netif *pNetIf = network_get_ap_if();

    if (pNetIf)
    {
        LOCK_TCPIP_CORE();

        netif_set_addr(pNetIf, &ip_addr, &netmask_addr, &gw_addr);
        network_stack_set_ap_ip_mode(IP_STATIC);

        UNLOCK_TCPIP_CORE();
    }
}

int8_t network_stack_set_dynamic_ip_if_ap(uint32_t ip, uint32_t netmask, uint32_t gw)
{
    ip4_addr_t ip_addr = { .addr = ip };
    ip4_addr_t netmask_addr = { .addr = netmask };
    ip4_addr_t gw_addr = { .addr = gw };
    
    struct netif *state_netif = network_get_ap_if();

    if (state_netif == NULL)
    {
        return -1;
    }
    
    LOCK_TCPIP_CORE();

    netif_set_addr(state_netif, &ip_addr, &netmask_addr, &gw_addr);

    network_stack_set_ap_ip_mode(IP_DHCP);

    UNLOCK_TCPIP_CORE();

    return 0;
}


int8_t network_stack_set_dhcp_server_if_ap(int enable)
{
    struct netif *pNetIf = network_get_ap_if();

    if (pNetIf == NULL)
    {
        return -1;
    }

    LOCK_TCPIP_CORE();

    if (enable)
    {
        dhcps_start(pNetIf->ip_addr.u_addr.ip4.addr, pNetIf);
        Report("\n\rDHCP Server: IPv4 DHCP enabled\n\r");
    }
    else
    {
        dhcps_stop();
        Report("\n\rDHCP Server: IPv4 DHCP disabled\n\r");
    }

    UNLOCK_TCPIP_CORE();

    return 0;
}

int8_t network_stack_get_if_ip(WlanRole_e role, uint32_t *ip, uint32_t *netmask, uint32_t *gw, uint32_t *dhcp)
{
    struct netif *intf = NULL;

    if (role == WLAN_ROLE_STA)
    {
        intf = (struct netif *)network_get_sta_if();
        if (dhcp)
        {
            if (sta_ip_mode == IP_DHCP)
            {
                *dhcp = true;
            }
            else if (sta_ip_mode == IP_STATIC)
            {
                *dhcp = false;
            }
        }
    }
    else if (role == WLAN_ROLE_AP)
    {
        intf = (struct netif *)network_get_ap_if();
        if (dhcp)
        {
            if (ap_ip_mode == IP_DHCP)
            {
                *dhcp = true;
            }
            else if (ap_ip_mode == IP_STATIC)
            {
                *dhcp = false;
            }
        }
    }
    else
    {
        return -1;
    }

    if (intf == NULL)
    {
        return -1;
    }

    if (ip)
    {
        *ip = netif_ip4_addr(intf)->addr;
    }
    if (netmask)
    {
        *netmask = netif_ip4_netmask(intf)->addr;
    }
    if (gw)
    {
        *gw = netif_ip4_gw(intf)->addr;
    }

    return 0;
}

void network_stack_register_extra_status_callback(void (*callback)(WlanRole_e, uint32_t, uint32_t[4], uint32_t[4]))
{
    extra_status_callback = callback;
}
