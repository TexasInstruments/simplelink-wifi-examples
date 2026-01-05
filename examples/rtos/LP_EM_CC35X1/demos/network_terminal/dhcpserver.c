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
#include "osi_kernel.h"
#include "lwip/inet.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/mem.h"
#include "lwip/ip_addr.h"
#include "dhcpserver.h"



#ifdef MEMLEAK_DEBUG
static const char mem_debug_file[]   = __FILE__;
#endif

////////////////////////////////////////////////////////////////////////////////////
static const uint32_t magic_cookie   = 0x63538263;
static struct udp_pcb *pcb_dhcps = NULL;
static struct ip_addr broadcast_dhcps;
static struct ip_addr server_address;
static struct ip_addr client_address;//added

struct ip_info ap_if_ip;

static struct dhcps_lease dhcps_lease;
//static BOOLEAN dhcps_lease_flag = TRUE;
static list_node *plist = NULL;
static uint8_t offer = 0xFF;
static BOOLEAN renew = FALSE;
#define DHCPS_LEASE_TIME_DEF    (120)
uint32_t dhcps_lease_time = DHCPS_LEASE_TIME_DEF;  //minute

void wifi_softap_dhcps_client_leave(uint8_t *bssid, struct ip_addr *ip,BOOLEAN force);
uint32_t wifi_softap_dhcps_client_update(uint8_t *bssid, struct ip_addr *ip);


void dhcps_set_ip_info(struct netif *netif)
{
    ap_if_ip.netmask.addr = netif->netmask.addr;
    ap_if_ip.ip.addr      = netif->ip_addr.addr;
    ap_if_ip.gw.addr      = netif->gw.addr;
}

void dhcps_get_ip_info(struct ip_info *if_ip)
{
    if_ip->netmask.addr   = ap_if_ip.netmask.addr;
    if_ip->ip.addr        = ap_if_ip.ip.addr;
    if_ip->gw.addr        = ap_if_ip.gw.addr;
}
/******************************************************************************
 * FunctionName : node_insert_to_list
 * Description  : insert the node to the list
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
void   node_insert_to_list(list_node **phead, list_node* pinsert)
{
    list_node *plist = NULL;
    struct dhcps_pool *pdhcps_pool = NULL;
    struct dhcps_pool *pdhcps_node = NULL;
    if (*phead == NULL)
        *phead = pinsert;
    else {
        plist = *phead;
        pdhcps_node = pinsert->pnode;
        pdhcps_pool = plist->pnode;

        if(pdhcps_node->ip.addr < pdhcps_pool->ip.addr) {
            pinsert->pnext = plist;
            *phead = pinsert;
        } else {
            while (plist->pnext != NULL) {
                pdhcps_pool = plist->pnext->pnode;
                if (pdhcps_node->ip.addr < pdhcps_pool->ip.addr) {
                    pinsert->pnext = plist->pnext;
                    plist->pnext = pinsert;
                    break;
                }
                plist = plist->pnext;
            }

            if(plist->pnext == NULL) {
                plist->pnext = pinsert;
            }
        }
    }
//  pinsert->pnext = NULL;
}

/******************************************************************************
 * FunctionName : node_delete_from_list
 * Description  : remove the node from list
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
void   node_remove_from_list(list_node **phead, list_node* pdelete)
{
    list_node *plist = NULL;

    plist = *phead;
    if (plist == NULL){
        *phead = NULL;
    } else {
        if (plist == pdelete){
            *phead = plist->pnext;
            pdelete->pnext = NULL;
        } else {
            while (plist != NULL) {
                if (plist->pnext == pdelete){
                    plist->pnext = pdelete->pnext;
                    pdelete->pnext = NULL;
                }
                plist = plist->pnext;
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////
/*
 * DHCP msg
 *
 * @param optptr -- DHCP msg
 * @param type -- option
 *
 * @return uint8_t* DHCP msgt
 */
///////////////////////////////////////////////////////////////////////////////////
static uint8_t*   add_msg_type(uint8_t *optptr, uint8_t type)
{

        *optptr++ = DHCP_OPTION_MSG_TYPE;
        *optptr++ = 1;
        *optptr++ = type;
        return optptr;
}
///////////////////////////////////////////////////////////////////////////////////
/*
 * DHCP msg offer
 *
 * @param optptr -- DHCP msg
 *
 * @return uint8_t* DHCP msgt
 */
///////////////////////////////////////////////////////////////////////////////////
static uint8_t*   add_offer_options(uint8_t *optptr)
{
        struct ip_addr ipadd;

        ipadd.addr = *( (uint32_t *) &server_address);

#ifdef USE_CLASS_B_NET
        *optptr++ = DHCP_OPTION_SUBNET_MASK;
        *optptr++ = 4;  //length
        *optptr++ = 255;
        *optptr++ = 240;
        *optptr++ = 0;
        *optptr++ = 0;
#else
        *optptr++ = DHCP_OPTION_SUBNET_MASK;
        *optptr++ = 4;
        *optptr++ = 255;
        *optptr++ = 255;
        *optptr++ = 255;
        *optptr++ = 0;
#endif

        *optptr++ = DHCP_OPTION_LEASE_TIME;
        *optptr++ = 4;
        *optptr++ = ((DHCPS_LEASE_TIMER * 60) >> 24) & 0xFF;
        *optptr++ = ((DHCPS_LEASE_TIMER * 60) >> 16) & 0xFF;
        *optptr++ = ((DHCPS_LEASE_TIMER * 60) >> 8) & 0xFF;
        *optptr++ = ((DHCPS_LEASE_TIMER * 60) >> 0) & 0xFF;

        *optptr++ = DHCP_OPTION_SERVER_ID;
        *optptr++ = 4;
        *optptr++ = ip4_addr1( &ipadd);
        *optptr++ = ip4_addr2( &ipadd);
        *optptr++ = ip4_addr3( &ipadd);
        *optptr++ = ip4_addr4( &ipadd);

        if (dhcps_router_enabled(offer)){
            struct ip_info if_ip;
            os_memset(&if_ip, 0, sizeof(struct ip_info));
            dhcps_get_ip_info(&if_ip);

            *optptr++ = DHCP_OPTION_ROUTER;
            *optptr++ = 4;
            *optptr++ = ip4_addr1( &if_ip.gw);
            *optptr++ = ip4_addr2( &if_ip.gw);
            *optptr++ = ip4_addr3( &if_ip.gw);
            *optptr++ = ip4_addr4( &if_ip.gw);
        }

#ifdef USE_DNS
        *optptr++ = DHCP_OPTION_DNS_SERVER;
        *optptr++ = 4;
        *optptr++ = ip4_addr1( &ipadd);
        *optptr++ = ip4_addr2( &ipadd);
        *optptr++ = ip4_addr3( &ipadd);
        *optptr++ = ip4_addr4( &ipadd);
#endif

#ifdef CLASS_B_NET
        *optptr++ = DHCP_OPTION_BROADCAST_ADDRESS;
        *optptr++ = 4;
        *optptr++ = ip4_addr1( &ipadd);
        *optptr++ = 255;
        *optptr++ = 255;
        *optptr++ = 255;
#else
        *optptr++ = DHCP_OPTION_BROADCAST_ADDRESS;
        *optptr++ = 4;
        *optptr++ = ip4_addr1( &ipadd);
        *optptr++ = ip4_addr2( &ipadd);
        *optptr++ = ip4_addr3( &ipadd);
        *optptr++ = 255;
#endif

        *optptr++ = DHCP_OPTION_INTERFACE_MTU;
        *optptr++ = 2;
#ifdef CLASS_B_NET
        *optptr++ = 0x05;
        *optptr++ = 0xdc;
#else
        *optptr++ = 0x02;
        *optptr++ = 0x40;
#endif

        *optptr++ = DHCP_OPTION_PERFORM_ROUTER_DISCOVERY;
        *optptr++ = 1;
        *optptr++ = 0x00;

        *optptr++ = 43;
        *optptr++ = 6;

        *optptr++ = 0x01;
        *optptr++ = 4;
        *optptr++ = 0x00;
        *optptr++ = 0x00;
        *optptr++ = 0x00;
        *optptr++ = 0x02;

        return optptr;
}
///////////////////////////////////////////////////////////////////////////////////
/*
 * DHCP msg
 *
 * @param optptr -- DHCP msg
 *
 * @return uint8_t* DHCP msgt
 */
///////////////////////////////////////////////////////////////////////////////////
static uint8_t*   add_end(uint8_t *optptr)
{

        *optptr++ = DHCP_OPTION_END;
        return optptr;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
static void   create_msg(struct dhcps_msg *m)
{
        struct ip_addr client;

        client.addr = client_address.addr;

        m->op = DHCP_REPLY;
        m->htype = DHCP_HTYPE_ETHERNET;
        m->hlen = 6;
        m->hops = 0;
//        os_memcpy((char *) xid, (char *) m->xid, sizeof(m->xid));
        m->secs = 0;
        m->flags = htons(BOOTP_BROADCAST);

        os_memcpy((char *) m->yiaddr, (char *) &client.addr, sizeof(m->yiaddr));

        os_memset((char *) m->ciaddr, 0, sizeof(m->ciaddr));
        os_memset((char *) m->siaddr, 0, sizeof(m->siaddr));
        os_memset((char *) m->giaddr, 0, sizeof(m->giaddr));
        os_memset((char *) m->sname, 0, sizeof(m->sname));
        os_memset((char *) m->file, 0, sizeof(m->file));

        os_memset((char *) m->options, 0, sizeof(m->options));

//For xiaomi crash bug
        uint32_t magic_cookie1 = magic_cookie;
        os_memcpy((char *) m->options, &magic_cookie1, sizeof(magic_cookie1));
}

struct pbuf * dhcps_pbuf_alloc(u16_t len)
{
    u16_t mlen = sizeof(struct dhcps_msg);

    if (len > mlen) {
#if DHCPS_DEBUG
        Report("dhcps: len=%d mlen=%d\n\r", len, mlen);
#endif
        mlen = len;
    }

    return pbuf_alloc(PBUF_TRANSPORT, mlen, PBUF_RAM);
}

///////////////////////////////////////////////////////////////////////////////////
/*
 * send_offer
 *
 * @param - m dhcps msg
 * @len - len
 */
///////////////////////////////////////////////////////////////////////////////////
static void   send_offer(struct dhcps_msg *m, u16_t len)
{
        uint8_t *end;
        struct pbuf *p, *q;
        u8_t *data;
        u16_t cnt=0;
        u16_t i;
        create_msg(m);

        end = add_msg_type(&m->options[4], DHCPOFFER);
        end = add_offer_options(end);
        end = add_end(end);

        p = dhcps_pbuf_alloc(len);
#if DHCPS_DEBUG
        Report("udhcp: send_offer>>p->ref = %d\n\r", p->ref);
#endif
        if(p != NULL){

#if DHCPS_DEBUG
            Report("dhcps: send_offer>>pbuf_alloc succeed\n\r");
            Report("dhcps: send_offer>>p->tot_len = %d\n\r", p->tot_len);
            Report("dhcps: send_offer>>p->len = %d\n\r", p->len);
#endif
            q = p;
            while(q != NULL){
                data = (u8_t *)q->payload;
                for(i=0; i<q->len; i++)
                {
                    data[i] = ((u8_t *) m)[cnt++];
                }

                q = q->next;
            }
        }else{

#if DHCPS_DEBUG
            Report("dhcps: send_offer>>pbuf_alloc failed\n\r");
#endif
            return;
        }
        ip_addr_t send;
        send.addr = broadcast_dhcps.addr;
#if DHCPS_DEBUG
        err_t SendOffer_err_t;
        SendOffer_err_t = udp_sendto( pcb_dhcps, p, &send, DHCPS_CLIENT_PORT );
        Report("dhcps: send_offer>>udp_sendto result %x\n\r",SendOffer_err_t);
#else
        udp_sendto( pcb_dhcps, p, &send, DHCPS_CLIENT_PORT );
#endif
        if(p->ref != 0){
#if DHCPS_DEBUG
            Report("udhcp: send_offer>>free pbuf\n\r");
#endif
            pbuf_free(p);
        }
}
///////////////////////////////////////////////////////////////////////////////////
/*
 * send_nak
 *
 * @param m - dhcps msg
 * @param len - length
 */
///////////////////////////////////////////////////////////////////////////////////
static void   send_nak(struct dhcps_msg *m, u16_t len)
{

        u8_t *end;
        struct pbuf *p, *q;
        u8_t *data;
        u16_t cnt=0;
        u16_t i;
        create_msg(m);

        end = add_msg_type(&m->options[4], DHCPNAK);
        end = add_end(end);

        p = dhcps_pbuf_alloc(len);
#if DHCPS_DEBUG
        Report("udhcp: send_nak>>p->ref = %d\n\r", p->ref);
#endif
        if(p != NULL){

#if DHCPS_DEBUG
            Report("dhcps: send_nak>>pbuf_alloc succeed\n\r");
            Report("dhcps: send_nak>>p->tot_len = %d\n\r", p->tot_len);
            Report("dhcps: send_nak>>p->len = %d\n\r", p->len);
#endif
            q = p;
            while(q != NULL){
                data = (u8_t *)q->payload;
                for(i=0; i<q->len; i++)
                {
                    data[i] = ((u8_t *) m)[cnt++];
                }

                q = q->next;
            }
        }else{

#if DHCPS_DEBUG
            Report("dhcps: send_nak>>pbuf_alloc failed\n\r");
#endif
            return;
        }
        ip_addr_t send;
        send.addr = broadcast_dhcps.addr;
#if DHCPS_DEBUG
        err_t SendNak_err_t;
        SendNak_err_t = udp_sendto( pcb_dhcps, p, &send, DHCPS_CLIENT_PORT );
        Report("dhcps: send_nak>>udp_sendto result %x\n\r",SendNak_err_t);
#else
        udp_sendto( pcb_dhcps, p, &send, DHCPS_CLIENT_PORT );
#endif
        if(p->ref != 0){
#if DHCPS_DEBUG
            Report("udhcp: send_nak>>free pbuf\n\r");
#endif
            pbuf_free(p);
        }
}
///////////////////////////////////////////////////////////////////////////////////
/*
 * send_ack
 *
 * @param m - dhcps msg
 * @param len - length
 */
///////////////////////////////////////////////////////////////////////////////////
static void   send_ack(struct dhcps_msg *m, u16_t len)
{

        u8_t *end;
        struct pbuf *p, *q;
        u8_t *data;
        u16_t cnt=0;
        u16_t i;

        create_msg(m);

        end = add_msg_type(&m->options[4], DHCPACK);
        end = add_offer_options(end);
        end = add_end(end);

        p = dhcps_pbuf_alloc(len);
#if DHCPS_DEBUG
        Report("udhcp: send_ack>>p->ref = %d\n\r", p->ref);
#endif
        if(p != NULL){

#if DHCPS_DEBUG
            Report("dhcps: send_ack>>pbuf_alloc succeed\n\r");
            Report("dhcps: send_ack>>p->tot_len = %d\n\r", p->tot_len);
            Report("dhcps: send_ack>>p->len = %d\n\r", p->len);
#endif
            q = p;
            while(q != NULL){
                data = (u8_t *)q->payload;
                for(i=0; i<q->len; i++)
                {
                    data[i] = ((u8_t *) m)[cnt++];
                }

                q = q->next;
            }
        }else{

#if DHCPS_DEBUG
            Report("dhcps: send_ack>>pbuf_alloc failed\n\r");
#endif
            return;
        }
        ip_addr_t send;
        send.addr = broadcast_dhcps.addr;
#if DHCPS_DEBUG
        err_t SendAck_err_t;
        SendAck_err_t = udp_sendto( pcb_dhcps, p, &send, DHCPS_CLIENT_PORT );
        Report("dhcps: send_ack>>udp_sendto result %x\n\r",SendAck_err_t);
#else
        udp_sendto( pcb_dhcps, p, &send, DHCPS_CLIENT_PORT );
#endif

        if(p->ref != 0){
#if DHCPS_DEBUG
            Report("udhcp: send_ack>>free pbuf\n\r");
#endif
            pbuf_free(p);
        }
}
///////////////////////////////////////////////////////////////////////////////////
/*
 * parse_options
 *
 * @param optptr DHCP msg
 * @param len length (byte)
 *
 * @return uint8_t state
 */
///////////////////////////////////////////////////////////////////////////////////
static uint8_t   parse_options(uint8_t *optptr, int16_t len)
{
        struct ip_addr client;
        BOOLEAN is_dhcp_parse_end = FALSE;
        struct dhcps_state s;

        client.addr = *( (uint32_t *) &client_address);// ??????DHCP??????IP

        u8_t *end = optptr + len;
        u16_t type = 0;

        s.state = DHCPS_STATE_IDLE;

        while (optptr < end) {
#if DHCPS_DEBUG
            Report("dhcps: (int16_t)*optptr = %d\n\r", (int16_t)*optptr);
#endif
            switch ((int16_t) *optptr) {

                case DHCP_OPTION_MSG_TYPE:  //53
                        type = *(optptr + 2);
                        break;

                case DHCP_OPTION_REQ_IPADDR://50
                        //Report("dhcps:0x%08x,0x%08x\n\r",client.addr,*(uint32_t*)(optptr+2));
                        if( os_memcmp( (char *) &client.addr, (char *) optptr+2,4)==0 ) {
#if DHCPS_DEBUG
                            Report("dhcps: DHCP_OPTION_REQ_IPADDR = 0 ok\n\r");
#endif
                            s.state = DHCPS_STATE_ACK;
                        }else {
#if DHCPS_DEBUG
                            Report("dhcps: DHCP_OPTION_REQ_IPADDR != 0 err\n\r");
#endif
                            s.state = DHCPS_STATE_NAK;
                        }
                        break;
                case DHCP_OPTION_END:
                        {
                            is_dhcp_parse_end = TRUE;
                        }
                        break;
            }

            if(is_dhcp_parse_end){
                    break;
            }

            optptr += optptr[1] + 2;
        }

        switch (type){

            case DHCPDISCOVER://1
                s.state = DHCPS_STATE_OFFER;
#if DHCPS_DEBUG
                Report("dhcps: DHCPD_STATE_OFFER\n\r");
#endif
                break;

            case DHCPREQUEST://3
                if ( !(s.state == DHCPS_STATE_ACK || s.state == DHCPS_STATE_NAK) ) {
                    if(renew == TRUE) {
                        s.state = DHCPS_STATE_ACK;
                    } else {
                        s.state = DHCPS_STATE_NAK;
                    }
#if DHCPS_DEBUG
                        Report("dhcps: DHCPD_STATE_NAK\n\r");
#endif
                }
                break;

            case DHCPDECLINE://4
                s.state = DHCPS_STATE_IDLE;
#if DHCPS_DEBUG
                Report("dhcps: DHCPD_STATE_IDLE\n\r");
#endif
                break;

            case DHCPRELEASE://7
                s.state = DHCPS_STATE_RELEASE;
#if DHCPS_DEBUG
                Report("dhcps: DHCPD_STATE_IDLE\n\r");
#endif
                break;
        }
#if DHCPS_DEBUG
        Report("dhcps: return s.state = %d\n\r", s.state);
#endif
        return s.state;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
static int16_t   parse_msg(struct dhcps_msg *m, u16_t len)
{
    if(os_memcmp((char *)m->options,
            &magic_cookie,
            sizeof(magic_cookie)) == 0){
        struct ip_addr ip;
        os_memcpy(&ip.addr,m->ciaddr,sizeof(ip.addr));
        client_address.addr = wifi_softap_dhcps_client_update(m->chaddr,&ip);

        int16_t ret = parse_options(&m->options[4], len);

        if(ret == DHCPS_STATE_RELEASE) {
            wifi_softap_dhcps_client_leave(m->chaddr,&ip,TRUE); // force to delete
            client_address.addr = ip.addr;
        }

        return ret;
    }
    return 0;
}
///////////////////////////////////////////////////////////////////////////////////
/*
 * handle_dhcp
 * udp_recv() from l
 *
 * @param arg
 * @param pcb
 * @param p
 * @param addr
 * @param port
 */
///////////////////////////////////////////////////////////////////////////////////
static void   handle_dhcp(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port)

{
        struct dhcps_msg *pmsg_dhcps = NULL;
        int16_t tlen = 0, malloc_len;
        u16_t i = 0;
        u16_t dhcps_msg_cnt = 0;
        u8_t *p_dhcps_msg = NULL;
        u8_t *data = NULL;

#if DHCPS_DEBUG
        Report("dhcps: handle_dhcp-> receive a packet\n\r");
#endif
        if (p==NULL) return;

        malloc_len = sizeof(struct dhcps_msg);
#if DHCPS_DEBUG
        Report("dhcps: handle_dhcp malloc_len=%d rx_len=%d\n\r", malloc_len, p->tot_len);
#endif
        if (malloc_len < p->tot_len) {
            malloc_len = p->tot_len;
        }

        pmsg_dhcps = (struct dhcps_msg *)malloc(malloc_len);
        if (NULL == pmsg_dhcps){
            pbuf_free(p);
            return;
        }
        memset(pmsg_dhcps , 0x00 , malloc_len);

        p_dhcps_msg = (u8_t *)pmsg_dhcps;
        tlen = p->tot_len;
        data = p->payload;

#if DHCPS_DEBUG
        Report("dhcps: handle_dhcp-> p->tot_len = %d\n\r", tlen);
        Report("dhcps: handle_dhcp-> p->len = %d\n\r", p->len);
#endif

        for(i=0; i<p->len; i++){
            p_dhcps_msg[dhcps_msg_cnt++] = data[i];
        }

        if(p->next != NULL) {
#if DHCPS_DEBUG
            Report("dhcps: handle_dhcp-> p->next != NULL\n\r");
            Report("dhcps: handle_dhcp-> p->next->tot_len = %d\n\r",p->next->tot_len);
            Report("dhcps: handle_dhcp-> p->next->len = %d\n\r",p->next->len);
#endif

            data = p->next->payload;
            for(i=0; i<p->next->len; i++){
                p_dhcps_msg[dhcps_msg_cnt++] = data[i];
            }
        }

        /*
         * DHCP
        */
#if DHCPS_DEBUG
        Report("dhcps: handle_dhcp-> parse_msg(p)\n\r");
#endif

        switch(parse_msg(pmsg_dhcps, tlen - 240)) {

            case DHCPS_STATE_OFFER://1
#if DHCPS_DEBUG
                 Report("dhcps: handle_dhcp-> DHCPD_STATE_OFFER\n\r");
#endif
                 send_offer(pmsg_dhcps, malloc_len);
                 break;
            case DHCPS_STATE_ACK://3
#if DHCPS_DEBUG
                 Report("dhcps: handle_dhcp-> DHCPD_STATE_ACK\n\r");
#endif
                 send_ack(pmsg_dhcps, malloc_len);
             //wifi_softap_set_station_info(pmsg_dhcps->chaddr, &client_address.addr);
                 break;
            case DHCPS_STATE_NAK://4
#if DHCPS_DEBUG
                 Report("dhcps: handle_dhcp-> DHCPD_STATE_NAK\n\r");
#endif
                 send_nak(pmsg_dhcps, malloc_len);
                 break;
            default :
                 break;
        }
#if DHCPS_DEBUG
        Report("dhcps: handle_dhcp-> pbuf_free(p)\n\r");
#endif
        pbuf_free(p);
        free(pmsg_dhcps);
        pmsg_dhcps = NULL;
}
///////////////////////////////////////////////////////////////////////////////////
static void   wifi_softap_init_dhcps_lease(uint32_t ip)
{
    uint32_t softap_ip = 0,local_ip = 0;
    uint32_t start_ip = 0;
    uint32_t end_ip = 0;
//  if (dhcps_lease_flag) {
    if (dhcps_lease.enable == TRUE) {
        softap_ip = htonl(ip);
        start_ip = htonl(dhcps_lease.start_ip.addr);
        end_ip = htonl(dhcps_lease.end_ip.addr);
        /*config ip information can't contain local ip*/
        if ((start_ip <= softap_ip) && (softap_ip <= end_ip)) {
            dhcps_lease.enable = FALSE;
        } else {
            /*config ip information must be in the same segment as the local ip*/
            softap_ip >>= 8;
            if (((start_ip >> 8 != softap_ip) || (end_ip >> 8 != softap_ip))
                    || (end_ip - start_ip > DHCPS_MAX_LEASE)) {
                dhcps_lease.enable = FALSE;
            }
        }
    }

    if (dhcps_lease.enable == FALSE) {
        local_ip = softap_ip = htonl(ip);
        softap_ip &= 0xFFFFFF00;
        local_ip &= 0xFF;
        if (local_ip >= 0x80)
            local_ip -= DHCPS_MAX_LEASE;
        else
            local_ip ++;

        os_memset(&dhcps_lease, 0,sizeof(dhcps_lease));
        dhcps_lease.start_ip.addr = softap_ip | local_ip;
        dhcps_lease.end_ip.addr = softap_ip | (local_ip + DHCPS_MAX_LEASE - 1);
        dhcps_lease.start_ip.addr = htonl(dhcps_lease.start_ip.addr);
        dhcps_lease.end_ip.addr= htonl(dhcps_lease.end_ip.addr);
    }
//  dhcps_lease.start_ip.addr = htonl(dhcps_lease.start_ip.addr);
//  dhcps_lease.end_ip.addr= htonl(dhcps_lease.end_ip.addr);
//  Report("start_ip = 0x%x, end_ip = 0x%x\n\r",dhcps_lease.start_ip, dhcps_lease.end_ip);
}
///////////////////////////////////////////////////////////////////////////////////
void dhcps_start(uint32_t addr, struct netif * apnetif)
{
/*    struct netif * apnetif = (struct netif *)eagle_lwip_getif(0x01);

    if(apnetif->dhcps_pcb != NULL) {
        udp_remove(apnetif->dhcps_pcb);
    }
    */

    pcb_dhcps = udp_new();
    if (pcb_dhcps == NULL)
    {
        Report("dhcps_start(): could not obtain pcb\n\r");
        return;
    }

    //apnetif->dhcps_pcb = pcb_dhcps;

    IP4_ADDR(&broadcast_dhcps, 255, 255, 255, 255);

    server_address.addr = addr;
    wifi_softap_init_dhcps_lease(server_address.addr);
    dhcps_lease.enable = TRUE;

    dhcps_set_ip_info(apnetif);

    udp_bind_netif(pcb_dhcps, apnetif);

    udp_bind(pcb_dhcps, IP_ADDR_ANY, DHCPS_SERVER_PORT);
    udp_recv(pcb_dhcps, handle_dhcp, NULL);
#if DHCPS_DEBUG
    Report("dhcps:dhcps_start->udp_recv function Set a receive callback handle_dhcp for UDP_PCB pcb_dhcps\n\r");
#endif

}

void   dhcps_stop(void)
{
    //struct netif * apnetif = (struct netif *)eagle_lwip_getif(0x01);

    udp_disconnect(pcb_dhcps);
//  dhcps_lease_flag = TRUE;
    if(pcb_dhcps != NULL)
    {
        udp_remove(pcb_dhcps);
        pcb_dhcps = NULL;
    }
    /*
    if(apnetif->dhcps_pcb != NULL) {
        udp_remove(apnetif->dhcps_pcb);
        apnetif->dhcps_pcb = NULL;
    }*/

    //udp_remove(pcb_dhcps);
    list_node *pnode = NULL;
    list_node *pback_node = NULL;

    struct ip_addr ip_zero;

    os_memset(&ip_zero,0x0,sizeof(ip_zero));
    pnode = plist;
    while (pnode != NULL) {
        pback_node = pnode;
        pnode = pback_node->pnext;
        node_remove_from_list(&plist, pback_node);
        /* Uncomment this in order to call client leave and set station info
            struct dhcps_pool* dhcp_node = NULL;
            dhcp_node = (struct dhcps_pool*)pback_node->pnode;
            wifi_softap_dhcps_client_leave(dhcp_node->mac,&dhcp_node->ip,TRUE); // force to delete
            wifi_softap_set_station_info(dhcp_node->mac, &ip_zero);
        */
        free(pback_node->pnode);
        pback_node->pnode = NULL;
        free(pback_node);
        pback_node = NULL;
    }

    dhcps_lease.enable = FALSE;
}

/******************************************************************************
 * FunctionName : wifi_softap_set_dhcps_lease
 * Description  : set the lease information of DHCP server
 * Parameters   : please -- Additional argument to set the lease information,
 *                          Little-Endian.
 * Returns      : TRUE or FALSE
*******************************************************************************/
BOOLEAN wifi_softap_set_dhcps_lease(struct dhcps_lease *please)
{
    struct ip_info info;
    uint32_t softap_ip = 0;
    uint32_t start_ip = 0;
    uint32_t end_ip = 0;
/*
    uint8_t opmode = wifi_get_opmode();

    if (opmode == STATION_MODE || opmode == NULL_MODE) {
        return FALSE;
    }

    if (please == NULL || wifi_softap_dhcps_status() == DHCP_STARTED)
        return FALSE;
*/
    if(please->enable) {
        os_memset(&info,0 ,sizeof(struct ip_info));
        dhcps_get_ip_info(&info);
        softap_ip = htonl(info.ip.addr);
        start_ip = htonl(please->start_ip.addr);
        end_ip = htonl(please->end_ip.addr);

        /*config ip information can't contain local ip*/
        if ((start_ip <= softap_ip) && (softap_ip <= end_ip))
            return FALSE;

        /*config ip information must be in the same segment as the local ip*/
        softap_ip >>= 8;
        if ((start_ip >> 8 != softap_ip)
                || (end_ip >> 8 != softap_ip)) {
            return FALSE;
        }

        if (end_ip - start_ip > DHCPS_MAX_LEASE)
            return FALSE;

        os_memset(&dhcps_lease, 0, sizeof(dhcps_lease));
//      dhcps_lease.start_ip.addr = start_ip;
//      dhcps_lease.end_ip.addr = end_ip;
        dhcps_lease.start_ip.addr = please->start_ip.addr;
        dhcps_lease.end_ip.addr = please->end_ip.addr;
    }
    dhcps_lease.enable = please->enable;
//  dhcps_lease_flag = FALSE;
    return TRUE;
}

/******************************************************************************
 * FunctionName : wifi_softap_get_dhcps_lease
 * Description  : get the lease information of DHCP server
 * Parameters   : please -- Additional argument to get the lease information,
 *                          Little-Endian.
 * Returns      : TRUE or FALSE
*******************************************************************************/
BOOLEAN wifi_softap_get_dhcps_lease(struct dhcps_lease *please)
{
    /*
    uint8_t opmode = wifi_get_opmode();

    if (opmode == STATION_MODE || opmode == NULL_MODE) {
        return FALSE;
    }
*/
    if (NULL == please)
        return FALSE;

//  if (dhcps_lease_flag){
    if (dhcps_lease.enable == FALSE){
        //if (wifi_softap_dhcps_status() == DHCP_STOPPED)
            return FALSE;
    } else {
//      os_bzero(please, sizeof(dhcps_lease));
//      if (wifi_softap_dhcps_status() == DHCP_STOPPED){
//          please->start_ip.addr = htonl(dhcps_lease.start_ip.addr);
//          please->end_ip.addr = htonl(dhcps_lease.end_ip.addr);
//      }
    }

//  if (wifi_softap_dhcps_status() == DHCP_STARTED){
//      os_bzero(please, sizeof(dhcps_lease));
//      please->start_ip.addr = dhcps_lease.start_ip.addr;
//      please->end_ip.addr = dhcps_lease.end_ip.addr;
//  }
    please->start_ip.addr = dhcps_lease.start_ip.addr;
    please->end_ip.addr = dhcps_lease.end_ip.addr;
    return TRUE;
}

static void   kill_oldest_dhcps_pool(void)
{
    list_node *pre = NULL, *p = NULL;
    list_node *minpre = NULL, *minp = NULL;
    struct dhcps_pool *pdhcps_pool = NULL, *pmin_pool = NULL;
    pre = plist;
    p = pre->pnext;
    minpre = pre;
    minp = p;
    while (p != NULL){
        pdhcps_pool = p->pnode;
        pmin_pool = minp->pnode;
        if (pdhcps_pool->lease_timer < pmin_pool->lease_timer){
            minp = p;
            minpre = pre;
        }
        pre = p;
        p = p->pnext;
    }
    minpre->pnext = minp->pnext;
    free(minp->pnode);
    minp->pnode = NULL;
    free(minp);
    minp = NULL;
}

void   dhcps_coarse_tmr(void)
{
    uint8_t num_dhcps_pool = 0;
    list_node *pback_node = NULL;
    list_node *pnode = NULL;
    struct dhcps_pool *pdhcps_pool = NULL;
    pnode = plist;
    while (pnode != NULL) {
        pdhcps_pool = pnode->pnode;
        if ( pdhcps_pool->type == DHCPS_TYPE_DYNAMIC) {
            pdhcps_pool->lease_timer --;
        }
        if (pdhcps_pool->lease_timer == 0){
            pback_node = pnode;
            pnode = pback_node->pnext;
            node_remove_from_list(&plist,pback_node);
            free(pback_node->pnode);
            pback_node->pnode = NULL;
            free(pback_node);
            pback_node = NULL;
        } else {
            pnode = pnode ->pnext;
            num_dhcps_pool ++;
        }
    }

    if (num_dhcps_pool >= MAX_STATION_NUM)
        kill_oldest_dhcps_pool();
}

BOOLEAN   wifi_softap_set_dhcps_offer_option(uint8_t level, void* optarg)
{
    BOOLEAN offer_flag = TRUE;

    if (optarg == NULL)// && wifi_softap_dhcps_status() == FALSE)
        return FALSE;

    if (level <= OFFER_START || level >= OFFER_END)
        return FALSE;

    switch (level){
        case OFFER_ROUTER:
            offer = (*(uint8_t *)optarg) & 0x01;
            offer_flag = TRUE;
            break;
        default :
            offer_flag = FALSE;
            break;
    }
    return offer_flag;
}

BOOLEAN wifi_softap_set_dhcps_lease_time(uint32_t minute)
{
    /*
    uint8_t opmode = wifi_get_opmode();

    if (opmode == STATION_MODE || opmode == NULL_MODE) {
        return FALSE;
    }

    if (wifi_softap_dhcps_status() == DHCP_STARTED) {
        return FALSE;
    }

    */
    if(minute == 0) {
        return FALSE;
    }
    dhcps_lease_time = minute;
    return TRUE;
}

BOOLEAN   wifi_softap_reset_dhcps_lease_time(void)
{
    /*
    uint8_t opmode = wifi_get_opmode();

    if (opmode == STATION_MODE || opmode == NULL_MODE) {
        return FALSE;
    }

    if (wifi_softap_dhcps_status() == DHCP_STARTED) {
        return FALSE;
    }
    */
    dhcps_lease_time = DHCPS_LEASE_TIME_DEF;
    return TRUE;
}

uint32_t   wifi_softap_get_dhcps_lease_time(void) // minute
{
    return dhcps_lease_time;
}

void   wifi_softap_dhcps_client_leave(uint8_t *bssid, struct ip_addr *ip,BOOLEAN force)
{
    struct dhcps_pool *pdhcps_pool = NULL;
    list_node *pback_node = NULL;

    if ((bssid == NULL) || (ip == NULL)) {
        return;
    }

    for (pback_node = plist; pback_node != NULL;pback_node = pback_node->pnext) {
        pdhcps_pool = pback_node->pnode;
        if (os_memcmp(pdhcps_pool->mac, bssid, sizeof(pdhcps_pool->mac)) == 0){
            if (os_memcmp(&pdhcps_pool->ip.addr, &ip->addr, sizeof(pdhcps_pool->ip.addr)) == 0) {
                if ((pdhcps_pool->type == DHCPS_TYPE_STATIC) || (force)) {
                    if(pback_node != NULL) {
                        node_remove_from_list(&plist,pback_node);
                        free(pback_node);
                        pback_node = NULL;
                    }

                    if (pdhcps_pool != NULL) {
                        free(pdhcps_pool);
                        pdhcps_pool = NULL;
                    }
                } else {
                    pdhcps_pool->state = DHCPS_STATE_OFFLINE;
                }

                struct ip_addr ip_zero;
                os_memset(&ip_zero,0x0,sizeof(ip_zero));
                //wifi_softap_set_station_info(bssid, &ip_zero);
                break;
            }
        }
    }
}

uint32_t   wifi_softap_dhcps_client_update(uint8_t *bssid, struct ip_addr *ip)
{
    struct dhcps_pool *pdhcps_pool = NULL;
    list_node *pback_node = NULL;
    list_node *pmac_node = NULL;
    list_node *pip_node = NULL;
    BOOLEAN flag = FALSE;
    uint32_t start_ip = dhcps_lease.start_ip.addr;
    uint32_t end_ip = dhcps_lease.end_ip.addr;
    dhcps_type_t type = DHCPS_TYPE_DYNAMIC;
    if (bssid == NULL) {
        return IPADDR_ANY;
    }

    if (ip) {
        if (IPADDR_BROADCAST == ip->addr) {
            return IPADDR_ANY;
        } else if (IPADDR_ANY == ip->addr) {
            ip = NULL;
        } else {
            type = DHCPS_TYPE_STATIC;
        }
    }

    renew = FALSE;
    for (pback_node = plist; pback_node != NULL;pback_node = pback_node->pnext) {
        pdhcps_pool = pback_node->pnode;
        //Report("mac:"MACSTR"bssid:"MACSTR"\r\n\r",MAC2STR(pdhcps_pool->mac),MAC2STR(bssid));
        if (os_memcmp(pdhcps_pool->mac, bssid, sizeof(pdhcps_pool->mac)) == 0){
            pmac_node = pback_node;
            if (ip == NULL) {
                flag = TRUE;
                break;
            }
        }
        if (ip != NULL) {
            if (os_memcmp(&pdhcps_pool->ip.addr, &ip->addr, sizeof(pdhcps_pool->ip.addr)) == 0) {
                pip_node = pback_node;
            }
        } else if (flag == FALSE){
            if (os_memcmp(&pdhcps_pool->ip.addr, &start_ip, sizeof(pdhcps_pool->ip.addr)) != 0) {
                flag = TRUE;
            } else {
                start_ip = htonl((ntohl(start_ip) + 1));
            }
        }
    }

    if ((ip == NULL) && (flag == FALSE)) {
        if (plist == NULL) {
            if (start_ip <= end_ip) {
                flag = TRUE;
            } else {
                return IPADDR_ANY;
            }
        } else {
            if (start_ip > end_ip) {
                return IPADDR_ANY;
            }
            //start_ip = htonl((ntohl(start_ip) + 1));
            flag = TRUE;
        }
    }

    if (pmac_node != NULL) { // update new ip
        if (pip_node != NULL){
            pdhcps_pool = pip_node->pnode;

            if (pip_node != pmac_node) {
                if(pdhcps_pool->state != DHCPS_STATE_OFFLINE) { // ip is used
                    return IPADDR_ANY;
                }

                // mac exists and ip exists in other node,delete mac
                node_remove_from_list(&plist,pmac_node);
                free(pmac_node->pnode);
                pmac_node->pnode = NULL;
                free(pmac_node);
                pmac_node = pip_node;
                os_memcpy(pdhcps_pool->mac, bssid, sizeof(pdhcps_pool->mac));
            } else {
                renew = TRUE;
                type = DHCPS_TYPE_DYNAMIC;
            }

            pdhcps_pool->lease_timer = DHCPS_LEASE_TIMER;
            pdhcps_pool->type = type;
            pdhcps_pool->state = DHCPS_STATE_ONLINE;

        } else {
            pdhcps_pool = pmac_node->pnode;
            if (ip != NULL) {
                pdhcps_pool->ip.addr = ip->addr;
            } else if (flag == TRUE) {
                pdhcps_pool->ip.addr = start_ip;
            } else {    // no ip to distribute
                return IPADDR_ANY;
            }

            node_remove_from_list(&plist,pmac_node);
            pdhcps_pool->lease_timer = DHCPS_LEASE_TIMER;
            pdhcps_pool->type = type;
            pdhcps_pool->state = DHCPS_STATE_ONLINE;
            node_insert_to_list(&plist,pmac_node);
        }
    } else { // new station
        if (pip_node != NULL) { // maybe ip has used
            pdhcps_pool = pip_node->pnode;
            if (pdhcps_pool->state != DHCPS_STATE_OFFLINE) {
                return IPADDR_ANY;
            }
            os_memcpy(pdhcps_pool->mac, bssid, sizeof(pdhcps_pool->mac));
            pdhcps_pool->lease_timer = DHCPS_LEASE_TIMER;
            pdhcps_pool->type = type;
            pdhcps_pool->state = DHCPS_STATE_ONLINE;
        } else {
            pdhcps_pool = (struct dhcps_pool *)calloc(1, sizeof(struct dhcps_pool));
            if (ip != NULL) {
                pdhcps_pool->ip.addr = ip->addr;
            } else if (flag == TRUE) {
                pdhcps_pool->ip.addr = start_ip;
            } else {    // no ip to distribute
                free(pdhcps_pool);
                return IPADDR_ANY;
            }
            if (pdhcps_pool->ip.addr > end_ip) {
                free(pdhcps_pool);
                return IPADDR_ANY;
            }
            os_memcpy(pdhcps_pool->mac, bssid, sizeof(pdhcps_pool->mac));
            pdhcps_pool->lease_timer = DHCPS_LEASE_TIMER;
            pdhcps_pool->type = type;
            pdhcps_pool->state = DHCPS_STATE_ONLINE;
            pback_node = (list_node *)calloc(1, sizeof(list_node ));
            pback_node->pnode = pdhcps_pool;
            pback_node->pnext = NULL;
            node_insert_to_list(&plist,pback_node);
        }
    }

    return pdhcps_pool->ip.addr;
}
