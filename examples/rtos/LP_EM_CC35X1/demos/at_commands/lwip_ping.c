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

#ifdef CC35XX

#include <string.h>
#include <stdio.h>
#include "lwip/sockets.h"
#include "uart_term.h"
#include "errors.h"
#include "FreeRTOSConfig.h"
#include "osi_kernel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmd_parser.h"

#include "lwip_ping.h"
#include "network_lwip.h"

#include "lwip/icmp.h"
#include "lwip/ip.h"
#include "lwip/ip4.h"
#include "lwip/raw.h"
#include "lwip/inet_chksum.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"

#define LWIP_PING_MAX_NUM_OF_SESSIONS   2

#define LWIP_PING_THREAD_PRIORITY       (tskIDLE_PRIORITY + 1)
#define LWIP_PING_THREAD_STACK_SIZE     512
#define LWIP_PING_THREAD_NAME           "LWIP_PING"

typedef struct {
    struct icmp_echo_hdr hdr;
} ping_msg_t;

typedef struct
{
    /* Session state */
    uint8_t is_running;
    /* User-defined ping parameters */
    PingParams_t params;
    /* Ping packets sent for this session */
    uint32_t packets_sent;
    /* Ping packets received for this session */
    uint32_t packets_received;
    /* IP in binary format (network representation) */
    ip_addr_t target_ip;
    /* Unique ICMP ID */
    uint16_t icmp_id;
    /* ICMP sequence number */
    uint16_t seqno;
    /* Ping session ID
     * Session ID is 1 based, and that's how it`s displayed to
     * the user.
     */
    uint16_t session_id;
    /* Session specific lwip raw pcb */
    struct raw_pcb *ping_pcb;
} lwip_ping_session_t;


lwip_ping_session_t lwip_ping_sessions[LWIP_PING_MAX_NUM_OF_SESSIONS] = {0};
static uint16_t ping_next_id = 0x1000;


void ping_session_task(void *arg);
uint8_t ping_recv(void *arg, struct raw_pcb *pcb, struct pbuf *p,
                  const ip_addr_t *addr);
void lwip_ping_deinit(void *args);
int8_t is_ip_addr_in_net_list(const ip_addr_t *addr);



static uint16_t ping_assign_cmp_id(void)
{
    if (ping_next_id == 0xFFFF)
    {
        ping_next_id = 0x1000;
    }
    return ping_next_id++;
}


static void ping_prepare(ping_msg_t *ping_msg, lwip_ping_session_t *ping_session)
{
    uint16_t i = 0;
    uint8_t *payload_ptr = NULL;

    ICMPH_TYPE_SET(&ping_msg->hdr, ICMP_ECHO);
    ICMPH_CODE_SET(&ping_msg->hdr, 0);
    ping_msg->hdr.chksum = 0;
    ping_msg->hdr.id = lwip_htons(ping_session->icmp_id);
    ping_msg->hdr.seqno = lwip_htons(ping_session->seqno);

    payload_ptr = (uint8_t *)(ping_msg + 1);
    for (i = 0; i < ping_session->params.payload_size; i++)
    {
        payload_ptr[i] = (uint8_t)(i % (UINT8_MAX + 1));
    }

    ping_msg->hdr.chksum = inet_chksum(ping_msg, sizeof(*ping_msg) + ping_session->params.payload_size);
}

/* session specific init routine */
static void lwip_ping_init(void *param)
{
    err_t err;
    BaseType_t xReturned;
    ip_addr_t source_ip;
    lwip_ping_session_t *ping_session = (lwip_ping_session_t *)param;

    ping_session->target_ip.addr = htonl((unsigned int )ping_session->params.target_ip.ipv4);
    ping_session->seqno = 0;
    ping_session->icmp_id = ping_assign_cmp_id();

    /* Init LWIP raw pcb */
    ping_session->ping_pcb = raw_new(IP_PROTO_ICMP);
    if (ping_session->ping_pcb == NULL)
    {
        ping_session->is_running = FALSE;
        Report("\n\rlwip_ping: ERROR! Failed to create ping pcb\n\r");
        return;
    }

    source_ip.addr = htonl((unsigned int )ping_session->params.source_ip.ipv4);
    if ((source_ip.addr != 0) && (is_ip_addr_in_net_list(&source_ip) != 0))
    {
        raw_remove(ping_session->ping_pcb);
        ping_session->is_running = FALSE;
        Report("\n\rlwip_ping: ERROR! Source IP address is not in netlist.\n\r");
        return;
    }
    
    err = raw_bind(ping_session->ping_pcb, &source_ip);
    if (err != ERR_OK)
    {
        raw_remove(ping_session->ping_pcb);
        ping_session->is_running = FALSE;
        Report("\n\rlwip_ping: ERROR! Failed to bind raw pcb\n\r");
        return;
    }

    raw_recv(ping_session->ping_pcb, ping_recv, ping_session);

    xReturned = xTaskCreate(ping_session_task,
                            LWIP_PING_THREAD_NAME,
                            LWIP_PING_THREAD_STACK_SIZE,
                            ping_session,
                            LWIP_PING_THREAD_PRIORITY,
                            NULL);
    
    if (xReturned != pdPASS)
    {
        raw_remove(ping_session->ping_pcb);
        ping_session->is_running = FALSE;
        Report("\n\rlwip_ping: ERROR! Failed to create ping thread\n\r");
    }
}

uint8_t ping_recv(void *arg, struct raw_pcb *pcb, struct pbuf *p, const ip_addr_t *addr)
{
    lwip_ping_session_t *ping_session = (lwip_ping_session_t *)arg;
    struct ip_hdr *iph = (struct ip_hdr *)p->payload;
    uint16_t iphdr_len = IPH_HL(iph) * 4; // header length in bytes
    struct icmp_echo_hdr *iecho = (struct icmp_echo_hdr *)((uint8_t *)p->payload + iphdr_len);

    if (p->tot_len >= sizeof(struct icmp_echo_hdr) + sizeof(struct ip_hdr))
    {
        if ((iecho->id == lwip_htons(ping_session->icmp_id)) &&
            (ICMPH_TYPE(iecho) == ICMP_ER))
        {
            if (ping_session->params.flags & PING_PRINT_RESPONSES)
            {
                Report("\n\rping_session_id=%u: %u bytes from %s: icmp_id=%u icmp_seq=%u\n\r",
                       ping_session->session_id,
                       ping_session->params.payload_size + sizeof(struct icmp_echo_hdr),
                       ipaddr_ntoa(addr),
                       lwip_ntohs(iecho->id),
                       lwip_ntohs(iecho->seqno));
            }
            
            ping_session->packets_received++;

            pbuf_free(p);

            /* Packet eaten */
            return 1;
        }
    }

    /* Packet not eaten, no need to call pbuf_free() */
    return 0;
}

/* Sent from LWIP thread */
void lwip_ping_send(void *ctx)
{
    struct pbuf *p = NULL;
    lwip_ping_session_t *ping_session = (lwip_ping_session_t *)ctx;
    err_t err;

    /* Allocate according to payload size */
    p = pbuf_alloc(PBUF_IP,
                   sizeof(struct icmp_echo_hdr) + ping_session->params.payload_size,
                   PBUF_RAM);
    if (!p)
    {
        return;
    }

    if ((p->len == p->tot_len) && (p->next == NULL))
    {
        ping_msg_t *ping_msg = (ping_msg_t *)p->payload;

        ping_prepare(ping_msg, ping_session);

        err = raw_sendto(ping_session->ping_pcb, p, &ping_session->target_ip);
        if (err != ERR_OK)
        {
            pbuf_free(p);
            return;
        }
        ping_session->packets_sent++;
        ping_session->seqno++;

        // Report("\n\rPing request sent to %s: id=%u seq=%u\n\r",
        //        ipaddr_ntoa(&ping_session->target_ip),
        //        ping_session->id,
        //        ping_session->seqno - 1);
    }

    pbuf_free(p);
}

static void ping_send(lwip_ping_session_t *ping_session)
{
    err_t err;

    err = tcpip_callback(lwip_ping_send, (void *)ping_session);
    if (err != ERR_OK)
    {
        Report("\n\rlwip_ping: ERROR ! Failed to send ping\n\r");
    }
}

void ping_session_task(void *arg)
{
    lwip_ping_session_t *session = (lwip_ping_session_t *)arg;
    char *target_ip_str;

    target_ip_str = inet_ntoa(session->target_ip);

    Report("\n\rPING %s %d bytes of data.\n\r",
           target_ip_str,
           session->params.payload_size);
    
    /* Start sending until session is stopped or if count is not 0 (infinite) and is reached */
    while ((session->is_running) &&
            (((session->params.count > 0) && (session->packets_sent < session->params.count)) ||
             (session->params.count == 0)))
    {
        ping_send(session);

        os_sleep(session->params.interval_time_ms / 1000,
                (session->params.interval_time_ms % 1000) * 1000);
    }

    /* Clean the session running flag in case count is reached.
     * Note: This is not required if count is 0 (infinite).
     */
    if ((session->params.count > 0) &&
        (session->packets_sent == session->params.count))
    {
        session->is_running = FALSE;
    }

    tcpip_callback(lwip_ping_deinit, (void *)session);

    vTaskDelete(NULL);
}

int32_t lwip_ping_start(PingParams_t *ping_params)
{
    int i;
    uint8_t found = FALSE;

    /* Find available slot for ping session */
    for (i = 0; i < LWIP_PING_MAX_NUM_OF_SESSIONS; i++)
    {
        if (!lwip_ping_sessions[i].is_running)
        {
            found = TRUE; /* Found available slot */

            os_memset(&lwip_ping_sessions[i],
                      0, sizeof(lwip_ping_sessions[i]));

            os_memcpy(&lwip_ping_sessions[i].params,
                      ping_params, sizeof(PingParams_t));

            lwip_ping_sessions[i].is_running = TRUE;
            lwip_ping_sessions[i].session_id = i + 1;
            break;
        }
    }

    if (found)
    {
        tcpip_callback(lwip_ping_init, (void *)&lwip_ping_sessions[i]);
        /* Allow lwip_ping_init to be triggered safely */
        os_sleep(0, 50);
        return lwip_ping_sessions[i].session_id;
    }
    else
    {
        Report("\n\rlwip_ping: Num of sessions exceeded, max num = %d\n\r",
               LWIP_PING_MAX_NUM_OF_SESSIONS);
        return -1;
    }
}

void lwip_ping_deinit(void *args)
{
    lwip_ping_session_t *ping_session = (lwip_ping_session_t *)args;

    raw_remove(ping_session->ping_pcb);

    if (ping_session->params.flags & PING_PRINT_REPORT)
    {
        Report("\n\r--- %s ping statistics (ping session id: %d) ---\n\r"
               "%u packets transmitted, %u received, %u%% packet loss\n\r",
               ipaddr_ntoa(&ping_session->target_ip),
               ping_session->session_id,
               ping_session->packets_sent,
               ping_session->packets_received,
               ping_session->packets_sent > 0 ?
               (ping_session->packets_sent - ping_session->packets_received) * 100 / ping_session->packets_sent : 0);
    }

    if (ping_session->params.ping_deinit_callback)
    {
        ping_results_report_t results;

        results.target_ip = ping_session->target_ip;
        results.session_id = ping_session->session_id;
        results.packets_sent = ping_session->packets_sent;
        results.packets_received = ping_session->packets_received;

        ping_session->params.ping_deinit_callback((void *)&results);
    }
}

int32_t lwip_ping_stop(int8_t session_id)
{
    /* Validate session number
     * Note: session id is 1 based for this module.
     *       Callers should use 1 based session ids.
     */
    if (session_id < 0 || 
       (session_id > LWIP_PING_MAX_NUM_OF_SESSIONS + 1))
    {
        return -1;
    }

    if (session_id == 0)
    {
        // Print information regarding which sessions are active
        for (uint8_t i = 0; i < LWIP_PING_MAX_NUM_OF_SESSIONS; i++)
        {
            if (lwip_ping_sessions[i].is_running)
            {
                Report("\n\rPing session %d is running\n\r",
                       lwip_ping_sessions[i].session_id);
            }
        }
        return 0;
    }
    else
    {
        if (lwip_ping_sessions[session_id - 1].is_running)
        {
            lwip_ping_sessions[session_id - 1].is_running = FALSE;
            return 0;
        }
        else
        {
            Report("\n\rlwip_ping: Ping session %d is not running\n\r", session_id);
            return -1;
        }
    }
}

#endif // CC35XX
