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
#include <string.h>
#include <stdio.h>
#include "lwip/sockets.h"
#include "cmd_parser.h"
#include "uart_term.h"
#include "errors.h"
#include "FreeRTOSConfig.h"
#include "osi_kernel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "network_lwip.h"


#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/tcpbase.h"
#include "lwip/ip6_addr.h"
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif
#include "lwip_iperf_examples.h"




#define IPERF_LWIP_CLIENT_DURATION_MS 10000 // Test duration (10 seconds)
#define SEND_BUFFER_SIZE_UDP_CLIENT      (1470)
#define SEND_BUFFER_SIZE_UDP_CLIENT_IPV6 (1452) /* 1500 MTU - 40 IPv6 - 8 UDP */

#define IPERF_LWIP_MAX_FORMAT_RATE_LENGTH  20

extern session_conn_t iperf_session[];

// Forward declarations
static void iperflwip_client_udp_init(void *param);
void  iperflwip_udp_client_tx(void* arg);
int32_t iperflwip_udp_client_start(void* args);
static void iperf_report_timer_cb(TimerHandle_t t);
static void iperflwip_report(void *arg);
void iperflwip_udp_client_close(session_conn_t* session_con);

extern void format_bps(double bps, char *output, size_t size);

extern unsigned char send_buffer[];
static void iperflwip_send_udp_client_iperf_fin(session_conn_t* session_con);



static void iperflwip_client_udp_init(void *param)
{
    session_conn_t* session_con = param;
    session_con->actualTestdurationMs = 0;
    session_con->actualNumOfDurations = 0;
    session_con->conn_pcb_udp = NULL;
    session_con->conn_pcb_tcp = NULL;
    session_con->report_task_handle = NULL;
    session_con->total_bytes = 0;
    session_con->bytes_per_period = 0;
    session_con->poll_count = 0;
    session_con->target_Bps = (session_con->lwipConfig.bandwidth * 1000* 1000)/8;

    if (session_con->lwipConfig.ipv6) {
        memcpy(&session_con->dest_ip.u_addr.ip6, session_con->lwipConfig.ipAddr.ipv6, 16);
        session_con->dest_ip.type = IPADDR_TYPE_V6;
        ip6_addr_assign_zone(&session_con->dest_ip.u_addr.ip6, IP6_UNICAST,
                             network_netif_find_by_ip6((const uint8_t *)session_con->dest_ip.u_addr.ip6.addr));
        session_con->conn_pcb_udp = udp_new_ip_type(IPADDR_TYPE_V6);
    } else {
        session_con->dest_ip.u_addr.ip4.addr = htonl((unsigned int )session_con->lwipConfig.ipAddr.ipv4);
        session_con->conn_pcb_udp = udp_new_ip_type(IPADDR_TYPE_V4);
    }
    session_con->dest_port = session_con->lwipConfig.destOrLocalPortNumber;
    if (session_con->conn_pcb_udp == NULL) {
        Report("\n\riperflwip_client: ERROR ! Failed to create pcb (free heap: %u bytes)\n", (unsigned)osi_GetFreeHeapSize());
        return;
    }

    /*err = udp_bind(session_con->conn_pcb_udp, IP4_ADDR_ANY4, LOCAL_UDP_CLIENT_PORT);
    if(err != ERR_OK){
        Report("\n\riperflwip_client: ERROR ! tcp_bind, port is in use\n");
        udp_remove(session_con->conn_pcb_udp);
        session_con->conn_pcb_udp = NULL;
        return;
    }


    udp_recv(session_con->conn_pcb_udp, lwiperf_udp_client_recv, (void*)session_con);
   */

    session_con->total_bytes = 0;
    session_con->is_running = true;

    if ( session_con->lwipConfig.period)
    {
        session_con->actualTestdurationMs = session_con->lwipConfig.period * 1000;
    }
    else if(session_con->lwipConfig.timeout >= (uint32_t)99999)
    {
        session_con->actualTestdurationMs = IPERF_LWIP_CLIENT_DURATION_MS * 1000;
    }
    else
    {
        session_con->actualTestdurationMs = session_con->lwipConfig.timeout*1000;//sec to ms
    }

    if (session_con->lwipConfig.period > 0)
    {
        session_con->report_task_handle = xTimerCreate("udp_report",
            pdMS_TO_TICKS(session_con->actualTestdurationMs),
            pdTRUE, session_con, iperf_report_timer_cb);
        if (session_con->report_task_handle)
            xTimerStart(session_con->report_task_handle, 0);
    }

    /* Compute and cache packet length */
    uint32_t max_packet = session_con->lwipConfig.ipv6 ? SEND_BUFFER_SIZE_UDP_CLIENT_IPV6 : SEND_BUFFER_SIZE_UDP_CLIENT;
    if (session_con->lwipConfig.packetLength > 0) {
        session_con->udp_pkt_len = session_con->lwipConfig.packetLength;
        if (session_con->udp_pkt_len > max_packet)
            session_con->udp_pkt_len = max_packet;
    } else {
        session_con->udp_pkt_len = max_packet;
    }

    session_con->total_bytes             = 0;
    session_con->bytes_per_period        = 0;
    session_con->udp_bytes_in_window     = 0;
    session_con->udp_throughput_timer    = osi_GetTimeMS();
    session_con->udp_heap_check_counter  = 0;
    session_con->udp_heap_ok             = 1;
    session_con->previous_time           = osi_GetTimeMS();
    session_con->start_time              = osi_GetTimeMS();

    tcpip_callback(iperflwip_udp_client_tx, session_con);
}



void udp_client_stop(session_conn_t* session_con)
{
    /* TI: Request abort - report task will see this and exit */
    session_con->is_req_to_abort_test = 1;
    tcpip_callback(session_con->iperf_reportFunc,session_con);
}


void iperflwip_udp_client_close(session_conn_t* session_con)
{
    session_con->is_running = false;
    sys_untimeout(iperflwip_udp_client_tx, session_con);

    if (session_con->report_task_handle != NULL)
    {
        xTimerStop(session_con->report_task_handle, 0);
        xTimerDelete(session_con->report_task_handle, 0);
        session_con->report_task_handle = NULL;
    }

    if(session_con->conn_pcb_udp != NULL)
    {
        udp_remove(session_con->conn_pcb_udp);
        session_con->conn_pcb_udp = NULL;
    }

}

/*
static void  lwiperf_udp_client_recv(void *arg, struct udp_pcb *pcb,
        struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    return;
}
*/


/* UDP TX callback - runs entirely in tcpip_thread, re-queues itself.
 * Eliminates the separate FreeRTOS task and its per-packet context switches. */
void iperflwip_udp_client_tx(void *arg)
{
    session_conn_t *session_con = arg;
    struct pbuf *p;
    uint32_t len;
    uint32_t time_mili_sec, sec, usec, count;

    if (!session_con->is_running)
        return;

    /* Periodic heap check */
    if (session_con->udp_heap_check_counter++ >= 16) {
        session_con->udp_heap_ok = (osi_GetFreeHeapSize() > HEAP_THRESHOLD_FOR_TX);
        session_con->udp_heap_check_counter = 0;
    }
    if (!session_con->udp_heap_ok) {
        sys_timeout(session_con->lwipConfig.ipv6 ? 2 : 1,
                    iperflwip_udp_client_tx, session_con);
        return;
    }

    /* Rate-limited path */
    if (session_con->target_Bps > 0) {
        uint32_t now  = osi_GetTimeMS();
        uint32_t bw100 = (uint32_t)(session_con->target_Bps / 10);
        if ((now - session_con->udp_throughput_timer) > 100) {
            session_con->udp_bytes_in_window  = 0;
            session_con->udp_throughput_timer = now;
        }
        if (session_con->udp_bytes_in_window >= bw100) {
            uint32_t sleep_ms = 100 - (now - session_con->udp_throughput_timer);
            if (sleep_ms == 0) sleep_ms = 1;
            sys_timeout(sleep_ms, iperflwip_udp_client_tx, session_con);
            return;
        }
        len = (uint32_t)MIN(bw100 - (uint32_t)session_con->udp_bytes_in_window,
                            session_con->udp_pkt_len);
    } else {
        len = session_con->udp_pkt_len;
    }

    p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
    if (!p) {
        sys_timeout(1, iperflwip_udp_client_tx, session_con);
        return;
    }

    os_memcpy(p->payload, send_buffer, len);

    time_mili_sec = osi_GetTimeMS();
    sec   = htonl(time_mili_sec / 1000);
    usec  = htonl((time_mili_sec % 1000) * 1000);
    count = htonl(session_con->packet_count);

    /* Write header directly into pbuf to avoid race between concurrent sessions */
    os_memcpy(p->payload,            &count, sizeof(count));
    os_memcpy((uint8_t*)p->payload + 4, &sec,   sizeof(sec));
    os_memcpy((uint8_t*)p->payload + 8, &usec,  sizeof(usec));

    err_t err = udp_sendto(session_con->conn_pcb_udp, p,
                           &session_con->dest_ip, (uint16_t)session_con->dest_port);
    if (err == ERR_OK) {
        /* nd6 holds the pbuf (ref>1) while the neighbor is unresolved */
        if (p->ref > 1) {
            session_con->neighbor_unresolved = 1;
        } else {
            session_con->neighbor_unresolved = 0;
            session_con->packet_count++;
            session_con->total_bytes     += len;
            session_con->bytes_per_period += len;
            if (session_con->target_Bps > 0)
                session_con->udp_bytes_in_window += len;
        }
    }

    pbuf_free(p);

    /* If nd6/ARP still holds the pbuf, back off 5ms so tcpip_thread can process
     * the incoming NA/ARP reply. The next invocation retries the send; a successful
     * send with p->ref==1 clears the flag and resumes full-rate re-queuing. */
    if (session_con->neighbor_unresolved) {
        sys_timeout(5, iperflwip_udp_client_tx, session_con);
        return;
    }

    /* Re-queue: pace via sys_timeout for rate-limited mode, immediate re-queue otherwise.
     * Posting to the back of the tcpip_thread mailbox gives other pending events
     * (ACKs, received data, timers) priority - no starvation of concurrent streams. */
    if (session_con->target_Bps > 0) {
        uint32_t ticks = (uint32_t)((uint64_t)session_con->udp_pkt_len * 1000u
                                    / session_con->target_Bps);
        if (ticks > 0) {
            sys_timeout(ticks, iperflwip_udp_client_tx, session_con);
            return;
        }
    }

    /* Yield 1ms every 500 packets so lower-priority tasks (TLS, UART input) get CPU.
     * At ~24k packets/sec this yields every ~20ms with ~5% throughput cost. */
    if ((session_con->packet_count % 500) == 0) {
        sys_timeout(1, iperflwip_udp_client_tx, session_con);
        return;
    }
    tcpip_callback(iperflwip_udp_client_tx, session_con);
}

static void iperflwip_send_udp_client_iperf_fin(session_conn_t* session_con)
{
    struct iperf_udp_hdr fin_pkt;
    fin_pkt.id     = htonl(-1);   // -1 indicates FIN
    fin_pkt.tv_sec = 0;
    fin_pkt.tv_usec = 0;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(fin_pkt), PBUF_RAM);
    if (!p)
    {
        Report("\n\riperflwip_send_udp_client_iperf_fin: Failed to allocate pbuf (free heap: %u bytes)\n", (unsigned)osi_GetFreeHeapSize());
        return;
    }
    
    os_memcpy(p->payload, &fin_pkt, sizeof(fin_pkt));

    udp_sendto(session_con->conn_pcb_udp, p, &session_con->dest_ip, (uint16_t)session_con->dest_port);

    pbuf_free(p);
}

static void iperf_report_timer_cb(TimerHandle_t t)
{
    tcpip_callback(iperflwip_report, pvTimerGetTimerID(t));
}

// Timer expired -> test done
static void iperflwip_report(void* arg)
{
    char ratestr[IPERF_LWIP_MAX_FORMAT_RATE_LENGTH];
    session_conn_t* session_con = (session_conn_t*)arg;
    double secondsFromStart, durationInSecond;
    uint32_t current_time;
    double bps = 0.0;

    char tag[24];
    snprintf(tag, sizeof(tag), "[UDP:%u] [%d]",
             (unsigned)session_con->lwipConfig.destOrLocalPortNumber,
             session_con->process_num);

    if (session_con->is_running && session_con->conn_pcb_udp != NULL)
    {

        session_con->actualNumOfDurations++;

        if(session_con->lwipConfig.period)
        {
            current_time = osi_GetTimeMS();
            uint32_t delta_ms = current_time - session_con->previous_time;
            durationInSecond = (double)delta_ms / 1000.0;
            if(durationInSecond){
                bps = ((double)session_con->bytes_per_period * 8.0) / durationInSecond;
            }

            format_bps(bps,ratestr, sizeof(ratestr));
            Report("\n\r%s %s", tag, ratestr);
        }

        session_con->previous_time = osi_GetTimeMS();
        session_con->bytes_per_period = 0;

        if (!session_con->is_req_to_abort_test && ((session_con->lwipConfig.timeout >= 99999) ||
                (session_con->lwipConfig.timeout*1000 > (session_con->actualTestdurationMs* session_con->actualNumOfDurations))))
        {
            /* timer is auto-reload - nothing to do */
        }
        else
        {
            uint32_t  curr_time = osi_GetTimeMS();
            secondsFromStart = ((double)(curr_time - session_con->start_time));
            if(secondsFromStart > 0)
            {
                secondsFromStart=secondsFromStart/1000.0;
                bps = ((session_con->total_bytes * 8.0)/secondsFromStart);
                format_bps(bps,ratestr, sizeof(ratestr));
            }
            else
            {
                bps = 0;
                snprintf(ratestr, sizeof(ratestr), "0 bps");
            }
            Report("\n\r%s Test complete  %lu bytes  %.2f sec  %s",
                   tag, (unsigned long)session_con->total_bytes, secondsFromStart, ratestr);
            Report("\n\riperf UDP %s :  %lu total bytes duration :%lu sec  %.2f Mbps",
                   session_con->is_server ? "server" : "client",
                   (unsigned long)session_con->total_bytes,
                   (unsigned long)secondsFromStart,
                   bps / 1e6);
            iperflwip_send_udp_client_iperf_fin(session_con);
            iperflwip_udp_client_close(session_con);
        }
    }
    else
    {
        Report("\n\riperflwip: UDP client Test finished\n");
    }
}



int32_t iperflwip_udp_client_start(void* args)
{
    int i;
    uint8_t found = FALSE;

    //find available iperf TCP server
    for(i=1; i< IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS+1; i++)
    {
        if(!iperf_session[i].is_running)
        {
            found = TRUE;//found not running process
            os_memset(&iperf_session[i], 0, sizeof(iperf_session[i]));
            iperf_session[i].is_server = 0;
            iperf_session[i].proto = IPERF_PROTO_UDP;
            iperf_session[i].process_num = i;
            iperf_session[i].iperf_reportFunc = iperflwip_report;
            iperf_session[i].is_req_to_abort_test = 0;
            iperf_session[i].is_stop_due_traffic_error = 0;
            iperf_session[i].packet_count = 0;
            iperf_session[i].is_running = true;

            break;
        }
    }

    if(found)
    {
        Report("\r\nTo stop the UDP process , iperf_stop -n %d ", i);
        os_memcpy((void *)&iperf_session[i].lwipConfig,args, sizeof(RecvCmd_t));
        tcpip_callback(iperflwip_client_udp_init,(void *)&iperf_session[i]);
        os_sleep(0,50);//give time to the iperflwip_server_tcp_init to be trigger
        return 0;
    }
    else
    {
        Report("\n\riperflwip_client: Num of sessions exceeded, max num = %d ",IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS);
        return -1;
    }

}
