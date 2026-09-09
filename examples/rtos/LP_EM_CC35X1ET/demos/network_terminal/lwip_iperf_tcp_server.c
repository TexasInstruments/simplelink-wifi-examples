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
#include "FreeRTOS.h"
#include "task.h"
#include "cmd_parser.h"
#include "uart_term.h"
#include "errors.h"
#include "FreeRTOSConfig.h"
#include "osi_kernel.h"
#include "timers.h"
#include "network_lwip.h"

#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/ip6_addr.h"
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif
#include "lwip_iperf_examples.h"



#define IPERF_TEST_DURATION 10000 // milliseconds (10 sec)
#define IPERF_TEST_PRINT_DURATION 1000 // milliseconds (1 sec)
#define IPERF_MAX_FORMAT_RATE_LENGTH  20


extern session_conn_t iperf_session[];

// Forward declarations
static err_t iperflwip_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t iperflwip_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t iperflwip_tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void iperf_report_timer_cb(TimerHandle_t t);
static void iperflwip_report(void *arg);

extern void format_bps(double bps, char *output, size_t size);
extern void iperflwip_tcp_stop(void *arg, uint8_t isError);
extern void iperflwip_tcp_err(void *arg, err_t err);
extern err_t lwiperf_tcp_poll(void *arg, struct tcp_pcb *tpcb);
extern err_t iperflwip_tcp_client_tx(session_conn_t* session_con);


// Initialize iperf server
void iperflwip_server_tcp_init(void* args)
{

    session_conn_t* session_con = args;
    session_con->actualTestdurationMs = 0;
    session_con->actualNumOfDurations = 0;
    session_con->conn_pcb_tcp = NULL;
    session_con->report_task_handle = NULL;
    session_con->total_bytes = 0;
    session_con->bytes_per_period = 0;

    session_con->src_port = session_con->lwipConfig.destOrLocalPortNumber;

    if (session_con->lwipConfig.ipv6) {
        memcpy(&session_con->src_ip.u_addr.ip6, session_con->lwipConfig.ipAddr.ipv6, 16);
        session_con->src_ip.type = IPADDR_TYPE_V6;
        if (!ip6_addr_isany(&session_con->src_ip.u_addr.ip6)) {
            /* Find which netif owns this bind address so the zone is correct.
             * Required when binding to an AP link-local address - using the STA
             * netif's zone would cause lwIP to silently drop incoming connections. */
            struct netif *bind_netif = NULL;
            struct netif *n;
            int idx;
            for (n = netif_list; n != NULL; n = n->next) {
                for (idx = 0; idx < LWIP_IPV6_NUM_ADDRESSES; idx++) {
                    if (memcmp(n->ip6_addr[idx].u_addr.ip6.addr,
                               session_con->src_ip.u_addr.ip6.addr, 16) == 0) {
                        bind_netif = n;
                        break;
                    }
                }
                if (bind_netif) break;
            }
            if (bind_netif == NULL) {
                bind_netif = (struct netif *)network_get_sta_if();
            }
            ip6_addr_assign_zone(&session_con->src_ip.u_addr.ip6, IP6_UNICAST, bind_netif);
        }
        session_con->server_pcb = tcp_new_ip_type(IPADDR_TYPE_V6);
    } else {
        memset(&session_con->src_ip, 0, sizeof(session_con->src_ip)); /* ensure type=IPADDR_TYPE_V4 */
        session_con->src_ip.u_addr.ip4.addr = htonl((unsigned int )session_con->lwipConfig.ipAddr.ipv4);
        if ((session_con->src_ip.u_addr.ip4.addr != 0) && (is_ip_addr_in_net_list(&session_con->src_ip) != 0))
        {
            Report("\n\riperflwip_server: ERROR: ERROR! Source IP address is not in netlist.\n\r");
            return;
        }
        session_con->server_pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    }
    if (session_con->server_pcb == NULL) {
        Report("\n\riperflwip_server: ERROR ! Failed to create pcb (free heap: %u bytes)\n", (unsigned)osi_GetFreeHeapSize());
        return;
    }

    //local port and local IP address
    if(tcp_bind(session_con->server_pcb, &session_con->src_ip, session_con->src_port)!= ERR_OK)
    {
        Report("\n\riperflwip_server: Failed to bind PCB, port may be in use\n");
        tcp_close(session_con->server_pcb);
        session_con->server_pcb = NULL;
        return;
    }

    tcp_arg(session_con->server_pcb, session_con);

    session_con->server_pcb = tcp_listen(session_con->server_pcb);
    if (session_con->server_pcb == NULL)
    {
        Report("\n\riperflwip_server: Failed to listen (out of memory)\n");
        return;
    }

    tcp_accept(session_con->server_pcb, iperflwip_tcp_accept);

    Report("\n\riperflwip_server: TCP Server is listening on port %d\n", session_con->lwipConfig.destOrLocalPortNumber);
}

// Accept callback
static err_t iperflwip_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    Report("\n\r iperflwip_server: Client connected !\n\r");

    session_conn_t* session_con = arg;


    if ((err != ERR_OK)  || (newpcb == NULL) || (arg == NULL)) {
        Report("\n\riperflwip_server: ERROR ! non valid configuration !\n\r");
        return ERR_VAL;
    }

    // Reset client state
    session_con->total_bytes = 0;
    session_con->is_running = true;
    session_con->bytes_per_period = 0;
    session_con->poll_count = 0;
    session_con->actualTestdurationMs = 0;
    session_con->actualNumOfDurations = 0;
    session_con->report_task_handle = NULL;
    session_con->previous_time = osi_GetTimeMS();
    session_con->start_time = osi_GetTimeMS();

    //the server pcb is freed and newpcb is allocated
    session_con->conn_pcb_tcp = newpcb;

    tcp_nagle_disable(newpcb);
    tcp_setprio(newpcb, TCP_PRIO_MAX);
    tcp_recv(newpcb, iperflwip_tcp_recv);
    tcp_err(newpcb, iperflwip_tcp_err);
    tcp_sent(newpcb, iperflwip_tcp_server_sent);
    tcp_poll(session_con->conn_pcb_tcp, lwiperf_tcp_poll, 100U);
    tcp_arg(newpcb, session_con);


    if ( session_con->lwipConfig.period || (session_con->lwipConfig.timeout >= 99999))
    {
        session_con->actualTestdurationMs = session_con->lwipConfig.period * 1000;
    }
    else
    {
        session_con->actualTestdurationMs = session_con->lwipConfig.timeout*1000;//sec to ms
    }

    if (session_con->lwipConfig.period > 0)
    {
        session_con->report_task_handle = xTimerCreate("tcp_srv_rpt",
            pdMS_TO_TICKS(session_con->actualTestdurationMs),
            pdTRUE, session_con, iperf_report_timer_cb);
        if (session_con->report_task_handle)
            xTimerStart(session_con->report_task_handle, 0);
    }

    return ERR_OK;
}

// Receive callback
static err_t iperflwip_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    session_conn_t *session_con = (session_conn_t *)arg;

    LWIP_ASSERT("pcb mismatch", session_con->conn_pcb_tcp == tpcb);

    if (err != ERR_OK){
        Report("\n\riperflwip_server: client closed connection\n");
        return ERR_OK;
    }

    if (p == NULL) {
        /* connection closed -> test done */
        Report("\n\riperflwip_server: null buffer received\n");
        iperflwip_tcp_stop(session_con,0);
        return ERR_OK;
    }

    session_con->poll_count = 0;

    //Ack that data received
    tcp_recved(tpcb, p->tot_len);

    session_con->total_bytes += p->tot_len;
    session_con->bytes_per_period += p->tot_len;
    pbuf_free(p);
#if 0
    //for fairness, verify if there is client which has something to send
    for(i=1; i<IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS+1; i++)
    {
        if(iperf_session[i].is_running &&
                !iperf_session[i].is_server &&
                iperf_session[i].conn_pcb_tcp)
        {
            if(iperf_session[i].proto == IPERF_PROTO_UDP)
            {
                iperflwip_udp_client_tx(&iperf_session[i]);
            }
            else
            {
                iperflwip_tcp_client_tx(&iperf_session[i]);
            }
        }
    }
#endif
    return ERR_OK;
}

// Sent callback (optional)
static err_t iperflwip_tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    return ERR_OK;
}


static void iperf_report_timer_cb(TimerHandle_t t)
{
    tcpip_callback(iperflwip_report, pvTimerGetTimerID(t));
}

// Timer callback after test duration
static void iperflwip_report(void* arg)
{
    double secondsFromStart, durationInSecond;
    uint32_t current_time;
    double bps;
    char ratestr[IPERF_MAX_FORMAT_RATE_LENGTH];
    session_conn_t* session_con = (session_conn_t*)arg;

    char tag[24];
    snprintf(tag, sizeof(tag), "[TCP:%u] [%d]",
             (unsigned)session_con->lwipConfig.destOrLocalPortNumber,
             session_con->process_num);

    if (session_con->is_running && session_con->conn_pcb_tcp != NULL) {

        session_con->actualNumOfDurations++;
        current_time = osi_GetTimeMS();
        uint32_t delta_ms = current_time - session_con->previous_time;
        durationInSecond = (double)delta_ms / 1000.0;    
        bps = ((double)session_con->bytes_per_period * 8.0) / durationInSecond;

        if(session_con->lwipConfig.period)
        {
            format_bps(bps,ratestr, sizeof(ratestr));

            Report("\n\r%s %s", tag, ratestr);
        }
        session_con->previous_time = osi_GetTimeMS();

        session_con->bytes_per_period = 0;

        if (!session_con->is_req_to_abort_test && ((session_con->lwipConfig.timeout >= 99999) ||
                ((session_con->lwipConfig.timeout*1000) > (session_con->actualTestdurationMs* session_con->actualNumOfDurations))))
        {
            /* timer is auto-reload - nothing to do */
        }
        else
        {
            //send iperf fin to the transmitter
            uint32_t  curr_time = osi_GetTimeMS();
            secondsFromStart = ((double)(curr_time - session_con->start_time));
            if(secondsFromStart > 0)
            {
                secondsFromStart=secondsFromStart/1000.0;
                bps = ((session_con->total_bytes * 8.0)/secondsFromStart);
                format_bps(bps,ratestr, sizeof(ratestr));
                Report("\n\r%s Test complete  %lu bytes  %.2f sec  %s",
                       tag, (unsigned long)session_con->total_bytes, secondsFromStart, ratestr);
                Report("\n\riperf TCP %s :  %lu total bytes duration :%lu sec  %.2f Mbps",
                       session_con->is_server ? "server" : "client",
                       (unsigned long)session_con->total_bytes,
                       (unsigned long)secondsFromStart,
                       bps / 1e6);
            }
            else
            {
                Report("\n\r%s Test finished\n", tag);
            }
            iperflwip_tcp_close(session_con);

        }
    }
    else
    {
        Report("\n\riperflwip_server: Test finished\n");
        iperflwip_tcp_close(session_con);
    }
}

int32_t iperflwip_tcp_server_start(void* args)
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
            iperf_session[i].is_server = 1;
            iperf_session[i].proto = IPERF_PROTO_TCP;
            iperf_session[i].process_num = i;
            iperf_session[i].iperf_reportFunc = iperflwip_report;
            iperf_session[i].is_req_to_abort_test = 0;
            iperf_session[i].is_stop_due_traffic_error = 0;
            iperf_session[i].udp_server_first_packet_recv = 0;
            iperf_session[i].is_running = true;
            break;
        }
    }

    if(found)
    {
        Report("\r\n!!To stop the TCP server , iperf_stop -n %d ", i);
        os_memcpy((void *)&iperf_session[i].lwipConfig,args, sizeof(RecvCmd_t));
        tcpip_callback(iperflwip_server_tcp_init, (void *)&iperf_session[i]);
        os_sleep(0,50);//give time to the iperf_server_init to be trigger
        return 0;
    }
    else
    {
        Report("Num of sessions exceeded, max num = %d ",IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS);
        return -1;
    }

}



