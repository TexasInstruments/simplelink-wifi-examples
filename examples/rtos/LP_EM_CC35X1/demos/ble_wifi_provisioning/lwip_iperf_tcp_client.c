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
#include "network_lwip.h"

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/tcpbase.h"
#include "lwip_iperf_examples.h"


#ifdef CC35XX


#define IPERF_LWIP_CLIENT_DURATION_MS 10000 // Test duration (10 seconds)

#define IPERF_LWIP_MAX_FORMAT_RATE_LENGTH  20

#define SEND_BUFFER_SIZE TCP_MSS


extern session_conn_t iperf_session[];
extern int32_t canSend;

// Forward declarations
static err_t iperflwip_tcp_session_conected(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t lwiperf_tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void iperflwip_client_tcp_init(void *param);
err_t iperflwip_tcp_client_tx(session_conn_t* session_con);

extern void format_bps(double bps, char *output, size_t size);
extern void iperflwip_tcp_stop(void *arg, uint8_t isError);
extern void iperflwip_tcp_err(void *arg, err_t err);
extern err_t lwiperf_tcp_poll(void *arg, struct tcp_pcb *tpcb);
static void iperflwip_client_tcp_os_timer_callback(union sigval sv);

extern unsigned char send_buffer[];


static void iperflwip_client_tcp_init(void *param)
{
    err_t err;

    session_conn_t* session_con = param;
    session_con->actualTestdurationMs = 0;
    session_con->actualNumOfDurations = 0;
    session_con->conn_pcb_tcp = NULL;
    session_con->os_timer = 0;
    session_con->total_bytes = 0;
    session_con->bytes_per_period = 0;
    session_con->poll_count = 0;

    session_con->dest_ip.addr = htonl((unsigned int )session_con->lwipConfig.ipAddr.ipv4);

    session_con->conn_pcb_tcp = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (session_con->conn_pcb_tcp == NULL) {
        Report("\n\riperflwip_client: ERROR ! Failed to create pcb\n");
        return;
    }

    err = tcp_bind(session_con->conn_pcb_tcp, IP4_ADDR_ANY4, LOCAL_TCP_CLIENT_PORT);
    if(err != ERR_OK){
        Report("\n\riperflwip_client: ERROR ! tcp_bind, port is in use\n");
        return;
    }

    tcp_nagle_disable(session_con->conn_pcb_tcp);
    tcp_arg(session_con->conn_pcb_tcp, session_con);
    tcp_err(session_con->conn_pcb_tcp, iperflwip_tcp_err);
    tcp_sent(session_con->conn_pcb_tcp, lwiperf_tcp_client_sent);
    tcp_poll(session_con->conn_pcb_tcp, lwiperf_tcp_poll, 100U);


    err = tcp_connect(session_con->conn_pcb_tcp, &session_con->dest_ip, session_con->lwipConfig.destOrLocalPortNumber, iperflwip_tcp_session_conected);
    if (err != ERR_OK) {
        Report("\n\riperflwip_client: ERROR ! TCP_connection failed: %d\n", err);
        tcp_close(session_con->conn_pcb_tcp);
    }
}

// Connection established
err_t iperflwip_tcp_session_conected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    session_conn_t* session_con = arg;
    struct sigevent         event;

    if(session_con->conn_pcb_tcp != tpcb)
    {
        Report("\n\riperflwip_client: Connect error\n");
    }

    if (err != ERR_OK) {
        Report("\n\riperflwip_client: Connect error\n");
        tcp_close(session_con->conn_pcb_tcp);
        return err;
    }

    Report("\n\riperflwip_client: Connected to server!\n\r");

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

    event.sigev_notify = SIGEV_THREAD;
    event.sigev_value.sival_ptr = session_con;
    event.sigev_notify_function = iperflwip_client_tcp_os_timer_callback;
    event.sigev_notify_attributes = NULL;


    // Start test timer
    timer_create(CLOCK_REALTIME, &event, &session_con->os_timer);


    if (session_con->os_timer != 0) {
        struct itimerspec       its = {0};
        its.it_value.tv_sec = (session_con->actualTestdurationMs / 1000);
        its.it_value.tv_nsec = (session_con->actualTestdurationMs % 1000)*1000000; // expiration
        timer_settime(session_con->os_timer, 0, &its, NULL);
    }
    else {
        Report("\n\riperflwip_client: ERROR ! fail to create timer");
    }

    canSend = 1;
    iperflwip_tcp_client_tx(session_con);

    return ERR_OK;
}

//function is been called when tcp-ack is received
static err_t lwiperf_tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    session_conn_t* session_con = arg;


    LWIP_ASSERT("iperflwip_client: invalid conn", session_con->conn_pcb_tcp == tpcb);
    LWIP_UNUSED_ARG(tpcb);
    LWIP_UNUSED_ARG(len);

    session_con->poll_count = 0;

    canSend = 1;

    return iperflwip_tcp_client_tx(session_con);
}


// Send function: chunked sending as fast as possible
err_t iperflwip_tcp_client_tx(session_conn_t* session_con) {
    err_t err = ERR_OK;

    struct tcp_pcb * tpcb = session_con->conn_pcb_tcp;
    while (err == ERR_OK && canSend && tcp_sndbuf(tpcb) > 0) {
        u16_t len = tcp_sndbuf(tpcb);

        if (len > SEND_BUFFER_SIZE)
        {
            len = SEND_BUFFER_SIZE;
        }


        err = tcp_write(tpcb, send_buffer, len, 0);
        if (err == ERR_OK) {
            session_con->packet_count++;
            session_con->total_bytes += len;
            session_con->bytes_per_period += len;
            // You can add optional logic here to track bytes sent
        } else if (err == ERR_MEM) {
            // Can't send more now, will try again in sent callback
            canSend = 0;
        }
    }
    //tcp_output(tpcb);  // Flush the data
    return ERR_OK;
}


// Timer expired -> test done
static void iperflwip_report(void* arg)
{
    uint64_t secondsFromStart, durationInSecond;
    float bps = 0;
    char ratestr[IPERF_LWIP_MAX_FORMAT_RATE_LENGTH];
    session_conn_t* session_con = (session_conn_t*)arg;


    if (session_con->is_running && session_con->conn_pcb_tcp != NULL) {

        session_con->actualNumOfDurations++;

        if(session_con->lwipConfig.period)
        {
            durationInSecond = session_con->actualTestdurationMs / 1000.0f;

            if(durationInSecond){
                bps = (session_con->bytes_per_period * 8)/durationInSecond;
            }

            format_bps(bps,ratestr, sizeof(ratestr));
            Report("\n\r[%d] %s",session_con->process_num,ratestr);

        }

        session_con->bytes_per_period = 0;

        if (!session_con->is_req_to_abort_test && ((session_con->lwipConfig.timeout >= 99999) ||
                (session_con->lwipConfig.timeout*1000 > (session_con->actualTestdurationMs* session_con->actualNumOfDurations))))
        {
            //trigger the timer again
            struct itimerspec       its = {0};
            its.it_value.tv_sec = (session_con->actualTestdurationMs / 1000);
            its.it_value.tv_nsec = (session_con->actualTestdurationMs % 1000)*1000000; // expiration
            timer_settime(session_con->os_timer, 0, &its, NULL);
        }
        else
        {
            secondsFromStart = (session_con->actualTestdurationMs* session_con->actualNumOfDurations)/1000;

            Report("\n\riperflwip: [%d] TCP client Test finished",session_con->process_num);

            bps = (session_con->total_bytes * 8)/secondsFromStart;
            format_bps(bps,ratestr, sizeof(ratestr));
            Report("\n\riperf TCP client :  %lu total bytes duration :%lu sec", (unsigned long )session_con->total_bytes,secondsFromStart);
            Report("\t %s \n", ratestr);

            iperflwip_tcp_close(session_con);

        }
    }
    else
    {
        Report("\n\riperflwip: TCP client Test finished\n");
    }

}


static void iperflwip_client_tcp_os_timer_callback(union sigval sv)
{
    tcpip_callback(iperflwip_report, sv.sival_ptr);
}

int32_t iperflwip_tcp_client_start(void* args)
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
            iperf_session[i].is_udp = 0;
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
        Report("\r\nTo stop the TCP client process , iperf_stop -n %d ", i);
        os_memcpy((void *)&iperf_session[i].lwipConfig,args, sizeof(RecvCmd_t));
        tcpip_callback(iperflwip_client_tcp_init,(void *) (void *)&iperf_session[i]);
        os_sleep(0,50);//give time to the iperflwip_server_tcp_init to be trigger
        return 0;
    }
    else
    {
        Report("\n\riperflwip_client: Num of sessions exceeded, max num = %d ",IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS);
        return -1;
    }

}


#endif
