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

#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip_iperf_examples.h"


#ifdef CC35XX


#define IPERF_TEST_DURATION 10000 // milliseconds (10 sec)
#define IPERF_TEST_PRINT_DURATION 1000 // milliseconds (1 sec)
#define IPERF_MAX_FORMAT_RATE_LENGTH  20


extern session_conn_t iperf_session[];
extern int32_t canSend;

// Forward declarations
static err_t iperflwip_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t iperflwip_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t iperflwip_tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void iperflwip_server_os_timer_callback(union sigval sv);

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
    session_con->os_timer = 0;
    session_con->total_bytes = 0;
    session_con->bytes_per_period = 0;

    session_con->src_ip.addr = htonl((unsigned int )session_con->lwipConfig.ipAddr.ipv4);
    session_con->src_port = session_con->lwipConfig.destOrLocalPortNumber;

    if ((session_con->src_ip.addr != 0) && (is_ip_addr_in_net_list(&session_con->src_ip) != 0))
    {
        Report("\n\riperflwip_server: ERROR: ERROR! Source IP address is not in netlist.\n\r");
        return;
    }


    session_con->server_pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (session_con->server_pcb == NULL) {
        Report("\n\riperflwip_server: ERROR ! Failed to create pcb\n");
        return;
    }

    //local port and local IP address
    if(tcp_bind(session_con->server_pcb, &session_con->src_ip, session_con->src_port)!= ERR_OK)
	{
	    Report("\n\riperflwip_server: Failed to bind PCB\n");
		tcp_close(session_con->server_pcb);
		session_con->server_pcb = NULL;
	}

    tcp_arg(session_con->server_pcb, session_con);
	
    session_con->server_pcb = tcp_listen(session_con->server_pcb);


    tcp_accept(session_con->server_pcb, iperflwip_tcp_accept);

    Report("\n\riperflwip_server: TCP Server is listening on port %d\n", session_con->lwipConfig.destOrLocalPortNumber);
}

// Accept callback
static err_t iperflwip_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    Report("\n\r iperflwip_server: Client connected !\n\r");

    session_conn_t* session_con = arg;
    struct sigevent         event;


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
    session_con->os_timer = 0;
    session_con->previous_time = osi_GetTimeMS();
    session_con->start_time = osi_GetTimeMS();

    //the server pcb is freed and newpcb is allocated
    session_con->conn_pcb_tcp = newpcb;

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

    // Start timer
    event.sigev_notify = SIGEV_THREAD;
    event.sigev_value.sival_ptr = session_con;
    event.sigev_notify_function = iperflwip_server_os_timer_callback;
    event.sigev_notify_attributes = NULL;


    timer_create(CLOCK_REALTIME, &event, &session_con->os_timer);

    if (session_con->os_timer != 0) {
        struct itimerspec       its = {0};
        its.it_value.tv_sec = (session_con->actualTestdurationMs / 1000);
        its.it_value.tv_nsec = (session_con->actualTestdurationMs % 1000)*1000000; // expiration
        timer_settime(session_con->os_timer, 0, &its, NULL);
    } else {
        Report("\n\riperflwip_server: Failed to create timer\n");
    }

    return ERR_OK;
}

// Receive callback
static err_t iperflwip_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    session_conn_t *session_con = (session_conn_t *)arg;
    int8_t i;

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

    struct pbuf *q = p;
    while (q != NULL) {
            session_con->total_bytes += q->len;
            session_con->bytes_per_period += q->len;;
            q = q->next;
        }

    //Ack that data received
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    for(i=1; i<IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS+1; i++)
    {
        if(iperf_session[i].is_running &&
                !iperf_session[i].is_server &&
                !iperf_session[i].is_udp &&
                iperf_session[i].conn_pcb_tcp)
        {
            canSend = 1;
            iperflwip_tcp_client_tx(&iperf_session[i]);
        }
    }
    return ERR_OK;
}

// Sent callback (optional)
static err_t iperflwip_tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    return ERR_OK;
}


// Timer callback after test duration
static void iperflwip_report(void* arg)
{
    double secondsFromStart, durationInSecond;
    uint32_t current_time;
    double bps;
    char ratestr[IPERF_MAX_FORMAT_RATE_LENGTH];
    session_conn_t* session_con = (session_conn_t*)arg;

    if (session_con->is_running && session_con->conn_pcb_tcp != NULL) {

        session_con->actualNumOfDurations++;
        current_time = osi_GetTimeMS();
        uint32_t delta_ms = current_time - session_con->previous_time;
        durationInSecond = (double)delta_ms / 1000.0;    
        bps = ((double)session_con->bytes_per_period * 8.0) / durationInSecond;

        if(session_con->lwipConfig.period)
        {
            format_bps(bps,ratestr, sizeof(ratestr));

            Report("\n\r[%d] %s",session_con->process_num,ratestr);
        }
        session_con->previous_time = osi_GetTimeMS();

        session_con->bytes_per_period = 0;

        if (!session_con->is_req_to_abort_test && ((session_con->lwipConfig.timeout >= 99999) || //any number bigger than 900000
                ((session_con->lwipConfig.timeout*1000) > (session_con->actualTestdurationMs* session_con->actualNumOfDurations))))
        {
            //trigger the timer again
            struct itimerspec       its = {0};
            its.it_value.tv_sec = (session_con->actualTestdurationMs / 1000);
            its.it_value.tv_nsec = (session_con->actualTestdurationMs % 1000)*1000000; // expiration
            timer_settime(session_con->os_timer, 0, &its, NULL);

        }
        else
        {
            //send iperf fin to the transmitter
            uint32_t  curr_time = osi_GetTimeMS();
            secondsFromStart = ((double)(curr_time - session_con->start_time));
            Report("\n\riperflwip: [%d] TCP server Test finished",session_con->process_num);

            if(secondsFromStart > 0)
            {
                secondsFromStart=secondsFromStart/1000.0;
                bps = ((session_con->total_bytes * 8.0)/secondsFromStart);
                format_bps(bps,ratestr, sizeof(ratestr));
                Report("\n\riperf TCP server :  %lu total bytes duration :%lu sec", (unsigned long )session_con->total_bytes,(unsigned long )secondsFromStart);
                Report("\t %s \n", ratestr);
            }
            iperflwip_tcp_close(session_con);

        }
    }
    else
    {

        Report("\n\riperflwip_server: Test finished\n");
    }
}

static void iperflwip_server_os_timer_callback(union sigval sv)
{
    tcpip_callback(iperflwip_report,sv.sival_ptr);
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
            iperf_session[i].is_udp = 0;
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


#endif
