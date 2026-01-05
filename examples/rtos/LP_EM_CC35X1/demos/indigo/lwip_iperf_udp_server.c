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


#define IPERF_LWIP_SERVER_DURATION_MS 10000 // Test duration (10 seconds)
#define IPERF_LWIP_MAX_FORMAT_RATE_LENGTH  20

extern session_conn_t iperf_session[];
extern int32_t canSend;

// Forward declarations
static void iperflwip_server_udp_init(void *param);
int32_t iperflwip_udp_server_start(void* args);

extern void format_bps(double bps, char *output, size_t size);
static void iperflwip_udp_os_timer_callback(union sigval sv);
static void  lwiperf_udp_server_recv(void *arg, struct udp_pcb *pcb,
        struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udp_server_stop(session_conn_t* session_con);
static void iperflwip_udp_server_stop(void *arg);
static void iperflwip_udp_server_close(session_conn_t *session_con);


static void iperflwip_server_udp_init(void *param)
{
    err_t err;

    session_conn_t* session_con = param;
    session_con->actualTestdurationMs = 0;
    session_con->actualNumOfDurations = 0;
    session_con->conn_pcb_udp = NULL;
    session_con->os_timer = 0;
    session_con->total_bytes = 0;
    session_con->bytes_per_period = 0;
    session_con->poll_count = 0;

    session_con->src_ip.addr = htonl((unsigned int )session_con->lwipConfig.ipAddr.ipv4);
    session_con->src_port = session_con->lwipConfig.destOrLocalPortNumber;

    if ((session_con->src_ip.addr != 0) && (is_ip_addr_in_net_list(&session_con->src_ip) != 0))
    {
        Report("\n\riperflwip_server: ERROR: ERROR! Source IP address is not in netlist.\n\r");
        return;
    }


    session_con->conn_pcb_udp = udp_new_ip_type(IPADDR_TYPE_V4);
    if (session_con->conn_pcb_udp == NULL) {
        Report("\n\riperflwip_server: ERROR ! Failed to create pcb\n");
        return;
    }

    err = udp_bind(session_con->conn_pcb_udp, &session_con->src_ip, session_con->src_port);
    if(err != ERR_OK){
        Report("\n\riperflwip_server: ERROR ! tcp_bind, port is in use\n");
        udp_remove(session_con->conn_pcb_udp);
        session_con->conn_pcb_udp = NULL;
        return;
    }

    udp_recv(session_con->conn_pcb_udp, lwiperf_udp_server_recv,(void*) session_con);


    printf("\n\rUDP iperf server started on port %d\n", (int)session_con->lwipConfig.destOrLocalPortNumber);


    session_con->total_bytes = 0;
    session_con->is_running = true;
    session_con->actualTestdurationMs = 0;


}



static void  lwiperf_udp_server_recv(void *arg, struct udp_pcb *pcb,
        struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    session_conn_t *session_con = (session_conn_t *)arg;
    struct sigevent         event;

    LWIP_ASSERT("pcb mismatch", session_con->conn_pcb_udp == pcb);

    if (p == NULL) {
        /* connection closed -> test done */
        Report("\n\rlwiperf_udp_server_recv: null buffer received\n");
        iperflwip_udp_server_stop(session_con);
        return;
    }

    if(!session_con->udp_server_first_packet_recv)
    {
        session_con->udp_server_first_packet_recv = 1;
        //in case it is the first packet, start counting
        session_con->previous_time = osi_GetTimeMS();
        session_con->start_time = osi_GetTimeMS();
        if ( session_con->lwipConfig.period)
        {
            session_con->actualTestdurationMs = session_con->lwipConfig.period * 1000;
        }
        else if(session_con->lwipConfig.timeout >= (uint32_t)99999)
        {
            session_con->actualTestdurationMs = IPERF_LWIP_SERVER_DURATION_MS * 1000;
        }
        else
        {
            session_con->actualTestdurationMs = session_con->lwipConfig.timeout*1000;//sec to ms
        }

        event.sigev_notify = SIGEV_THREAD;
        event.sigev_value.sival_ptr = session_con;
        event.sigev_notify_function = iperflwip_udp_os_timer_callback;
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
            Report("\n\riperflwip_client udp: ERROR ! fail to create timer");
        }

        Report("Server listening on port:%d, packet recv from remote %s:%d\n",
                    pcb->local_port,
                    ipaddr_ntoa(addr),
                    port);

    }

    struct pbuf *q = p;
    if (q->len >= sizeof(struct iperf_udp_hdr)){
        struct iperf_udp_hdr *hdr = (struct iperf_udp_hdr *)p->payload;
        int32_t seq = lwip_ntohl(hdr->id);
        if (seq < 0) {
            Report("\n\rUDP FIN received from %s:%d",
                    ipaddr_ntoa(addr), port);
            iperflwip_udp_server_stop(session_con);
        } else {
            while (q != NULL) {
                session_con->total_bytes += q->len;
                session_con->bytes_per_period += q->len;;
                q = q->next;
            }
        }
    }
    // Optional: echo back the received packet (iperf expects no response)
    // udp_sendto(pcb, p, addr, port);

    pbuf_free(p);

}

// Timer expired -> test done
static void iperflwip_report(void* arg)
{
    double secondsFromStart;
    double durationInSecond;
    uint32_t current_time;
    double bps;
    char ratestr[IPERF_LWIP_MAX_FORMAT_RATE_LENGTH];
    session_conn_t* session_con = (session_conn_t*)arg;


    if (session_con->is_running && session_con->conn_pcb_udp != NULL)
    {
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

        if (!session_con->is_req_to_abort_test &&  ((session_con->lwipConfig.timeout >= 99999) ||
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

            //send iperf fin to the transmitter
            uint32_t  curr_time = osi_GetTimeMS();
            secondsFromStart = ((double)(curr_time - session_con->start_time));
            Report("\n\riperflwip: [%d] UDP server Test finished",session_con->process_num);

            if(secondsFromStart > 0)
            {
                secondsFromStart=secondsFromStart/1000.0;
                bps = ((session_con->total_bytes * 8.0)/secondsFromStart);
                format_bps(bps,ratestr, sizeof(ratestr));
                Report("\n\riperf UDP server :  %lu total bytes duration :%lu sec", (unsigned long )session_con->total_bytes,(unsigned long )secondsFromStart);
                Report("\t %s \n", ratestr);
            }
            iperflwip_udp_server_close(session_con);
        }
    }
    else
    {
        Report("\n\riperflwip: UDP  Test finished\n");
    }

}

void udp_server_stop(session_conn_t* session_con)
{
    tcpip_callback(iperflwip_udp_server_stop, session_con);
}

static void iperflwip_udp_server_stop(void *arg)
{
    session_conn_t *session_con = arg;

    if (session_con->os_timer != 0) {

        struct itimerspec       its = {0};
        //stop the timer
        timer_settime(session_con->os_timer, 0, &its, NULL);
        timer_delete(session_con->os_timer);

        session_con->os_timer = 0;
    }


    session_con->is_req_to_abort_test = 1;
    tcpip_callback(session_con->iperf_reportFunc,session_con);


}

static void iperflwip_udp_server_close(session_conn_t *session_con)
{
    if (session_con->os_timer != 0) {

        struct itimerspec       its = {0};
        //stop the timer
        timer_settime(session_con->os_timer, 0, &its, NULL);
        timer_delete(session_con->os_timer);

        session_con->os_timer = 0;
    }

    if (session_con->conn_pcb_udp != NULL) {
        if(session_con->is_running)
        {
            udp_recv(session_con->conn_pcb_udp, NULL, NULL);
            udp_remove(session_con->conn_pcb_udp);
        }
    }
    session_con->is_running = false;
    session_con->conn_pcb_udp = NULL;

}


static void iperflwip_udp_os_timer_callback(union sigval sv)
{
    tcpip_callback(iperflwip_report, sv.sival_ptr);
}

int32_t iperflwip_udp_server_start(void* args)
{
    int i;
    uint8_t found = FALSE;

    //find available iperf UDPs server
    for(i=1; i< IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS+1; i++)
    {
        if(!iperf_session[i].is_running)
        {
            found = TRUE;//found not running process
            os_memset(&iperf_session[i],0,sizeof(iperf_session[i]));
            iperf_session[i].is_server = 1;
            iperf_session[i].is_udp = 1;
            iperf_session[i].process_num = i;
            iperf_session[i].iperf_reportFunc = iperflwip_report;
            iperf_session[i].is_running = true;
            break;
        }
    }

    if(found)
    {
        Report("\r\nTo stop the UDP process , iperf_stop -n %d ", i);
        os_memcpy((void *)&iperf_session[i].lwipConfig,args, sizeof(RecvCmd_t));
        tcpip_callback(iperflwip_server_udp_init,(void *) (void *)&iperf_session[i]);
        os_sleep(0,50);//give time to the iperflwip_server_udp_init to be trigger
        return 0;
    }
    else
    {
        Report("\n\riperflwip_: Num of sessions exceeded, max num = %d ",IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS);
        return -1;
    }

}


#endif
