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
#include "lwip_iperf_examples.h"


#ifdef CC35XX


#define IPERF_LWIP_CLIENT_DURATION_MS 10000 // Test duration (10 seconds)
#define SEND_BUFFER_SIZE_UDP_CLIENT (TCP_MSS)

#define IPERF_LWIP_MAX_FORMAT_RATE_LENGTH  20

extern session_conn_t iperf_session[];

// Forward declarations
static void iperflwip_client_udp_init(void *param);
static void  iperflwip_udp_client_tx(void* arg);
int32_t iperflwip_udp_client_start(void* args);

extern void format_bps(double bps, char *output, size_t size);
static void iperflwip_client_udp_os_timer_callback(union sigval sv);
//static void  lwiperf_udp_client_recv(void *arg, struct udp_pcb *pcb,
//        struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udp_client_task(void *arg);

extern unsigned char send_buffer[];
static void iperflwip_udp_close(void *arg);
static void iperflwip_send_udp_client_iperf_fin(session_conn_t* session_con);


static void iperflwip_client_udp_init(void *param)
{
    struct sigevent         event;

    session_conn_t* session_con = param;
    session_con->actualTestdurationMs = 0;
    session_con->actualNumOfDurations = 0;
    session_con->conn_pcb_udp = NULL;
    session_con->conn_pcb_tcp = NULL;
    session_con->os_timer = 0;
    session_con->total_bytes = 0;
    session_con->bytes_per_period = 0;
    session_con->poll_count = 0;
    session_con->target_Bps = (session_con->lwipConfig.bandwidth * 1000* 1000)/8;

    session_con->dest_ip.addr = htonl((unsigned int )session_con->lwipConfig.ipAddr.ipv4);
    session_con->dest_port = session_con->lwipConfig.destOrLocalPortNumber;

    session_con->conn_pcb_udp = udp_new_ip_type(IPADDR_TYPE_V4);
    if (session_con->conn_pcb_udp == NULL) {
        Report("\n\riperflwip_client: ERROR ! Failed to create pcb\n");
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

    event.sigev_notify = SIGEV_THREAD;
    event.sigev_value.sival_ptr = session_con;
    event.sigev_notify_function = iperflwip_client_udp_os_timer_callback;
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

    xTaskCreate(udp_client_task, "udp_client", 512, session_con, tskIDLE_PRIORITY + 1, NULL);
}


void udp_client_task(void *arg) {
    session_conn_t* session_con = arg;
    uint32_t throughput_timer;
    uint32_t now;
    uint32 Bandwidth_byte_per_100_mili;
    uint64_t number_of_bytes_send_from_last_mili;


    session_con->total_bytes =0;
    session_con->bytes_per_period =0;

    number_of_bytes_send_from_last_mili = 0;
    Bandwidth_byte_per_100_mili =  (uint32_t)((uint64_t)session_con->target_Bps/10);//bytes per 100 mili
    //Report("\n\r Bandwidth_byte_per_100_mili:%d bytes ",
    //        Bandwidth_byte_per_100_mili);

    throughput_timer = osi_GetTimeMS();

    while (session_con->is_running)
    {
        if(session_con->target_Bps>0)
        {
            now = osi_GetTimeMS();
            if((now-throughput_timer)>100)
            {
                //Report("\n\r 100 mili passed, during it send:%d bytes ",
                //        number_of_bytes_send_from_last_mili);

                number_of_bytes_send_from_last_mili = 0;
                throughput_timer = now;
            }
            else if(number_of_bytes_send_from_last_mili >= (Bandwidth_byte_per_100_mili))
            {
                   //sleep 100Ms
                   uint32_t sleep_ms = 100 - (now-throughput_timer) ;
                   os_sleep(sleep_ms/1000, (sleep_ms%1000)*1000 );
                   //Report("\n\r  after sleep, sleep_ms:%d", sleep_ms);
                   continue;
            }

            if(osi_GetFreeHeapSize() > HEAP_THRESHOLD_FOR_TX)
            {
                session_con->number_of_bytes_to_send_on_current_tx =
                        MIN(Bandwidth_byte_per_100_mili - number_of_bytes_send_from_last_mili,
                                SEND_BUFFER_SIZE_UDP_CLIENT);
                //Report("\n\r number_of_bytes_to_send_on_current_tx:%d ",
                //      session_con->number_of_bytes_to_send_on_current_tx);
                tcpip_callback(iperflwip_udp_client_tx,(void *) session_con);
                number_of_bytes_send_from_last_mili += session_con->number_of_bytes_to_send_on_current_tx;

            }
        }
        else
        {
            if(osi_GetFreeHeapSize() > HEAP_THRESHOLD_FOR_TX)
            {
                session_con->number_of_bytes_to_send_on_current_tx = SEND_BUFFER_SIZE_UDP_CLIENT;
                tcpip_callback(iperflwip_udp_client_tx,(void *) session_con);
            }
        }
        taskYIELD();//allow others to run


    }
    tcpip_callback(iperflwip_udp_close, session_con);
    vTaskDelete(NULL);

}

void udp_client_stop(session_conn_t* session_con)
{
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


void iperflwip_udp_client_close(session_conn_t* session_con)
{
    session_con->is_running = false;
    if (session_con->os_timer != 0) {

        struct itimerspec       its = {0};
        //stop the timer
        timer_settime(session_con->os_timer, 0, &its, NULL);
        timer_delete(session_con->os_timer);

        session_con->os_timer = 0;
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


// Send function: chunked sending as fast as possible
static void  iperflwip_udp_client_tx(void* arg)
{
    struct pbuf *p;
    session_conn_t* session_con = arg;
    uint32_t time_mili_sec,sec,usec,count;
    uint32_t len = session_con->number_of_bytes_to_send_on_current_tx;

    // Allocate a pbuf
    p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
    if (!p) {
        Report("\n\rFailed to allocate pbuf\n");
        session_con->number_of_bytes_to_send_on_current_tx = 0;
        return;
    }

    os_memcpy(p->payload, send_buffer, len);

    time_mili_sec = osi_GetTimeMS();

    sec = htonl(time_mili_sec/1000);
    usec = htonl((time_mili_sec%1000)*1000);
    count = htonl(session_con->packet_count);

    os_memcpy(send_buffer, &count, sizeof(count));
    os_memcpy(send_buffer+4, &sec, sizeof(sec));
    os_memcpy(send_buffer+8, &usec, sizeof(usec));


    // Send the data
    err_t err = udp_sendto(session_con->conn_pcb_udp, p, &session_con->dest_ip, (uint16_t)session_con->dest_port);
    if (err != ERR_OK) {
        //Report("\n\rudp_sendto failed: %d\n", err);
        session_con->number_of_bytes_to_send_on_current_tx = 0;
    } else {
        session_con->packet_count++;
        session_con->total_bytes += len;
        session_con->bytes_per_period += len;

    }

    pbuf_free(p);
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
        Report("\n\riperflwip_send_udp_client_iperf_fin: Failed to allocate pbuf\n");
        return;
    }
    
    os_memcpy(p->payload, &fin_pkt, sizeof(fin_pkt));

    udp_sendto(session_con->conn_pcb_udp, p, &session_con->dest_ip, (uint16_t)session_con->dest_port);

    pbuf_free(p);
}

// Timer expired -> test done
static void iperflwip_report(void* arg)
{
    uint64_t secondsFromStart, durationInSecond;
    float bps;
    char ratestr[IPERF_LWIP_MAX_FORMAT_RATE_LENGTH];
    session_conn_t* session_con = (session_conn_t*)arg;


    if (session_con->is_running && session_con->conn_pcb_udp != NULL)
    {

        session_con->actualNumOfDurations++;

        if(session_con->lwipConfig.period)
        {
            durationInSecond = session_con->actualTestdurationMs / 1000.0f;

            bps = (session_con->bytes_per_period * 8)/durationInSecond;

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

            Report("\n\riperflwip: [%d] UDP client Test finished, udp fin send to the server",session_con->process_num);

            bps = (session_con->total_bytes * 8)/secondsFromStart;
            format_bps(bps,ratestr, sizeof(ratestr));
            Report("\n\riperf UDP client :  %lu total bytes duration :%lu sec", (unsigned long )session_con->total_bytes,secondsFromStart);
            Report("\t %s \n", ratestr);
            iperflwip_send_udp_client_iperf_fin(session_con);
            iperflwip_udp_client_close(session_con);
        }
    }
    else
    {
        Report("\n\riperflwip: UDP client Test finished\n");
    }
}


static void iperflwip_udp_close(void *arg)
{
    session_conn_t *session_con = arg;

    if (session_con->conn_pcb_udp != NULL)
    {
        udp_recv(session_con->conn_pcb_udp, NULL, NULL);
        udp_remove(session_con->conn_pcb_udp);
        session_con->conn_pcb_udp = NULL;
    }
    if (session_con->os_timer != 0) {

        struct itimerspec       its = {0};
        //stop the timer
        timer_settime(session_con->os_timer, 0, &its, NULL);
        timer_delete(session_con->os_timer);
        session_con->os_timer = 0;
    }
    session_con->is_running = false;
}


static void iperflwip_client_udp_os_timer_callback(union sigval sv)
{
    tcpip_callback(iperflwip_report, sv.sival_ptr);
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
            iperf_session[i].is_udp = 1;
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


#endif
