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
#include "lwip_iperf_examples.h"
#include "str.h"
#include "lwip/opt.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmd_parser.h"
#include "uart_term.h"
#include "errors.h"
#include "FreeRTOSConfig.h"
#include "osi_kernel.h"
#include "wlan_if.h"
#include "network_lwip.h"
#include "cmd_parser.h"


extern int32_t iperflwip_tcp_server_start(void* args);
extern int32_t iperflwip_tcp_client_start(void* args);
extern err_t iperflwip_tcp_client_tx(session_conn_t* session_con);
extern int32_t iperflwip_udp_client_start(void* args);
extern void udp_client_stop(session_conn_t* session_con);
extern int32_t iperflwip_udp_server_start(void* args);
extern void udp_server_stop(session_conn_t* session_con);
extern int32_t tls_iperf_client_start(void* args);
extern int32_t tls_iperf_server_start(void* args);


int32_t iperflwip_stop(void* args);


#define MAX_SEND_BUFFER_SIZE (1470)
session_conn_t iperf_session[IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS] = {0};
unsigned char send_buffer[MAX_SEND_BUFFER_SIZE];
uint8_t send_buffer_initialized = 0;


void iperflwip_tcp_stop_from_cb(void *arg);


int32_t cmdTestIperfCallback(void *arg)
{
    int32_t ret = 0;
    RecvCmd_t IperfCmdParams;
    int i;

    if(!send_buffer_initialized)
    {
        for(i=0;i<MAX_SEND_BUFFER_SIZE;i++)
        {
            send_buffer[i]= (uint8_t)i;
        }
        send_buffer_initialized = 1;
    }

     /* Call the command parser */
     memset(&IperfCmdParams, 0x0, sizeof(RecvCmd_t));
     ret = ParseTestIperfCmd(arg, &IperfCmdParams);

     if (ret < 0)
     {
         Report("\n\r wrong choices");
         return (-1);
     }
     uint8_t protocol = IperfCmdParams.udpFlag? SOCK_DGRAM : SOCK_STREAM ;

     if (IperfCmdParams.server)
     {
         if(protocol == SOCK_STREAM)//TCP server
         {
             ret = iperflwip_tcp_server_start((void *)&IperfCmdParams);
         }
         else
         {
             ret = iperflwip_udp_server_start((void *)&IperfCmdParams);
         }
     }
     else
     {
         if(protocol == SOCK_STREAM)//TCP client
         {
             ret = iperflwip_tcp_client_start((void *)&IperfCmdParams);
         }
         else
         {
             ret = iperflwip_udp_client_start((void *)&IperfCmdParams);
         }
     }

     return (0);
 }


int32_t cmdStopTestIperfCallback(void *arg)
{
    stopCmd_t stopCmd;
    int32_t ret = 0;

    memset(&stopCmd, 0x0, sizeof(stopCmd_t));
    ret = ParseStopTestIperfCmd(arg, &stopCmd);

    if (ret < 0)
    {
        return (-1);
    }

    Report("\r\nRequest to Stop iperf process number :%d", stopCmd.processNum );
    ret = iperflwip_stop(&stopCmd.processNum);
    return ret;

}

static const char *proto_name(iperf_proto_t proto)
{
    switch (proto)
    {
        case IPERF_PROTO_TCP: return "TCP";
        case IPERF_PROTO_UDP: return "UDP";
        case IPERF_PROTO_TLS: return "TLS";
        default:              return "???";
    }
}

int32_t iperflwip_stop(void* args)
{
    int process_num = *((uint32_t *)args);
    int i;

    if( iperf_session[process_num].is_running )
    {
        session_conn_t *session = &iperf_session[process_num];

        switch (session->proto)
        {
            case IPERF_PROTO_TCP:
                tcpip_callback(iperflwip_tcp_stop_from_cb, (void *)session);
                os_sleep(0,50);
                Report("\r\niperf tcp process number :%d stopped! ", process_num);
                return 0;

            case IPERF_PROTO_UDP:
                if (session->is_server)
                {
                    udp_server_stop(session);
                }
                else
                {
                    udp_client_stop(session);
                }
                os_sleep(0,50);
                Report("\r\niperf udp %s process number :%d stopped! ",
                       session->is_server ? "server" : "client", process_num);
                return 0;

            case IPERF_PROTO_TLS:
                session->is_req_to_abort_test = 1;
                Report("\r\niperf tls %s process number :%d stop requested",
                       session->is_server ? "server" : "client", process_num);
                return 0;

            default:
                return -1;
        }
    }
    else
    {
        if(process_num != 0)
        {
            Report("\r\nError, nothing to stop");
        }
        Report("\r\nlist of running processes:");
        for(i=1;i<IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS;i++)
        {
            if(iperf_session[i].is_running)
            {
                Report("\r\n process num : %d is_server:%d proto:%s port:%u  (stop: iperf_stop -n %d)",
                        i,
                        iperf_session[i].is_server,
                        proto_name(iperf_session[i].proto),
                        (unsigned)iperf_session[i].lwipConfig.destOrLocalPortNumber,
                        i);
            }
        }
        return -1;
    }
}



/*!
 \brief          Prints Receive command help menu.

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.


 */
int32_t printTestIperfUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(TestIperf);
    Report(recvTestIperfUsage2Str);
    Report(descriptionStr);
    Report(recvTestIperfDetailsStr);

    Report(recvTestIperf_s_optionDetailsStr);
    Report(recvTestIperf_c_optionDetailsStr);
    Report(recvTestIperf_u_optionDetailsStr);
    Report(recvTestIperf_p_optionDetailsStr);
    Report(recvTestIperf_i_optionDetailsStr);
    Report(recvTestIperf_t_optionDetailsStr);
    Report(recvTestIperf_b_optionDetailsStr);
    Report(recvTestIperf_B_optionDetailsStr);
    Report(recvTestIperf_l_optionDetailsStr);
    Report(recvTestIperf_6_optionDetailsStr);
    Report(lineBreak);
    return (0);
}



int32_t printStopTestIperfUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(StopTestIperf);
    Report(recvStopTestIperfUsage2Str);
    Report(descriptionStr);
    Report(recvStopTestIperfDetailsStr);

    Report(recvStopTestIperf_n_optionDetailsStr);
    Report(lineBreak);
    return (0);
}


int32_t printTestTlsIperfUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(TestTlsIperf);
    Report(recvTestTlsIperfUsage2Str);
    Report(descriptionStr);
    Report(recvTestTlsIperfDetailsStr);

    Report(recvTestTlsIperf_s_optionDetailsStr);
    Report(recvTestTlsIperf_c_optionDetailsStr);
    Report(recvTestTlsIperf_p_optionDetailsStr);
    Report(recvTestTlsIperf_t_optionDetailsStr);
    Report(recvTestTlsIperf_i_optionDetailsStr);
    Report(recvTestTlsIperf_l_optionDetailsStr);
    Report(recvTestIperf_6_optionDetailsStr);
    Report(lineBreak);
    return (0);
}


int32_t cmdTestTlsIperfCallback(void *arg)
{
    RecvCmd_t params;
    int32_t   ret;

    memset(&params, 0, sizeof(RecvCmd_t));
    ret = ParseTlsIperfCmd(arg, &params);

    if (ret < 0)
    {
        return -1;
    }

    if (params.server)
    {
        return tls_iperf_server_start(&params);
    }
    else
    {
        return tls_iperf_client_start(&params);
    }
}



err_t lwiperf_tcp_poll(void *arg, struct tcp_pcb *tpcb)
{
    session_conn_t *session_con = (session_conn_t *)arg;
    //LWIP_ASSERT("iperflwip_client: pcb mismatch", session_con->conn_pcb_tcp == tpcb);
    LWIP_UNUSED_ARG(tpcb);
    if (++session_con->poll_count >= IPERF_LWIP_TCP_MAX_IDLE_SEC) {
        Report("\n\riperflwip: idle connection close connection  is server:%d port number:%d\n",
                session_con->lwipConfig.server,session_con->lwipConfig.destOrLocalPortNumber);
        iperflwip_tcp_stop(session_con,0);
        return ERR_OK; /* lwiperf_tcp_close frees conn */
    }

    if (!session_con->lwipConfig.server) {
        iperflwip_tcp_client_tx(session_con);
    }

    return ERR_OK;
}

void iperflwip_tcp_stop_from_cb(void *arg)
{
    iperflwip_tcp_stop(arg, 0);
}

void iperflwip_tcp_stop(void *arg, uint8_t isError)
{
    session_conn_t *session_con = arg;

    if(session_con->is_running == 0)
    {
        return;
    }

    /* TI: Request abort - report task will see this and exit */
    session_con->is_req_to_abort_test = 1;
    session_con->is_stop_due_traffic_error = isError;
    tcpip_callback(session_con->iperf_reportFunc,session_con);
}

void iperflwip_tcp_close(void *arg)
{

    session_conn_t *session_con = arg;
    uint8_t is_running = session_con->is_running;
    uint8_t isError = session_con->is_stop_due_traffic_error;

    session_con->is_running = false;

    if (session_con->report_task_handle != NULL)
    {
        xTimerStop(session_con->report_task_handle, 0);
        xTimerDelete(session_con->report_task_handle, 0);
        session_con->report_task_handle = NULL;
    }


    if ((!session_con->lwipConfig.server) && session_con->conn_pcb_tcp != NULL) {
        if(is_running && !isError)
        {
            /* Normal close: un-register callbacks and abort.
             * When isError=1 the tcp_err callback already fired, meaning lwIP
             * has freed the PCB - do NOT call any tcp_* functions on it. */
            tcp_arg(session_con->conn_pcb_tcp, NULL);
            tcp_sent(session_con->conn_pcb_tcp, NULL);
            tcp_poll(session_con->conn_pcb_tcp, NULL, 0U);
            tcp_err(session_con->conn_pcb_tcp, NULL);
            tcp_recv(session_con->conn_pcb_tcp, NULL);
            tcp_abort(session_con->conn_pcb_tcp);
        }
        session_con->conn_pcb_tcp = NULL;
    }
    else if (session_con->lwipConfig.server)
    {
        if(is_running)
        {
            /* no conn pcb, this is the listener pcb */
            if(session_con->conn_pcb_tcp != NULL)
            {
                if(!isError)
                {
                    tcp_close(session_con->conn_pcb_tcp);//close the listen pcb
                } 
                session_con->conn_pcb_tcp = NULL;
            }

            if(session_con->server_pcb)
            {
                tcp_close(session_con->server_pcb);//close the pcb
                session_con->server_pcb = NULL;
            }

        }
    }

}

// Error callback
void iperflwip_tcp_err(void *arg, err_t err)
{
    session_conn_t *session_con = (session_conn_t *)arg;
    if(session_con->is_running == 0)
    {
        //session is not running
        return;
    }
    Report("\n\riperflwip: tcp error num: %d, is server:%d port number:%d\n",
           err, session_con->lwipConfig.server,session_con->lwipConfig.destOrLocalPortNumber);
    iperflwip_tcp_stop(session_con, 1);
}

int8_t is_ip_addr_in_net_list(const ip_addr_t *addr)
{
    struct netif *netif = netif_list;

    while (netif)
    {
        if (ip4_addr_cmp(&(netif->ip_addr.u_addr.ip4), &(addr->u_addr.ip4)))
        {
            return 0;
        }
        netif = netif->next;
    }

    return -1;
}

void format_bps(double bps, char *output, size_t size)
{
    const char *units[] = {"bps", "Kbps", "Mbps", "Gbps", "Tbps"};
    int unit_index = 0;

    while (bps >= 1000 && unit_index < 4)
    {
        bps /= 1000;
        unit_index++;
    }

    uint32_t integer_part = (uint32_t)bps;
    uint32_t decimal_part = (uint32_t)((bps - integer_part) * 100 + 0.5);

    if (decimal_part >= 100)
    {
        integer_part++;
        decimal_part = 0;
    }

    snprintf(output, size, "%lu.%02lu %s",
             (unsigned long)integer_part,
             (unsigned long)decimal_part,
             units[unit_index]);
}


