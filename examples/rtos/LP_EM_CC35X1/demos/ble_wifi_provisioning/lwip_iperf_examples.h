#ifndef __IPERF_LWIP_EXAMPLES_H__
#define __IPERF_LWIP_EXAMPLES_H__

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "lwip/sockets.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_term.h"
#include "errors.h"
#include "FreeRTOSConfig.h"


#include "lwipopts.h"

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/inet.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"


#include "cmd_parser.h"
#include "osi_kernel.h"




#ifdef CC35XX

struct iperf_udp_hdr {
    s32_t id;       /* sequence number, negative -> FIN */
    u32_t tv_sec;   /* seconds part of client timestamp */
    u32_t tv_usec;  /* microseconds part of client timestamp */
};

typedef void (*reportFunc_t)(void* arg);

// State for a client
typedef struct
{
    uint32_t previous_time;
    uint32_t start_time;
    uint8_t process_num;
    bool is_running;
    bool is_server;
    bool is_udp;
    bool udp_server_first_packet_recv;
    reportFunc_t iperf_reportFunc;
    bool is_req_to_abort_test;
    bool is_stop_due_traffic_error;
    uint64_t total_bytes;
    uint32_t packet_count;//for udp client
    uint64_t bytes_per_period;
    uint64_t number_of_bytes_to_send_on_current_tx;
    timer_t os_timer;
    //TimerHandle_t os_timer;
    struct tcp_pcb *conn_pcb_tcp;
    struct udp_pcb *conn_pcb_udp;
    struct tcp_pcb *server_pcb;
    uint8_t poll_count;
    uint64_t target_Bps; //byte per second
    uint32_t actualTestdurationMs;
    uint32_t actualNumOfDurations;
    ip_addr_t dest_ip;//for client
    uint32_t dest_port;//for client
    ip_addr_t src_ip;//for server
    uint32_t src_port;//for server

    RecvCmd_t lwipConfig;
}session_conn_t;

#ifndef IPERF_LWIP_TCP_MAX_IDLE_SEC
#define IPERF_LWIP_TCP_MAX_IDLE_SEC    10U
#endif
#if IPERF_LWIP_TCP_MAX_IDLE_SEC > 255
#error IPERF_LWIP_TCP_MAX_IDLE_SEC must fit into an u8_t
#endif

#define IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS 5
#define LOCAL_UDP_CLIENT_PORT  5006
#define LOCAL_TCP_CLIENT_PORT  5006

//FreeRtos provides only pdMS_TO_TICK
#ifndef pdUS_TO_TICKS
#define pdUS_TO_TICKS(us) (((us) + (1000000 /configTICK_RATE_HZ -1))/ ( 1000000/ configTICK_RATE_HZ))
#endif

void iperflwip_tcp_stop(void *arg, uint8_t isError);

int32_t cmdStopTestIperfCallback(void *arg);

int32_t cmdTestIperfCallback(void *arg);

int32_t printTestIperfUsage(void *arg);

int32_t printStopTestIperfUsage(void *arg);

void iperflwip_tcp_close(void *arg);

int8_t is_ip_addr_in_net_list(const ip_addr_t *addr);


#endif

#endif
