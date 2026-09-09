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

/*
 * TLS iperf server for CC35xx.
 *
 * Listens for incoming TLS connections, receives data, and reports throughput
 * every period seconds - similar to the Python tls_server.py but running on
 * the CC35xx device itself.
 *
 * Sessions are tracked in the shared iperf_session[] array (proto = IPERF_PROTO_TLS)
 * so they appear in iperf_stop listings alongside TCP and UDP sessions and can be
 * stopped with iperf_stop -n <num>.
 *
 * Pair with tools/tls_server_client_remote_app/tls_client.py or any TLS client.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip6_addr.h"
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif

#include "osi_kernel.h"
#include "uart_term.h"
#include "lwip_iperf_examples.h"
#include "network_lwip.h"
#include "tls_iperf_server.h"
#include "tls_iperf_mbedtls.h"

/* ------------------------------------------------------------------ */
/*  BIO callbacks - lwIP socket transport                             */
/* ------------------------------------------------------------------ */

static int tls_bio_send(void *ctx, const unsigned char *buf, size_t len)
{
    int fd = *(int *)ctx;
    int ret = lwip_send(fd, buf, len, 0);
    if (ret < 0)
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? TLS_IPERF_BIO_WANT_WRITE : TLS_IPERF_BIO_ERR_IO;
    return ret;
}

static int tls_bio_recv(void *ctx, unsigned char *buf, size_t len)
{
    int fd  = *(int *)ctx;
    int ret = lwip_recv(fd, buf, len, 0);
    if (ret < 0)
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? TLS_IPERF_BIO_WANT_READ : TLS_IPERF_BIO_ERR_IO;
    if (ret == 0) return TLS_IPERF_BIO_ERR_IO;  /* TCP peer closed */
    return ret;
}

/* Server cert and key compiled in from tools/playground_certificates/ */
#include "tls_iperf_server_cert.h"
#include "tls_iperf_server_key.h"

#ifdef CC35XX

extern session_conn_t iperf_session[];

extern void format_bps(double bps, char *output, size_t size);

/* ------------------------------------------------------------------ */
/*  Connection handler task - spawned per accepted client             */
/* ------------------------------------------------------------------ */

typedef struct {
    session_conn_t *session;
    int clientFd;
    struct sockaddr_storage clientAddr;
    TaskHandle_t serverTaskHandle;
} client_handler_args_t;

static void tls_iperf_server_connection_task(void *arg)
{
    client_handler_args_t *args = (client_handler_args_t *)arg;
    session_conn_t *session       = args->session;
    int clientFd                  = args->clientFd;
    struct sockaddr_storage clientAddr = args->clientAddr;
    TaskHandle_t serverTaskHandle = args->serverTaskHandle;
    os_free(args);

    RecvCmd_t *cfg = &session->lwipConfig;
    uint8_t  period   = cfg->period;
    uint32_t timeout  = cfg->timeout;
    uint32_t port     = cfg->destOrLocalPortNumber ? cfg->destOrLocalPortNumber : TLS_IPERF_SERVER_DEFAULT_PORT;

    char tag[24];
    snprintf(tag, sizeof(tag), "[TLS:%u] [%d]", (unsigned)port, session->process_num);

    tls_iperf_ctx_t *tlsCtx  = NULL;
    unsigned char   *recv_buf = NULL;

    /* ---- TLS context setup ---- */

    tlsCtx = tls_iperf_ctx_alloc(TLS_IPERF_ENDPOINT_SERVER);
    if (!tlsCtx)
    {
        Report("\n\r%s ERROR: tls_iperf_ctx_alloc failed (free heap: %u bytes)\n\r",
               tag, (unsigned)osi_GetFreeHeapSize());
        lwip_close(clientFd);
        goto cleanup_tls;
    }

    int ret = tls_iperf_set_own_cert(tlsCtx,
                                     tls_iperf_server_cert, tls_iperf_server_cert_len,
                                     tls_iperf_server_key,  tls_iperf_server_key_len);
    if (ret != 0)
    {
        Report("\n\r%s tls_iperf_set_own_cert failed: -0x%04x (free heap: %u bytes)\n\r",
               tag, -ret, (unsigned)osi_GetFreeHeapSize());
        lwip_close(clientFd);
        goto cleanup_tls;
    }

    /* ---- TLS handshake ---- */

    char ipStr[INET6_ADDRSTRLEN] = {0};
    uint16_t clientPort = 0;
    if (clientAddr.ss_family == AF_INET6) {
        struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)&clientAddr;
        inet_ntop(AF_INET6, &addr6->sin6_addr, ipStr, sizeof(ipStr));
        clientPort = ntohs(addr6->sin6_port);
    } else {
        struct sockaddr_in *addr4 = (struct sockaddr_in *)&clientAddr;
        inet_ntop(AF_INET, &addr4->sin_addr, ipStr, sizeof(ipStr));
        clientPort = ntohs(addr4->sin_port);
    }
    Report("\n\r%s Client connected from %s:%u, starting TLS handshake...\n\r",
           tag, ipStr, clientPort);

    ret = tls_iperf_handshake(tlsCtx, &clientFd, tls_bio_send, tls_bio_recv);
    if (ret != 0)
    {
        Report("\n\r%s TLS handshake failed: -0x%04x (free heap: %u bytes)\n\r",
               tag, -ret, (unsigned)osi_GetFreeHeapSize());
        lwip_close(clientFd);
        goto cleanup_tls;
    }

    lwip_fcntl(clientFd, F_SETFL, lwip_fcntl(clientFd, F_GETFL, 0) | O_NONBLOCK);

    Report("%s TLS handshake complete. Cipher: %s. Receiving data...\n\r",
           tag, tls_iperf_get_ciphersuite(tlsCtx));

    /* ---- Receive loop ---- */

    recv_buf = os_malloc(TLS_IPERF_SERVER_RECV_BUF_SIZE);
    if (!recv_buf)
    {
        Report("\n\r%s ERROR: recv_buf alloc failed (free heap: %u bytes)\n\r",
               tag, (unsigned)osi_GetFreeHeapSize());
        lwip_close(clientFd);
        goto cleanup_tls;
    }

    session->total_bytes      = 0;
    session->bytes_per_period = 0;

    uint32_t startMs       = osi_GetTimeMS();
    uint32_t periodStartMs = startMs;
    uint32_t timeoutMs     = (timeout && timeout < 99999) ? (timeout * 1000) : 0;
    uint32_t periodMs      = period ? (uint32_t)period * 1000 : 0;
    char     rateStr[32];

    session->start_time    = startMs;
    session->previous_time = startMs;

    uint32_t lastRxMs = startMs;

    while (1)
    {
        uint32_t nowMs     = osi_GetTimeMS();
        uint32_t elapsedMs = nowMs - startMs;

        if (session->is_req_to_abort_test || (timeoutMs > 0 && elapsedMs >= timeoutMs))
        {
            break;
        }

        ret = tls_iperf_read(tlsCtx, recv_buf, TLS_IPERF_SERVER_RECV_BUF_SIZE);
        if (tls_iperf_is_want_io(ret))
        {
            if (nowMs - lastRxMs >= TLS_IPERF_SERVER_IDLE_TIMEOUT_MS)
            {
                Report("\n\r%s No data for %u ms, closing connection\n\r",
                       tag, TLS_IPERF_SERVER_IDLE_TIMEOUT_MS);
                break;
            }
            /* Under memory pressure stagger by process_num to desynchronize
             * concurrent sessions; otherwise use minimum delay. */
            TickType_t delay = (osi_GetFreeHeapSize() < HEAP_THRESHOLD_FOR_TX)
                               ? (1 + (session->process_num & 1)) : 1;
            vTaskDelay(delay);
            continue;
        }
        if (tls_iperf_is_peer_close(ret))
        {
            Report("\n\r%s Client closed connection\n\r", tag);
            break;
        }
        if (ret < 0)
        {
            Report("\n\r%s Read error: -0x%04x\n\r", tag, -ret);
            break;
        }

        lastRxMs = nowMs;
        session->total_bytes      += (uint64_t)ret;
        session->bytes_per_period += (uint64_t)ret;

        if (periodMs > 0)
        {
            uint32_t periodElapsed = nowMs - periodStartMs;
            if (periodElapsed >= periodMs)
            {
                double bps = ((double)session->bytes_per_period * 8.0)
                             / ((double)periodElapsed / 1000.0);
                format_bps(bps, rateStr, sizeof(rateStr));
                Report("\n\r%s %s", tag, rateStr);
                session->bytes_per_period = 0;
                periodStartMs = nowMs;
                session->previous_time = nowMs;
            }
        }
    }

    /* ---- Final report ---- */

    uint32_t endMs   = osi_GetTimeMS();
    double   seconds = (double)(endMs - startMs) / 1000.0;
    double   bps     = (seconds > 0.0) ? ((double)session->total_bytes * 8.0 / seconds) : 0.0;
    format_bps(bps, rateStr, sizeof(rateStr));

    Report("\n\r%s Test complete  %llu bytes  %.2f sec  %s\n\r",
           tag, (unsigned long long)session->total_bytes, seconds, rateStr);
    Report("iperf TLS server :  %llu total bytes duration :%lu sec  %.2f Mbps\n\r",
           (unsigned long long)session->total_bytes, (unsigned long)seconds, bps / 1e6);

    tls_iperf_close_notify(tlsCtx);
    lwip_close(clientFd);

cleanup_tls:
    if (recv_buf)
        os_free(recv_buf);
    tls_iperf_ctx_free(tlsCtx);



    /* Signal server task that this connection is done so it can clean up */
    session->is_running = false;
    xTaskNotifyGive(serverTaskHandle);

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Main server task - listens and spawns handlers per connection     */
/* ------------------------------------------------------------------ */

static void tls_iperf_server_task(void *arg)
{
    session_conn_t *session = (session_conn_t *)arg;
    session->tls_task_handle = xTaskGetCurrentTaskHandle();
    RecvCmd_t      *cfg     = &session->lwipConfig;

    uint32_t port = cfg->destOrLocalPortNumber ? cfg->destOrLocalPortNumber : TLS_IPERF_SERVER_DEFAULT_PORT;

    char tag[24];
    snprintf(tag, sizeof(tag), "[TLS:%u] [%d]", (unsigned)port, session->process_num);

    /* ---- TCP listen ---- */

    int af_family = cfg->ipv6 ? AF_INET6 : AF_INET;
    int listenFd = lwip_socket(af_family, SOCK_STREAM, IPPROTO_TCP);
    if (listenFd < 0)
    {
        Report("\n\r%s socket() failed\n\r", tag);
        goto cleanup_session;
    }

    int reuse = 1;
    lwip_setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_storage serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    if (cfg->ipv6) {
        struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)&serverAddr;
        addr6->sin6_family = AF_INET6;
        addr6->sin6_port   = lwip_htons((uint16_t)port);
        memcpy(&addr6->sin6_addr, cfg->ipAddr.ipv6, 16);
        /* Find which netif owns this bind address for the correct scope_id.
         * Hardcoding STA would break binding to an AP link-local address. */
        addr6->sin6_scope_id = 0;
        if (memcmp(cfg->ipAddr.ipv6, "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", 16) != 0) {
            struct netif *n;
            int idx;
            for (n = netif_list; n != NULL; n = n->next) {
                for (idx = 0; idx < LWIP_IPV6_NUM_ADDRESSES; idx++) {
                    if (memcmp(n->ip6_addr[idx].u_addr.ip6.addr, cfg->ipAddr.ipv6, 16) == 0) {
                        addr6->sin6_scope_id = n->num + 1;
                        goto scope_found;
                    }
                }
            }
            addr6->sin6_scope_id = ((struct netif *)network_get_sta_if())->num + 1;
            scope_found:;
        }
    } else {
        struct sockaddr_in *addr4 = (struct sockaddr_in *)&serverAddr;
        addr4->sin_family      = AF_INET;
        addr4->sin_port        = lwip_htons((uint16_t)port);
        /* Use -B address; 0 means bind to any IPv4 address (INADDR_ANY) */
        addr4->sin_addr.s_addr = htonl(cfg->ipAddr.ipv4);
    }

    socklen_t bind_len = cfg->ipv6 ? (socklen_t)sizeof(struct sockaddr_in6)
                                   : (socklen_t)sizeof(struct sockaddr_in);
    int ret = lwip_bind(listenFd, (struct sockaddr *)&serverAddr, bind_len);
    if (ret != 0)
    {
        Report("\n\r%s bind() failed on port %u\n\r", tag, (unsigned)port);
        goto cleanup_socket;
    }

    ret = lwip_listen(listenFd, 5);
    if (ret != 0)
    {
        Report("\n\r%s listen() failed\n\r", tag);
        goto cleanup_socket;
    }

    Report("\n\r%s TLS Server is listening on port %u\n\r", tag, (unsigned)port);

    /* Accept loop - spawn a new handler task per connection */
    while (!session->is_req_to_abort_test)
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(listenFd, &readfds);
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        int sel = lwip_select(listenFd + 1, &readfds, NULL, NULL, &tv);
        if (sel <= 0)
        {
            /* timeout or error - check abort flag and retry */
            continue;
        }

        struct sockaddr_storage clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);

        int clientFd = lwip_accept(listenFd, (struct sockaddr *)&clientAddr, &clientAddrLen);
        if (clientFd < 0)
        {
            if (session->is_req_to_abort_test)
                break;
            Report("\n\r%s accept() error, continuing...\n\r", tag);
            continue;
        }

        client_handler_args_t *handlerArgs = (client_handler_args_t *)os_malloc(sizeof(client_handler_args_t));
        if (!handlerArgs)
        {
            Report("\n\r%s Out of memory for connection handler\n\r", tag);
            lwip_close(clientFd);
            continue;
        }

        handlerArgs->session          = session;
        handlerArgs->clientFd         = clientFd;
        handlerArgs->clientAddr       = clientAddr;
        handlerArgs->serverTaskHandle = xTaskGetCurrentTaskHandle();

        TaskHandle_t handle = NULL;
        BaseType_t rc = xTaskCreate(tls_iperf_server_connection_task,
                                    "tls_iperf_conn",
                                    TLS_IPERF_SERVER_TASK_STACK_WORDS,
                                    handlerArgs,
                                    TLS_IPERF_SERVER_THREAD_PRIORITY,
                                    &handle);
        if (rc != pdPASS)
        {
            Report("\n\r%s xTaskCreate failed for connection handler\n\r", tag);
            lwip_close(clientFd);
            os_free(handlerArgs);
            continue;
        }

        /* Wait for the connection handler to finish, then exit.
         * The handler sets is_running=false and notifies us when done. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        break;
    }

    Report("\n\r%s Server stopping\n\r", tag);

cleanup_socket:
    lwip_close(listenFd);

cleanup_session:
    session->tls_task_handle = NULL;
    /* is_running already cleared by connection handler; clear again defensively
     * (covers bind/listen failures where the handler was never spawned) */
    session->is_running = false;

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Public entry point                                                 */
/* ------------------------------------------------------------------ */

int32_t tls_iperf_server_start(void *args)
{
    int i;
    uint8_t found = 0;

    /* Reject duplicate port before allocating a session slot */
    uint32_t reqPort = ((RecvCmd_t *)args)->destOrLocalPortNumber;
    if (!reqPort) reqPort = TLS_IPERF_SERVER_DEFAULT_PORT;
    for (i = 1; i < IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS + 1; i++)
    {
        if (iperf_session[i].is_running && iperf_session[i].is_server &&
            iperf_session[i].proto == IPERF_PROTO_TLS)
        {
            uint32_t runningPort = iperf_session[i].lwipConfig.destOrLocalPortNumber;
            if (!runningPort) runningPort = TLS_IPERF_SERVER_DEFAULT_PORT;
            if (runningPort == reqPort)
            {
                Report("\n\r[TLS iperf] ERROR: port %u already in use by session %d\n\r",
                       reqPort, i);
                return -1;
            }
        }
    }

    for (i = 1; i < IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS + 1; i++)
    {
        if (!iperf_session[i].is_running)
        {
            found = 1;
            os_memset(&iperf_session[i], 0, sizeof(iperf_session[i]));
            iperf_session[i].is_server           = 1;
            iperf_session[i].proto               = IPERF_PROTO_TLS;
            iperf_session[i].process_num         = i;
            iperf_session[i].is_req_to_abort_test = 0;
            iperf_session[i].is_running          = true;
            os_memcpy(&iperf_session[i].lwipConfig, args, sizeof(RecvCmd_t));
            break;
        }
    }

    if (!found)
    {
        Report("\n\r[TLS iperf] ERROR: max sessions reached (%d)\n\r",
               IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS);
        return -1;
    }

    Report("\r\nTo stop the TLS server process , iperf_stop -n %d ", i);

    TaskHandle_t handle = NULL;
    BaseType_t rc = xTaskCreate(tls_iperf_server_task,
                                "tls_iperf_srv",
                                TLS_IPERF_SERVER_TASK_STACK_WORDS,
                                &iperf_session[i],
                                TLS_IPERF_SERVER_THREAD_PRIORITY,
                                &handle);
    if (rc != pdPASS)
    {
        Report("\n\r[TLS iperf] ERROR: xTaskCreate failed\n\r");
        iperf_session[i].is_running = false;
        return -1;
    }

    return i;
}

#endif /* CC35XX */
