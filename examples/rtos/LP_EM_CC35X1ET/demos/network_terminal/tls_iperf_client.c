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
 * TLS iperf-like client for CC35xx.
 *
 * Connects to a TLS server, sends data as fast as possible for a configurable
 * duration, and reports throughput every period seconds - similar to the lwIP
 * iperf TCP client but using BSD sockets and mbedTLS instead of the raw lwIP
 * callback API.
 *
 * Sessions are tracked in the shared iperf_session[] array (proto = IPERF_PROTO_TLS)
 * so they appear in iperf_stop listings alongside TCP and UDP sessions and can be
 * stopped with iperf_stop -n <num>.
 *
 * Pair with tools/tls_server_client_remote_app/tls_server.py on the Windows side.
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
#include "tls_iperf_client.h"
#include "tls_iperf_mbedtls.h"

/* ------------------------------------------------------------------ */
/*  BIO callbacks - lwIP socket transport                             */
/* ------------------------------------------------------------------ */

/* BIO send: non-blocking after O_NONBLOCK is set on the socket post-handshake.
 * lwip_send with a non-blocking socket sends what fits and returns EAGAIN (not
 * WOULDBLOCK like MSG_DONTWAIT) when the TCP window is full, allowing partial
 * sends and letting mbedTLS retry for the remainder. */
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

/* CA cert compiled in from tools/playground_certificates/rootCA.pem */
#include "tls_iperf_ca_cert.h"

#ifdef CC35XX

extern session_conn_t iperf_session[];
extern unsigned char send_buffer[];

extern void format_bps(double bps, char *output, size_t size);

/* ------------------------------------------------------------------ */
/*  Main client task                                                   */
/* ------------------------------------------------------------------ */

static void tls_iperf_client_task(void *arg)
{
    session_conn_t *session = (session_conn_t *)arg;
    session->tls_task_handle = xTaskGetCurrentTaskHandle();
    RecvCmd_t      *cfg     = &session->lwipConfig;

    uint32_t port     = cfg->destOrLocalPortNumber ? cfg->destOrLocalPortNumber : TLS_IPERF_DEFAULT_PORT;
    uint8_t  endless  = (cfg->timeout == 0 || cfg->timeout >= 99999);
    uint32_t duration = endless ? 0 : cfg->timeout;
    uint32_t chunkSz  = cfg->packetLength          ? cfg->packetLength          : TLS_IPERF_SEND_BUF_SIZE;
    uint8_t  period   = cfg->period;

    char tag[24];
    snprintf(tag, sizeof(tag), "[TLS:%u] [%d]", (unsigned)port, session->process_num);

    if (chunkSz > TLS_IPERF_MAX_CHUNK_SIZE)
    {
        chunkSz = TLS_IPERF_MAX_CHUNK_SIZE;
        Report("\n\r%s chunk size capped to %u\n\r", tag, chunkSz);
    }

    tls_iperf_ctx_t *tlsCtx  = NULL;

    /* ---- TLS context setup ---- */

    tlsCtx = tls_iperf_ctx_alloc(TLS_IPERF_ENDPOINT_CLIENT);
    if (!tlsCtx)
    {
        Report("\n\r%s ERROR: tls_iperf_ctx_alloc failed (free heap: %u bytes)\n\r",
               tag, (unsigned)osi_GetFreeHeapSize());
        goto cleanup_session;
    }

    int ret = tls_iperf_set_ca_cert(tlsCtx, tls_iperf_ca_cert, tls_iperf_ca_cert_len);
    if (ret != 0)
    {
        Report("\n\r%s CA cert parse failed: -0x%04x (free heap: %u bytes)\n\r",
               tag, -ret, (unsigned)osi_GetFreeHeapSize());
        goto cleanup_tls;
    }

    /* ---- TCP connect ---- */

    int af_family = cfg->ipv6 ? AF_INET6 : AF_INET;
    int sockFd = lwip_socket(af_family, SOCK_STREAM, IPPROTO_TCP);
    if (sockFd < 0)
    {
        Report("\n\r%s socket() failed\n\r", tag);
        goto cleanup_tls;
    }

    struct timeval tv = { .tv_sec = 15, .tv_usec = 0 };
    lwip_setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    lwip_setsockopt(sockFd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_storage serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    char ipStr[INET6_ADDRSTRLEN] = {0};

    if (cfg->ipv6) {
        struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)&serverAddr;
        addr6->sin6_family = AF_INET6;
        addr6->sin6_port   = lwip_htons((uint16_t)port);
        memcpy(&addr6->sin6_addr, cfg->ipAddr.ipv6, 16);
        addr6->sin6_scope_id = network_netif_find_by_ip6(cfg->ipAddr.ipv6)->num + 1;
        inet_ntop(AF_INET6, &addr6->sin6_addr, ipStr, sizeof(ipStr));
    } else {
        struct sockaddr_in *addr4 = (struct sockaddr_in *)&serverAddr;
        addr4->sin_family      = AF_INET;
        addr4->sin_port        = lwip_htons((uint16_t)port);
        addr4->sin_addr.s_addr = htonl(cfg->ipAddr.ipv4);
        inet_ntop(AF_INET, &addr4->sin_addr, ipStr, sizeof(ipStr));
    }

    Report("\n\r%s Connecting to %s:%u ...\n\r", tag, ipStr, (unsigned)port);

    socklen_t addr_len = cfg->ipv6 ? (socklen_t)sizeof(struct sockaddr_in6)
                                   : (socklen_t)sizeof(struct sockaddr_in);
    ret = lwip_connect(sockFd, (struct sockaddr *)&serverAddr, addr_len);
    if (ret != 0)
    {
        Report("\n\r%s TCP connect failed: %d\n\r", tag, ret);
        goto cleanup_socket;
    }

    /* ---- TLS handshake ---- */

    Report("%s TCP connected, starting TLS handshake...\n\r", tag);

    ret = tls_iperf_handshake(tlsCtx, &sockFd, tls_bio_send, tls_bio_recv);
    if (ret != 0)
    {
        Report("\n\r%s TLS handshake failed: -0x%04x (free heap: %u bytes)\n\r",
               tag, -ret, (unsigned)osi_GetFreeHeapSize());
        uint32_t vflags = tls_iperf_get_verify_flags(tlsCtx);
        if (vflags)
        {
            Report("%s Cert verify flags: 0x%08x", tag, (unsigned)vflags);
            if (vflags & 0x001) Report(" EXPIRED");
            if (vflags & 0x200) Report(" NOT_YET_VALID (check date/time: use set_date_time)");
            if (vflags & 0x008) Report(" NOT_TRUSTED");
            if (vflags & 0x004) Report(" CN_MISMATCH");
            Report("\n\r");
        }
        goto cleanup_socket;
    }

    /* Switch to non-blocking so lwip_send returns EAGAIN instead of blocking
     * when the TCP window is full during the data transfer phase. */
    lwip_fcntl(sockFd, F_SETFL, lwip_fcntl(sockFd, F_GETFL, 0) | O_NONBLOCK);

    Report("%s TLS handshake complete. Cipher: %s. Sending for %u seconds... (free heap: %u bytes)\n\r",
           tag, tls_iperf_get_ciphersuite(tlsCtx), (unsigned)duration,
           (unsigned)osi_GetFreeHeapSize());

    /* ---- Send loop ---- */

    session->total_bytes      = 0;
    session->bytes_per_period = 0;

    uint32_t startMs       = osi_GetTimeMS();
    uint32_t periodStartMs = startMs;
    uint32_t durationMs    = endless ? 0 : duration * 1000;
    uint32_t periodMs      = period ? (uint32_t)period * 1000 : 0;
    char     rateStr[32];

    session->start_time    = startMs;
    session->previous_time = startMs;

    uint32_t lastTxMs = startMs;

    while (1)
    {
        uint32_t nowMs     = osi_GetTimeMS();
        uint32_t elapsedMs = nowMs - startMs;

        if (session->is_req_to_abort_test || (!endless && elapsedMs >= durationMs))
            break;

        ret = tls_iperf_write(tlsCtx, send_buffer, (int)chunkSz);
        if (tls_iperf_is_want_io(ret))
        {
            uint32_t stallMs = osi_GetTimeMS() - lastTxMs;

            if (stallMs >= TLS_IPERF_CLIENT_STALL_TIMEOUT_MS)
            {
                Report("\n\r%s No progress for %u ms, closing connection\n\r",
                       tag, TLS_IPERF_CLIENT_STALL_TIMEOUT_MS);
                break;
            }
            TickType_t delay;
            if (ret == TLS_IPERF_BIO_WANT_WRITE || ret == TLS_IPERF_BIO_WANT_READ)
            {
                /* TCP send buffer full: give lwIP time to receive ACKs and
                 * free pbufs. Stagger concurrent sessions to avoid re-colliding
                 * on the same tick. */
                delay = MAX(1, pdMS_TO_TICKS(10)) + (session->process_num & 3);
            }
            else
            {
                /* Heap pressure (ALLOC_FAILED): shorter stagger. */
                delay = (osi_GetFreeHeapSize() < HEAP_THRESHOLD_FOR_TX)
                        ? (1 + (session->process_num & 1)) : 1;
            }
            vTaskDelay(delay);
            continue;
        }
        if (ret <= 0)
        {
            if (!tls_iperf_is_peer_close(ret))
                Report("\n\r%s Write error: -0x%04x\n\r", tag, -ret);
            else
                Report("\n\r%s Server closed connection\n\r", tag);
            break;
        }

        lastTxMs = osi_GetTimeMS();
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
    Report("iperf TLS client :  %llu total bytes duration :%lu sec  %.2f Mbps\n\r",
           (unsigned long long)session->total_bytes, (unsigned long)seconds, bps / 1e6);

    tls_iperf_close_notify(tlsCtx);

cleanup_socket:
    lwip_close(sockFd);

cleanup_tls:
    tls_iperf_ctx_free(tlsCtx);

cleanup_session:


    /* Mark the session slot as free */
    session->is_running      = false;
    session->tls_task_handle = NULL;

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Public entry point                                                 */
/* ------------------------------------------------------------------ */

int32_t tls_iperf_client_start(void *args)
{
    int i;
    uint8_t found = 0;

    for (i = 1; i < IPERF_LWIP_MAX_NUM_OF_IPERF_SESSIONS + 1; i++)
    {
        if (!iperf_session[i].is_running)
        {
            found = 1;
            os_memset(&iperf_session[i], 0, sizeof(iperf_session[i]));
            iperf_session[i].is_server           = 0;
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

    Report("\r\nTo stop the TLS client process , iperf_stop -n %d ", i);

    TaskHandle_t handle = NULL;
    BaseType_t rc = xTaskCreate(tls_iperf_client_task,
                                "tls_iperf_cli",
                                TLS_IPERF_TASK_STACK_WORDS,
                                &iperf_session[i],
                                IPERF_THREAD_PRIORITY,
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
