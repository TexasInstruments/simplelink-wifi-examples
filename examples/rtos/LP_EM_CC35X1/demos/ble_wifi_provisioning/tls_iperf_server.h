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
#ifndef TLS_IPERF_SERVER_H
#define TLS_IPERF_SERVER_H

#ifdef CC35XX

#include <stdint.h>
#include "cmd_parser.h"

/* Default listen port */
#define TLS_IPERF_SERVER_DEFAULT_PORT       5555

/* Receive buffer size - one TCP MSS, matches lwIP iperf and client send chunk */
#define TLS_IPERF_SERVER_RECV_BUF_SIZE      1460

/* Idle timeout: close the connection if no data received for this long (ms).
 * Catches clients that drop without sending a TLS close_notify or TCP FIN. */
#define TLS_IPERF_SERVER_IDLE_TIMEOUT_MS    10000

/* FreeRTOS task stack in 32-bit words (1 word = 4 bytes).
 * Measured peak: ~670 words (2.6 KB). 1024 words (4 KB) gives ~50% headroom. */
#define TLS_IPERF_SERVER_TASK_STACK_WORDS   1024

/* FreeRTOS task priority - below tcpip_thread (8) */
#define TLS_IPERF_SERVER_THREAD_PRIORITY      8

/*
 * Start a TLS iperf server session in a new FreeRTOS task.
 * args must point to a RecvCmd_t with:
 *   destOrLocalPortNumber - local port to listen on (0 => TLS_IPERF_SERVER_DEFAULT_PORT)
 *   period                - throughput report interval in seconds (0 = no periodic report)
 *   timeout               - max session duration in seconds (0 = unlimited)
 *
 * Returns 0 on task creation success, -1 on failure.
 */
int32_t tls_iperf_server_start(void *args);

#endif /* CC35XX */

#endif /* TLS_IPERF_SERVER_H */
