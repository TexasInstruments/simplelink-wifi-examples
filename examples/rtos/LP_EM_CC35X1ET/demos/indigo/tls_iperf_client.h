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
#ifndef TLS_IPERF_CLIENT_H
#define TLS_IPERF_CLIENT_H

#ifdef CC35XX

#include <stdint.h>
#include "cmd_parser.h"

/* Default server port matching tls_server.py */
#define TLS_IPERF_DEFAULT_PORT      5555

/* Default send chunk size - one TCP MSS */
#define TLS_IPERF_SEND_BUF_SIZE     1460

/* Maximum send chunk size - one TCP MSS, matches lwIP iperf send unit */
#define TLS_IPERF_MAX_CHUNK_SIZE    1460

/* FreeRTOS task stack in 32-bit words (1 word = 4 bytes).
 * Measured peak: ~680 words (2.7 KB). 1024 words (4 KB) gives ~50% headroom. */
#define TLS_IPERF_TASK_STACK_WORDS  1024

/* FreeRTOS task priority - defined as IPERF_THREAD_PRIORITY in lwip_iperf_examples.h */

/* Abort the send loop if no bytes are successfully written for this long.
 * Mirrors TLS_IPERF_SERVER_IDLE_TIMEOUT_MS on the server side. */
#define TLS_IPERF_CLIENT_STALL_TIMEOUT_MS   10000

/*
 * Start a TLS iperf client session in a new FreeRTOS task.
 * args must point to a RecvCmd_t with:
 *   ipAddr.ipv4           - server IPv4 address (host byte order)
 *   destOrLocalPortNumber - server port (0 => TLS_IPERF_DEFAULT_PORT)
 *   timeout               - test duration in seconds (0 or >=99999 => run until stopped)
 *   period                - throughput report interval in seconds (0 = no periodic report)
 *   packetLength          - send chunk size in bytes (0 => TLS_IPERF_SEND_BUF_SIZE)
 *
 * Returns 0 on task creation success, -1 on failure.
 */
int32_t tls_iperf_client_start(void *args);

#endif /* CC35XX */

#endif /* TLS_IPERF_CLIENT_H */
