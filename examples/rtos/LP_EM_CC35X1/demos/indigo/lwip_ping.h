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

#ifndef __LWIP_PING_H__
#define __LWIP_PING_H__

#ifdef CC35XX

#define PING_DEFAULT_DATA_SIZE          (56)
#define PING_MAX_DATA_SIZE              (1452)
#define PING_DEFAULT_INTERVAL_TIME_MS   (1000)   // 1 second
#define PING_DEFAULT_COUNT              (10)
#define PING_INTERVAL_MIN_TIME_MS       (100)    // 0.1 seconds
#define PING_INTERVAL_MAX_TIME_MS       (120000) // 120 seconds

/* Ping module flags, each flag is represented by a bit */
#define PING_PRINT_RESPONSES            (0x00000001)
#define PING_PRINT_REPORT               (0x00000002)

int32_t lwip_ping_start(PingParams_t *ping_params);
int32_t lwip_ping_stop(int8_t session_id);

typedef struct {
    ip_addr_t target_ip;
    uint16_t session_id;
    uint32_t packets_sent;
    uint32_t packets_received;
} ping_results_report_t;

#endif // CC35XX

#endif /* __LWIP_PING_H__ */