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
#include <stdlib.h>
#include <string.h>
#include "osi_kernel.h"
#include "dns_if.h"
#include "lwip/dns.h"

#include "uart_term.h"

typedef struct
{
    ip_addr_t     *pIpAddr;
    OsiSyncObj_t  syncObj;
} DnsCtx_t;



void DnsCb_found(const char *name, const ip_addr_t *pIpAddr, void *arg)
{
    DnsCtx_t       *pCtx = (DnsCtx_t*)arg;
    *pCtx->pIpAddr = *pIpAddr;
    osi_SyncObjSignal(pCtx->syncObj);
}

int DNS_IF_gethostbyname(const char *pURL, ip_addr_t *pIpAddr)
{
    OsiReturnVal_e   rc;
    err_t            lwipRet;
    DnsCtx_t         ctx;
    ip_addr_t        ipAddr;

    rc = osi_SyncObjCreate(&ctx.syncObj);
    ASSERT_GENERAL(rc == 0);
    ctx.pIpAddr = &ipAddr;

    LOCK_TCPIP_CORE();
    lwipRet = dns_gethostbyname(pURL, ctx.pIpAddr, DnsCb_found, &ctx);
    UNLOCK_TCPIP_CORE();

    if(lwipRet == ERR_INPROGRESS)
    {
        lwipRet = osi_SyncObjWait(&ctx.syncObj, 10000); /* 10 Secs */
    }
    if(lwipRet == 0)
    {
        *pIpAddr = *ctx.pIpAddr;
        UART_PRINT("Found address %s for url:%s\r\n", ip_ntoa(pIpAddr), pURL);
    }
    osi_SyncObjDelete(&ctx.syncObj);

    return (lwipRet);
}

int DNS_IF_init()
{
    LOCK_TCPIP_CORE();
    dns_init();
    UNLOCK_TCPIP_CORE();
    return 0;
}

