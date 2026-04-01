/*
 * Copyright (c) 2025, Texas Instruments Incorporated
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
#ifndef HTTPS_CLIENT_H
#define HTTPS_CLIENT_H

#include <stdint.h>
#include <stddef.h>

/*****************************************************************************/
/*                           Defines                                         */
/*****************************************************************************/

#define HTTPS_MAX_HOST_LEN      128
#define HTTPS_MAX_PATH_LEN      256
#define HTTPS_DEFAULT_PORT      443
#define HTTPS_RECV_BUF_SIZE     4096

/*****************************************************************************/
/*                           Types                                           */
/*****************************************************************************/

typedef int (*https_data_cb_t)(const uint8_t *pData, uint32_t dataLen, void *pUserCtx);

typedef struct
{
    const char    *pURL;
    const uint8_t *pCaCert;
    uint32_t       caCertLen;
    https_data_cb_t dataCb;
    void          *pUserCtx;
} HTTPS_Request_t;

typedef struct
{
    int      httpStatus;
    uint32_t contentLength;
    uint32_t totalReceived;
} HTTPS_Result_t;

/*****************************************************************************/
/*                           API Functions                                    */
/*****************************************************************************/

int HTTPS_parseUrl(const char *pURL, char *pHost, uint16_t *pPort, char *pPath);
int HTTPS_download(const HTTPS_Request_t *pReq, HTTPS_Result_t *pResult);

#endif /* HTTPS_CLIENT_H */
