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

/*****************************************************************************/
/* HTTPS client using lwIP BSD sockets and mbedTLS.                          */
/* MBEDTLS_NET_C is disabled in this SDK, so we provide custom BIO callbacks */
/* that wrap lwip_send() / lwip_recv().                                      */
/*****************************************************************************/

#define MBEDTLS_CONFIG_FILE "config-hsm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>

#include "https_client.h"
#include "dns_if.h"
#include "uart_term.h"

/* lwIP BSD sockets */
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* mbedTLS */
#include <mbedtls/ssl.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/error.h>
#include <mbedtls/platform.h>

/* PSA Crypto for hardware RNG (entropy/ctr_drbg are disabled in config-hsm.h) */
#include <psa/crypto.h>

/* Thread-safe OS allocation wrappers (osi_dpl.c) */
#include "osi_kernel.h"

/*****************************************************************************/
/*                      Internal Defines                                     */
/*****************************************************************************/

#define HTTP_HEADER_BUF_SIZE    512
#define HTTP_RECV_BUF_SIZE      HTTPS_RECV_BUF_SIZE
#define HTTPS_SOCKET_TIMEOUT_MS 15000


/*****************************************************************************/
/*                      PSA RNG callback for mbedTLS                         */
/*****************************************************************************/

/*!
    \brief  RNG callback using PSA Crypto API (hardware-backed).
            Used as f_rng for mbedtls_ssl_conf_rng().
*/
static int psa_rng_callback(void *ctx, unsigned char *buf, size_t len)
{
    (void)ctx;
    psa_status_t status = psa_generate_random(buf, len);
    return (status == PSA_SUCCESS) ? 0 : -1;
}

/*****************************************************************************/
/*                      Custom BIO Callbacks for mbedTLS                     */
/*****************************************************************************/

static int tls_lwip_send(void *ctx, const unsigned char *buf, size_t len)
{
    int fd = *((int *)ctx);
    int ret = lwip_send(fd, buf, len, 0);
    if (ret < 0)
    {
        return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    }
    return ret;
}

static int tls_lwip_recv(void *ctx, unsigned char *buf, size_t len)
{
    int fd = *((int *)ctx);
    int ret = lwip_recv(fd, buf, len, 0);
    if (ret < 0)
    {
        return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    }
    if (ret == 0)
    {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }
    return ret;
}

/*****************************************************************************/
/*                      Internal Helper Functions                            */
/*****************************************************************************/

static int parseHttpResponseHeader(mbedtls_ssl_context *pSsl,
                                    int *pHttpStatus,
                                    uint32_t *pContentLength,
                                    uint8_t *pHeaderBuf,
                                    int headerBufSize)
{
    int headerLen = 0;
    int headerComplete = 0;
    int ret;

    *pHttpStatus = 0;
    *pContentLength = 0;

    while (!headerComplete && headerLen < headerBufSize - 1)
    {
        ret = mbedtls_ssl_read(pSsl, pHeaderBuf + headerLen, 1);
        if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE)
        {
            continue;
        }
        if (ret <= 0)
        {
            return -1;
        }
        headerLen += ret;

        if (headerLen >= 4 &&
            pHeaderBuf[headerLen - 4] == '\r' &&
            pHeaderBuf[headerLen - 3] == '\n' &&
            pHeaderBuf[headerLen - 2] == '\r' &&
            pHeaderBuf[headerLen - 1] == '\n')
        {
            headerComplete = 1;
        }
    }

    if (!headerComplete)
    {
        return -2;
    }

    pHeaderBuf[headerLen] = '\0';

    {
        char *pStatus = strstr((char *)pHeaderBuf, "HTTP/1.");
        if (pStatus)
        {
            pStatus = strchr(pStatus, ' ');
            if (pStatus)
            {
                *pHttpStatus = atoi(pStatus + 1);
            }
        }
    }

    {
        char *p = (char *)pHeaderBuf;
        while (*p)
        {
            if ((*p == 'C' || *p == 'c') &&
                strncasecmp(p, "Content-Length:", 15) == 0)
            {
                *pContentLength = (uint32_t)atoi(p + 15);
                break;
            }
            char *nl = strchr(p, '\n');
            if (!nl) break;
            p = nl + 1;
        }
    }

    return 0;
}

/*****************************************************************************/
/*                           API Implementation                              */
/*****************************************************************************/

int HTTPS_parseUrl(const char *pURL, char *pHost, uint16_t *pPort, char *pPath)
{
    const char *p;
    const char *hostStart;
    const char *hostEnd;
    int hostLen;

    if (!pURL || !pHost || !pPort || !pPath)
    {
        return -1;
    }

    if (strncmp(pURL, "https://", 8) == 0)
    {
        p = pURL + 8;
    }
    else if (strncmp(pURL, "http://", 7) == 0)
    {
        p = pURL + 7;
    }
    else
    {
        p = pURL;
    }

    hostStart = p;
    *pPort = HTTPS_DEFAULT_PORT;

    hostEnd = strpbrk(hostStart, ":/");
    if (!hostEnd)
    {
        hostLen = strlen(hostStart);
        if (hostLen >= HTTPS_MAX_HOST_LEN)
        {
            return -2;
        }
        memcpy(pHost, hostStart, hostLen);
        pHost[hostLen] = '\0';
        strcpy(pPath, "/");
        return 0;
    }

    hostLen = hostEnd - hostStart;
    if (hostLen >= HTTPS_MAX_HOST_LEN)
    {
        return -2;
    }
    memcpy(pHost, hostStart, hostLen);
    pHost[hostLen] = '\0';

    p = hostEnd;
    if (*p == ':')
    {
        p++;
        *pPort = (uint16_t)atoi(p);
        while (*p && *p != '/')
        {
            p++;
        }
    }

    if (*p == '/')
    {
        int pathLen = strlen(p);
        if (pathLen >= HTTPS_MAX_PATH_LEN)
        {
            return -3;
        }
        strcpy(pPath, p);
    }
    else
    {
        strcpy(pPath, "/");
    }

    return 0;
}

int HTTPS_download(const HTTPS_Request_t *pReq, HTTPS_Result_t *pResult)
{
    char *host = NULL;
    char *path = NULL;
    uint16_t port;
    ip_addr_t serverIp;
    int sockFd = -1;
    struct sockaddr_in serverAddr;
    int ret;

    /* Allocate mbedTLS contexts on heap - they are too large for the thread stack */
    mbedtls_ssl_context *pSsl = NULL;
    mbedtls_ssl_config *pConf = NULL;
    mbedtls_x509_crt *pCaCert = NULL;

    uint8_t *headerBuf = NULL;
    uint8_t *recvBuf = NULL;

    int httpStatus = 0;
    uint32_t contentLength = 0;
    uint32_t totalReceived = 0;

    if (!pReq || !pResult || !pReq->pURL || !pReq->dataCb)
    {
        return -1;
    }

    memset(pResult, 0, sizeof(HTTPS_Result_t));

    host = (char *)malloc(HTTPS_MAX_HOST_LEN);
    path = (char *)malloc(HTTPS_MAX_PATH_LEN);
    if (!host || !path)
    {
        UART_PRINT("[HTTPS] Memory allocation failed\n\r");
        ret = -4;
        goto cleanup_bufs;
    }

    ret = HTTPS_parseUrl(pReq->pURL, host, &port, path);
    if (ret != 0)
    {
        UART_PRINT("[HTTPS] URL parse error: %d\n\r", ret);
        ret = -2;
        goto cleanup_bufs;
    }

    UART_PRINT("[HTTPS] Host: %s, Port: %d, Path: %s\n\r", host, port, path);

    ret = DNS_IF_gethostbyname(host, &serverIp);
    if (ret != 0)
    {
        UART_PRINT("[HTTPS] DNS resolution failed: %d\n\r", ret);
        ret = -3;
        goto cleanup_bufs;
    }

    UART_PRINT("[HTTPS] Resolved IP: %s\n\r", ipaddr_ntoa(&serverIp));

    headerBuf = (uint8_t *)malloc(HTTP_HEADER_BUF_SIZE);
    recvBuf = (uint8_t *)malloc(HTTP_RECV_BUF_SIZE);
    pSsl = (mbedtls_ssl_context *)malloc(sizeof(mbedtls_ssl_context));
    pConf = (mbedtls_ssl_config *)malloc(sizeof(mbedtls_ssl_config));
    pCaCert = (mbedtls_x509_crt *)malloc(sizeof(mbedtls_x509_crt));
    if (!headerBuf || !recvBuf || !pSsl || !pConf || !pCaCert)
    {
        UART_PRINT("[HTTPS] Memory allocation failed\n\r");
        ret = -4;
        goto cleanup_bufs;
    }

    /* Use the thread-safe OS allocation wrappers (os_calloc/os_free)
       for all mbedTLS internal allocations.  Raw calloc/free are NOT
       thread-safe in this SDK — concurrent calls from the WiFi and
       TCPIP threads corrupt the C-library heap, which in turn
       overwrites lwIP's static heap metadata. */
    mbedtls_platform_set_calloc_free(os_calloc, os_free);

    mbedtls_ssl_init(pSsl);
    mbedtls_ssl_config_init(pConf);
    mbedtls_x509_crt_init(pCaCert);

    psa_crypto_init();

    ret = mbedtls_ssl_config_defaults(pConf,
                                       MBEDTLS_SSL_IS_CLIENT,
                                       MBEDTLS_SSL_TRANSPORT_STREAM,
                                       MBEDTLS_SSL_PRESET_DEFAULT);
    if (ret != 0)
    {
        UART_PRINT("[HTTPS] mbedtls_ssl_config_defaults failed: -0x%04x\n\r", -ret);
        ret = -6;
        goto cleanup_tls;
    }

    if (pReq->pCaCert && pReq->caCertLen > 0)
    {
        ret = mbedtls_x509_crt_parse(pCaCert, pReq->pCaCert, pReq->caCertLen);
        if (ret != 0)
        {
            UART_PRINT("[HTTPS] CA cert parse failed: -0x%04x\n\r", -ret);
            ret = -7;
            goto cleanup_tls;
        }
        mbedtls_ssl_conf_ca_chain(pConf, pCaCert, NULL);
        mbedtls_ssl_conf_authmode(pConf, MBEDTLS_SSL_VERIFY_REQUIRED);
    }
    else
    {
        mbedtls_ssl_conf_authmode(pConf, MBEDTLS_SSL_VERIFY_NONE);
    }

    mbedtls_ssl_conf_rng(pConf, psa_rng_callback, NULL);

    ret = mbedtls_ssl_setup(pSsl, pConf);
    if (ret != 0)
    {
        UART_PRINT("[HTTPS] mbedtls_ssl_setup failed: -0x%04x\n\r", -ret);
        ret = -8;
        goto cleanup_tls;
    }

    /* Set hostname for SNI and certificate verification.
       Skip for IP addresses — SNI is not valid for IPs (RFC 6066), and
       hostname matching against IP SANs requires specific SAN entries
       that test/playground certificates typically don't have. */
    {
        int isIpAddr = 1;
        const char *p = host;
        while (*p)
        {
            if ((*p < '0' || *p > '9') && *p != '.')
            {
                isIpAddr = 0;
                break;
            }
            p++;
        }
        if (!isIpAddr)
        {
            ret = mbedtls_ssl_set_hostname(pSsl, host);
            if (ret != 0)
            {
                UART_PRINT("[HTTPS] mbedtls_ssl_set_hostname failed: -0x%04x\n\r", -ret);
                ret = -9;
                goto cleanup_tls;
            }
        }
    }

    sockFd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockFd < 0)
    {
        UART_PRINT("[HTTPS] Socket creation failed\n\r");
        ret = -10;
        goto cleanup_tls;
    }

    /* Set socket timeouts so connect/handshake/read don't block forever */
    {
        struct timeval tv;
        tv.tv_sec  = HTTPS_SOCKET_TIMEOUT_MS / 1000;
        tv.tv_usec = (HTTPS_SOCKET_TIMEOUT_MS % 1000) * 1000;
        lwip_setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        lwip_setsockopt(sockFd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = lwip_htons(port);
    serverAddr.sin_addr.s_addr = ip4_addr_get_u32(ip_2_ip4(&serverIp));

    UART_PRINT("[HTTPS] Connecting to %s:%d...\n\r", host, port);

    ret = lwip_connect(sockFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (ret != 0)
    {
        UART_PRINT("[HTTPS] TCP connect failed: %d\n\r", ret);
        ret = -11;
        goto cleanup_socket;
    }

    UART_PRINT("[HTTPS] TCP connected, starting TLS handshake...\n\r");

    mbedtls_ssl_set_bio(pSsl, &sockFd, tls_lwip_send, tls_lwip_recv, NULL);

    while ((ret = mbedtls_ssl_handshake(pSsl)) != 0)
    {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE)
        {
            UART_PRINT("[HTTPS] TLS handshake failed: -0x%04x\n\r", -ret);
            if (ret == MBEDTLS_ERR_X509_CERT_VERIFY_FAILED)
            {
                uint32_t flags = mbedtls_ssl_get_verify_result(pSsl);
                UART_PRINT("[HTTPS] Verify flags: 0x%08x\n\r", flags);
            }
            ret = -12;
            goto cleanup_socket;
        }
    }

    UART_PRINT("[HTTPS] TLS handshake complete\n\r");

    {
        char reqBuf[HTTP_HEADER_BUF_SIZE];
        int reqLen = snprintf(reqBuf, sizeof(reqBuf),
            "GET %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Connection: close\r\n"
            "User-Agent: CC35xx-OTA/1.0\r\n"
            "\r\n",
            path, host);

        int written = 0;
        while (written < reqLen)
        {
            ret = mbedtls_ssl_write(pSsl, (unsigned char *)reqBuf + written, reqLen - written);
            if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE)
            {
                continue;
            }
            if (ret <= 0)
            {
                UART_PRINT("[HTTPS] Write failed: -0x%04x\n\r", -ret);
                ret = -13;
                goto cleanup_socket;
            }
            written += ret;
        }
    }

    UART_PRINT("[HTTPS] GET request sent, waiting for response...\n\r");

    ret = parseHttpResponseHeader(pSsl, &httpStatus, &contentLength,
                                   headerBuf, HTTP_HEADER_BUF_SIZE);
    if (ret != 0)
    {
        UART_PRINT("[HTTPS] Header parse error: %d\n\r", ret);
        ret = -14;
        goto cleanup_socket;
    }

    pResult->httpStatus = httpStatus;
    pResult->contentLength = contentLength;

    UART_PRINT("[HTTPS] HTTP %d, Content-Length: %u\n\r", httpStatus, contentLength);

    if (httpStatus != 200)
    {
        UART_PRINT("[HTTPS] Non-200 response, aborting\n\r");
        ret = -15;
        goto cleanup_socket;
    }

    {
        int readLen;
        int done = 0;

        while (!done)
        {
            readLen = mbedtls_ssl_read(pSsl, recvBuf, HTTP_RECV_BUF_SIZE);

            if (readLen == MBEDTLS_ERR_SSL_WANT_READ || readLen == MBEDTLS_ERR_SSL_WANT_WRITE)
            {
                continue;
            }

            if (readLen == 0 || readLen == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
            {
                done = 1;
                break;
            }

            if (readLen < 0)
            {
                UART_PRINT("[HTTPS] Read error: -0x%04x\n\r", -readLen);
                ret = -16;
                goto cleanup_socket;
            }

            ret = pReq->dataCb(recvBuf, (uint32_t)readLen, pReq->pUserCtx);
            if (ret != 0)
            {
                UART_PRINT("[HTTPS] Data callback returned error: %d\n\r", ret);
                ret = -17;
                goto cleanup_socket;
            }

            totalReceived += readLen;

            if (contentLength > 0 && totalReceived >= contentLength)
            {
                done = 1;
            }
        }
    }

    pResult->totalReceived = totalReceived;
    UART_PRINT("[HTTPS] Download complete: %u bytes received\n\r", totalReceived);
    ret = 0;

cleanup_socket:
    if (sockFd >= 0)
    {
        /* Ignore close_notify errors - server may have already closed */
        mbedtls_ssl_close_notify(pSsl);
        lwip_close(sockFd);
        sockFd = -1;
    }

cleanup_tls:
    mbedtls_x509_crt_free(pCaCert);
    mbedtls_ssl_free(pSsl);
    mbedtls_ssl_config_free(pConf);

cleanup_bufs:
    free(headerBuf);
    free(recvBuf);
    free(pSsl);
    free(pConf);
    free(pCaCert);
    free(host);
    free(path);

    return ret;
}
