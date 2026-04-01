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

/*!
    \file   ota_server.c
    \brief  Server interaction over HTTPS — query for updates and download
            firmware images for the OTA example.

    This module handles querying an HTTPS server for available firmware
    updates (auto-discovery) and downloading selected update images,
    streaming them to the PSA FWU API.

    Dependencies: https_client.h, ca_certificate.h, ota_fwu.h, psa_fwu.h,
                  uart_term.h
*/

/* Standard includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  /* calloc, free, malloc, atoi */

/* Application headers */
#include "ota_server.h"
#include "ota_fwu.h"
#include "https_client.h"
#include "uart_term.h"
#include "ca_certificate.h"

/* PSA Firmware Update API */
#include "ti/utils/FWU/psa_fwu.h"

/* ota_example.h for MAX_URL_LEN */
#include "ota_example.h"

/*****************************************************************************/
/*                           Defines                                         */
/*****************************************************************************/

#define OTA_QUERY_PATH          "/api/updates"
#define OTA_QUERY_BUF_SIZE      2048

/*****************************************************************************/
/*                           Structures (private)                            */
/*****************************************************************************/

/*!
    \brief  Context for the HTTPS data callback used during OTA download.
*/
typedef struct
{
    psa_fwu_component_t componentId;   /*!< PSA FWU component ID */
    uint32_t            manifestSize;  /*!< Size of manifest (TI_FWU_MANIFEST_SIZE) */
    uint8_t            *pManifestBuf;  /*!< Buffer to accumulate manifest bytes */
    uint32_t            manifestRecvd; /*!< Bytes of manifest received so far */
    int                 startCalled;   /*!< Flag: psa_fwu_start() called */
    uint32_t            imageOffset;   /*!< Current image offset for psa_fwu_write() */
    uint32_t            totalWritten;  /*!< Total bytes written via psa_fwu_write() */
    int                 error;         /*!< Error code if any step failed */
} OtaDownloadCtx_t;

/*!
    \brief  Context for the HTTPS data callback used during JSON query.
*/
typedef struct
{
    char    *pBuf;        /*!< Accumulation buffer */
    uint32_t bufSize;     /*!< Total buffer capacity */
    uint32_t received;    /*!< Bytes received so far */
    int      error;       /*!< Error flag */
} OtaQueryCtx_t;

/*****************************************************************************/
/*                      Query Callback                                       */
/*****************************************************************************/

/*!
    \brief  HTTPS data callback that accumulates JSON response into a buffer.

    \param[in] pData     Pointer to received data chunk
    \param[in] dataLen   Length of this chunk in bytes
    \param[in] pUserCtx  Pointer to OtaQueryCtx_t

    \return 0 on success, negative on error
*/
static int queryDataCallback(const uint8_t *pData, uint32_t dataLen,
                             void *pUserCtx)
{
    OtaQueryCtx_t *pCtx = (OtaQueryCtx_t *)pUserCtx;
    uint32_t space;

    if (pCtx->error != 0)
    {
        return pCtx->error;
    }

    /* Leave room for null terminator */
    space = pCtx->bufSize - 1 - pCtx->received;
    if (dataLen > space)
    {
        dataLen = space;
    }
    if (dataLen > 0)
    {
        memcpy(pCtx->pBuf + pCtx->received, pData, dataLen);
        pCtx->received += dataLen;
        pCtx->pBuf[pCtx->received] = '\0';
    }

    return 0;
}

/*****************************************************************************/
/*                      JSON Parser Helpers                                  */
/*****************************************************************************/

/*!
    \brief  Extract a JSON string value for a given key from a JSON object.

    Minimal parser using strstr(). Searches for "key":"value" patterns.

    \param[in]  pJson    JSON string to search
    \param[in]  pKey     Key name (without quotes)
    \param[out] pOut     Output buffer
    \param[in]  outSize  Size of output buffer

    \return 0 on success, -1 if key not found
*/
static int jsonGetString(const char *pJson, const char *pKey,
                         char *pOut, uint32_t outSize)
{
    char pattern[64];
    const char *p;
    const char *start;
    const char *end;
    uint32_t len;

    snprintf(pattern, sizeof(pattern), "\"%s\"", pKey);
    p = strstr(pJson, pattern);
    if (p == NULL)
    {
        return -1;
    }
    /* Skip past "key" */
    p += strlen(pattern);

    /* Skip optional whitespace and colon */
    while (*p == ' ' || *p == ':')
    {
        p++;
    }

    if (*p != '"')
    {
        return -1;
    }
    start = p + 1;
    end = strchr(start, '"');
    if (end == NULL)
    {
        return -1;
    }

    len = (uint32_t)(end - start);
    if (len >= outSize)
    {
        len = outSize - 1;
    }
    memcpy(pOut, start, len);
    pOut[len] = '\0';
    return 0;
}

/*!
    \brief  Extract a JSON integer value for a given key from a JSON object.

    \param[in]  pJson    JSON string to search
    \param[in]  pKey     Key name (without quotes)
    \param[out] pValue   Output integer

    \return 0 on success, -1 if key not found
*/
static int jsonGetInt(const char *pJson, const char *pKey, int *pValue)
{
    char pattern[64];
    const char *p;

    snprintf(pattern, sizeof(pattern), "\"%s\"", pKey);
    p = strstr(pJson, pattern);
    if (p == NULL)
    {
        return -1;
    }
    p += strlen(pattern);
    while (*p == ' ' || *p == ':')
    {
        p++;
    }
    *pValue = atoi(p);
    return 0;
}

/*!
    \brief  Parse the /api/updates JSON response into an array of entries.

    Expected format:
    {"updates":[{"file":"...","component":"...","slot1_id":N,"slot2_id":N,
                 "version":"M.m.p.b","size":N,"url":"/..."},...]}}

    \param[in]  pJson       JSON string
    \param[out] pEntries    Array of OtaUpdateEntry_t
    \param[in]  maxEntries  Maximum entries to parse

    \return Number of entries parsed, or -1 on error
*/
static int parseUpdateJson(const char *pJson, OtaUpdateEntry_t *pEntries,
                           int maxEntries)
{
    const char *p;
    int count = 0;
    int intVal;

    /* Find the "updates" array */
    p = strstr(pJson, "\"updates\"");
    if (p == NULL)
    {
        return -1;
    }
    p = strchr(p, '[');
    if (p == NULL)
    {
        return -1;
    }
    p++; /* skip '[' */

    while (count < maxEntries)
    {
        /* Find next object */
        const char *objStart = strchr(p, '{');
        const char *objEnd;

        if (objStart == NULL)
        {
            break;
        }
        objEnd = strchr(objStart, '}');
        if (objEnd == NULL)
        {
            break;
        }

        /* Parse fields from this object substring.
           We work on the full JSON from objStart since our parser
           finds the first matching key. */
        memset(&pEntries[count], 0, sizeof(OtaUpdateEntry_t));

        jsonGetString(objStart, "file", pEntries[count].filename,
                      OTA_MAX_FILENAME_LEN);
        jsonGetString(objStart, "component", pEntries[count].component,
                      OTA_MAX_COMPONENT_LEN);
        jsonGetString(objStart, "version", pEntries[count].version,
                      OTA_MAX_VERSION_LEN);
        jsonGetString(objStart, "url", pEntries[count].urlPath,
                      OTA_MAX_URL_PATH_LEN);

        if (jsonGetInt(objStart, "slot1_id", &intVal) == 0)
        {
            pEntries[count].slot1Id = intVal;
        }
        if (jsonGetInt(objStart, "slot2_id", &intVal) == 0)
        {
            pEntries[count].slot2Id = intVal;
        }
        if (jsonGetInt(objStart, "size", &intVal) == 0)
        {
            pEntries[count].size = (uint32_t)intVal;
        }

        count++;
        p = objEnd + 1;
    }

    return count;
}

/*****************************************************************************/
/*                      Version Comparison                                   */
/*****************************************************************************/

/*!
    \brief  Parse a version string "major.minor.patch.build" into components.

    \param[in]  pVerStr  Version string
    \param[out] pMajor   Major version
    \param[out] pMinor   Minor version
    \param[out] pPatch   Patch version
    \param[out] pBuild   Build number

    \return 0 on success, -1 on parse error
*/
static int parseVersionString(const char *pVerStr, uint8_t *pMajor,
                              uint8_t *pMinor, uint16_t *pPatch,
                              uint32_t *pBuild)
{
    int maj = 0, min = 0, pat = 0, bld = 0;
    const char *p = pVerStr;

    /* Manual parsing since sscanf may not be available */
    maj = atoi(p);
    p = strchr(p, '.');
    if (p == NULL) return -1;
    p++;
    min = atoi(p);
    p = strchr(p, '.');
    if (p == NULL) return -1;
    p++;
    pat = atoi(p);
    p = strchr(p, '.');
    if (p == NULL) return -1;
    p++;
    bld = atoi(p);

    *pMajor = (uint8_t)maj;
    *pMinor = (uint8_t)min;
    *pPatch = (uint16_t)pat;
    *pBuild = (uint32_t)bld;
    return 0;
}

/*!
    \brief  Compare server version against a local PSA FWU version.

    \param[in] pServerVersion  Version string from server (e.g. "1.2.0.45")
    \param[in] pLocalVersion   Local version from psa_fwu_query()

    \return 1 if server version is newer, 0 otherwise
*/
static int isServerVersionNewer(const char *pServerVersion,
                                const psa_fwu_image_version_t *pLocalVersion)
{
    uint8_t sMaj, sMin;
    uint16_t sPat;
    uint32_t sBld;

    if (parseVersionString(pServerVersion, &sMaj, &sMin, &sPat, &sBld) != 0)
    {
        return 0;
    }

    if (sMaj != pLocalVersion->major)
    {
        return (sMaj > pLocalVersion->major) ? 1 : 0;
    }
    if (sMin != pLocalVersion->minor)
    {
        return (sMin > pLocalVersion->minor) ? 1 : 0;
    }
    if (sPat != pLocalVersion->patch)
    {
        return (sPat > pLocalVersion->patch) ? 1 : 0;
    }
    if (sBld != pLocalVersion->build)
    {
        return (sBld > pLocalVersion->build) ? 1 : 0;
    }

    return 0; /* identical */
}

/*****************************************************************************/
/*                      OTA Download Callback                                */
/*****************************************************************************/

/*!
    \brief  HTTPS data callback that streams firmware data to PSA FWU API.

    The first TI_FWU_MANIFEST_SIZE bytes are buffered as the manifest and
    passed to psa_fwu_start(). Subsequent bytes are written via
    psa_fwu_write() with image_offset starting at TI_FWU_MANIFEST_SIZE.

    \param[in] pData     Pointer to received data chunk
    \param[in] dataLen   Length of this chunk in bytes
    \param[in] pUserCtx  Pointer to OtaDownloadCtx_t

    \return 0 on success, negative on error
*/
static int otaDataCallback(const uint8_t *pData, uint32_t dataLen, void *pUserCtx)
{
    OtaDownloadCtx_t *pCtx = (OtaDownloadCtx_t *)pUserCtx;
    uint32_t offset = 0;
    int32_t ret;

    if (pCtx->error != 0)
    {
        return pCtx->error;
    }

    /* Phase 1: Accumulate manifest bytes */
    if (!pCtx->startCalled)
    {
        uint32_t remaining = pCtx->manifestSize - pCtx->manifestRecvd;
        uint32_t toCopy = (dataLen < remaining) ? dataLen : remaining;

        memcpy(pCtx->pManifestBuf + pCtx->manifestRecvd, pData, toCopy);
        pCtx->manifestRecvd += toCopy;
        offset = toCopy;

        if (pCtx->manifestRecvd >= pCtx->manifestSize)
        {
            /* Got full manifest, call psa_fwu_start() */
            UART_PRINT("[OTA] Manifest received (%u bytes), calling psa_fwu_start()...\n\r",
                        pCtx->manifestSize);

            ret = psa_fwu_start(pCtx->componentId, pCtx->pManifestBuf,
                                pCtx->manifestSize);
            if (ret != PSA_SUCCESS)
            {
                UART_PRINT("[OTA] psa_fwu_start failed: %d\n\r", ret);
                pCtx->error = (int)ret;
                return -1;
            }

            pCtx->startCalled = 1;
            pCtx->imageOffset = pCtx->manifestSize;

            UART_PRINT("[OTA] psa_fwu_start() OK, streaming image data...\n\r");
        }
    }

    /* Phase 2: Write image data */
    if (pCtx->startCalled && offset < dataLen)
    {
        uint32_t writeLen = dataLen - offset;
        const uint8_t *writePtr = pData + offset;

        ret = psa_fwu_write(pCtx->componentId, pCtx->imageOffset,
                            writePtr, writeLen);
        if (ret != PSA_SUCCESS)
        {
            UART_PRINT("[OTA] psa_fwu_write failed at offset %u: %d\n\r",
                        pCtx->imageOffset, ret);
            pCtx->error = (int)ret;
            return -1;
        }

        pCtx->imageOffset += writeLen;
        pCtx->totalWritten += writeLen;

        /* Progress indication every 64KB */
        if ((pCtx->totalWritten % (64 * 1024)) < writeLen)
        {
            UART_PRINT("[OTA] Written %u bytes...\n\r",
                        pCtx->totalWritten + pCtx->manifestSize);
        }
    }

    return 0;
}

/*****************************************************************************/
/*                      Firmware Download Helper                             */
/*****************************************************************************/

/*!
    \brief  Download a firmware image via HTTPS and stream to PSA FWU.

    Sets up the OTA download context, performs HTTPS download with
    otaDataCallback, and handles errors.

    \param[in] componentId  PSA FWU component ID
    \param[in] pUrl         Full HTTPS URL of the firmware image

    \return 0 on success, negative on error
*/
static int performOtaDownload(psa_fwu_component_t componentId, const char *pUrl)
{
    OtaDownloadCtx_t otaCtx;
    HTTPS_Request_t httpsReq;
    HTTPS_Result_t httpsResult;
    int ret;

    UART_PRINT("[OTA] Starting firmware download for component %d (%s)...\n\r",
                componentId, OTA_FWU_componentName(componentId));

    memset(&otaCtx, 0, sizeof(otaCtx));
    otaCtx.componentId = componentId;
    otaCtx.manifestSize = TI_FWU_MANIFEST_SIZE;
    otaCtx.pManifestBuf = (uint8_t *)malloc(TI_FWU_MANIFEST_SIZE);
    if (!otaCtx.pManifestBuf)
    {
        UART_PRINT("[OTA] Failed to allocate manifest buffer\n\r");
        return -1;
    }

    memset(&httpsReq, 0, sizeof(httpsReq));
    httpsReq.pURL = pUrl;
    httpsReq.pCaCert = ca_certificate;
    httpsReq.caCertLen = sizeof(ca_certificate);
    httpsReq.dataCb = otaDataCallback;
    httpsReq.pUserCtx = &otaCtx;

    ret = HTTPS_download(&httpsReq, &httpsResult);

    free(otaCtx.pManifestBuf);

    if (ret != 0 || otaCtx.error != 0)
    {
        UART_PRINT("[OTA] Download failed (HTTPS: %d, OTA: %d)\n\r",
                    ret, otaCtx.error);
        if (otaCtx.startCalled)
        {
            psa_fwu_cancel(otaCtx.componentId);
        }
        return -1;
    }

    UART_PRINT("[OTA] Download complete: %u bytes total\n\r",
                httpsResult.totalReceived);
    return 0;
}

/*****************************************************************************/
/*                           API Functions                                   */
/*****************************************************************************/

int OTA_SERVER_queryUpdates(const char *pServerBaseUrl,
                            OtaUpdateEntry_t *pEntries, int maxEntries)
{
    char queryUrl[MAX_URL_LEN];
    char *pJsonBuf;
    OtaQueryCtx_t queryCtx;
    HTTPS_Request_t httpsReq;
    HTTPS_Result_t httpsResult;
    int entryCount;
    int ret;
    int i;

    /* Build the query URL */
    snprintf(queryUrl, sizeof(queryUrl), "%s%s", pServerBaseUrl, OTA_QUERY_PATH);
    UART_PRINT("[OTA] Querying server: %s\n\r", queryUrl);

    /* Allocate JSON response buffer */
    pJsonBuf = (char *)malloc(OTA_QUERY_BUF_SIZE);
    if (pJsonBuf == NULL)
    {
        UART_PRINT("[OTA] Failed to allocate query buffer\n\r");
        return -1;
    }

    /* Query the server */
    memset(&queryCtx, 0, sizeof(queryCtx));
    queryCtx.pBuf = pJsonBuf;
    queryCtx.bufSize = OTA_QUERY_BUF_SIZE;

    memset(&httpsReq, 0, sizeof(httpsReq));
    httpsReq.pURL = queryUrl;
    httpsReq.pCaCert = ca_certificate;
    httpsReq.caCertLen = sizeof(ca_certificate);
    httpsReq.dataCb = queryDataCallback;
    httpsReq.pUserCtx = &queryCtx;

    ret = HTTPS_download(&httpsReq, &httpsResult);
    if (ret != 0 || queryCtx.error != 0 || httpsResult.httpStatus != 200)
    {
        UART_PRINT("[OTA] Server query failed (HTTPS: %d, HTTP status: %d)\n\r",
                    ret, httpsResult.httpStatus);
        free(pJsonBuf);
        return -1;
    }

    UART_PRINT("[OTA] Received %u bytes from server\n\r", queryCtx.received);

    /* Parse JSON response */
    entryCount = parseUpdateJson(pJsonBuf, pEntries, maxEntries);
    free(pJsonBuf);

    if (entryCount <= 0)
    {
        UART_PRINT("[OTA] No update entries found in server response\n\r");
        return -1;
    }

    /* Display compact summary table */
    UART_PRINT("\n\rAvailable updates from server:\n\r");
    UART_PRINT(" %-3s %-20s %-16s %-12s %-8s\n\r",
                "#", "Component", "Active (slot)", "Server", "Status");
    UART_PRINT("--------------------------------------------------------------\n\r");

    for (i = 0; i < entryCount; i++)
    {
        psa_fwu_component_t targetId;
        psa_fwu_image_version_t primaryVer;
        char activeStr[OTA_MAX_VERSION_LEN + 8];
        const char *status;

        ret = OTA_FWU_selectTargetSlot(pEntries[i].slot1Id, pEntries[i].slot2Id,
                               &targetId, &primaryVer);
        if (ret != 0)
        {
            snprintf(activeStr, sizeof(activeStr), "N/A");
            status = "[ERR]";
        }
        else
        {
            /* The active slot is whichever one selectTargetSlot didn't pick.
               Even IDs are Slot_1, odd IDs are Slot_2. */
            int activeSlotId = (targetId == (psa_fwu_component_t)pEntries[i].slot2Id)
                               ? pEntries[i].slot1Id : pEntries[i].slot2Id;
            int slotNum = (activeSlotId % 2) + 1;

            snprintf(activeStr, sizeof(activeStr), "%d.%d.%d.%d (S%d)",
                     primaryVer.major, primaryVer.minor,
                     primaryVer.patch, primaryVer.build, slotNum);
            status = isServerVersionNewer(pEntries[i].version, &primaryVer)
                     ? "[NEW]" : "[OK]";
        }

        UART_PRINT(" [%d] %-20s %-16s %-12s %s\n\r",
                    i, pEntries[i].component, activeStr,
                    pEntries[i].version, status);
    }
    UART_PRINT("\n\r");

    return entryCount;
}

int OTA_SERVER_downloadUpdate(const char *pServerBaseUrl,
                              OtaUpdateEntry_t *pEntry)
{
    psa_fwu_component_t targetId;
    psa_fwu_image_version_t primaryVer;
    char fullUrl[MAX_URL_LEN];
    int ret;

    /* Find the target (non-primary) slot */
    ret = OTA_FWU_selectTargetSlot(pEntry->slot1Id, pEntry->slot2Id,
                           &targetId, &primaryVer);
    if (ret != 0)
    {
        UART_PRINT("[OTA] No suitable target slot for %s\n\r",
                    pEntry->component);
        return -1;
    }

    UART_PRINT("[OTA] Updating %s -> component %d (%s)\n\r",
                pEntry->component, targetId,
                OTA_FWU_componentName(targetId));

    /* Prepare the target slot */
    ret = OTA_FWU_prepareSlot(targetId);
    if (ret != 0)
    {
        UART_PRINT("[OTA] Failed to prepare component %d\n\r", targetId);
        return -1;
    }

    /* Build full download URL: serverBaseUrl + urlPath */
    snprintf(fullUrl, sizeof(fullUrl), "%s%s", pServerBaseUrl,
             pEntry->urlPath);

    /* Download and flash */
    ret = performOtaDownload(targetId, fullUrl);
    if (ret != 0)
    {
        UART_PRINT("[OTA] Download failed for %s\n\r", pEntry->filename);
        return -1;
    }

    /* Finalize this component */
    UART_PRINT("[OTA] Calling psa_fwu_finish() for component %d...\n\r",
                targetId);
    ret = psa_fwu_finish(targetId);
    if (ret != PSA_SUCCESS)
    {
        UART_PRINT("[OTA] psa_fwu_finish failed: %d\n\r", ret);
        psa_fwu_cancel(targetId);
        return -1;
    }
    UART_PRINT("[OTA] psa_fwu_finish() OK — image is CANDIDATE\n\r");
    UART_PRINT("[OTA] Use 'Install' to stage all candidates, then 'Request reboot'\n\r");

    return 0;
}
