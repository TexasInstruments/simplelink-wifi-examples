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
    \file   ota_server.h
    \brief  Server interaction over HTTPS — query for updates and download
            firmware images for the OTA example.

    This module handles querying an HTTPS server for available firmware
    updates (auto-discovery) and downloading selected update images,
    streaming them to the PSA FWU API.
*/

#ifndef OTA_SERVER_H
#define OTA_SERVER_H

#include <stdint.h>

/*****************************************************************************/
/*                           Defines                                         */
/*****************************************************************************/

#define OTA_MAX_UPDATES         6
#define OTA_MAX_FILENAME_LEN    64
#define OTA_MAX_COMPONENT_LEN   32
#define OTA_MAX_VERSION_LEN     20
#define OTA_MAX_URL_PATH_LEN    128

/*****************************************************************************/
/*                           Types                                           */
/*****************************************************************************/

/*!
    \brief  Parsed update entry from the server's /api/updates JSON response.
*/
typedef struct
{
    char     filename[OTA_MAX_FILENAME_LEN];
    char     component[OTA_MAX_COMPONENT_LEN];
    int      slot1Id;
    int      slot2Id;
    char     version[OTA_MAX_VERSION_LEN];
    uint32_t size;
    char     urlPath[OTA_MAX_URL_PATH_LEN];
} OtaUpdateEntry_t;

/*****************************************************************************/
/*                           API Functions                                   */
/*****************************************************************************/

/*!
    \brief  Query server for available updates and return parsed entries.

    Connects to serverBaseUrl/api/updates, parses the JSON response, and
    displays a compact summary table showing each available update with
    component name, primary version, server version, and a [NEW] marker
    if the server version is newer than the locally installed primary.

    \param[in]  pServerBaseUrl  Server base URL (e.g. "https://192.168.50.100:8443")
    \param[out] pEntries        Array to receive parsed update entries
    \param[in]  maxEntries      Maximum entries to parse

    \return Number of entries parsed (>= 0), or -1 on error
*/
int OTA_SERVER_queryUpdates(const char *pServerBaseUrl,
                            OtaUpdateEntry_t *pEntries, int maxEntries);

/*!
    \brief  Download a single update entry chosen by the user.

    Selects the target slot, prepares it, downloads the firmware image,
    and finalizes (CANDIDATE). The user must then call Install from the
    menu to stage all candidates, then Request reboot to apply.

    \param[in] pServerBaseUrl  Server base URL
    \param[in] pEntry          Pointer to the chosen OtaUpdateEntry_t

    \return 0 on success, -1 on error
*/
int OTA_SERVER_downloadUpdate(const char *pServerBaseUrl,
                              OtaUpdateEntry_t *pEntry);

#endif /* OTA_SERVER_H */
