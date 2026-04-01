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
    \file   ota_wifi.h
    \brief  WiFi connection handling for the OTA example.

    This module manages WiFi initialization, scanning, and interactive
    connection for the OTA example application. It encapsulates all WiFi
    driver interaction, TCP/IP stack setup, and connection state.
*/

#ifndef OTA_WIFI_H
#define OTA_WIFI_H

/*****************************************************************************/
/*                           API Functions                                   */
/*****************************************************************************/

/*!
    \brief  One-time initialization of sync objects, TCP/IP stack, and DNS.

    Called once at startup. These resources persist across WiFi
    connect/disconnect cycles.

    \return 0 on success, negative on error
*/
int OTA_WIFI_initStack(void);

/*!
    \brief  Start WiFi driver, scan, and connect to a user-selected network.

    Can be called multiple times (after Wlan_Stop). One-time initialization
    must be done beforehand via OTA_WIFI_initStack().

    \return 0 on success, negative on error
*/
int OTA_WIFI_connect(void);

/*!
    \brief  Check whether WiFi is currently connected with an IP address.

    \return 1 if connected, 0 otherwise
*/
int OTA_WIFI_isConnected(void);

/*!
    \brief  Disconnect from WiFi and stop the WiFi driver.

    Calls Wlan_Disconnect, waits for the disconnect event, then calls
    Wlan_Stop and clears the connected state.
*/
void OTA_WIFI_disconnectAndStop(void);

#endif /* OTA_WIFI_H */
