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
//*****************************************************************************
// Includes
//*****************************************************************************
// Standard includes
#ifndef WIFI_SETTINGS_H
#define WIFI_SETTINGS_H

//*****************************************************************************
//                 WIFI IF INTRODUCTION
//*****************************************************************************
/* This module enables an easy integration of Wi-Fi to a SimpleLink Networking
 * framework.
 * It was designed for applications that use the Wi-Fi Station role only.
 */

//*****************************************************************************
//                WIFI IF USER SETTINGS
//*****************************************************************************

/*** WIFI CONNECTION DEFINITIONS */
#define WIFI_IF_STATIC_PROFILE_SUPPORT  (1)
#define WIFI_IF_PROVISIONING_SUPPORT    (0) /* PROVISIONING IS NOT SUPPORTED */

#if WIFI_IF_PROVISIONING_SUPPORT
#define PROVISIONING_MODE   0 
#endif


#if WIFI_IF_STATIC_PROFILE_SUPPORT
/* Static Profile setting 
 * Define AP_SSID and optionally AP_PASSWORD - to connect to local network
 * Hard-Coded Definition: update AP_SSID and AP_PASSWORD (NULL means OPEN, else is WPA2)
 */
#warning "Please set the credentials of the designated AP in wifi_settings.h file"

#define AP_SSID 	"ShlomiMobile"       // "network-name"
#define AP_SEC_TYPE	WLAN_SEC_TYPE_WPA_WPA2  // WLAN_SEC_TYPE_OPEN, WLAN_SEC_TYPE_WPA3
#define AP_PASSWORD     "Lihi1206"	        // "network-password"
#endif

/*** UI DEFINITIONS */

/* Define (if needed) the external handle for TI Driver's LED for wi-fi status:
 * off: disconnected, blinking: provisionig, on: connected
 * Comment the definition in case the auto control is not required */
#define WIFI_LED_INDEX CONFIG_LED_GREEN

#endif // WIFI_SETTINGS_H
