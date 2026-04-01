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
#ifndef MQTT_SETTINGS_H
#define MQTT_SETTINGS_H

//*****************************************************************************
//                MQTT IF USER SETTINGS
//*****************************************************************************

#define MQTT_STATIC_CLIENT_SETTINGS              (1)
#define MQTT_STATIC_CONNECTION_SETTINGS          (1)
//#define MQTT_SECURE_CLIENT

#if MQTT_STATIC_CLIENT_SETTINGS
#define MQTT_WILL_TOPIC             "cc32xx_will_topic"
#define MQTT_WILL_MSG               "will_msg_works"
#define MQTT_WILL_QOS               MQTT_QOS_2
#define MQTT_WILL_RETAIN            false

#define MQTT_CLIENT_CLIENTID        "sl-1234567545" // Use NULL to set the default client ID (SL-<MACADDR>)
#define MQTT_CLIENT_PASSWORD        NULL
#define MQTT_CLIENT_USERNAME        NULL
#define MQTT_CLIENT_KEEPALIVE       0
#define MQTT_CLIENT_CLEAN_CONNECT   true
#define MQTT_CLIENT_MQTT_V3_1       true
#define MQTT_CLIENT_BLOCKING_SEND   true
#endif // MQTT_STATIC_CLIENT_SETTING

#if MQTT_STATIC_CONNECTION_SETTINGS

#ifdef MQTT_SECURE_CLIENT
#else
#define MQTT_CONNECTION_FLAGS           MQTTCLIENT_NETCONN_URL
#define MQTT_CONNECTION_ADDRESS         "broker.hivemq.com"
//#define MQTT_CONNECTION_ADDRESS         "test.mosquitto.org"
#define MQTT_CONNECTION_PORT_NUMBER     1883
#endif

/* Define (if needed) the external handle for TI Driver's LED for MQTT status (for default server):
 * off: disconnected, on: connected
 * Comment the definition in case the auto LED control is not required
 */
#define MQTT_LED_INDEX                  CONFIG_LED_RED

#endif // MQTT_STATIC_CONNECTION_SETTINGS

#endif // MQTT_SETTINGS_H
