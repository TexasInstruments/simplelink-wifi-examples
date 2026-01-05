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
#ifndef MQTT_IF_H_
#define MQTT_IF_H_

#include "osi_kernel.h"
#include "ti_drivers_config.h"

/* The following are example of client and connection structures settings.
 * Note: this is needed only in case the static configuration is not used
 *
 * MQTTClient_Will mqttWillParams =
 * {
 *     MQTT_WILL_TOPIC,    // will topic
 *     MQTT_WILL_MSG,      // will message
 *     MQTT_WILL_QOS,      // will QoS
 *     MQTT_WILL_RETAIN    // retain flag
 * };

 * char ClientId[13] = "clientId123";

 * MQTT_IF_ClientParams_t mqttClientParams =
 * {
 *     ClientId,                  // client ID
 *    MQTT_CLIENT_USERNAME,      // user name
 *    MQTT_CLIENT_PASSWORD,      // password
 *    MQTT_CLIENT_KEEPALIVE,     // keep-alive time
 *    MQTT_CLIENT_CLEAN_CONNECT, // clean connect flag
 *    MQTT_CLIENT_MQTT_V3_1,     // true = 3.1, false = 3.1.1
 *    MQTT_CLIENT_BLOCKING_SEND, // blocking send flag
 *     &mqttWillParams            // will parameters
 * };
 *
 * char *MQTTClient_secureFiles[1] = {"mosquitto.org.der"};
 *
 * #ifdef MQTT_SECURE_CLIENT
 * #else
 * MQTTClient_ConnParams mqttConnParams =
 * {
 *    MQTT_CONNECTION_FLAGS,         // connection flags
 *    MQTT_CONNECTION_ADDRESS,       // server address
 *    MQTT_CONNECTION_PORT_NUMBER,   // port number of MQTT server
 *    0,                             // method for secure socket
 *    0,                             // cipher for secure socket
 *    0,                             // number of files for secure connection
 *    NULL                           // secure files
 * };
 * #endif
 */

#ifndef MQTT_QOS_0
#define MQTT_QOS_0  (0)
#endif
#ifndef MQTT_QOS_1
#define MQTT_QOS_1  (1)
#endif
#ifndef MQTT_QOS_2
#define MQTT_QOS_2  (2)
#endif

#define MQTTCLIENT_NETCONN_IP4                                      0x00   /**< Assert for IPv4 connection                  */
#define MQTTCLIENT_NETCONN_TCP                                      0x01   /**< Server address is an URL and not IP address */
#define MQTTCLIENT_NETCONN_UDP                                      0x02   /**< Server address is an URL and not IP address */
#define MQTTCLIENT_NETCONN_IP6                                      0x04   /**< Assert for IPv6 connection, otherwise  IPv4 */
#define MQTTCLIENT_NETCONN_URL                                      0x08   /**< Server address is an URL and not IP address */
#define MQTTCLIENT_NETCONN_SEC                                      0x10   /**< Connection to server  must  be secure (TLS) */
#define MQTTCLIENT_NETCONN_SKIP_DOMAIN_NAME_VERIFICATION            0x20  /**< Assert to skip domain name verification*/
#define MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION    0x40  /**< Assert to skip certificate catalog */
#define  MQTTCLIENT_NETCONN_SKIP_DATE_VERIFICATION                  0x80  /**< Assert to skip date verification */

#include "mqtt_settings.h"

typedef void * MQTTClient_Handle;

typedef struct MQTTClient_Will
{
    const char *willTopic; /**< Will Topic   */
    const char *willMsg;   /**< Will message */
    int8_t      willQos;   /**< Will Qos     */
    bool        retain;    /**< Retain Flag  */
} MQTTClient_Will;

typedef struct MQTTClient_ConnParams
{
    uint32_t netconnFlags; /**< Enumerate connection type  */
    const char *serverAddr; /**< Server Address: URL or IP  */
    uint16_t port; /**< Port number of MQTT server */
    uint8_t method; /**< Method to tcp secured socket */
    uint32_t cipher; /**< Cipher to tcp secured socket */
    uint32_t nFiles; /**< Number of files for secure transfer */
    char * const *secureFiles; /* SL needs 4 files*/
} MQTTClient_ConnParams;


#define MQTTCLIENT_MAX_SIMULTANEOUS_SUB_TOPICS 4

// MQTT events for event callback
enum
{
    MQTT_EVENT_CONNACK,
    MQTT_EVENT_SUBACK,
    MQTT_EVENT_PUBACK,
    MQTT_EVENT_UNSUBACK,
    MQTT_EVENT_SERVER_DISCONNECT,
    MQTT_EVENT_CLIENT_DISCONNECT,
    MQTT_EVENT_DESTROY,
    MQTT_EVENT_MAX
};

typedef void (*MQTT_IF_TopicCallback_f)(char* topic, uint8_t* payload, uint16_t len, uint8_t qos);
typedef void (*MQTT_IF_EventCallback_f)(MQTTClient_Handle hConn, int32_t event, void *args);

typedef struct MQTT_IF_ClientParams
{
    char                *clientID;
    char                *username;
    char                *password;
    uint16_t            keepaliveTime;
    bool                cleanConnect;
    bool                mqttMode31;     // false 3.1.1 (default) : true 3.1
    bool                blockingSend;
    MQTTClient_Will     *willParams;
} MQTT_IF_ClientParams_t;

typedef struct MQTT_IF_initParams
{
    unsigned int stackSize;
    uint8_t threadPriority;
} MQTT_IF_InitParams_t;

/**
 \brief  Create the infrastructure for the MQTT_IF (mqtt interface) module

 \param[in] initParams: parameters to set stack size and thread priority for module

 \return Success 0 or Failure -1

 \sa MQTT_IF_Deinit()
 */
int MQTT_IF_init(MQTT_IF_InitParams_t initParams);

/**
 \brief  Destroys the infrastructure created from calling MQTT_IF_Init

 \param[in] mqttClientHandle: handle for the mqtt client module instance

 \return Success 0 or Failure -1

 \sa MQTT_IF_Init()
 */
int MQTT_IF_deinit();


/**
 \brief  Create Client context

 \param[in] mqttClientParams: params for the mqtt client parameters, or NULL for using
                              the static client profile (as defined in mqtt_settings.h)
 \param[in] mqttCB: the callback that is registered when for MQTT operation events (e.g. CONNACK, SUBACK...)

 \return Success 0 or Failure -1

 \sa MQTT_IF_Disconnect()
 */
MQTTClient_Handle MQTT_IF_clientCreate(MQTT_IF_ClientParams_t *pClientParams, MQTT_IF_EventCallback_f mqttCB);

/**
 \brief  Destroy the Client context

 \param[in] mqttClientHandle: handle for the mqtt client module instance

 \return Success 0 or Failure -1

 \sa MQTT_IF_Connect()
 */
int MQTT_IF_clientDestroy(MQTTClient_Handle hClient);

/**
 \brief  Connect will set all client and connection parameters and initiate a connection to the broker.
         It will also register the event callback defined in the user's application and the create an
         internal context thread for the internal MQTT library

 \param[in] mqttClientHandle: handle for the mqtt client module instance
 \param[in] mqttConnParams: params for the mqtt connection parameters, or NULL for using
                              the static connection settings (as defined in mqtt_settings.h)

 \return Success 0 or Failure -1

 \sa MQTT_IF_Disconnect()
 */
int MQTT_IF_clientConnect(MQTTClient_Handle hClient, MQTTClient_ConnParams *pConnParams);

/**
 \brief  Instructs the internal MQTT library to close the MQTT connection to the broker

 \param[in] mqttClientHandle: handle for the mqtt client module instance

 \return Success 0 or Failure -1

 \sa MQTT_IF_Connect()
 */
int MQTT_IF_clientDisconnect(MQTTClient_Handle hClient);

/**
 \brief  Subscribes to the topics specified by the caller in subscriptionInfo. Topic subscriptions are agnostic to the
 \ connection status. Meaning if you subscribe to topics then disconnect, the MQTT_IF module will still hold the topics
 \ so on re-connect the original topics are subscribed to again automatically.

 \param[in] mqttClientHandle: handle for the mqtt client module instance
 \param[in] subscriptionInfo: data structure containing all the data required to subscribe
 \param[in] numOfTopics: number of topics stored in subscriptionInfo

 \return Success 0 or Failure -1

 \sa MQTT_IF_Unsubscribe()
 */
int MQTT_IF_subscribe(MQTTClient_Handle hClient, char* topic, unsigned int qos, MQTT_IF_TopicCallback_f topicCB);

/**
 \brief  Unsubscribes to the topics specified by the caller in subscriptionInfo

 \param[in] mqttClientHandle: handle for the mqtt client module instance
 \param[in] subscriptionInfo: data structure containing all the data required to subscribe
 \param[in] numOfTopics: number of topics stored in subscriptionInfo

 \return Success 0 or Failure -1

 \sa MQTT_IF_Subscribe()
 */
int MQTT_IF_unsubscribe(MQTTClient_Handle hClient, char* topic);

/**
 \brief  Publishes to the topic specified by the user

 \param[in] mqttClientHandle: handle for the mqtt client module instance
 \param[in] topic: topic user wants to publish to
 \param[in] payload: payload to publish to topic
 \param[in] payloadLen: length of payload passed by the user
 \param[in] flags QOS define MQTT_PUBLISH_QOS_0, MQTT_PUBLISH_QOS_1 or MQTT_PUBLISH_QOS_2
             use MQTT_PUBLISH_RETAIN is message should be retained

 \return Success 0 or Failure -1

 */
//\param[in] flags QOS define MQTT_PUBLISH_QOS_0, MQTT_PUBLISH_QOS_1 or MQTT_PUBLISH_QOS_2
//use MQTT_PUBLISH_RETAIN if message should be retained
int MQTT_IF_publish(MQTTClient_Handle hClient, char* topic, char* payload, unsigned short payloadLen, int flags);

#endif /* MQTT_IF_H_ */
