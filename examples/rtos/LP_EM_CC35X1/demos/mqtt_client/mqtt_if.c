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
#include "mqtt_if.h"
#include "lwip/apps/mqtt.h"
#include "osi_kernel.h"
#include "dns_if.h"


#ifdef MQTT_LED_INDEX
#include "led_if.h"
#endif

#include "uart_term.h"

typedef struct mqttSub
{
    char                    *pTopic;
    MQTT_IF_TopicCallback_f fUserCB;
    struct mqttSub          *pNext;
} mqttSub_t;

typedef struct mqttIfClient
{
    MQTT_IF_EventCallback_f            fUserCB;
    void*                              hLWIPClient;
    struct mqtt_connect_client_info_t  clientInfo;
    mqttSub_t                         *pSubFirst;
    mqttSub_t                         *pTopicFound;
    struct mqttIfClient               *pNext;
} mqttIfClient_t;

static struct
{
    mqttIfClient_t *pClientFirst;
    /*** TODO - Add mutex for thread safe operation ***/
} m_ctx;

static void LwipCB_mqttPubAck(void *arg, err_t result)        //added
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)arg;

    UART_PRINT("Publish result: %d\r\n", result);

    if(result == 0 && pClient->fUserCB)
    {
        pClient->fUserCB(pClient, MQTT_EVENT_PUBACK, 0);
    }
}

static void LwipCB_mqttSubAck(void *arg, err_t result)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)arg;

    UART_PRINT("Subscription result: %d\r\n", result);
    if(result == 0 && pClient->fUserCB)
    {
        pClient->fUserCB(pClient, MQTT_EVENT_SUBACK, 0);
    }
}

static void LwipCB_mqttUnsubAck(void *arg, err_t result)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)arg;

    UART_PRINT("Un-subscription result: %d\r\n", result);
    if(result == 0 && pClient->fUserCB)
    {
        pClient->fUserCB(pClient, MQTT_EVENT_UNSUBACK, 0);
    }
}

static void LwipCB_mqttConnectComplete(mqtt_client_t *pMqttClient, void *arg, mqtt_connection_status_t status)
{
    int mqttIfStatus = MQTT_EVENT_SERVER_DISCONNECT;
    mqttIfClient_t *pClient = (mqttIfClient_t*)arg;

    ASSERT_GENERAL((void*)pMqttClient == pClient->hLWIPClient);

    UART_PRINT("MQTT client \"%s\" connection cb: status %d\r\n", pClient->clientInfo.client_id, (int)status);

    if(status == MQTT_CONNECT_ACCEPTED)
    {
#ifdef MQTT_LED_INDEX
       LED_IF_set(MQTT_LED_INDEX, 100);
#endif
        mqttIfStatus = MQTT_EVENT_CONNACK;
    }
    else
    {
#ifdef MQTT_LED_INDEX
       LED_IF_set(MQTT_LED_INDEX, 0);
#endif
    }
    if(pClient->fUserCB)
        pClient->fUserCB(pClient->hLWIPClient, mqttIfStatus, (void*)status);
}


static void LwipCB_mqttIncomingData(void *arg, const u8_t *data, u16_t len, u8_t flags)                   //updated
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)arg;

    UART_PRINT("IncomingData: %s, len=%d\n\r\n", pClient->pTopicFound->pTopic, len);
    if(pClient->pTopicFound && pClient->pTopicFound->fUserCB)
    {
        pClient->pTopicFound->fUserCB(pClient->pTopicFound->pTopic, (uint8_t *)data, len, flags);
    }
    pClient->pTopicFound = NULL;
}

static void LwipCB_mqttIncomingTopic(void *arg, const char *topic, u32_t len)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)arg;
    mqttSub_t *pSub = pClient->pSubFirst;

    UART_PRINT("IncomingTopic: %s, len=%d\r\n", topic, len);

    while(pSub)
    {
        if(!strcmp(topic,pSub->pTopic))
        {
            UART_PRINT("found match\r\n");
            pClient->pTopicFound = pSub;
            break;
        }
        pSub = pSub->pNext;
    }
}
static void FreeAllSubscriptions(MQTTClient_Handle hClient)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)hClient;
    mqttSub_t *pSub = pClient->pSubFirst, *pNext;

    while(pSub)
    {
        pNext = pSub->pNext;
        free(pSub);
        pSub = pNext;
    }
    pClient->pSubFirst = 0;
}

static void FreeSubscription(MQTTClient_Handle hClient, char *pTopic)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)hClient;
    mqttSub_t *pSub = pClient->pSubFirst, *pPrev = NULL;

    if(pTopic == NULL)
        FreeAllSubscriptions(hClient);
    else
    {
        while(pSub)
        {
            if(strcmp(pTopic, pSub->pTopic) == 0)
            {
                if(pPrev == NULL)
                {
                    pClient->pSubFirst = pSub->pNext;
                }
                else
                {
                    pPrev->pNext = pSub->pNext;
                }
                free(pSub);
                break;
            }
            pPrev = pSub;
            pSub = pSub->pNext;
        }

    }
}


int MQTT_IF_init(MQTT_IF_InitParams_t initParams)
{
    int ret = 0;
    DNS_IF_init();
    m_ctx.pClientFirst = NULL;
#ifdef MQTT_LED_INDEX
    LED_IF_set(MQTT_LED_INDEX, 0);
#endif
    return ret;
}

int MQTT_IF_deinit()
{
    mqttIfClient_t *pCur = m_ctx.pClientFirst, *pFree = NULL;

    /* Free all clients */
    while(pCur)
    {
        FreeAllSubscriptions(pCur); /* Delete all subscriptions */
        pFree = pCur;
        pCur = pCur->pNext;
        free(pFree);
    }
    return 0;
}


MQTTClient_Handle MQTT_IF_clientCreate(MQTT_IF_ClientParams_t *pClientParams, MQTT_IF_EventCallback_f fUserCB)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)malloc(sizeof(mqttIfClient_t));
    if(pClient)
    {
        memset(pClient, 0, sizeof(mqttIfClient_t));
        pClient->fUserCB = fUserCB;
        if(pClientParams)
        {
            pClient->clientInfo.client_id = pClientParams->clientID;
            pClient->clientInfo.client_pass = pClientParams->password;
            pClient->clientInfo.client_user = pClientParams->username;
            pClient->clientInfo.keep_alive = pClientParams->keepaliveTime;
            pClient->clientInfo.will_msg = pClientParams->willParams->willMsg;
            pClient->clientInfo.will_qos = pClientParams->willParams->willQos;
            pClient->clientInfo.will_retain = pClientParams->willParams->retain;
            pClient->clientInfo.will_topic = pClientParams->willParams->willTopic;
        }
        else
        {
            pClient->clientInfo.client_id = MQTT_CLIENT_CLIENTID;
            pClient->clientInfo.client_pass = MQTT_CLIENT_PASSWORD;
            pClient->clientInfo.client_user = MQTT_CLIENT_USERNAME;
            pClient->clientInfo.keep_alive = MQTT_CLIENT_KEEPALIVE;
            pClient->clientInfo.will_msg = MQTT_WILL_MSG;
            pClient->clientInfo.will_qos = MQTT_WILL_QOS;
            pClient->clientInfo.will_retain = MQTT_WILL_RETAIN;
            pClient->clientInfo.will_topic = MQTT_WILL_TOPIC;
        }

        LOCK_TCPIP_CORE();
        pClient->hLWIPClient = mqtt_client_new();
        if(pClient->hLWIPClient == NULL)
        {
            UART_PRINT("mqtt_client_new() failure\r\n");
            free(pClient);
            pClient = NULL;
        }
        else
        {
            /* add to head of client list */
            pClient->pNext = m_ctx.pClientFirst;
            m_ctx.pClientFirst = pClient;
        }
        UNLOCK_TCPIP_CORE();
    }
    return (MQTTClient_Handle)pClient;
}

int MQTT_IF_clientDestroy(MQTTClient_Handle hClient)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)hClient, *pCur = m_ctx.pClientFirst, *pPrev = NULL;
    mqtt_client_free(pClient->hLWIPClient);

    /* remove from client list */
    while(pCur)
    {
        if(pCur == pClient)
        {
            if(pPrev == NULL)
                m_ctx.pClientFirst = pCur->pNext;
            else
                pPrev->pNext = pCur->pNext;
        }
        pPrev = pCur;
        pCur = pCur->pNext;
    }
    FreeAllSubscriptions(pClient);

    free(pClient);
    return 0;
}

int MQTT_IF_clientConnect(MQTTClient_Handle hClient, MQTTClient_ConnParams *pConnParams)
{
    err_t rc = ERR_MEM;
    ip_addr_t mqttIp = {0};
    mqttIfClient_t *pClient = (mqttIfClient_t*)hClient;
    MQTTClient_ConnParams connParams = {0};
#if MQTT_SECURE_CLIENT
    char *pSecureFiles[1];
#endif

    if(pConnParams == NULL)
    {
        pConnParams = &connParams;

        connParams.serverAddr = MQTT_CONNECTION_ADDRESS;
        connParams.port = MQTT_CONNECTION_PORT_NUMBER;
        connParams.netconnFlags = MQTT_CONNECTION_FLAGS;

#ifdef MQTT_SECURE_CLIENT
#endif
    }
    if(pClient && pClient->hLWIPClient)
    {
        rc = ERR_OK;
        if(pConnParams->netconnFlags & MQTTCLIENT_NETCONN_URL)
        {
            rc = DNS_IF_gethostbyname(pConnParams->serverAddr, &mqttIp);
        }
        else
        {
            ipaddr_aton(pConnParams->serverAddr, &mqttIp);
        }
        while(rc == 0)
        {
            LOCK_TCPIP_CORE();
            rc = mqtt_client_connect(pClient->hLWIPClient,
                                     &mqttIp, pConnParams->port,
                                     LwipCB_mqttConnectComplete, pClient,
                                     &pClient->clientInfo);
            if(rc == 0)
            {
                mqtt_set_inpub_callback(pClient->hLWIPClient,
                        LwipCB_mqttIncomingTopic,
                        LwipCB_mqttIncomingData,
                        pClient);
                UNLOCK_TCPIP_CORE();
                break;
            }
            else
            {
                UNLOCK_TCPIP_CORE();
                UART_PRINT("mqtt_client_connect() = %d (Retrying...)\r\n", rc);
                os_sleep(1,0);
                rc = 0;
            }
        }
    }
    return (rc);

}



int MQTT_IF_clientDisconnect(MQTTClient_Handle hClient)
{
    mqttIfClient_t *pClient = (mqttIfClient_t*)hClient;
    LOCK_TCPIP_CORE();
    mqtt_disconnect(pClient->hLWIPClient);
    UNLOCK_TCPIP_CORE();
    if(pClient->fUserCB)
        pClient->fUserCB(pClient->hLWIPClient, MQTT_EVENT_CLIENT_DISCONNECT, (void*)NULL);
#ifdef MQTT_LED_INDEX
    LED_IF_set(MQTT_LED_INDEX, 0);
#endif
    return 0;
}


int MQTT_IF_subscribe(MQTTClient_Handle hClient, char* pTopic, unsigned int qos, MQTT_IF_TopicCallback_f fUserCB)
{
    err_t rc = ERR_MEM;
    mqttSub_t *pSub = (mqttSub_t*)malloc(sizeof(mqttSub_t));
    if(pSub)
    {
        mqttIfClient_t *pClient = (mqttIfClient_t*)hClient;
        pSub->pTopic = pTopic;
        pSub->fUserCB = fUserCB;
        pSub->pNext = pClient->pSubFirst;
        pClient->pSubFirst = pSub;
        LOCK_TCPIP_CORE();
        rc = mqtt_sub_unsub(pClient->hLWIPClient, pTopic, qos, LwipCB_mqttSubAck, pClient, 1);
        UNLOCK_TCPIP_CORE();
    }
    return (int)rc;
}

int MQTT_IF_unsubscribe(MQTTClient_Handle hClient, char* pTopic)
{
    err_t rc;
    mqttIfClient_t *pClient = (mqttIfClient_t*)hClient;
    LOCK_TCPIP_CORE();
    rc = mqtt_sub_unsub(pClient->hLWIPClient, pTopic, 0, LwipCB_mqttUnsubAck, pClient, 0);
    UNLOCK_TCPIP_CORE();
    FreeSubscription(pClient, pTopic);

    return (int)rc;
}

int MQTT_IF_publish(MQTTClient_Handle hClient, char* topic, char* payload, unsigned short payloadLen, int flags)
{
    int ret = 0;
    mqttIfClient_t *pClient = (mqttIfClient_t*)hClient;
    LOCK_TCPIP_CORE();
    mqtt_publish(pClient->hLWIPClient, topic, payload, payloadLen, 0, 0, LwipCB_mqttPubAck, pClient);
    UNLOCK_TCPIP_CORE();
    return ret;
}
