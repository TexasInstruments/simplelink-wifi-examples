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
/* This example demonstrates the McSPI RX and TX operation configured
 * in blocking, interrupt mode of operation.
 *
 * This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
 * and then receives the same in RX mode. Internal pad level loopback mode
 * is enabled to receive data.
 * To enable internal pad level loopback mode, D0 pin is configured to both
 * TX Enable as well as RX input pin in the SYSCFG.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */

#include "mqtt_client_app.h"
#include "ti_drivers_config.h"
#include <stdlib.h>
#include <stdlib.h>
#include "FreeRTOS.h"
/* Example/Board Header files */
#include "osi_kernel.h"
#include "uart_term.h"

//LWIP
#include "wifi_if.h"
#include "tcpip_if.h"
#include "mqtt_if.h"
#include "led_if.h"
#include "button_if.h"

#include <ti/drivers/I2C.h>

#define NUM_OF_SENSORS  (2)
#define TEMP_SENSOR_IDX (0)
#define ACC_SENSOR_IDX (1)
/* I2C temperature sensor addresses */
/* I2C temperature sensor target addresses */
#define TMP107_BASSENSORS_ADDR 0x48
/* Temperature result registers */
#define TMP107_RESULT_REG 0x0000

/* I2C accelerometer sensor addresses */
/* I2C accelerometer sensor target addresses */
#define BMA456_BASSENSORS_ADDR 0x18
/* Accelerometer configuration register */
#define BMA456_ACC_CONF_REG 0x0040
/* Accelerometer power control register */
#define BMA456_PWR_CTRL_REG 0x007D
/* Accelerometer chip ID register */
#define BMA456_CHIPID_REG 0x0000
/* Accelerometer x-axis register */
#define BMA456_ACC_X_REG 0x0013
/* Accelerometer y-axis register */
#define BMA456_ACC_Y_REG 0x0015
/* Accelerometer z-axis register */
#define BMA456_ACC_Z_REG 0x0017

/*
 * Data structure containing currently supported I2C TMP sensors.
 * Sensors are ordered by descending preference.
 */
static struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
    uint8 status;
} sensors[NUM_OF_SENSORS] = {{TMP107_BASSENSORS_ADDR, TMP107_RESULT_REG, "TMP107", false},
                             {BMA456_BASSENSORS_ADDR, BMA456_CHIPID_REG, "BMA456", false}};


enum{
    APP_MQTT_PUBLISH,
    APP_MQTT_CON_TOGGLE,
    APP_MQTT_DEINIT,
    APP_MQTT_CONNECTED,
    APP_OTA_TRIGGER,
    APP_MQTT_PUBLISH_FW_VER,
    APP_MQTT_PUBLISH_SENSORS,
};

struct msgQueue
{
    int   event;
    char* payload;
};

static OsiMsgQ_t appQueue;
static int connected;
static int deinit;
static ip4addr_t gIp4Addr = 0, gIp4Mask = 0, gIp4GW = 0;

/****************************************************************************
                      LOCAL FUNCTION PROTOTYPES
****************************************************************************/
static char *IP4ToStr(ip4addr_t ipAddress)
{
    static char ip4str[16];
    uint8_t *pU8 = (uint8_t*)&ipAddress;
    sprintf(ip4str, "%d.%d.%d.%d", pU8[0], pU8[1], pU8[2], pU8[3]);
    return ip4str;
}

static void OnWifiEvent(WifiConnStatus_e status, void *params)
{
    switch (status)
    {
        case WIFI_STATUS_CONNECTED_IP:
        {
            TCPIP_IF_getIp4Addr(params, &gIp4Addr, &gIp4Mask, &gIp4GW);
            if(gIp4Addr)
            {
                UART_PRINT("OnWifiEvent(CONNECTED_IP): addr=%s\r\n", IP4ToStr(gIp4Addr));
                UART_PRINT("                           mask=%s\r\n", IP4ToStr(gIp4Mask));
                UART_PRINT("                           gw=%s\r\n", IP4ToStr(gIp4GW));
            }
            else
            {
                UART_PRINT("OnWifiEvent(CONNECTED_IP): IPv6 only\r\n");
            }
        }
        break;

        default:
            UART_PRINT("OnWifiEvent(%d)\r\n", status);
            break;
    }
}

/****************************************************************************
                      GLOBAL VARIABLES
****************************************************************************/


/*!
    \brief          Display application banner

    This routine shows how to get device information form the NWP.
    Also, it prints the PHY, MAC, NWP and Driver versions.

    \param          appName    -   points to a string representing
                                   application name.

    \param          appVersion -   points to a string representing
                                   application version number.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,
                    this function would return negative value.

    \sa             sl_DeviceGet, sl_NetCfgGet

*/
int32_t DisplayAppBanner(char* appName, char* appVersion)
{
    UART_PRINT("******************************************************************\r\n");
    UART_PRINT("***************** %-28s *******************\r\n", APPLICATION_NAME);
    UART_PRINT("***************** %-28s *******************\r\n", APPLICATION_VERSION);
    UART_PRINT("******************************************************************\r\n");
    return(0);
}

void ButtonSw2EventHandler(BUTTON_IF_events_bm events)
{
    int ret;
    struct msgQueue queueElement;

    if (events & BUTTON_IF_EV_CLICKED)
    {
        //UART_PRINT("APP_BTN_HANDLER SHORT PRESS (CONN/DISC)\r\n");
        queueElement.event = APP_MQTT_CON_TOGGLE;
    }
    if (events & BUTTON_IF_EV_LONG_CLICKED)
    {
        //UART_PRINT("APP_BTN_HANDLER LONG PRESS (DEINIT)\r\n");
        queueElement.event = APP_MQTT_DEINIT;
    }

    ret = osi_MsgQWrite(&appQueue,&queueElement, OSI_WAIT_FOREVER,OSI_FLAG_NOT_FROM_INTR);
    if(ret < 0){
        //UART_PRINT("msg queue send error %d\r\n", ret);
    }
}

void ButtonSw1EventHandler(BUTTON_IF_events_bm events)
{
    int ret;
    struct msgQueue queueElement;

    if (events & BUTTON_IF_EV_CLICKED)
    {
        //UART_PRINT("APP_BTN_HANDLER SHORT PRESS (PUBLISH)\r\n");
        queueElement.event = APP_MQTT_PUBLISH;
    }
    else if (events & BUTTON_IF_EV_LONG_CLICKED)
    {
        //UART_PRINT("APP_BTN_HANDLER LONG PRESS (OTA)\r\n");
        queueElement.event = APP_OTA_TRIGGER;
    }
    ret = osi_MsgQWrite(&appQueue, &queueElement, OSI_WAIT_FOREVER,OSI_FLAG_NOT_FROM_INTR);
    if(ret < 0){
        UART_PRINT("msg queue send error %d\r\n", ret);
    }
}

/*
 * Subscribe topic callbacks
 * Topic and payload data is deleted after topic callbacks return.
 * User must copy the topic or payload data if it needs to be saved.
 */
void BrokerCB(char* topic, uint8_t* payload, uint16_t len, uint8_t qos){
    UART_PRINT("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, payload, qos);
}

void GetFwVerCB(char* topic, uint8_t* payload, uint16_t len, uint8_t qos)
{
    struct msgQueue queueElement;
    int ret;

    UART_PRINT("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, payload, qos);

    queueElement.event = APP_MQTT_PUBLISH_FW_VER;

    ret = osi_MsgQWrite(&appQueue, &queueElement, OSI_WAIT_FOREVER,OSI_FLAG_NOT_FROM_INTR);
    if(ret < 0){
        UART_PRINT("msg queue send error %d\r\n", ret);
    }
}

void GetSensorsCB(char* topic, uint8_t* payload, uint16_t len, uint8_t qos)
{
    struct msgQueue queueElement;
    int ret;

    UART_PRINT("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, payload, qos);

    queueElement.event = APP_MQTT_PUBLISH_SENSORS;

    ret = osi_MsgQWrite(&appQueue, &queueElement, OSI_WAIT_FOREVER,OSI_FLAG_NOT_FROM_INTR);
    if(ret < 0){
        UART_PRINT("msg queue send error %d\r\n", ret);
    }
}

void ToggleLEDCB(char* topic, uint8_t* pPayload, uint16_t len, uint8_t qos)
{
    char *pStr = malloc(len+1);

    if(pStr)
    {
        memcpy(pStr, pPayload, len);
        pStr[len] = 0;
        if (strstr(pStr, "red") != NULL)
        {
            LED_IF_toggle(CONFIG_LED_RED, 100);
        }
        else if (strstr(pStr, "green") != NULL)
        {
            LED_IF_toggle(CONFIG_LED_GREEN, 100);
        }
        else if (strstr(pStr, "blue") != NULL)
        {
            LED_IF_toggle(CONFIG_LED_BLUE, 100);
        }
        else
        {
            UART_PRINT("TOPIC: %s PAYLOAD: %s QOS: %d - no matching color LED\r\n", topic, pStr, qos);
            return;
        }
        UART_PRINT("TOPIC: %s PAYLOAD: %s QOS: %d\r\n", topic, pStr, qos);
        free(pStr);
    }
}

void OnMqttEvent(MQTTClient_Handle hMqttConn, int32_t event, void *args){

    struct msgQueue queueElement;
    char *pStr;

    switch(event){

        case MQTT_EVENT_CONNACK:
        {
            deinit = 0;
            connected = 1;
            pStr = "CONNACK";
            queueElement.event = APP_MQTT_CONNECTED;
            int res = osi_MsgQWrite(&appQueue, &queueElement, OSI_WAIT_FOREVER,OSI_FLAG_NOT_FROM_INTR);
            ASSERT_GENERAL(res >= 0);

            BUTTON_IF_enable(CONFIG_BUTTON_SW2);
            break;
        }

        case MQTT_EVENT_SUBACK:
        {
            pStr = "SUBACK";
            break;
        }

        case MQTT_EVENT_PUBACK:
        {
            pStr = "PUBACK";
            break;
        }

        case MQTT_EVENT_UNSUBACK:
        {
            pStr = "UNSUBACK";
            break;
        }

        case MQTT_EVENT_CLIENT_DISCONNECT:
        {
            connected = 0;
            pStr = "CLIENT_DISCONNECT";
            if(deinit == 0){

                BUTTON_IF_enable(CONFIG_BUTTON_SW2);
            }
            break;
        }

        case MQTT_EVENT_SERVER_DISCONNECT:
        {
            connected = 0;

            pStr = "SERVER_DISCONNECT";

            queueElement.event = APP_MQTT_CON_TOGGLE;
            int res = osi_MsgQWrite(&appQueue, &queueElement, OSI_WAIT_FOREVER,OSI_FLAG_NOT_FROM_INTR);
            ASSERT_GENERAL(res >= 0);
            break;
        }

        case MQTT_EVENT_DESTROY:
        {
            pStr = "DESTROY";
            break;
        }
        default:
        {
            pStr = "UNKNOWN";
        }

    }
    UART_PRINT("MQTT EVENT: %s (%p)\r\n", pStr, hMqttConn);
}

//OSPREY_MX-38
#define HWREG(x)                                                              \
        (*((volatile unsigned long *)(x))) //TODO temporary need to be removed
#define ICACHE_BASE 0x41902000  //TODO temporary need to be removed, only for M3, M$ has different address

void *mainThread(void *args)
{
    int32_t RetVal = -1;
    void *hWifiConn;
    OsiReturnVal_e rc;
    MQTT_IF_InitParams_t initParams = {0};
    WlanFWVersions_t wlanVer = {0};
    char verStr[100];
    char sensorsStr[100];
    char accStr[100];
    I2C_Handle i2c;
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];
    uint16_t sensorIdx;
    int16_t temperature;
    uint8_t accXYZ;
    uint8_t netIdx;
    WlanNetworkEntry_t   netEntry;

    /* init drivers and services */
    HWREG(ICACHE_BASE + 0x84) |= 0x00000001  ;//OSPREY_MX-38
    HWREG(ICACHE_BASE + 0x4) |= 0xc0000000  ;//OSPREY_MX-38

    Board_init();
    InitTerm();

    BUTTON_IF_init();
    BUTTON_IF_registertCallback(CONFIG_BUTTON_SW2, CONFIG_GPIO_BUTTON_SW2_INPUT,
                                ButtonSw2EventHandler, BUTTON_IF_EV_CLICKED, 1000U);
    BUTTON_IF_registertCallback(CONFIG_BUTTON_SW1, CONFIG_GPIO_BUTTON_SW1_INPUT,
                                ButtonSw1EventHandler, BUTTON_IF_EV_CLICKED, 1000U);

    /* Open LED0 (green) and LED1 (red) with default params */
    /* NOTE: by default the LEDS are used to reflect the status of WI-FI (green) and
     *       MQTT (red) connections. Once MQTT is connected -  the app will take control
     *       (through MQTT subscriptions).
     *       To control the leds from the APP only - uncomment WIFI_LED_HANDLE (in wifi_settings.h)
     *       and/or MQTT_LED_HANDLE (in mqtt_settings.h)
     */
    LED_IF_init();

    I2C_init();

    /* Output device information to the UART terminal */
    RetVal = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c               = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        UART_PRINT("error Initializing I2C\n\r");
        while (1) {}
    }
    else
    {
        UART_PRINT("I2C Initialized!\n\r");
    }

    rc = osi_MsgQCreate(&appQueue, "appQueue", sizeof(struct msgQueue), 10);
    ASSERT_GENERAL(rc == OSI_OK);

    rc = WIFI_IF_init(true);
    ASSERT_GENERAL(rc == OSI_OK);

    // delay needed for proper printout
    osi_Sleep(1);

    Wlan_Get(WLAN_GET_FWVERSION, &wlanVer);

    sprintf(verStr, "Firmware version:%d.%d.%d.%d. Phy version:%d.%d.%d.%d.%d.%d",
             wlanVer.major_version,
             wlanVer.minor_version,
             wlanVer.api_version,
             wlanVer.build_version,
             wlanVer.phy_version[5],
             wlanVer.phy_version[4],
             wlanVer.phy_version[3],
             wlanVer.phy_version[2],
             wlanVer.phy_version[1],
             wlanVer.phy_version[0]);

    UART_PRINT("Firmware version:%d.%d.%d.%d\r\n",
             wlanVer.major_version,
             wlanVer.minor_version,
             wlanVer.api_version,
             wlanVer.build_version);

    UART_PRINT("Phy version:%d.%d.%d.%d.%d.%d\r\n",
                     wlanVer.phy_version[5],
                     wlanVer.phy_version[4],
                     wlanVer.phy_version[3],
                     wlanVer.phy_version[2],
                     wlanVer.phy_version[1],
                     wlanVer.phy_version[0]);

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 0;

    /*
     * Determine if I2C sensor is present by querying known I2C
     * target addresses.
     */
    for (sensorIdx = 0; sensorIdx < NUM_OF_SENSORS; sensorIdx++)
    {
        i2cTransaction.targetAddress = sensors[sensorIdx].address;
        txBuffer[0]                  = sensors[sensorIdx].resultReg;

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            UART_PRINT(   "Detected %s sensor with target address 0x%x\n\r",
                          sensors[sensorIdx].id,
                          sensors[sensorIdx].address);
            sensors[sensorIdx].status = true;
        }
        else
        {
            UART_PRINT("Failed to detect %s sensor\n\r", sensors[sensorIdx].id);
            sensors[sensorIdx].status = false;
        }
    }

    i2cTransaction.targetAddress = sensors[ACC_SENSOR_IDX].address;
    i2cTransaction.writeCount = 2;
    txBuffer[0] = BMA456_ACC_CONF_REG;
    txBuffer[1] = 0x17;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        UART_PRINT(   "Accelerometer performance mode disabled\n\r");
    }

    txBuffer[0] = BMA456_PWR_CTRL_REG;
    txBuffer[1] = 0x4;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        UART_PRINT(   "Accelerometer mode enabled\n\r");
    }

    rc = MQTT_IF_init(initParams);
    ASSERT_GENERAL(rc == OSI_OK);

    rc = WIFI_IF_start(OnWifiEvent, WIFI_SERVICE_LVL_IP, 10000, &hWifiConn);

    /* if predefined AP credentials fail, open interactive mode */
    if (rc != OSI_OK)
    {
        while (1)
        {
            WIFI_IF_scan(WLAN_ROLE_STA);
            osi_Sleep(2);
            RetVal = GetCmd((char *)accStr, 100, "please choose a network index to connect to or any other key to rescan: ");
            netIdx = atoi(accStr);
            if ((netIdx >= 1) && (netIdx <= WIFI_IF_getNetEntrySize()))
            {
                break;
            }
        }

        RetVal = WIFI_IF_getNetEntry(netIdx, &netEntry);
        if (OSI_OK == RetVal)
        {
            if (netEntry.SecurityInfo != 0)
            {
                RetVal = GetCmd((char *)accStr, 100, "please enter the password: ");
                if (strlen((char *)accStr) <= PASSWD_LEN_MAX)
                {
                    UART_PRINT("\n\rconnecting to %s\n\r", netEntry.Ssid);
                }

                rc = WIFI_IF_connect(hWifiConn, netIdx, (int8_t *)accStr, strlen((char *)accStr), WIFI_SERVICE_LVL_IP, 10000);
            }
            else
            {
                UART_PRINT("\n\rconnecting to %s\n\r", netEntry.Ssid);
                rc = WIFI_IF_connect(hWifiConn, netIdx, NULL, 0, WIFI_SERVICE_LVL_IP, 10000);
            }
        }
    }

    while(gIp4Addr == 0)
    {
        osi_uSleep(1000);
    }

    UART_PRINT("Connected to AP. Starting MQTT...\r\n");
    if(rc == OSI_OK)
    {
        int ret;
        MQTTClient_Handle hMqttConn = MQTT_IF_clientCreate(NULL, OnMqttEvent);
        ASSERT_GENERAL((uint32_t)hMqttConn);
        ret = MQTT_IF_clientConnect(hMqttConn, NULL);

        while(1)
        {
            struct msgQueue queueElement;
            UART_PRINT("Waiting for messages...\r\n");
            osi_MsgQRead(&appQueue, (char*)&queueElement, OSI_WAIT_FOREVER);
            UART_PRINT("Received message: %d\r\n", queueElement.event);

            if(queueElement.event == APP_MQTT_PUBLISH)
            {
                UART_PRINT("APP_MQTT_PUBLISH\r\n");

                MQTT_IF_publish(hMqttConn,
                                "cc35xx/To/Broker",
                                "message from cc35xx to Broker\r\n",
                                strlen("message from cc35xx to Broker\r\n"),
                                MQTT_QOS_2);
            }
            else if(queueElement.event == APP_MQTT_PUBLISH_FW_VER)
            {
                UART_PRINT("APP_MQTT_PUBLISH_FW_VER\r\n");

                MQTT_IF_publish(hMqttConn,
                                "cc35xx/FwVersion",
                                verStr,
                                strlen(verStr),
                                MQTT_QOS_2);
            }

            else if(queueElement.event == APP_MQTT_PUBLISH_SENSORS)
            {
                UART_PRINT("APP_MQTT_PUBLISH_SENSORS\r\n");

                sensorsStr[0]='\0';
                /*
                 * read temperature sensor
                 */
                if (true == sensors[TEMP_SENSOR_IDX].status)
                {
                    i2cTransaction.targetAddress = sensors[TEMP_SENSOR_IDX].address;
                    txBuffer[0] = sensors[TEMP_SENSOR_IDX].resultReg;
                    i2cTransaction.readCount = 2;

                    if (I2C_transfer(i2c, &i2cTransaction))
                    {
                        /*
                         * Extract degrees C from the received data;
                         * see TMP sensor datasheet
                         */
                        temperature = (rxBuffer[0]);

                        sprintf(sensorsStr, "temperature=%d degC\n\r", temperature);
                        UART_PRINT(sensorsStr);
                    }
                    else
                    {
                        sprintf(sensorsStr, "fail to read temperature sensor\n\r");
                        UART_PRINT(sensorsStr);
                    }
                }

                if (true == sensors[ACC_SENSOR_IDX].status)
                {
                    i2cTransaction.targetAddress = sensors[ACC_SENSOR_IDX].address;
                    txBuffer[0] = BMA456_ACC_X_REG;
                    i2cTransaction.readCount = 1;

                    if (I2C_transfer(i2c, &i2cTransaction))
                    {
                        accXYZ = (rxBuffer[0]);

                        sprintf(accStr, "axisX=%d\n\r", accXYZ);
                        UART_PRINT(accStr);
                    }
                    else
                    {
                        sprintf(accStr, "fail to read x-axis\n\r");
                        UART_PRINT(accStr);
                    }
                    strcat(sensorsStr, accStr);

                    txBuffer[0] = BMA456_ACC_Y_REG;
                    if (I2C_transfer(i2c, &i2cTransaction))
                    {
                        accXYZ = (rxBuffer[0]);

                        sprintf(accStr, "axisY=%d\n\r", accXYZ);
                        UART_PRINT(accStr);
                    }
                    else
                    {
                        sprintf(accStr, "fail to read y-axis\n\r");
                        UART_PRINT(accStr);
                    }
                    strcat(sensorsStr, accStr);

                    txBuffer[0] = BMA456_ACC_Z_REG;
                    if (I2C_transfer(i2c, &i2cTransaction))
                    {
                        accXYZ = (rxBuffer[0]);

                        sprintf(accStr, "axisZ=%d\n\r", accXYZ);
                        UART_PRINT(accStr);
                    }
                    else
                    {
                        sprintf(accStr, "fail to read z-axis\n\r");
                        UART_PRINT(accStr);
                    }
                    strcat(sensorsStr, accStr);
                }

                MQTT_IF_publish(hMqttConn,
                                "cc35xx/Sensors",
                                sensorsStr,
                                strlen(sensorsStr),
                                MQTT_QOS_2);
            }

            else if(queueElement.event == APP_MQTT_CONNECTED)
            {
                UART_PRINT("APP_MQTT_CONNECTED\r\n");
                int ret = MQTT_IF_subscribe(hMqttConn, "Broker/To/cc35xx", MQTT_QOS_2, BrokerCB);
                ret |= MQTT_IF_subscribe(hMqttConn, "cc35xx/ToggleLED", MQTT_QOS_2, ToggleLEDCB);
                ret |= MQTT_IF_subscribe(hMqttConn, "cc35xx/GetFwVersion", MQTT_QOS_2, GetFwVerCB);
                ret |= MQTT_IF_subscribe(hMqttConn, "cc35xx/GetSensors", MQTT_QOS_2, GetSensorsCB);
                ASSERT_GENERAL(ret == 0);

            }
            else if(queueElement.event == APP_MQTT_CON_TOGGLE)
            {
                UART_PRINT("APP_MQTT_CON_TOGGLE %d->%d\r\n", connected, 1-connected);

                if(connected){
                    ret = MQTT_IF_clientDisconnect(hMqttConn);
                    ASSERT_GENERAL(ret >= 0);
                }
                else{
                    ret = MQTT_IF_clientConnect(hMqttConn, NULL);
                    ASSERT_GENERAL(ret >= 0);
                    /* If failed to re-connect to mqtt start over (this will also include waiting for
                     * the wi-fi connection (in case failure of AP connection caused the disconnection )
                     *
                     */
                }
            }
            else if(queueElement.event == APP_MQTT_DEINIT){
                UART_PRINT("APP_MQTT_DEINIT\r\n", connected, 1-connected);

                break;
            }
        }
        MQTT_IF_clientDestroy(hMqttConn);
    }
    WIFI_IF_stop(hWifiConn);
    MQTT_IF_deinit();

    return NULL;
}
