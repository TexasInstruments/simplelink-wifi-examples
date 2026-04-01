/*
 * Copyright (c) 2016-2025, Texas Instruments Incorporated
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

// Standard includes
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* POSIX Header files */
#include <pthread.h>
#include <unistd.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerWFF3.h>

/* WiFi driver includes */
#include "wlan_if.h"

/* lwIP includes */
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* Application includes */
#include "power_measure.h"
#include "uart_term.h"
#include "network_lwip.h"
#include "ti_drivers_config.h"
#include "nimble_host.h"
#include "ble_if.h"

/* Nimble host includes */
#include "host/ble_gap.h"

//*****************************************************************************
// Globals
//*****************************************************************************

PowerMeasure_AppData PowerMeasure_appData = {0};  /* Initialize all fields to 0 */
uint32_t g_Status = 0;
static volatile bool g_LwipInitialized = false;
OsiSyncObj_t bleConnSyncObj = NULL;

//*****************************************************************************
// Static globals
//*****************************************************************************

/* Application control block used by network stack */
appControlBlock app_CB;

//*****************************************************************************
// Forward declarations
//*****************************************************************************

static int setTechnology(void);
static void setUseCase(int technology);
static void setAlwaysConnectedUseCase(void);
static void getBLEAdvParams(void);
static void setBLEConnParams(void);
static int32_t initializeLWIPIfNeeded(void);
static int32_t wlanConnectLayer2Only(void);

//*****************************************************************************
//
//! Register extra BLE GAP event callback
//!
//! \param  event - BLE GAP event structure
//!
//! \return None
//!
//*****************************************************************************
static void extraGapEventCallback(void *event, void *arg)
{
    struct ble_gap_event *bleGapEvent = (struct ble_gap_event *)event;

    switch (bleGapEvent->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            /* Signal sync object */
            osi_SyncObjSignal(&bleConnSyncObj);
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
//! \brief Network Status Callback
//!
//! This callback is called when the network interface acquires an IP address
//! through DHCP.
//!
//! \param roleid - Role ID (STA or AP)
//! \param address - IP address acquired
//!
//! \return none
//!
//*****************************************************************************
void NetworkStatusCallback(WlanRole_e roleid, uint32_t address)
{
    if (roleid == WLAN_ROLE_STA && address != 0)
    {
        SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
        UART_PRINT("[NETWORK] IP Acquired: %d.%d.%d.%d\n\r",
                   (address & 0xFF),
                   ((address >> 8) & 0xFF),
                   ((address >> 16) & 0xFF),
                   ((address >> 24) & 0xFF));
    }
}

//*****************************************************************************
//
//! \brief WLAN Event Handler
//!
//! This handler gets called whenever a WLAN event is reported by the host
//! driver / NWP. It handles connection, disconnection, and IP acquisition events.
//!
//! \param pWlanEvent - pointer to WLAN event structure
//!
//! \return none
//!
//*****************************************************************************
void WlanStackEventHandler(WlanEvent_t *pWlanEvent)
{
    void *staif = NULL;

    if (!pWlanEvent)
    {
        return;
    }

    switch (pWlanEvent->Id)
    {
        case WLAN_EVENT_CONNECT:
        {
            if (pWlanEvent->Data.Connect.Status < 0)
            {
                UART_PRINT("[WLAN] Connection failed\n\r");
                break;
            }

            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            UART_PRINT("[WLAN] Connected to AP: %s\n\r",
                       pWlanEvent->Data.Connect.SsidName);

            /* Bring up network interface only if LWIP is initialized */
            /* For Layer 2-only mode (sub-mode 2.3), skip network stack interaction */
            if (g_LwipInitialized)
            {
                staif = network_get_sta_if();
                if (staif != NULL)
                {
                    network_set_up(staif);
                }
            }
        }
        break;

        case WLAN_EVENT_DISCONNECT:
        {
            UART_PRINT("[WLAN] Disconnected from AP\n\r");
            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /* Bring down network interface only if LWIP is initialized */
            if (g_LwipInitialized)
            {
                staif = network_get_sta_if();
                if (staif != NULL)
                {
                    network_set_down(staif);
                }
            }
        }
        break;

        default:
            break;
    }
}

//*****************************************************************************
//
//! Main application thread
//!
//! \param  arg0 - ignored
//!
//! \return none
//!
//*****************************************************************************
void *mainThread(void *arg0)
{
    int32_t ret = 0;

    /* Call driver init functions */
    GPIO_init();
    Power_init();

    /* NOTE: LWIP initialization moved to individual mode functions */
    /* It will be initialized only when needed (sub-modes 2.1 and 2.2) */

    /* Initialize UART terminal */
    InitTerm();
    ClearTerm();

    UART_PRINT("About to call Wlan_Start...\n\r");
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to flush

    /* Initialize WiFi driver - MUST be done before Wlan_Get/Wlan_Set calls */
    ret = Wlan_Start(WlanStackEventHandler);
    if (ret < 0)
    {
        UART_PRINT("[ERROR] Failed to start WiFi driver\n\r");
        while (1) {}
    }

    /* Display application banner (uses Wlan_Get for MAC address) */
    displayBanner();

    /* Get technology selection (WiFi or BLE) */
    int technology = setTechnology();

    /* Get user selection for power mode use case */
    setUseCase(technology);

    /* Get WiFi credentials if connecting to AP */
    if (PowerMeasure_appData.useCase == UseCase_AlwaysConnected)
    {
        getWiFiCredentials();
    }

    /* Get BLE parameters if using BLE modes */
    if (PowerMeasure_appData.useCase == UseCase_BleAdvertisement)
    {
        getBLEAdvParams();
    }
    else if (PowerMeasure_appData.useCase == UseCase_BleConnection)
    {
        setBLEConnParams();
    }

    /* Setup WiFi only for WiFi use cases */
    if (PowerMeasure_appData.useCase != UseCase_BleAdvertisement &&
        PowerMeasure_appData.useCase != UseCase_BleConnection && 
        PowerMeasure_appData.useCase != UseCase_SLEEP)
    {
        /* Bring up WiFi station role */
        RoleUpStaCmd_t roleParams = {0};
        uint8_t sta_wifi_band =  (uint8_t)BAND_SEL_BOTH;

        Wlan_Set(WLAN_SET_STA_WIFI_BAND, &sta_wifi_band);

        ret = Wlan_RoleUp(WLAN_ROLE_STA, &roleParams, WLAN_WAIT_FOREVER);
        if (ret < 0)
        {
            UART_PRINT("[ERROR] Failed to bring up station role\n\r");
            while (1) {}
        }

        /* NOTE: network_stack_add_if_sta() moved to individual mode functions */
        /* It will be called only when LWIP is initialized */
    }

    /* Note: Power configuration moved inside use case functions to ensure */
    /* it happens AFTER connection/DHCP for modes that require it */

    /* Display start measurement banner */
    startMeasureBanner();

    /* Execute the selected use case */
    switch (PowerMeasure_appData.useCase)
    {
        case UseCase_SLEEP:
            sleepMode();
            break;

        case UseCase_AlwaysConnected:
            alwaysConnected();
            break;

        case UseCase_BleAdvertisement:
            bleAdvertisementMode();
            break;

        case UseCase_BleConnection:
            bleConnectionMode();
            break;

        default:
            UART_PRINT("[ERROR] Invalid use case selected\n\r");
            break;
    }

    /* Should never reach here */
    while (1)
    {
        sleep(1000);
    }
}

//*****************************************************************************
//
//! Get technology selection from user (WiFi or BLE)
//!
//! \param  None
//!
//! \return Technology selection: 1 for WiFi, 2 for BLE
//!
//*****************************************************************************
static int setTechnology(void)
{
    char sel[2];
    int technology;

    UART_PRINT("\n\r*** Select Technology: ***\n\r");
    UART_PRINT("    1) WiFi\n\r");
    UART_PRINT("    2) BLE\n\r");
    UART_PRINT("\n\rPlease enter your selection: ");

    GetCmd((char *)sel, sizeof(sel), "");
    UART_PRINT("\n\r");

    technology = atoi((const char*)sel);

    if (technology != 1 && technology != 2)
    {
        UART_PRINT("[WARN] Invalid selection, defaulting to WiFi\n\r");
        technology = 1;
    }

    return technology;
}

//*****************************************************************************
//
//! Get from the user the selected Power Management use case
//!
//! \param  technology - Technology selection (1 = WiFi, 2 = BLE)
//!
//! \return None
//!
//*****************************************************************************
static void setUseCase(int technology)
{
    char sel[2];
    int selection;

    if (technology == 1) /* WiFi */
    {
        UART_PRINT("\n\r*** WiFi Power Management Options: ***\n\r");
        UART_PRINT("    1) SLEEP Mode (WiFi + Host low power)\n\r");
        UART_PRINT("    2) Always Connected\n\r");
        UART_PRINT("\n\rPlease enter your selection: ");

        GetCmd((char *)sel, sizeof(sel), "");
        UART_PRINT("\n\r");

        selection = atoi((const char*)sel);

        if (selection == 1)
        {
            PowerMeasure_appData.useCase = UseCase_SLEEP;
        }
        else if (selection == 2)
        {
            PowerMeasure_appData.useCase = UseCase_AlwaysConnected;
            setAlwaysConnectedUseCase();
        }
        else
        {
            UART_PRINT("[WARN] Invalid selection, defaulting to Always Connected\n\r");
            PowerMeasure_appData.useCase = UseCase_AlwaysConnected;
            setAlwaysConnectedUseCase();
        }
    }
    else /* BLE */
    {
        UART_PRINT("\n\r*** BLE Power Management Options: ***\n\r");
        UART_PRINT("    1) BLE Advertisement\n\r");
        UART_PRINT("    2) BLE Connection (Peripheral Role)\n\r");
        UART_PRINT("\n\rPlease enter your selection: ");

        GetCmd((char *)sel, sizeof(sel), "");
        UART_PRINT("\n\r");

        selection = atoi((const char*)sel);

        if (selection == 1)
        {
            PowerMeasure_appData.useCase = UseCase_BleAdvertisement;
        }
        else if (selection == 2)
        {
            PowerMeasure_appData.useCase = UseCase_BleConnection;
        }
        else
        {
            UART_PRINT("[WARN] Invalid selection, defaulting to BLE Advertisement\n\r");
            PowerMeasure_appData.useCase = UseCase_BleAdvertisement;
        }
    }
}

//*****************************************************************************
//
//! Get from the user the selected Always Connected use case sub-mode
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
static void setAlwaysConnectedUseCase(void)
{
    char sel[4];
    int selection;
    int dtimValue;

    UART_PRINT("\n\r*** Always Connected Sub-Mode Options: ***\n\r");
    UART_PRINT("    1) Host Always On (LWIP active, CPU active)\n\r");
    UART_PRINT("    2) Host Sleep with LWIP (LWIP active, CPU sleeps)\n\r");
    UART_PRINT("    3) Host Always Sleep (LWIP disabled, Layer 2 only)\n\r");
    UART_PRINT("\n\rPlease enter your selection: ");

    GetCmd((char *)sel, sizeof(sel), "");
    UART_PRINT("\n\r");

    selection = atoi((const char*)sel);

    /* Set sub-mode based on selection */
    if (selection == 1)
    {
        PowerMeasure_appData.alwaysConnectedUseCase = AlwaysConnected_HostAlwaysOn;
    }
    else if (selection == 2)
    {
        PowerMeasure_appData.alwaysConnectedUseCase = AlwaysConnected_HostSleepWithLWIP;
    }
    else if (selection == 3)
    {
        PowerMeasure_appData.alwaysConnectedUseCase = AlwaysConnected_HostAlwaysSleep;
    }
    else
    {
        UART_PRINT("[WARN] Invalid selection, defaulting to Host Sleep with LWIP\n\r");
        PowerMeasure_appData.alwaysConnectedUseCase = AlwaysConnected_HostSleepWithLWIP;
    }

    /* Always prompt for DTIM interval (all sub-modes use same DTIM config) */
    UART_PRINT("\n\rEnter DTIM interval (1-20) [1]: ");
    GetCmd((char *)sel, sizeof(sel), "");
    UART_PRINT("\n\r");

    /* Use default if empty input */
    if (strlen(sel) == 0)
    {
        dtimValue = 1;
        UART_PRINT("Using default DTIM interval: 1\n\r");
    }
    else
    {
        dtimValue = atoi((const char*)sel);

        /* Validate and store */
        if (dtimValue < 1 || dtimValue > 20)
        {
            UART_PRINT("[WARN] Invalid DTIM interval, using default value 1\n\r");
            PowerMeasure_appData.customDtimInterval = 1;
        }
        else
        {
            PowerMeasure_appData.customDtimInterval = (uint8_t)dtimValue;
            UART_PRINT("DTIM interval set to: %d\n\r", PowerMeasure_appData.customDtimInterval);
        }
    }

    /* Store the value if it was default */
    if (dtimValue == 1 && strlen(sel) == 0)
    {
        PowerMeasure_appData.customDtimInterval = 1;
    }
}

//*****************************************************************************
//
//! Display Application Banner
//!
//! \param  none
//!
//! \return 0 on success
//!
//*****************************************************************************
int32_t displayBanner(void)
{
    WlanMacAddress_t macAddressParams;
    int32_t ret = 0;

    /* Initialize MAC address structure */
    memset(&macAddressParams, 0, sizeof(WlanMacAddress_t));
    macAddressParams.roleType = WLAN_ROLE_STA;

    /* Get MAC address */
    ret = Wlan_Get(WLAN_GET_MACADDRESS, (void *)&macAddressParams);
    if (ret < 0)
    {
        UART_PRINT("[WARN] Failed to get MAC address\n\r");
        memset(macAddressParams.pMacAddress, 0, sizeof(macAddressParams.pMacAddress));
    }

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\t    %s Ver: %s\n\r", APPLICATION_NAME, APPLICATION_VERSION);
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t Device: CC35X1E (Cortex-M33)\n\r");
    UART_PRINT("\t MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",
               macAddressParams.pMacAddress[0], macAddressParams.pMacAddress[1], macAddressParams.pMacAddress[2],
               macAddressParams.pMacAddress[3], macAddressParams.pMacAddress[4], macAddressParams.pMacAddress[5]);
    UART_PRINT("\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");

    return 0;
}

//*****************************************************************************
//
//! Start measure banner printing
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void startMeasureBanner(void)
{
    vTaskDelay(pdMS_TO_TICKS(10)); /* Allow UART to flush */

    switch (PowerMeasure_appData.useCase)
    {
        case UseCase_SLEEP:
            UART_PRINT("\n\rEntering SLEEP mode, start measuring current...\n\r");
            break;

        case UseCase_AlwaysConnected:
            switch (PowerMeasure_appData.alwaysConnectedUseCase)
            {
                case AlwaysConnected_HostAlwaysOn:
                    UART_PRINT("\n\rStarting Always Connected - Sub-mode 2.1: Host Always On (DTIM: %d), start measuring current...\n\r",
                               PowerMeasure_appData.customDtimInterval);
                    break;

                case AlwaysConnected_HostSleepWithLWIP:
                    UART_PRINT("\n\rStarting Always Connected - Sub-mode 2.2: Host Sleep with LWIP (DTIM: %d), start measuring current...\n\r",
                               PowerMeasure_appData.customDtimInterval);
                    break;

                case AlwaysConnected_HostAlwaysSleep:
                    UART_PRINT("\n\rStarting Always Connected - Sub-mode 2.3: Host Always Sleep (DTIM: %d), start measuring current...\n\r",
                               PowerMeasure_appData.customDtimInterval);
                    break;
            }
            break;

        case UseCase_BleAdvertisement:
            UART_PRINT("\n\rStarting BLE Advertisement mode (interval: %.2f ms), start measuring current...\n\r",
                       (float)PowerMeasure_appData.bleAdvInterval_us / 1000.0f);
            break;

        case UseCase_BleConnection:
            UART_PRINT("\n\rStarting BLE Connection mode\n\r");
            break;
        default:
            break;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); /* Allow UART to flush */
}

//*****************************************************************************
//
//! Prompt user for WiFi credentials
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void getWiFiCredentials(void)
{
    char input[8];

    UART_PRINT("\n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("        WiFi Configuration\n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("\n\r");

    /* Get SSID */
    UART_PRINT("Enter WiFi SSID (max 32 characters): ");
    GetCmd(PowerMeasure_appData.ssid, sizeof(PowerMeasure_appData.ssid), "");
    UART_PRINT("\n\r");

    /* Get Password */
    UART_PRINT("Enter WiFi Password (max 63 characters, leave empty for OPEN): ");
    GetCmd(PowerMeasure_appData.password, sizeof(PowerMeasure_appData.password), "");
    UART_PRINT("\n\r");

    /* Determine security type based on password */
    if (strlen(PowerMeasure_appData.password) == 0)
    {
        PowerMeasure_appData.securityType = WLAN_SEC_TYPE_OPEN;
        UART_PRINT("Security: OPEN\n\r");
    }
    else
    {
        /* Prompt for security type */
        UART_PRINT("\n\rSelect Security Type:\n\r");
        UART_PRINT("  1) WPA/WPA2\n\r");
        UART_PRINT("  2) WPA2/WPA3\n\r");
        UART_PRINT("  3) WPA3\n\r");
        UART_PRINT("\n\rPlease enter your selection [1]: ");

        GetCmd(input, sizeof(input), "");
        UART_PRINT("\n\r");

        /* Default to WPA/WPA2 if empty or invalid input */
        if (input[0] == '2')
        {
            PowerMeasure_appData.securityType = WLAN_SEC_TYPE_WPA2_WPA3;
            UART_PRINT("Security: WPA2/WPA3\n\r");
        }
        else if (input[0] == '3')
        {
            PowerMeasure_appData.securityType = WLAN_SEC_TYPE_WPA3;
            UART_PRINT("Security: WPA3\n\r");
        }
        else
        {
            PowerMeasure_appData.securityType = WLAN_SEC_TYPE_WPA_WPA2;
            UART_PRINT("Security: WPA/WPA2\n\r");
        }
    }

    UART_PRINT("\n\r");
    UART_PRINT("WiFi Configuration Complete\n\r");
    UART_PRINT("  SSID: %s\n\r", PowerMeasure_appData.ssid);
    UART_PRINT("==============================================\n\r");
    UART_PRINT("\n\r");
}

//*****************************************************************************
//
//! Connect to WLAN Access Point
//!
//! \param  None
//!
//! \return 0 on success, negative on error
//!
//*****************************************************************************
int32_t wlanConnect(void)
{
    int32_t ret = 0;

    UART_PRINT("Connecting to AP: %s\n\r", PowerMeasure_appData.ssid);

    /* Connect to AP using runtime credentials from user input */
    ret = Wlan_Connect((const signed char *)PowerMeasure_appData.ssid,
                       strlen(PowerMeasure_appData.ssid),
                       NULL,  /* MAC address - NULL for any */
                       PowerMeasure_appData.securityType,
                       (const char *)PowerMeasure_appData.password,
                       strlen(PowerMeasure_appData.password),
                       0);  /* flags */
    if (ret < 0)
    {
        UART_PRINT("[ERROR] Failed to initiate connection\n\r");
        return ret;
    }

    /* Wait for connection and IP acquisition with LED blinking */
    while (!IS_CONNECTED(g_Status) || !IS_IP_ACQUIRED(g_Status))
    {
        /* Blink LED to indicate waiting for connection */
        GPIO_toggle(CONFIG_GPIO_LED_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    UART_PRINT("Connected to AP successfully\n\r");
    UART_PRINT("IP address acquired\n\r");

    /* Turn on LED to indicate connected */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    return 0;
}

//*****************************************************************************
//
//! Configure SimpleLink to the selected use case power settings
//!
//! \param  None
//!
//! \return 0 on success, negative on error
//!
//*****************************************************************************
int32_t configSimplelinkToUseCase(void)
{
    int32_t ret = 0;

    switch (PowerMeasure_appData.useCase)
    {
        case UseCase_AlwaysConnected:
        {
            /* All Always Connected sub-modes use the same WiFi power configuration */
            /* Enable WiFi power save mode */
            WlanPowerSave_e psMode = WLAN_STATION_POWER_SAVE_MODE;
            ret = Wlan_Set(WLAN_SET_POWER_SAVE, &psMode);
            if (ret < 0)
            {
                UART_PRINT("[ERROR] Failed to set power save mode\n\r");
                return ret;
            }

            /* Enable ELP (Extremely Low Power) mode */
            WlanPowerManagement_e pmMode = POWER_MANAGEMENT_ELP_MODE;
            ret = Wlan_Set(WLAN_SET_POWER_MANAGEMENT, &pmMode);
            if (ret < 0)
            {
                UART_PRINT("[ERROR] Failed to set power management\n\r");
                return ret;
            }

            /* Configure DTIM interval (always configured, even if interval = 1) */
            WlanLongSleepInterval lsiParams;
            lsiParams.ListenInterval = PowerMeasure_appData.customDtimInterval;
            lsiParams.WakeUpEvent = WAKE_UP_EVENT_N_DTIM;

            ret = Wlan_Set(WLAN_SET_LSI, &lsiParams);
            if (ret < 0)
            {
                UART_PRINT("[ERROR] Failed to set DTIM interval\n\r");
                return ret;
            }

            /* Display configuration message based on sub-mode */
            switch (PowerMeasure_appData.alwaysConnectedUseCase)
            {
                case AlwaysConnected_HostAlwaysOn:
                    UART_PRINT("Configured for Sub-mode 2.1: Host Always On (DTIM interval: %d)\n\r",
                               PowerMeasure_appData.customDtimInterval);
                    break;

                case AlwaysConnected_HostSleepWithLWIP:
                    UART_PRINT("Configured for Sub-mode 2.2: Host Sleep with LWIP (DTIM interval: %d)\n\r",
                               PowerMeasure_appData.customDtimInterval);
                    break;

                case AlwaysConnected_HostAlwaysSleep:
                    UART_PRINT("Configured for Sub-mode 2.3: Host Always Sleep (DTIM interval: %d)\n\r",
                               PowerMeasure_appData.customDtimInterval);
                    break;
                default:
                    break;
            }
            break;
        }

        case UseCase_BleAdvertisement:
        case UseCase_BleConnection:
        {
            /* Enable ELP (Extremely Low Power) mode for BLE */
            WlanPowerManagement_e pmMode = POWER_MANAGEMENT_ELP_MODE;
            ret = Wlan_Set(WLAN_SET_POWER_MANAGEMENT, &pmMode);
            if (ret < 0)
            {
                UART_PRINT("[ERROR] Failed to set ELP mode for BLE\n\r");
                return ret;
            }

            UART_PRINT("Configured ELP mode for BLE\n\r");
            break;
        }
        default:
            break;
    }

    return 0;
}

//*****************************************************************************
//
//! Initialize LWIP stack and network interface
//!
//! \param  None
//!
//! \return 0 on success, negative on error
//!
//*****************************************************************************
static int32_t initializeLWIPIfNeeded(void)
{
    UART_PRINT("Initializing network stack (LWIP)...\n\r");

    /* Initialize network stack (lwIP) */
    network_stack_init();

    /* Register network status callback for IP acquisition */
    network_stack_register_extra_status_callback(NetworkStatusCallback);

    /* Add station network interface */
    network_stack_add_if_sta();

    UART_PRINT("Network stack initialized.\n\r");

    /* Mark LWIP as initialized */
    g_LwipInitialized = true;

    return 0;
}

//*****************************************************************************
//
//! Connect to AP at Layer 2 only (no IP address acquisition)
//!
//! \param  None
//!
//! \return 0 on success, negative on error
//!
//*****************************************************************************
static int32_t wlanConnectLayer2Only(void)
{
    int32_t ret = 0;

    UART_PRINT("Connecting to AP (Layer 2 only): %s\n\r", PowerMeasure_appData.ssid);

    /* Connect to AP using runtime credentials from user input */
    ret = Wlan_Connect((const signed char *)PowerMeasure_appData.ssid,
                       strlen(PowerMeasure_appData.ssid),
                       NULL,  /* MAC address - NULL for any */
                       PowerMeasure_appData.securityType,
                       (const char *)PowerMeasure_appData.password,
                       strlen(PowerMeasure_appData.password),
                       0);  /* flags */
    if (ret < 0)
    {
        UART_PRINT("[ERROR] Failed to initiate connection\n\r");
        return ret;
    }

    /* Wait ONLY for Layer 2 connection (WiFi association), NOT for IP */
    while (!IS_CONNECTED(g_Status))
    {
        /* Blink LED to indicate waiting for connection */
        GPIO_toggle(CONFIG_GPIO_LED_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    UART_PRINT("Connected to AP (Layer 2 only - no IP address)\n\r");

    /* Turn on LED to indicate connected */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    return 0;
}

//*****************************************************************************
//
//! SLEEP Mode - Demonstrates WiFi and host low power sleep
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void sleepMode(void)
{
    int32_t ret;

    UART_PRINT("Entering SLEEP mode...\n\r");
    UART_PRINT("WiFi radio and host CPU will sleep\n\r");
    UART_PRINT("Press reset to restart\n\r\n");

    /* Configure WiFi radio power settings (no AP connection needed) */
    WlanPowerSave_e psMode = WLAN_STATION_POWER_SAVE_MODE;
    ret = Wlan_Set(WLAN_SET_POWER_SAVE, &psMode);
    if (ret < 0)
    {
        UART_PRINT("[WARN] Failed to set power save mode\n\r");
    }

    WlanPowerManagement_e pmMode = POWER_MANAGEMENT_ELP_MODE;
    ret = Wlan_Set(WLAN_SET_POWER_MANAGEMENT, &pmMode);
    if (ret < 0)
    {
        UART_PRINT("[WARN] Failed to set power management mode\n\r");
    }

    UART_PRINT("WiFi radio configured for low power mode\n\r");

    /* NOTE: LWIP is disabled in SLEEP mode (never initialized) */

    /* Close UART to reduce power consumption */
    UART_PRINT("Closing UART for low power mode...\n\r");
    UART_PRINT("Start measure current. \n\r");
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to flush
    DeinitTerm();

    /* Stay in low power mode indefinitely */
    /* vTaskDelay() blocks the task, allowing FreeRTOS tickless idle */
    /* Power policy will transition CPU to SLEEP/LPDS automatically */
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(SLEEP_IDLE_TIME_MSEC));
    }
}

//*****************************************************************************
//
//! Always Connected Mode - Maintain connection with power save enabled
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void alwaysConnected(void)
{
    int32_t ret;

    UART_PRINT("Always Connected mode\n\r");
    UART_PRINT("WiFi will wake for DTIMs to maintain connection\n\r");
    UART_PRINT("Press reset to restart\n\r\n");

    /* Sub-mode specific initialization and connection */
    switch (PowerMeasure_appData.alwaysConnectedUseCase)
    {
        case AlwaysConnected_HostAlwaysOn:
        case AlwaysConnected_HostSleepWithLWIP:
        {
            /* Initialize LWIP for sub-modes 2.1 and 2.2 */
            ret = initializeLWIPIfNeeded();
            if (ret < 0)
            {
                UART_PRINT("[ERROR] LWIP initialization failed\n\r");
                return;
            }

            /* Full Layer 3 connection with DHCP */
            ret = wlanConnect();
            if (ret < 0)
            {
                UART_PRINT("[ERROR] Connection failed\n\r");
                return;
            }
            break;
        }

        case AlwaysConnected_HostAlwaysSleep:
        {
            /* NO LWIP initialization for sub-mode 2.3 */
            /* Layer 2 only connection (no IP, no DHCP) */
            ret = wlanConnectLayer2Only();
            if (ret < 0)
            {
                UART_PRINT("[ERROR] Layer 2 connection failed\n\r");
                return;
            }
            break;
        }
        default:
            UART_PRINT("[ERROR] Invalid Always Connected sub-mode\n\r");
            return;
    }

    /* Configure power settings AFTER connection completes */
    ret = configSimplelinkToUseCase();
    if (ret < 0)
    {
        UART_PRINT("[ERROR] Failed to configure power settings\n\r");
        return;
    }

    /* Sub-mode specific behavior */
    switch (PowerMeasure_appData.alwaysConnectedUseCase)
    {
        case AlwaysConnected_HostAlwaysOn:
        {
            /* Sub-mode 2.1: Host Always On - LWIP active, CPU active */
            UART_PRINT("Sub-mode 2.1: Host Always On\n\r");
            UART_PRINT("LWIP: Active | CPU: Active (polling loop)\n\r");
            UART_PRINT("Closing UART for low power mode...\n\r");
            UART_PRINT("Start measure current. \n\r");
            vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to flush
            DeinitTerm();

            /* CPU stays active - no vTaskDelay(), continuous polling loop */
            /* LWIP remains active and processes network traffic normally */
            while (1)
            {
                /* Active polling loop - keeps CPU from sleeping */
                /* This is intentional to measure power with host always on */
            }
        }
        break;

        case AlwaysConnected_HostSleepWithLWIP:
        {
            /* Sub-mode 2.2: Host Sleep with LWIP - LWIP active, CPU sleeps */
            UART_PRINT("Sub-mode 2.2: Host Sleep with LWIP\n\r");
            UART_PRINT("LWIP: Active | CPU: Sleeping (wakes for LWIP ticks)\n\r");
            UART_PRINT("Closing UART for low power mode...\n\r");
            UART_PRINT("Start measure current. \n\r");
            vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to flush
            DeinitTerm();

            /* CPU sleeps via vTaskDelay(), wakes for LWIP timer events */
            /* LWIP remains active and processes network traffic */
            while (1)
            {
                vTaskDelay(pdMS_TO_TICKS(SLEEP_IDLE_TIME_MSEC));
            }
        }
        break;

        case AlwaysConnected_HostAlwaysSleep:
        {
            /* Sub-mode 2.3: Host Always Sleep - NO LWIP (never initialized) */
            UART_PRINT("Sub-mode 2.3: Host Always Sleep\n\r");
            UART_PRINT("LWIP: Disabled | CPU: Deep Sleep | Layer 2 only\n\r");

            /* NOTE: LWIP is disabled (never initialized) */

            UART_PRINT("Closing UART for low power mode...\n\r");
            UART_PRINT("Start measure current. \n\r");
            vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to flush
            DeinitTerm();

            /* CPU sleeps deeply - no LWIP timer wake events */
            /* Only WiFi DTIM interrupts will wake the CPU */
            while (1)
            {
                vTaskDelay(pdMS_TO_TICKS(SLEEP_IDLE_TIME_MSEC));
            }
        }
        break;
        default:
            UART_PRINT("[ERROR] Invalid Always Connected sub-mode\n\r");
            break;
    }
}

//*****************************************************************************
//
//! Get BLE Advertisement parameters from user
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
static void getBLEAdvParams(void)
{
    char input[16];

    UART_PRINT("\n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("        BLE Advertisement Configuration\n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("\n\r");

    /* Get advertisement interval */
    UART_PRINT("Enter advertisement interval in ms (20-10240, step 0.625) [100]: ");
    GetCmd(input, sizeof(input), "");
    UART_PRINT("\n\r");

    float interval_ms;
    uint32_t interval_us;

    if (strlen(input) > 0)
    {
        interval_ms = (float)atof(input);
    }
    else
    {
        interval_ms = 100.0f; /* Default 100ms */
    }

    /* Convert to microseconds for validation and storage */
    interval_us = (uint32_t)(interval_ms * 1000.0f);

    /* Validate interval range (BLE spec: 20ms to 10.24s) */
    if (interval_us < 20000)
    {
        interval_us = 20000;
        UART_PRINT("[WARN] Interval too low, set to minimum 20 ms\n\r");
    }
    else if (interval_us > 10240000)
    {
        interval_us = 10240000;
        UART_PRINT("[WARN] Interval too high, set to maximum 10240 ms\n\r");
    }

    /* Store in microseconds */
    PowerMeasure_appData.bleAdvInterval_us = interval_us;

    UART_PRINT("\n\r");
    UART_PRINT("BLE Advertisement Configuration Complete\n\r");
    UART_PRINT("  Interval: %.2f ms\n\r", (float)interval_us / 1000.0f);
    UART_PRINT("  PHY: 1M (Legacy)\n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("\n\r");
}

//*****************************************************************************
//
//! Set BLE Connection parameters
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
static void setBLEConnParams(void)
{
    /* 100 ms advertising interval for connection mode */
    PowerMeasure_appData.bleAdvInterval_us = 100000;
}

//*****************************************************************************
//
//! Start BLE Advertisement mode with configured parameters
//!
//! \param  None
//!
//! \return 0 on success, negative on error
//!
//*****************************************************************************
int32_t bleStartAdvertisement(void)
{
    int32_t ret;
    ExtAdvCfg_t advCfg;
    ExtAdvEnable_t advEnable;

    UART_PRINT("BLE Advertisement mode: Legacy 1M advertising\n\r");
    UART_PRINT("Interval: %.2f ms\n\r", (float)PowerMeasure_appData.bleAdvInterval_us / 1000.0f);
    UART_PRINT("Press reset to restart\n\r\n");

    /* Configure power settings for BLE */
    ret = configSimplelinkToUseCase();
    if (ret < 0)
    {
        UART_PRINT("[ERROR] Failed to configure power settings\n\r");
        return ret;
    }

    /* Open BLE transport */
    BleIf_OpenTransport();

    /* Start NimBLE host (also enables the controller) */
    ret = nimble_host_start();
    if (ret != 0)
    {
        UART_PRINT("[ERROR] Failed to start NimBLE host: %ld\n\r", ret);
        return ret;
    }

    UART_PRINT("NimBLE host started\n\r");

    /* Configure extended advertising */
    memset(&advCfg, 0, sizeof(advCfg));
    advCfg.instance = 0;
    advCfg.legacy = 1;                                      /* Legacy PDU */
    advCfg.interval_ms = PowerMeasure_appData.bleAdvInterval_us / 1000;  /* Convert us to ms */
    advCfg.prim_phy = 1;                                    /* 1M PHY */
    advCfg.sec_phy = 0;                                     /* Not used for legacy */

    ret = nimble_host_ext_adv_cfg(&advCfg);
    if (ret != 0)
    {
        UART_PRINT("[ERROR] Failed to configure advertising: %ld\n\r", ret);
        return ret;
    }

    UART_PRINT("\n\rAdvertising configured\n\r");

    /* Enable advertising */
    memset(&advEnable, 0, sizeof(advEnable));
    advEnable.enable = 1;
    advEnable.instance = 0;
    advEnable.duration = 0;      /* Run forever */
    advEnable.max_events = 0;    /* No event limit */

    ret = nimble_host_ext_adv_enable(&advEnable);
    if (ret != 0)
    {
        UART_PRINT("[ERROR] Failed to enable advertising: %ld\n\r", ret);
    }

    return ret;
}

//*****************************************************************************
//
//! Stop BLE Advertisement mode with configured parameters
//!
//! \param  None
//!
//! \return 0 on success, negative on error
//!
//*****************************************************************************
int32_t bleStopAdvertisement(void)
{
    int32_t ret;
    ExtAdvEnable_t advEnable;
    
    UART_PRINT("\n\rAdvertising stopped\n\r");
    
    /* Enable advertising */
    memset(&advEnable, 0, sizeof(advEnable));
    advEnable.enable = 0;
    advEnable.instance = 0;
    advEnable.duration = 0;      /* Run forever */
    advEnable.max_events = 0;    /* No event limit */

    ret = nimble_host_ext_adv_enable(&advEnable);
    if (ret != 0)
    {
        UART_PRINT("[ERROR] Failed to disable advertising: %ld\n\r", ret);
    }

    return ret;
}
    
//*****************************************************************************
//
//! BLE Advertisement Mode - Legacy 1M advertisement
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void bleAdvertisementMode(void)
{
    int32_t ret = 0;

    ret = bleStartAdvertisement();
    if (ret < 0)
    {
        return;
    }

    UART_PRINT("Advertising started\n\r");
    UART_PRINT("Device is now advertising...\n\r");

    /* Close UART to reduce power consumption */
    UART_PRINT("Closing UART for low power mode...\n\r");
    UART_PRINT("Start measure current.\n\r");
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to flush
    DeinitTerm();

    /* Stay in advertising mode indefinitely */
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(SLEEP_IDLE_TIME_MSEC));
    }
}

//*****************************************************************************
//
//! BLE Connection Mode - Connect to central peer device
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void bleConnectionMode(void)
{
    int32_t ret;

	ret = osi_SyncObjCreate(&(bleConnSyncObj));
    if (ret != 0)
    {
        UART_PRINT("[ERROR] Failed to create sync object: %ld\n\r", ret);
        return;
    }

    register_extra_gap_event_cb(extraGapEventCallback);

    ret = bleStartAdvertisement();
    if (ret < 0)
    {
        return;
    }

    UART_PRINT("Device is now advertising, waiting for connection...\n\r");

    ret = osi_SyncObjWait(&(bleConnSyncObj), OSI_WAIT_FOREVER);
    if (OSI_OK != ret)
    {
        UART_PRINT("[ERROR] Failed to wait for connection event: %ld\n\r", ret);
        return;
    }

    bleStopAdvertisement();

    /* Close UART to reduce power consumption */
    UART_PRINT("Closing UART for low power mode...\n\r");
    UART_PRINT("Start measure current. \n\r");
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow UART to flush
    DeinitTerm();
    
    /* Stay in connection mode indefinitely */
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(SLEEP_IDLE_TIME_MSEC));
    }
}

//*****************************************************************************
//
//! FreeRTOS User Hook Functions enabled in FreeRTOSConfig.h
//!
//*****************************************************************************

//*****************************************************************************
//
//! \brief  Application defined hook (or callback) function - assert
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
    /* Handle FreeRTOS assertion */
    while (1) {}
}

//*****************************************************************************
//
//! \brief  Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationIdleHook(void)
{
    /* Handle application idle hook for Processor activity */
}

//*****************************************************************************
//
//! \brief  Application defined malloc failed hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook(void)
{
    /* Handle malloc failure */
    while (1) {}
}

//*****************************************************************************
//
//! \brief  Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    /* Handle stack overflow */
    while (1) {}
}
