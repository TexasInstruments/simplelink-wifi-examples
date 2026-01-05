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
/* Standard includes */
#include "network_terminal.h"
#include <stdlib.h>

/* Example Header files */
#include "cmd_parser.h"
#include "wlan_cmd.h"
#include "wlan_if.h"
#include "errors.h"
#include "uart_term.h"
#include "osi_kernel.h"
#include "network_lwip.h"
#include "FreeRTOS.h"
#include "ble_if.h"
#include "lwip/sockets.h"
#include "lwip_ping.h"
#include "dhcpserver.h"
//#define GENERATE_RANDOM_MAC
#ifdef GENERATE_RANDOM_MAC
#ifdef CC35XX
	#include "generate_mac_address.h"
#elif defined(CC33XX)
	#include <wlan_irq_adapt.h>
#endif 
#endif

#define ENT_EXAMPLE //when enterprise example is enabled the code size is increased  due to the inclusion of the certificate files

#ifdef ENT_EXAMPLE

#define FOrCE_CA_CRT_VERIFY
//to create binary certificate from .pem, perform :
//xxd -i ca.pem > ca_certificate.h
//xxd -i wifiuser.pem > client_certificate.h
//add 0 at the end of the blob
#include "client_certificate.h"
#include "client_private_key.h"
#include "ca_certificate.h"
#endif //ENT_EXAMPLE

#ifdef CC35XX
extern Bool_e g_wait_p2p_scan_complete;
extern OsiSyncObj_t p2p_find_stopped_syncObj;
#endif // CC35XX

//Indigo 
extern LoadCertiCmd_t client_certi;
extern LoadCertiCmd_t ca_certi;
extern LoadCertiCmd_t private_key_certi;

/* Application defines */
#ifdef CC35XX
#define WLAN_EVENT_TOUT             (20000)
#else
#define WLAN_EVENT_TOUT             (40000)
#endif
#define WLAN_WPS_TOUT               (121000)
#define WLAN_ENT_TOUT               (1000000)
#define MAX_SCAN_TRAILS             (10)
#define P2P_CONNECT_PRIORITY        (SPAWN_TASK_PRIORITY - 1)
#define P2P_STACK_SIZE              (2048)
#define P2P_REMOTE_DEVICE           ("StartScan")
#define P2P_DEVICE_TYPE             ("1-0050F204-1")
#define P2P_DEVICE_NAME             ("cc32xx_p2p_device")
#define LISTEN_CHANNEL              (11)
#define LISTEN_REGULATORY_CLASS     (81)
#define OPRA_CHANNEL                (6)
#define OPRA_REGULATORY_CLASS       (81)
#define TIMEOUT_SEM                 (5)
#define LPDS_WAKEUP_SW              (1)
#define MGMT                        (0)
#define CTRL                        (1)
#define DATA                        (2)
#define CC3x35_BIT                  (0x100000)

#define DEFAULT_SCAN_MAX_DWELL_TIME_PASSIVE_MSEC            (110)
#define DEFAULT_SCAN_MIN_DWELL_TIME_PASSIVE_MSEC            (110)
#define DEFAULT_SCAN_MAX_DWELL_TIME_ACTIVE_MSEC             (60)
#define DEFAULT_SCAN_MIN_DWELL_TIME_ACTIVE_MSEC             (25)

#define DEFAULT_SCAN_DFS_DWELL_TIME_PASSIVE_MSEC             (150)

typedef enum
{
    STA_SCAN,
    P2P_DEVICE_SCAN,
}scan_t;
/******************************************************************************
                  Security Type Macros
******************************************************************************/
//----------------------------------------------------------------
// Don't change these values, they are related to values in rsn.h
#define SECURITY_TYPE_BITMAP_OPEN            (0)
#define SECURITY_TYPE_BITMAP_WPA             (1 << 1)
#define SECURITY_TYPE_BITMAP_WPA2            (1 << 2)
#define SECURITY_TYPE_BITMAP_WPA3            (1 << 3)
#define SECURITY_TYPE_BITMAP_PMF_CAPABLE     (1 << 4)
#define SECURITY_TYPE_BITMAP_PMF_REQUIRED    (1 << 5)
//--------------------------
// Defines for the security type and PMF
#define SECURITY_TYPE_MASK                   (SECURITY_TYPE_BITMAP_OPEN  | SECURITY_TYPE_BITMAP_WPA | SECURITY_TYPE_BITMAP_WPA2 | SECURITY_TYPE_BITMAP_WPA3)
#define SECURITY_PMF_CAPABILITIES_MASK       (SECURITY_TYPE_BITMAP_PMF_CAPABLE | SECURITY_TYPE_BITMAP_PMF_REQUIRED)
//----------------------------------------------------------------

/******************************************************************************
                      LOCAL FUNCTION PROTOTYPES
******************************************************************************/
int32_t setStaticIPConfig(uint8_t* pIP,
                          uint8_t* pGw,
                          uint8_t* pDns);
int32_t sendConnectionReq(void);
int32_t ScanUsage(void *arg);
void setScanResultsConfig(uint8 maxNumOfResult);

int32_t printRoleIdGetUsage(void *arg);
int32_t ParseGetRoleId(void *arg, WlanRole_e* role);

#ifdef CC35XX
#define VENDOR_SPECIFIC_ID  0xdd
#define END_VENDOR_LIST     0
#define OUI_SIZE       3
#define MAX_IE_DATALEN 255

struct ie_vendor_node_t
{
    uint32_t oui;
    struct ie_vendor_node_t *next;
    uint8_t length;
    uint8_t *data;
};

typedef struct
{
  unsigned char oui[OUI_SIZE];
  unsigned char length;
  unsigned char data[MAX_IE_DATALEN];
} vendor_ie_t;

struct ie_vendor_node_t *g_vendor_ie[3] = {NULL};//sta offset 0, ap = offset 2


int set_vendor_ie(WlanRole_e role);


int32_t ParseSetpeerAgingTimeout(void *arg, uint32_t* timeOut);

#endif
/******************************************************************************
                      GLOBAL VARIABLES
******************************************************************************/
uint8_t DataFrames[] = { 0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78,
                         0x88, 0x98, 0xA8, 0xB8, 0xC8, 0xD8, 0xE8, 0xF8 };

uint8_t CtrlFrames[] = { 0x84, 0x94, 0xA4, 0xB4, 0xC4, 0xD4, 0xE4, 0xF4 };

uint8_t MgmtFrames[] = { 0x00, 0x01, 0x20, 0x30, 0x40, 0x50,
                         0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0 };

uint32_t ActiveNetIfBitMap = 0x00;



int csi_keep_reading = 0;
OsiSyncObj_t csi_thread_sync = NULL;
OsiThread_t csi_thread = NULL;

#ifdef CC35XX
    //prepare configuration for WPS connection
    char g_wpsConfigMethods[] = "virtual_display virtual_push_button keypad";
    const char g_manufacturer[64+1] = "TI";
    const char g_modelName[32+1] = "TI CC35XX";
    const char g_modelNumber[32+1] = "2025";
    const char g_serialNumber[32+1] = "20252025";
    const unsigned char g_uuid_string[16+1] = "0123456789123456";
    const char g_primaryDeviceType[8+1] = {0x00, 0x06, 0x00, 0x00, 0xf2, 0x04, 0x00, 0x01};
#endif

/******************************************************************************
                  Callback Functions
******************************************************************************/
#define AP_CONFIG
#ifdef AP_CONFIG
/*!
    \brief          isNetIFActive

    Check if network IF is already active

    \return         If network IF is already active return 1.
                    else return 0;

*/
int32_t isNetIFActive(void)
{
    return ( IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP) );
}

/*!
    \brief          Wlan role up AP callback.

    This routine shows how to role up the device in AP role.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdWlanRoleUpApCallback(void *arg)
{

    int32_t      ret = 0;
    RoleUpApCmd_t RoleUpApParams;
    void *apif = NULL;

    /* Call the command parser */
    memset(&RoleUpApParams, 0x0, sizeof(RoleUpApParams));
    ret = ParseRoleUpApCmd(arg , &RoleUpApParams);

    if(ret < 0)
    {
        printWlanRoleUpApUsage(arg);
        FreeRoleUpApCmd(&RoleUpApParams);
        return -1;
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
        return -1;
    }

    /* Check if network AP is already active */
    if (IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
        UART_PRINT("\n\rNetwork AP Is Already Active.\n\r");
        return -1;
    }

    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_AP_CONNECTION);
    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_PEER_CONNECTED);

#ifdef CC35XX
    {
    //prepare configuration for WPS connection
    RoleUpApParams.wpsParams.deviceName = (char *) g_modelName;
    RoleUpApParams.wpsParams.configMethods = (char *) g_wpsConfigMethods;
    RoleUpApParams.wpsParams.manufacturer = (char *) g_manufacturer;
    RoleUpApParams.wpsParams.modelName = (char *) g_modelName;
    RoleUpApParams.wpsParams.modelNumber = (char *) g_modelNumber;
    RoleUpApParams.wpsParams.serialNumber = (char *) g_serialNumber;
    RoleUpApParams.wpsParams.uuid = (uint8_t *) g_uuid_string;
    RoleUpApParams.wpsParams.deviceType = (uint8_t *) g_primaryDeviceType;

    }
#endif // CC35XX	

    network_stack_add_if_ap();

    os_sleep(1, 0);

    Report("\n\rSetting AP: SSID %s, channel: %d",
           RoleUpApParams.ssid, RoleUpApParams.channel);

    ret = Wlan_RoleUp(WLAN_ROLE_AP, &RoleUpApParams , OSI_WAIT_FOR_SECOND * 10);
    if (WlanError_GetType(ret) == WLAN_ERROR_TYPE__INVALID_PARAM_CHANNEL)
    {
        Report("\n\rFailed setting up AP on channel %d."
               "\n\rDevice doesn't support 5GHz band or "
               "channel is DFS (not supported).",
               RoleUpApParams.channel);
    }

    if (ret < 0)
    {
        network_stack_remove_if_ap();
        FreeRoleUpApCmd(&RoleUpApParams);
        return -1;
    }

    apif = network_get_ap_if();
    if(apif != NULL)
    {
        network_set_up(apif);
    }

    FreeRoleUpApCmd(&RoleUpApParams);

    SET_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_AP_BIT);
    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_AP_CONNECTION);
    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);

    app_CB.Role = WLAN_ROLE_AP;
    return ret;

}

/*!
    \brief          Prints Wlan AP role up command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanRoleUpCallback
*/
int32_t printWlanRoleUpApUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanRoleUpApStr);
    UART_PRINT(wlan_role_up_ap_UsageStr_first);
    UART_PRINT(wlan_role_up_ap_UsageStr_second);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_up_ap_DetailsStr);
#ifdef CC35XX
    UART_PRINT(wlan_role_up_ap_t_optionDetailsStr);
    UART_PRINT(wlan_role_up_ap_w_optionDetailsStr);
#endif
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

#ifdef CC35XX
int32_t printCreateVendoIEListUsage(void *arg)
{
    printRoleIdGetUsage(arg);
    return(0);
}

int32_t printDeleteVendoIEListUsage(void *arg)
{
    printRoleIdGetUsage(arg);
    return(0);
}

int32_t printaddVendoIEUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanAddVendorIEStr);
    UART_PRINT(wlan_AddVendorIE_UsageStr_first);
    UART_PRINT(wlan_AddVendorIE_UsageStr_second);

    printRoleIdGetUsage(arg);
    return(0);
}

int32_t printDeleteVendoIEUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanRemoveVendorIEStr);
    UART_PRINT(wlan_DeleteVendorIE_UsageStr_first);
    UART_PRINT(wlan_DeleteVendorIE_UsageStr_second);
    printRoleIdGetUsage(arg);
    return(0);
}



int32_t printConfiPeerAgingUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanConfigPeerAgingStr);
    UART_PRINT(wlan_ConfigPeerAging_UsageStr_first);
    UART_PRINT(wlan_ConfigPeerAging_UsageStr_second);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_ConfigPeerAging_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}


int32_t printWlanRoleUpP2PUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanRoleUpP2PStr);

    UART_PRINT(wlan_role_up_p2p_UsageStr_first);
    UART_PRINT(wlan_role_up_p2p_UsageStr_second);
    UART_PRINT(wlan_role_up_p2p_UsageStr_third);
    UART_PRINT(wlan_role_up_p2p_i_optionDetailsStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_up_p2p_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printWlanRoleDownP2PUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanRoleDownP2PStr);

    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_down_p2p_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printWlanP2PFindUsage(void *arg)
{

    scan_t scanType = P2P_DEVICE_SCAN;
    return( ScanUsage(&scanType));
}

int32_t printWlan2PConnectUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanP2PConnectStr);

    UART_PRINT(wlan_p2p_connect_UsageStr_first);
    UART_PRINT(wlan_p2p_connect_UsageStr_second);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_p2p_connect_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return(0);
}

int32_t printWlanP2PFindStopUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanP2PStopFindStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_p2p_find_stop_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return(0);
}

int32_t printWlanP2PGrpRemoveUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanP2PGrpRemoveStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_p2p_group_remove_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return(0);
}

int32_t printWlanP2PSetChannelUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanP2PSetchannelStr);

    UART_PRINT(wlan_role_up_group_remove_UsageStr_third);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_up_p2pSetChannel_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printWlanP2PGetChannelUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanP2PGetchannelStr);

    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_up_p2pGetChannel_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printWlanP2PListenUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanP2PListenStr);

    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_up_p2pListen_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printWlanP2PCancelUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanP2PCancelStr);

    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_up_p2pCancel_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

#endif

/*!
    \brief          Wlan role down AP callback.

    This routine shows how to role down the device.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdWlanRoleDownApCallback(void *arg)
{
    int32_t           ret = 0;
    ret = ParseCmd(arg);

    if(ret < 0)
    {
        printWlanRoleDownApUsage(arg);
        return -1;
    }

    network_stack_remove_if_ap();
    ret = Wlan_RoleDown(WLAN_ROLE_AP, WLAN_WAIT_FOREVER);
    if (ret < 0)
    {
        Report("\n\r _role_ap_down: Wlan_RoleDown Failed with error code: %d", ret);
        return ret;
    }

    CLEAR_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_AP_BIT);
    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_AP_CONNECTION);
    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_PEER_CONNECTED);
    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);

    app_CB.Role = WLAN_ROLE_RESERVED;
    return ret;
}

/*!
    \brief          Prints Wlan role dwon command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanRoleDownCallback
*/
int32_t printWlanRoleDownApUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanRoleDownApStr);
    UART_PRINT(wlan_role_down_ap_UsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_down_ap_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

#endif

/*!
    \brief          Wlan role up STA callback.

    This routine shows how to role up the device in STA role.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdWlanRoleUpStaCallback(void *arg)
{
#ifdef CC35XX
    os_sleep(2, 0);
#endif // CC35XX	

    /* Call the command parser */
    int32_t ret = 0;
    RoleUpStaCmd_t RoleUpStaParams;

    /* Call the command parser */
    memset(&RoleUpStaParams, 0x0, sizeof(RoleUpStaCmd_t));
    ret = ParseRoleUpStaCmd(arg, &RoleUpStaParams);

    if(ret < 0)
    {
        printWlanRoleUpStaUsage(arg);
        return -1;
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
        return -1;
    }

    /* Check if network station is already active */
    if (IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
        UART_PRINT("\n\rNetwork Station Is Already Active.\n\r");
        return -1;
    }

    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);

    // configuration required for wps */
#ifdef CC35XX

    //Set 2.4G and 5G for MX
    uint8_t sta_wifi_band =  (uint8_t)BAND_SEL_BOTH;
    Wlan_Set(WLAN_SET_STA_WIFI_BAND, &sta_wifi_band);


    RoleUpStaParams.wpsDisabled = FALSE; /* WPS is enabled by default */
    RoleUpStaParams.wpsParams.deviceName = (char *) g_modelName;
    RoleUpStaParams.wpsParams.configMethods = (char *) g_wpsConfigMethods;
    RoleUpStaParams.wpsParams.manufacturer = (char *) g_manufacturer;
    RoleUpStaParams.wpsParams.modelName = (char *) g_modelName;
    RoleUpStaParams.wpsParams.modelNumber = (char *) g_modelNumber;
    RoleUpStaParams.wpsParams.serialNumber = (char *) g_serialNumber;
    RoleUpStaParams.wpsParams.uuid = (uint8_t *) g_uuid_string;
    Report("\n\r wps model_name[%s]\n", RoleUpStaParams.wpsParams.modelName);
    Report("\n\r wps model_number[%s]\n", RoleUpStaParams.wpsParams.modelNumber);
    Report("\n\r wps serial_number[%s]\n", RoleUpStaParams.wpsParams.serialNumber);
    Report("\n\r wps uuid : 0x%x 0x%x 0x%x 0x%x -- 0x%x 0x%x 0x%x 0x%x\n",
    RoleUpStaParams.wpsParams.uuid[0],RoleUpStaParams.wpsParams.uuid[1],
    RoleUpStaParams.wpsParams.uuid[2],RoleUpStaParams.wpsParams.uuid[3],
    RoleUpStaParams.wpsParams.uuid[12],RoleUpStaParams.wpsParams.uuid[13],
    RoleUpStaParams.wpsParams.uuid[14],RoleUpStaParams.wpsParams.uuid[15]);


#endif // CC35XX

    network_stack_add_if_sta();//send callback to tcp

    //Wait for getting wlan role up response
    ret = osi_SyncObjWait(&(app_CB.CON_CB.staRoleupSyncObj), OSI_WAIT_FOR_SECOND * 10);
    if(OSI_OK != ret)
    {
        Report("\n\r[ERROR]cmdWlanRoleUpStaCallback: Failed waiting sync object\n\r");
        ASSERT_GENERAL(0);
        return ret;
    }

    ret = Wlan_RoleUp(WLAN_ROLE_STA, &RoleUpStaParams, WLAN_WAIT_FOREVER);

    if (ret < 0)
    {
        network_stack_remove_if_sta();
        Report("\n\r _role_sta_up: Wlan_RoleUp Failed with error code: %d", ret);
        return ret; //Return -1 to lwip is Wlan_RoleUp is failed
    }

    SET_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_STA_BIT);
    app_CB.Role = WLAN_ROLE_STA;

#ifdef CC35XX
    os_sleep(1, 0);
#elif defined(CC33XX)
    os_sleep(0, 50000);
#endif	



    return ret;
}

/*!
    \brief          Prints Wlan AP role up command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanRoleUpCallback
*/
int32_t printWlanRoleUpStaUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanRoleUpStaStr);
    UART_PRINT(wlan_role_up_sta_UsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_up_sta_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}


/*!
    \brief          Wlan role down callback.

    This routine shows how to role down the device.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdWlanRoleDownStaCallback(void *arg)
{
    int32_t           ret = 0;
    ret = ParseCmd(arg);

    if(ret < 0)
    {
        printWlanRoleDownStaUsage(arg);
        return -1;
    }
#ifndef CC35XX
    if(IS_STA_CONNECTED(app_CB.Status))
    {
        ret = Wlan_Disconnect(WLAN_ROLE_STA,NULL);
        if(ret == OK)
        {
            ret = osi_SyncObjWait(&(app_CB.CON_CB.disconnectEventSyncObj), OSI_WAIT_FOR_SECOND * 10);
            if(OSI_OK != ret)
            {
                Report("\n\r[ERROR]cmdWlanStopCallback: Failed waiting sync object\n\r");
                ASSERT_GENERAL(0);
                return ret;
            }
        }
    }
#endif 
   
    network_stack_remove_if_sta();

    //Wait for getting wlan role down response
    ret = osi_SyncObjWait(&(app_CB.CON_CB.staRoledownSyncObj), OSI_WAIT_FOR_SECOND * 10);
    if(OSI_OK != ret)
    {
        Report("\n\r[ERROR]cmdWlanRoleDownStaCallback: Failed waiting sync object (%d)\n\r", ret);
        ASSERT_GENERAL(0);
        return ret;
    }
    
    ret = Wlan_RoleDown(WLAN_ROLE_STA, WLAN_WAIT_FOREVER);
    if (ret < 0)
    {
        Report("\n\r[ERROR] Wlan_RoleDown Failed with error code: %d", ret);
        return ret;
    }
    CLEAR_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_STA_BIT);
    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_STA_CONNECTION);
    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);

    app_CB.Role = WLAN_ROLE_RESERVED;

    return ret;
}

/*!
    \brief          Prints Wlan role dwon command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanRoleDownCallback
*/
int32_t printWlanRoleDownStaUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanRoleDownStaStr);
    UART_PRINT(wlan_role_down_sta_UsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlan_role_down_sta_DetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Prints Start Ap command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanStartApCallback
 */
int32_t printWlanStartApUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(ap_start_str);
    UART_PRINT(ap_start_UsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(ap_start_DetailsStr);
    UART_PRINT(wlanConnect_s_optionDetailsStr);
    UART_PRINT(wlanConnect_t_optionDetailsStr);
    UART_PRINT(wlanConnect_p_optionDetailsStr);
    UART_PRINT(ap_start_h_optionDetailsStr);
    UART_PRINT(ap_start_txp_optionDetailsStr);
    UART_PRINT(ap_start_channel_optionDetailsStr);
    UART_PRINT(ap_start_l_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          WLAN connect callback.

    This routine shows how to Connect to an AP.
    As a part of the connection process,
    we also demonstrate how to set a static IP,
    Static default gateway address and static DNS.
    Enterprise credentials are also shown for users who wish to connect
    to an enterprise network.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             setStaticIPConfig

 */
int32_t cmdWlanConnectCallback(void *arg)
{
    int32_t ret = 0;
    ConnectCmd_t ConnectParams;
    WlanEapConnectParams_t EapConnectParams;
    uint8_t flags =0;
    uint8_t isEnt = 0;


    /* Call the command parser */
    memset(&ConnectParams, 0x0, sizeof(ConnectCmd_t));
    memset(&EapConnectParams, 0x0, sizeof(WlanEapConnectParams_t));

    ret = ParseConnectCmd(arg, &ConnectParams, &EapConnectParams, &isEnt);

    if(ret < 0)
    {
        FreeConnectCmd(&ConnectParams);
        return(-1);
    }


#ifdef ENT_EXAMPLE
    if(isEnt)
    {
        uint32_t   ca_cert_len = ca_certi.size;
        uint8_t*  ca_cert_pt =  (uint8_t*)ca_certi.certi;
        uint32_t   user_cert_len = client_certi.size;
        uint8_t*  user_cert_pt =  (uint8_t*)client_certi.certi;
        uint32_t   key_cert_len = private_key_certi.size;
        uint8_t*  key_cert_pt =  (uint8_t*)private_key_certi.certi;

#ifdef FOrCE_CA_CRT_VERIFY  //option not to verify the cA cert
        EapConnectParams.pEap_ca_cert = (uint8_t*)ca_cert_pt;
        EapConnectParams.eap_ca_cert_len = ca_cert_len;//length without null terminator
#else
        EapConnectParams.pEap_ca_cert = NULL;
        EapConnectParams.eap_ca_cert_len = 0;//length without null terminator
#endif
        EapConnectParams.pEap_client_cert=(uint8_t*)user_cert_pt;
        EapConnectParams.eap_client_cert_len=user_cert_len;//length without null terminator

        EapConnectParams.pEap_private_key = (uint8_t*)key_cert_pt;
        EapConnectParams.eap_private_key_len = key_cert_len;//length without null terminator

        EapConnectParams.eapAnonUserLen = 0;
        os_memset(EapConnectParams.eapAnonymous, 0 , sizeof(EapConnectParams.eapAnonymous));

    }
#endif

   if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
   {
       UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
       return (-1);
   }

   if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
   {
       UART_PRINT("\n\rNo STA role up\n\r");
       return (-1);
   }

   Report("\r\n wlan_connect, ssid:%s secType:%d key:%s", ConnectParams.ssid,ConnectParams.secParams.Type,ConnectParams.secParams.Key);

   osi_SyncObjClear(&(app_CB.CON_CB.connectEventSyncObj));

#ifndef ENT_EXAMPLE
    /* Connect to AP */
    ret = Wlan_Connect((const signed char *)(ConnectParams.ssid),
                       strlen((const char *)(ConnectParams.ssid)),
                       ConnectParams.bssid,//macAddr, empty by default
                       ConnectParams.secParams.Type,
                       (const char *)(ConnectParams.secParams.Key),
                       ((ConnectParams.secParams.Key != NULL) ? strlen((const char *)ConnectParams.secParams.Key) : 0),
                       0);
#else

    if(isEnt)
    {
        flags |= WLAN_CONNECT_FLAG_ENTERPRISE_CONNECT;
    }


    ret = Wlan_Connect_extended((const signed char *)(ConnectParams.ssid),
            strlen((const char *)(ConnectParams.ssid)),
            ConnectParams.bssid,//macAddr, empty by default
            ConnectParams.secParams.Type,
            (const char *)(ConnectParams.secParams.Key),
            ((ConnectParams.secParams.Key != NULL) ? strlen((const char *)ConnectParams.secParams.Key) : 0),
            flags,
            &EapConnectParams);

    //note !! ! only After Wlan_Connect_extended is connected or disconnect
    //the below  can be freed in case they were dynamically allocated !!!
    //os_free(EapConnectParams.pEap_ca_cert),
    //os_free(EapConnectParams.pEap_client_cert)
    //os_free(EapConnectParams.pEap_private_key)
    //So add those line on the connect/ disconnect event
#endif
    ASSERT_AND_CLEAN_CONNECT(ret, WLAN_ERROR_MSG, &ConnectParams);


    /* Wait for connection events:
     * In order to verify that connection was successful,
     * we pend on two incoming events: Connected and Ip acquired.
     * The semaphores below are pend by this (Main) context.
     * They will be signaled once an asynchronous event
     * Indicating that the NWP has connected and acquired IP address is raised.
     * For further information, see this application read me file.
     */
    if(!IS_STA_CONNECTED(app_CB.Status))
    {
        if(isEnt)
        {
            ret = osi_SyncObjWait(&(app_CB.CON_CB.connectEventSyncObj),
                    WLAN_ENT_TOUT);
        }
        else if ((WLAN_SEC_TYPE_WPS_PBC == ConnectParams.secParams.Type) ||
                 (WLAN_SEC_TYPE_WPS_PIN == ConnectParams.secParams.Type))
        {
            // ret = osi_SyncObjWait(&(app_CB.CON_CB.connectEventSyncObj),
            //                      WLAN_WPS_TOUT);
        }
        else
        {
            ret = osi_SyncObjWait(&(app_CB.CON_CB.connectEventSyncObj),
                                   WLAN_EVENT_TOUT);
        }
        if(ret != 0)
        {
            if ((WLAN_SEC_TYPE_WPS_PBC != ConnectParams.secParams.Type) &&
                (WLAN_SEC_TYPE_WPS_PIN != ConnectParams.secParams.Type))
            {
                UART_PRINT("\n\r[wlan_connect app] : Timeout expired connecting to AP: %s\n\r",
                                       ConnectParams.ssid);
                Wlan_Disconnect(WLAN_ROLE_STA,NULL);
            }
            else
            {
                UART_PRINT("\n\r[wlan_connect app] : Timeout expired connecting to WPS AP\n\r");
            }
            FreeConnectCmd(&ConnectParams);
            return(-1);
        }
        else
        {
            UART_PRINT("\n\r[wlan_connect app] : connected !!!!");
        }

    }

    FreeConnectCmd(&ConnectParams);
    return(0);
}

/*!
    \brief          Prints WLAN connect command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanConnectCallback
 */
int32_t printWlanConnectUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanConnectStr);
    UART_PRINT(wlanConnectUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanConnectDetailsStr);
    UART_PRINT(wlanConnect_s_optionDetailsStr);
    UART_PRINT(wlanConnect_t_optionDetailsStr);
    UART_PRINT(wlanConnect_p_optionDetailsStr);
#ifdef CC35XX
    UART_PRINT(wlanConnect_e_optionDetailsStr);
    UART_PRINT(wlanConnect_i_optionDetailsStr);
#endif
    UART_PRINT(help_optaionDetails);
#ifdef CC35XX
    UART_PRINT(wlanConnect_ent_usageDetailsStr_1);
    UART_PRINT(wlanConnect_ent_usageDetailsStr_2);
    UART_PRINT(wlanConnect_ent_usageDetailsStr_3);
    UART_PRINT(wlanConnect_ent_usageDetailsStr_4);
#endif
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Disconnect callback.

    This routine shows how to disconnect a device from an AP/STA.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed to the
                                  parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

 */
int32_t cmdWlanDisconnectCallback(void *arg)
{
    int32_t          ret;

    /* After calling WlanDisconnect(),
     *    we expect WLAN disconnect asynchronous event.
     * Cleaning the former connection information from
     * the application control block
     * is handled in that event handler,
     * as well as getting the disconnect reason.
     */

    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    uint8_t help = FALSE;
    char    *macAddressStr = NULL;
    uint8_t pMacAddress[6] = {0};

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL) {
        help = TRUE;
    }

    while(token) {
        if(!strcmp(token, m_optionStr)) {
            macAddressStr = strtok(NULL, space_str);
            if (macAddressStr == NULL) {
                Report("[Cmd Parser] : Missing MAC address after -m option");
            return (-1);
            }
            ret = macAddressParse(macAddressStr, pMacAddress);
            if (ret < 0) {
                Report("\r\n[Cmd Parser] : Invalid MacAddress. Format xx:xx:xx:xx:xx:xx\n\r");
                return (-1);
            }
        }
        token = strtok(NULL, space_str);
    }

    if(IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
        ret = Wlan_Disconnect(WLAN_ROLE_STA,NULL);
        if(ret == OK){
            ret = osi_SyncObjWait(&(app_CB.CON_CB.disconnectEventSyncObj), OSI_WAIT_FOR_SECOND * 60);
            if(OSI_OK !=ret)
            {
                Report("\n\r[ERROR]cmdWlanDisconnectCallback: Failed waiting sync object\n\r");
                ASSERT_GENERAL(0);
                return ret;
            }
        }
    }
    else if (IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT)) {
        ret = Wlan_Disconnect(WLAN_ROLE_AP,pMacAddress);
    }
    else
    {
        ret = 0;
        Report("\n\r[ERROR]cmdWlanDisconnectCallback: No STA role up\n\r");
    }

    return(ret);
}

/*!
    \brief          Prints Disconnect command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanDisconnectCallback
*/
int32_t printWlanDisconnectUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanDisconnectStr);
    UART_PRINT(wlanDisconnectUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanDisconnectDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief         Sets the early termination option for connection scans.

    \param          arg        Points to command line buffer containing 0 (disable) or 1 (enable).

    \return         Upon successful completion, the function shall return 0. In case of error, returns a negative value.

    \sa             cmdWlanSetScanEarlyTerminationCallback
*/
int32_t cmdWlanSetScanEarlyTerminationCallback(void *arg)
{
    int32_t ret;
    uint8_t enable;

    ret = parseSetEarlyTerminationArgs(arg, &enable);
    if (ret != 0)
    {
        return ret;
    }

    ret = Wlan_Set(WLAN_SET_CONNECTION_SCAN_EARLY_TERMINATION, &enable);

    if (ret == 0)
    {
        Report("\n\rEarly Termination successfully %s.\n\r", 
               enable ? "Enabled" : "Disabled");
    } else
    {
        Report("\n\rFailed to set Early Termination\n\r");
    }
    return ret;
}

/*!
    \brief          Prints Set Early Termination command help menu.

    \param          arg     -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanSetScanEarlyTerminationCallback
 */
int32_t printSetEarlyTermUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanSetEarlyTermStr);
    UART_PRINT(setEarlyTermUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetEarlyTermDetailsStr);
    UART_PRINT(wlanSetEarlyTerm_e_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief         Gets the early termination option for connection scans.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0. In case of error, returns a negative value.

    \sa             cmdWlanGetScanEarlyTerminationCallback
*/
int32_t cmdWlanGetScanEarlyTerminationCallback(void *arg)
{
    int32_t ret;
    uint8_t enabled;
    ret = ParseCmd(arg);
    if (ret < 0)
    {
        printGetEarlyTermUsage(arg);
        return ret;
    }

    ret = Wlan_Get(WLAN_GET_CONNECTION_SCAN_EARLY_TERMINATION, &enabled);

    if (ret == 0)
    {
        Report("\n\rEarly Termination is %s.\n\r", 
               enabled ? "Enabled" : "Disabled");
    } else
    {
        Report("\n\rFailed to get Early Termination setting, error: %d\n\r", ret);
    }
    return ret;
}

/*!
    \brief          Prints Get Early Termination command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanGetScanEarlyTerminationCallback
 */
int32_t printGetEarlyTermUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanGetEarlyTermStr);
    UART_PRINT(getEarlyTermUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanGetEarlyTermDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Prints scan command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdScanCallback
*/
int32_t ScanUsage(void *arg)
{
    scan_t *scanType = arg;

    Report(lineBreak);
    Report(usageStr);
    if(*scanType == STA_SCAN)
    {
        Report(scanStr);
    }
#ifdef CC35XX
    else
    {
        Report(wlanP2PFindStr);
    }
#endif
    Report(scanUsageStr);
    Report(descriptionStr);
    Report(scanDetailsStr);
    Report(scan_n_optionDetailsStr);
    Report(help_optaionDetails);
    Report(lineBreak);
    Report(scan_Note_optionDetailsStr);
    Report(lineBreak);
    return(0);
}
/*!
    \brief          cmdAddVendorIE.

    This routine shows how to retrieve scan results form the NWP.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;


    \sa             ParseaddVendorIECmd

*/

#ifdef CC35XX

////////////////////  Vendor IE utility function /////////////////////////////////
/* Reads the AP vendor info IEs from the data structure of given role */
/* Parses it into one buffer and sends the command to FW */
static uint32_t get_oui(const unsigned char* oui)
{
    return oui[0] | (oui[1] << 8) | (oui[2] << 16);
}


/*
 *  MARS doesn't have an upper layer handling vendor IE,
 *  for testing purposes we define a sample DOT11_COMMON_VENDOR_IE_t here
 */

const vendor_ie_t example_1_vendor_ie =
{
        .oui = { 0x00, 0x02, 0xB3},
        .length = 9,
        .data = { 0x10, 0x01, 0x01,0x5A,0x02,0x00,0x64,0x00,0xC8}
};

const vendor_ie_t example_2_vendor_ie =
{
        .oui = {0x00, 0x12, 0x4B},
        .length = 3,
        .data = { 0xAA, 0xBB, 0xCC}
};

const vendor_ie_t example_3_vendor_ie =
{
        .oui = { 0x00, 0x02, 0xB3},
        .length = 9,
        .data = { 0x10, 0x01, 0x01,0x02,0x03,0x04,0x05,0x06,0x07}
};

const vendor_ie_t example_4_vendor_ie =
{
        .oui = {0x00, 0x12, 0x4B},
        .length = 3,
        .data = { 0xDD, 0xEE, 0xFF}
};

/* create empty list for vendor IE */
int32_t create_vendor_ie_list(WlanRole_e role)
{
    struct ie_vendor_node_t *ie_list_head;
    struct ie_vendor_node_t *node;

    node = g_vendor_ie[role];

    if(node != NULL)
    {
        Report ("\r\n Vendor IE list already exists, delete it first!");
    }


    ie_list_head = os_malloc(sizeof(struct ie_vendor_node_t));

    if (NULL == ie_list_head)
    {
        Report("\r\n allocate memory for vendor IE failed");
        return FALSE;
    }
    /* Initialize the vendor info element list */
    /* Put an empty node to mark the head */
    ie_list_head->oui = END_VENDOR_LIST;
    ie_list_head->next = ie_list_head;
    ie_list_head->data = NULL;


    g_vendor_ie[role] = ie_list_head;

    return TRUE;
}

int32_t clear_vendor_ie_list(WlanRole_e role)
{
    struct ie_vendor_node_t *node;
    struct ie_vendor_node_t *next = NULL;

    int32_t ret = 0;

    node = g_vendor_ie[role];
    /* Free IE linked list */
    while (END_VENDOR_LIST != node->oui)
    {
        next = node->next;
        os_free(node->data);
        os_free(node);
        node = next;
    }

    node->next = node;


    return ret;
}

int32_t delete_vendor_ie_list(WlanRole_e role)
{
    int32_t ret = 0;
    struct ie_vendor_node_t *node;
    wlanSetVendorInfo_t vendor_info;


    Report("\r\nclear vendor ies called.\r\n");

    node = g_vendor_ie[role];

    if (END_VENDOR_LIST == node->next->oui)
    {
        /* List is empty so do nothing */
        goto out;
    }

    clear_vendor_ie_list(role);
    vendor_info.role_type = role;
    vendor_info.length = 0;
    while((ret = Wlan_Set(WLAN_SET_VENDOR_IE, &vendor_info))==WLAN_RET_OPER_IN_PROGRESS);


out:
    os_free(node);

    g_vendor_ie[role] = NULL;

    return ret;
}

int add_vendor_ie(WlanRole_e role, const vendor_ie_t *ie)
{
    int32_t ret = 0;
    uint32_t oui;
    uint8_t *vendor_data;
    struct ie_vendor_node_t *node;
    struct ie_vendor_node_t *vendor_node;

    /*  should Add given IE data as a vendor-specific (0xdd type) IE to
        802.11 Beacons and Probe Responses for the uAP interface here */
    Report("\r\nadd_vendor_ie function.\r\n");


    if (NULL == ie)
    {
        Report("\r\n invalid param ie for add vendor ie command.\r\n");
        ret = -1;
        goto out;
    }

    /* Check ie length */
    if ((0 == ie->length) || (252 < ie->length))
    {
        Report("\r\n invalid ie length for add vendor ie command.\r\n");
        ret = -1;
        goto out;
    }

    node = g_vendor_ie[role];

    if(node == NULL)
    {
        Report("\r\n error, Vendor IE list was not created!!!.\r\n");
        ret = -1;
        goto out;
    }
    vendor_node = os_malloc(sizeof(struct ie_vendor_node_t));
    vendor_data = os_malloc(ie->length + OUI_SIZE);
    if (NULL == vendor_data || NULL == vendor_node) {
        Report("\r\n malloc failed for add vendor ie.\r\n");
        ret = -1;
        goto out;
    }

    /* Construct vendor Information Element */
    oui = get_oui(ie->oui);
    vendor_node->oui = oui;
    vendor_node->length = ie->length + OUI_SIZE;

    os_memcpy(vendor_data, ie->oui, OUI_SIZE);
    os_memcpy(vendor_data + OUI_SIZE, ie->data, ie->length);
    vendor_node->data = vendor_data;

    while (true)
    {
        if (END_VENDOR_LIST == node->oui)
        {
            /* Add vendor data to list */
            vendor_node->next = node->next;
            node->next = vendor_node;
            break;
        }
        else if (oui == node->oui)
        {
            /* Replace existing vendor data */
            node->length = vendor_node->length;
            os_free(node->data);
            node->data = vendor_data;
            os_free(vendor_node);
            break;
        }

        node = node->next;
    }

    /* Parse list and send to FW */
    ret = set_vendor_ie(role);

out:
    return ret;
}

int delete_vendor_ie(WlanRole_e role,const unsigned char *oui)
{
    int32_t ret = 0;
    uint32_t oui_num;
    struct ie_vendor_node_t *prev;
    struct ie_vendor_node_t *node;

    /*  should delete a previously added based on oui vendor-specific IE here */
    Report("\r\n delete vendor ie called.\r\n");

    if (NULL == oui)
    {
        Report("\r\ninvalid param oui for delete vendor ie command.\r\n");
        ret = -1;
        goto out;
    }

    oui_num = get_oui(oui);

    node = g_vendor_ie[role];

    /* Find and remove node in list with given OUI */
    while(true)
    {
        prev = node;
        node = node->next;

        if (oui_num == node->oui)
        {
            /* Found the node with the oui number, delete it */
            prev->next = node->next;
            os_free(node->data);
            os_free(node);
            break;
        }
        else if (END_VENDOR_LIST == node->oui)
        {
            /* Specified IE doesn't exist so do nothing */
            goto out;
        }
    }

    /* Parse list and send to FW */
    ret = set_vendor_ie(role);

out:
    return ret;
}




int set_vendor_ie(WlanRole_e role)
{
    int32_t ret = 0;
    uint8_t len;
    uint16_t offset = 0;
    struct ie_vendor_node_t *vendor_node ;
    wlanSetVendorInfo_t *vendor_info = os_malloc(sizeof(wlanSetVendorInfo_t));

    vendor_node = g_vendor_ie[role];

    if (NULL == vendor_info)
    {
        Report("\r\n error vendor list was not created");
        ret = -1;
        goto out;
    }

    vendor_node = vendor_node->next;

    /* Parse all the info elements into a single buffer */
    while (END_VENDOR_LIST != vendor_node->oui)
    {
        /* Get the length of the next info element */
        len = vendor_node->length;

        /* Check if the total length overflows the buffer */
        /* Account for the extra 2 bytes of id and length */
        if (offset + len + 2 > WLAN_BEACON_MAX_SIZE)
        {
            Report("\r\nvendor ie command template is too long.\r\n");
            ret = -1;
            goto out_free;
        }

        /* Write the vendor specific id and length before the IE */
        vendor_info->data[offset++] = VENDOR_SPECIFIC_ID;
        vendor_info->data[offset++] = len;

        /* Copy the next info element to the buffer */
        os_memcpy(vendor_info->data + offset, vendor_node->data, len);

        /* Advance the offset in the buffer and move to the next IE in the list */
        offset += len;
        vendor_node = vendor_node->next;
    }

    vendor_info->role_type = role;
    vendor_info->length = offset;

    {
    int i;
    Report("\r\n length:%d",vendor_info->length);
    for(i=0;i<vendor_info->length; i++)
        Report("%02x",vendor_info->data[i] );
    }
    /* Send command to FW */
    while((ret = Wlan_Set(WLAN_SET_VENDOR_IE, vendor_info))==WLAN_RET_OPER_IN_PROGRESS);


out_free:
    os_free(vendor_info);

out:
    return ret;
}


#endif
/*!
    \brief          Prints scan command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdScanCallback
*/
int32_t printScanUsage(void *arg)
{
    scan_t scanType = STA_SCAN;
    return( ScanUsage(&scanType));
}
/*!
    \brief          Scan callback.

    This routine shows how to retrieve scan results form the NWP.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note           If scans aren't active, this function triggers one scan
                    and later prints the results.

    \sa             ParseScanCmd

*/


int32_t cmdScan(void *arg, scan_t scanType)
{
    ScanCmd_t       ScanParams;
    int32_t ret = 0;
    /* Call the command parser */
    memset(&ScanParams, 0x0, sizeof(ScanParams));
    ScanParams.numOfentries = MAX_SSID_ENTRIES;

#ifdef CC35XX
    if(scanType == P2P_DEVICE_SCAN)
    {
        ScanParams.numOfentries = MAX_SSID_ENTRIES_FOR_P2P;//10
    }
#endif
    ret = ParseScanCmd(arg , &ScanParams);

    if(ret < 0)
    {
        if(scanType == STA_SCAN)
        {
            printScanUsage(NULL);
        }
#ifdef CC35XX
        else
        {
            printWlanP2PFindUsage(NULL);
        }
#endif
        return(-1);
    }

    /* Clear the results buffer */
    memset(&app_CB.gDataBuffer, 0x0, sizeof(app_CB.gDataBuffer));

#ifdef CC35XX
    scanCommon_t scanCommo;
    WlanRole_e      role;

    os_memset(&scanCommo, 0x0, sizeof(scanCommon_t));
    scanCommo.Band = BAND_SEL_BOTH;//options are BAND_SEL_ONLY_2_4GHZ , BAND_SEL_ONLY_5GHZ , BAND_SEL_BOTH

    if(scanType == STA_SCAN)
    {
        role = WLAN_ROLE_STA;
    }
#ifdef CC35XX
    else
    {
        role =  WLAN_ROLE_DEVICE; //P2P scan
    }
#endif

    Report("\r\n scan num of entries:%d role:%d band:%d",ScanParams.numOfentries, role, scanCommo.Band);
    ScanParams.index = 0;
    ret = Wlan_Scan(role,
            &scanCommo,
            ScanParams.numOfentries);
#elif defined(CC33XX)
        /* Get scan results from NWP -
    results would be placed inside the provided buffer */
    ret = Wlan_Scan(app_CB.Role,
                    &app_CB.gDataBuffer.netEntries[ScanParams.index],
                    ScanParams.numOfentries);
#endif

    if(ret == OK && (scanType == STA_SCAN))
    {
        ret = osi_SyncObjWait(&app_CB.eventCompletedScanObj, OSI_WAIT_FOR_SCAN_MS);
        if(OSI_OK != ret)
        {
            Report("\n\r[ERROR]cmdScanCallback: Failed waiting sync object \n\r");
            Report("\n\r[ERROR]cmdScanCallback: Probably FW crashed \n\r");
            return ret;
        }
    }
    return(ret);

}
/*!
    \brief          Scan callback.

    This routine shows how to retrieve scan results form the NWP.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note           If scans aren't active, this function triggers one scan
                    and later prints the results.

    \sa             ParseScanCmd

*/
int32_t cmdScanCallback(void *arg)
{

    /* Check if role id valid */
    if(!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
        Report("\n\r not role id STA \n\r");
        return -1;
    }
    return cmdScan(arg, STA_SCAN);
}

//this will consume more memory and will return more IEs per result
void setScanResultsConfig(uint8 maxNumOfResult)
{
    uint16_t scanResultsSize = maxNumOfResult;
    int32_t ret  = 0;
    while((ret = Wlan_Set(WLAN_SET_SCAN_RESULTS_SIZE, &scanResultsSize))==WLAN_RET_OPER_IN_PROGRESS);
}

/*!
    \brief          Prints Get MacAddress command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdSetMacAddressCallback
 */

int32_t printGetMacAddressUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(GetMacAddressStr);
    UART_PRINT(getMacAddressUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanGetMacAddressDetailsStr);
    UART_PRINT(wlanGetMacAddress_i_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Get MacAdress.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

int32_t cmdGetMacAddressCallback(void *arg)
{
    WlanMacAddress_t macAddressParams;
    int16_t    ret = 0;
    memset(&macAddressParams, 0, sizeof(WlanMacAddress_t));

    /* Call the command parser */
    uint32_t tmpRoleId;
    ret = ParseGetMacAddressCmd(arg, &tmpRoleId);

    if (0 == ret)
    {
        macAddressParams.roleType = tmpRoleId;
    }

    if(ret < 0 || 
        (( WLAN_ROLE_STA != macAddressParams.roleType && 
           WLAN_ROLE_AP  !=  macAddressParams.roleType &&
           WLAN_ROLE_DEVICE !=  macAddressParams.roleType)))
    {
        printGetMacAddressUsage(arg);
        return(0);
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
        return (-1);
    }

    if ((!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT)) &&
        (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT)) &&
        (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_DEVICE_BIT)))
    {
        UART_PRINT("\n\rNo role up\n\r");
        return (-1);
    }

    if (((WLAN_ROLE_AP     ==  macAddressParams.roleType) && (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT)))  ||
        ((WLAN_ROLE_STA    ==  macAddressParams.roleType) && (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))) ||
        ((WLAN_ROLE_DEVICE ==  macAddressParams.roleType) && (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_DEVICE_BIT))) )
    {
        UART_PRINT("\n\rEither role is up\n\r");
        return (-1);
    }

    ret = Wlan_Get(WLAN_GET_MACADDRESS, (void *)&macAddressParams);
    ASSERT_ON_ERROR(ret, WLAN_ERROR_MSG);
    UART_PRINT(
        "\n\r[MAC ADDRESS] : %02x:%02x:%02x:%02x:%02x:%02x\n\r",
        macAddressParams.pMacAddress[0],
        macAddressParams.pMacAddress[1],
        macAddressParams.pMacAddress[2],
        macAddressParams.pMacAddress[3],
        macAddressParams.pMacAddress[4],
        macAddressParams.pMacAddress[5]);

    return (ret);
}


/*!
    \brief          Prints Set MacAddress command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdGetMacAddressCallback
 */

int32_t printSetMacAddressUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(SetMacAddressStr);
    UART_PRINT(setMacAddressUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetMacAddressDetailsStr);
    UART_PRINT(wlanSetMacAddress_i_optionDetailsStr);
    UART_PRINT(wlanSetMacAddress_m_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Set MacAdress.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

int32_t cmdSetMacAddressCallback(void *arg)
{
    WlanMacAddress_t macAddressParams;
    int16_t    ret = 0;
    memset(&macAddressParams, 0, sizeof(WlanMacAddress_t));
    /* Call the command parser */
    uint32_t tmpRoleId;
    ret = ParseSetMacAddressCmd(arg, macAddressParams.pMacAddress, &tmpRoleId);

    if (0 == ret)
    {
        macAddressParams.roleType = tmpRoleId;
    }

    if(ret < 0 || (( WLAN_ROLE_STA != macAddressParams.roleType && WLAN_ROLE_AP !=  macAddressParams.roleType)))
    {
        printSetMacAddressUsage(arg);
        return(0);
    }
    ret = Wlan_Set(WLAN_SET_MACADDRESS, (void *)&macAddressParams);
    ASSERT_ON_ERROR(ret, WLAN_ERROR_MSG);

    return (ret);
}

/*!
    \brief          Netcfg Set Interface IP callback.

    This routine allows configuring an interface IP address.
    Whether it's an AP or STA and whether the IP is static or DHCP.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t cmdSetInterfaceIpCallback(void *arg)
{
    int32_t ret = 0;
    SetInterfaceIpParams_t params = {0};

    ret = ParseSetInterfaceIpCmd((char *)arg, &params);
    if (ret < 0)
    {
        printSetInterfaceIpUsage(arg);
        return ret;
    }

    if (params.roleType == WLAN_ROLE_STA)
    {
        if (params.ipMode == IP_DHCP)
        {
            Report("\n\rWarning: IP address parameters are ignored in DHCP mode (not static IP mode). Device will receive IP address automatically once connected to AP.\n\r");
            network_stack_set_dynamic_ip_if_sta();
            return ret;
        }
        else if (params.ipMode == IP_STATIC)
        {
            if (params.setDhcpServerAddress)
            {
                network_stack_set_static_ip_if_sta(htonl(params.ipAddress),
                                               htonl(params.netmask),
                                               htonl(params.gateway));
                return ret;
            }
        }
    }
    else if (params.roleType == WLAN_ROLE_AP)
    {
        if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
        {
            UART_PRINT("\n\rError! Network AP Is not Active.\n\r");
            return -1;
        }

        if (params.ipMode == IP_DHCP)
        {
            network_stack_set_dhcp_server_if_ap(0);

            if (params.setDhcpServerAddress)
            {
                network_stack_set_dynamic_ip_if_ap(htonl(params.ipAddress),
                                                   htonl(params.netmask),
                                                   htonl(params.gateway));
            }

            network_stack_set_dhcp_server_if_ap(1);
            return ret;
        }
        else if (params.ipMode == IP_STATIC)
        {
            if (params.setDhcpServerAddress)
            {
                network_stack_set_static_ip_if_ap(htonl(params.ipAddress),
                                              htonl(params.netmask),
                                              htonl(params.gateway));
                return ret;
            }
        }
    }
    SHOW_WARNING(-1, CMD_ERROR);
    return -1;
}


int32_t printSetInterfaceIpUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(SetInterfaceIpStr);
    UART_PRINT(SetInterfaceIpUsageStr);
    UART_PRINT(SetInterfaceIpDetailsStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    
    return(0);
}

/*!
    \brief          Netcfg Get Interface IP callback.

    This routine gets network configurations of a specific
    requested interface.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t cmdGetInterfaceIpCallback(void *arg)
{
    int32_t ret = 0;
    WlanRole_e roleType;
    uint32_t ipAddress = 0, netmask = 0, gateway = 0, dhcp = 0;
    SetInterfaceIpParams_t *params = NULL;

    ret = ParseGetInterfaceIpCmd((char *)arg, &roleType);
    if (ret < 0)
    {
        printGetInterfaceIpUsage(arg);
        return ret;
    }

    params = os_malloc(sizeof(SetInterfaceIpParams_t));
    if (params == NULL)
    {
        Report("\n\rMemory allocation failed.\n\r");
        return -1;
    }
    os_memset(params, 0x0, sizeof(SetInterfaceIpParams_t));

    ret = network_stack_get_if_ip(roleType,
                                  &ipAddress,
                                  &netmask,
                                  &gateway,
                                  &dhcp);
    if (ret < 0)
    {
        Report("\n\rFailed to get Interface IP Parameters\n\r");
        os_free(params);
        return ret;
    }

    params->ipAddress = htonl(ipAddress);
    params->netmask = htonl(netmask);
    params->gateway = htonl(gateway);
    Report("\n\n\rInterface IP Parameters");
    Report("\n\rIP Address: ");
    PrintIPAddress(FALSE, &params->ipAddress);
    Report("\n\rNetmask Address: ");
    PrintIPAddress(FALSE,&params->netmask);
    Report("\n\rGateway Address: ");
    PrintIPAddress(FALSE,&params->gateway);
    if (dhcp)
    {
        Report("\n\rIP is Dhcp\n\r");
    }
    else
    {
        Report("\n\rIP is Static\n\r");
    }
    os_free(params);
    return ret;
}

/*!
    \brief          Prints Get Interface IP command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdGetInterfaceIpCallback
 */

int32_t printGetInterfaceIpUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(GetInterfaceIpStr);
    UART_PRINT(GetInterfaceIpUsageStr);
    UART_PRINT(GetInterfaceIpDetailsStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Netcfg set DHCP server configurations callback.

    This routine allows setting the DHCP server configurations.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;
*/

int32_t cmdSetDhcpServerCallback(void *arg)
{
    int32_t ret = 0;
    uint32_t leaseTime = 0, startIp = 0, endIp = 0;
    uint32_t dhcp = 0;
    struct dhcps_lease lease = {0};

    /* In order to set DHCP server configurations AP role must
       be up and active, and hence DHCP server must be enabled. */
    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
        UART_PRINT("\n\rError! Network AP Is not Active.\n\r");
        return -1;
    }

    /* Check if DHCP is enabled */
    ret = network_stack_get_if_ip(WLAN_ROLE_AP,
                                  NULL, NULL, NULL,
                                  &dhcp);
    if (!dhcp || (ret < 0))
    {
        ret = -1;
        UART_PRINT("\n\rDHCP is not enabled.\n\r");
        return ret;
    }

    ret = ParseSetDhcpServerCmd((char *)arg,
                                    &leaseTime,
                                    &startIp,
                                    &endIp);
    if (ret < 0)
    {
        SHOW_WARNING(-1, CMD_ERROR);
        printSetDhcpServerUsage(arg);
        return ret;
    }

    ret = network_stack_set_dhcp_server_if_ap(0);
    if (ret < 0)
    {
        SHOW_WARNING(-1, CMD_ERROR);
        Report("AP Interface Error\n\r");
        return ret;
    }

    lease.enable = TRUE;
    lease.start_ip.addr = htonl(startIp);
    lease.end_ip.addr = htonl(endIp);
    ret = wifi_softap_set_dhcps_lease(&lease);
    if (ret == TRUE) // Address config succeeded - now set lease time
    {
        ret = wifi_softap_set_dhcps_lease_time(leaseTime);
        if (ret == TRUE)  // Should always succeed (validated in parser)
        {
            Report("\n\rDHCP configuration applied successfully\n\r");
        }
    }

    else
    {
        SHOW_WARNING(-1, CMD_ERROR);
        Report("\n\rInvalid parameters - server running with previous settings\n\r");
    }

    ret = network_stack_set_dhcp_server_if_ap(1);
    if (ret < 0)
    {
        SHOW_WARNING(-1, CMD_ERROR);
        Report("\n\rAP Interface Error\n\r");
        return ret;
    }
    return ret;
}

/*!
    \brief          Netcfg get DHCP callback.

    This routine retrieves current DHCP server configurations.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t cmdGetDhcpServerCallback(void *arg)
{
    int32_t ret = 0;
    DhcpParams_t *params = NULL;
    struct dhcps_lease lease = {0};

    params = os_malloc(sizeof(DhcpParams_t));
    if (params == NULL)
    {
        Report("Memory allocation failed.\n\r");
        os_free(params);
        return -1;
    }
    os_memset(params, 0, sizeof(DhcpParams_t));

    ret = wifi_softap_get_dhcps_lease(&lease);
    if (ret <= 0)
    {
        Report("\n\rAP role is not up.\r\n");
        printGetDhcpServerUsage(arg);
        os_free(params);
        return -1;
    }
    params->leaseTime = wifi_softap_get_dhcps_lease_time();
    params->startAddress =  htonl(lease.start_ip.addr);
    params->endAddress =  htonl(lease.end_ip.addr);
    Report("\n\n\rDHCP Parameters: \n\rLease Time: %d\n\r" ,params->leaseTime);
    Report("Start Address: ");
    PrintIPAddress(FALSE, &params->startAddress);
    Report("\n\rEnd Address: ");
    PrintIPAddress(FALSE,&params->endAddress);
    return(0);
}

/*!
    \brief          Prints Set DHCP Server command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdGetPsModeCallback
 */

int32_t printSetDhcpServerUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(SetDhcpServerStr);
    UART_PRINT(SetDhcpServerUsageStr);
    UART_PRINT(SetDhcpServerDetailsStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);

}

/*!
    \brief          Prints Get DHCP Server command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdGetPsModeCallback
 */

int32_t printGetDhcpServerUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(GetDhcpServerStr);
    UART_PRINT(GetDhcpServerUsageStr);
    UART_PRINT(GetDhcpServerDetailsStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);

}

/*!
    \brief          Prints Get Power save mode command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdGetPsModeCallback
 */

int32_t printGetPsModeUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(GetPsModeStr);
    UART_PRINT(getPsModeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanGetPsModeDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Get Power save mode.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

int32_t cmdGetPsModeCallback(void *arg)
{
    int16_t         ret = 0;
    WlanPowerSave_e currentPsMode;

    ret = Wlan_Get(WLAN_GET_POWER_SAVE,(void *)&currentPsMode);

    char *psMode;

    if(!ret)
    {
        switch(currentPsMode)
        {
        case WLAN_STATION_AUTO_PS_MODE:
        {
            psMode = "Auto Power save";
        }break;
        case WLAN_STATION_ACTIVE_MODE:
        {
            psMode = "Active mode";
        }break;
        case WLAN_STATION_POWER_SAVE_MODE:
        {
            psMode = "Power save mode";
        }break;
        default:
        {
            Report("\n\rPower save mode invalid!\n\r");
            return ret;
        }

        }
        Report("\n\r[PS MODE] current PS mode is %s", psMode);
    }

    return (ret);
}


/*!
    \brief          Prints Set Power save mode command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdSetPsModeCallback
 */

int32_t printSetPsModeUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(SetPsModeStr);
    UART_PRINT(setPsModeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetPsModeDetailsStr);
    UART_PRINT(wlanSetPsMode_m_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}


int32_t printWlanSetScanDwellTimeUsage(void *arg);
/*!
    \brief          Parse set scan dwell time


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          id              -  power save mode opiton.



    \return         Upon successful completion, the function shall return the RoleId number.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             ParseSetScanDwellTimeCmd
 */
int32_t ParseSetScanDwellTimeCmd(void *arg, WlanScanDwellTime_t *dwell_times)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token,  min_dwell_active_optionStr))
        {
            strId = strtok(NULL, space_str);
            dwell_times->min_dwell_time_active_msec = atoi((const char*) strId);
        }
        else if(!strcmp(token,  max_dwell_active_optionStr))
        {
            strId = strtok(NULL, space_str);
            dwell_times->max_dwell_time_active_msec = atoi((const char*) strId);
        }
        else if(!strcmp(token,  min_dwell_passive_optionStr))
        {
            strId = strtok(NULL, space_str);
            dwell_times->min_dwell_time_passive_msec = atoi((const char*) strId);
        }
        else if(!strcmp(token,  max_dwell_passive_optionStr))
        {
            strId = strtok(NULL, space_str);
            dwell_times->max_dwell_time_passive_msec = atoi((const char*) strId);       
        }
        else if(!strcmp(token,  dfs_dwell_passive))
        {
            strId = strtok(NULL, space_str);
            dwell_times->dwell_time_dfs_passive_msec = atoi((const char*) strId);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printWlanSetScanDwellTimeUsage(NULL);
        return (-1);
    }

/*
    //check if role id valid
    if ((*mode < 0) || (*mode > 2))
    {
        Report("\r\n[Cmd Parser] : Invalid mode. Range [0-2]\n\r");
        return (-1);
    }
    */
    return (0);
}



/*!
    \brief          Parse set power save mode command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          id              -  power save mode opiton.



    \return         Upon successful completion, the function shall return the RoleId number.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdSetPsModeCallBack
 */
int32_t ParseSetPsModeCmd(void *arg, WlanPowerSave_e *mode)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, m_optionStr))
        {
            strId = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printSetPsModeUsage(NULL);
        return (-1);
    }

    *mode = atoi((const char*) strId);

    //check if role id valid
    if ((*mode < 0) || (*mode > 2))
    {
        Report("\r\n[Cmd Parser] : Invalid mode. Range [0-2]\n\r");
        return (-1);
    }
    return (0);
}

/*!
    \brief          Set Power save mode.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

int32_t cmdSetPsModeCallback(void *arg)
{
    int16_t         ret = 0;
    WlanPowerSave_e PsMode;

    ret = ParseSetPsModeCmd(arg, &PsMode);

    if(ret < 0)
    {
        return ret;
    }

    ret = Wlan_Set(WLAN_SET_POWER_SAVE, (void *)&PsMode);

    if(ret < 0)
    {
        Report("\n\r[PS MODE] Failed to set PS mode %d\n\r", PsMode);
    }
    else
    {
        Report("\n\r[PS MODE] Successfully set PS mode %d\n\r", PsMode);
    }

    return (ret);
}


int32_t printSetPmModeUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(SetPmModeStr);
    UART_PRINT(setPmModeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetPmModeDetailsStrAlwaysActive);
    UART_PRINT(wlanSetPmModeDetailsStrPowerDown);
    UART_PRINT(wlanSetPmModeDetailsStrELP);
    UART_PRINT(wlanSetPmMode_m_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Parse set power management mode command.


    \param          arg            -  Points to command line buffer.
                                      Contains the command line typed by user.
    \param          mode           -  power management mode option.

    \return         Upon successful completion, the function shall return success (0).
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdSetPmModeCallBack
 */
int32_t ParseSetPmModeCmd(void *arg, WlanPowerManagement_e *mode)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, m_optionStr))
        {
            strId = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printSetPmModeUsage(NULL);
        return (-1);
    }

    *mode = atoi((const char*) strId);

    //check if PM mode valid
    if ((*mode < 0) || (*mode > 2))
    {
        Report("\r\n[Cmd Parser] : Invalid mode. Range [0-2]\n\r");
        return (-1);
    }
    return (0);
}

/*!
    \brief          Set Power management mode.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

int32_t cmdSetPmModeCallback(void *arg)
{
    int16_t               ret = 0;
    WlanPowerManagement_e PmMode;

    ret = ParseSetPmModeCmd(arg, &PmMode);

    if(ret < 0)
    {
        return ret;
    }

    ret = Wlan_Set(WLAN_SET_POWER_MANAGEMENT, (void *)&PmMode);

    if(ret < 0)
    {
        Report("\n\r[PM MODE] Failed to set PM mode %d\n\r", PmMode);
    }
    else
    {
        Report("\n\r[PM MODE] Successfully set PM mode %d\n\r", PmMode);
    }

    return (ret);
}

int32_t printSetLsiUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(SetLsiStr);
    UART_PRINT(setLsiUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetLsiDetailsStr);
    UART_PRINT(wlanSetLsi_n_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);

}

int32_t cmdSetWsocPrimaryCallback(void *arg)
{
    int16_t         ret = 0;
    WlanConnectivityFWSlot_t   WsocSlot;

    ret = ParseSetWsocPrimaryCmd(arg, &WsocSlot);

    if( (ret < 0) || (WsocSlot.Connectivityslot > 2) || (WsocSlot.Connectivityslot == 0) )
    {
        printSetWsocPrimaryUsage(arg);
        return ret;
    }

    ret = Wlan_Set(WLAN_SET_PRIMARY_CONNECTIVITY_FW, (void *)&WsocSlot);

    return (ret);
}

int32_t printSetWsocPrimaryUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(SetWsocPrimaryStr);
    Report("\t");
    Report(SetWsocPrimaryUsageStr);
    Report(descriptionStr);
    Report(SetWsocPrimaryDetailsStr);
    Report(help_optaionDetails);
    Report(lineBreak);
    return (0);
}

/*!
    \brief          Parse set power management mode command.


    \param          arg            -  Points to command line buffer.
                                      Contains the command line typed by user.
    \param          mode           -  power management mode option.

    \return         Upon successful completion, the function shall return success (0).
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdSetPmModeCallBack
 */
int32_t ParseSetLsiCmd(void *arg, WlanLongSleepInterval *LsiParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, n_optionStr))
        {
            strId = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printSetLsiUsage(NULL);
        return (-1);
    }

    LsiParams->ListenInterval = atoi((const char*) strId);

    //check if PM mode valid
    if ((LsiParams->ListenInterval < 1) || (LsiParams->ListenInterval > 10))
    {
        Report("\r\n[Cmd Parser] : Invalid mode. Range [1-10]\n\r");
        return (-1);
    }

    if (LsiParams->ListenInterval == 1)
    {
        LsiParams->WakeUpEvent = WAKE_UP_EVENT_DTIM;
    }
    else
    {
        LsiParams->WakeUpEvent = WAKE_UP_EVENT_N_DTIM;
    }

    return (0);
}

int32_t cmdSetLsiCallback(void *arg)
{
    int16_t               ret = 0;
    WlanLongSleepInterval LsiParams;
    ret = ParseSetLsiCmd(arg, &LsiParams);

    if(ret < 0)
    {
        return ret;
    }

    ret = Wlan_Set(WLAN_SET_LSI, (void *)&LsiParams);

    if(ret < 0)
    {
        Report("\n\r[SET LSI] Failed to set LSI %d\n\r", LsiParams.ListenInterval);
    }
    else
    {
        Report("\n\r[SET LSI] Successfully set LSI to %d\n\r", LsiParams.ListenInterval);
    }

    return (ret);
}

#ifdef CC35XX
 /* WLan Test Sleep */
int32_t printSleepTestUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(SleepTestStr);
    UART_PRINT(SleepTestUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(sleepTestDetailsStr);
    UART_PRINT(sleepTest_t_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
} 


/*!
    \brief          Parse test sleep period command.


    \param          arg            -  Points to command line buffer.
                                      Contains the command line typed by user.
    \param          SleepDuration  -  sleep duration in seconds.

    \return         Upon successful completion, the function shall return success (0).
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa
 */
int32_t ParseTestSleepCmd(void *arg, uint32_t *SleepDuration)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, t_optionStr))
        {
            strId = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printSleepTestUsage(NULL);
        return (-1);
    }

    *SleepDuration = atoi((const char*) strId);

    return (0);
}

#include <ti/drivers/UART2.h>
#include "ti_drivers_config.h"

extern UART2_Handle uartHandle;

int32_t cmdSleepTestCallback(void *arg)
{
    int32_t ret = 0;
    uint32_t SleepDuration;
    UART2_Params params;

    ret = ParseTestSleepCmd(arg, &SleepDuration);

    if(ret < 0)
    {
        return ret;
    }

    if (0 == ret)
    {
        UART2_close(uartHandle);
        osi_Sleep(SleepDuration);

        UART2_Params_init(&params);
        params.baudRate = 115200;
        uartHandle = UART2_open(CONFIG_UART2_0, &params);
    }
    else
    {
        Report("\n\rWlan test sleep failed: %d. please refer errors.h \n\r", ret);
    }

    return ret;
}
#endif

/*!
    \brief          Prints Get FW version command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdGetFwVerCallback
 */

int32_t printGetFwVerUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(GetFwVerStr);
    UART_PRINT(getFwVerUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanGetFwVerDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Get FW version.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

int32_t cmdGetFwVerCallback(void *arg)
{
    int16_t         ret = 0;
    WlanFWVersions_t wlanVer = {0};
    WlanSPVersions_t spVer = {0};

    ret = Wlan_Get(WLAN_GET_FWVERSION,(void *)&wlanVer);

    if(!ret)
    {
        /* Print firmware version if read success */
        UART_PRINT("\r\nFirmware version: %d.%d.%d.%d\r\n",
                        wlanVer.major_version,
                        wlanVer.minor_version,
                        wlanVer.api_version,
                        wlanVer.build_version);

        UART_PRINT("Phy version: %d.%d.%d.%d.%d.%d.%d.%d\r\n",
                        wlanVer.phy_version[7],
                        wlanVer.phy_version[6],
                        wlanVer.phy_version[5],
                        wlanVer.phy_version[4],
                        wlanVer.phy_version[3],
                        wlanVer.phy_version[2],
                        wlanVer.phy_version[1],
                        wlanVer.phy_version[0]);

        ret = Wlan_Get(WLAN_GET_SPVERSION,(void *)&spVer);

        /* Print service pack version if read success */
        if(!ret)
        {
            UART_PRINT("Service pack firmware version: %d.%d.%d.%d\r\n",
                         spVer.major_version,
                         spVer.minor_version,
                         spVer.revision_version,
                         spVer.build_version);

            UART_PRINT("Service pack phy version: %d.%d.%d.%d.%d.%d.%d.%d\r\n",
                         spVer.phy_version[7],
                         spVer.phy_version[6],
                         spVer.phy_version[5],
                         spVer.phy_version[4],
                         spVer.phy_version[3],
                         spVer.phy_version[2],
                         spVer.phy_version[1],
                         spVer.phy_version[0]);
        }
    }
    else
    {
        UART_PRINT("Version read failure with error %d\r\n", ret);
    }

    return (ret);
}

/*!
    \brief          Prints Wlan start command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanStop
 */

int32_t printWlanStartUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanStartStr);
    UART_PRINT(wlanStartUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanStartDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Wlan Stop.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */
extern void WlanStackEventHandler(WlanEvent_t *pWlanEvent);
int32_t cmdWlanStartCallback(void *arg)
{

    int32_t ret = 0;

    ret = Wlan_Start(WlanStackEventHandler);

#ifdef GENERATE_RANDOM_MAC
#ifdef CC35XX
#warning This is temporary patch for set random mac address while using FPGA

    const uint8 address2[MAC_ADDRESS_LEN] = {0};
    WlanMacAddress_t macAddressParams;
    uint8_t bdAddress[MAC_ADDRESS_LEN];
    
    macAddressParams.roleType = WLAN_ROLE_STA;
    Wlan_Get(WLAN_GET_MACADDRESS, (void *)&macAddressParams);

    if(!os_memcmp(macAddressParams.pMacAddress, address2, MAC_ADDRESS_LEN - 1))
    {
        os_memcpy(macAddressParams.pMacAddress, gStaMacAddress, MAC_ADDRESS_LEN);
        Wlan_Set(WLAN_SET_MACADDRESS, &macAddressParams);
    }

    macAddressParams.roleType = WLAN_ROLE_AP;
    Wlan_Get(WLAN_GET_MACADDRESS, (void *)&macAddressParams);

    if(!os_memcmp(macAddressParams.pMacAddress, address2, MAC_ADDRESS_LEN -1))
    {
        os_memcpy(macAddressParams.pMacAddress, gApMacAddress, MAC_ADDRESS_LEN);
        Wlan_Set(WLAN_SET_MACADDRESS, &macAddressParams);
    }

    os_memcpy(bdAddress, gBleMacAddress, MAC_ADDRESS_LEN);
    BleIf_SetBdAddr(bdAddress);


#endif
#endif 
#ifdef CC35XX
    //Enable ELP in FW
    if (ret == OSI_OK)
    {
        uint32_t powerManagement = (uint32_t)POWER_MANAGEMENT_ELP_MODE;
        Wlan_Set(WLAN_SET_POWER_MANAGEMENT, &powerManagement);
    }
#endif
    if (0 == ret)
    {
        SET_BIT_IN_BITMAP(ActiveNetIfBitMap, NET_IF_IS_UP);
        Report("\n\rWlan start success!\n\r");
        {
            uint32_t powerManagement = (uint32_t)POWER_MANAGEMENT_ELP_MODE;
            ret = Wlan_Set(WLAN_SET_POWER_MANAGEMENT, &powerManagement);
            {
                WlanCtrlBlk_t CtrlBlkParam;
                CtrlBlkParam.TxSendPaceThresh = 1;
                CtrlBlkParam.TransmitQOnTxComplete = 1;
                CtrlBlkParam.TxSendPaceTimeoutMsec = 1;
                ret = Wlan_Set(WLAN_SET_TX_CTRL, &CtrlBlkParam);
            }
        }
    }
    else
    {
        Report("\n\rWlan start failed: %d. please refer errors.h \n\r", ret);
    }


    return ret;
}


/*!
    \brief          Prints Wlan stop command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanStart
 */

int32_t printWlanStopUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanStopStr);
    UART_PRINT(wlanStopUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanStopDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}



/*!
    \brief          Wlan Stop.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

int32_t cmdWlanStopCallback(void *arg)
{

    int32_t ret = 0;
    StopCmd_t       StopParams;

    /*
    if (DEVICE_OFF == wlan_GetStateWlan())
	//if (DEVICE_OFF == GetState_WL())	
    {
        Report("\n\rWlan already stopped !!!\n\r");
        return -1;
    }*/
    /* Call the command parser */
    memset(&StopParams, 0x0, sizeof(StopParams));
    ret = ParseStopCmd(arg , &StopParams);

    if(ret < 0)
    {
        printWlanStopUsage(arg);
        return(-1);
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        Report("\n\rWlan already stopped !!!\n\r");
        return -1;
    }

    if(IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
        Report("\n\rcmdWlanRoleDownStaCallback, role id 0 \n\r");
        cmdWlanRoleDownStaCallback(" ");
        os_sleep(1, 0);
    }

    if(IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
        Report("\n\rcmdWlanRoleDownApCallback, role id 2 \n\r");
        cmdWlanRoleDownApCallback(" ");
        os_sleep(1, 0);
    }

    ret = Wlan_Stop(StopParams.isRecovery);
    if (0 == ret)
    {
        CLEAR_BIT_IN_BITMAP(ActiveNetIfBitMap, NET_IF_IS_UP);
    }
    else
    {
        Report("\n\r   Wlan stop failed: %d. please refer errors.h\n\r", ret);
    }
    return ret;
}

/*!
    \brief          Send ETHr Buffer.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */

//TODO add cmdSendEtherCallback to the  menu
int32_t cmdSendEtherCallback(WlanRole_e role, uint8_t *inbuf, uint32_t inbuf_len,uint32_t flags)
{

    int32_t ret = 0;


    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        Report("\n\rWlan  stopped !!!\n\r");
        return -1;
    }

    if(!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
        Report("\n\r not role id STA \n\r");
        return -1;
    }


    ret = Wlan_EtherPacketSend(role, inbuf, inbuf_len,flags);

    if (0 == ret)
    {
        //Report("\n\r cmdSendEtherCallback: buffer was send successfully\n\r");
    }
    else
    {
        Report("\n\rError!! cmdSendEtherCallback:buffer failed to send ethernet packet\n\r");
    }

    return ret;
}

int32_t cmdtestCallback(void *arg)
{

#if 0 //TODO to remove
    uint32_t ip = 0xf00000a;
    Report("test \n\r");
    extern void socket_connect(uint32_t ip);
    cmdWlanRoleUpStaCallback(" ");
    {
        osi_Sleep(1);
    }
    cmdWlanConnectCallback(" -s \"Elad_SSID\" -t OPEN");
    while(isIp == 0)
    {
        osi_uSleep(500 * 1000); //500 mili
    }
    socket_connect(ip);
#endif
    return 0;

}


int32_t printtestStopUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(testStr);
    UART_PRINT(testUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(testDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Check sub-frame type validity.

    This function takes a byte representing a sub-frame type defined by
    802.11 MAC and checks to make sure it's a valid frame type.

    \param          pSubFrameType   -   Points to byte representing frame type.

    \return         Upon successful completion (valid frame), the function
                    shall return 0.
                    In case of failure (invalid frame), this function
                    would return -1;

    \sa             cmdCreateFilterCallback

*/
int32_t CheckSubFrameType(uint8_t *pSubFrameType)
{
    int32_t  ret = -1;
    uint8_t i ;

    for(i = 0; i < sizeof(MgmtFrames) ; i++)
    {
        if (MgmtFrames[i] == *(pSubFrameType))
        {
            return(0);
        }
    }

    for(i = 0; i < sizeof(DataFrames); i++)
    {
        if(DataFrames[i] == *(pSubFrameType))
        {
            return(0);
        }
    }

    for(i = 0; i < sizeof(CtrlFrames) ; i++)
    {
        if (CtrlFrames[i] == *(pSubFrameType))
        {
            return(0);
        }
    }

    return(ret);
}


/*!
    \brief          Print sub-frames type.

    This function print the various bytes represnting sub-frame types
    from a table.

    \return         Upon successful completion, the function shall return 0.

    \sa             CheckSubFrameType, cmdCreateFilterCallback

*/
void printFrameSubTyps(void)
{
    uint8_t i = 0;

    UART_PRINT("\n\rManagement sub frame types \n\r");
    for(i = 0; i < sizeof(MgmtFrames) ; i++)
    {
        UART_PRINT("0x%02x : %s \n\r", MgmtFrames[i], MangmentFrames_str[i] );
    }

    UART_PRINT("\n\rControl sub frame types \n\r");
    for(i = 0; i < sizeof(CtrlFrames) ; i++)
    {
        UART_PRINT("0x%02x: %s \n\r", CtrlFrames[i], CtrlFrames_str[i]);
    }

    UART_PRINT("\n\rData sub frame types \n\r");
    for(i = 0; i < sizeof(DataFrames) ; i++)
    {
        UART_PRINT("0x%02x : %s \n\r", DataFrames[i], DataFrames_str[i]);
    }

    return;
}

/////************ csi *************////
int32_t printCsiEnableUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(csiEnableStr);
    UART_PRINT(csiEnableUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(csiEnableDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printCsiStopUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(csiStopStr);
    UART_PRINT(csiStopUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(csiStopDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printCsiDisableUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(csiDisableStr);
    UART_PRINT(csiDisableUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(csiDisableDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t printCsiGetResultsUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(csiGetResultsStr);
    UART_PRINT(csiGetResultsUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(csiGetResultsDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}



/*!
    \brief          Print CSI data to terminal UART

    \param          *pCsiData  -   Points to CSI data structure

    \return         NA

    \sa
 */
void CSI_InfoDump (WlanGetCSIData_t *pCsiData)
{
    Report("\n\rCSI data:");

    Report("\n\rMAC: %02x:%02x:%02x:%02x:%02x:%02x",
           pCsiData->tMacAddr[0],
           pCsiData->tMacAddr[1],
           pCsiData->tMacAddr[2],
           pCsiData->tMacAddr[3],
           pCsiData->tMacAddr[4],
           pCsiData->tMacAddr[5]);
    Report("\n\rRSSI %d timestamp 0x%x", pCsiData->rssi, pCsiData->timestamp);

}

/*!
    \brief          CSI thread main function responsible for pulling CSI results

    \param          *arg

    \return         NA

    \sa
 */
void csi_thread_entry(void *arg)
{
    WlanGetCSIData_t csiData;
    while (1)
    {
        osi_SyncObjWait(&csi_thread_sync, OSI_WAIT_FOREVER);
        while (csi_keep_reading)
        {

            Wlan_Get(WLAN_GET_CSI,&csiData);
            CSI_InfoDump(&csiData);
            os_sleep(1,0);
        }
    }
}

/*!
    \brief          Enabling the CSI feature and creating the CSI thread

    \param          *arg

    \return         NA

    \sa
 */
int32_t cmdCsiEnableCallback (void *arg)
{
    int32_t  ret;
    WlanCfgCsi_t csiCfg;
    csiCfg.csiEnable = TRUE;

    if (csi_thread == NULL)
    {
        osi_SyncObjCreate(&csi_thread_sync);

        osi_ThreadCreate(&csi_thread,     // Thread control block
                                  "",                       // Thread name
                                  4096,                     // STACK size
                                  4,                        // Priority
                                  (void*) csi_thread_entry, // Execute function
                                  NULL);                    // params
    }
    ret = Wlan_Set(WLAN_SET_CSI,(void *)&csiCfg);
    return(ret);
}


/*!
    \brief          Stop pulling CSI results

    \param          *arg

    \return         NA

    \sa
 */
int32_t cmdCsiStopCallback (void *arg)
{
    csi_keep_reading = 0;
    return(0);
}

/*!
    \brief          Disabling the CSI feature and deleting the CSI thread

    \param          *arg

    \return         NA

    \sa
 */
int32_t cmdCsiDisableCallback (void *arg)
{
    int32_t  ret;
    WlanCfgCsi_t csiCfg;
    csiCfg.csiEnable = FALSE;

    if (csi_thread == NULL)
    {
        // Thread doesn't exist, nothing to do
    }
    else
    {
        osi_ThreadDelete(&csi_thread);

        osi_SyncObjDelete(&csi_thread_sync);
    }
    ret = Wlan_Set(WLAN_SET_CSI,(void *)&csiCfg);
    return(ret);
}

/*!
    \brief          Start pulling CSI results

    \param          *arg

    \return         NA

    \sa
 */
int32_t cmdCsiGetResultsCallback (void *arg)
{
    csi_keep_reading = 1;
    osi_SyncObjSignal(&csi_thread_sync);

    return(0);
}
/////************ end of csi *************////
/*!
    \brief          Print scan AP security results.
    This function prints the security type and PMF capability of the scanned AP.

    \param apSecurity The security type and PMF capability of the AP.

    \return         None.
*/
void PrintScanApSecurityResults(uint32_t apSecurity)
{
    // Print which security the AP supports
    uint32_t apSecurityType = (apSecurity & SECURITY_TYPE_MASK);
    uint32_t keyMgmt;
    switch (apSecurityType)
    {
        case SECURITY_TYPE_BITMAP_OPEN :
                UART_PRINT(" %-4s      |", OPEN_str);
                break;
        case SECURITY_TYPE_BITMAP_WPA :
                UART_PRINT(" %-3s       |", WPA_str);
                break;
        case SECURITY_TYPE_BITMAP_WPA2 :
                UART_PRINT(" %-4s      |", WPA2_str);
                break;
        case SECURITY_TYPE_BITMAP_WPA3 :
                UART_PRINT(" %-4s      |", WPA3_str);
                break;
        case (SECURITY_TYPE_BITMAP_WPA | SECURITY_TYPE_BITMAP_WPA2):
                UART_PRINT(" %-8s  |", WPAWPA2_str);
                break;
        case (SECURITY_TYPE_BITMAP_WPA2 | SECURITY_TYPE_BITMAP_WPA3):
                UART_PRINT(" %-9s |", WPA2WPA3_str);
                break;
        default :
                break;
    }

    // Print if the AP supports PMF
    if (0 != (apSecurity & SECURITY_TYPE_BITMAP_PMF_REQUIRED))
    {
        UART_PRINT(" %-8s |", REQUIRED_str);
    }
    else if (0 != (apSecurity & SECURITY_TYPE_BITMAP_PMF_CAPABLE))
    {
        UART_PRINT(" %-7s  |", CAPABLE_str);
    }
    else
    {
        UART_PRINT(" %-7s  |", DISABLE_str);
    }

    keyMgmt = apSecurity << 14 ;//RSN_WLAN_SCAN_RESULT_KEY_MGMT_POSITION;
    UART_PRINT(" % 0x%x  |", keyMgmt);

}


#ifdef CC35XX
/*!
    \brief          Prints start AP WPS command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdStartApWpsCallback
*/
int32_t printStartApWpsUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(startApWpsStr);
    UART_PRINT(startApWpsUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(startApWpsDetailsStr);
    UART_PRINT(startApWps_w_optionDetailsStr);
    UART_PRINT(startApWps_p_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Start AP WPS.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdStartApWpsCallback(void *arg)
{
    int32_t ret = 0;
    wlanWpsSession_t wpsSession;
    memset(&wpsSession, 0x0, sizeof(wpsSession));

    ret = ParseStartApWpsSessionCmd(arg, &wpsSession);
    if(ret < 0)
    {
        printStartApWpsUsage(NULL);
        return -1;
    }

    while((ret = Wlan_Set(WLAN_SET_WPS_SESSION, &wpsSession))==WLAN_RET_OPER_IN_PROGRESS);

    return ret;
}
#endif

/*****************************************************************************
                  Local Functions
*****************************************************************************/

/*!
    \brief          Print scan results.

    This function print the scan results neatly in a table.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdScancallback

*/
void printScanResults(uint32_t res_num)
{

    uint32_t    index;
    uint32_t    sub_index;
    uint32_t    sec_type;
    uint32_t    i;
    uint8_t     ssid_len;


    /* Print table column headers */
    UART_PRINT(lineBreak);
    printBorder('-', 106);
    UART_PRINT(lineBreak);
    UART_PRINT(
        "    |               SSID               |       BSSID       | RSSI  | Ch  | Hidden | Security  | PMF      |\n\r");
    printBorder('-', 106);
    UART_PRINT(lineBreak);

    /* Print the table */
    for(index = 0; index < MIN(res_num,WLAN_MAX_SCAN_COUNT); index++)
    {
        UART_PRINT(" %-2d ", index+1);

        /* In case the SSID length is 32 characters (the maximum valid size),
         * the NWP sends the SSID field without NULL terminating character.
         * In order to avoid printing a string which has no NULL terminated
         character,
         * print each character individually.
         */
        ssid_len = app_CB.gDataBuffer.netEntries[index].SsidLen;

        if (ssid_len < WLAN_SSID_MAX_LENGTH)
        {
            UART_PRINT("| %-32s | ", app_CB.gDataBuffer.netEntries[index].Ssid);
        }
        else
        {
            UART_PRINT("| ");
            for(i = 0; i < MIN(ssid_len,WLAN_SSID_MAX_LENGTH); i++)
            {
                UART_PRINT("%c", app_CB.gDataBuffer.netEntries[index].Ssid[i]);
            }
            UART_PRINT(" | ");
        }

        for(sub_index = 0; sub_index < WLAN_BSSID_LENGTH-1 ; sub_index++)
        {
            UART_PRINT
            ("%02x:", app_CB.gDataBuffer.netEntries[index].Bssid[sub_index]);
        }
        UART_PRINT("%02x |", app_CB.gDataBuffer.netEntries[index].Bssid[WLAN_BSSID_LENGTH-1]);

        UART_PRINT(" %-5d |", app_CB.gDataBuffer.netEntries[index].Rssi);

        UART_PRINT(" %-3d |", app_CB.gDataBuffer.netEntries[index].Channel);

        if ((0 == (app_CB.gDataBuffer.netEntries[index].SsidLen)) || (0 == (app_CB.gDataBuffer.netEntries[index].Ssid[0])))
        {
            Report(" YES    |");
        }
        else
        {
            Report(" NO     |");
        }

        sub_index = app_CB.gDataBuffer.netEntries[index].SecurityInfo;

        sec_type = WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(sub_index);

        PrintScanApSecurityResults(sec_type);

        UART_PRINT("\n\r");
    }

    printBorder('-', 106);
    UART_PRINT(lineBreak);

    return;
}

#ifdef CC35XX
const char g_p2p_cmd_modelName[32+1] = "TI_CC351XX";
char g_p2p_cmd_wpsConfigMethods[] = "push_button physical_display";
const char g_p2p_cmd_manufacturer[64+1] = "TI";
const char g_p2p_cmd_modelNumber[32+1] = "2025";
const char g_p2p_cmd_serialNumber[32+1] = "12345678";
const char g_p2p_cmd_uuid_string[16+1] = "1234567890123456";
char g_p2p_cmd_primaryDeviceType[8+1] = {0x01, 0x00, 0x00, 0x50, 0xf2, 0x04, 0x00, 0x01};

//////////////////////// P2P /////////////////////////////
/*!
    \brief          Wlan role up P2P callback.

    This routine shows how to role up of P2P role.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/

int32_t cmdWlanRoleUpP2PCallback(void *arg)
{

    int32_t      ret = 0;
    RoleUpStaCmd_t RoleUpP2PParams;

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
        return -1;
    }

    /* Check if network Device is already active */
    if (IS_BIT_SET(ActiveNetIfBitMap, NET_IF_DEVICE_BIT))
    {
        Report("\n\rDevice role Is Already Active.\n\r");
        return -1;
    }

    if (IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
        Report("\n\rSTA role need to be down.\n\r");
        return -1;
    }
    if (IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
        Report("\n\rAP role need to be down.\n\r");
        return -1;
    }


    WlanCtrlBlk_t CtrlBlkParam;
    CtrlBlkParam.TxSendPaceThresh = 1;
    CtrlBlkParam.TransmitQOnTxComplete = 0;
    CtrlBlkParam.TxSendPaceTimeoutMsec = 16;
    Wlan_Set(WLAN_SET_TX_CTRL, &CtrlBlkParam);


    memset(&RoleUpP2PParams, 0x0, sizeof(RoleUpStaCmd_t));

    //set reg domain
    RoleUpP2PParams.countryDomain[0] = '0';
    RoleUpP2PParams.countryDomain[1] = '0';
    RoleUpP2PParams.countryDomain[2] = '\0';

    RoleUpP2PParams.wpsDisabled = FALSE; /* WPS is enabled by default */
    RoleUpP2PParams.wpsParams.deviceName = (char *) g_p2p_cmd_modelName;
    RoleUpP2PParams.wpsParams.configMethods = (char *) g_p2p_cmd_wpsConfigMethods;
    RoleUpP2PParams.wpsParams.manufacturer = (char *) g_p2p_cmd_manufacturer;
    RoleUpP2PParams.wpsParams.modelName = (char *) g_p2p_cmd_modelName;
    RoleUpP2PParams.wpsParams.modelNumber = (char *) g_p2p_cmd_modelNumber;
    RoleUpP2PParams.wpsParams.serialNumber = (char *) g_p2p_cmd_serialNumber;
    RoleUpP2PParams.wpsParams.uuid = (uint8_t *) g_p2p_cmd_uuid_string;
    RoleUpP2PParams.wpsParams.deviceType = (uint8_t *) g_p2p_cmd_primaryDeviceType;

    RoleUpP2PParams.p2pDeviceEnabled = TRUE;

    /* Set default parameters */
    RoleUpP2PParams.P2pParams.operChannel = 0;
    RoleUpP2PParams.P2pParams.operReg = 81;
    RoleUpP2PParams.P2pParams.listenChannel = 0;
    RoleUpP2PParams.P2pParams.listenReg = 81;
    RoleUpP2PParams.P2pParams.goIntent = 0;//can be 0-15


    /* Call the command parser */
    ret = ParseRoleUpP2PCmd(arg , &RoleUpP2PParams);

    if(ret < 0)
    {
        printWlanRoleUpP2PUsage(arg);
        return -1;
    }

    {
        WlanCtrlBlk_t CtrlBlkParam;
        CtrlBlkParam.TxSendPaceThresh = 1;
        CtrlBlkParam.TransmitQOnTxComplete = 0;
        CtrlBlkParam.TxSendPaceTimeoutMsec = 16;
        Wlan_Set(WLAN_SET_TX_CTRL, &CtrlBlkParam);

        uint8_t sta_wifi_band =  (uint8_t)BAND_SEL_BOTH;
        Wlan_Set(WLAN_SET_STA_WIFI_BAND, &sta_wifi_band);
    }

    SET_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_DEVICE_BIT);

    //station role up + p2p dev role up
    Report("Starting Device with p2p params: operational channel=%d, listen channel=%d, goIntent=%d\n\r",
            RoleUpP2PParams.P2pParams.operChannel,
            RoleUpP2PParams.P2pParams.listenChannel,
            RoleUpP2PParams.P2pParams.goIntent);

    Report("operational regulatory class=%d, listen regulatory class=%d\n\r",
            RoleUpP2PParams.P2pParams.operReg,
            RoleUpP2PParams.P2pParams.listenReg);

    ret = Wlan_RoleUp(WLAN_ROLE_DEVICE, &RoleUpP2PParams , OSI_WAIT_FOR_SECOND * 10);

    if(ret < 0)
    {
        CLEAR_BIT_IN_BITMAP(ActiveNetIfBitMap,NET_IF_DEVICE_BIT);
        return -1;
    }


    return ret;

}

#ifdef CC35XX
//////////////////////// vendor IE /////////////////////////////
/*!
    \brief          ucreate vendor IE lists

    This routine shows how to handle  vendor ie

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa

*/
int32_t cmdCreateVendorIEListCallback(void *arg)
{
    WlanRole_e role = WLAN_ROLE_AP;
    int32_t ret;

    ret = ParseGetRoleId(arg , &role);
    if(ret < 0)
    {
        printRoleIdGetUsage(arg);
        return 0;
    }
    ret = create_vendor_ie_list(role);
    if(ret)
    {
        Report("Creating vendor IE list for role:%d \n", role);
    }
    else
    {
        Report("Creating vendor IE list for role:%d failed !!! \n", role);
    }
    return ret;
}

/*!
    \brief          Delete Vendor IE list

    This routine shows how to handle  vendor ie

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             cmdSetVendorIE

*/

int32_t cmdDeleteVendorIEListCallback(void *arg)
{
    WlanRole_e role = WLAN_ROLE_AP;
    int32_t ret = 0;

    ret = ParseGetRoleId(arg , &role);
    if(ret < 0)
    {
        printRoleIdGetUsage(arg);
        return 0;
    }

    ret = delete_vendor_ie_list(role);
    if(ret)
    {
        Report("Deleting vendor IE list for role:%d \n", role);
    }
    else
    {
        Report("Deleting vendor IE list for role:%d failed !!! \n", role);
    }
    return ret;

}


/*!
    \brief         adds vendor IE.

    This routine shows how to handle  vendor ie

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             cmdSetVendorIE

*/
int32_t cmdAddVendorIECallback(void *arg)
{
    WlanRole_e role = WLAN_ROLE_AP;
    int32_t ret;
    struct ie_vendor_node_t *node;

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
        UART_PRINT("\n\rError! Network AP Is not Active.\n\r");
        return -1;
    }

    ret = ParseGetRoleId(arg , &role);

    if(ret < 0)
    {
        printRoleIdGetUsage(arg);
        return 0;
    }

    Report("\r\nAdding vendor IEs for role:%d\n", role);

    ret = create_vendor_ie_list(role);
    if(ret)
    {
        Report("Creating vendor IE list for role:%d \n", role);
    }
    else
    {
        Report("Creating vendor IE list for role:%d failed !!! \n", role);
        return -1;
    }

    if( (role == WLAN_ROLE_AP) && (!IS_BIT_SET(ActiveNetIfBitMap,NET_IF_AP_BIT)))
    {
        Report("\r\nError !! Adding vendor IEs for role:%d, role AP is not up !!!\n", role);
        return -1;
    }
    else if ((role == WLAN_ROLE_STA) && (!IS_BIT_SET(ActiveNetIfBitMap,NET_IF_STA_BIT)))
    {
        Report("\r\nError !!Adding vendor IEs for role:%d, role sTA is not up !!!\n", role);
        return -1;
    }

    node = g_vendor_ie[role];

    if(node == NULL)
    {
        Report("\r\n error, Vendor IE list was not created for this role!!!.\r\n");
        ret = -1;
        return ret;
    }

    if(role == WLAN_ROLE_AP)
    {
        ret = add_vendor_ie(role, &example_1_vendor_ie);
        if(ret< 0)
        {
            Report("\r\nAdd vendor IE role:%d  example 1 failed ret:0x%x!!\n", role, ret);
            return ret;
        }
        ret = add_vendor_ie(role, &example_2_vendor_ie);
        if(ret< 0)
        {
            Report("\r\nAdd vendor IE role:%d  example 2 failed, ret:0x%x !!\n", role, ret);
            return ret;
        }
    }
    else
    {
        ret = add_vendor_ie(role, &example_3_vendor_ie);
        if(ret< 0)
        {
            Report("\r\nAdd vendor IE role:%d  example 1 failed ret:0x%x!!\n", role, ret);
            return ret;
        }
        ret = add_vendor_ie(role, &example_4_vendor_ie);
        if(ret< 0)
        {
            Report("\r\nAdd vendor IE role:%d  example 2 failed, ret:0x%x !!\n", role, ret);
            return ret;
        }
    }
    Report("\r\n2 vendor IEs were added for role:%d !!\n", role);
    return ret;
}


/*!
    \brief          delete vendor IE.

    This routine shows how to handle  vendor ie

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             cmdSetVendorIE

*/
int32_t cmdDeleteVendorIECallback(void *arg)
{
    WlanRole_e role = WLAN_ROLE_AP;
    int32_t ret;

    ret = ParseGetRoleId(arg , &role);

    if(ret < 0)
    {
        printRoleIdGetUsage(arg);
        return 0;
    }

    Report("Delete vendor IEs for role:%d\n", role);

    ret = delete_vendor_ie(role, example_1_vendor_ie.oui);
    if(ret< 0)
    {
        Report("Delete vendor IE role:%d  example 1 failed!!\n", role);
        return ret;
    }
    ret = delete_vendor_ie(role, example_2_vendor_ie.oui);
    if(ret< 0)
    {
        Report("Delete vendor IE role:%d  example 2 failed!!\n", role);
        return ret;
    }

    Report("2 vendor IEs were deleted for role:%d !!\n", role);

    ret = delete_vendor_ie_list(role);
    if(ret)
    {
        Report("Deleting vendor IE list for role:%d \n", role);
    }
    else
    {
        Report("Deleting vendor IE list for role:%d failed !!! \n", role);
    }
    return(ret);

}


/*!
    \brief          Configure event periodic for STA aging

    configure time after which an event for sTa aging will be send

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;


*/
int32_t cmdConfigStaAgingEventCallback(void *arg)
{
    int32_t ret;
    wlanSetStaAgingTimout_t staAging;
    uint32_t timeOut;

    ret = ParseSetpeerAgingTimeout(arg , &timeOut);

    if(ret < 0)
    {
        printConfiPeerAgingUsage(arg);
        return 0;
    }

    staAging.peerAgingTimeoutMs = timeOut;
    while((ret = Wlan_Set(WLAN_SET_PEER_AGING_TIMEOUT, &staAging))==WLAN_RET_OPER_IN_PROGRESS);

    return(ret);

}

#endif

//////////////////////// P2P /////////////////////////////
/*!
    \brief          Wlan role down P2P callback.

    This routine shows how to role down of P2P  role.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdWlanRoleDownP2PCallback(void *arg)
{
    int32_t      ret = 0;
    WlanP2pCmd_t pParams;

    /* Check if network Device is already deactivated */
    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_DEVICE_BIT))
    {
        Report("\n\rDevice role Is Already Down.\n\r");
        return -1;
    }

    ret = ParseCmd(arg);

    if(ret < 0)
    {
        printWlanRoleDownP2PUsage(arg);
        return -1;
    }

    //group remove - if exist
    pParams.Id = P2P_CMD_ID_GROUP_REMOVE;
    while((ret = Wlan_Set(WLAN_SET_P2P_CMD, &pParams))==WLAN_RET_OPER_IN_PROGRESS);
    if (ret != 0)
    {
        Report("\n\rError!! cmdWlanRoleDownP2PCallback: failed to remove group\n\r");
        return ret;
    }

    ret = Wlan_RoleDown(WLAN_ROLE_DEVICE, OSI_WAIT_FOR_SECOND);
    if(ret < 0)
    {
        return -1;
    }

    CLEAR_BIT_IN_BITMAP(ActiveNetIfBitMap, NET_IF_DEVICE_BIT);

    app_CB.Role = WLAN_ROLE_RESERVED;

    return ret;

}

//////////////////////// P2P /////////////////////////////
/*!
    \brief          Wlan P2P discover.

    This routine shows how to perform P2P discover

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdWlanP2PFindCallback(void *arg)
{
    Report("Received P2P discover\n");

    return cmdScan(arg, P2P_DEVICE_SCAN);
}

//////////////////////// P2P /////////////////////////////
/*!
    \brief          Wlan P2P connect.

    This routine shows how to connect to P2P device

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdWlanP2PConnectCallback(void *arg)
{
    uint8_t peer_mac[6] = {0, 0, 0, 0, 0, 0};
    uint32_t wps_method = 0;
    uint8_t security_type;
    char pin[9] = {0};
    char* ssid = " ";
    int ssid_len = 1;
    int pin_len = 0; // 0 or 8
    int32_t ret = 0;

    ret = Wlan_Disconnect(WLAN_ROLE_STA,NULL);
    if(ret == OK){
        ret = osi_SyncObjWait(&(app_CB.CON_CB.disconnectEventSyncObj), OSI_WAIT_FOR_SECOND * 60);
        if(OSI_OK != ret)
        {
            Report("\n\r[ERROR]cmdWlanStopCallback: Failed waiting sync object\n\r");
            ASSERT_GENERAL(0);
            return ret;
        }
    }

    ret = ParseP2PConnectCmd(arg, peer_mac, &wps_method, pin);
    if(ret < 0)
    {
        printWlan2PConnectUsage(arg);
        return -1;
    }
    if (wps_method == 0)
    {
        security_type = WLAN_SEC_TYPE_P2P_PBC;
    }
    else if (wps_method == 1)
    {
        security_type = WLAN_SEC_TYPE_P2P_PIN_DISPLAY;
        if(strlen(pin) != 8)
        {
            Report("Error ! P2P connect , Invalid pin size for PIN_DISPLAY\n");
            return -1;
        }
        pin_len = 8;
    }
    else if (wps_method == 2)
    {
        security_type = WLAN_SEC_TYPE_P2P_PIN_KEYPAD;
        if(strlen(pin) != 8)
        {
            Report("Error ! P2P connect , Invalid pin size for PIN_KEYPAD\n");
            return -1;
        }
        pin_len = 8;
    }
    else
    {
        Report("Error ! P2P, Invalid  WPS method\n");
        return -1;
    }

    Report("P2P connect, peer address: %02x:%02x:%02x:%02x:%02x:%02x security-type:%d pin:%s pin_len:%d\n",
            peer_mac[0], peer_mac[1], peer_mac[2], peer_mac[3], peer_mac[4], peer_mac[5],
            security_type, pin, pin_len);

    osi_SyncObjClear(&(app_CB.CON_CB.connectEventSyncObj));

    ret = Wlan_Connect(
                         (const signed char *)ssid,
                         ssid_len,
                         (const unsigned char *)peer_mac,
                         security_type,
                         (const char *)pin,
                         pin_len,
                         0);

    if(ret == 0)
    {
        ret = osi_SyncObjWait(&(app_CB.CON_CB.connectEventSyncObj),
                                WLAN_WPS_TOUT);
       
        if(ret != 0)
        {
            WlanP2pCmd_t pParams;
            UART_PRINT("\n\r[p2p_connect app] : Timeout expired connecting WiFi-Direct: %d\n\r",
                                               security_type);
            pParams.Id = P2P_CMD_ID_GROUP_REMOVE;
            ret = Wlan_Set(WLAN_SET_P2P_CMD, &pParams);
            if (ret != 0)
            {
                UART_PRINT("\n\r[p2p_connect app] : Failed to call P2P group remove: %d\n\r", ret);
            }
            
            return(-1);
        }
        else
        {
            UART_PRINT("\n\r[p2p_connect app] : connected !!!!");
        }

    }
    else
    {
        UART_PRINT("\n\r[p2p_connect app] : Failed calling P2P Connect (%d)\n\r", ret);
    }

    return ret;
 }


int32_t cmdWlanP2PFindStopCallback(void *arg)
{
    WlanP2pCmd_t pParams;
    int32_t retVal;
    int32_t ret = 0;

    //creating sync obj for waiting scan completed event
    retVal = osi_SyncObjCreate(&p2p_find_stopped_syncObj);
    if(retVal < 0)
    {
        Report("\n\rERROR !p2p_find_stopped_syncObj creation failed\n");
        return -1;
    }
    g_wait_p2p_scan_complete = TRUE;

    //send p2p scan stop command
    pParams.Id = P2P_CMD_ID_SCAN_STOP;

    while((ret = Wlan_Set(WLAN_SET_P2P_CMD, &pParams))==WLAN_RET_OPER_IN_PROGRESS);
    if(ret == 0)
    {
        //command was send now expect to scan to complete
        osi_SyncObjWait(&p2p_find_stopped_syncObj, OSI_WAIT_FOR_SECOND* 100);
    }
    else
    {
        Report("\n\rP2p stop find command failed\n");
    }

    retVal = osi_SyncObjDelete(&p2p_find_stopped_syncObj);
    if(retVal < 0)
    {
        Report("ERROR !Sync obj ,p2p_find_stopped_syncObj deletion failed\n");
        ret = -1;
    }
    g_wait_p2p_scan_complete = FALSE;
    return ret;
}


int32_t cmdWlanP2PGrpRemoveCallback(void *arg)
{
    WlanP2pCmd_t pParams;
    int32_t ret;

    pParams.Id = P2P_CMD_ID_GROUP_REMOVE;
    while((ret = Wlan_Set(WLAN_SET_P2P_CMD, &pParams))==WLAN_RET_OPER_IN_PROGRESS);
    return ret;
}

int32_t cmdWlanP2PSetChannelCallback(void *arg)
{
    WlanP2pCmd_t P2PSetChannel;
    int32_t ret;


    P2PSetChannel.Id = P2P_CMD_ID_CONFIG_CHANNELS;

    P2PSetChannel.Data.cfgParams.operChannel = 0;
    P2PSetChannel.Data.cfgParams.operClass = 81;
    P2PSetChannel.Data.cfgParams.listenChannel = 0;
    P2PSetChannel.Data.cfgParams.listenClass = 81;
    P2PSetChannel.Data.cfgParams.goIntent = 0;

    /* Call the command parser */
    ret = ParseSetChannelCmd(arg , &P2PSetChannel);

    if(ret < 0)
    {
        printWlanRoleUpP2PUsage(arg);
        return -1;
    }

    Report("p2p set channel: operational channel=%d, listen channel=%d, goIntent=%d\n\r",
            P2PSetChannel.Data.cfgParams.operChannel,
            P2PSetChannel.Data.cfgParams.listenChannel,
            P2PSetChannel.Data.cfgParams.goIntent);

    Report("p2p set channel: operational regulatory class=%d, listen regulatory class=%d\n\r",
            P2PSetChannel.Data.cfgParams.operClass,
            P2PSetChannel.Data.cfgParams.listenClass);

    while((ret = Wlan_Set(WLAN_SET_P2P_CMD, &P2PSetChannel))==WLAN_RET_OPER_IN_PROGRESS);

    return ret;
}

int32_t cmdWlanP2PGetChannelCallback(void *arg)
{
    int32_t ret;

    WlanP2pConfigChannelParam_t pParams;

    while((ret=Wlan_Get(WLAN_GET_P2P_CONFIG_CHANNEL_PARAMS, &pParams))==WLAN_RET_OPER_IN_PROGRESS);

    Report("p2p channel configuration: oper=%d %d, listen=%d %d go_intent=%d\n",
                pParams.operChannel, pParams.operClass, pParams.listenChannel,
                pParams.listenClass, pParams.goIntent);


    return ret;
}

int32_t cmdWlanP2PListenCallback(void *arg)
{
    int32_t ret;

    WlanP2pCmd_t pParams;

    pParams.Id = P2P_CMD_ID_LISTEN;
    while((ret = Wlan_Set(WLAN_SET_P2P_CMD, &pParams))==WLAN_RET_OPER_IN_PROGRESS);


    return ret;
}

int32_t cmdWlanP2PCancelCallback(void *arg)
{
    int32_t ret;

    WlanP2pCmd_t pParams;

    pParams.Id = P2P_CMD_ID_CANCEL;
    while((ret = Wlan_Set(WLAN_SET_P2P_CMD, &pParams))==WLAN_RET_OPER_IN_PROGRESS);


    return ret;
}


/////////////// Connection Policy ////////////////

/*!
    \brief          WLAN Connection Policy Set callback.

    This routine shows how to Set Connection Policy.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */

int32_t cmdWlanSetConnPolicyCallback(void *arg)
{
    int32_t ret = 0;
    WlanPolicySetGet_t ConnPolicyParams;

    /* Call the command parser */
    os_memset(&ConnPolicyParams, 0x0, sizeof(WlanPolicySetGet_t));
    ret = ParseConnPolicySetCmd(arg, &ConnPolicyParams);

    if(ret < 0)
    {
        printWlanSetConnPolicyUsage(arg);
        return -1;
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
        return (-1);
    }

    Report("\r\n wlan_set_conn_policy, auto connect=%d, fast connect=%d fast persistant=%d",
             ConnPolicyParams.autoPolicy, ConnPolicyParams.fastPolicy, ConnPolicyParams.fastPersistant);

    while((ret = Wlan_Set(WLAN_SET_CONNECTION_POLICY, &ConnPolicyParams))==WLAN_RET_OPER_IN_PROGRESS);

    return(0);
}

/*!
    \brief          Prints WLAN Connection Policy Set command help menu.

    \param          arg   -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanSetConnPolicyCallback
 */
int32_t printWlanSetConnPolicyUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanSetConnPolicyStr);
    UART_PRINT(wlanSetConnPolicyUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetConnPolicyDetailsStr);
    UART_PRINT(wlanSetConnPolicy_a_optionDetailsStr);
    UART_PRINT(wlanSetConnPolicy_f_optionDetailsStr);
    UART_PRINT(wlanSetConnPolicy_p_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

int32_t cmdWlanGetConnPolicyCallback(void *arg)
{
    int32_t ret = 0;
    WlanPolicySetGet_t ConnPolicyParams;

    /* Call the command parser */
    os_memset(&ConnPolicyParams, 0x0, sizeof(WlanPolicySetGet_t));

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
        return (-1);
    }

    while((ret = Wlan_Get(WLAN_GET_CONNECTION_POLICY, &ConnPolicyParams))==WLAN_RET_OPER_IN_PROGRESS);

    Report("\r\n wlan_get_conn_policy, auto connect=%d, fast connect=%d fast_persistant=%d",
             ConnPolicyParams.autoPolicy, ConnPolicyParams.fastPolicy, ConnPolicyParams.fastPersistant);


    return(0);
}



/*!
    \brief          Prints WLAN Connection Policy Get command help menu.

    \param          arg   -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanGetConnPolicyCallback
 */
int32_t printWlanGetConnPolicyUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanGetConnPolicyStr);
    UART_PRINT(wlanGetConnPolicyUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanGetConnPolicyDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          WLAN Add Profile callback.

    This routine shows how to Add Profile of a preferred network.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */
int32_t cmdWlanAddProfileCallback(void *arg)
{
    int32_t ret = 0;
    ProfileCmd_t  ProfileParams;

    /* Call the command parser */
    memset(&ProfileParams, 0x0, sizeof(ProfileCmd_t));
    ret = ParseProfileCmd(arg, &ProfileParams);

    if(ret < 0)
    {
        FreeProfileCmd(&ProfileParams);
        return(-1);
    }

   if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
   {
       UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
       return (-1);
   }

   if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
   {
       UART_PRINT("\n\rNo STA role up\n\r");
       return (-1);
   }

   memset(ProfileParams.mac, 0, WLAN_BSSID_LENGTH);
   ProfileParams.mac = NULL;

   Report("\r\n wlan_add_profile, ssid:%s ssidlen:%d secType:%d hidden:%d priority:%d key:%s", 
                ProfileParams.ssid, strlen((const char *)(ProfileParams.ssid)),
                ProfileParams.secParams.Type, *ProfileParams.hidden,
                *ProfileParams.priority , ProfileParams.secParams.Key);
    
    ret = Wlan_ProfileAdd((const signed char *)(ProfileParams.ssid),
                          strlen((const char *)(ProfileParams.ssid)),
                          ProfileParams.mac, //macAddr
                          &ProfileParams.secParams,
                          NULL, // ent 
                          *ProfileParams.priority,
                          *ProfileParams.hidden,
                          0);
    
    if (ret < 0)
    {
        Report("\n\rError! wlan_add_profile failed\n\r");
        FreeProfileCmd(&ProfileParams);
        return(-1);
    }   
    else
    {
        Report("\n\r wlan_add_profile success, index = %d\n\r", ret);
    }

    FreeProfileCmd(&ProfileParams);
    return(0);
}

/*!
    \brief          Prints WLAN Add Profile command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanAddProfileCallback
 */
int32_t printAddProfileUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanAddProfileStr);
    UART_PRINT(wlanAddProfileUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanAddProfileDetailsStr);
    UART_PRINT(wlanAddProfile_s_optionDetailsStr);
    UART_PRINT(wlanAddProfile_t_optionDetailsStr);
    UART_PRINT(wlanAddProfile_p_optionDetailsStr);
    UART_PRINT(wlanAddProfile_pr_optionDetailsStr);
    UART_PRINT(wlanAddProfile_h_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          WLAN Delete Profile callback.

    This routine shows how to Remove Profile from NV

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */
int32_t cmdWlanDeleteProfileCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t index;

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
        return (-1);
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
        UART_PRINT("\n\rNo STA role up\n\r");
        return (-1);
    }

    //parse index
    index = ParseDelProfileCmd(arg);
    if(index < 0)
    {
        UART_PRINT("\n\rError! Invalid index\n\r");
        return(-1);
    }

    ret = Wlan_ProfileDel((uint8_t)index);

    if (ret < 0)
    {
        UART_PRINT("\n\rError! wlan_delete_profile failed\n\r");
        return(-1);
    }

    UART_PRINT("\n\r wlan_delete_profile success\n\r");
    return (0);
}

/*!
    \brief          Prints WLAN Delete Profile command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanDeleteProfileCallback
 */
int32_t printDeleteProfileUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanDelProfileStr);
    UART_PRINT(wlanDelProfileUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanDelProfileDetailsStr);
    UART_PRINT(wlanDelProfile_i_optionDetailsStr);
    UART_PRINT(lineBreak);
    return(0);
}


/*!
    \brief          WLAN Get Profile callback.

    This routine shows how to get Profile from NV

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */
int32_t cmdWlanGetProfileCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t index;
    signed char ssid[WLAN_SSID_MAX_LENGTH + 1];
    int ssidLen = WLAN_SSID_MAX_LENGTH + 1;
    uint8_t bssid[WLAN_BSSID_LENGTH];
    WlanSecParams_t secParams;
    //WlanSecParamsExt_t secExtParams;

    //initialize 
    uint32_t priority = 0;
    uint32_t hidden = 0;
    os_memset(ssid, 0, WLAN_SSID_MAX_LENGTH + 1);
    os_memset(bssid, 0, WLAN_BSSID_LENGTH);
    memset(&secParams, 0, sizeof(WlanSecParams_t));

    //parse index
   index = ParseGetProfileCmd(arg);
   if(index < 0)
   {
        UART_PRINT("\n\rError! Invalid index\n\r");
        return(-1);
   }

    ret = Wlan_ProfileGet(index, ssid, &ssidLen, bssid, &secParams, NULL, &priority, &hidden);

    if (ret < 0)
    {
        UART_PRINT("\n\rError! Get Profile failed ret = %d\n\r", ret);
        return(-1);
    }

    UART_PRINT("\n\r Get profile index %d: ssid = %s, bssid = %02x:%02x:%02x:%02x:%02x:%02x, security = %d, priority = %d scan_ssid = %d\n\r",
                index, ssid, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5], secParams.Type, priority, hidden);
    return (0);
}


/*!
    \brief          Prints WLAN Get Profile command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa            cmdWlanGetProfileCallback     
 */
int32_t printGetProfileUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanGetProfileStr);
    UART_PRINT(wlanGetProfileUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanGetProfileDetailsStr);
    UART_PRINT(wlanGetProfile_i_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}



/*!
    \brief          WLAN Set Scan Dwell Time callback.

    This routine shows how to Set Dwell Time Callback

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */
int32_t cmdWlanSetScanDwellTimeCallback(void *arg)
{

    int16_t         ret = 0;
    WlanScanDwellTime_t scanDwellTimes = 
    {
        .max_dwell_time_passive_msec = DEFAULT_SCAN_MAX_DWELL_TIME_PASSIVE_MSEC,
        .min_dwell_time_passive_msec = DEFAULT_SCAN_MIN_DWELL_TIME_PASSIVE_MSEC,
        .max_dwell_time_active_msec  = DEFAULT_SCAN_MAX_DWELL_TIME_ACTIVE_MSEC,
        .min_dwell_time_active_msec  = DEFAULT_SCAN_MIN_DWELL_TIME_ACTIVE_MSEC,
        .dwell_time_dfs_passive_msec = DEFAULT_SCAN_DFS_DWELL_TIME_PASSIVE_MSEC
    };

    ret = ParseSetScanDwellTimeCmd(arg, &scanDwellTimes);

    if(ret < 0)
    {
        return ret;
    }

    ret = Wlan_Set(WLAN_SET_SCAN_DWELL_TIME, (void *)&scanDwellTimes);

    if(ret < 0)
    {
        Report("\n\r[SCAN DWELL TIME] Failed to set scan dwell times\n\r");
    }
    else
    {
        Report("\n\r[SCAN DWELL TIME] Successfully set dwell times\n\r");
    }

    return (ret);
}

//
/*!
    \brief          Prints WLAN Set Scan Dwell Time command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa            cmdWlanSetScanDwellTimeCallback     
 */
int32_t printWlanSetScanDwellTimeUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanSetScanDwellTimeStr);
    UART_PRINT(wlanSetScanDwellTimeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetScanDwellTimeDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}


/*!
    \brief          WLAN Profile Connect callback.

    This routine shows how to Connect via a Profile index 

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */
int32_t cmdWlanProfileConnectCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t index;
    signed char ssid[WLAN_SSID_MAX_LENGTH + 1];
    int ssidLen = 0;
    uint8_t bssid[WLAN_BSSID_LENGTH];

    //parse index
    index = ParseProfileConnectCmd(arg);
    if((index < 0) || (index > 5))
    {
        UART_PRINT("\n\rError! Invalid index\n\r");
        return(-1);
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
       UART_PRINT("\n\rDevice is stopped, run wlan_start.\n\r");
       return (-1);
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT))
    {
       UART_PRINT("\n\rNo STA role up\n\r");
       return (-1);
    }

    os_memset(ssid, 0, WLAN_SSID_MAX_LENGTH + 1);
    os_memset(bssid, 0, WLAN_BSSID_LENGTH);

    osi_SyncObjClear(&(app_CB.CON_CB.connectEventSyncObj));

    ret = Wlan_Connect((const signed char *)(ssid), // ssid empty
                       ssidLen, // ssid len = 0
                       bssid,   //macAddr, empty by default
                       index, // profile index
                       0, // passphrase empty
                       0, // passphrase len 0
                       WLAN_CONNECT_FLAG_PROFILE_CONNECT);

    if (ret != 0)
    {
        UART_PRINT("\n\rError! Connect failed ret = %d\n\r", ret);
        return(-1);
    }

    /* Wait for connection events:
     * In order to verify that connection was successful,
     * we pend on two incoming events: Connected and Ip acquired.
     * The semaphores below are pend by this (Main) context.
     * They will be signaled once an asynchronous event
     * Indicating that the NWP has connected and acquired IP address is raised.
     * For further information, see this application read me file.
     */
    if(!IS_STA_CONNECTED(app_CB.Status))
    {

        ret = osi_SyncObjWait(&(app_CB.CON_CB.connectEventSyncObj),
                                   WLAN_EVENT_TOUT);
        
        if(ret != 0)
        {

            UART_PRINT("\n\r[wlan_connect app] : Timeout expired connecting to AP");
            Wlan_Disconnect(WLAN_ROLE_STA,NULL);

            return(-1);
        }
        else
        {
            UART_PRINT("\n\r[wlan_connect app] : connected !!!!");
        }

    }
    return (0);
}

/*!
    \brief          Prints WLAN Profile Connect command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa            cmdWlanProfileConnectCallback     
 */
int32_t printWlanProfileConnectUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanProfileConnectStr);
    UART_PRINT(wlanProfileConnectUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanProfileConnectDetailsStr);
    UART_PRINT(wlanProfileConnect_i_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Prints start ping command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdPingStartCallback
*/
int32_t printPingStartUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(pingStartStr);
    UART_PRINT(pingStartUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(pingStartDetailsStr);
    UART_PRINT(pingStart_c_optionDetailsStr);
    UART_PRINT(pingStart_i_optionDetailsStr);
    UART_PRINT(pingStart_s_optionDetailsStr);
    UART_PRINT(pingStart_I_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Start a ping session.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdPingStartCallback(void *arg)
{
    int32_t ret = 0;
    PingParams_t pingParams = {0};

    ret = ParsePingCmd(arg, &pingParams);
    if (ret < 0)
    {
        return -1;
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
       Report("\n\rPing command failed: Device is stopped\n\r");
       return -1;
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT) &&
        !IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
       Report("\n\rPing command failed: No role is up\n\r");
       return -1;
    }

    ret = lwip_ping_start(&pingParams);
    if (ret < 0)
    {
        Report("\n\rPing command failed: lwip_ping_start failed\n\r");
        return -1;
    }

    return ret;
}

/*!
    \brief          Prints stop ping command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdPingStopCallback
*/
int32_t printPingStopUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(pingStopStr);
    UART_PRINT(pingStopUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(pingStopDetailsStr);
    UART_PRINT(pingStop_i_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Stop a ping session.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdPingStopCallback(void *arg)
{
    int32_t ret = 0;
    int8_t pingSessionId = 0;

    ret = ParsePingStopCmd(arg, &pingSessionId);
    if (ret < 0)
    {
        return -1;
    }

    ret = lwip_ping_stop(pingSessionId);
    if (ret < 0)
    {
        Report("\n\rPing stop command failed: lwip_ping_stop failed\n\r");
        return -1;
    }

    return ret;
}

/*!
    \brief          WLAN Regulatory Domain Entry Set callback.

    This routine sets a regulatory domain rule to the WLAN regulatory
    domain database. See "wlan_if.h" for more details.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */
int32_t cmdWlanSetRegDomainEntryCallback(void *arg)
{
    int32_t ret = 0;
    WlanSetRegDomainCustomEntry_t entryParams;

    /* Call the command parser */
    os_memset(&entryParams, 0x0, sizeof(WlanSetRegDomainCustomEntry_t));

    ret = ParseRegDomEntrySetCmd(arg, &entryParams);
    if (ret < 0)
    {
        printWlanSetRegDomainEntryUsage(arg);
        return -1;
    }

    ret = Wlan_Set(WLAN_SET_CUSTOM_DOMAIN_ENTRY, &entryParams);
    if (ret < 0)
    {
        return ret;
    }

    return(0);
}

/*!
    \brief          Prints WLAN set regulatory domain entry command help menu.

    \param          arg   -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanSetRegDomainEntryCallback
 */
int32_t printWlanSetRegDomainEntryUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanSetRegDomEntryStr);
    UART_PRINT(wlanSetRegDomEntryUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanSetRegDomEntryDetailsStr);
    UART_PRINT(wlanSetRegDomEntry_i_optionDetailsStr);
    UART_PRINT(wlanSetRegDomEntry_b_optionDetailsStr);
    UART_PRINT(wlanSetRegDomEntry_p_optionDetailsStr);
    UART_PRINT(wlanSetRegDomEntry_d_optionDetailsStr);
    UART_PRINT(wlanSetRegDomEntry_n_optionDetailsStr);
    UART_PRINT(wlanSetRegDomEntry_bm_optionDetailsStr);
    UART_PRINT(wlanSetRegDomEntry_r_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          WLAN Regulatory Domain Entry Get callback.

    This routine retrieves a regulatory domain rule from the WLAN regulatory
    domain database. See "wlan_if.h" for more details.

    \param          arg       -   Points to command line buffer.
                                  This container would be passed
                                  to the parser module.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \note

    \sa             

 */
int32_t cmdWlanGetRegDomainEntryCallback(void *arg)
{
    int32_t ret = 0;
    WlanSetRegDomainCustomEntry_t entryParams;

    /* Call the command parser */
    os_memset(&entryParams, 0x0, sizeof(WlanSetRegDomainCustomEntry_t));

    ret = ParseRegDomEntryGetCmd(arg, &entryParams);
    if (ret < 0)
    {
        printWlanGetRegDomainEntryUsage(arg);
        return -1;
    }

    ret = Wlan_Get(WLAN_GET_CUSTOM_DOMAIN_ENTRY, &entryParams);
    if (ret < 0)
    {
        Report("\n\rERROR - failed getting index %d. "
               "Check the boundaries", entryParams.customIndex);
        return ret;
    }

    Report("\n\rEntry index %d is set with the following parameters:\n\r"
		   "IsDfsChannel:%d, MaxTxPower:%d, band:%d, chanBitmap:0x%08x, "
		   "minChannel:%d, maxChannel: %d, numOfChannels:%d",
		   entryParams.customIndex, entryParams.IsDfsChannel, entryParams.MaxTxPower,
		   entryParams.band, entryParams.chanBitmap, entryParams.minChannel,
		   entryParams.maxChannel, entryParams.numOfChannels);

    return(0);
}

/*!
    \brief          Prints WLAN get regulatory domain entry command help menu.

    \param          arg   -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdWlanGetRegDomainEntryCallback
 */
int32_t printWlanGetRegDomainEntryUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(wlanGetRegDomEntryStr);
    UART_PRINT(wlanGetRegDomEntryUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(wlanGetRegDomEntryDetailsStr);
    UART_PRINT(wlanGetRegDomEntry_i_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

//Indigo
int32_t cmdloadCartificateCallback(void *arg){
    int32_t ret = 0;
    LoadCertiCmd_t LoadCertiParams;

    ret = ParseLoadCartificateCmd(arg, &LoadCertiParams);

    if(ret < 0)
    {
        os_free(client_certi.certi);
        os_free(ca_certi.certi);
        os_free(private_key_certi.certi);
        return(-1);
    }

    return(0);
}

#endif // CC35XX
