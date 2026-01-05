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
// includes
//*****************************************************************************
#include <stdlib.h>
#include <stdio.h>

#include "osi_kernel.h"
#include "str_mpl.h"

/* AT Header files */
#include "atcmd_ble.h"
#include "atcmd_defs.h"
#include "atcmd_event.h"
#include "atcmd_gen.h"
#include "atcmd.h"

#include "ble_if.h"
#include "nimble_host_at_commands.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#include "cmd_parser.h"

//*****************************************************************************
// defines
//*****************************************************************************
#define ATCMD_BLE_DEINIT        (0)
#define ATCMD_BLE_INIT          (1)

#define ATCMD_BLE_MAX_NAME_LEN  MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME_MAX_LENGTH)

//*****************************************************************************
// typedefs
//*****************************************************************************


//*****************************************************************************
// Externs and globals
//*****************************************************************************
extern uint32_t ActiveNetIfBitMap;


//*****************************************************************************
// AT Command Wlan Routines
//*****************************************************************************

/*!
    \brief          Parse BLE init command.

    This routine takes a mode variable, and fill its content with parameters
    taken from command line. In case of a parsing error or invalid parameters,
    this function sets default values.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          mode        -   Points to number of entries to be scanned at maximum.
                                    This value will later be read by the scan callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function returns a negative value.

    \sa             
*/
int32_t ATCmdBle_initParse(char *buff, uint8_t *mode)
{
    int32_t ret = 0;

    /* num of results */
    ret = StrMpl_getVal(&buff, mode,
                        ATCMD_DELIM_TRM, sizeof(*mode));

    return ret;
}

int32_t bleInit()
{
    int32_t ret = 0;

    BleIf_OpenTransport();

    if (!nimble_host_is_enabled())
    {
        ret = nimble_host_start();

        if (nimble_host_is_enabled() && ret == 0)
        {
            ATCmd_okResult();
        }
        else
        {
            ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
            ret = -1;
        }
    }
    else
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
    }

    return ret;
}

int32_t bleDeinit()
{
    int32_t ret = 0;

    ret = nimble_host_stop();

    BleIf_CloseTransport();

    if (ret != 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        ret = -1;
    }
    else
    {
        ATCmd_okResult();
    }

    return ret;
}

/*!
       \brief          BLE init command.

       This routine starts the BLE module.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             

*/
int32_t ATCmdBle_bleInitCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t mode = 0;

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorDeviceNotStartedStr, ret);
        return ret;
    }

    ret = ATCmdBle_initParse((char *)arg, &mode);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    if (mode == ATCMD_BLE_INIT)
    {
        ret = bleInit();
    }
    else if (mode == ATCMD_BLE_DEINIT)
    {
        ret = bleDeinit();
    }
    else
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
    }

    return ret;
}

/*!
    \brief          Parse BLE MAC address command.

    This routine takes a type variable, and fill its content with parameters
    taken from command line. In case of a parsing error or invalid parameters,
    this function returns a negative value.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          mode        -   Points to number of entries to be scanned at maximum.
                                    This value will later be read by the scan callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function returns a negative value.

    \sa             
*/
int32_t ATCmdBle_setMacAddressParse(char *buff, uint8_t *type)
{
    int32_t ret = 0;

    /* num of results */
    ret = StrMpl_getVal(&buff, type,
                        ATCMD_DELIM_TRM, sizeof(*type));

    return ret;
}

/*!
       \brief          BLE set MAC address command.

       This routine sets BLE MAC address.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_setMacAddressCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t addressType;

    ret = ATCmdBle_setMacAddressParse((char *)arg, &addressType);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    ret = nimble_host_set_mac_addr(addressType);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        return -1;
    }

    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Return get BLE random MAC address result.

    This routine send the BLE random MAC address result.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdBle_getMacAddressResult(void *args, int32_t num, char *buff)
{    
    uint8_t *macAddress = (uint8_t *)args;
    uint8_t reversedAddress[BLE_DEV_ADDR_LEN];

    os_memcpy(reversedAddress, macAddress, BLE_DEV_ADDR_LEN);
    BLE_COPY_BD_ADDRESS(reversedAddress, macAddress);

    StrMpl_setStr(ATCmd_bleGetMacAddressStr, &buff, ATCMD_DELIM_EVENT);

    StrMpl_setArrayVal(reversedAddress, &buff,
                       BLE_DEV_ADDR_LEN,
                       ATCMD_DELIM_TRM, ATCMD_DELIM_ARRAY,
                       STRMPL_FLAG_PARAM_HEX |
                       STRMPL_FLAG_PARAM_SIZE_8 |
                       STRMPL_FLAG_PARAM_UNSIGNED |
                       STRMPL_FLAG_PARAM_NO_HEX_PREFIX);

    os_free(macAddress);

    return 0;
}

/*!
       \brief          BLE get random MAC address command.

       This routine gets BLE random MAC address.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_getMacAddressCallback(void *arg)
{
    int32_t ret = 0;
    ble_addr_t addr;
    uint8_t *macAddress = NULL;

    ret = nimble_host_get_mac_addr(&addr);
    if (ret != 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        return -1;
    }
    
    macAddress = os_malloc(BLE_DEV_ADDR_LEN);
    if (macAddress == NULL)
    {
        ATCmd_errorResult(ATCmd_errorAllocStr, 0);
        return -1;
    }
    os_memcpy(macAddress, addr.val, BLE_DEV_ADDR_LEN);

    ATCmd_commandResult(ATCmdBle_getMacAddressResult, macAddress, 0);
    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Parse BLE device name command.

    This routine takes a name variable, and fills its content with parameters
    taken from command line. In case of a parsing error or invalid parameters,
    this function returns a negative value.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          mode        -   Points to number of entries to be scanned at maximum.
                                    This value will later be read by the scan callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function returns a negative value.

    \sa             
*/
int32_t ATCmdBle_setDeviceNameParse(char *buff, char *name)
{
    int32_t ret = 0;

    ret = StrMpl_getStr(&buff, name, ATCMD_DELIM_TRM,
                        ATCMD_BLE_MAX_NAME_LEN, ATCmd_excludeDelimStr);

    return ret;
}

/*!
       \brief          BLE set device name command.

       This routine sets BLE device name.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_setDeviceNameCallback(void *arg)
{
    int32_t ret = 0;
    char name[ATCMD_BLE_MAX_NAME_LEN] = {0};

    ret = ATCmdBle_setDeviceNameParse((char *)arg, name);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    ret = ble_svc_gap_device_name_set(name);
    if (ret != 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        return -1;
    }

    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Return get BLE device name result.

    This routine send the BLE device name result.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdBle_getDeviceNameResult(void *args, int32_t num, char *buff)
{
    char *name = (char *)args;

    StrMpl_setStr(ATCmd_bleGetDeviceNameStr, &buff, ATCMD_DELIM_EVENT);

    StrMpl_setStr(name, &buff, ATCMD_DELIM_TRM);

    os_free(args);

    return 0;
}

/*!
       \brief          BLE get device name command.

       This routine gets BLE device name.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_getDeviceNameCallback(void *arg)
{
    int32_t ret = 0;
    const char *name = NULL;
    char *buff = NULL;
    uint8_t nameLen = 0;

    name = ble_svc_gap_device_name();
    nameLen = os_strlen(name);

    buff = os_malloc(nameLen + 1);
    if (buff == NULL)
    {
        ATCmd_errorResult(ATCmd_errorAllocStr, 0);
        return -1;
    }
    os_memcpy(buff, name, nameLen);
    buff[nameLen] = '\0';

    ATCmd_commandResult(ATCmdBle_getDeviceNameResult, buff, 0);
    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Parse BLE set scan configurations command.

    This routine takes a params variable, and fills its content with parameters
    taken from command line. In case of a parsing error or invalid parameters,
    this function returns a negative value.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          params      -   Points to output parameters.
                                    Will contain the user-specified parameters.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function returns a negative value.

    \sa             
*/
int32_t ATCmdBle_setScanCfgParse(char *buff, AtCmdExtScanCfg_t *params)
{
    int32_t ret = 0;

    ret = StrMpl_getVal(&buff, &params->scan_type,
                        ATCMD_DELIM_ARG, sizeof(params->scan_type));
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, &params->own_address_type,
                        ATCMD_DELIM_ARG, sizeof(params->own_address_type));
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, &params->filter_policy,
                        ATCMD_DELIM_ARG, sizeof(params->filter_policy));
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, &params->scan_interval,
                        ATCMD_DELIM_ARG, sizeof(params->scan_interval));
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, &params->scan_window,
                        ATCMD_DELIM_TRM, sizeof(params->scan_window));
    if (ret < 0)
    {
        return ret;
    }

    return ret;
}

/*!
       \brief          BLE set scan configurations command.

       This routine sets BLE scan configurations.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_setScanCfgCallback(void *arg)
{
    int32_t ret = 0;
    AtCmdExtScanCfg_t cmdParams;

    os_memset(&cmdParams, 0x0, sizeof(AtCmdExtScanCfg_t));
    ret = ATCmdBle_setScanCfgParse(arg, &cmdParams);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    cmdParams.scan_phy = BLE_GAP_LE_PHY_1M;

    ret = nimble_host_ext_scan_cfg(&cmdParams);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        ret = -1;
    }
    else
    {
        ATCmd_okResult();
    }

    return ret;
}

/*!
    \brief          Return get BLE scan configurations result.

    This routine send the BLE scan configurations result.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdBle_getScanCfgResult(void *args, int32_t num, char *buff)
{
    AtCmdExtScanCfg_t *scanCfg = (AtCmdExtScanCfg_t *)args;

    StrMpl_setStr(ATCmd_bleGetScanCfgStr, &buff, ATCMD_DELIM_EVENT);

    StrMpl_setVal(&scanCfg->scan_type, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&scanCfg->own_address_type, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&scanCfg->filter_policy, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&scanCfg->scan_interval, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_16 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&scanCfg->scan_window, &buff, ATCMD_DELIM_TRM,
                  STRMPL_FLAG_PARAM_SIZE_16 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    os_free(args);

    return 0;
}

/*!
       \brief          BLE get scan configurations command.

       This routine gets BLE scan configurations.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_getScanCfgCallback(void *arg)
{
    int32_t ret = 0;
    AtCmdExtScanCfg_t *scanCfg = NULL;

    scanCfg = os_malloc(sizeof(AtCmdExtScanCfg_t));
    if (scanCfg == NULL)
    {
        ATCmd_errorResult(ATCmd_errorAllocStr, 0);
        return -1;
    }

    ret = nimble_host_ext_scan_cfg_get(scanCfg);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        os_free(scanCfg);
        return -1;
    }

    ATCmd_commandResult(ATCmdBle_getScanCfgResult, scanCfg, 0);
    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Parse BLE scan command.

    This routine takes a params variable, and fills its content with parameters
    taken from command line. In case of a parsing error or invalid parameters,
    this function returns a negative value.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          mode        -   Points to number of entries to be scanned at maximum.
                                    This value will later be read by the scan callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function returns a negative value.

    \sa             
*/
int32_t ATCmdBle_scanParse(char *buff, ExtScanEnable_t *params)
{
    int32_t ret;

    ret = StrMpl_getVal(&buff, &params->enable,
                        ATCMD_DELIM_ARG, sizeof(params->enable));
    if (ret < 0)
    {
        /* If enable is 0 we're not interested in the other parameters
          and there shouldn't be any others, so we look for a terminating 
          delimiter instead of an argument one. Also if enable is 1 and we
          don't want to set duration we need to parse again with TRM delim */
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getVal(&buff, &params->enable,
                                ATCMD_DELIM_TRM, sizeof(params->enable));
        }
        return ret;
    }
    
    /* The following parameters are taken only if enable isn't 0 */
    ret = StrMpl_getVal(&buff, &params->duration,
                        ATCMD_DELIM_TRM, sizeof(params->duration));
    if (ret == 0)
    {
        /* Convert from seconds duration to centiseconds */
        params->duration = params->duration * 100;
    }

    return ret;
}

/*!
       \brief          BLE scan command.

       This routine enables BLE scan.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_scanCallback(void *arg)
{
    int32_t ret = 0;
    ExtScanEnable_t scanParams;

    os_memset(&scanParams, 0x0, sizeof(ExtScanEnable_t));
    ret = ATCmdBle_scanParse(arg, &scanParams);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }
    scanParams.filter_duplicate = 1;
    scanParams.period = 0;

    ret = nimble_host_ext_scan_enable(&scanParams);
    if (ret != 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        return -1;
    }

    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Parse BLE connect command.

    This routine takes some variables, and fills their content with parameters
    taken from command line. In case of a parsing error or invalid parameters,
    this function returns a negative value.

    \param          buff        -   Points to command line buffer.
                                    Contains the command line typed by user.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function returns a negative value.

    \sa             
*/
int32_t ATCmdBle_connectParse(char *buff, uint8_t *bdAddress,
                              uint8_t *addressType)
{
    int32_t ret = 0;

    ret = StrMpl_getArrayVal(&buff, (void *)bdAddress, BLE_DEV_ADDR_LEN,
                             ATCMD_DELIM_ARG, ATCMD_DELIM_ARRAY,
                             STRMPL_FLAG_PARAM_SIZE_8 |
                             STRMPL_FLAG_PARAM_NO_HEX_PREFIX,
                             ATCmd_excludeDelimArray);
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, addressType,
                        ATCMD_DELIM_TRM, sizeof(*addressType));

    if ((*addressType != BLE_OWN_ADDR_PUBLIC) &&
        (*addressType != BLE_OWN_ADDR_RANDOM))
    {
        ret = -1;
    }

    return ret;
}

/*!
       \brief          BLE connect command.

       This routine start BLE connection.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_connectCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t bdAddress[BLE_DEV_ADDR_LEN] = {0};
    uint8_t addressType;

    ret = ATCmdBle_connectParse(arg, bdAddress, &addressType);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    ret = nimble_host_ext_connect(bdAddress, addressType);
    if (ret != 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        return -1;
    }

    ATCmd_okResult();

    return ret;
}

/*!
       \brief          BLE disconnect command.

       This routine starts BLE disconnection.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_disconnectCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t bdAddr[BLE_DEV_ADDR_LEN] = {0};
    uint8_t addressType;

    ret = nimble_host_connected_peers(bdAddr, &addressType);

    if (ret > 0)
    {
        ret = nimble_host_ext_disconnect(bdAddr, addressType);
        if (ret != 0)
        {
            ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
            return -1;
        }
    }
    else
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        return -1;
    }

    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Return get BLE peers result.

    This routine send the BLE peers result.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdBle_getConnectedPeersResult(void *args, int32_t num, char *buff)
{
    uint8_t *peerAddress = (uint8_t *)args;
    
    StrMpl_setStr(ATCmd_bleConnectedPeersStr, &buff, ATCMD_DELIM_EVENT);

    if (num > 0)
    {
        /* MAC address */
        StrMpl_setArrayVal(peerAddress, &buff,
                           BLE_DEV_ADDR_LEN,
                           ATCMD_DELIM_TRM, ATCMD_DELIM_ARRAY,
                           STRMPL_FLAG_PARAM_HEX |
                           STRMPL_FLAG_PARAM_SIZE_8 |
                           STRMPL_FLAG_PARAM_UNSIGNED |
                           STRMPL_FLAG_PARAM_NO_HEX_PREFIX);
    }
    else
    {
        *buff = '\0';
        buff++;
    }

    os_free(peerAddress);

    return 0;
}

/*!
       \brief          BLE connected peers command.

       This routine gets BLE connected peers.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_connectedPeersCallback(void *arg)
{
    int32_t ret = 0;
    uint8_t *peerAddress = NULL;

    peerAddress = os_malloc(BLE_DEV_ADDR_LEN);
    if (peerAddress == NULL)
    {
        ATCmd_errorResult(ATCmd_errorAllocStr, 0);
        return -1;
    }
    os_memset(peerAddress, 0x0, BLE_DEV_ADDR_LEN);

    /* ret holds the number of peers */
    ret = nimble_host_connected_peers(peerAddress, NULL);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorBleOperationStr, ret);
        os_free(peerAddress);
        return -1;
    }
 
    ATCmd_okResult();
    ATCmd_commandResult(ATCmdBle_getConnectedPeersResult, peerAddress, ret);

    return ret;
}

/*!
    \brief          Parse BLE set advertise configurations command.

    This routine takes a params variable, and fills its content with parameters
    taken from command line. In case of a parsing error or invalid parameters,
    this function returns a negative value.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          mode        -   Points to number of entries to be scanned at maximum.
                                    This value will later be read by the scan callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function returns a negative value.

    \sa             
*/
int32_t ATCmdBle_setAdvCfgParse(char *buff, AtCmdExtAdvCfg_t *params)
{
    int32_t ret = 0;
    int32_t peerAddrTypeRet = 0;

    /* Defaults (which are different from 0) */
    params->primPhy = BLE_GAP_LE_PHY_1M;
    params->secPhy = BLE_GAP_LE_PHY_1M;

    ret = StrMpl_getVal(&buff, &params->intervalMin,
                        ATCMD_DELIM_ARG, sizeof(params->intervalMin));
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, &params->intervalMax,
                        ATCMD_DELIM_ARG, sizeof(params->intervalMax));
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, &params->advType,
                        ATCMD_DELIM_ARG, sizeof(params->advType));
    if (ret < 0)
    {
        return ret;
    }

    ret = StrMpl_getVal(&buff, &params->ownAddrType,
                        ATCMD_DELIM_ARG, sizeof(params->ownAddrType));
    if (ret < 0)
    {
        return ret;
    }

    /* Start scanning for optional parameters */
    ret = StrMpl_getVal(&buff, &params->channelMap,
                        ATCMD_DELIM_ARG, sizeof(params->channelMap));
    if (ret < 0)
    {
        /* If the delimiter is missing, re-scan with TRM delimiter and return.
         *
         * This fits the following cmd:
         * at+blesetadvcfg=intMin,intMax,advType,ownAddrType,channelMap
         * 
         */
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getVal(&buff, &params->channelMap,
                                ATCMD_DELIM_TRM, sizeof(params->channelMap));
            if (ret == STRMPL_ERROR_PARAM_MISSING)
            {
                ret = 0;
            }
            return ret;
        }
        else if (ret != STRMPL_ERROR_PARAM_MISSING)
        {
            return ret;
        }
    }

    ret = StrMpl_getVal(&buff, &params->filterPolicy,
                        ATCMD_DELIM_ARG, sizeof(params->filterPolicy));
    if (ret < 0)
    {
        /* If the delimiter is missing, re-scan with TRM delimiter and return.
         *
         * This fits the following cmd:
         * at+blesetadvcfg=intMin,intMax,advType,ownAddrType,channelMap,filterPolicy
         * 
         */
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getVal(&buff, &params->filterPolicy,
                        ATCMD_DELIM_TRM, sizeof(params->filterPolicy));
            if (ret == STRMPL_ERROR_PARAM_MISSING)
            {
                ret = 0;
            }
            return ret;
        }
        else if (ret != STRMPL_ERROR_PARAM_MISSING)
        {
            return ret;
        }
    }

    /*
     * Scan for peer address type and peer address.
     * They must be set together.
     */
    peerAddrTypeRet = StrMpl_getVal(&buff, &params->peerAddr.type,
                                    ATCMD_DELIM_ARG, sizeof(params->peerAddr.type));
    if ((peerAddrTypeRet < 0) && (peerAddrTypeRet != STRMPL_ERROR_PARAM_MISSING))
    {
        return ret;
    }

    ret = StrMpl_getArrayVal(&buff, (void *)params->peerAddr.val,
                             BLE_DEV_ADDR_LEN,
                             ATCMD_DELIM_ARG, ATCMD_DELIM_ARRAY,
                             STRMPL_FLAG_PARAM_SIZE_8 |
                             STRMPL_FLAG_PARAM_NO_HEX_PREFIX,
                             ATCmd_excludeDelimArray);
    if ((ret == 0) && (peerAddrTypeRet == STRMPL_ERROR_PARAM_MISSING))
    {
        /* peer address type is missing - must be set together */
        return STRMPL_ERROR_PARAM_MISSING;
    }
    else if (ret < 0)
    {
        /* If the delimiter is missing, re-scan with TRM delimiter and return.
         *
         * This fits the following cmd:
         * at+blesetadvcfg=intMin,intMax,advType,ownAddrType,channelMap,filterPolicy,peerAddressType,peerAddress
         * 
         * Since address type and value must be set together we must verify it.
         */
        if (ret == STRMPL_ERROR_PARAM_MISSING)
        {
            if (peerAddrTypeRet != STRMPL_ERROR_PARAM_MISSING)
            {
                return ret;
            }
        }
        else if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getArrayVal(&buff, (void *)params->peerAddr.val,
                                     BLE_DEV_ADDR_LEN,
                                     ATCMD_DELIM_TRM, ATCMD_DELIM_ARRAY,
                                     STRMPL_FLAG_PARAM_SIZE_8 |
                                     STRMPL_FLAG_PARAM_NO_HEX_PREFIX,
                                     ATCmd_excludeDelimArray);
            /* Allow the case in which both address and type are empty at the end. */
            if ((ret == STRMPL_ERROR_PARAM_MISSING) &&
                (peerAddrTypeRet == STRMPL_ERROR_PARAM_MISSING))
            {
                ret = 0;
            }
            return ret;
        }
    }

    /* Full command is being used, scan all remaining parameters */
    ret = StrMpl_getVal(&buff, &params->primPhy,
                        ATCMD_DELIM_ARG, sizeof(params->primPhy));
    if (ret < 0)
    {
        /* If the delimiter is missing, re-scan with TRM delimiter and return. */
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getVal(&buff, &params->primPhy,
                                ATCMD_DELIM_TRM, sizeof(params->primPhy));
            if (ret == STRMPL_ERROR_PARAM_MISSING)
            {
                ret = 0;
            }
            return ret;
        }
        else if (ret != STRMPL_ERROR_PARAM_MISSING)
        {
            return ret;
        }
    }


    ret = StrMpl_getVal(&buff, &params->secPhy,
                        ATCMD_DELIM_TRM, sizeof(params->secPhy));
    if (ret == STRMPL_ERROR_PARAM_MISSING)
    {
        return 0;
    }

    return ret;
}

/*!
       \brief          BLE set advertise configurations command.

       This routine sets BLE advertise configurations.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_setAdvCfgCallback(void *arg)
{
    int32_t ret = 0;
    AtCmdExtAdvCfg_t cmdParams;

    os_memset(&cmdParams, 0x0, sizeof(AtCmdExtAdvCfg_t));
    ret = ATCmdBle_setAdvCfgParse(arg, &cmdParams);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    ret = nimble_host_ext_adv_cfg(&cmdParams);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
    }
    else
    {
        ATCmd_okResult();
    }

    return ret;
}

/*!
    \brief          Return get BLE advertise configuration result.

    This routine send the BLE get advertise configuration result.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdBle_getAdvCfgResult(void *args, int32_t num, char *buff)
{
    AtCmdExtAdvCfg_t *pAdvCfg = (AtCmdExtAdvCfg_t *)args;
    
    StrMpl_setStr(ATCmd_bleGetAdvCfgStr, &buff, ATCMD_DELIM_EVENT);

    StrMpl_setVal(&pAdvCfg->intervalMin, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_32 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&pAdvCfg->intervalMax, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_32 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&pAdvCfg->advType, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&pAdvCfg->ownAddrType, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&pAdvCfg->channelMap, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&pAdvCfg->filterPolicy, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&pAdvCfg->peerAddr.type, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setArrayVal(pAdvCfg->peerAddr.val, &buff,
                       BLE_DEV_ADDR_LEN,
                       ATCMD_DELIM_ARG, ATCMD_DELIM_ARRAY,
                       STRMPL_FLAG_PARAM_HEX |
                       STRMPL_FLAG_PARAM_SIZE_8 |
                       STRMPL_FLAG_PARAM_UNSIGNED |
                       STRMPL_FLAG_PARAM_NO_HEX_PREFIX);

    StrMpl_setVal(&pAdvCfg->primPhy, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    StrMpl_setVal(&pAdvCfg->secPhy, &buff, ATCMD_DELIM_TRM,
                  STRMPL_FLAG_PARAM_SIZE_8 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    os_free(pAdvCfg);

    return 0;
}

/*!
       \brief          BLE get advertise configurations command.

       This routine sets BLE advertise configurations.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_getAdvCfgCallback(void *arg)
{
    int32_t ret = 0;
    AtCmdExtAdvCfg_t *cfg = NULL;

    cfg = os_malloc(sizeof(AtCmdExtAdvCfg_t));
    if (cfg == NULL)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorAllocStr, ret);
        return ret;
    }

    nimble_host_get_ext_adv_cfg(cfg);

    ATCmd_commandResult(ATCmdBle_getAdvCfgResult, cfg, 0);

    ATCmd_okResult();

    return ret;
}

/*!
       \brief          BLE start advertise command.

       This routine starts BLE advertise.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_advStartCallback(void *arg)
{
    int32_t ret = 0;
    ExtAdvEnable_t cmdParams;

    /* Use defaults */
    os_memset(&cmdParams, 0x0, sizeof(ExtAdvEnable_t));
    cmdParams.enable = 1;

    ret = nimble_host_ext_adv_enable(&cmdParams);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
    }
    else
    {
        ATCmd_okResult();
    }

    return ret;
}

/*!
       \brief          BLE stop advertise command.

       This routine stops BLE advertise.

       \return         Upon successful completion, the function shall return 0.
                       In case of failure, this function would return -1;

       \sa             
*/
int32_t ATCmdBle_advStopCallback(void *arg)
{
    int32_t ret = 0;
    ExtAdvEnable_t cmdParams;

    os_memset(&cmdParams, 0x0, sizeof(ExtAdvEnable_t));
    cmdParams.enable = 0; /* Make it explicit */

    ret = nimble_host_ext_adv_enable(&cmdParams);
    if (ret < 0)
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
    }
    else
    {
        ATCmd_okResult();
    }

    return ret;
}