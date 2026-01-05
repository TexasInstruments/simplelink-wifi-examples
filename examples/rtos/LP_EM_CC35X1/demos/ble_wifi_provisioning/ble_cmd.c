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
#include "osi_kernel.h"
#include "ble_cmd.h"
#include "ble_if.h"
#include "ble_transport.h"
#include "npi_if.h"
#include "uart_term.h"
#include "nimble_host.h"
#include "wlan_cmd.h"

/******************************************************************************
                      Callback Functions
******************************************************************************/
/*!
    \brief          Ble ADV configure callback.

    This routine shows how to start the BLE.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleAdvCfgCallback(void *arg)
{
    int ret = 0;
    ExtAdvCfg_t cmdParams;

    /* Call the command parser */
    memset(&cmdParams, 0x0, sizeof(ExtAdvCfg_t));
    ret = ParseBleAdvCfgCmd(arg, &cmdParams);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_ext_adv_cfg(&cmdParams);

}

/*!
    \brief          Prints Ble Adv Configuration command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleAdvCfgCallback
*/
int32_t printBleAdvCfgUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleAdvCfgStr);
    UART_PRINT(bleAdvCfgUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleAdvCfgDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble ADV configure callback.

    This routine shows how to start the BLE.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleAdvEnableCallback(void *arg)
{
    int ret = 0;
    ExtAdvEnable_t cmdParams;

    /* Call the command parser */
    memset(&cmdParams, 0x0, sizeof(ExtAdvEnable_t));
    ret = ParseBleAdvEnableCmd(arg, &cmdParams);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_ext_adv_enable(&cmdParams);
}

/*!
    \brief          Prints Ble Adv Enable command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleStartCallback
*/
int32_t printBleAdvEnableUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleAdvEnableStr);
    UART_PRINT(bleAdvEnableUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleAdvEnableDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble Scan configure callback.


    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleScanCfgCallback(void *arg)
{
    int ret = 0;
    ExtScanCfg_t cmdParams;

    /* Call the command parser */
    memset(&cmdParams, 0x0, sizeof(ExtScanCfg_t));
    ret = ParseBleScanCfgCmd(arg, &cmdParams);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_ext_scan_cfg(&cmdParams);
}

/*!
    \brief          Prints Ble Scan Configuration command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleScanCfgCallback
*/
int32_t printBleScanCfgUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleScanCfgStr);
    UART_PRINT(bleScanCfgUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleScanCfgDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble Scan enable callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd
*/
int32_t cmdBleScanEnableCallback(void *arg)
{
    int ret = 0;
    ExtScanEnable_t cmdParams;

    /* Call the command parser */
    memset(&cmdParams, 0x0, sizeof(ExtScanEnable_t));
    ret = ParseBleScanEnableCmd(arg, &cmdParams);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_ext_scan_enable(&cmdParams);
}

/*!
    \brief          Prints Ble Scan Enable command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleScanEnableCallback
*/
int32_t printBleScanEnableUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleScanEnableStr);
    UART_PRINT(bleScanEnableUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleScanEnableDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble Connect callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd
*/
int32_t cmdBleConnectCallback(void *arg)
{
    int ret = 0;
    uint8_t bdAddr[6] = {0};
    uint8_t addr_type = 0;

    /* Call the command parser */
    ret = ParseBleConnectCmd(arg, bdAddr, &addr_type);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_ext_connect(bdAddr, addr_type);
}

/*!
    \brief          Prints Ble connect command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleConnectCallback
*/
int32_t printBleConnectUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleConnectStr);
    UART_PRINT(bleConnectUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleConnectDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble disconnect callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd
*/
int32_t cmdBleDisconnectCallback(void *arg)
{
    int ret = 0;
    uint8_t bdAddr[6] = {0};
    uint8_t addr_type = 0;

    /* Call the command parser */
    ret = ParseBleDisconnectCmd(arg, bdAddr, &addr_type);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_ext_disconnect(bdAddr, addr_type);
}

/*!
    \brief          Prints Ble disconnect command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleDisconnectCallback
*/
int32_t printBleDisconnectUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleDisconnectStr);
    UART_PRINT(bleDisconnectUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleDisconnectDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble peers callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd
*/
int32_t cmdBlePeersCallback(void *arg)
{
    return nimble_host_connected_peers();
}

/*!
    \brief          Prints Ble peers command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBlePeersCallback
*/
int32_t printBlePeersUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(blePeersStr);
    UART_PRINT(blePeersUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(blePeersDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble delete bonds callback.

    This routine shows how to delete all the BLE bonds.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleDeleteBondsCallback(void *arg)
{
    int ret = 0;

    ret = nimble_host_delete_all_bond_info();

    return ret;
}

/*!
    \brief          Prints Ble delete bonds command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleDeleteBondsCallback
*/
int32_t printBleDeleteBondsUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleDeleteBondsStr);
    UART_PRINT(bleDeleteBondsUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleDeleteBondsDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble get bd address callback.

    This routine shows how to get the BLE BD address

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleGetBdAddrCallback(void *arg)
{
    int ret = 0;
    uint8_t addr_type = 0;

    /* Call the command parser */
    ret = ParseBleGetBdAddressCmd(arg, &addr_type);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_get_bd_address(addr_type);
}

/*!
    \brief          Prints Ble get bd address command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleStartCallback
*/
int32_t printBleGetBdAddrUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleGetBdAddrStr);
    UART_PRINT(bleGetBdAddrUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleGetBdAddrDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble set bd address callback.

    This routine shows how to get the BLE BD address

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleSetBdAddrCallback(void *arg)
{
    int ret = 0;
    uint8_t addr_type = 0;

    /* Call the command parser */
    ret = ParseBleSetBdAddressCmd(arg, &addr_type);

    if (ret < 0)
    {
        return (-1);
    }

    return nimble_host_set_bd_address(addr_type);
}

/*!
    \brief          Prints Ble set bd address command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleStartCallback
*/
int32_t printBleSetBdAddrUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleSetBdAddrStr);
    UART_PRINT(bleSetBdAddrUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleSetBdAddrDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble start callback.

    This routine shows how to start the BLE.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleStartCallback(void *arg)
{
    int ret = 0;

    // Check if network IF is already active
    if (!isNetIFActive())
    {
        Report("\n\rDevice is stopped, run wlan_start.\n\r");
        return ( -1 );
    }

    //Open BLE transport
    BleIf_OpenTransport();

    //Check if host was already enabled
    if (!nimble_host_is_enabled())
    {
        //Start NimBLE host (also enable the controller)
        ret = nimble_host_start();

        if (nimble_host_is_enabled())
        {
            Report("\n\rBLE start success!\n\r");
        }
        else
        {
            Report("\n\rBLE failed to initialize, error: %d\n\r",ret);
        }
    }
    else
    {
        Report("\n\rBLE is already running\n\r");
    }

    return ret;
}

/*!
    \brief          Prints Ble start command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleStartCallback
*/
int32_t printBleStartUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleStartStr);
    UART_PRINT(bleStartUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleStartDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}

/*!
    \brief          Ble stop callback.

    This routine shows how to stop the BLE.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

    \sa             ParseCmd

*/
int32_t cmdBleStopCallback(void *arg)
{
    int ret = 0;

    //Stop the BLE Host
    ret = nimble_host_stop();

    //Close the BLE transport
    BleIf_CloseTransport();

    return ret;
}

/*!
    \brief          Prints Ble stop command help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdBleStopCallback
*/
int32_t printBleStopUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(bleStopStr);
    UART_PRINT(bleStopUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(bleStopDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}
