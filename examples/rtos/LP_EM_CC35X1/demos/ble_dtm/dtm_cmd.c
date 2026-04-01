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

#include "dtm_cmd.h"
#include <stdlib.h>
#include <string.h>
#include "ble_if.h"
#include "uart_term.h"

/****************************************************************************
 *                      LOCAL DEFINITIONS
 ****************************************************************************/
#define CMD_BUFFER_LEN  256

/****************************************************************************
 *                      STRING DEFINITIONS
 ****************************************************************************/
static const char lineBreak[]      = "\n\r";
static const char usageStr[]       = "Usage: ";
static const char descriptionStr[] = "Description: ";

/****************************************************************************
 *                      GLOBAL VARIABLES
 ****************************************************************************/
dtm_control_block_t dtm_CB;

/****************************************************************************
 *                      LOCAL FUNCTION PROTOTYPES
 ****************************************************************************/
static int dtmEventHandler(uint8_t* data, uint16_t len);
static int32_t sendDtmCommand(uint8_t* cmd, uint16_t cmdLen);

/****************************************************************************
 *                      DTM INITIALIZATION
 ****************************************************************************/
int32_t DtmInit(void)
{
    int32_t ret = 0;

    dtm_CB.state = DTM_STATE_IDLE;
    dtm_CB.hciStatus = 0;
    dtm_CB.packetCount = 0;

    ret = osi_SyncObjCreate(&dtm_CB.responseSyncObj);
    if(ret != 0)
    {
        UART_PRINT("[DTM] Failed to create sync object\n\r");
        return -1;
    }

    return 0;
}

int32_t DtmRegisterEventHandler(void)
{
    BleIf_EventCbRegister(dtmEventHandler);
    return 0;
}

/****************************************************************************
 *                      DTM HCI EVENT HANDLER
 ****************************************************************************/
static int dtmEventHandler(uint8_t* data, uint16_t len)
{
    /* HCI Event format:
     * data[0] = HCI packet type (0x04 for event)
     * data[1] = Event code
     * data[2] = Parameter length
     * data[3...] = Parameters
     */

    if(data == NULL)
    {
        UART_PRINT("\n\r[DTM] Invalid data pointer\n\r");
        return -1;
    }

    if(len < HCI_EVENT_HDR_SIZE)
    {
        UART_PRINT("\n\r[DTM] Invalid event length: %d\n\r", len);
        return -1;
    }

    uint8_t pktType = data[0];
    uint8_t eventCode = data[1];
    uint8_t paramLen = data[2];
    uint8_t *params = &data[3];

    /* Check for HCI Event packet */
    if(pktType != HCI_EVENT_PACKET)
    {
        UART_PRINT("\n\r[DTM] Unexpected packet type: 0x%02X\n\r", pktType);
        return -1;
    }

    /* Validate buffer length against paramLen to prevent over-read */
    if(len < (HCI_EVENT_HDR_SIZE + paramLen))
    {
        UART_PRINT("\n\r[DTM] Buffer too short: len=%d, paramLen=%d\n\r", len, paramLen);
        return -1;
    }

    /* All DTM responses require parameters */
    if(paramLen == 0)
    {
        UART_PRINT("\n\r[DTM] Event with no parameters\n\r");
        return -1;
    }

    /* Command Complete Event
     * params[0] = Num HCI cmd packets
     * params[1-2] = Command opcode (LE)
     * params[3] = Status
     * params[4-5] = Return parameters (e.g., packet count)
     */
    if(eventCode == HCI_EVENT_CMD_COMPLETE)
    {
        /* Need at least 4 bytes: numCmd(1) + opcode(2) + status(1) */
        if(paramLen < HCI_CMD_COMPLETE_MIN_PARAM_LEN)
        {
            UART_PRINT("\n\r[DTM] Command complete with insufficient params: %d\n\r", paramLen);
            return -1;
        }

        uint16_t opcode = params[1] | (params[2] << 8);
        uint8_t status = params[3];

        dtm_CB.hciStatus = status;

        switch(opcode)
        {
            case DTM_CMD_RX_TEST_V2:
                if(status == 0)
                {
                    UART_PRINT("\n\r[DTM] RX Test V2 started successfully\n\r");
                    dtm_CB.state = DTM_STATE_RX_TEST;
                }
                else
                {
                    UART_PRINT("\n\r[DTM] RX Test V2 failed, status: 0x%02X\n\r", status);
                }
                break;

            case DTM_CMD_TX_TEST_V2:
                if(status == 0)
                {
                    UART_PRINT("\n\r[DTM] TX Test V2 started successfully\n\r");
                    dtm_CB.state = DTM_STATE_TX_TEST;
                }
                else
                {
                    UART_PRINT("\n\r[DTM] TX Test V2 failed, status: 0x%02X\n\r", status);
                }
                break;

            case DTM_CMD_TEST_END:
                if(status == 0)
                {
                    /* Extract packet count (2 bytes, little endian) */
                    if(paramLen >= 6)
                    {
                        dtm_CB.packetCount = params[4] | (params[5] << 8);
                        UART_PRINT("\n\r[DTM] Test ended successfully\n\r");
                        UART_PRINT("[DTM] Packets received: %d\n\r", dtm_CB.packetCount);
                    }
                    else
                    {
                        UART_PRINT("\n\r[DTM] Test ended successfully (no packet count)\n\r");
                    }
                    dtm_CB.state = DTM_STATE_IDLE;
                }
                else
                {
                    UART_PRINT("\n\r[DTM] Test End failed, status: 0x%02X\n\r", status);
                }
                break;

            default:
                UART_PRINT("\n\r[DTM] Unknown command complete opcode: 0x%04X\n\r", opcode);
                break;
        }

        /* Signal response received */
        osi_SyncObjSignal(&dtm_CB.responseSyncObj);
    }
    /* TI Vendor Specific Event
     * params[0-1] = Vendor event opcode
     * params[2] = Status
     * params[3-4] = Command opcode (LE)
     */
    else if(eventCode == HCI_EVENT_VENDOR_SPECIFIC)
    {
        /* Need at least 5 bytes: vendorOpcode(2) + status(1) + cmdOpcode(2) */
        if(paramLen < HCI_VENDOR_SPECIFIC_MIN_PARAM_LEN)
        {
            UART_PRINT("\n\r[DTM] Vendor event with insufficient params: %d\n\r", paramLen);
            return -1;
        }

        uint8_t status = params[2];
        uint16_t cmdOpcode = params[3] | (params[4] << 8);

        dtm_CB.hciStatus = status;

        if(cmdOpcode == DTM_CMD_SET_TX_POWER)
        {
            if(status == 0)
            {
                UART_PRINT("\n\r[DTM] Set Max TX Power successful\n\r");
            }
            else
            {
                UART_PRINT("\n\r[DTM] Set Max TX Power failed, status: 0x%02X\n\r", status);
            }
            /* Signal response received */
            osi_SyncObjSignal(&dtm_CB.responseSyncObj);
        }
        else
        {
            UART_PRINT("\n\r[DTM] Vendor event for opcode: 0x%04X, status: 0x%02X\n\r", cmdOpcode, status);
        }
    }
    else
    {
        UART_PRINT("\n\r[DTM] Received event code: 0x%02X\n\r", eventCode);
    }

    return 0;
}

/****************************************************************************
 *                      DTM INTERNAL COMMAND FUNCTION
 ****************************************************************************/
static int32_t sendDtmCommand(uint8_t* cmd, uint16_t cmdLen)
{
    int ret;

    /* Clear sync object before sending */
    osi_SyncObjClear(&dtm_CB.responseSyncObj);

    /* Send command via BLE interface */
    ret = BleIf_SendCommand(cmd, cmdLen);
    if(ret != 0)
    {
        UART_PRINT("\n\r[DTM] Failed to send command, error: %d\n\r", ret);
        return ret;
    }

    /* Wait for response with timeout (2 seconds) */
    ret = osi_SyncObjWait(&dtm_CB.responseSyncObj, 2 * OSI_WAIT_FOR_SECOND);
    if(ret != OSI_OK)
    {
        UART_PRINT("\n\r[DTM] Command timeout\n\r");
        return -1;
    }

    return (dtm_CB.hciStatus == 0) ? 0 : -1;
}

/****************************************************************************
 *                      DTM STATE FUNCTIONS
 ****************************************************************************/
dtm_state_t DtmGetState(void)
{
    return dtm_CB.state;
}

int32_t DtmIsTestRunning(void)
{
    return (dtm_CB.state != DTM_STATE_IDLE) ? 1 : 0;
}

/****************************************************************************
 *                      DTM COMMAND CALLBACKS
 ****************************************************************************/
int32_t cmdDtmRxCallback(void *arg)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    dtm_rx_params_t rxParams;
    uint8_t hciCmd[7];
    int32_t ret;

    /* Default parameters */
    rxParams.channel = 0;
    rxParams.phy = DTM_PHY_1M;
    rxParams.modulation_index = DTM_MOD_INDEX_STANDARD;

    /* Check if already running */
    if(DtmIsTestRunning())
    {
        UART_PRINT("\n\r[DTM] Test already running. Use 'dtm_test_end' first.\n\r");
        return -1;
    }

    /* Parse arguments */
    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, " ");

    while(token)
    {
        if(!strcmp(token, "-help") || !strcmp(token, "-h"))
        {
            printDtmRxUsage(NULL);
            return 0;
        }
        else if(!strcmp(token, "-c"))
        {
            token = strtok(NULL, " ");
            if(token) rxParams.channel = (uint8_t)atoi(token);
        }
        else if(!strcmp(token, "-p"))
        {
            token = strtok(NULL, " ");
            if(token) rxParams.phy = (uint8_t)atoi(token);
        }
        else if(!strcmp(token, "-m"))
        {
            token = strtok(NULL, " ");
            if(token) rxParams.modulation_index = (uint8_t)atoi(token);
        }
        token = strtok(NULL, " ");
    }

    /* Validate parameters */
    if(rxParams.channel > DTM_CHANNEL_MAX)
    {
        UART_PRINT("\n\r[DTM] Invalid channel: %d (valid: 0-%d)\n\r", rxParams.channel, DTM_CHANNEL_MAX);
        return -1;
    }

    if(rxParams.phy < DTM_PHY_1M || rxParams.phy > DTM_PHY_CODED_S2)
    {
        UART_PRINT("\n\r[DTM] Invalid PHY: %d (valid: 1-4)\n\r", rxParams.phy);
        return -1;
    }

    if(rxParams.modulation_index > DTM_MOD_INDEX_STABLE)
    {
        UART_PRINT("\n\r[DTM] Invalid modulation index: %d (valid: 0-1)\n\r", rxParams.modulation_index);
        return -1;
    }

    UART_PRINT("\n\r[DTM] Starting RX Test V2 (0x2033)\n\r");
    UART_PRINT("[DTM] Channel: %d, PHY: %d, Mod Index: %d\n\r",
               rxParams.channel, rxParams.phy, rxParams.modulation_index);

    /* Build HCI command */
    hciCmd[0] = HCI_CMD_PACKET;
    hciCmd[1] = (DTM_CMD_RX_TEST_V2 & 0xFF);
    hciCmd[2] = ((DTM_CMD_RX_TEST_V2 >> 8) & 0xFF);
    hciCmd[3] = 3;
    hciCmd[4] = rxParams.channel;
    hciCmd[5] = rxParams.phy;
    hciCmd[6] = rxParams.modulation_index;

    ret = sendDtmCommand(hciCmd, sizeof(hciCmd));

    return ret;
}

int32_t cmdDtmTxCallback(void *arg)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    dtm_tx_params_t txParams;
    uint8_t hciCmd[8];
    int32_t ret;

    /* Default parameters */
    txParams.channel = 0;
    txParams.data_len = 37;
    txParams.payload = DTM_PAYLOAD_PRBS9;
    txParams.phy = DTM_PHY_1M;

    /* Check if already running */
    if(DtmIsTestRunning())
    {
        UART_PRINT("\n\r[DTM] Test already running. Use 'dtm_test_end' first.\n\r");
        return -1;
    }

    /* Parse arguments */
    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, " ");

    while(token)
    {
        if(!strcmp(token, "-help") || !strcmp(token, "-h"))
        {
            printDtmTxUsage(NULL);
            return 0;
        }
        else if(!strcmp(token, "-c"))
        {
            token = strtok(NULL, " ");
            if(token) txParams.channel = (uint8_t)atoi(token);
        }
        else if(!strcmp(token, "-l"))
        {
            token = strtok(NULL, " ");
            if(token) txParams.data_len = (uint8_t)atoi(token);
        }
        else if(!strcmp(token, "-d"))
        {
            token = strtok(NULL, " ");
            if(token) txParams.payload = (uint8_t)atoi(token);
        }
        else if(!strcmp(token, "-p"))
        {
            token = strtok(NULL, " ");
            if(token) txParams.phy = (uint8_t)atoi(token);
        }
        token = strtok(NULL, " ");
    }

    /* Validate parameters */
    if(txParams.channel > DTM_CHANNEL_MAX)
    {
        UART_PRINT("\n\r[DTM] Invalid channel: %d (valid: 0-%d)\n\r", txParams.channel, DTM_CHANNEL_MAX);
        return -1;
    }

    if(txParams.phy < DTM_PHY_1M || txParams.phy > DTM_PHY_CODED_S2)
    {
        UART_PRINT("\n\r[DTM] Invalid PHY: %d (valid: 1-4)\n\r", txParams.phy);
        return -1;
    }

    if(txParams.payload > DTM_PAYLOAD_01010101)
    {
        UART_PRINT("\n\r[DTM] Invalid payload: %d (valid: 0-7)\n\r", txParams.payload);
        return -1;
    }

    UART_PRINT("\n\r[DTM] Starting TX Test V2 (0x2034)\n\r");
    UART_PRINT("[DTM] Channel: %d, Length: %d, Payload: %d, PHY: %d\n\r",
               txParams.channel, txParams.data_len, txParams.payload, txParams.phy);

    /* Build HCI command */
    hciCmd[0] = HCI_CMD_PACKET;
    hciCmd[1] = (DTM_CMD_TX_TEST_V2 & 0xFF);
    hciCmd[2] = ((DTM_CMD_TX_TEST_V2 >> 8) & 0xFF);
    hciCmd[3] = 4;
    hciCmd[4] = txParams.channel;
    hciCmd[5] = txParams.data_len;
    hciCmd[6] = txParams.payload;
    hciCmd[7] = txParams.phy;

    ret = sendDtmCommand(hciCmd, sizeof(hciCmd));

    return ret;
}

int32_t cmdDtmEndCallback(void *arg)
{
    uint8_t hciCmd[4];
    int32_t ret;
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;

    /* Parse arguments */
    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, " ");

    while(token)
    {
        if(!strcmp(token, "-help") || !strcmp(token, "-h"))
        {
            printDtmEndUsage(NULL);
            return 0;
        }
        token = strtok(NULL, " ");
    }

    /* Check if test is running */
    if(!DtmIsTestRunning())
    {
        UART_PRINT("\n\r[DTM] No test running\n\r");
        return 0;
    }

    UART_PRINT("\n\r[DTM] Ending Test (0x201F)\n\r");

    /* Build HCI command */
    hciCmd[0] = HCI_CMD_PACKET;
    hciCmd[1] = (DTM_CMD_TEST_END & 0xFF);
    hciCmd[2] = ((DTM_CMD_TEST_END >> 8) & 0xFF);
    hciCmd[3] = 0;

    ret = sendDtmCommand(hciCmd, sizeof(hciCmd));

    return ret;
}

/****************************************************************************
 *                      DTM USAGE PRINT FUNCTIONS
 ****************************************************************************/
int32_t printDtmRxUsage(void *arg)
{
    UART_PRINT("\n\r%s\n\r", lineBreak);
    UART_PRINT("%sdtm_rx_test [-c channel] [-p phy] [-m mod_index]\n\r", usageStr);
    UART_PRINT("%sStart Enhanced Receiver Test (HCI 0x2033)\n\r", descriptionStr);
    UART_PRINT("\n\r");
    UART_PRINT("Options:\n\r");
    UART_PRINT("  -c channel     : RX channel (0-39), default: 0\n\r");
    UART_PRINT("  -p phy         : PHY type, default: 1\n\r");
    UART_PRINT("                   1 = 1M PHY\n\r");
    UART_PRINT("                   2 = 2M PHY\n\r");
    UART_PRINT("                   3 = Coded PHY (S=8, 125 kbps)\n\r");
    UART_PRINT("                   4 = Coded PHY (S=2, 500 kbps)\n\r");
    UART_PRINT("  -m mod_index   : Modulation index, default: 0\n\r");
    UART_PRINT("                   0 = Standard\n\r");
    UART_PRINT("                   1 = Stable\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("Example: dtm_rx_test -c 17 -p 1\n\r");
    UART_PRINT("%s\n\r", lineBreak);
    return 0;
}

int32_t printDtmTxUsage(void *arg)
{
    UART_PRINT("\n\r%s\n\r", lineBreak);
    UART_PRINT("%sdtm_tx_test [-c channel] [-l length] [-d payload] [-p phy]\n\r", usageStr);
    UART_PRINT("%sStart Enhanced Transmitter Test (HCI 0x2034)\n\r", descriptionStr);
    UART_PRINT("\n\r");
    UART_PRINT("Options:\n\r");
    UART_PRINT("  -c channel     : TX channel (0-39), default: 0\n\r");
    UART_PRINT("  -l length      : Test data length (0-255), default: 37\n\r");
    UART_PRINT("  -d payload     : Payload pattern, default: 0\n\r");
    UART_PRINT("                   0 = PRBS9 sequence\n\r");
    UART_PRINT("                   1 = 11110000 pattern\n\r");
    UART_PRINT("                   2 = 10101010 pattern\n\r");
    UART_PRINT("                   3 = PRBS15 sequence\n\r");
    UART_PRINT("                   4 = 11111111 (all 1s)\n\r");
    UART_PRINT("                   5 = 00000000 (all 0s)\n\r");
    UART_PRINT("                   6 = 00001111 pattern\n\r");
    UART_PRINT("                   7 = 01010101 pattern\n\r");
    UART_PRINT("  -p phy         : PHY type, default: 1\n\r");
    UART_PRINT("                   1 = 1M PHY\n\r");
    UART_PRINT("                   2 = 2M PHY\n\r");
    UART_PRINT("                   3 = Coded PHY (S=8, 125 kbps)\n\r");
    UART_PRINT("                   4 = Coded PHY (S=2, 500 kbps)\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("Example: dtm_tx_test -c 17 -l 37 -d 0 -p 1\n\r");
    UART_PRINT("%s\n\r", lineBreak);
    return 0;
}

int32_t printDtmEndUsage(void *arg)
{
    UART_PRINT("\n\r%s\n\r", lineBreak);
    UART_PRINT("%sdtm_test_end\n\r", usageStr);
    UART_PRINT("%sEnd DTM Test and report packet count (HCI 0x201F)\n\r", descriptionStr);
    UART_PRINT("\n\r");
    UART_PRINT("This command ends the current RX or TX test and reports\n\r");
    UART_PRINT("the number of packets received (for RX test).\n\r");
    UART_PRINT("%s\n\r", lineBreak);
    return 0;
}

int32_t cmdDtmSetMaxTxPowerCallback(void *arg)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    uint8_t hciCmd[5];
    int32_t ret;
    int8_t txPowerDbm = 0;  /* Default TX power in dBm */
    uint8_t txPowerIndex;

    /* Parse arguments */
    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, " ");

    while(token)
    {
        if(!strcmp(token, "-help") || !strcmp(token, "-h"))
        {
            printDtmSetMaxTxPowerUsage(NULL);
            return 0;
        }
        else if(!strcmp(token, "-p"))
        {
            token = strtok(NULL, " ");
            if(token) txPowerDbm = (int8_t)atoi(token);
        }
        token = strtok(NULL, " ");
    }

    /* Check if test is already running */
    if(DtmIsTestRunning())
    {
        UART_PRINT("\n\r[DTM] Test already running. Use 'dtm_test_end' first.\n\r");
        return -1;
    }

    /* Convert dBm to power index (floor to nearest valid value):
     * -20 dBm = 0, -10 dBm = 1, -5 dBm = 2, 0 dBm = 3,
     *  5 dBm = 4,  10 dBm = 5, 20 dBm = 6
     */
    if(txPowerDbm >= 20)
    {
        txPowerIndex = 6;
    }
    else if(txPowerDbm >= 10)
    {
        txPowerIndex = 5;
    }
    else if(txPowerDbm >= 5)
    {
        txPowerIndex = 4;
    }
    else if(txPowerDbm >= 0)
    {
        txPowerIndex = 3;
    }
    else if(txPowerDbm >= -5)
    {
        txPowerIndex = 2;
    }
    else if(txPowerDbm >= -10)
    {
        txPowerIndex = 1;
    }
    else
    {
        txPowerIndex = 0;
    }

    UART_PRINT("\n\r[DTM] Setting TX Power to %d dBm\n\r", txPowerDbm);
    UART_PRINT("[DTM] Note: Value will be rounded down to nearest valid step.\n\r");

    /* Build HCI command */
    hciCmd[0] = HCI_CMD_PACKET;
    hciCmd[1] = (DTM_CMD_SET_TX_POWER & 0xFF);
    hciCmd[2] = ((DTM_CMD_SET_TX_POWER >> 8) & 0xFF);
    hciCmd[3] = 1;                    /* Parameter length */
    hciCmd[4] = txPowerIndex;         /* TX power index */

    ret = sendDtmCommand(hciCmd, sizeof(hciCmd));

    return ret;
}

int32_t printDtmSetMaxTxPowerUsage(void *arg)
{
    UART_PRINT("\n\r%s\n\r", lineBreak);
    UART_PRINT("%sdtm_max_tx_pwr [-p power]\n\r", usageStr);
    UART_PRINT("%sSet TX Power for DTM (Vendor HCI 0xFC11)\n\r", descriptionStr);
    UART_PRINT("\n\r");
    UART_PRINT("Options:\n\r");
    UART_PRINT("  -p power       : TX power in dBm, default: 0\n\r");
    UART_PRINT("                   Valid values: -20, -10, -5, 0, 5, 10, 20\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("Example: dtm_max_tx_pwr -p 5\n\r");
    UART_PRINT("%s\n\r", lineBreak);
    return 0;
}
