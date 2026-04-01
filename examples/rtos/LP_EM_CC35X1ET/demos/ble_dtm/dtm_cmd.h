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

#ifndef __DTM_CMD_H__
#define __DTM_CMD_H__

#include <stdint.h>
#include "osi_kernel.h"

/****************************************************************************
 *                      HCI DEFINITIONS
 ****************************************************************************/
/* HCI Packet Type */
#define HCI_CMD_PACKET                         (0x01)
#define HCI_EVENT_PACKET                       (0x04)

/* HCI OGF and OCF definitions */
#define BLE_HCI_OGF_LE                         (0x08)
#define BLE_HCI_OGF_VENDOR                     (0x3F)
#define BLE_HCI_OP(ogf, ocf)                   ((ocf) | ((ogf) << 10))

/* DTM Command OCFs */
#define BLE_HCI_OCF_LE_TEST_END                (0x001F)
#define BLE_HCI_OCF_LE_RX_TEST_V2              (0x0033)
#define BLE_HCI_OCF_LE_TX_TEST_V2              (0x0034)
#define BLE_HCI_OCF_VS_SET_TX_POWER            (0x0011)

/* DTM Command Opcodes (16-bit: OGF << 10 | OCF) */
#define DTM_CMD_RX_TEST_V2                     BLE_HCI_OP(BLE_HCI_OGF_LE, BLE_HCI_OCF_LE_RX_TEST_V2)  /* 0x2033 */
#define DTM_CMD_TX_TEST_V2                     BLE_HCI_OP(BLE_HCI_OGF_LE, BLE_HCI_OCF_LE_TX_TEST_V2)  /* 0x2034 */
#define DTM_CMD_TEST_END                       BLE_HCI_OP(BLE_HCI_OGF_LE, BLE_HCI_OCF_LE_TEST_END)    /* 0x201F */
#define DTM_CMD_SET_TX_POWER                   BLE_HCI_OP(BLE_HCI_OGF_VENDOR, BLE_HCI_OCF_VS_SET_TX_POWER) /* 0xFC11 */

/* HCI Event codes */
#define HCI_EVENT_CMD_COMPLETE                 (0x0E)
#define HCI_EVENT_VENDOR_SPECIFIC              (0xFF)

/* HCI Event header size: packet type (1) + event code (1) + param length (1) */
#define HCI_EVENT_HDR_SIZE                     (3)

/* Minimum param length for Command Complete: numCmd (1) + opcode (2) + status (1) */
#define HCI_CMD_COMPLETE_MIN_PARAM_LEN         (4)

/* Minimum param length for Vendor Specific: vendorOpcode (2) + status (1) + cmdOpcode (2) */
#define HCI_VENDOR_SPECIFIC_MIN_PARAM_LEN      (5)

/****************************************************************************
 *                      DTM PARAMETERS DEFINITIONS
 ****************************************************************************/
/* PHY types */
#define DTM_PHY_1M                             (0x01)
#define DTM_PHY_2M                             (0x02)
#define DTM_PHY_CODED_S8                       (0x03)  /* Coded PHY with S=8 (125 kbps) */
#define DTM_PHY_CODED_S2                       (0x04)  /* Coded PHY with S=2 (500 kbps) */

/* Payload patterns */
#define DTM_PAYLOAD_PRBS9                      (0x00)
#define DTM_PAYLOAD_11110000                   (0x01)
#define DTM_PAYLOAD_10101010                   (0x02)
#define DTM_PAYLOAD_PRBS15                     (0x03)
#define DTM_PAYLOAD_11111111                   (0x04)
#define DTM_PAYLOAD_00000000                   (0x05)
#define DTM_PAYLOAD_00001111                   (0x06)
#define DTM_PAYLOAD_01010101                   (0x07)

/* Modulation index */
#define DTM_MOD_INDEX_STANDARD                 (0x00)
#define DTM_MOD_INDEX_STABLE                   (0x01)

/* Channel range */
#define DTM_CHANNEL_MIN                        (0)
#define DTM_CHANNEL_MAX                        (39)

/* Test data length range */
#define DTM_DATA_LEN_MIN                       (0)
#define DTM_DATA_LEN_MAX                       (255)

/****************************************************************************
 *                      DTM STATE DEFINITIONS
 ****************************************************************************/
/* DTM Test state */
typedef enum
{
    DTM_STATE_IDLE = 0,
    DTM_STATE_RX_TEST,
    DTM_STATE_TX_TEST
} dtm_state_t;

/****************************************************************************
 *                      DTM PARAMETER STRUCTURES
 ****************************************************************************/
/* RX Test parameters */
typedef struct
{
    uint8_t channel;
    uint8_t phy;
    uint8_t modulation_index;
} dtm_rx_params_t;

/* TX Test parameters */
typedef struct
{
    uint8_t channel;
    uint8_t data_len;
    uint8_t payload;
    uint8_t phy;
} dtm_tx_params_t;

/* DTM Result structure */
typedef struct
{
    uint8_t  status;        /* HCI status code */
    uint16_t packetCount;   /* Packet count from test end */
} dtm_result_t;

/****************************************************************************
 *                      DTM CONTROL BLOCK
 ****************************************************************************/
typedef struct dtm_control_block_t
{
    dtm_state_t   state;              /* Current DTM state */
    OsiSyncObj_t  responseSyncObj;    /* Sync object for response */
    uint8_t       hciStatus;          /* Last HCI status */
    uint16_t      packetCount;        /* Packet count from test end */
} dtm_control_block_t;

extern dtm_control_block_t dtm_CB;

/****************************************************************************
 *                      DTM COMMAND CALLBACKS
 ****************************************************************************/

int32_t cmdDtmRxCallback(void *arg);

int32_t printDtmRxUsage(void *arg);

int32_t cmdDtmTxCallback(void *arg);

int32_t printDtmTxUsage(void *arg);

int32_t cmdDtmEndCallback(void *arg);

int32_t printDtmEndUsage(void *arg);

int32_t cmdDtmSetMaxTxPowerCallback(void *arg);

int32_t printDtmSetMaxTxPowerUsage(void *arg);

/****************************************************************************
 *                      DTM INTERNAL FUNCTIONS
 ****************************************************************************/

/*!
 * \brief Initialize DTM module
 *
 * \return 0 on success, negative on error
 */
int32_t DtmInit(void);

/*!
 * \brief Register DTM event handler with BLE interface
 *
 * \return 0 on success, negative on error
 */
int32_t DtmRegisterEventHandler(void);

/*!
 * \brief Get current DTM state
 *
 * \return Current DTM state
 */
dtm_state_t DtmGetState(void);

/*!
 * \brief Check if DTM test is running
 *
 * \return 1 if test is running, 0 otherwise
 */
int32_t DtmIsTestRunning(void);

#endif /* __DTM_CMD_H__ */
