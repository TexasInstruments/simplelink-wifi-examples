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
#ifndef __BLE_CMD_H__
#define __BLE_CMD_H__

#include <stdint.h>

/* Function prototypes */

// ****************************************
int32_t cmdBleAdvCfgCallback(void *arg);

int32_t printBleAdvCfgUsage(void *arg);

int32_t cmdBleAdvEnableCallback(void *arg);

int32_t printBleAdvEnableUsage(void *arg);

int32_t cmdBleScanCfgCallback(void *arg);

int32_t printBleScanCfgUsage(void *arg);

int32_t cmdBleScanEnableCallback(void *arg);

int32_t printBleScanEnableUsage(void *arg);

int32_t cmdBleConnectCallback(void *arg);

int32_t printBleConnectUsage(void *arg);

int32_t cmdBleDisconnectCallback(void *arg);

int32_t printBleDisconnectUsage(void *arg);

int32_t cmdBlePeersCallback(void *arg);

int32_t printBlePeersUsage(void *arg);

int32_t cmdBleDeleteBondsCallback(void *arg);

int32_t printBleDeleteBondsUsage(void *arg);

int32_t cmdBleGetBdAddrCallback(void *arg);

int32_t printBleGetBdAddrUsage(void *arg);

int32_t cmdBleSetBdAddrCallback(void *arg);

int32_t printBleSetBdAddrUsage(void *arg);

int32_t cmdBleStartCallback(void *arg);

int32_t printBleStartUsage(void *arg);

int32_t cmdBleStopCallback(void *arg);

int32_t printBleStopUsage(void *arg);

#endif /* __BLE_CMD_H__ */
