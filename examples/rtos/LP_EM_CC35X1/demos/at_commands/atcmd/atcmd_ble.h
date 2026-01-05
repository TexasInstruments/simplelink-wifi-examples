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

#ifndef __ATCMD_BLE_H__
#define __ATCMD_BLE_H__

#include <stdint.h>

int32_t ATCmdBle_bleInitCallback(void *arg);

int32_t ATCmdBle_setMacAddressCallback(void *arg);

int32_t ATCmdBle_getMacAddressCallback(void *arg);

int32_t ATCmdBle_setDeviceNameCallback(void *arg);

int32_t ATCmdBle_getDeviceNameCallback(void *arg);

int32_t ATCmdBle_setScanCfgCallback(void *arg);

int32_t ATCmdBle_getScanCfgCallback(void *arg);

int32_t ATCmdBle_scanCallback(void *arg);

int32_t ATCmdBle_connectCallback(void *arg);

int32_t ATCmdBle_disconnectCallback(void *arg);

int32_t ATCmdBle_connectedPeersCallback(void *arg);

int32_t ATCmdBle_setAdvCfgCallback(void *arg);

int32_t ATCmdBle_getAdvCfgCallback(void *arg);

int32_t ATCmdBle_advStartCallback(void *arg);

int32_t ATCmdBle_advStopCallback(void *arg);

#endif /* __ATCMD_BLE_H__ */