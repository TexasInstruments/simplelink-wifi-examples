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
#ifndef __WLAN_CMD_H__
#define __WLAN_CMD_H__

#include <stdint.h>

/******************************************************************************
 Bit Manipulations Macros
 ******************************************************************************/
#define BIT_x(x)                                        (1 << (x))
#define IS_BIT_SET(bit_field, bit_num)                  (((bit_field) & BIT_x(bit_num)) > 0)
#define CLEAR_BIT_IN_BITMAP(bit_field,bit_num)          { (bit_field) &= ~ BIT_x(bit_num) ; }
#define SET_BIT_IN_BITMAP(bit_field,bit_num)            { (bit_field) |=   BIT_x(bit_num) ; }

#define NET_IF_STA_BIT              (0)
#define NET_IF_AP_BIT               (1)
#define NET_IF_IS_UP                (2)
#define NET_IF_DEVICE_BIT           (4)

/******************************************************************************/

/* Function prototypes */

// ****************************************
void printScanResults(uint32_t res_num);

int32_t cmdWlanRoleUpApCallback(void *arg);

int32_t printWlanRoleUpApUsage(void *arg);

int32_t printWlanStartApUsage(void *arg);

int32_t cmdWlanRoleDownApCallback(void *arg);

int32_t printWlanRoleDownApUsage(void *arg);

int32_t cmdWlanRoleUpStaCallback(void *arg);

int32_t printWlanRoleUpStaUsage(void *arg);

int32_t cmdWlanRoleDownStaCallback(void *arg);

int32_t printWlanRoleDownStaUsage(void *arg);

int32_t cmdWlanConnectCallback(void *arg);

int32_t printWlanConnectUsage(void *arg);

int32_t cmdWlanDisconnectCallback(void *arg);

int32_t printWlanDisconnectUsage(void *arg);

int32_t cmdWlanSetScanEarlyTerminationCallback(void *arg);

int32_t cmdWlanGetScanEarlyTerminationCallback(void *arg);

int32_t printSetEarlyTermUsage(void *arg);

int32_t printGetEarlyTermUsage(void *arg);

int32_t cmdScanCallback(void *arg);

int32_t printScanUsage(void *arg);

int32_t printGetMacAddressUsage(void *arg);

int32_t cmdGetMacAddressCallback(void *arg);

int32_t printSetMacAddressUsage(void *arg);

int32_t cmdSetMacAddressCallback(void *arg);

int32_t printGetPsModeUsage(void *arg);

int32_t cmdGetPsModeCallback(void *arg);

int32_t printSetPsModeUsage(void *arg);

int32_t cmdSetPsModeCallback(void *arg);

int32_t cmdGetPmModeCallback(void *arg);

int32_t printSetPmModeUsage(void *arg);

int32_t cmdSetPmModeCallback(void *arg);

int32_t printSetLsiUsage(void *arg);

int32_t cmdSetLsiCallback(void *arg);

int32_t printGetFwVerUsage(void *arg);

int32_t cmdGetFwVerCallback(void *arg);

#ifndef CC35XX
int32_t printEnableBLEUsage(void *arg);

int32_t cmdEnableBLECallback(void *arg);
#endif // CC35XX

#ifdef CC35XX

int32_t printCreateVendoIEListUsage(void *arg);
int32_t printDeleteVendoIEListUsage(void *arg);
int32_t printaddVendoIEUsage(void *arg);
int32_t printDeleteVendoIEUsage(void *arg);

int32_t printConfiPeerAgingUsage(void *arg);


int32_t cmdCreateVendorIEListCallback(void *arg);
int32_t cmdDeleteVendorIEListCallback(void *arg);
int32_t cmdAddVendorIECallback(void *arg);
int32_t cmdDeleteVendorIECallback(void *arg);
int32_t cmdConfigStaAgingEventCallback(void *arg);

int32_t cmdWlanSetScanDwellTimeCallback(void *arg);
int32_t printWlanSetScanDwellTimeUsage(void *arg);

int32_t cmdWlanRoleUpP2PCallback(void *arg);

int32_t printWlanRoleUpP2PUsage(void *arg);

int32_t cmdWlanRoleDownP2PCallback(void *arg);

int32_t printWlanRoleDownP2PUsage(void *arg);

int32_t cmdWlanP2PFindCallback(void *arg);

int32_t printWlanP2PFindUsage(void *arg);

int32_t cmdWlanP2PConnectCallback(void *arg);

int32_t cmdWlanP2PFindStopCallback(void *arg);

int32_t cmdWlanP2PSetChannelCallback(void *arg);

int32_t cmdWlanP2PGrpRemoveCallback(void *arg);

int32_t cmdWlanP2PGetChannelCallback(void *arg);


int32_t printWlan2PConnectUsage(void *arg);

int32_t printWlanP2PFindStopUsage(void *arg);

int32_t printWlanP2PGrpRemoveUsage(void *arg);

int32_t printWlanP2PSetChannelUsage(void *arg);

int32_t printWlanP2PGetChannelUsage(void *arg);

int32_t printWlanP2PListenUsage(void *arg);

int32_t printWlanP2PCancelUsage(void *arg);


int32_t cmdWlanP2PListenCallback(void *arg);

int32_t cmdWlanP2PCancelCallback(void *arg);

int32_t cmdSleepTestCallback(void *arg);

int32_t printSleepTestUsage(void *arg);

int32_t cmdStartApWpsCallback(void *arg);

int32_t printStartApWpsUsage(void *arg);

int32_t cmdSetWsocPrimaryCallback(void *arg);

int32_t printSetWsocPrimaryUsage(void *arg);


int32_t printWlanSetConnPolicyUsage(void *arg);
int32_t cmdWlanSetConnPolicyCallback(void *arg);
int32_t printWlanGetConnPolicyUsage(void *arg);
int32_t cmdWlanGetConnPolicyCallback(void *arg);

int32_t cmdWlanAddProfileCallback(void *arg);
int32_t cmdWlanDeleteProfileCallback(void *arg);
int32_t cmdWlanGetProfileCallback(void *arg);
int32_t cmdWlanProfileConnectCallback(void *arg);

int32_t cmdPingStartCallback(void *arg);
int32_t printPingStartUsage(void *arg);

int32_t cmdPingStopCallback(void *arg);
int32_t printPingStopUsage(void *arg);

int32_t cmdWlanSetRegDomainEntryCallback(void *arg);
int32_t printWlanSetRegDomainEntryUsage(void *arg);

int32_t cmdWlanGetRegDomainEntryCallback(void *arg);
int32_t printWlanGetRegDomainEntryUsage(void *arg);

#endif // CC35XX

int32_t printWlanStartUsage(void *arg);

int32_t cmdWlanStartCallback(void *arg);

int32_t printWlanStopUsage(void *arg);

int32_t cmdWlanStopCallback(void *arg);

int32_t cmdSendEtherCallback(WlanRole_e role, uint8_t *inbuf, uint32_t inbuf_len,uint32_t flags);

int32_t cmdtestCallback(void *arg);

int32_t printCsiEnableUsage(void *arg);

int32_t printCsiStopUsage(void *arg);

int32_t printCsiDisableUsage(void *arg);

int32_t printCsiGetResultsUsage(void *arg);

int32_t cmdCsiEnableCallback (void *arg);

int32_t cmdCsiStopCallback (void *arg);

int32_t cmdCsiDisableCallback (void *arg);

int32_t cmdCsiGetResultsCallback (void *arg);
// ****************************************


int32_t printAddProfileUsage(void *arg);

int32_t printGetProfileUsage(void *arg);

int32_t printDeleteProfileUsage(void *arg);

int32_t printWlanProfileConnectUsage(void *arg);

int32_t cmdSetPolicyCallback(void *arg);

int32_t printSetPolicyUsage(void *arg);


int32_t cmdConnectedStationsCallback(void *arg);

int32_t printConnectedStationsUsage(void *arg);

int32_t cmdCreateFilterCallback(void *arg);

int32_t printCreateFilterUsage(void *arg);

int32_t cmdEnableFilterCallback(void *arg);

int32_t printEnableFilterUsage(void *arg);

int32_t cmdDisableFilterCallback(void *arg);

int32_t printDisableFilterUsage(void *arg);

int32_t cmdDeleteFilterCallback(void *arg);

int32_t printDeleteFilterUsage(void *arg);

int32_t cmdEnableWoWLANCallback(void *arg);

int32_t printEnableWoWLANUsage(void *arg);

int32_t cmdP2PModecallback(void *arg);

int32_t printP2PStartUsage(void *arg);

int32_t cmdP2PStopcallback(void *arg);

int32_t printP2PStopUsage(void *arg);

int32_t	cmdSoftRoamingEnablecallback(void *arg);

int32_t printSoftRoamingEnableUsage(void *arg);

int32_t cmdSoftRoamingDisablecallback(void *arg);

int32_t printSoftRoamingDisableUsage(void *arg);

int32_t cmdAntSelectionEnablecallback(void *arg);

int32_t printAntSelectionEnableUsage(void *arg);

int32_t cmdAntSelectionDisablecallback(void *arg);

int32_t printAntSelectionDisableUsage(void *arg);

int32_t cmdCoexEnablecallback(void *arg);

int32_t printCoexEnableUsage(void *arg);

int32_t printCountrycodeeUsage(void *arg);

int32_t cmdCoexDisablecallback(void *arg);

int32_t printCoexDisableUsage(void *arg);

int32_t cmdCountrycodecallback(void *arg);

int32_t cmdAddProfilecallback(void *arg);

int32_t cmdGetProfilecallback(void *arg);

int32_t printTriggeredRoamingEnableUsage(void *arg);

int32_t cmdTriggeredRoamingEnablecallback(void *arg);

int32_t printTriggeredRoamingDisableUsage(void *arg);

int32_t cmdTriggeredRoamingDisablecallback(void *arg);

int32_t cmdSetInterfaceIpCallback(void *arg);

int32_t cmdGetInterfaceIpCallback(void *arg);

int32_t cmdSetDhcpServerCallback(void *arg);

int32_t cmdGetDhcpServerCallback(void *arg);

int32_t printSetInterfaceIpUsage(void *arg);

int32_t printGetInterfaceIpUsage(void *arg);

int32_t printSetDhcpServerUsage(void *arg);

int32_t printGetDhcpServerUsage(void *arg);

int32_t printAPTransitionEnableUsage(void *arg);

int32_t cmdAPTransitionEnablecallback(void *arg);

int32_t printAPTransitionDisableUsage(void *arg);

int32_t cmdAPTransitionDisablecallback(void *arg);

int32_t CheckSubFrameType(uint8_t *pSubFrameType);

int32_t isNetIFActive(void);

void printFrameSubTyps(void);

//Indigo 
int32_t cmdloadCartificateCallback(void *arg);
int32_t printloadCartificateUsage(void *arg);

#endif /* __WLAN_CMD_H__ */
