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
#ifndef __STR_H__
#define __STR_H__

#include "network_terminal.h"

extern const char lineBreak[];

extern const char cmdPromptStr[];
extern const char usageStr[];
extern const char *descriptionStr;

extern const char helpStr[];
extern const char helpUsageStr[];
extern const char *helpDetailsStr;
extern const char *help_optaionDetails;

extern const char LoadFWcmdStr[];
extern const char LoadFWUsageStr[];
extern const char *LoadFWStr;


extern const char unLoadFWcmdStr[];
extern const char unLoadFWUsageStr[];
extern const char *unLoadFWStr;



extern const char clearStr[];
extern const char *clearDetailsStr;

extern const char exitStr[];
extern const char exitUsageStr[];
extern const char *exitDetailsStr;


extern const char wlanRoleUpApStr[];
extern const char wlan_role_up_ap_UsageStr_first[];
extern const char wlan_role_up_ap_UsageStr_second[];
extern const char *wlan_role_up_ap_DetailsStr;
#ifdef CC35XX
extern const char *wlan_role_up_ap_t_optionDetailsStr;
extern const char *wlan_role_up_ap_w_optionDetailsStr;
#endif


extern const char wlanRoleDownApStr[];
extern const char wlan_role_down_ap_UsageStr[];
extern const char *wlan_role_down_ap_DetailsStr;


extern const char ap_start_str[];
extern const char ap_start_UsageStr[];
extern const char *ap_start_DetailsStr;
extern const char *ap_start_h_optionDetailsStr;
extern const char *ap_start_txp_optionDetailsStr;
extern const char *ap_start_channel_optionDetailsStr;
extern const char *ap_start_l_optionDetailsStr;


extern const char wlanRoleUpStaStr[];
extern const char wlan_role_up_sta_UsageStr[];
extern const char *wlan_role_up_sta_DetailsStr;


extern const char wlanRoleDownStaStr[];
extern const char wlan_role_down_sta_UsageStr[];
extern const char *wlan_role_down_sta_DetailsStr;

extern const char wlanGetEarlyTermStr[];
extern const char getEarlyTermUsageStr[];
extern const char *wlanGetEarlyTermDetailsStr;

extern const char wlanSetEarlyTermStr[];
extern const char setEarlyTermUsageStr[];
extern const char *wlanSetEarlyTermDetailsStr;
extern const char *wlanSetEarlyTerm_e_optionDetailsStr;

extern const char wlanConnectStr[];
extern const char wlanConnectUsageStr[];
extern const char *wlanConnectDetailsStr;
extern const char *wlanConnect_s_optionDetailsStr;
extern const char *wlanConnect_t_optionDetailsStr;
extern const char *wlanConnect_p_optionDetailsStr;
#ifdef CC35XX
extern const char *wlanConnect_e_optionDetailsStr;
extern const char *wlanConnect_i_optionDetailsStr;
extern const char *wlanConnect_ent_usageDetailsStr_1;
extern const char *wlanConnect_ent_usageDetailsStr_2;
extern const char *wlanConnect_ent_usageDetailsStr_3;
extern const char *wlanConnect_ent_usageDetailsStr_4;
#endif

extern const char wlanDisconnectStr[];
extern const char wlanDisconnectUsageStr[];
extern const char *wlanDisconnectDetailsStr;

extern const char scanStr[];
extern const char scanUsageStr[];
extern const char *scanDetailsStr;
extern const char *scan_n_optionDetailsStr;
extern const char *scan_Note_optionDetailsStr;

extern const char GetMacAddressStr[];
extern const char getMacAddressUsageStr[];
extern const char *wlanGetMacAddressDetailsStr;
extern const char *wlanGetMacAddress_i_optionDetailsStr;


extern const char SetMacAddressStr [];
extern const char setMacAddressUsageStr [];
extern const char *wlanSetMacAddressDetailsStr;
extern const char *wlanSetMacAddress_i_optionDetailsStr;
extern const char *wlanSetMacAddress_m_optionDetailsStr;

extern const char GetPsModeStr[];
extern const char getPsModeUsageStr[];
extern const char *wlanGetPsModeDetailsStr;

extern const char SetPsModeStr[];
extern const char setPsModeUsageStr[];
extern const char *wlanSetPsModeDetailsStr;
extern const char *wlanSetPsMode_m_optionDetailsStr;

extern const char SetPmModeStr[];
extern const char setPmModeUsageStr[];
extern const char *wlanSetPmModeDetailsStrAlwaysActive;
extern const char *wlanSetPmModeDetailsStrPowerDown;
extern const char *wlanSetPmModeDetailsStrELP;
extern const char *wlanSetPmMode_m_optionDetailsStr;

extern const char GetFwVerStr[];
extern const char getFwVerUsageStr[];
extern const char *wlanGetFwVerDetailsStr;

extern const char SetInterfaceIpStr[]; 
extern const char SetInterfaceIpUsageStr[];
extern const char SetInterfaceIpDetailsStr[];

extern const char GetInterfaceIpStr[];
extern const char GetInterfaceIpUsageStr[];
extern const char GetInterfaceIpDetailsStr[];

extern const char SetDhcpServerStr[];
extern const char SetDhcpServerUsageStr[];
extern const char SetDhcpServerDetailsStr[];

extern const char GetDhcpServerStr[];
extern const char GetDhcpServerUsageStr[];
extern const char GetDhcpServerDetailsStr[];

extern const char SetLsiStr[];
extern const char setLsiUsageStr[];
extern const char *wlanSetLsiDetailsStr;
extern const char *wlanSetLsi_n_optionDetailsStr;

#ifdef CC35XX
extern const char SleepTestStr[];
extern const char SleepTestUsageStr[];
extern const char *sleepTestDetailsStr;
extern const char *sleepTest_t_optionDetailsStr;
#endif

extern const char wlanStartStr[]      ;
extern const char wlanStartUsageStr[] ;
extern const char *wlanStartDetailsStr;

extern const char wlanStopStr[]      ;
extern const char wlanStopUsageStr[] ;
extern const char *wlanStopDetailsStr;

#ifdef CC35XX
extern const char wlanRoleUpP2PStr[];
extern const char wlan_role_up_p2p_UsageStr_first[];
extern const char wlan_role_up_p2p_UsageStr_second[];
extern const char wlan_role_up_p2p_UsageStr_third[];
extern const char *wlan_role_up_p2p_DetailsStr;
extern const char wlan_role_up_p2p_i_optionDetailsStr[];

extern const char *wlan_p2p_connect_DetailsStr;

extern const char wlanRoleDownP2PStr[];
extern const char *wlan_role_down_p2p_DetailsStr;

extern const char wlanP2PFindStr[];
extern const char printWlanP2PFindUsageStr[];

extern const char wlanP2PConnectStr[];
extern const char wlan_p2p_connect_UsageStr_first[];
extern const char wlan_p2p_connect_UsageStr_second[];
extern const char *wlan_p2p_find_stop_DetailsStr;

extern const char  wlan_role_up_group_remove_UsageStr_third[];

extern const char *wlan_p2p_group_remove_DetailsStr;

extern const char *wlan_role_up_p2pSetChannel_DetailsStr;

extern const char *wlan_role_up_p2pGetChannel_DetailsStr;

extern const char *wlan_role_up_p2pListen_DetailsStr;

extern const char *wlan_role_up_p2pCancel_DetailsStr;

extern const char wlanP2PStopFindStr[];

extern const char wlanP2PGrpRemoveStr[];

extern const char wlanP2PSetchannelStr[];

extern const char wlanP2PGetchannelStr[];

extern const char wlanP2PListenStr[];

extern const char wlanP2PCancelStr[];


extern const char wlanSetConnPolicyStr[];
extern const char wlanSetConnPolicyUsageStr[];
extern const char *wlanSetConnPolicyDetailsStr;
extern const char *wlanSetConnPolicy_a_optionDetailsStr;
extern const char *wlanSetConnPolicy_f_optionDetailsStr;
extern const char *wlanSetConnPolicy_p_optionDetailsStr;

extern const char wlanGetConnPolicyStr[];
extern const char wlanGetConnPolicyUsageStr[];
extern const char *wlanGetConnPolicyDetailsStr;

extern const char wlanAddProfileStr[];
extern const char wlanAddProfileUsageStr[];
extern const char *wlanAddProfileDetailsStr;
extern const char *wlanAddProfile_s_optionDetailsStr;
extern const char *wlanAddProfile_t_optionDetailsStr;
extern const char *wlanAddProfile_p_optionDetailsStr;
extern const char *wlanAddProfile_pr_optionDetailsStr;
extern const char *wlanAddProfile_h_optionDetailsStr;

extern const char wlanGetProfileStr[];
extern const char wlanGetProfileUsageStr[];
extern const char *wlanGetProfileDetailsStr;
extern const char *wlanGetProfile_i_optionDetailsStr;

extern const char wlanDelProfileStr[];
extern const char wlanDelProfileUsageStr[];
extern const char *wlanDelProfileDetailsStr;
extern const char *wlanDelProfile_i_optionDetailsStr;

extern const char wlanProfileConnectStr[];
extern const char wlanProfileConnectUsageStr[];
extern const char *wlanProfileConnectDetailsStr;
extern const char *wlanProfileConnect_i_optionDetailsStr;

#endif

extern const char emptyDeviceStr[];

extern const char bleAdvCfgStr[];
extern const char bleAdvCfgUsageStr[];
extern const char *bleAdvCfgDetailsStr;

extern const char bleAdvEnableStr[];
extern const char bleAdvEnableUsageStr[];
extern const char *bleAdvEnableDetailsStr;

extern const char bleScanCfgStr[];
extern const char bleScanCfgUsageStr[];
extern const char *bleScanCfgDetailsStr;

extern const char bleScanEnableStr[];
extern const char bleScanEnableUsageStr[];
extern const char *bleScanEnableDetailsStr;

extern const char bleConnectStr[];
extern const char bleConnectUsageStr[];
extern const char bleConnectDetailsStr[];

extern const char bleDisconnectStr[];
extern const char bleDisconnectUsageStr[];
extern const char bleDisconnectDetailsStr[];

extern const char blePeersStr[];
extern const char blePeersUsageStr[];
extern const char blePeersDetailsStr[];

extern const char bleDeleteBondsStr[];
extern const char bleDeleteBondsUsageStr[];
extern const char bleDeleteBondsDetailsStr[];

extern const char bleGetBdAddrStr[];
extern const char bleGetBdAddrUsageStr[];
extern const char bleGetBdAddrDetailsStr[];

extern const char bleSetBdAddrStr[];
extern const char bleSetBdAddrUsageStr[];
extern const char bleSetBdAddrDetailsStr[];

extern const char bleStartStr[];
extern const char bleStartUsageStr[];
extern const char bleStartDetailsStr[];

extern const char bleStopStr[];
extern const char bleStopUsageStr[];
extern const char bleStopDetailsStr[];

extern char csiEnableStr[];
extern char csiEnableUsageStr[];
extern char csiEnableDetailsStr[];

extern char csiStopStr[];
extern char csiStopUsageStr[];
extern char csiStopDetailsStr[];

extern char csiDisableStr[];
extern char csiDisableUsageStr[];
extern char csiDisableDetailsStr[];

extern char csiGetResultsStr[];
extern char csiGetResultsUsageStr[];
extern char csiGetResultsDetailsStr[];

extern const char sendStr[];
extern const char sendUsage1Str[];
extern const char sendUsage2Str[];
extern const char sendDetailsStr[];
extern const char send_c_optionDetailsStr[];
extern const char send_s_optionDetailsStr[];
extern const char send_u_optionDetailsStr[];
extern const char send_p_optionDetailsStr[];
extern const char send_nb_optionDetailsStr[];
extern const char send_n_optionDetailsStr[];
extern const char send_6_optionDetailsStr[];
extern const char send_7_optionDetailsStr[];
extern const char iperf_v_optionDetailsStr[];
extern const char iperf_t_optionDetailsStr[];


extern const char TestIperf[];
extern const char recvTestIperfDetailsStr[];
extern const char recvTestIperfUsage2Str[];
extern const char *recvTestIperf_s_optionDetailsStr;
extern const char *recvTestIperf_c_optionDetailsStr;
extern const char *recvTestIperf_p_optionDetailsStr;
extern const char *recvTestIperf_u_optionDetailsStr;
extern const char recvTestIperf_i_optionDetailsStr[];
extern const char *recvTestIperf_t_optionDetailsStr;
extern const char *recvTestIperf_b_optionDetailsStr;
extern const char *recvTestIperf_B_optionDetailsStr;

extern const char StopTestIperf[];
extern const char *recvStopTestIperfUsage2Str;
extern const char *recvStopTestIperfDetailsStr;
extern const char *recvStopTestIperf_s_optionDetailsStr;
extern const char *recvStopTestIperf_n_optionDetailsStr;

extern const char recvStr[];
extern const char *recvUsage1Str;
extern const char *recvUsage2Str;
extern const char recvDetailsStr[];
extern const char *recv_c_optionDetailsStr;
extern const char *recv_s_optionDetailsStr;
extern const char *recv_u_optionDetailsStr;
extern const char *recv_p_optionDetailsStr;
extern const char *recv_nb_optionDetailsStr;
extern const char send_n_r_optionDetailsStr[];
extern const char *recv_6_optionDetailsStr;
extern const char recv_Note_optionDetailsStr[];

extern const char testStr[];
extern const char testUsageStr[];
extern const char testDetailsStr[];

extern const char showStr[];
extern const char showUsageStr[];
extern const char *showDetailsStr;

extern const char killStr[];
extern const char killUsageStr[];
extern const char *killDetailsStr;

extern const char SntpConfigNTPServers[];
extern const char SntpConfigNTPServersUsageStr[];
extern const char SntpConfigNTPServers_s_optionDetailsStr[];
extern const char SntpConfigNTPServers_s_optionDetailsStrExpand[];
extern const char SntpUpdateDateTime[];
extern const char SntpUpdateTimeUsageStr[];
extern const char SetDateTime[];
extern const char SetDateTimeUsageStr[] ;
extern const char SetDateTime_t_optionDetailsStr[];
extern const char GetDateTime[];
extern const char GetDateTimeUsageStr[];
extern const char GetDateTime_t_optionDetailsStr[];
extern const char getRoleIdUsageStr[];

#ifdef CC35XX
extern const char wlanCreateVendorIEStr[];
extern const char wlanDeleteVendorIEStr[];
extern const char wlanAddVendorIEStr[];
extern const char wlanRemoveVendorIEStr[];
extern const char wlan_AddVendorIE_UsageStr_first[];
extern const char wlan_AddVendorIE_UsageStr_second[];
extern const char wlan_DeleteVendorIE_UsageStr_first[];
extern const char wlan_DeleteVendorIE_UsageStr_second[];

extern const char wlanConfigPeerAgingStr[];
extern const char wlan_ConfigPeerAging_UsageStr_first[];
extern const char wlan_ConfigPeerAging_UsageStr_second[];
extern const char wlan_ConfigPeerAging_DetailsStr[];

extern const char startApWpsStr[];
extern const char startApWpsUsageStr[];
extern const char startApWpsDetailsStr[];
extern const char startApWps_w_optionDetailsStr[];
extern const char startApWps_p_optionDetailsStr[];

extern const char pingStartStr[];
extern const char pingStartUsageStr[];
extern const char pingStartDetailsStr[];
extern const char pingStart_h_optionDetailsStr[];
extern const char pingStart_c_optionDetailsStr[];
extern const char pingStart_i_optionDetailsStr[];
extern const char pingStart_s_optionDetailsStr[];
extern const char pingStart_I_optionDetailsStr[];

extern const char pingStopStr[];
extern const char pingStopUsageStr[];
extern const char pingStopDetailsStr[];
extern const char pingStop_i_optionDetailsStr[];

extern const char wlanSetRegDomEntryStr[];
extern const char wlanSetRegDomEntryUsageStr[];
extern const char wlanSetRegDomEntryDetailsStr[];
extern const char wlanSetRegDomEntry_i_optionDetailsStr[];
extern const char wlanSetRegDomEntry_b_optionDetailsStr[];
extern const char wlanSetRegDomEntry_p_optionDetailsStr[];
extern const char wlanSetRegDomEntry_d_optionDetailsStr[];
extern const char wlanSetRegDomEntry_n_optionDetailsStr[];
extern const char wlanSetRegDomEntry_bm_optionDetailsStr[];
extern const char wlanSetRegDomEntry_r_optionDetailsStr[];

extern const char wlanGetRegDomEntryStr[];
extern const char wlanGetRegDomEntryUsageStr[];
extern const char wlanGetRegDomEntryDetailsStr[];
extern const char wlanGetRegDomEntry_i_optionDetailsStr[];

#endif // CC35XX

extern const char a_optionStr[];
extern const char b_optionStr[];
extern const char B_optionStr[];
extern const char c_optionStr[];
extern const char f_optionStr[];
extern const char g_optionStr[];
extern const char r_optionStr[];
extern const char h_optionStr[];
extern const char i_optionStr[];
extern const char l_optionStr[];
extern const char n_optionStr[];
extern const char o_optionStr[];
extern const char p_optionStr[];
extern const char s_optionStr[];
extern const char t_optionStr[];
extern const char w_optionStr[];
extern const char I_optionStr[];
extern const char st_optionStr[];
extern const char tx_optionStr[];
extern const char tone_optionStr[];
extern const char cca_optionStr[];
extern const char u_optionStr[];
extern const char v_optionStr[];
extern const char d_optionStr[];
extern const char e_optionStr[];
extern const char x_optionStr[];
extern const char m_optionStr[];
extern const char ch_optionStr[];
extern const char fs_optionStr[];
extern const char nb_optionStr[];
extern const char lh_optionStr[];
extern const char dp_optionStr[];
extern const char V_optionStr[];
extern const char ip_optionStr[];
extern const char mac_optionStr[];
extern const char priority_optionStr[];
extern const char dns_optionStr[];
extern const char gw_optionStr[];
extern const char ent_optionStr[];
extern const char help_optionStr[];
extern const char txpow_optionStr[];
extern const char ttl_optionStr[];
extern const char so_optionStr[];
extern const char ext_optionStr[];
extern const char bitmap_optionStr[];
extern const  char min_dwell_active_optionStr[];
extern const  char max_dwell_active_optionStr[];
extern const  char min_dwell_passive_optionStr[];
extern const  char max_dwell_passive_optionStr[];
extern const  char dfs_dwell_passive[];

extern const char wlanSetScanDwellTimeStr[];
extern const char wlanSetScanDwellTimeUsageStr[];
extern const char *wlanSetScanDwellTimeDetailsStr;
extern const char WPA_str[];
extern const char WPA2_str[];
extern const char WPS_str[];
extern const char OPEN_str[];
extern const char WPAWPA2_str[];
extern const char WPA2_PLUS_str[];
extern const char WPA3_str[];
extern const char WPA2WPA3_str[];
extern const  char TLS_str[];
extern const  char TTLS_MSCHAP_str[];
extern const  char PEAP0_MSCHAP_str[];
extern const  char PEAP2_MSCHAP_str[];
extern const  char PEAP0_GTC_str[];
extern const  char PEAP2_GTC_str[];
extern const char REQUIRED_str[];
extern const char CAPABLE_str[];
extern const char DISABLE_str[];
extern const char YES_str[];
extern const char NO_str[];
extern const char space_str[];
extern const char CONNECTED_str[];
extern const char PROMISCUOUS_str[];
extern const char SOURCE_MAC_str[];
extern const char DESTINATION_MAC_str[];
extern const char BSSID_str[];
extern const char S_IP_str[];
extern const char D_IP_str[];
extern const char FRAME_TYPE_str[];
extern const char FRAME_SUBTYPE_str[];
extern const char PATTERN_str[];
extern const char PUBLIC_str[];
extern const char RANDOM_str[];
extern const char *MangmentFrames_str[];
extern const char *CtrlFrames_str[];
extern const char *DataFrames_str[];


extern const char fsError[];
extern const char socketError[];
extern const char netappError[];
extern const char cmdError[];
extern const char nwError[];

extern const char SetWsocPrimaryStr[];
extern const char SetWsocPrimaryUsageStr[];
extern const char *SetWsocPrimaryDetailsStr;

extern const char loadCertificate[];
extern const char loadCertificateUsageStr[];
extern const char loadCertificateDetailsStr[];

void printBorder(char ch,
                 int n);

#endif /* __STR_H__ */
