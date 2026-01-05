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
#include "network_terminal.h"
#include "uart_term.h"


const char lineBreak[]                = "\n\r";

const char cmdPromptStr[]             = "user:";
const char usageStr[]                 = "Usage: \n\r\t";
const char *descriptionStr            = "\n\rDescription:\n\r\t";

const char helpStr[]                  = "help";
const char helpUsageStr[]             = " [command name] \n\r";
const char *helpDetailsStr            = "To know more about the command name.\n\r";
const char *help_optaionDetails       = "\n\r\t-help\tDisplay this help\n\r\n\r";

const char clearStr[]                 = "clear";
const char *clearDetailsStr           = "To clear the terminal screen.\n\r";


/* Role up AP */
const char wlanRoleUpApStr[]          = "wlan_ap_role_up";
const char wlan_role_up_ap_UsageStr_first[]   =  " [-help] [-s <\"ssid name\">] "
                                        "[-t <security type>] [-p <\"password\">]"
                                        "[-h <hidden AP>]\n\r\t[-txp <Tx power for 2.4GHz channels only [0-15]>]";
const char wlan_role_up_ap_UsageStr_second[]  =  "[-c <Wlan channel>]"
                                        "[-l <STA connection limit [1-4]>]"
                                        "[-r <AP Regulatory Domain [\"US\" \"JP\" \"00\"]>]\n\r";
#ifdef CC35XX
const char *wlan_role_up_ap_t_optionDetailsStr    = "\n\r\t-t\tType of security "
                                          "(security type = "
                                          "[OPEN, WPA2, WPA2_PLUS, WPA3, WPA2/WPA3])\n\r";
const char *wlan_role_up_ap_w_optionDetailsStr    = "\n\r\t-w\tSAE PWE Type "
                                          "(SAE PWE = "
                                          "0 - hunting-and-pecking, 1 - hash-to-element, 2 - both)\n\r";
#endif // CC35XX	
const char *wlan_role_up_ap_DetailsStr    = "Role up device as AP.\n\r\t"
                                            "Note: CC350X devices do not support 5Ghz, therefore choosing a 5GHz operating channel will fail.\n\r";


/* Role down AP */
const char wlanRoleDownApStr[]          = "wlan_ap_role_down";
const char wlan_role_down_ap_UsageStr[]     = " [-help]";
const char *wlan_role_down_ap_DetailsStr    = "role down AP device.\n\r";

/* Role up STA */
const char wlanRoleUpStaStr[]          = "wlan_sta_role_up";
const char wlan_role_up_sta_UsageStr[]     = " [-help] \n\r"
                                      "[-r <STA Regulatory Domain [\"US\" \"JP\" \"EU\" \"00\" \"CS\"]>] Default Parameter:\"00\" \n\r";
const char *wlan_role_up_sta_DetailsStr    = "Role up device as STA.\n\r\t"
                                             "Note: CC350X devices do not support 5GHz band.\n\r";


/* Role down STA */
const char wlanRoleDownStaStr[]          = "wlan_sta_role_down";
const char wlan_role_down_sta_UsageStr[]     = " [-help] \n\r";
const char *wlan_role_down_sta_DetailsStr    = "role down device STA.\n\r";

/* Get Early Termination setting */
const char wlanGetEarlyTermStr[]                 = "wlan_get_early_term";
const char getEarlyTermUsageStr[]                = " [-help]\n\r";
const char *wlanGetEarlyTermDetailsStr           = "Get current connection scan early termination setting. 0 - Disabled, 1 - Enabled\n\r";

/* Set Early Termination setting */
const char wlanSetEarlyTermStr[]                 = "wlan_set_early_term";
const char setEarlyTermUsageStr[]                = " [-help] [-e <EarlyTermSetting>]\n\r";
const char *wlanSetEarlyTermDetailsStr           = "Set connection scan early termination setting. 0 - Disabled, 1 - Enabled\n\r";
const char *wlanSetEarlyTerm_e_optionDetailsStr  = "\n\r\t-e\tEarly termination setting (0=Disabled, 1=Enabled)\n\r";

/* Connect */
const char wlanConnectStr[]             = "wlan_connect";
const char wlanConnectUsageStr[]        = " [-help] [-s <\"ssid name\">] "
                                    "[-t <security type>]"
                                  " [-p <\"password\">]\n\r\t[-ip <static ip>]"
                                  " [-gw <static gw>] [-dns <static dns>][-e <enterprise>]"
                                  "[-i <enterprise identity>]\n\r ";

const char *wlanConnectDetailsStr       = "Connect .\n\r";
const char *wlanConnect_s_optionDetailsStr    = "\n\r\t-s\tSSID\n\r";
#ifdef CC33XX
const char *wlanConnect_t_optionDetailsStr    = "\n\r\t-t\tType of security "
                                          "(security type = "
                                          "[OPEN, WPA2, WPA3])\n\r";
#endif // CC33XX
#ifdef CC35XX
const char *wlanConnect_t_optionDetailsStr    = "\n\r\t-t\tType of security "
                                                "(security type = "
                                                "[OPEN, WPA, WPA2, WPA2_PLUS, WPA3, WPS, WPA2/WPA3])\n\r";
#endif // CC35XX									  

const char *wlanConnect_p_optionDetailsStr    = "\n\r\t-p\tPassword in ascii "
                                          "character (pin in case of WPS)\n\r";

#ifdef CC35XX

const char *wlanConnect_e_optionDetailsStr="\n\r\t-e\tEnterprise connection"
        "(EAP methods = "
        "[TLS, TTLS_MSCHAP, PEAP0_MSCHAP])\n\r";

const char *wlanConnect_i_optionDetailsStr="\n\r\t-i\tEnterprise identity\n\r";


const char *wlanConnect_ent_usageDetailsStr_1= "\n\r note: For enterprise the possible security types : WPA2, WPA2_PLUS, WPA3 ,WPA2/WPA3";
const char *wlanConnect_ent_usageDetailsStr_2 = "\n\r TLS :        wlan_connect -t WPA2/WPA3 -e TLS -s \"QuickTrack_1652185873\" -i \"Client Certificate IDL\" ";
const char *wlanConnect_ent_usageDetailsStr_3 = "\n\r TTLS_MSCHAP: wlan_connect -t WPA2/WPA3 -e TTLS_MSCHAP -s \"QuickTrack_1652185873\" -i \"wifi-user@ttls\" -p \"test11\" ";
const char *wlanConnect_ent_usageDetailsStr_4 = "\n\r PEAP0_MSCHAP: wlan_connect -t WPA3 -e PEAP0_MSCHAP -s \"QuickTrack_1652185873\" -i \"wifi-user@peap1\" -p \"test11\" ";
#endif


/* Disconnect */
const char wlanDisconnectStr[]         = "wlan_disconnect";
const char wlanDisconnectUsageStr[]    = " [-help] "
                                         " [-m <macAddress>]\n\r ";
const char *wlanDisconnectDetailsStr   = "Disconnect from AP.\n\r";


/* Wlan start ap */
const char ap_start_str[]         = "wlan_ap_start";
const char ap_start_UsageStr[]    = " [-help] [-s <\"ssid name\">] "
                              "[-t <security type>] [-p <\"password\">]"
                              "[-h <hidden AP>]\n\r\t[-txp <Tx power for 2.4GHz channels only [0-15]>]"
                              "[-c <Wlan channel [2.4GHz: 1-13 , 5GHz:36,40,44,48]>] "
                              "[-l <STA connection limit [1-4]>]\n\r";
const char *ap_start_DetailsStr    = "Set device in AP mode.\n\r";

const char ap_stop_str[]          = "wlan_ap_stop";
const char ap_stop_UsageStr[]     = " [-help] \n\r";
const char *ap_stop_DetailsStr    = "Set device in Station mode.\n\r";

const char *ap_start_h_optionDetailsStr       = "\n\r\t-h\tStart the AP in hidden"
                                          " mode (hidden mode = "
                                          "[YES, NO])\n\r";
const char *ap_start_txp_optionDetailsStr     = "\n\r\t-txp\tSet Wlan "
                                          "Tx power for 2.4GHz channels only"
                                          " (0 = Max Power)\n\r";
const char *ap_start_channel_optionDetailsStr = "\n\r\t-c\tSet channel "
                                          "for the AP\n\r";
const char *ap_start_l_optionDetailsStr       = "\n\r\t-l\tLimit the number of"
                                          " connected stations\n\r";


/* Scan */
const char scanStr[]                  = "wlan_scan";
const char scanUsageStr[]             =
" [-help] [-n <number of AP to scan>]\n\r";
const char *scanDetailsStr            = "Scan for available APs. Maximum 30 APs can"
                                  " be listed.\n\r";

const char *scan_n_optionDetailsStr    = "\n\r\t-n\tMaximum number of Scan"
                                   " results to show\n\r";
const char *scan_Note_optionDetailsStr = "\tNote:\tIf no policy is set,"
                                   " 'Scan' command issue One-shot scan, "
                                   "which shuts off immediately\n\r";

/* Get MacAddress */
const char GetMacAddressStr[]                      = "wlan_get_mac";
const char getMacAddressUsageStr[]                 = " [-help] [-i <RoleType>] \n\r";
const char *wlanGetMacAddressDetailsStr            = "Get MacAddress. role type 0 - STA , 2 - AP, 3 - Device \n\r";
const char *wlanGetMacAddress_i_optionDetailsStr      = "\n\r\t-i\tRoleType, if role type not found then error is returned\n\r";

/* Set MacAddress */
const char SetMacAddressStr[]                      = "wlan_set_mac";
const char setMacAddressUsageStr[]                 = " [-help] [-i <RoleType>] [-m <macAdress>]\n\r";
const char *wlanSetMacAddressDetailsStr            = "Set MacAddress.role type 0 - STA , 2 - AP \n\r";
const char *wlanSetMacAddress_i_optionDetailsStr      = "\n\r\t-i\tRoleType, if role type does not exists error is return\n\r";
const char *wlanSetMacAddress_m_optionDetailsStr      = "\n\r\t-m\tMacAddress, if MacAdrress does not valid error is return\n\r";

/* Get Power save mode */
const char GetPsModeStr[]                          = "wlan_get_ps";
const char getPsModeUsageStr[]                     = " [-help]\n\r";
const char *wlanGetPsModeDetailsStr                = "Get current power save mode. 0 - Auto PS , 1 - Active mode, 2 - power save mode \n\r";

/* Set power save mode */
const char SetPsModeStr[]                        = "wlan_set_ps";
const char setPsModeUsageStr[]                   = " [-help] [-m <PowerSaveMode>]\n\r";
const char *wlanSetPsModeDetailsStr              = "Set power save mode. 0 - Auto PS , 1 - Active mode, 2 - power save mode \n\r";
const char *wlanSetPsMode_m_optionDetailsStr     = "\n\r\t-m\tMode, if device not started error is return\n\r";

/* Set Power management  mode     */
const char SetPmModeStr[]                        = "wlan_set_pm";
const char setPmModeUsageStr[]                   = " [-help] [-m <PowerManagementMode>]\n\r";
const char *wlanSetPmModeDetailsStrAlwaysActive  = "Set power management mode:\n\r\t\t0 - Always active mode\n\r\t";
const char *wlanSetPmModeDetailsStrPowerDown     = "\t1 - Power down mode (light / fast sleep)\n\r\t";
const char *wlanSetPmModeDetailsStrELP           = "\t2 - ELP mode (Deep / Max sleep)\n\r";
const char *wlanSetPmMode_m_optionDetailsStr     = "\n\r\t-m\tMode, if device not started error is return\n\r";

/* Set Interface Ip mode     */
const char SetInterfaceIpStr[]                   ="wlan_set_if_ip";
const char SetInterfaceIpUsageStr[]              = " [-help],[-i <RoleType>],[-ip <IP Mode>],[-c <ip address>],[-v <Netmask>],[-gw <gw>]>\n\r\t";
const char SetInterfaceIpDetailsStr[]            = "role type: 0 - STA , 2 - AP.  IP Mode: 0 - DHCP , 1 - STATIC\n\r\t";

/* Get Interface Ip mode     */
const char GetInterfaceIpStr[]                  = "wlan_get_if_ip";
const char GetInterfaceIpUsageStr[]             = "[-help],[-i <RoleType>]\n\r\t";
const char GetInterfaceIpDetailsStr[]           = "role type 0 - STA , 2 - AP \n\r\t";

/* Set DHCP server parameters     */
const char SetDhcpServerStr[]                   = "wlan_set_dhcp" ;
const char SetDhcpServerUsageStr[]              = "[-help],[ -t <Lease Time>],[-s <IP Start Address>],[-e <IP End Address>]\n\r\t";
const char SetDhcpServerDetailsStr[]            = "\n\r\tSet current DHCP server details:\n\r\tLease Time must be greater then 0.\n\r\tThe range must not include the AP's IP address.\n\r\tThe range must be in the same subnet as the AP.\n\r ";

/* Get DHCP server parameters     */
const char GetDhcpServerStr[]                   = "wlan_get_dhcp" ;
const char GetDhcpServerUsageStr[]              = "[-help]\n\r\t";
const char GetDhcpServerDetailsStr[]            = "Get current DHCP server details: Lease Time, IP Start Address, IP End Address \n\r\t";

/* Get FW version */
const char GetFwVerStr[]                          = "wlan_get_fw_ver";
const char getFwVerUsageStr[]                     = " [-help]\n\r";
const char *wlanGetFwVerDetailsStr                = "Get fw and phy version from the device\n\r";

/* Set LSI     */
const char SetLsiStr[]                           = "wlan_set_LSI";
const char setLsiUsageStr[]                      = " [-help] [-n <number of DTIMs [1-10]>]\n\r";
const char *wlanSetLsiDetailsStr                 = "Set Long Sleep Interval, allow STA to wake up every N DTIMs\n\r";
const char *wlanSetLsi_n_optionDetailsStr        = "\n\r\t-n\tNumber of DTIMs to wake upon\n\r";

#ifdef CC35XX
/* Sleep test*/
const char SleepTestStr[]                              = "test_sleep";
const char SleepTestUsageStr[]                         = " [-help] [-t number of seconds to sleep]\n\r";
const char *sleepTestDetailsStr                  = "Set the test sleep period\n\r";
const char *sleepTest_t_optionDetailsStr         = "\n\r\t-t\tnumber of seconds to sleep\n\r";
#endif

/* wlan start */
const char wlanStartStr[]                      = "wlan_start";
const char wlanStartUsageStr[]                 = " [-help] \n\r";
const char *wlanStartDetailsStr                = "Wlan start .\n\r";

/* wlan stop */
const char wlanStopStr[]                      = "wlan_stop";
const char wlanStopUsageStr[]                 = " [-help] [-r] <For Recovery wlan_stop> ";
const char *wlanStopDetailsStr                = "\rWlan Stop include two modes.\n\r"
                                          "Recovery and non Recovery. \n\r";

/* ble Adv Cfg */
const char bleAdvCfgStr[]                      = "ble_adv_cfg";
const char bleAdvCfgUsageStr[]                 = " [-help] [-i <instance>] "
                                           "[-l <legacy [0:extended/1:legacy]>] [-n <interval (ms)>] "
                                           "[-p <primary phy [1:1M/2:2M/3:coded]>] [-s <secondary phy [1:1M/2:2M/3:coded]>]\n\r";
const char *bleAdvCfgDetailsStr                = "Set Ble Advertise Configuration. Default Parameters: instance 0, extended, 100ms interval, 1M PHY.\n\r";

/* ble Adv Enable */
const char bleAdvEnableStr[]                   = "ble_adv_enable";
const char bleAdvEnableUsageStr[]              = " [-help] [-e <enable [0/1]>] [-i <instance>] "
                                           "[-d <duration (10ms units)>] [-m <max events]\n\r";
const char *bleAdvEnableDetailsStr             = "Ble Advertise enable/disable. Default Parameters: enable with no expiration and max events 0.\n\r";

/* ble Scan Cfg */
const char bleScanCfgStr[]                     = "ble_scan_cfg";
const char bleScanCfgUsageStr[]                = " [-help] [-i <scan interval (ms)>] [-w <scan window (ms)>] "
                                           "[-p <phy [ 1:1M/4:coded/5:mixed]>]\n\r";
const char *bleScanCfgDetailsStr               = "Set Ble Scan Configuration. Default Parameters: extended 1M PHY, scan interval of 100ms and scan window of 50ms.\n\r";

/* ble Scan Enable */
const char bleScanEnableStr[]                  = "ble_scan_enable";
const char bleScanEnableUsageStr[]             = " [-help] [-e <enable [0/1]>] [-f <filter duplicate [0/1/2]>] "
                                           "[-p <period (sec)>]\n\r";
const char *bleScanEnableDetailsStr            = "Ble Scan enable/disable. Default Parameters: enable with scan duration of 3s, scan period 0s and no filter duplicate.\n\r";

/* ble Connect */
const char bleConnectStr[]                     = "ble_connect";
const char bleConnectUsageStr[]                = " [-help] [-b <bd address>] [-t <address type [PUBLIC/RANDOM]>]";
const char bleConnectDetailsStr[]               = "Ble Connect.\n\r";

/* ble Disconnect */
const char bleDisconnectStr[]                  = "ble_disconnect";
const char bleDisconnectUsageStr[]             = " [-help] [-a <address>] [-t <address type [PUBLIC/RANDOM]>]";
const char bleDisconnectDetailsStr[]            = "Ble Disconnect. Default action when no parameters is to disconnect all peers\n\r";

/* ble Connected Peers */
const char blePeersStr[]                       = "ble_peers";
const char blePeersUsageStr[]                  = " [-help] ";
const char blePeersDetailsStr[]                 = "Ble Peers.\n\r";

/* ble Delete Bonds */
const char bleDeleteBondsStr[]                 = "ble_delete_bonds";
const char bleDeleteBondsUsageStr[]            = " [-help] ";
const char bleDeleteBondsDetailsStr[]           = "Ble delete all bonds.\n\r";

/* ble Get BD address */
const char bleGetBdAddrStr[]                   = "ble_get_bd_addr";
const char bleGetBdAddrUsageStr[]              = " [-help] [-t <address type [PUBLIC/RANDOM]>]";
const char bleGetBdAddrDetailsStr[]            = "Ble get BD address.\n\r";

/* ble Set BD address */
const char bleSetBdAddrStr[]                   = "ble_set_bd_addr";
const char bleSetBdAddrUsageStr[]              = " [-help] [-t <address type [PUBLIC/RANDOM]>]";
const char bleSetBdAddrDetailsStr[]            = "Ble set BD address.\n\r";

/* ble start */
const char bleStartStr[]                       = "ble_start";
const char bleStartUsageStr[]                  = " [-help] \n\r";
const char bleStartDetailsStr[]                 = "Ble start. \n\r";

/* ble stop */
const char bleStopStr[]                        = "ble_stop";
const char bleStopUsageStr[]                   = " [-help] \n\r";
const char bleStopDetailsStr[]                  = "Ble stop. \n\r";

/* csi enable */
char csiEnableStr[]                       = "csi_enable";
char csiEnableUsageStr[]                  = " [-help] \n\r";
char csiEnableDetailsStr[]                 = "CSI enable. \n\r";

/* csi stop */
char csiStopStr[]                       = "csi_stop";
char csiStopUsageStr[]                  = " [-help] \n\r";
char csiStopDetailsStr[]                 = "CSI stop. \n\r";

/* csi disable */
char csiDisableStr[]                       = "csi_disable";
char csiDisableUsageStr[]                  = " [-help] \n\r";
char csiDisableDetailsStr[]                 = "CSI disable. \n\r";

/* csi get results */
char csiGetResultsStr[]                       = "csi_get_results";
char csiGetResultsUsageStr[]                  = " [-help] \n\r";
char csiGetResultsDetailsStr[]                 = "CSI get results. \n\r";

/* test */
const char testStr[]                      = "test";
const char testUsageStr[]                 = " [-help] \n\r";
const char testDetailsStr[]                = "test .\n\r";

const char sendStr[]                   = "send";
const char sendUsage1Str[]             = " [-help] [-c <server ip address>] [-u] "
                                   "[-p <port number>]  "
                                   "[-nb] [-n <number of packets>] \n\r";
const char sendUsage2Str[]             = " [-help] [-s] [-u] [-p <port number>] [-b bandwidth]  "
                                   "[-V] [-nb] [-n <number of packets>] \n\r";
const char sendDetailsStr[]             = "To send ip packets over network.\n\r";
const char send_c_optionDetailsStr[]   = "\n\r\t-c\tRun in client mode and connect"
                                    " to mentioned server -\n\r"
                                        "\t  \tIP should be in '.' format for"
                                        " ipv4 and in ':' for ipv6\n\r";
const char send_s_optionDetailsStr[]   = "\n\r\t-s\tRun in server mode\n\r";
const char send_u_optionDetailsStr[]   = "\n\r\t-u\tUse UDP rather than TCP\n\r";
const char send_p_optionDetailsStr[]   = "\n\r\t-p\tPort number to send/receive data (Default for server is 5001)\n\r";
const char send_nb_optionDetailsStr[]  = "\n\r\t-nb\tCreate non-blocking socket "
                                   "rather than blocking\n\r";
const char send_n_optionDetailsStr[]   = "\n\r\t-n\tNumber of packets to transmit"
                                   " (Default is 1000)\n\r";
const char send_6_optionDetailsStr[]   = "\n\r\t-V\tUse IPv6 rather than IPv4\n\r";
const char send_7_optionDetailsStr[]   = "\n\r\t-b\tset bandwith, Mbps\n\r";
const char iperf_v_optionDetailsStr[]   = "\n\r\t-v\t verbose mode,default is not verbose\n\r";
const char iperf_t_optionDetailsStr[]   = "\n\r\t-t\t time in second to run, default is endless\n\r";


const char TestIperf[]           = "iperf";
const char recvTestIperfDetailsStr[]    = "To receive/send iperf packets over network.\n\r";
const char recvTestIperfUsage2Str[]     = " [-help] [-s] [-c <server ip address>] [-p <port number>] [-i <number>] [-t <time in sec> >99999 endless>][-B <server ip>] \n\r";
const char *recvTestIperf_s_optionDetailsStr   = send_s_optionDetailsStr;
const char *recvTestIperf_c_optionDetailsStr   = send_c_optionDetailsStr;
const char *recvTestIperf_p_optionDetailsStr   = send_p_optionDetailsStr;
const char *recvTestIperf_u_optionDetailsStr   = send_u_optionDetailsStr;
const char recvTestIperf_i_optionDetailsStr[]   = "\n\r\t-i\t set the interval in second , 1 means sec\n\r";
const char *recvTestIperf_t_optionDetailsStr   = iperf_t_optionDetailsStr;
const char *recvTestIperf_b_optionDetailsStr   = "\n\r\t-b\t set the max udp client bandwidth in Mbps , 0 means no limit\n\r";
const char *recvTestIperf_B_optionDetailsStr   = "\n\r\t-B\t binds the server to a specific local IP";


const char StopTestIperf[]           = "iperf_stop";
const char *recvStopTestIperfDetailsStr    = "stop iperf process.\n\r";
const char *recvStopTestIperfUsage2Str     = " [-help] [-n <process number>\n\r";
const char *recvStopTestIperf_n_optionDetailsStr   = "\n\r\t-n\t id of the iperf process, set to 0 in order to view available process \n\r";



const char recvStr[]                  = "recv";
const char *recvUsage1Str             = sendUsage1Str;
const char *recvUsage2Str             = sendUsage2Str;
const char recvDetailsStr[]            = "To receive ip packets over network.\n\r";
const char *recv_c_optionDetailsStr   = send_c_optionDetailsStr;
const char *recv_s_optionDetailsStr   = send_s_optionDetailsStr;
const char *recv_u_optionDetailsStr   = send_u_optionDetailsStr;
const char *recv_p_optionDetailsStr   = send_p_optionDetailsStr;
const char *recv_nb_optionDetailsStr  = send_nb_optionDetailsStr;
const char send_n_r_optionDetailsStr[] = "\n\r\t-n\tNumber of packets to receive "
                                   "(Default is 1000)\n\r";
const char *recv_6_optionDetailsStr      = send_6_optionDetailsStr;
const char recv_Note_optionDetailsStr[] = "\n\r\tNote:\tUDP triggers a timeout "
                                    "which waits 60 seconds unless all the"
                                    " packets were received\n\r";

const char showStr[]                      = "socket_show";
const char showUsageStr[]                 = " [-help] \n\r";
const char *showDetailsStr                = "show available running process.\n\r";

const char killStr[]                      = "kill";
const char killUsageStr[]                 = "[-help] [-i <id>] \n\r";
const char *killDetailsStr                = "kill available running process by id .\n\r";

const char SntpConfigNTPServers[]             = "sntp_config_servers";
const char SntpConfigNTPServersUsageStr[]     = "[-help] [-s <server ip>] \n\r";
const char SntpConfigNTPServers_s_optionDetailsStr[] = "\n\r\t-s\t server ip, can define up to 3 servers ";
const char SntpConfigNTPServers_s_optionDetailsStrExpand[] = "\n\r\t-s\t for example: sntp_config_servers  -s 10.167.188.21 -s 219.239.35.0";
const char SntpUpdateDateTime[]               = "sntp_update";
const char SntpUpdateTimeUsageStr[]              = "[-help] \n\r";

const char SetDateTime[]                      = "set_date_time";
const char SetDateTimeUsageStr[]              = "[-help] [-t <yyyy-mm-ddThh:mm:ss>] \n\r";
const char SetDateTime_t_optionDetailsStr[]    = "\n\r\t-t\t set current date and time ";
const char GetDateTime[]                      = "get_date_time";
const char GetDateTimeUsageStr[]              = "[-help] \n\r";
const char GetDateTime_t_optionDetailsStr[]    = "\n\r\t-t\t Get current date and time ";
const char getRoleIdUsageStr[]             =
"[-help] [-r <choose AP or STA role>][\"ap\" \"sta\"]>]\n\r";

#ifdef CC35XX
const char wlanCreateVendorIEStr[]         = "vend_IE_create_lst";
const char wlanDeleteVendorIEStr[]         = "vend_IE_delete_lst";
const char wlanAddVendorIEStr[]            = "vend_IE_add";
const char wlanRemoveVendorIEStr[]         = "vend_IE_remove";
const char wlan_AddVendorIE_UsageStr_first[]   =   " [-help]";
const char wlan_AddVendorIE_UsageStr_second[]   =     "Adds examples vendor IE";
const char wlan_DeleteVendorIE_UsageStr_first[]   =   " [-help]";
const char wlan_DeleteVendorIE_UsageStr_second[]   =  "Deletes the examples vendor IE";


const char wlanConfigPeerAgingStr[]         = "set_peer_aging";
const char wlan_ConfigPeerAging_UsageStr_first[]   =   " [-help]";
const char wlan_ConfigPeerAging_UsageStr_second[]  =   "[-t <Peer Aging timeout,Mili, 0 means to stop]>]";
const char wlan_ConfigPeerAging_DetailsStr[]    = "configure timeout for getting Peer aging async event .\n\r";



/* Role up Device */
const char wlanRoleUpP2PStr[]          = "p2p_role_up";
const char wlan_role_up_p2p_UsageStr_first[]   =   " [-help]";
const char wlan_role_up_p2p_UsageStr_second[]  =   "[-r <AP Regulatory Domain [\"US\" \"JP\" \"00\"]>]\n\r";
const char wlan_role_up_p2p_UsageStr_third[]   =   "[-c <oper channel>]"
                                                "[-o <oper reg class>]"
                                                "[-s <listen channel>]"
                                                "[-m <listen reg class>]"
                                                "[-i <Go intent]>]\n\r";
const char wlan_role_up_p2p_i_optionDetailsStr[] = "\n\r\t-i\tGo intent : 0 - 15 \n\r";

const char *wlan_role_up_p2p_DetailsStr    = "P2P Role up .\n\r";

const char wlanRoleDownP2PStr[]             = "p2p_role_down";
const char *wlan_role_down_p2p_DetailsStr   = "P2P Role Down.\n\r";

const char wlanP2PFindStr[]             = "p2p_find";
const char printWlanP2PFindUsageStr[]   = "";


const char wlanP2PConnectStr[]          = "p2p_connect";//<peer_mac> <wps_method> <pin code>

const char wlanP2PStopFindStr[]         =  "p2p_stop_find";

const char wlanP2PGrpRemoveStr[]        =  "p2p_group_remove";

const char wlanP2PSetchannelStr[]       =  "p2p_set_channel";//<oper channel> <oper reg> <listen channel> <listen reg> <go intent>\n"

const char wlanP2PGetchannelStr[]       =  "p2p_get_channel";

const char wlanP2PListenStr[]       =  "p2p_listen";

const char wlanP2PCancelStr[]       =  "p2p_cancel";


const char *wlan_p2p_connect_DetailsStr    = "P2P connect .\n\r";
const char wlan_p2p_connect_UsageStr_first[]   =   " [-help]\n\r";
const char wlan_p2p_connect_UsageStr_second[]  =   "[-m <peer_macAdress>]\n\r"
                                             "[-w <wps_method [0 1 2]>] 0=PBC 1=PIN DISPLAY 2= PIN keypad\n\r"
                                             "[-p <pin_code>]\n\r";

const char *wlan_p2p_find_stop_DetailsStr =   "P2P stop find .\n\r";


const char *wlan_p2p_group_remove_DetailsStr =   "P2P remove group .\n\r";

const char  wlan_role_up_group_remove_UsageStr_third[]  =   "[-c <oper channel>]"
                                                "[-o <oper reg class>]"
                                                "[-s <listen channel>]"
                                                "[-m <listen reg class>]"
                                                "[-i <Go intent]>]\n\r";

const char *wlan_role_up_p2pSetChannel_DetailsStr    = "P2P Set channel .\n\r";
const char *wlan_role_up_p2pGetChannel_DetailsStr    = "P2P Get channel .\n\r";
const char *wlan_role_up_p2pListen_DetailsStr        = "P2P listen .\n\r";
const char *wlan_role_up_p2pCancel_DetailsStr    = "P2P cancel .\n\r";



/* Start AP WPS */
const char startApWpsStr[]                      = "start_ap_wps";
const char startApWpsUsageStr[]                 = " [-help] [-w <wps_method [0 1]> 0=PBC 1=PIN] [-p <pin_code>] \n\r";
const char startApWpsDetailsStr[]               = "Start an AP WPS session \n\r";
const char startApWps_w_optionDetailsStr[]      = "\n\r\t-w\tWPS method (0=PBC, 1=PIN) \n\r";
const char startApWps_p_optionDetailsStr[]      = "\n\r\t-p\t8-digit PIN code, required for WPS PIN method \n\r";


const char SetWsocPrimaryStr[]        = "set_wsoc_primary";
const char SetWsocPrimaryUsageStr[]   = " [-help] [-i <SlotNumber>] \n\r";
const char *SetWsocPrimaryDetailsStr  = "Set Wsoc Primary. 1 - Slot1(Default) , 2 - Slot2 \n\r";


const char pingStartStr[]                 = "ping";
const char pingStartUsageStr[]            = " [-help] [-c <count>] [-i <interval>] [-s <data_size>] [-I <source_address>]\n\r";
const char pingStartDetailsStr[]          = "Start a ping session. Target IP must be the first argument.\n\r\tIn order to stop run \"ping stop\" command with the same ping session id.\n\r";
const char pingStart_c_optionDetailsStr[] = "\n\r\t-c\tStop after sending count ECHO_REQUEST packets.\n\r\t  \t"
                                            "The default is to send 10 ECHO_REQUEST packets.\n\r\t  \t"
                                            "Use 0 for infinite count.\n\r";
const char pingStart_i_optionDetailsStr[] = "\n\r\t-i\tWait interval seconds between sending each packet.\n\r\t  \t"
                                            "The default is to wait for one second between each packet.\n\r\t  \t"
                                            "Range 100 - 120,000 ms.\n\r";
const char pingStart_s_optionDetailsStr[] = "\n\r\t-s\tSpecifies the number of data bytes to be sent.\n\r\t  \t"
                                            "The default is 56, which translates into 64 ICMP data bytes\n\r\t  \t"
                                            "when combined with the 8 bytes of ICMP header data.\n\r\t  \t"
                                            "Max is 1452 bytes.\n\r";
const char pingStart_I_optionDetailsStr[] = "\n\r\t-I\tSet source address to specified interface address.\n\r\t  \t"
                                            "Argument must be in numeric IP address format.\n\r";


const char pingStopStr[]                 = "ping_stop";
const char pingStopUsageStr[]            = " [-help] [-i <session_id>]\n\r";
const char pingStopDetailsStr[]          = "Stop a ping session.\n\r\tRun this command in order to stop a specific ping session\n\r\t";
const char pingStop_i_optionDetailsStr[] = "\n\r\t-i\tSession ID to stop. 0 to print all active sessions\n\r";


const char wlanSetRegDomEntryStr[]       = "set_cstm_reg_domain";
const char wlanSetRegDomEntryUsageStr[]  = " [-help] [-i <index>] [-b <band>] [-p <power>] [-d <dfs>] [-min <min_channel>] [-max <max_channel>]"
                                           " [-n <num_of_channels>] [-bm <channels_bitmap>]\n\r";
const char wlanSetRegDomEntryDetailsStr[] = "Set a regulatory domain rule to the WLAN regulatory domain DB.\n\r\t"
                                            "A rule is a set of adjacent channels that can be identified by a minimum channel,\n\r\t"
                                            "maximum channel and leaps of 20 MHz between them.\n\r\t"
                                            "Note! Only 7 rules can be applied.\n\r";
const char wlanSetRegDomEntry_i_optionDetailsStr[] = "\n\r\t-i\tRule index. Range is [0,6]. Mandatory\n\r";
const char wlanSetRegDomEntry_b_optionDetailsStr[] = "\n\r\t-b\tRule band. 0 for 2.4 GHz, 1 for 5 GHz. Mandatory\n\r";
const char wlanSetRegDomEntry_p_optionDetailsStr[] = "\n\r\t-p\tMax TX power. Up to 21 dBm. Mandatory.\n\r";
const char wlanSetRegDomEntry_d_optionDetailsStr[] = "\n\r\t-d\tDFS. Are the specified channels are DFS channels. Non-DFS is default.\n\r";
const char wlanSetRegDomEntry_n_optionDetailsStr[] = "\n\r\t-n\tNumber of channels in rule. Max is 32. Mandatory\n\r";
const char wlanSetRegDomEntry_bm_optionDetailsStr[] = "\n\r\t-bm\tBitmap (hex format) representing the channels in the rule. \n\r\t\tStarting from the LSB, "
                                                      "each bit represent a channel between the specified\n\r\t\tmin and max channels, in leaps "
                                                      "of 20 MHz bandwith channels (according to the band).\n\rMandatory\n\r";
const char wlanSetRegDomEntry_r_optionDetailsStr[] = "\n\r\t-r\tReset rule entry at given rule index\n\r";

const char wlanGetRegDomEntryStr[]       = "get_cstm_reg_domain";
const char wlanGetRegDomEntryUsageStr[]  = " [-help] [-i <index>]";
const char wlanGetRegDomEntryDetailsStr[] = "Get a regulatory domain rule from the WLAN regulatory domain DB.\n\r\t"
                                            "Returns a structure filled with the rule`s content.";
const char wlanGetRegDomEntry_i_optionDetailsStr[] = "\n\r\t-i\tRule index to retrieve. Range is [0,6]\n\r";

#endif

/* Connection Policy Set */
const char wlanSetConnPolicyStr[]         = "wlan_set_con_policy";
const char wlanSetConnPolicyUsageStr[]    = " [-help] [-a <Auto connect>] "
                                            " [-f <Fast Connect>] [-p <fast persistant>]\n\r";
const char *wlanSetConnPolicyDetailsStr   = "Connection Policy Set.\n\r";
const char *wlanSetConnPolicy_a_optionDetailsStr    = "\n\r\t-a \t1 - Auto connect enable \n\r";
const char *wlanSetConnPolicy_f_optionDetailsStr    = "\n\r\t-f \t1 - Fast connect enable \n\r";
const char *wlanSetConnPolicy_p_optionDetailsStr    = "\n\r\t-p \t1 - Fast connect persistant enable \n\r";

/* Connection Policy Get */
const char wlanGetConnPolicyStr[]         = "wlan_get_con_policy";
const char wlanGetConnPolicyUsageStr[]    = " [-help]\n\r";
const char *wlanGetConnPolicyDetailsStr   = "Connection Policy Get.\n\r";

/* Add Profile */
const char wlanAddProfileStr[]            = "wlan_add_profile";
const char wlanAddProfileUsageStr[]       = " [-help]  [-s <\"ssid name\">] "
                                            " [-t <security type>] [-p <\"password\">] "
                                            " [-ent <enterprise user name>] "
                                            " [-pr <priority>] [-h <hidden AP>]\n\r";
const char *wlanAddProfileDetailsStr      = "Add Profile.\n\r";
const char *wlanAddProfile_s_optionDetailsStr  = "\n\r\t-s\tSSID\n\r";
const char *wlanAddProfile_t_optionDetailsStr  = "\n\r\t-t\tType of security "
                                                 "(security type = "
                                                 "[OPEN, WPA, WPA2, WPA2_PLUS, WPA3, WPA2/WPA3])\n\r";
const char *wlanAddProfile_p_optionDetailsStr  = "\n\r\t-p\tPassword in ascii characters\n\r";                                                
const char *wlanAddProfile_pr_optionDetailsStr = "\n\r\t-pr \t 0 - 15 priority\n\r";
const char *wlanAddProfile_h_optionDetailsStr  = "\n\r\t-h\tScan SSID: Hidden - 1, Wildcard - 0\n\r";

/* Get Profile */
const char wlanGetProfileStr[]            = "wlan_get_profile";
const char wlanGetProfileUsageStr[]       = " [-help] [-i <ProfileIndex>]\n\r";
const char *wlanGetProfileDetailsStr      = "Get Profile.\n\r";
const char *wlanGetProfile_i_optionDetailsStr = "\n\r\t-i \tProfile index\n\r";


/* Set Scan Dwell Time */
const char wlanSetScanDwellTimeStr[]            = "wlan_set_dwell_time";
const char wlanSetScanDwellTimeUsageStr[]       = " [-help] [-min_dwell_active <min dwell msecs>] [-max_dwell_active <max dwell msec>]"
                                                  " [-min_dwell_passive <min dwell msec>] [-max_dwell_passive <max dwell msec>] "
                                                  " [-dwell_dfs_passive <dwell msec>]\n\r";
const char *wlanSetScanDwellTimeDetailsStr      = "Set Dwell Time.\n\r";

/* Delete Profile */
const char wlanDelProfileStr[]            = "wlan_del_profile";
const char wlanDelProfileUsageStr[]       = " [-help] [-i <ProfileIndex>]\n\r";
const char *wlanDelProfileDetailsStr      = "Delete Profile.\n\r";
const char *wlanDelProfile_i_optionDetailsStr = "\n\r\t-i \tProfile index\n\r";

/* Wlan Profile Connect */
const char wlanProfileConnectStr[]        = "wlan_profile_connect";
const char wlanProfileConnectUsageStr[]   = " [-help] [-i <ProfileIndex>]\n\r";
const char *wlanProfileConnectDetailsStr  = "Wlan Profile Connect.\n\r";
const char *wlanProfileConnect_i_optionDetailsStr = "\n\r\t-i \tProfile index\n\r";

/* ------  */
const char emptyDeviceStr[]          = "";



const  char a_optionStr[]            = "-a";
const  char b_optionStr[]            = "-b";
const  char B_optionStr[]            = "-B";
const  char c_optionStr[]            = "-c";
const  char f_optionStr[]            = "-f";
const  char g_optionStr[]            = "-g";
const  char r_optionStr[]            = "-r";
const  char h_optionStr[]            = "-h";
const  char i_optionStr[]            = "-i";
const  char l_optionStr[]            = "-l";
const  char n_optionStr[]            = "-n";
const  char o_optionStr[]            = "-o";
const  char p_optionStr[]            = "-p";
const  char s_optionStr[]            = "-s";
const  char t_optionStr[]            = "-t";
const  char w_optionStr[]            = "-w";
const  char I_optionStr[]            = "-I";
const  char st_optionStr[]           = "-st";
const  char tx_optionStr[]           = "-tx";
const  char tone_optionStr[]         = "-tone";
const  char cca_optionStr[]          = "-cca";
const  char u_optionStr[]            = "-u";
const  char m_optionStr[]            = "-m";
const  char x_optionStr[]            = "-x";
const  char v_optionStr[]            = "-v";
const  char d_optionStr[]            = "-d";
const  char e_optionStr[]            = "-e";
const  char ch_optionStr[]           = "-ch";
const  char txpow_optionStr[]        = "-txp";
const  char fs_optionStr[]           = "-fs";
const  char nb_optionStr[]           = "-nb";
const  char lh_optionStr[]           = "-lh";
const  char dp_optionStr[]           = "-dp";
const  char V_optionStr[]            = "-V";
const  char ip_optionStr[]           = "-ip";
const  char mac_optionStr[]          = "-mac";
const  char priority_optionStr[]     = "-pr";
const  char dns_optionStr[]          = "-dns";
const  char gw_optionStr[]           = "-gw";
const  char ent_optionStr[]          = "-ent";
const  char help_optionStr[]         = "-help";
const  char ttl_optionStr[]          = "-ttl";
const  char so_optionStr[]           = "-so";
const  char ext_optionStr[]          = "EXT";
const  char bitmap_optionStr[]       = "-bm";


const  char min_dwell_active_optionStr[]   = "-min_dwell_active";
const  char max_dwell_active_optionStr[]   = "-max_dwell_active";
const  char min_dwell_passive_optionStr[]  = "-min_dwell_passive";
const  char max_dwell_passive_optionStr[]  = "-max_dwell_passive";
const  char dfs_dwell_passive[]            = "-dwell_dfs_passive";



const  char WPA_str[]                = "WPA";
const  char WPA2_str[]               = "WPA2";
const  char WPS_str[]                = "WPS";
const  char OPEN_str[]               = "OPEN";
const  char WPAWPA2_str[]            = "WPA/WPA2";
const  char WPA2_PLUS_str[]          = "WPA2_PLUS";
const  char WPA3_str[]               = "WPA3";
const  char WPA2WPA3_str[]           = "WPA2/WPA3";
const  char DISABLE_str[]            = "Disable";
const  char CAPABLE_str[]            = "Capable";
const  char REQUIRED_str[]           = "Required";
const  char YES_str[]                = "YES";
const  char NO_str[]                 = "NO";
const  char CONNECTED_str[]          = "CON";
const  char PROMISCUOUS_str[]        = "TRANS";
const  char space_str[]              = " ";
const  char SOURCE_MAC_str[]         = "S_MAC";
const  char DESTINATION_MAC_str[]    = "D_MAC";
const  char BSSID_str[]              = "BSSID";
const  char S_IP_str[]               = "S_IP";
const  char D_IP_str[]               = "D_IP";
const  char FRAME_TYPE_str[]         = "FRAME_TYPE";
const  char FRAME_SUBTYPE_str[]      = "FRAME_SUBTYPE";
const  char PATTERN_str[]            = "PATTERN";
const  char PUBLIC_str[]             = "PUBLIC";
const  char RANDOM_str[]             = "RANDOM";
const  char TLS_str[]                = "TLS";
const  char TTLS_MSCHAP_str[]        = "TTLS_MSCHAP";
const  char PEAP0_MSCHAP_str[]       = "PEAP0_MSCHAP";
const  char PEAP0_GTC_str[]          = "PEAP0_GTC";
const  char PEAP2_MSCHAP_str[]       = "PEAP2_MSCHAP";
const  char PEAP2_GTC_str[]          = "PEAP2_GTC";

const char *MangmentFrames_str[]    =
{"ASSOCIATION REQ", "ASSOCIATION RESPONSE", "REASSOCIATION REQ" ,
"REASSOCIATION RESPONSE", "PROBE REQ", "PROBE RESPONSE", "BEACON", "ATIM",
 "DISASSOCIATION" , "AUTHENTICATION", "DEAUTHENTICATION","ACTION CTRL FRAMES"};

const char *CtrlFrames_str[]        =
{"BLOCK ACK REQ", "BLOCK ACK", "PS POLL", "RTS", "CTS", "ACK", "CF END",
"CF END ACK" };

const char *DataFrames_str[]        =
{"DATA", "DATA CF ACK", "DATA CF POLL", "DATA CF ACK POLL", "NO DATA FRAME",
 "CF ACK", "CF POLL", "CF ACK POLL", "QOS DATA", "QOS DATA CF ACK",
 "QOS DATA CF POLL", "QOS DATA CF ACK POLL","QOS NO DATA FRAME" ,
 "QOS CF ACK", "QOS CF POLL", "QOS CF ACK POLL"};


const char fsError[]                =
"File system error, please refer  \"FS ERRORS CODES\" section in errors.h";
const char socketError[]            =
"BSD Socket error, please refer \"BSD SOCKET ERRORS CODES\" "
"section in errors.h";
const char netappError[]            =
"Netapp error, please refer \"NETAPP ERRORS CODES\" section in errors.h";
const char cmdError[]               =
"Invalid option/command.";
const char nwError[]                =
"Network error";
const char radioToolError[]         =
"Radiotool error";

// Indigo
const char loadCertificate[]           = "load_cert";
const char loadCertificateUsageStr[]   = "[-help] "
                                         "[-t <file type>] "
                                         "[-s <size>]\n\r ";
const char loadCertificateDetailsStr[] = "Load certificate: fileType 0 - client_cert, 1 - ca_cert, 2 - private_key_cert\n\r";

void printBorder(char ch, int n)
{
    int        i = 0;
    for(i=0; i<n; i++)    putch(ch);
}





