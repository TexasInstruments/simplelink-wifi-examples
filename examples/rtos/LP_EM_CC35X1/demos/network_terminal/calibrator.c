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
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "calibrator.h"
#include "uart_term.h"
#include "cmd_parser.h"
#include "wlan_if.h"
#include "errors.h"

#define MAC_ADDRESS_STR_LEN             (18)

#define RX_PARAMS_INVALID_AID           (0xFFFF)
#define RX_PARAMS_INVALID_PREAMBLE_TYPE (0xFF)
#define RX_PARAMS_INVALID_RATE          (0xFF)

#define ROLE_TRANSCEIVER                (16)

/*************************************************
 *              General Utilities                *
 *************************************************/

/*  Note: This function returns a pointer to a substring of the original string.
 *  If the given string was allocated dynamically, the caller must not overwrite
 *  that pointer with the returned value, since the original pointer must be
 *  deallocated using the same allocator with which it was allocated.  The return
 *  value must NOT be deallocated using free() etc.
 */
char *trimLeadingWhitespace(char *str)
{
    char *end;


    while(isspace((unsigned char)*str)) // Trim leading space
    {
        str++;
    }

    if(*str == 0)  // All spaces?
    {
        return str;
    }

    // Trim trailing space
    end = str + strlen(str) - 1;
    while((end > str) && isspace((unsigned char)*end))
    {
        end--;
    }

    // Write new null terminator character
    end[1] = '\0';

    return str;
}


/*************************************************
 *              Calibrator Strings               *
 *************************************************/

char calibratorSetPowerModeActionStr[] =            "power_mode";
char calibratorChannelTuneActionStr[] =             "tune_channel";
char calibratorStartTxActionStr[] =                 "start_tx";
char calibratorStartTxToneActionStr[] =             "start_tx_tone";
char calibratorStopTxActionStr[] =                  "stop_tx";
char calibratorStartRxActionStr[] =                 "start_rx";
char calibratorStopRxActionStr[] =                  "stop_rx";
char calibratorGetRxStatsActionStr[] =              "get_rx_stats";
char calibratorSetTxParamsActionStr[] =             "set_tx";
char calibratorGetTxParamsActionStr[] =             "get_tx_params";
char calibratorSetTbTxParamsActionStr[] =           "set_tb_tx";
char calibratorRateOverrideActionStr[] =            "rate_override";
char calibratorGetBeaconRssiActionStr[] =           "beacon_rssi";
char calibratorSetGiLtfActionStr[] =                "gi_ltf";
char calibratorSetUplinkMuActionStr[] =             "uplink_mu";
char calibratorSetOperationModeControlActionStr[] = "oper_mode";
char calibratorSetMcsRateActionStr[] =              "mcs_rate";
char calibratorSetUplinkMuDataActionStr[] =         "uplink_mu_data";
char calibratorSetPsmActionStr[] =                  "psm";
char calibratorSetUplinkPowerHeaderActionStr[] =    "power_head";
char calibratorSetTransmitOMIActionStr[] =          "trans_omi";
const char calibratorSetBAEnableActionStr[] =       "ba_enable";
const char calibratorLinkAdaptActionStr[] =         "la";
const char calibratorSetManualCalibActionStr[] =    "set_manual_calib";



char *calibratorPowerOptions[] = {"off", "on", "chip_awake"};

/* Calibrator set tx options */
char *calibratorSetTxDefaultOption =        "-default";
char *calibratorSetTxPreambleTypeOption =   "-preamble_type";
char *calibratorSetTxPhyRateOption =        "-phy_rate";
char *calibratorSetTxTxPowerOption =        "-tx_power";
char *calibratorSetTxGiLtfTypeOption =      "-gi_ltf_type";
char *calibratorSetTxDCMOption =            "-dcm";
char *calibratorSetTxLengthOption =         "-length";
char *calibratorSetTxDelayOption =          "-delay";
char *calibratorSetTxPktModeOption =        "-pkt_mode";
char *calibratorSetTxNumPktsOption =        "-num_pkts";
char *calibratorSetTxDataModeOption =       "-data_mode";
char *calibratorSetTxDataConstValOption =   "-data_const_val";
char *calibratorSetTxCCAOption =            "-cca";
char *calibratorSetTxBssColorOption =       "-bss_color";
char *calibratorSetTxSuErBwOption =         "-su_er_bw";
char *calibratorSetTxPartialAidOption =     "-partial_aid";
char *calibratorSetTxSrcAddrOption =        "-src_addr";
char *calibratorSetTxDstAddrOption =        "-dst_addr";
char *calibratorSetTxNominalPktExtOption =  "-nominal_pkt_ext";
char *calibratorSetTxFeedStatusOption =     "-feed_status";
char *calibratorSetTxAidOption =            "-aid";
char *calibratorSetTxGroupIdOption =        "-group_id";
char *calibratorSetTxMimoLtfMode =          "-mimo_ltf_mode";
char *calibratorSetTxHeLtfNum =             "-he_ltf_num";
char *calibratorSetTxDisamb =               "-disamb";
char *calibratorSetTxPreFecPaddingFactor =  "-pre_fec_padding_factor";
char *calibratorSetTxCommonInfoLen =        "-common_info_len";
char *calibratorSetTxRuAlloc =              "-ru_alloc";
char *calibratorSetTxUlBw =                 "-ul_bw";
char *calibratorSetTxStartsStsNum =         "-starts_sts_num";
char *calibratorSetTxTbAuto =               "-tb_auto";

/* calibrator start rx options */
char *calibratorStartRxSourceMacOption =    "-source_mac";
char *calibratorStartRxAckEnableOption =    "-ack_enable";
char *calibratorStartRxAidOption =          "-aid";
char *calibratorStartRxRateOption =         "-rate";
char *calibratorStartRxPreambleTypeOption = "-preamble_type";

/* Calibrator usage strings */
char calibratorStr[]                    = "calibrator";
char calibratorUsageStr[]               = "[-help] [ACTIONS] \n\r";
char *calibratorFirstDetailsStr         = "Calibrator - a tool for testing.\r\n"
                                          "\tSupported Calibrator actions:\r\n"
                                          "\t\tpower_mode\r\n"
                                          "\t\ttune_channel\r\n"
                                          "\t\tstart_tx\r\n"
                                          "\t\tstart_tx_tone\r\n"
                                          "\t\tstop_tx\r\n"
                                          "\t\tstart_rx\r\n"
                                          "\t\tstop_rx\r\n"
                                          "\t\tget_rx_stats\r\n"
                                          "\t\tset_tx_params\r\n"
                                          "\t\tget_tx_params\r\n"
                                          "\t\tset_tb_tx\r\n"
                                          "\t\trate_override\r\n";
char *calibratorSecondDetailsStr        = "\t\tbeacon_rssi\r\n"
                                          "\t\tgi_ltf\r\n"
                                          "\t\tuplink_mu\r\n"
                                          "\t\toper_mode\r\n"
                                          "\t\tmcs_rate\r\n"
                                          "\t\tuplink_mu_data\r\n"
                                          "\t\tpsm\r\n"
                                          "\t\tpower_head\r\n"
                                          "\t\ttrans_omi\r\n"
                                          "\t\tba_enable\r\n"
                                          "\t\tla\r\n"
                                          "\t\tset_manual_calib\r\n"
                                          "\tUse -help with action to get help for a specific action\r\n";

char *calibratorSetPowerModeUsageStr = " [-help] <power mode>\r\n";
char *calibratorSetPowerModeDetailsStr = "Set PLT power mode.\r\n";
char *calibratorSetPower_mode_optionDetailsStr = "\r\n\tpower mode\trequested PLT power mode (on, off or chip_awake)\r\n";

char *calibratorTuneChannelUsageStr = " [-help] <channel> <band> <bandwidth>\r\n";
char *calibratorTuneChannelDetailsStr = "Set channel, band and bandwidth for PLT.\r\n";
char *calibratorTuneChannel_channel_optionDetailsStr = "\r\n\tchannel\tRequsted channel.\r\n";
char *calibratorTuneChannel_band_optionDetailsStr = "\r\n\tband\tRequsted band.\r\n";
char *calibratorTuneChannel_bandwidth_optionDetailsStr = "\r\n\tbandwidth\tRequsted bandwidth.\r\n";

char *calibratorStartTxUsageStr = " [-help]\r\n";
char *calibratorStartTxDetailsStr = "Start Calibrator TX.\r\n";

char *calibratorStartTxToneFirstUsageStr = " [-help] <mode> <offset MHz> <output_power dBm>\r\n";
char *calibratorStartTxToneSecondUsageStr = "\t <mode (0=silence (n/a), 1=carrier_feedthrough (n/a), 2=single_tone>\r\n";
char *calibratorStartTxToneThirdUsageStr = "\t <offset MHz (-10 to +10, step 0.25)>\r\n";
char *calibratorStartTxToneFourthUsageStr = "\t <output_power dBm (0 or 1)>\r\n";
char *calibratorStartTxToneDetailsStr = "Start Calibrator TX TONE.\r\n";

char *calibratorStopTxUsageStr = " [-help]\r\n";
char *calibratorStopTxDetailsStr = "Stop Calibrator TX.\r\n";

char *calibratorSetTxFirstUsageStr = " [-help] [-default 0] [-preamble_type <preamble>] [-phy_rate <rate>]"
                                     " [-tx_power <power>] [-gi_ltf_type <type>] [-dcm <enable>]";
char *calibratorSetTxSecondUsageStr = " [-length range <start length> <end length>] [-length const packet <length>]"
                                      " [-delay <delay>] [-pkt_mode <mode>] [-num_pkts <num>] [-data_mode <mode>]";
char *calibratorSetTxThirdUsageStr = " [-data_const_val <value>] [-cca <enable>] [-bss_color <value>] [-su_er_bw <value>]"
                                      " [-partial_aid <aid>] [-src_addr <address>] [-dst_addr <address>]"
                                      " [-nominal_pkt_ext <extension>] [-feed_status <enable>] [-aid <aid>]"
                                      " [-group_aid <aid>]\r\n";
char *calibratorSetTxDetailsStr = "Set TX transmissions for PLT\r\n"
                                  "\tFor default values use with -default 0\r\n";
char *calibratorSetTx_preamble_optionFirstDetailsStr = "\r\n\t-preamble_type\tvalid range: 0-10 \r\n"
                                                       "\t<Preamble Types>\r\n"
                                                       "\t\t0  - 11b short preamble\r\n"
                                                       "\t\t1  - 11b long preamble\r\n"
                                                       "\t\t2  - 11a/g (legacy OFDM)\r\n"
                                                       "\t\t3  - 11n mixed mode\r\n"
                                                       "\t\t4  - 11n GF (Not Supported)\r\n";
char *calibratorSetTx_preamble_optionSecondDetailsStr = "\t\t5  - 11ax SU\r\n"
                                                        "\t\t6  - 11ax MU (Not Supported)\r\n"
                                                        "\t\t7  - 11ax SU ER\r\n"
                                                        "\t\t8  - 11ax TB (non NDP FB)\r\n"
                                                        "\t\t9  - 11ax TB NDP FB\r\n"
                                                        "\t\t10 - 11ac VHT\r\n";
char *calibratorSetTx_phyRate_optionFirstDetailsStr = "\r\n\t-phy_rate\r\n"
                                                      "\t<Phy Rate>\tvalid range: 1-20\r\n"
                                                      "\t\t1  -  RATE 1MBPS\r\n"
                                                      "\t\t2  -  RATE 2MBPS\r\n"
                                                      "\t\t3  -  RATE 5_5MBPS\r\n"
                                                      "\t\t4  -  RATE 11MBPS\r\n"
                                                      "\t\t5  -  RATE 6MBPS\r\n"
                                                      "\t\t6  -  RATE 9MBPS\r\n";
char *calibratorSetTx_phyRate_optionSecondDetailsStr = "\t\t7  -  RATE 12MBPS\r\n"
                                                       "\t\t8  -  RATE 18MBPS\r\n"
                                                       "\t\t9  -  RATE 24MBPS\r\n"
                                                       "\t\t10 -  RATE 36MBPS\r\n"
                                                       "\t\t11 -  RATE 48MBPS\r\n"
                                                       "\t\t12 -  RATE 54MBPS\r\n"
                                                       "\t\t13 -  RATE 6.5MBPS (MCS0)\r\n"
                                                       "\t\t14 -  RATE 13 MBPS (MCS1)\r\n";
char *calibratorSetTx_phyRate_optionThirdDetailsStr = "\t\t15 -  RATE 19.5 MBPS (MCS2)\r\n"
                                                      "\t\t16 -  RATE 26MBPS (MCS3)\r\n"
                                                      "\t\t17 -  RATE 39MBPS (MCS4)\r\n"
                                                      "\t\t18 -  RATE 52MBPS (MCS5)\r\n"
                                                      "\t\t19 -  RATE 58.5MBPS (MCS6)\r\n"
                                                      "\t\t20 -  RATE 65MBPS (MCS7)\r\n";
char *calibratorSetTx_txPower_optionDetailsStr = "\r\n\t-tx_power\r\n"
                                                 "\t<Tx Power>\tvalid range: 0-30 \n\r \t\tPower level: -10dBm to 20dBm with 1dB resolution.\r\n"
                                                 "\t\t0: -10dBm\r\n"
                                                 "\t\t1: -9dBm\r\n"
                                                 "\t\t2: -8dBm\r\n"
                                                 "\t\t...\r\n"
                                                 "\t\t30: +20dBm\r\n";
char *calibratorSetTx_giLtfType_optionDetailsStr = "\r\n\t-gi_ltf_type\r\n"
                                                   "\t<GI LTF Type>\tvalid range: 0-5\r\n"
                                                   "\t\t0 - 1xLTF+1.6 us GI\r\n"
                                                   "\t\t1 - 2xLTF+1.6 us GI\r\n"
                                                   "\t\t2 - 4xLTF+3.2 us GI\r\n"
                                                   "\t\t3 - 2xLTF+0.8 us GI\r\n"
                                                   "\t\t4 - 1xLTF+0.8 us GI\r\n"
                                                   "\t\t5 - 4xLTF+0.8 us GI\r\n";
char *calibratorSetTx_dcm_optionDetailsStr = "\r\n\t-dcm\r\n"
                                             "\t<DCM>\tvalid range: 0-1\r\n"
                                             "\t\tBoolean indicating if Dual carrier modulation is used\r\n"
                                             "\t\tNote: DCM is only applied to MCS0, MCS1, MCS3 and MCS4\r\n"
                                             "\t\t0 - Dual carrier modulation isn't used\r\n"
                                             "\t\t1 - Dual carrier modulation is used\r\n";
char *calibratorSetTx_lengthRange_optionDetailsStr = "\r\n\t-length range <start length> <end_length>\r\n"
                                                     "\t<Packet length>\tvalid range: 100-3500\r\n";
char *calibratorSetTx_lengthConst_optionDetailsStr = "\r\n\t-length const packet <length>\r\n"
                                                     "\t\tNon MCS rate: Number of data bytes (except mac 80211 header) range: 0-3500\r\n"
                                                     "\t\tMCS rate: Number of data bytes (except mac 80211 header) range: 0-16000\r\n";
char *calibratorSetTx_delay_optionDetailsStr = "\r\n\t-delay\r\n"
                                               "\t<Delay>\tvalid range: 50-1000000 \r\n\t\tDelay between packets [µs] range: 50 - 1000000\r\n";
char *calibratorSetTx_packetMode_optionFirstDetailsStr = "\r\n\t-pkt_mode\r\n"
                                                         "\t<Packet mode>\tvalid range: 0-2\r\n"
                                                         "\t\t0 - Continuous mode (Will send packets till \"tx_stop\" will be performed\r\n";
char *calibratorSetTx_packetMode_optionSecondDetailsStr = "\t\t1 - Single packet (Will send only single packet)\r\n"
                                                          "\t\t2 - Multi packets (Need to update next variable:"
                                                          " \"<Number of packets>\" with number of packets. Range: 1-10000)\r\n";
char *calibratorSetTx_numPackets_optionDetailsStr = "\r\n\t-num_pkts\r\n"
                                                    "\t<Number of packets>\tvalid range: 1-10000 (In case where Multi mode is chosen range: 1-10000)\r\n";
char *calibratorSetTx_dataMode_optionDetailsStr = "\r\n\t-data_mode\r\n"
                                                  "\t<Data mode>\tvalid range: 0-2\r\n"
                                                  "\t\t0 - constant value\r\n"
                                                  "\t\t1 - increment\r\n"
                                                  "\t\t2 - random value\r\n";
char *calibratorSetTx_dataConstVal_optionDetailsStr = "\r\n\t-data_const_val\r\n"
                                                      "\t<Data const value>\tvalid range: 0-255 (In case that \"Data mode\" == 0,"
                                                      " each data byte will carry this value (from 0-255))\r\n";
char *calibratorSetTx_cca_optionDetailsStr = "\r\n\t-cca\r\n"
                                             "\t<Enable CCA>\tvalid range: 0-1\r\n"
                                             "\t\t0 - Disable CCA\r\n"
                                             "\t\t1 - Enable CCA\r\n";
char *calibratorSetTx_bssColor_optionDetailsStr = "\r\n\t-bss_color\r\n"
                                                  "\t<BSS Color>\tvalid range: 0-63 "
                                                  "(This field is relevant for HE_SU and HE_SU_ER, Values: 0-63)\r\n";
char *calibratorSetTx_suErBw_optionDetailsStr =  "\r\n\t-su_er_bw\r\n"
                                                 "\t<SU_ER_Bandwidth>\tvalid range: 0-1 (0 for 242-tone RU,"
                                                 " 1 for upper frequency 106-tone RU within the primary 20MHz)\r\n";
char *calibratorSetTx_partialAid_optionDetailsStr = "\r\n\t-partial_aid\r\n"
                                                    "\t<Partial AID>\t(This field is relevant for VHT transmission)\r\n";
char *calibratorSetTx_srcAddr_optionDetailsStr = "\r\n\t-src_addr\r\n"
                                                 "\t<source MAC>\tsource MAC address (XX:XX:XX:XX:XX:XX)\r\n";
char *calibratorSetTx_dstAddr_optionDetailsStr = "\r\n\t-dst_addr\r\n"
                                                 "\t<dest MAC>\tdestination MAC address (XX:XX:XX:XX:XX:XX)\r\n";
char *calibratorSetTx_nominalPktExt_optionDetailsStr = "\r\n\t-nominal_pkt_ext\r\n"
                                                       "\t<Nominal packet extension>\tvalid range:"
                                                       " 0-2(This field is relevant for 11ax transmissions only)\r\n"
                                                       "\t\t0 - 0 usec (1)\r\n"
                                                       "\t\t1 - 8 usec\r\n"
                                                       "\t\t2 - 16 usec\r\n";
char *calibratorSetTx_feedStatus_optionDetailsStr = "\r\n\t-feed_status\r\n"
                                                    "\t<Feedback status>\tvalid range: 0-1 "
                                                    "(Indicates the value of the one bit used to modulate the tones in each tone set."
                                                    "This field is relevant for HE TB NDP FB transmissions only)\r\n";
char *calibratorSetTx_aid_optionDetailsStr = "\r\n\t-aid\n\r"
                                             "\t<AID>\tvalid range: 0-16383. AID of station can be up to 16383 (14 bits)\r\n";

char *calibratorSetTbTxUsageStr = " [-help] [-mimo_ltf_mode <mode> [-he_ltf_mode <mode>] [-pre_fec_padding factor <factor>]"
                                  " [-common_info_len <len>] [-ru_alloc <value>] [-ul_bw <bw>] [-starts_sts_num <num>]"
                                  " [-tb_auto <enable>] [-disamb <value>]\r\n";
char *calibratorSetTbTxDetailsStr = "Set TB_TX parameters.\r\n";

char *calibratorGetTxParamsUsageStr = " [-help] \r\n";
char *calibratorGetTxParamsDetailsStr = "Retrieve TX params for PLT.\r\n";

char *calibratorStartRxUsageStr = " [-help] [-source_mac <address>] [-ack_enable <enable>] [-aid <aid>] [-rate <rate>] [-preamble_type <type>]\r\n";
char *calibratorStartRxFirstDetailsStr = "\tThis command triggers the device to gather RX statistics for PLT. Use SWITCHES to add optional\r\n"
                                         "\tfilters for RXed packets. Use <source_mac> to designate source MAC address to listen to.\r\n";
char *calibratorStartRxSecondDetailsStr = "If you don't specify rate/preamble_type/aid, it won't be filtered at all. ACK_enable is defaulted FALSE\r\n";
char *calibratorStartRx_sourceMac_optionDetailsStr = "\r\n\t-source_mac\r\n"
                                                     "\t\tDesignate source mac address to filter RXed packets. Default is FF:FF:FF:FF:FF:FF\r\n";
char *calibratorStartRx_ackEnable_optionDetailsStr = "\r\n\t-ack_enable\r\n"
                                                     "\t\tChoose whether you'd like to work with ACKs or not. Default is false - disabling ACK response\r\n"
                                                     "\t\t0 - disabled\r\n"
                                                     "\t\t1 - enabled\r\n";
char *calibratorStartRx_aid_optionDetailsStr = "\r\n\t-aid\r\n\t\tSpecify requseted AID for PHY configuration\r\n";
char *calibratorStartRx_rate_optionFirstDetailsStr = "\r\n\t-rate\r\n"
                                                     "\t\tChoose desired rate to filter from received packets\r\n"
                                                     "\t\t\t1 - 1Mb/s\r\n"
                                                     "\t\t\t2 - 2Mb/s\r\n"
                                                     "\t\t\t3 - 5.5Mb/s\r\n"
                                                     "\t\t\t4 - 11Mb/s\r\n"
                                                     "\t\t\t5 - 6Mb/s\r\n"
                                                     "\t\t\t6 - 9Mb/s\r\n"
                                                     "\t\t\t7 - 12Mb/s\r\n"
                                                     "\t\t\t8 - 18Mb/s\r\n"
                                                     "\t\t\t9 - 24Mb/s\r\n"
                                                     "\t\t\t10 - 36Mb/s\r\n"
                                                     "\t\t\t11 - 48Mb/s\r\n"
                                                     "\t\t\t12 - 54Mb/s\r\n";
char *calibratorStartRx_rate_optionSecondDetailsStr = "\t\t\t13 - MCS0 (6.5Mb/s)\r\n"
                                                      "\t\t\t14 - MCS1 (13Mb/s)\r\n"
                                                      "\t\t\t15 - MCS2 (19.5.5Mb/s)\r\n"
                                                      "\t\t\t16 - MCS3 (26Mb/s)\r\n"
                                                      "\t\t\t17 - MCS4 (39Mb/s)\r\n"
                                                      "\t\t\t18 - MCS5 (52Mb/s)\r\n"
                                                      "\t\t\t19 - MCS6 (58.5Mb/s)\r\n"
                                                      "\t\t\t20 - MCS7 (65Mb/s)\r\n";
char *calibratorStartRx_preambleType_optionFirstDetailsStr = "\r\n\t-preamble_type\r\n"
                                                             "\t\tChoose preamble type - standard to filter from received packets\r\n"
                                                             "\t\t\t0 - PREAMBLE_TYPE_SHORT\r\n"
                                                             "\t\t\t1 - PREAMBLE_TYPE_LONG\r\n"
                                                             "\t\t\t2 - PREAMBLE_TYPE_OFDM\r\n"
                                                             "\t\t\t3 - PREAMBLE_TYPE_N_MIXED_MODE\r\n"
                                                             "\t\t\t4 - PREAMBLE_TYPE_GREENFIELD\r\n";
char *calibratorStartRx_preambleType_optionSecondDetailsStr = "\t\t\t5 - PREAMBLE_TYPE_AX_SU\r\n"
                                                             "\t\t\t6 - PREAMBLE_TYPE_AX_MU\r\n"
                                                             "\t\t\t7 - PREAMBLE_TYPE_AX_SU_ER\r\n"
                                                             "\t\t\t8 - PREAMBLE_TYPE_AX_TB\r\n"
                                                             "\t\t\t9 - PREAMBLE_TYPE_AX_TB_NDP_FB\r\n"
                                                             "\t\t\t10 - PREAMBLE_TYPE_AC_VHT\r\n";

char *calibratorStopRxUsageStr = " [-help]\r\n";
char *calibratorStopRxDetailsStr = "Stop any RX statistics work.\r\n";

char *calibratorGetRxStatsUsageStr = " [-help]\r\n";
char *calibratorGetRxStatsDetailsStr = "Retrieve RX statistics for PLT.\r\n\tUse when rx is ON.\r\n";

char *calibratorRateOverrideUsageStr = " [-help] <overEnable> <bw> <preamble> <rate> <dcm> <txPower> <giLTF>\r\n";
char *calibratorRateOverrideDetailsStr = "Rate override.\r\n";

char *calibratorGetBeaconRssiUsageStr = " [-help]\r\n";
char *calibratorGetBeaconRssiDetailsStr = "Get last beacon RSSI value.\r\n";

char *calibratorSetGiLtfUsageStr = " [-help] <value>\r\n";
char *calibratorSetGiLtfDetailsStr = "Set gi_ltf value.\r\n";

char *calibratorSetUplinkMuUsageStr = " [-help] <enable>\r\n";
char *calibratorSetUplinkMuDetailsStr = "Uplink Multiuser enable/disable.\r\n";

char *calibratorSetOperationModeControlUsageStr = " [-help] <enable>\r\n";
char *calibratorSetOperationModeControlDetailsStr = "Operation mode control enable/disable.\r\n";

char *calibratorSetMcsRateUsageStr = " [-help] <rate>\r\n";
char *calibratorSetMcsRateDetailsStr = "Set MCS rate value.\r\n";

char *calibratorSetUplinkMuDataUsageStr = " [-help] <enable>\r\n";
char *calibratorSetUplinkMuDataDetailsStr = "Set Uplink Multiuser data enable/disable.\r\n";

char *calibratorSetPsmUsageStr = " [-help] <index> <value>\r\n";
char *calibratorSetPsmDetailsStr = "Power Mode index and value.\r\n";

char *calibratorSetPowerHeaderUsageStr = " [-help] <enable>\r\n";
char *calibratorSetPowerHeaderDetailsStr = "Uplink power head enable/disable.\r\n";

char *calibratorSetTransmitOmiUsageStr = " [-help] <enable>\r\n";
char *calibratorSetTransmitOmiDetailsStr = "TRANSMIT_OMI enable/disable.\r\n";

const char *calibratorBAsessionUsageStr = " [-help] <block_ack_rx,1:0>  <block_ack_tx, 1:0> \r\n";
const char *calibratorBaSessionDetailsStr = "BA session RX enable, BA session TX enable .\r\n";

const char *calibratorLinkAdaptationUsageStr_1 = "\r\n [-help] <type : 0- FORCE_DISABLE_DCM|1- FORCE_DISABLE_ER|2- FORCE_DISABLE_ER_UPPER \r\n";
const char *calibratorLinkAdaptationUsageStr_2 = " [-help] <type : 3- FORCE_LONG_TERM_POLICY|4- FORCE_NOMINAL_PADDING|5- ENABLE_DEBUG_TRACE \r\n";
const char *calibratorLinkAdaptationDetailsStr_1 = "Param: 0- gForceDisableDcm |1- gForceDisableEr |2- gForceDisableErUpper \r\n ";
const char *calibratorLinkAdaptationDetailsStr_2 = "  Param: 3- gOverrideLongTermPolicyLsb + gOverrideLongTermPolicyMsb \r\n";
const char *calibratorLinkAdaptationDetailsStr_3 = "  Param: 4- gOverrideNominalPacketPaddingEn+gOverrideNominalPacketPaddingVal \r\n";
const char *calibratorLinkAdaptationDetailsStr_4 = "  Param: 5- gEnableLinkAdaptDebug \r\n";

const char *calibratorSetManualCalibUsageStr = " [-help]  -rx -tx \r\n";

const char *calibratorSetManualCalibDetailStr = "Set Manual Calibration for Rx/Tx\r\n"
    "\texample: set_manual_calib -rx 1 -tx 1\r\n"
    "\tcalibrate rx: range: 0-1 (False/True)\r\n"
    "\tcalibrate tx:  range 0-1 (False/True)\r\n";


/*************************************************
 *             Commands Parsing zone             *
 *************************************************/
/*!
    \brief       Parse calibrator start_rx command.

    This routine parses the calibrator`s start rx command. It receives a buffer
    to read from, parses the requested parameters and fills it in startRxParams parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorStartRxParams_t     -   Pointer received from caller, to be filled
                                                    in this routine with the stop rx
                                                    parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorStartRx(void *arg, CalibratorStartRxParams_t *startRxParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    /* Set default params */
    ret = macAddressParse("ff:ff:ff:ff:ff:ff", startRxParams->macAddress);
    if (ret < 0)
    {
        Report("\r\nError parsing mac address");
        return 0;
    }
    startRxParams->ackEnable = FALSE;
    startRxParams->aid = (uint16_t )RX_PARAMS_INVALID_AID;
    startRxParams->rate = (uint8_t )RX_PARAMS_INVALID_RATE;
    startRxParams->preambleType = (uint8_t )RX_PARAMS_INVALID_PREAMBLE_TYPE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while(token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (!strcmp(token, calibratorStartRxSourceMacOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -source_mac option, see help");
                return -1;
            }

            memset(startRxParams->macAddress, 0, MAC_ADDRESS_LEN);
            ret = macAddressParse(token, startRxParams->macAddress);
            if (ret < 0)
            {
                Report("\r\nError parsing source mac address");
                return -1;
            }
        }
        else if (!strcmp(token, calibratorStartRxAckEnableOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -ack_enable option, see help");
                return -1;
            }

            uint8_t ackEnable = (uint8_t )atoi(token);

            startRxParams->ackEnable = ackEnable;
        }
        else if (!strcmp(token, calibratorStartRxAidOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -aid option, see help");
                return -1;
            }

            uint16_t aid = (uint16_t )atoi(token);

            startRxParams->aid = aid;
        }
        else if (!strcmp(token, calibratorStartRxRateOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -rate option, see help");
                return -1;
            }

            uint8_t rate = (uint8_t )atoi(token);

            startRxParams->rate = rate;
        }
        else if (!strcmp(token, calibratorStartRxPreambleTypeOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -preamble_type option, see help");
                return -1;
            }

            uint8_t preambleType = (uint8_t )atoi(token);

            startRxParams->preambleType = preambleType;
        }
        else
        {
            Report("\r\nError, Wrong Syntax not a valid parameter");
            return -1;
        }
        paramCount += 2;
        token = strtok(NULL, space_str);
    }

    if ((paramCount % 2) != 0)
    {
        return -1;
    }

    return 0;
}

/*!
    \brief       Parse calibrator set_tb_tx command.

    This routine parses the calibrator`s set tb tx command. It receives a buffer
    to read from, parses the requested parameters and fills it in setTbTxParams parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorStartTxParams_t     -   Pointer received from caller, to be filled
                                                    in this routine with the set trigger
                                                    based TX parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetTbTx(void *arg, CalibratorStartTxParams_t *setTbTxParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int8_t paramCount = 0;

    setTbTxParams->bitmask = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while(token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (!strcmp(token, calibratorSetTxMimoLtfMode))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -mimo_ltf_mode option, see help");
                return -1;
            }

            uint8_t mimoLtfMode = (uint8_t )atoi(token);
            if ((mimoLtfMode > 1) || (mimoLtfMode < 0))
            {
                Report("\r\nmimoLtfMode is out of range (valid range: 0 or 1)");
                return -1;
            }

            setTbTxParams->ltfMode = mimoLtfMode;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_LTF_MODE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxHeLtfNum))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -he_ltf_num option, see help");
                return -1;
            }

            uint8_t heLtfNum = (uint8_t )atoi(token);
            if ((heLtfNum > 4) || (heLtfNum < 0))
            {
                Report("\r\nheLtfNum is out of range (valid range: 0-4)");
                return -1;
            }

            setTbTxParams->heLtfNum = heLtfNum;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_HE_LTF_NUM_BIT);
        }
        else if (!strcmp(token, calibratorSetTxDisamb))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -disamb option, see help");
                return -1;
            }

            uint8_t disamb = (uint8_t )atoi(token);
            if ((disamb > 1) || (disamb < 0))
            {
                Report("\r\ndisamb is out of range (valid range: 0 or 1)");
                return -1;
            }

            setTbTxParams->disamb = disamb;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_TB_DISAMB_MODE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxPreFecPaddingFactor))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -pre_fec_padding_factor option, see help");
                return -1;
            }

            uint8_t preFecPaddingFactor = (uint8_t )atoi(token);
            if ((preFecPaddingFactor > 7) || (preFecPaddingFactor < 0))
            {
                Report("\r\npreFecPaddingFactor is out of range (valid range: 0-7)");
                return -1;
            }

            setTbTxParams->preFecPaddingFactor = preFecPaddingFactor;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_PRE_FEC_PADDING_FACTOR_BIT);
        }
        else if (!strcmp(token, calibratorSetTxCommonInfoLen))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -common_info_len option, see help");
                return -1;
            }

            uint16_t commonInfoLen = (uint16_t )atoi(token);

            setTbTxParams->commonInfoLen = commonInfoLen;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_COMMON_INFO_LEN_BIT);
        }
        else if (!strcmp(token, calibratorSetTxRuAlloc))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -ru_alloc option, see help");
                return -1;
            }

            uint16_t ruAlloc = (uint16_t )atoi(token);
            if ((ruAlloc > 61) || (ruAlloc < 0))
            {
                Report("\r\nruAlloc is out of range (valid range: 0-61)");
                return -1;
            }

            setTbTxParams->ruAlloc = ruAlloc;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_RU_ALLOC_BIT);
        }
        else if (!strcmp(token, calibratorSetTxUlBw))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -ul_bw option, see help");
                return -1;
            }

            uint8_t ulBw = (uint8_t )atoi(token);
            if ((ulBw > 2) || (ulBw < 0))
            {
                Report("\r\nulBw is out of range (valid range: 0-2)");
                return -1;
            }

            setTbTxParams->ulBw = ulBw;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_UL_BW_BIT);
        }
        else if (!strcmp(token, calibratorSetTxStartsStsNum))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -starts_sts_num option, see help");
                return -1;
            }

            uint8_t startsStsNum = (uint8_t )atoi(token);
            if ((startsStsNum > 3) || (startsStsNum < 0))
            {
                Report("\r\nstartsStsNum is out of range (valid range: 0-3)");
                return -1;
            }

            setTbTxParams->startsStsNum = startsStsNum;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_STARTS_STS_NUM_BIT);
        }
        else if (!strcmp(token, calibratorSetTxTbAuto))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -tb_auto option, see help");
                return -1;
            }

            uint8_t tbAuto = (uint8_t )atoi(token);

            setTbTxParams->tbAutoMode = tbAuto;
            setTbTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_TB_AUTO_MODE_BIT);
        }
        else
        {
            Report("\r\nError, Wrong Syntax not a valid parameter");
            return -1;
        }
        paramCount += 2;
        token = strtok(NULL, space_str);
    }

    if ((paramCount % 2) != 0)
    {
        return -1;
    }

    return 0;
}


/*!
    \brief       Parse calibrator set_tx command.

    This routine parses the calibrator`s set tx command. It receives a buffer
    to read from, parses the requested parameters and fills it in setTxParams parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorStartTxParams_t     -   Pointer received from caller, to be filled
                                                    in this routine with the set TX parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetTx(void *arg, CalibratorStartTxParams_t *setTxParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    setTxParams->bitmask = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (!strcmp(token, calibratorSetTxDefaultOption)) /* for default values use "-default 0"*/
        {
            /* Other parameters aren't of interest if default is set */
            setTxParams->bitmask = 0;
            paramCount += 2;
            break;
        }
        else if (!strcmp(token, calibratorSetTxPreambleTypeOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -preamble_type option, see help");
                return -1;
            }

            uint8_t preambleType = (uint8_t )atoi(token);
            if ((preambleType > 10) || (preambleType < 0))
            {
                Report("\r\npreambleType is out of range (valid range: 0-10)");
                return -1;
            }

            if((preambleType == 6) || (preambleType == 4))
            {
                Report("\r\n11n GF and 11ax MU are currently not supported (preamble types 4,6)");
                return -1;
            }

            setTxParams->preambleType = preambleType;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_PREAMBLE_TYPE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxPhyRateOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -phy_rate option, see help");
                return -1;
            }
            uint8_t phyRate = (uint8_t )atoi(token);
            if ((phyRate > 20) || (phyRate < 1))
            {
                Report("\r\nPhyRate is out of range (valid range: 0-20");
                return -1;
            }

            setTxParams->phyRate = phyRate;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_PHY_RATE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxTxPowerOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -tx_power option, see help");
                return -1;
            }

            int8_t txPower = (int8_t )atoi(token);
            if ((txPower > 30) || (txPower < 0))
            {
                Report("\r\nTxPower is out of range (valid range: 0-30)");
                return -1;
            }

            setTxParams->txPower = txPower;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_TX_POWER_BIT);
        }
        else if (!strcmp(token, calibratorSetTxGiLtfTypeOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -gi_ltf_type option, see help");
                return -1;
            }
            uint8_t giLtfType = (uint8_t )atoi(token);
            if ((giLtfType > 5) || (giLtfType < 0))
            {
                Report("\r\nGI_LTF_Type is out of range (valid range: 0-5)");
                return -1;
            }

            setTxParams->GI_LTF_Type = giLtfType;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_GI_LTF_TYPE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxDCMOption))
        {
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -dcm option, see help");
                return -1;
            }

            uint8_t dcm = (uint8_t )atoi(token);
            if ((dcm > 1) || (dcm < 0))
            {
                Report("\r\nDCM is out of range (valid range: 0-1)");
                return -1;
            }

            setTxParams->DCM = dcm;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_DCM_BIT);
        }
        else if (!strcmp(token, calibratorSetTxLengthOption))
        {
            /* length <range> <start_length> <end_length>
                            OR
               length <const> <packet> <length> */

            token = strtok(NULL, space_str); /* range or const */
            if (!strcmp(token, "const"))
            {
                token = strtok(NULL, space_str); /* packet */

                if((strcmp(token, "packet") != 0) || !token)
                {
                    Report("\r\nError, length syntax is: -length const packet <length size>");
                    return -1;
                }

                token = strtok(NULL, space_str); /* length */
                if (!token)
                {
                    Report("\r\nError parsing -length option, see help");
                    return -1;
                }

                uint16_t length = (uint16_t )atoi(token);
                if ((length > 16000) || (length < 0))
                {
                    Report("\r\nConst length is out of range\r\n(valid range: Non MCS rate: 0-3500)\r\n(valid range: for MCS rate :0-16000)");
                    return -1;
                }
                else if (length > 3500)
                {
                    Report("\r\nWarning: length over 3500, make sure you are using aggregation (MCS rates)");
                }

                setTxParams->startLength = length;
                setTxParams->endLength = length;
            }
            else if (!strcmp(token, "range"))
            {
                token = strtok(NULL, space_str); /* start_length */
                if (!token)
                {
                    Report("\r\nError parsing -length option, see help");
                    return -1;
                }
                uint16_t startLength = (uint16_t )atoi(token);

                token = strtok(NULL, space_str); /* end_length */
                if (!token)
                {
                    Report("\r\nError parsing -length option, see help");
                    return -1;
                }
                uint16_t endLength = (uint16_t )atoi(token);

                if ((startLength > 3500) || (startLength < 100))
                {
                    Report("\r\nstart length is out of range (valid range: 100-3500)");
                    return -1;
                }
                if (endLength < startLength)
                {
                    Report("\r\nend length can't be less than start length");
                    return -1;
                }
                if (endLength > 3500)
                {
                    Report("\r\nend length can't be more than 3500");
                    return -1;
                }

                setTxParams->startLength = startLength;
                setTxParams->endLength = endLength;
            }
            else
            {
                Report("\r\nError parsing -length option, see help");
            }

            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_LENGTH_BIT);
        }
        else if (!strcmp(token, calibratorSetTxDelayOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -delay option, see help");
                return -1;
            }

            uint32_t delay = (uint32_t )atoi(token);
            if ((delay > 1000000) || (delay < 50))
            {
                Report("\r\nDelay is out of range (valid range: 50-1000000)");
                return -1;
            }

            setTxParams->delay = delay;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_DELAY_BIT);
        }
        else if (!strcmp(token, calibratorSetTxPktModeOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -pkt_mode option, see help");
                return -1;
            }

            uint8_t packetMode = (uint8_t )atoi(token);
            if ((packetMode > 2) || (packetMode < 0))
            {
                Report("\r\npacketMode is out of range (valid range: 0-2)");
                return -1;
            }

            setTxParams->packetMode = packetMode;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_PACKET_MODE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxNumPktsOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -num_pkts option, see help");
                return -1;
            }

            uint16_t packetsNum = (uint16_t )atoi(token);
            if ((packetsNum > 10000) || (packetsNum < 1))
            {
                Report("\r\npacketsNum is out of range (valid range: 1-10000)");
                return -1;
            }

            setTxParams->numberOfPackets = packetsNum;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_NUMBER_OF_PACKETS_BIT);

        }
        else if (!strcmp(token, calibratorSetTxDataModeOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -data_mode option, see help");
                return -1;
            }

            uint8_t dataMode = (uint8_t )atoi(token);
            if ((dataMode > 2) || (dataMode < 0))
            {
                Report("\r\ndataMode is out of range (valid range: 0-2)");
                return -1;
            }

            setTxParams->dataMode = dataMode;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_DATA_MODE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxDataConstValOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -data_const_val option, see help");
                return -1;
            }

            uint8_t dataConstValue = (uint8_t )atoi(token);
            if ((dataConstValue > 255) || (dataConstValue < 0))
            {
                Report("\r\ndataConstValue is out of range (valid range: 0-255)");
                return -1;
            }

            setTxParams->dataConstValue = dataConstValue;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_DATA_CONST_VALUE_BIT);
        }
        else if (!strcmp(token, calibratorSetTxCCAOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -cca option, see help");
                return -1;
            }

            uint8_t enableCCA = (uint8_t )atoi(token);
            if ((enableCCA > 1) || (enableCCA < 0))
            {
                Report("\r\nenableCCA is out of range (valid range: 0-1)");
                return -1;
            }

            setTxParams->enableCCA = enableCCA;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_ENABLE_CCA_BIT);
        }
        else if (!strcmp(token, calibratorSetTxBssColorOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -bss_color option, see help");
                return -1;
            }

            uint8_t bssColor = (uint8_t )atoi(token);
            if ((bssColor > 63) || (bssColor < 0))
            {
                Report("\r\nbssColor is out of range (valid range: 0-63)");
                return -1;
            }

            setTxParams->bssColor = bssColor;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_BSS_COLOR_BIT);
        }
        else if (!strcmp(token, calibratorSetTxSuErBwOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -su_er_bw option, see help");
                return -1;
            }

            uint16_t suErBw = (uint16_t )atoi(token);
            if ((suErBw > 1) || (suErBw < 0))
            {
                Report("\r\nSU_ER_BW is out of range (valid range: 0-1)");
                return -1;
            }

            setTxParams->SU_ER_Bandwidth = suErBw;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_SU_ER_BANDWIDTH_BIT);
        }
        else if (!strcmp(token, calibratorSetTxPartialAidOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -partial_aid option, see help");
                return -1;
            }

            uint8_t partialAID = (uint8_t )atoi(token);
            if ((partialAID > 255) || (partialAID < 0))
            {
                Report("\r\npartialAID is out of range (valid range: 0-255)");
                return -1;
            }

            setTxParams->partialAID = partialAID;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_PARTIAL_AID_BIT);
        }
        else if (!strcmp(token, calibratorSetTxSrcAddrOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -src_addr option, see help");
                return -1;
            }

            ret = macAddressParse(token, setTxParams->srcAddr);
            if (ret < 0)
            {
                Report("\r\nError parsing source MAC address");
                return -1;
            }

            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_SRC_ADDR_BIT);
        }
        else if (!strcmp(token, calibratorSetTxDstAddrOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -dst_addr option, see help");
                return -1;
            }

            ret = macAddressParse(token, setTxParams->dstAddr);
            if (ret < 0)
            {
                Report("\r\nError parsing destination MAC address");
                return -1;
            }

            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_DST_ADDR_BIT);
        }
        else if (!strcmp(token, calibratorSetTxNominalPktExtOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -nominal_pkt_ext option, see help");
                return -1;
            }

            uint8_t nominalPktExt = (uint8_t )atoi(token);
            if ((nominalPktExt > 2) || (nominalPktExt < 0))
            {
                Report("\r\nnominalPktExt is out of range (valid range: 0-2)");
                return -1;
            }

            setTxParams->nominalPacketExtension = nominalPktExt;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_NOMINAL_PACKET_EXTENSION_BIT);
        }
        else if (!strcmp(token, calibratorSetTxFeedStatusOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -feed_status option, see help");
                return -1;
            }

            uint8_t feedbackStatus = (uint8_t )atoi(token);
            if ((feedbackStatus > 1) || (feedbackStatus < 0))
            {
                Report("\r\feedbackStatus is out of range (valid range: 0-1)");
                return -1;
            }

            setTxParams->feedbackStatus = feedbackStatus;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_FEEDBACK_STATUS_BIT);

        }
        else if (!strcmp(token, calibratorSetTxAidOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -aid option, see help");
                return -1;
            }

            uint16_t aid = (uint16_t )atoi(token);
            if ((aid > 16383) || (aid < 0))
            {
                Report("\r\naid is out of range (valid range: 0-16383)");
                return -1;
            }

            setTxParams->aid = aid;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_AID_BIT);

        }
        else if (!strcmp(token, calibratorSetTxGroupIdOption))
        {
            token = strtok(NULL, space_str);

            if (!token)
            {
                Report("\r\nError parsing -group_id option, see help");
                return -1;
            }

            uint16_t groupId = (uint16_t )atoi(token);

            setTxParams->groupId = groupId;
            setTxParams->bitmask |= (1 << CALIBRATOR_SET_TX_GROUP_ID_BIT);
        }
        else
        {
            Report("\r\nError, Wrong Syntax not a valid parameter");
            return -1;
        }
        paramCount += 2;
        token = strtok(NULL, space_str);
    }

    if ((paramCount % 2) != 0)
    {
        return -1;
    }

    return 0;
}

/*!
    \brief       Parse calibrator channel_tune command.

    This routine parses the calibrator`s channel tune command. It receives a buffer
    to read from, parses the requested parameters and fills it in tuneParams parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorChannelTuneParams   -   Pointer received from caller, to be filled
                                                    in this routine with the requested channel
                                                    tune parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorChannelTune(void *arg, CalibratorChannelTuneParams_t *tuneParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        return -1; /* Return a print-triggering value */
    }

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            tuneParams->channel = (uint8_t )atoi(token);
        }
        else if (1 == paramCount)
        {
            tuneParams->band = (uint8_t )atoi(token);
        }
        else if (2 == paramCount)
        {
            tuneParams->bandwidth = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (paramCount != 3)
    {
        Report("\r\nWrong amount of arguments for calibrator channel tune. Expected 3, got %d", paramCount);
        ret = -1;
    }

    return ret;
}

/*!
    \brief       Parse calibrator rate override command.

    This routine parses the calibrator`s rate override command. It receives a buffer
    to read from, parses the requested parameters and fills it in override parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        rateOverrideParams            -   Pointer received from caller, to be filled
                                                    in this routine with the requested rate
                                                    override parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorRateOverride(void *arg, CalibratorRateOverrideParams_t *rateOverrideParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            rateOverrideParams->overEnable = (uint8_t )atoi(token);
        }
        else if (1 == paramCount)
        {
            rateOverrideParams->bw = (uint8_t )atoi(token);
        }
        else if (2 == paramCount)
        {
            rateOverrideParams->preamble = (uint8_t )atoi(token);
        }
        else if (3 == paramCount)
        {
            rateOverrideParams->rate = (uint8_t )atoi(token);
        }
        else if (4 == paramCount)
        {
            rateOverrideParams->dcm = (uint8_t )atoi(token);
        }
        else if (5 == paramCount)
        {
            rateOverrideParams->txPower = (uint8_t )atoi(token);
        }
        else if (6 == paramCount)
        {
            rateOverrideParams->giLTF = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (paramCount != 7)
    {
        Report("\r\nWrong amount of arguments for calibrator rate override. Expected 7, got %d", paramCount);
        ret = -1;
    }

    return ret;
}

/*!
    \brief       Parse calibrator ba enable command.

    This routine parses the calibrator`s ba enable command. It receives a buffer
    to read from, parses the requested parameters and fills it in ba enable parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        rateOverrideParams            -   Pointer received from caller, to be filled
                                                    in this routine with the requested rate
                                                    override parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorBAEnable(void *arg, CalibratorBASessionparams_t *ba_enable)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            ba_enable->block_ack_rx = (uint8_t )atoi(token);
        }
        else if (1 == paramCount)
        {
            ba_enable->block_ack_tx = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (paramCount != 2)
    {
        Report("\r\nWrong amount of arguments for calibrator ba enable . Expected 2, got %d", paramCount);
        ret = -1;
    }

    return ret;
}

/*!
    \brief       Parse calibrator link adapt command.

    This routine parses the calibrator`s link adapt command. It receives a buffer
    to read from, parses the requested parameters and fills it in link adapt  parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        rateOverrideParams            -   Pointer received from caller, to be filled
                                                    in this routine with the requested rate
                                                    override parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorLinkAdapt(void *arg, CalibratorLinkAdaptParams_t *link_adapt)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;
    uint8_t maxParam = 2;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            link_adapt->type = (uint8_t )atoi(token);

            switch(link_adapt->type)
            {
                case CAL_FORCE_DISABLE_DCM:
                case CAL_FORCE_DISABLE_ER:
                case CAL_FORCE_DISABLE_ER_UPPER:
                case CAL_ENABLE_DEBUG_TRACE:
                    maxParam = 2;
                    break;
                case CAL_FORCE_LONG_TERM_POLICY:
                case CAL_FORCE_NOMINAL_PADDING:
                    maxParam = 3;
                    break;
                default:
                    Report("\r\nWrong type  %d ",link_adapt->type);
                    ret = -1;
                    return ret;
                    break;

            }
        }
        else if (1 == paramCount)
        {
            link_adapt->param1 = (uint8_t )atoi(token);
        }
        else if (2 == paramCount)
        {
            link_adapt->param2 = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (paramCount != maxParam)
    {
        Report("\r\nWrong amount of arguments for calibrator link adaption .for type  %d Expected %d, got %d",link_adapt->type,maxParam,paramCount);
        ret = -1;
    }

    return ret;
}

/*!
    \brief       Parse calibrator set gi_ltf command.

    This routine parses the calibrator`s gi_ltf command. It receives a buffer
    to read from, parses the requested parameters and fills it in value parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorGiLtfModeParams_t   -   Pointer received from caller, to be filled
                                                    in this routine with the requested gi_ltf
                                                    parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorsetGiLtf(void *arg, CalibratorGiLtfModeParams_t *setGiLtfParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            setGiLtfParams->value = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (setGiLtfParams->value <= 0)
    {
        Report("\r\nGI_LTF must be grater than 0\r\n");
        return -1;
    }

    if (paramCount != 1)
    {
        Report("\r\nWrong amount of arguments for calibrator gi_ltf. Expected 1, got %d", paramCount);
        return -1;
    }

    Report("\r\nGI_LTF value: %d\r\n", setGiLtfParams->value);

    return ret;
}

/*!
    \brief       Parse calibrator set uplink_mu command.

    This routine parses the calibrator`s uplink_mu command. It receives a buffer
    to read from, parses the requested parameters and fills it in enable parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorUplinkMuParams_t    -   Pointer received from caller, to be filled
                                                    in this routine with the requested
                                                    uplink_mu parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetUplinkMu(void *arg, CalibratorUplinkMuParams_t *uplinkMuParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            uplinkMuParams->enableVal = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if ((uplinkMuParams->enableVal > 1) || (uplinkMuParams->enableVal < 0))
    {
        Report("\r\nenable value can be 0/1\r\n");
        return -1;
    }

    if (paramCount != 1)
    {
        Report("\r\nWrong amount of arguments for calibrator uplink_mu. Expected 1, got %d", paramCount);
        return -1;
    }

    Report("\r\nUplink Multiuser enable/disable: %d\r\n", uplinkMuParams->enableVal);

    return ret;
}

/*!
    \brief       Parse calibrator set oper_mode command.

    This routine parses the calibrator`s oper_mode command. It receives a buffer
    to read from, parses the requested parameters and fills it in enable parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorOperationModeControlParams_t    -   Pointer received from caller, to be filled
                                                                in this routine with the requested
                                                                oper_mode parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetOperationModeControl(void *arg, CalibratorOperationModeControlParams_t *operationModeControlParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            operationModeControlParams->enableVal = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if ((operationModeControlParams->enableVal > 1) || (operationModeControlParams->enableVal < 0))
    {
        Report("\r\nenable value can be 0/1\r\n");
        return -1;
    }

    if (paramCount != 1)
    {
        Report("\r\nWrong amount of arguments for calibrator oper_mode. Expected 1, got %d", paramCount);
        return -1;
    }

    Report("\r\nOperation mode control enable/disable: %d\r\n", operationModeControlParams->enableVal);

    return ret;
}

/*!
    \brief       Parse calibrator set mcs_rate command.

    This routine parses the calibrator`s mcs_rate command. It receives a buffer
    to read from, parses the requested parameters and fills it in rate parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorSetMcsRateParams_t  -   Pointer received from caller, to be filled
                                                    in this routine with the requested
                                                    mcs_rate parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetMcsRate(void *arg, CalibratorSetMcsRateParams_t *setMcsRateParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            setMcsRateParams->rateVal = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (setMcsRateParams->rateVal <= 0)
    {
        Report("\r\nMCS rate must be greater than 0\r\n");
        return -1;
    }

    if (paramCount != 1)
    {
        Report("\r\nWrong amount of arguments for calibrator mcs_rate. Expected 1, got %d", paramCount);
        return -1;
    }

    Report("\r\nMCS Fixed rate value: %d\r\n", setMcsRateParams->rateVal);

    return ret;
}

/*!
    \brief       Parse calibrator set uplink_mu_data command.

    This routine parses the calibrator`s uplink_mu_data command. It receives a buffer
    to read from, parses the requested parameters and fills it in enable parameter.

    \param        arg                                -   Points to command line buffer.
                                                         Contains the command line typed by user.

    \param        CalibratorSetUplinkMuDataParams_t  -   Pointer received from caller, to be filled
                                                         in this routine with the requested
                                                         uplink_mu_data parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetUplinkMuData(void *arg, CalibratorSetUplinkMuDataParams_t *setUplinkMuDataParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            setUplinkMuDataParams->enableVal = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if ((setUplinkMuDataParams->enableVal < 0) || (setUplinkMuDataParams->enableVal > 1))
    {
        Report("\r\nenable value can be 0/1\r\n");
        return -1;
    }

    if (paramCount != 1)
    {
        Report("\r\nWrong amount of arguments for calibrator uplink_mu_data. Expected 1, got %d", paramCount);
        return -1;
    }

    Report("\r\nUplink Multiuser data enable/disable: %d\r\n", setUplinkMuDataParams->enableVal);

    return ret;
}

/*!
    \brief       Parse calibrator set_tx command.

    This routine parses the calibrator`s set tx command. It receives a buffer
    to read from, parses the requested parameters and fills it in setTxParams parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        CalibratorStartTxParams_t     -   Pointer received from caller, to be filled
                                                    in this routine with the set TX parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 *///
int32_t parseCalibratorSetManualCalib(void *arg, uint16* calibration_bitmap)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);


    if(token == NULL)
    {
        return -1;
    }


    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        if (!strcmp(token, "-rx"))
        {
            /* rx */
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -rx option, see help");
                return -1;
            }

            if((atoi(token) > 1 ) || (atoi(token) < 0))
            {
                Report("\r\nrx is out of range (valid range: 0 or 1)\n");
                return -1;
            }
            if(atoi(token) == 1)
                *calibration_bitmap = (*calibration_bitmap) | CALIB_RX_IQMM_BITWISE_MASK | CALIB_RX_DC_BITWISE_MASK | CALIB_RX_SPUR_CANCELER_BITWISE_MASK;

        } else if (!strcmp(token, "-tx"))
        {
            /* tx */
            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing -tx option, see help");
                return -1;
            }
            /* tx */
            if ((atoi(token) > 1 ) || (atoi(token) < 0))
            {
                Report("\r\ntx is out of range (valid range: 0 or 1)\n");
                return -1;
            }

            if (atoi(token) == 1)
                    *calibration_bitmap = (*calibration_bitmap) | CALIB_TX_AUX_RX_DC_BITWISE_MASK | CALIB_TX_IQMM_BITWISE_MASK
                                                | CALIB_TX_LOL_BITWISE_MASK | CALIB_TX_RFNL_AND_DPD_BITWISE_MASK;
        } else {
            Report("\r\nError, Wrong Syntax, %s is not a valid parameter \n",token);
            return -1;
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }


    if ((paramCount == 0) || ((paramCount % 2) != 0))
    {
        Report("\r\nWrong amount of arguments for calibrator channel tune. Expected 1, got %d", paramCount);
        ret = -1;
    }

    return ret;
}


/*!
    \brief       Parse calibrator set psm command.

    This routine parses the calibrator`s psm command. It receives a buffer
    to read from, parses the requested parameters and fills it in setPsmParams parameter.

    \param        arg                           -   Points to command line buffer.
                                                    Contains the command line typed by user.

    \param        setPsmParams                  -   Pointer received from caller, to be filled
                                                    in this routine with the requested
                                                    psm parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetPsm(void *arg, CalibratorSetPsmParams_t *setPsmParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            setPsmParams->index = (uint8_t )atoi(token);

            token = strtok(NULL, space_str);
            if (!token)
            {
                Report("\r\nError parsing value");
                return -1;
            }
            setPsmParams->value = (uint32_t )atoi(token);

            switch (setPsmParams->index)
            {
                case CALIBRATOR_ACTIVE_MODE_RX_TH:
                case CALIBRATOR_ACTIVE_MODE_TX_TH:
                {
                    if(setPsmParams->value > 255)
                    {
                        Report("\r\nValue threshold too high (max 255)");
                        return -1;
                    }

                }
                    break;
                case CALIBRATOR_ACTIVE_MODE_RX_TO:
                case CALIBRATOR_ACTIVE_MODE_TX_TO:
                {

                }
                    break;
                case CALIBRATOR_FORCE_POWER_MODE:
                {
                    if(setPsmParams->value > 2)
                    {
                        Report("\r\nValue for force power mode must be 2 or lower.");
                        return -1;
                    }
                }
                    break;
                default:
                    Report("unvalid param index\r\n");
                    return 1;
            }

            paramCount++;
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (paramCount != 2)
    {
        Report("\r\nWrong amount of arguments for calibrator psm. Expected 2, got %d", paramCount);
        return -1;
    }

    Report("\r\nparam_index: %d param_value%d\r\n", setPsmParams->index, setPsmParams->value);

    return ret;
}

/*!
    \brief       Parse calibrator set power_head command.

    This routine parses the calibrator`s power_head command. It receives a buffer
    to read from, parses the requested parameters and fills it in setPowerHeaderParams parameter.

    \param        arg                               -   Points to command line buffer.
                                                        Contains the command line typed by user.

    \param        CalibratorSetPowerHeaderParams_t  -   Pointer received from caller, to be filled
                                                        in this routine with the requested
                                                        power_head parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetPowerHeader(void *arg, CalibratorSetPowerHeaderParams_t *setPowerHeaderParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            setPowerHeaderParams->enableVal = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if ((setPowerHeaderParams->enableVal < 0) || (setPowerHeaderParams->enableVal > 1))
    {
        Report("\r\nenable value can be 0/1\r\n");
        return -1;
    }

    if (paramCount != 1)
    {
        Report("\r\nWrong amount of arguments for calibrator power_head. Expected 1, got %d", paramCount);
        return -1;
    }

    Report("\r\nUplink power header enable/disable: %d\r\n", setPowerHeaderParams->enableVal);

    return ret;
}

/*!
    \brief       Parse calibrator set trans_omi command.

    This routine parses the calibrator`s trans_omi command. It receives a buffer
    to read from, parses the requested parameters and fills it in setPowerHeaderParams parameter.

    \param        arg                               -   Points to command line buffer.
                                                        Contains the command line typed by user.

    \param        CalibratorSetTransmitOmiParams_t  -   Pointer received from caller, to be filled
                                                        in this routine with the requested
                                                        trans_omi parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetTransmitOmi(void *arg, CalibratorSetTransmitOmiParams_t *setTransmitOmiParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            setTransmitOmiParams->enableVal = (uint8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if ((setTransmitOmiParams->enableVal < 0) || (setTransmitOmiParams->enableVal > 1))
    {
        Report("\r\nTRANSMIT_OMI value can be 0/1\r\n");
        return -1;
    }

    if (paramCount != 1)
    {
        Report("\r\nWrong amount of arguments for calibrator trans_omi. Expected 1, got %d", paramCount);
        return -1;
    }

    Report("\r\nTRANSMIT_OMI value: %d\r\n", setTransmitOmiParams->enableVal);

    return ret;
}

/*!
    \brief       Parse calibrator power_mode command.

    This routine parses the calibrator`s power mode command. It receives a buffer
    to read from, parses the requested power mode and fills it in requestedCalibratorMode
    parameter.

    \param        arg                       -   Points to command line buffer.
                                                Contains the command line typed by user.

    \param        requestedCalibratorMode   -   Pointer received from caller, to be filled
                                                in this routine with the requested working
                                                mode for the calibrator tool.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorSetPowerMode(void *arg, CalibratorPowerModes_e *requestedCalibratorMode)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    int32_t ret = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        return -1; /* Return a print-triggering value */
    }

    if(!strcmp(token, calibratorPowerOptions[0]))
    {
        *requestedCalibratorMode = PLT_OFF;
    }
    else if (!strcmp(token, calibratorPowerOptions[1]))
    {
        *requestedCalibratorMode = PLT_ON;
    }
    else if (!strcmp(token, help_optionStr))
    {
        return -1;
    }
    else
    {
        return -1;
    }


    return ret;
}
/*!
    \brief       Parse calibrator start tx tone command.

    This routine parses the calibrator`s start tx tone command. It receives a buffer
    to read from, parses the requested parameters and fills it in startTxTone parameter.

    \param        arg                               -   Points to command line buffer.
                                                        Contains the command line typed by user.

    \param        CalibratorStartTxToneParams_t     -   Pointer received from caller, to be filled
                                                        in this routine with the requested
                                                        start tx tone parameters.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdCalibratorCallback
 */
int32_t parseCalibratorStartTxTone(void *arg, CalibratorStartTxToneParams_t *startTxToneParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    int32_t ret = 0;
    int8_t paramCount = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            return -1;
        }
        else if (0 == paramCount)
        {
            startTxToneParams->mode = (uint8_t )atoi(token);
        }
        else if (1 == paramCount)
        {
            startTxToneParams->offset = (int8_t ) floor((atof(token) / 0.25));
        }
        else if (2 == paramCount)
        {
            startTxToneParams->output_power = (int8_t )atoi(token);
        }

        paramCount++;
        token = strtok(NULL, space_str);
    }

    if (paramCount != 3)
    {
        Report("\r\nWrong amount of arguments for calibrator start tone tx. Expected 3, got %d", paramCount);
        ret = -1;
    }

    if ((startTxToneParams->mode < 0) || (startTxToneParams->mode > 2))
    {
        Report("\r\nmode value can be 0/1/2\r\n");
        return -1;
    }

    if ((startTxToneParams->offset < (-10 * 4)/*floor(-10/0.25)*/) || (startTxToneParams->offset > (10 * 4)/*floor(10/0.25)*/))
    {
        Report("\r\noffset value can be -10 to +10 MHz\r\n");
        return -1;
    }

    if ((startTxToneParams->output_power < -10) || (startTxToneParams->output_power > 15))
    {
        Report("\r\noutput_power value can be -10 to +15 dBm\r\n");
        return -1;
    }

    Report("\r\nTx Tone mode: %d\r\n", startTxToneParams->mode);
    Report("\r\nTx Tone offset: (%d) %.2f MHz\r\n", startTxToneParams->offset, (startTxToneParams->offset * 0.25));
    Report("\r\nTx Tone output_power: %d dBm\r\n", startTxToneParams->output_power);
    return ret;
}

/*!
    \brief          Parse Calibrator action.

    This routine takes a CalibratorAction_e enum variable, and fills its content
    with the action the user requested for the calibrator to follow.

    \param          arg                 -       Points to command line buffer.
                                                Contains the command line typed by user.

    \param          CalibratorAction    -       The action requested from calibrator
                                                tool. Filled by this routine.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1.
                    It's the callback's responsibility to check the return value,
                    and print the help for this command.

    \sa             cmdCalibratorCallback
 */
int32_t parseCalibratorAction(void *arg, CalibratorAction_e *calibratorAction)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    int32_t ret = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        return -1; /* Return a print-triggering value */
    }

    if(!strcmp(token, help_optionStr))
    {
        return -1; /* Return a print-triggering value */
    }
    else if (!strcmp(token, calibratorSetPowerModeActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_POWER_MODE;
    }
    else if (!strcmp(token, calibratorChannelTuneActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_CHANNEL_TUNE;
    }
    else if (!strcmp(token, calibratorStartTxActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_TX_START;
    }
    else if (!strcmp(token, calibratorStartTxToneActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_TX_TONE_START;
    }
    else if (!strcmp(token, calibratorStopTxActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_TX_STOP;
    }
    else if (!strcmp(token, calibratorStartRxActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_START_RX;
    }
    else if (!strcmp(token, calibratorStopRxActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_STOP_RX;
    }
    else if (!strcmp(token, calibratorGetRxStatsActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_GET_RX_STATS;
    }
    else if (!strcmp(token, calibratorSetTxParamsActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_TX_PARAMS;
    }
    else if (!strcmp(token, calibratorGetTxParamsActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_GET_TX_PARAMS;
    }
    else if (!strcmp(token, calibratorSetTbTxParamsActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_TB_TX_PARAMS;
    }
    else if (!strcmp(token, calibratorRateOverrideActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_RATE_OVERRIDE;
    }
    else if (!strcmp(token, calibratorGetBeaconRssiActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_GET_BEACON_RSSI;
    }
    else if (!strcmp(token, calibratorSetGiLtfActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_GI_LTF;
    }
    else if (!strcmp(token, calibratorSetUplinkMuActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_UPLINK_MU;
    }
    else if (!strcmp(token, calibratorSetOperationModeControlActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_OPERATION_MODE_CONTROL;
    }
    else if (!strcmp(token, calibratorSetMcsRateActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_MCS_RATE;
    }
    else if (!strcmp(token, calibratorSetUplinkMuDataActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_UPLINK_MU_DATA;
    }
    else if (!strcmp(token, calibratorSetPsmActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_PSM;
    }
    else if (!strcmp(token, calibratorSetUplinkPowerHeaderActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_UPLINK_POWER_HEADER;
    }
    else if (!strcmp(token, calibratorSetTransmitOMIActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_SET_TRANSMIT_OMI;
    }
    else if (!strcmp(token, calibratorSetBAEnableActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_BA_SESSION;
    }
    else if (!strcmp(token, calibratorLinkAdaptActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_LINK_ADAPT;
    }
    else if (!strcmp(token, calibratorSetManualCalibActionStr))
    {
        *calibratorAction = CALIBRATOR_ACTION_MANUAL_CALIBRATION;
    }
    else
    {
        return -1;
    }

    return ret;
}


/*************************************************
 *             Usage and prints zone             *
 *************************************************/

void calibratorDisplayTxParams(CalibratorStartTxParams_t *getTxParams)
{
    Report("\r\nParameter\t\t\tValue");
    Report("\r\nPreamble Type:\t\t\t%d", getTxParams->preambleType);
    Report("\r\nPhyRate:\t\t\t%d", getTxParams->phyRate);
    Report("\r\nTx Power:\t\t\t%d", getTxParams->txPower);
    Report("\r\nGI LTF TYPE:\t\t\t%d", getTxParams->GI_LTF_Type);
    Report("\r\nDCM:\t\t\t\t%d", getTxParams->DCM);
    Report("\r\nstart Length:\t\t\t%hi", getTxParams->startLength);
    Report("\r\nend Length:\t\t\t%hi", getTxParams->endLength);
    Report("\r\nDelay:\t\t\t\t%hi", getTxParams->delay);
    Report("\r\nPacket Mode:\t\t\t%hi", getTxParams->packetMode);
    Report("\r\nNumber of Packets:\t\t%hi", getTxParams->numberOfPackets);
    Report("\r\ndataMode:\t\t\t%hi", getTxParams->dataMode);
    Report("\r\ndata Const Value:\t\t%hi", getTxParams->dataConstValue);
    Report("\r\nEnable CCA:\t\t\t%hi", getTxParams->enableCCA);
    Report("\r\nBSS Color:\t\t\t%hi", getTxParams->bssColor);
    Report("\r\nSU ER Bandwidth:\t\t%hi", getTxParams->SU_ER_Bandwidth);
    Report("\r\nSRC Addr:  \t\t\t%02x:%02x:%02x:%02x:%02x:%02x", getTxParams->srcAddr[0],
           getTxParams->srcAddr[1],getTxParams->srcAddr[2],getTxParams->srcAddr[3],
           getTxParams->srcAddr[4],getTxParams->srcAddr[5]);
    Report("\r\nDST Addr:  \t\t\t%02x:%02x:%02x:%02x:%02x:%02x", getTxParams->dstAddr[0],
           getTxParams->dstAddr[1],getTxParams->dstAddr[2],getTxParams->dstAddr[3],
           getTxParams->dstAddr[4],getTxParams->dstAddr[5]);
    Report("\r\nnominal Packet Extension:\t%hi", getTxParams->nominalPacketExtension);
    Report("\r\nFeedback Status:\t\t%hi", getTxParams->feedbackStatus);
    Report("\r\nAid:\t\t\t\t%hi", getTxParams->aid);
    Report("\r\nGroup ID:\t\t\t%hi", getTxParams->groupId);
    Report("\r\nTB MIMO LTF MODE:\t\t%hi", getTxParams->ltfMode);
    Report("\r\nTB HE LTF Number:\t\t%hi", getTxParams->heLtfNum);
    Report("\r\nTB Pre Fec Padding Factor:\t%hi", getTxParams->preFecPaddingFactor);
    Report("\r\nTB Common_info_len:\t\t%hi", getTxParams->commonInfoLen);
    Report("\r\nTB RU ALLOCATION:\t\t%hi", getTxParams->ruAlloc);
    Report("\r\nTB UL BW:\t\t\t%hi", getTxParams->ulBw);
    Report("\r\nTB Starts STS num:\t\t%hi", getTxParams->startsStsNum);
    Report("\r\nTB auto mode:     \t\t%hi", getTxParams->tbAutoMode);
    Report("\r\nTB disamb:\t\t\t%hi", getTxParams->disamb);
    Report("status:\t\t\t\t0\r\n");
}

void calibratorDisplayRxStats(CalibratorGetRxParams_t *getRxParams)
{
    Report("\r\nTotal Received Packets:\t%d", getRxParams->receivedTotalPacketsNumber);
    Report("\r\nFCS Errors:\t\t%d", getRxParams->receivedFcsErrorPacketsNumber);
    Report("\r\nMAC Mismatch:\t\t%d", getRxParams->receivedAddressMismatchPacketsNumber);
    Report("\r\nGood Packets:\t\t%d", getRxParams->receivedGoodPackets);
    Report("\r\nAverage RSSI (SOC):\t%hi", getRxParams->averageDataCtrlRssi);
    Report("\r\nAverage RSSI (ANT):\t%hi", getRxParams->averageDataCtrlSNR);
    Report("status: 0\r\n");
    if (getRxParams->receivedTotalPacketsNumber)
    {
        float per = ((float)getRxParams->receivedTotalPacketsNumber - \
                    (float)getRxParams->receivedGoodPackets)/(float)getRxParams->receivedTotalPacketsNumber;
        Report("\r\nPER:\t\t\t%f     # PER = Total Bad / Total Received\r\n", per);
    }
    else
    {
        Report("\r\nPER:\t\t\tN/A     # PER = Total Bad / Total Received\r\n");
    }
}

int32_t printCalibratorUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorStr);
    UART_PRINT(calibratorUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorFirstDetailsStr);
    UART_PRINT(calibratorSecondDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return 0;
}

void printCalibratorPowerModeUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetPowerModeActionStr);
    UART_PRINT(calibratorSetPowerModeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetPowerModeDetailsStr);
    UART_PRINT(calibratorSetPower_mode_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorTuneChannelUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorChannelTuneActionStr);
    UART_PRINT(calibratorTuneChannelUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorTuneChannelDetailsStr);
    UART_PRINT(calibratorTuneChannel_channel_optionDetailsStr);
    UART_PRINT(calibratorTuneChannel_band_optionDetailsStr);
    UART_PRINT(calibratorTuneChannel_bandwidth_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorStartTxUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorStartTxActionStr);
    UART_PRINT(calibratorStartTxUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorStartTxDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorStartTxToneUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorStartTxToneActionStr);
    UART_PRINT(calibratorStartTxToneFirstUsageStr);
    UART_PRINT(calibratorStartTxToneSecondUsageStr);
    UART_PRINT(calibratorStartTxToneThirdUsageStr);    
    UART_PRINT(calibratorStartTxToneFourthUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorStartTxToneDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorStopTxUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorStopTxActionStr);
    UART_PRINT(calibratorStopTxUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorStopTxDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetTxUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetTxParamsActionStr);
    UART_PRINT(calibratorSetTxFirstUsageStr);
    UART_PRINT(calibratorSetTxSecondUsageStr);
    UART_PRINT(calibratorSetTxThirdUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetTxDetailsStr);
    UART_PRINT(calibratorSetTx_preamble_optionFirstDetailsStr);
    UART_PRINT(calibratorSetTx_preamble_optionSecondDetailsStr);
    UART_PRINT(calibratorSetTx_phyRate_optionFirstDetailsStr);
    UART_PRINT(calibratorSetTx_phyRate_optionSecondDetailsStr);
    UART_PRINT(calibratorSetTx_phyRate_optionThirdDetailsStr);
    UART_PRINT(calibratorSetTx_txPower_optionDetailsStr);
    UART_PRINT(calibratorSetTx_giLtfType_optionDetailsStr);
    UART_PRINT(calibratorSetTx_dcm_optionDetailsStr);
    UART_PRINT(calibratorSetTx_lengthRange_optionDetailsStr);
    UART_PRINT(calibratorSetTx_lengthConst_optionDetailsStr);
    UART_PRINT(calibratorSetTx_delay_optionDetailsStr);
    UART_PRINT(calibratorSetTx_packetMode_optionFirstDetailsStr);
    UART_PRINT(calibratorSetTx_packetMode_optionSecondDetailsStr);
    UART_PRINT(calibratorSetTx_numPackets_optionDetailsStr);
    UART_PRINT(calibratorSetTx_dataMode_optionDetailsStr);
    UART_PRINT(calibratorSetTx_dataConstVal_optionDetailsStr);
    UART_PRINT(calibratorSetTx_cca_optionDetailsStr);
    UART_PRINT(calibratorSetTx_bssColor_optionDetailsStr);
    UART_PRINT(calibratorSetTx_srcAddr_optionDetailsStr);
    UART_PRINT(calibratorSetTx_dstAddr_optionDetailsStr);
    UART_PRINT(calibratorSetTx_nominalPktExt_optionDetailsStr);
    UART_PRINT(calibratorSetTx_feedStatus_optionDetailsStr);
    UART_PRINT(calibratorSetTx_aid_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetTbTxUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetTbTxParamsActionStr);
    UART_PRINT(calibratorSetTbTxUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetTbTxDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorGetTxParamsUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorGetTxParamsActionStr);
    UART_PRINT(calibratorGetTxParamsUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorGetTxParamsDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorStartRxUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorStartRxActionStr);
    UART_PRINT(calibratorStartRxUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorStartRxFirstDetailsStr);
    UART_PRINT(calibratorStartRxSecondDetailsStr);
    UART_PRINT(calibratorStartRx_sourceMac_optionDetailsStr);
    UART_PRINT(calibratorStartRx_ackEnable_optionDetailsStr);
    UART_PRINT(calibratorStartRx_aid_optionDetailsStr);
    UART_PRINT(calibratorStartRx_rate_optionFirstDetailsStr);
    UART_PRINT(calibratorStartRx_rate_optionSecondDetailsStr);
    UART_PRINT(calibratorStartRx_preambleType_optionFirstDetailsStr);
    UART_PRINT(calibratorStartRx_preambleType_optionSecondDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorStopRxUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorStopRxActionStr);
    UART_PRINT(calibratorStopRxUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorStopRxDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorGetRxStatsUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorGetRxStatsActionStr);
    UART_PRINT(calibratorGetRxStatsUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorGetRxStatsDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorRateOverrideUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorRateOverrideActionStr);
    UART_PRINT(calibratorRateOverrideUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorRateOverrideDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorBAEnableUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetBAEnableActionStr);
    UART_PRINT(calibratorBAsessionUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorBaSessionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorLinkAdaptUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorLinkAdaptActionStr);
    UART_PRINT(calibratorLinkAdaptationUsageStr_1);
    UART_PRINT(calibratorLinkAdaptationUsageStr_2);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorLinkAdaptationDetailsStr_1);
    UART_PRINT(calibratorLinkAdaptationDetailsStr_2);
    UART_PRINT(calibratorLinkAdaptationDetailsStr_3);
    UART_PRINT(calibratorLinkAdaptationDetailsStr_4);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorGetBeaconRssiUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorGetBeaconRssiActionStr);
    UART_PRINT(calibratorGetBeaconRssiUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorGetBeaconRssiDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetGiLtfUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetGiLtfActionStr);
    UART_PRINT(calibratorSetGiLtfUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetGiLtfDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetManualCalibUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetManualCalibActionStr);
    UART_PRINT(calibratorSetManualCalibUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetManualCalibDetailStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

}

void printCalibratorSetUplinkMuUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetUplinkMuActionStr);
    UART_PRINT(calibratorSetUplinkMuUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetUplinkMuDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetOperationModeControlUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetOperationModeControlActionStr);
    UART_PRINT(calibratorSetOperationModeControlUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetOperationModeControlDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetMcsRateUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetMcsRateActionStr);
    UART_PRINT(calibratorSetMcsRateUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetMcsRateDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetUplinkMuDataUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetUplinkMuDataActionStr);
    UART_PRINT(calibratorSetUplinkMuDataUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetUplinkMuDataDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}


void printCalibratorSetPsmUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetPsmActionStr);
    UART_PRINT(calibratorSetPsmUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetPsmDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}

void printCalibratorSetPowerHeaderUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetUplinkPowerHeaderActionStr);
    UART_PRINT(calibratorSetPowerHeaderUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetPowerHeaderDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}


void printCalibratorSetTransmitOmiUsage()
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(calibratorSetTransmitOMIActionStr);
    UART_PRINT(calibratorSetTransmitOmiUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(calibratorSetTransmitOmiDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
}


/*************************************************
 *          Calibrator callback zone             *
 *************************************************/

/*!
    \brief          Calibrator command.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa
 */
int32_t cmdCalibratorCallback(void *arg)
{
    int32_t ret = 0;
    CalibratorAction_e calibratorAction;
    CalibratorCommandWrapper_t calibratorCmdWrapper;


    ret = parseCalibratorAction(arg, &calibratorAction);

    if (ret < 0)
    {
        printCalibratorUsage(arg);
        return -1;
    }

    calibratorCmdWrapper.calibratorAction = calibratorAction;

    arg = trimLeadingWhitespace(arg); /* Note that arg is changed */

    switch (calibratorAction)
    {
    case (CALIBRATOR_ACTION_SET_POWER_MODE):
    {
        CalibratorPowerModes_e requestedCalibratorMode = {0};

        arg += strlen(calibratorSetPowerModeActionStr);
        ret = parseCalibratorSetPowerMode(arg, &requestedCalibratorMode);

        if (ret < 0)
        {
            printCalibratorPowerModeUsage();
            return ret;
        }
        if (PLT_ON == requestedCalibratorMode)
        {
            ret = Wlan_RoleUp(ROLE_TRANSCEIVER, NULL, 0);
        }
        else if (PLT_OFF == requestedCalibratorMode)
        {
            ret = Wlan_RoleDown(ROLE_TRANSCEIVER, 0);
        }
    }
        break;
    case (CALIBRATOR_ACTION_CHANNEL_TUNE):
    {
        CalibratorChannelTuneParams_t tuneParams = {0};

        arg += strlen(calibratorChannelTuneActionStr);
        ret = parseCalibratorChannelTune(arg, &tuneParams);

        if (ret < 0)
        {
            printCalibratorTuneChannelUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&tuneParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_CHANNEL_TUNE, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_TX_START):

        arg += strlen(calibratorStartTxActionStr);
        ret = ParseCmd(arg);
        if (ret < 0)
        {
            printCalibratorStartTxUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = NULL;

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_TX_START, &calibratorCmdWrapper);
        break;
    case (CALIBRATOR_ACTION_TX_TONE_START):
        {
            CalibratorStartTxToneParams_t startTxToneParams = {0, 0, 0};
    
            arg += strlen(calibratorStartTxToneActionStr);
            ret = parseCalibratorStartTxTone(arg, &startTxToneParams);
            if (ret < 0)
            {
                printCalibratorStartTxToneUsage();
                return ret;
            }
    
            calibratorCmdWrapper.calibratorCommandParams = (void *)(&startTxToneParams);
    
            ret = Wlan_Set(WLAN_SET_CALIBRATOR_TX_TONE_START, &calibratorCmdWrapper);
        }
            break;    
    case (CALIBRATOR_ACTION_TX_STOP):

        arg += strlen(calibratorStopTxActionStr);
        ret = ParseCmd(arg);
        if (ret < 0)
        {
            printCalibratorStopTxUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = NULL;

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_TX_STOP, &calibratorCmdWrapper);
        break;
    case (CALIBRATOR_ACTION_SET_TX_PARAMS):
    {
        CalibratorStartTxParams_t setTxParams = {0}; /* Parameters for start, set, set_tb tx are the same */

        arg += strlen(calibratorSetTxParamsActionStr);

        ret = parseCalibratorSetTx(arg, &setTxParams);
        if (ret < 0)
        {
            printCalibratorSetTxUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&setTxParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_TX_PARAMS, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_SET_TB_TX_PARAMS):
    {
        CalibratorStartTxParams_t setTbTxParams = {0};

        arg += strlen(calibratorSetTbTxParamsActionStr);

        ret = parseCalibratorSetTbTx(arg, &setTbTxParams);

        if (ret < 0)
        {
            printCalibratorSetTbTxUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&setTbTxParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_TB_TX_PARAMS, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_GET_TX_PARAMS):
    {
        CalibratorStartTxParams_t getTxParams = {0};

        arg += strlen(calibratorGetTxParamsActionStr);
        ret = ParseCmd(arg);
        if (ret < 0)
        {
            printCalibratorGetTxParamsUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&getTxParams);

        ret = Wlan_Get(WLAN_GET_CALIBRATOR_TX_PARAMS, &calibratorCmdWrapper);
        if (ret < 0)
        {
            Report("\r\nFailure at getting tx parameters");
            return -1;
        }

        calibratorDisplayTxParams(&getTxParams);
    }
        break;
    case (CALIBRATOR_ACTION_START_RX):
    {
        CalibratorStartRxParams_t startRxParams = {0};

        arg += strlen(calibratorStartRxActionStr);
        ret = parseCalibratorStartRx(arg, &startRxParams);
        if (ret < 0)
        {
            printCalibratorStartRxUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&startRxParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_START_RX, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_STOP_RX):

        arg += strlen(calibratorStopRxActionStr);
        ret = ParseCmd(arg);
        if (ret < 0)
        {
            printCalibratorStopRxUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = NULL;

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_STOP_RX, &calibratorCmdWrapper);
        break;
    case (CALIBRATOR_ACTION_GET_RX_STATS):
    {
        CalibratorGetRxParams_t getRxParams = {0};

        arg += strlen(calibratorGetRxStatsActionStr);
        ret = ParseCmd(arg);
        if (ret < 0)
        {
            printCalibratorGetRxStatsUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&getRxParams);

        ret = Wlan_Get(WLAN_GET_CALIBRATOR_RX_STATS, &calibratorCmdWrapper);
        if (ret < 0)
        {
            Report("\r\nFailure at getting rx parameters, try to start rx run");
            return -1;
        }

        calibratorDisplayRxStats(&getRxParams);
    }
        break;
    case (CALIBRATOR_ACTION_BA_SESSION):
    {
        CalibratorBASessionparams_t calibratorCmdParam = {0};

        arg += strlen(calibratorSetBAEnableActionStr);
        ret = parseCalibratorBAEnable(arg, &calibratorCmdParam);
        if (ret < 0)
        {
            printCalibratorBAEnableUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&calibratorCmdParam);
        while((ret = Wlan_Set(WLAN_SET_CALIBRATOR_BA_SESSION, &calibratorCmdWrapper))==WLAN_RET_OPER_IN_PROGRESS);
        if (ret < 0)
        {
            Report("\r\nFailure at getting rx parameters, try to start rx run");
            return -1;
        }

    }
    break;

    case (CALIBRATOR_ACTION_LINK_ADAPT):
    {
        CalibratorLinkAdaptParams_t calibratorCmdParam = {0};

        arg += strlen(calibratorLinkAdaptActionStr);
        ret = parseCalibratorLinkAdapt(arg, &calibratorCmdParam);
        if (ret < 0)
        {
            printCalibratorLinkAdaptUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&calibratorCmdParam);
        while((ret = Wlan_Set(WLAN_SET_CALIBRATOR_LINK_ADAPT, &calibratorCmdWrapper))==WLAN_RET_OPER_IN_PROGRESS);
        if (ret < 0)
        {
            Report("\r\nFailure at getting rx parameters, try to start rx run");
            return -1;
        }

    }
    break;

    case (CALIBRATOR_ACTION_RATE_OVERRIDE):
    {
        CalibratorRateOverrideParams_t rateOverrideParams = {0};

        arg += strlen(calibratorRateOverrideActionStr);

        ret = parseCalibratorRateOverride(arg, &rateOverrideParams);
        if (ret < 0)
        {
            printCalibratorRateOverrideUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&rateOverrideParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_RATE_OVERRIDE, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_GET_BEACON_RSSI):
    {
        CalibratorBeaconRssiParams_t beaconRssiParams = {0};

        arg += strlen(calibratorGetBeaconRssiActionStr);

        ret = ParseCmd(arg);
        if (ret < 0)
        {
            printCalibratorGetBeaconRssiUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&beaconRssiParams);

        ret = Wlan_Get(WLAN_GET_CALIBRATOR_BEACON_RSSI, &calibratorCmdWrapper);
        if (ret < 0)
        {
            Report("\r\nFailure at getting last beacon RSSI value");
            return -1;
        }

        if (beaconRssiParams.beaconRssi)
        {
            Report("\r\nBeacon RSSI: -%d\r\n", 1 + (uint8_t)(~ beaconRssiParams.beaconRssi));
        }
        else
        {
            Report("\r\nBeacon RSSI: Not connected!\r\n");
        }
    }
        break;
    case (CALIBRATOR_ACTION_SET_GI_LTF):
    {
        CalibratorGiLtfModeParams_t giLtfParams = {0};

        arg += strlen(calibratorSetGiLtfActionStr);

        ret = parseCalibratorsetGiLtf(arg, &giLtfParams);
        if (ret < 0)
        {
            printCalibratorSetGiLtfUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&giLtfParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_GI_LTF, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_SET_UPLINK_MU):
    {
        CalibratorUplinkMuParams_t uplinkMuParams = {0};

        arg += strlen(calibratorSetUplinkMuActionStr);

        ret = parseCalibratorSetUplinkMu(arg, &uplinkMuParams);
        if (ret < 0)
        {
            printCalibratorSetUplinkMuUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&uplinkMuParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_UPLINK_MU, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_SET_OPERATION_MODE_CONTROL):
    {
        CalibratorOperationModeControlParams_t operationModeControlParams = {0};

        arg += strlen(calibratorSetOperationModeControlActionStr);

        ret = parseCalibratorSetOperationModeControl(arg, &operationModeControlParams);
        if (ret < 0)
        {
            printCalibratorSetOperationModeControlUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&operationModeControlParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_OPERATION_MODE_CONTROL, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_SET_MCS_RATE):
    {
        CalibratorSetMcsRateParams_t setMcsRateParams = {0};

        arg += strlen(calibratorSetMcsRateActionStr);

        ret = parseCalibratorSetMcsRate(arg, &setMcsRateParams);
        if (ret < 0)
        {
            printCalibratorSetMcsRateUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&setMcsRateParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_MCS_RATE, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_SET_UPLINK_MU_DATA):
    {
        CalibratorSetUplinkMuDataParams_t setUplinkMuDataParams = {0};

        arg += strlen(calibratorSetUplinkMuDataActionStr);

        ret = parseCalibratorSetUplinkMuData(arg, &setUplinkMuDataParams);
        if (ret < 0)
        {
            printCalibratorSetUplinkMuDataUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&setUplinkMuDataParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_UPLINK_MU_DATA, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_MANUAL_CALIBRATION):
    {
        uint16_t calibration_bitmap = {0}; /* Parameters for start, set, set_tb tx are the same */

        arg += strlen(calibratorSetManualCalibActionStr);

        ret = parseCalibratorSetManualCalib(arg, &calibration_bitmap);
        if (ret < 0)
        {
            printCalibratorSetManualCalibUsage();
            return ret;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&calibration_bitmap);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_MANUAL_CALIBRATION, &calibratorCmdWrapper);
    }
    break;

    case (CALIBRATOR_ACTION_SET_PSM):
    {
        CalibratorSetPsmParams_t setPsmParams = {0};

        arg += strlen(calibratorSetPsmActionStr);

        ret = parseCalibratorSetPsm(arg, &setPsmParams);
        if (ret < 0)
        {
            printCalibratorSetPsmUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&setPsmParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_PSM, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_SET_UPLINK_POWER_HEADER):
    {
        CalibratorSetPowerHeaderParams_t setPowerHeaderParams = {0};

        arg += strlen(calibratorSetUplinkPowerHeaderActionStr);

        ret = parseCalibratorSetPowerHeader(arg, &setPowerHeaderParams);
        if (ret < 0)
        {
            printCalibratorSetPowerHeaderUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&setPowerHeaderParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_UPLINK_POWER_HEADER, &calibratorCmdWrapper);
    }
        break;
    case (CALIBRATOR_ACTION_SET_TRANSMIT_OMI):
    {
        CalibratorSetTransmitOmiParams_t setTransmitOmiParams = {0};

        arg += strlen(calibratorSetTransmitOMIActionStr);

        ret = parseCalibratorSetTransmitOmi(arg, &setTransmitOmiParams);
        if (ret < 0)
        {
            printCalibratorSetTransmitOmiUsage();
            return -1;
        }

        calibratorCmdWrapper.calibratorCommandParams = (void *)(&setTransmitOmiParams);

        ret = Wlan_Set(WLAN_SET_CALIBRATOR_TRANSMIT_OSI, &calibratorCmdWrapper);
    }
        break;
    default:
        break;
    }

    if (ret < 0)
    {
        Report("\r\nFailed executing calibrator command");
    }

    return ret;
}
