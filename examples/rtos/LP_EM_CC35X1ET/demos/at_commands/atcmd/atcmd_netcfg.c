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

#include "str_mpl.h"

#include "atcmd_netcfg.h"
#include "atcmd_defs.h"
#include "atcmd_event.h"
#include "atcmd_gen.h"
#include "atcmd.h"

#include "wlan_if.h"
#include "network_terminal.h"
#include "cmd_parser.h"
#include "network_lwip.h"
#include "lwip/sockets.h"
#include "dhcpserver.h"
#include "lwip_ping.h"

//*****************************************************************************
// defines
//*****************************************************************************

//*****************************************************************************
// typedefs
//*****************************************************************************

typedef struct _ATCmdNetCfgIpParams_t {
    WlanRole_e  roleType;
    uint32_t    ipMode;
    uint32_t    ipAddress;
    uint32_t    netmask;
    uint32_t    gateway;
    uint32_t    dhcp;
    Bool_e      setDhcpServerAddress;
} ATCmdNetCfgIpParams_t;

typedef struct _ATCmdNetCfgDhcpParams_t {
    uint32_t    leaseTime;
    uint32_t    startAddress;
    uint32_t    endAddress;
} ATCmdNetCfgDhcpParams_t;

#if 0

typedef struct _ATCmdNetcfg_t_
{
    uint16_t 	id;
    uint16_t 	option;
    uint16_t 	len; 
    uint8_t     *value;
}ATCmdNetcfg_t;

#endif

//*****************************************************************************
// globals and externs
//*****************************************************************************
extern uint32_t ActiveNetIfBitMap;

//*****************************************************************************
// AT Command Netcfg Routines
//*****************************************************************************

#if 0

/*!
    \brief          Free allocated memory

    \param          params       -   Points to buffer for deallocate.

    \return         Upon successful completion, the function shall return 0.

*/
int32_t ATCmdNetcfg_setFree(ATCmdNetcfg_t *params)
{    
    if (params->value != NULL)
    {
        free(params->value);
    }  
    return 0;
}

/*!
    \brief          Parse command.

    \param          buff       -   Points to command line buffer.
    \param          params     -   Points to create socket struct.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_setParse(char *buff, ATCmdNetcfg_t *params)
{
    int32_t ret = 0;
    uint8_t optional = 0;
    uint8_t ip[4];
    
    /* ID */
    if ((ret = StrMpl_getListVal(ATCmd_netcfgId, sizeof(ATCmd_netcfgId)/sizeof(StrMpl_List_t), &buff, &params->id, ATCMD_DELIM_ARG, STRMPL_FLAG_PARAM_SIZE_16 )) < 0)
    {
        return ret;
    }

    if ((params->id == SL_NETCFG_MAC_ADDRESS_SET) || 
        (params->id == SL_NETCFG_AP_STATION_DISCONNECT) || 
        (params->id == SL_NETCFG_IPV4_DNS_CLIENT))
    {
        optional = 1;
    }
    /* Option */
    if ((ret = StrMpl_getListVal(ATCmd_netcfgOption, sizeof(ATCmd_netcfgOption)/sizeof(StrMpl_List_t), &buff, &params->option, ATCMD_DELIM_ARG, STRMPL_FLAG_PARAM_SIZE_16 )) < 0)
    {
        if ((ret != STRMPL_ERROR_PARAM_MISSING) || (optional == 0))
        {
            return ret;
        }
    }

    /* value */
    switch (params->id)
    {
        case SL_NETCFG_MAC_ADDRESS_SET:
        case SL_NETCFG_AP_STATION_DISCONNECT:
            params->option = 1;
            params->value = malloc(SL_WLAN_BSSID_LENGTH);
            if (params->value == NULL)
            {
                return -1;
            }
            if ((ret = StrMpl_getArrayVal(&buff, (void *)params->value, SL_WLAN_BSSID_LENGTH,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_SIZE_8,ATCmd_excludeDelimArray)) < 0)
            {
                return ret;        
            }
            params->len = SL_WLAN_BSSID_LENGTH;
            break;
        case SL_NETCFG_IF:
            if (params->option == SL_NETCFG_IF_STATE)
            {
                params->len = sizeof(uint32_t);
                params->value = malloc(params->len);
                if (params->value == NULL)
                {
                    return -1;
                }
                if ((ret = StrMpl_getBitmaskListVal(ATCmd_netcfgIfState, sizeof(ATCmd_netcfgIfState)/sizeof(StrMpl_List_t), &buff, (void *)params->value, ATCMD_DELIM_TRM, ATCMD_DELIM_BIT, ATCmd_excludeDelimArray, STRMPL_FLAG_PARAM_SIZE_32)) < 0)
                {
                    if (ret != STRMPL_ERROR_PARAM_MISSING)
                    {
                        return ret;
                    }
                    ret = 0;
                }
            }
            else
            {
                return -1;
            }
            break;
        case SL_NETCFG_IPV4_STA_ADDR_MODE:
        case SL_NETCFG_IPV4_AP_ADDR_MODE:
            switch (params->option)
            {
                case SL_NETCFG_ADDR_STATIC:
                    params->value = malloc(sizeof(SlNetCfgIpV4Args_t));
                    if (params->value == NULL)
                    {
                        return -1;
                    }
                    /* ip address */
                    if ((ret = StrMpl_getArrayVal(&buff,(void *)ip,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8 ,ATCmd_excludeDelimArray)) < 0)
                    {
                	    return ret;
                    }
                    ((SlNetCfgIpV4Args_t *)(params->value))->Ip = SL_IPV4_VAL(ip[0],ip[1],ip[2],ip[3]);
                    /* Subnet mask */
                    if ((ret = StrMpl_getArrayVal(&buff,(void *)ip,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8 ,ATCmd_excludeDelimArray)) < 0)
                    {
                	    return ret;
                    }
                    ((SlNetCfgIpV4Args_t *)(params->value))->IpMask = SL_IPV4_VAL(ip[0],ip[1],ip[2],ip[3]);
                    /* Default gateway address */
                    if ((ret = StrMpl_getArrayVal(&buff,(void *)ip,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8 ,ATCmd_excludeDelimArray)) < 0)
                    {
                	    return ret;
                    }
                    ((SlNetCfgIpV4Args_t *)(params->value))->IpGateway = SL_IPV4_VAL(ip[0],ip[1],ip[2],ip[3]);
                    /* DNS server address */
                    if ((ret = StrMpl_getArrayVal(&buff,(void *)ip,4,ATCMD_DELIM_TRM,ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8 ,ATCmd_excludeDelimArray)) < 0)
                    {
                	    return ret;
                    }
                    ((SlNetCfgIpV4Args_t *)(params->value))->IpDnsServer = SL_IPV4_VAL(ip[0],ip[1],ip[2],ip[3]);
                    params->len = sizeof(SlNetCfgIpV4Args_t);
                    break;
                case SL_NETCFG_ADDR_DHCP:
                case SL_NETCFG_ADDR_DHCP_LLA:
                case SL_NETCFG_ADDR_RELEASE_IP_SET:
                case SL_NETCFG_ADDR_RELEASE_IP_OFF:
                    if (params->id == SL_NETCFG_IPV4_AP_ADDR_MODE)
                    {
                        return -1;
                    }
                    params->len = 0;
                    break;
                default:
                    return -1;
            }
            break;
        case SL_NETCFG_IPV6_ADDR_LOCAL:
            switch (params->option)
            {
                case SL_NETCFG_ADDR_STATIC:
                    params->value = malloc(sizeof(SlNetCfgIpV6Args_t));
                    if (params->value == NULL)
                    {
                        return -1;
                    }
                    /* ip address */
                    if ((ret = StrMpl_getArrayVal(&buff, (void *)((SlNetCfgIpV6Args_t *)(params->value))->Ip,4,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_SIZE_32,ATCmd_excludeDelimArray)) < 0)
                    {
                        return ret;
                    }
                    params->len = sizeof(SlNetCfgIpV6Args_t);
                    break;
                case SL_NETCFG_ADDR_STATEFUL:
                case SL_NETCFG_ADDR_STATELESS:
                    params->len = 0;
                    break;
                default:
                    return -1;
            }
            break;
        case SL_NETCFG_IPV6_ADDR_GLOBAL:
            switch (params->option)
            {
                case SL_NETCFG_ADDR_STATIC:
                    params->value = malloc(sizeof(SlNetCfgIpV6Args_t));
                    if (params->value == NULL)
                    {
                        return -1;
                    }
                    /* ip address */
                    if ((ret = StrMpl_getArrayVal(&buff, (void *)((SlNetCfgIpV6Args_t *)(params->value))->Ip,4,ATCMD_DELIM_ARG,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_SIZE_32,ATCmd_excludeDelimArray)) < 0)
                    {
                        return ret;
                    }
                    /* DNS server address */
                    if ((ret = StrMpl_getArrayVal(&buff, (void *)((SlNetCfgIpV6Args_t *)(params->value))->IpDnsServer,4,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_SIZE_32,ATCmd_excludeDelimArray)) < 0)
                    {
                        return ret;
                    }
                    params->len = sizeof(SlNetCfgIpV6Args_t);
                    break;
                case SL_NETCFG_ADDR_STATEFUL:
                case SL_NETCFG_ADDR_STATELESS:
                    params->len = 0;
                    break;
                default:
                    return -1;
            }
            break;
        case SL_NETCFG_IPV4_DNS_CLIENT:
            params->value = malloc(sizeof(SlNetCfgIpV4DnsClientArgs_t));
            if (params->value == NULL)
            {
                return -1;
            }
            params->option = 0;
            /* DNS server address */
            if ((ret = StrMpl_getArrayVal(&buff,(void *)ip,4,ATCMD_DELIM_TRM,ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8 ,ATCmd_excludeDelimArray)) < 0)
            {
        	    return ret;
            }
            ((SlNetCfgIpV4DnsClientArgs_t *)(params->value))->DnsSecondServerAddr = SL_IPV4_VAL(ip[0],ip[1],ip[2],ip[3]);
            params->len = sizeof(SlNetCfgIpV4DnsClientArgs_t);
            break;

        default:
            return -1;
    }

    return ret;
}

/*!
    \brief          Netcfg Set callback.

    This routine sets network configurations

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_setCallback(void *arg)
{
    int32_t ret = 0;
    ATCmdNetcfg_t params;

    memset(&params, 0x0, sizeof(ATCmdNetcfg_t));
    
    /* Call the command parser */
    ret = ATCmdNetcfg_setParse((char *)arg, &params);

    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr,ret);
        ATCmdNetcfg_setFree(&params);
	    return -1;
    }
    
    /* set netcfg option */
    ret = sl_NetCfgSet(params.id,params.option,params.len,params.value);

    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr,ret);
    }
    else
    {
        ATCmd_okResult();
    }

    ATCmdNetcfg_setFree(&params);
    return ret;
}


/*!
    \brief          Free allocated memory

    \param          params       -   Points to buffer for deallocate.

    \return         Upon successful completion, the function shall return 0.

*/
int32_t ATCmdNetcfg_getFree(ATCmdNetcfg_t *params)
{    
    if (params != NULL)
    {
        if (params->value != NULL)
        {
            free(params->value);
        }
        free(params);
    }
    return 0;
}

/*!
    \brief          Parse command.

    \param          buff       -   Points to command line buffer.
    \param          params     -   Points to create socket struct.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_getParse(char *buff, ATCmdNetcfg_t *params)
{
    int32_t ret = 0;
    
    /* ID */
    if ((ret = StrMpl_getListVal(ATCmd_netcfgId, sizeof(ATCmd_netcfgId)/sizeof(StrMpl_List_t), &buff, &params->id, ATCMD_DELIM_TRM, STRMPL_FLAG_PARAM_SIZE_16 )) < 0)
    {
        return ret;
    }

    /* value */
    switch (params->id)
    {
        case SL_NETCFG_MAC_ADDRESS_GET:
            params->len = SL_WLAN_BSSID_LENGTH;
            break;
        case SL_NETCFG_IF:
            params->len = sizeof(uint32_t);
            break;
        case SL_NETCFG_IPV4_STA_ADDR_MODE:
        case SL_NETCFG_IPV4_AP_ADDR_MODE:
            params->len = sizeof(SlNetCfgIpV4Args_t);
            break;
        case SL_NETCFG_IPV6_ADDR_LOCAL:
        case SL_NETCFG_IPV6_ADDR_GLOBAL:
            params->len = sizeof(SlNetCfgIpV6Args_t);
            break;
        case SL_NETCFG_IPV4_DHCP_CLIENT:
            params->len = sizeof(SlNetCfgIpv4DhcpClient_t);
            break;
        case SL_NETCFG_IPV4_DNS_CLIENT:
            params->len = sizeof(SlNetCfgIpV4DnsClientArgs_t);
            break;
        case SL_NETCFG_AP_STATIONS_NUM_CONNECTED:
            params->len = sizeof(uint8_t);
            break;
        case SL_NETCFG_AP_STATIONS_INFO_LIST:
            params->len = sizeof(SlNetCfgStaInfo_t) * 4;
            break;

        default:
            return -1;
    }
    params->value = malloc(params->len);
    if (params->value == NULL)
    {
        return -1;
    }

    return ret;
}

/*!
    \brief          Return result.

    \param          arg       -   Points to list of arguments
                    buff      -   points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdNetcfg_getResult(void *args, int32_t num, char *buff)
{    
    int32_t ret = 0;
    ATCmdNetcfg_t *params = (ATCmdNetcfg_t *)args;
    uint8_t ip[4];
    uint8_t i, stationListSize;
    char delim;

    StrMpl_setStr(ATCmd_netcfgGetStr,&buff,ATCMD_DELIM_EVENT);

    switch (params->id)
    {
        case SL_NETCFG_MAC_ADDRESS_GET:
            StrMpl_setArrayVal(params->value,&buff,SL_WLAN_BSSID_LENGTH,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        case SL_NETCFG_IF:
            StrMpl_setBitmaskListStr(ATCmd_netcfgIfState, sizeof(ATCmd_netcfgIfState)/sizeof(StrMpl_List_t), params->value, &buff, ATCMD_DELIM_TRM, ATCMD_DELIM_BIT, STRMPL_FLAG_PARAM_SIZE_32);
            break;
        case SL_NETCFG_IPV4_STA_ADDR_MODE:
        case SL_NETCFG_IPV4_AP_ADDR_MODE:
            StrMpl_setListStr(ATCmd_netcfgOption, sizeof(ATCmd_netcfgOption)/sizeof(StrMpl_List_t), &params->option,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_16|STRMPL_FLAG_PARAM_UNSIGNED);
            /* address */
            ATCmd_valToIPv4(((SlNetCfgIpV4Args_t *)(params->value))->Ip, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Subnet mask */
            ATCmd_valToIPv4(((SlNetCfgIpV4Args_t *)(params->value))->IpMask, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Gateway */
            ATCmd_valToIPv4(((SlNetCfgIpV4Args_t *)(params->value))->IpGateway, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* DNS */
            ATCmd_valToIPv4(((SlNetCfgIpV4Args_t *)(params->value))->IpDnsServer, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_TRM,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        case SL_NETCFG_IPV6_ADDR_LOCAL:
        case SL_NETCFG_IPV6_ADDR_GLOBAL:
            StrMpl_setListStr(ATCmd_netcfgOption, sizeof(ATCmd_netcfgOption)/sizeof(StrMpl_List_t), &params->option,&buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_16|STRMPL_FLAG_PARAM_UNSIGNED);
            /* address */
            StrMpl_setArrayVal(((SlNetCfgIpV6Args_t *)(params->value))->Ip,&buff,4,ATCMD_DELIM_TRM,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_32 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        case SL_NETCFG_IPV4_DHCP_CLIENT:
            /* address */
            ATCmd_valToIPv4(((SlNetCfgIpv4DhcpClient_t *)(params->value))->Ip, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Subnet mask */
            ATCmd_valToIPv4(((SlNetCfgIpv4DhcpClient_t *)(params->value))->Mask, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Gateway */
            ATCmd_valToIPv4(((SlNetCfgIpv4DhcpClient_t *)(params->value))->Gateway, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* DNS 1 */
            ATCmd_valToIPv4(((SlNetCfgIpv4DhcpClient_t *)(params->value))->Dns[0], ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* DNS 2 */
            ATCmd_valToIPv4(((SlNetCfgIpv4DhcpClient_t *)(params->value))->Dns[1], ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* DHCP Server */
            ATCmd_valToIPv4(((SlNetCfgIpv4DhcpClient_t *)(params->value))->DhcpServer, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            /* Lease Time */
            StrMpl_setVal(&((SlNetCfgIpv4DhcpClient_t *)(params->value))->LeaseTime, &buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_32 |STRMPL_FLAG_PARAM_UNSIGNED|STRMPL_FLAG_PARAM_DEC);
            /* Time To Renew */
            StrMpl_setVal(&((SlNetCfgIpv4DhcpClient_t *)(params->value))->TimeToRenew, &buff,ATCMD_DELIM_ARG,STRMPL_FLAG_PARAM_SIZE_32 |STRMPL_FLAG_PARAM_UNSIGNED|STRMPL_FLAG_PARAM_DEC);
            /* Dhcp State */
            StrMpl_setListStr(ATCmd_netcfgDhcpState, sizeof(ATCmd_netcfgDhcpState)/sizeof(StrMpl_List_t), &((SlNetCfgIpv4DhcpClient_t *)(params->value))->DhcpState,&buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_8|STRMPL_FLAG_PARAM_UNSIGNED);
            break;

        case SL_NETCFG_IPV4_DNS_CLIENT:
            /* DNS */
            ATCmd_valToIPv4(((SlNetCfgIpV4DnsClientArgs_t *)(params->value))->DnsSecondServerAddr, ip);
            StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_TRM,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
            break;
        case SL_NETCFG_AP_STATIONS_NUM_CONNECTED:
            StrMpl_setVal(params->value, &buff,ATCMD_DELIM_TRM,STRMPL_FLAG_PARAM_SIZE_8 |STRMPL_FLAG_PARAM_UNSIGNED|STRMPL_FLAG_PARAM_DEC);
            break;
        case SL_NETCFG_AP_STATIONS_INFO_LIST:
            stationListSize = params->len / sizeof(SlNetCfgStaInfo_t);
            delim = ATCMD_DELIM_LIST;
            for (i=0; i<stationListSize; i++)
            {
                if (i == (stationListSize - 1))
                {
                    delim = ATCMD_DELIM_TRM;
                }
                /* address */
                ATCmd_valToIPv4(((SlNetCfgStaInfo_t *)(params->value))->Ip, ip);
                StrMpl_setArrayVal(ip,&buff,4,ATCMD_DELIM_ARG,ATCMD_DELIM_INTER,STRMPL_FLAG_PARAM_DEC | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
                /* MAC address */
                StrMpl_setArrayVal(((SlNetCfgStaInfo_t *)(params->value))->MacAddr,&buff,SL_WLAN_BSSID_LENGTH,ATCMD_DELIM_ARG,ATCMD_DELIM_ARRAY,STRMPL_FLAG_PARAM_HEX | STRMPL_FLAG_PARAM_SIZE_8 | STRMPL_FLAG_PARAM_UNSIGNED);
                /* name */
                StrMpl_setStr((char *)((SlNetCfgStaInfo_t *)(params->value))->Name,&buff,delim);
            }
            break;

        default:
            ret = -1;
    }
    ATCmdNetcfg_getFree(params);
    return ret;
}

/*!
    \brief          Netcfg Get callback.

    This routine gets network configurations

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_getCallback(void *arg)
{
    int32_t ret = 0;
    ATCmdNetcfg_t *params;
    uint16_t *pOption = NULL;
    
    params = malloc(sizeof(ATCmdNetcfg_t));

    if (params == NULL)
    {
        ATCmd_errorResult(ATCmd_errorAllocStr,0);
        return -1;     
    }
    memset(params, 0x0, sizeof(ATCmdNetcfg_t));
    
    /* Call the command parser */
    ret = ATCmdNetcfg_getParse((char *)arg, params);

    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr,ret);
        ATCmdNetcfg_getFree(params);
	    return -1;
    }
    if ((params->id != SL_NETCFG_IF) && (params->id != SL_NETCFG_AP_STATIONS_NUM_CONNECTED))
    {
        pOption = &params->option;
    }

    /* set netapp option */
    ret = sl_NetCfgGet(params->id,pOption,&params->len,params->value);

    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr,ret);
        ATCmdNetcfg_getFree(params);
    }
    else
    {
        ATCmd_commandResult(ATCmdNetcfg_getResult,params,0);
        ATCmd_okResult();
    }

    return ret;
}
#endif

/*!
    \brief          Parse set interface IP command.

    \param          arg          -   Points to command line buffer.
    \param          params       -   Points to user`s params.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;
*/
int32_t ATCmd_setIpModeParse(char *arg, ATCmdNetCfgIpParams_t *params)
{
    int32_t ret = 0;
    uint8_t ip[IPV4_ADDR_LEN] = {0};

    /* role type */
    ret = StrMpl_getListVal(ATCmd_wlanRoles,
                            sizeof(ATCmd_wlanRoles) / sizeof(StrMpl_List_t),
                            (char **)&arg, &params->roleType, ATCMD_DELIM_ARG,
                            STRMPL_FLAG_PARAM_SIZE_8);
    if (ret < 0)
    {
        return ret;
    }

    /* ip address mode */
    ret = StrMpl_getListVal(ATCmd_netcfgIpMode,
                            sizeof(ATCmd_netcfgIpMode) / sizeof(StrMpl_List_t),
                            (char **)&arg, &params->ipMode, ATCMD_DELIM_ARG,
                            STRMPL_FLAG_PARAM_SIZE_8);

    /* if the ip mode is the last param, parse with different delimeter */
    if (ret == STRMPL_ERROR_DELIM_MISSING)
    {
        ret = StrMpl_getListVal(ATCmd_netcfgIpMode,
                                sizeof(ATCmd_netcfgIpMode)
                                / sizeof(StrMpl_List_t),
                                (char **)&arg, &params->ipMode,
                                ATCMD_DELIM_TRM,
                                STRMPL_FLAG_PARAM_SIZE_8);

        return ret;
    }
    else if (ret < 0)
    {
        return ret;
    }

    /* DHCP IP/netmask/gateway address can't be set for STA */
    if ((params->ipMode == IP_DHCP) &&
        (params->roleType == WLAN_ROLE_STA))
    {
        ret = -1;
    }

    /* IPv4 address */
    ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_ARG,
                             ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                             ATCmd_excludeDelimArray);
    if (ret < 0)
    {
        return ret;
    }
    params->ipAddress = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);

    /* netmask */
    ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_ARG,
                             ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                             ATCmd_excludeDelimArray);
    if (ret < 0)
    {
        return ret;
    }
    params->netmask = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);

    /* gateway */
    ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_TRM,
                             ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                             ATCmd_excludeDelimArray);
    if (ret < 0)
    {
        return ret;
    }
    params->gateway = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);

    /* Accept AP DHCP server parameters only if all 3 addresses were filled */
    if (params->ipMode == IP_DHCP)
    {
        params->setDhcpServerAddress = TRUE;
    }
  
    return ret;
}

/*!
    \brief          Netcfg Set Interface IP callback.

    This routine allows configuring an interface IP address.
    Whether it's an AP or STA and whether the IP is static or DHCP.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_setInterfaceIpCallback(void *arg)
{
    int32_t ret = 0;
    ATCmdNetCfgIpParams_t params = {0};

    ret = ATCmd_setIpModeParse((char *)arg, &params);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    if (params.roleType == WLAN_ROLE_STA)
    {
        if (params.ipMode == IP_DHCP)
        {
            network_stack_set_dynamic_ip_if_sta();
        }
        else if (params.ipMode == IP_STATIC)
        {
            network_stack_set_static_ip_if_sta(htonl(params.ipAddress),
                                               htonl(params.netmask),
                                               htonl(params.gateway));
        }
    }
    else if (params.roleType == WLAN_ROLE_AP)
    {
        if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
        {
            ret = -1;
            ATCmd_errorResult(ATCmd_errorApRoleNotUpStr, ret);
            return ret;
        }

        if (params.ipMode == IP_DHCP)
        {
            network_stack_set_dhcp_server_if_ap(0);

            if (params.setDhcpServerAddress)
            {
                network_stack_set_dynamic_ip_if_ap(htonl(params.ipAddress),
                                                   htonl(params.netmask),
                                                   htonl(params.gateway));
            }

            network_stack_set_dhcp_server_if_ap(1);
        }
        else if (params.ipMode == IP_STATIC)
        {
            network_stack_set_static_ip_if_ap(htonl(params.ipAddress),
                                              htonl(params.netmask),
                                              htonl(params.gateway));
        }
    }

    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Return result.

    \param          arg       -   Points to list of arguments.
                    num       -   Token which can be used for this callback.
                    buff      -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdNetcfg_getInterfaceIpResults(void *args, int32_t num, char *buff)
{    
    int32_t ret = 0;
    ATCmdNetCfgIpParams_t *params = (ATCmdNetCfgIpParams_t *)args;
    char hostAddress[INET_ADDRSTRLEN] = {0};
    const char *addr = NULL;

    StrMpl_setStr(ATCmd_netcfgGetInterfaceIpStr, &buff, ATCMD_DELIM_EVENT);

    addr = inet_ntop(AF_INET, &params->ipAddress,
                     hostAddress, INET_ADDRSTRLEN);
    if (addr == NULL)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        os_free(params);
        return -1;
    }
    StrMpl_setStr(hostAddress, &buff, ATCMD_DELIM_ARG);

    addr = inet_ntop(AF_INET, &params->netmask,
                     hostAddress, INET_ADDRSTRLEN);
    if (addr == NULL)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        os_free(params);
        return -1;
    }
    StrMpl_setStr(hostAddress, &buff, ATCMD_DELIM_ARG);

    addr = inet_ntop(AF_INET, &params->gateway,
                     hostAddress, INET_ADDRSTRLEN);
    if (addr == NULL)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        os_free(params);
        return -1;
    }
    StrMpl_setStr(hostAddress, &buff, ATCMD_DELIM_ARG);

    if (params->dhcp)
    {
        StrMpl_setStr("DHCP", &buff, ATCMD_DELIM_TRM);
    }
    else
    {
        StrMpl_setStr("STATIC", &buff, ATCMD_DELIM_TRM);
    }

    os_free(params);
    return ret;
}

/*!
    \brief          Parse get interface IP command.

    \param          arg          -   Points to command line buffer.
    \param          roleType     -   Points to requested role type.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;
*/
int32_t ATCmdNetcfg_getInterfaceIpParse(char *arg, WlanRole_e *roleType)
{
    int32_t ret = 0;

    /* role type */
    ret = StrMpl_getListVal(ATCmd_wlanRoles,
                            sizeof(ATCmd_wlanRoles) / sizeof(StrMpl_List_t),
                            (char **)&arg, roleType, ATCMD_DELIM_TRM,
                            sizeof(*roleType));
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
    }

    return ret;
}

/*!
    \brief          Netcfg get interface IP callback.

    This routine gets network configurations of a specific
    requested interface.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_getInterfaceIpCallback(void *arg)
{
    int32_t ret = 0;
    WlanRole_e roleType;
    uint32_t ipAddress = 0, netmask = 0, gateway = 0, dhcp = 0;
    ATCmdNetCfgIpParams_t *params = NULL;

    ret = ATCmdNetcfg_getInterfaceIpParse((char *)arg, &roleType);
    if (ret < 0)
    {
        return ret;
    }

    params = os_malloc(sizeof(ATCmdNetCfgIpParams_t));
    if (params == NULL)
    {
        ATCmd_errorResult(ATCmd_errorAllocStr, 0);
        return -1;
    }
    os_memset(params, 0x0, sizeof(ATCmdNetCfgIpParams_t));

    ret = network_stack_get_if_ip(roleType,
                                  &ipAddress,
                                  &netmask,
                                  &gateway,
                                  &dhcp);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        os_free(params);
        return ret;
    }

    params->ipAddress = ipAddress;
    params->netmask = netmask;
    params->gateway = gateway;
    params->dhcp = dhcp;

    ATCmd_okResult();
    ATCmd_commandResult(ATCmdNetcfg_getInterfaceIpResults, params, 0);

    return ret;
}

/*!
    \brief          Parse set DHCP server command.

    \param          arg          -   Points to command line buffer.
    \param          leaseTime    -   Points to lease time.
    \param          startAddress -   Points to DHCP server start address.
    \param          endAddress   -   Points to DHCP server end address.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;
*/
int32_t ATCmdNetcfg_setDhcpServerParse(char *arg,
                                       uint32_t *leaseTime,
                                       uint32_t *startAddress,
                                       uint32_t *endAddress)
{
    int32_t ret = 0;
    uint8_t ip[IPV4_ADDR_LEN] = {0};

    char *tempPtr = arg;
    if (*tempPtr == '-')
    {
        ATCmd_errorResult(ATCmd_errorInvalidParamValueStr, ret);
        return STRMPL_ERROR_WRONG_PARAM;
    }
    /* lease time */
    ret = StrMpl_getVal(&arg, leaseTime,
                        ATCMD_DELIM_ARG, STRMPL_FLAG_PARAM_SIZE_32);
    if (ret == STRMPL_ERROR_PARAM_MISSING)
    {
        return ret;

    }
    if (*leaseTime == 0)
    {
        ATCmd_errorResult(ATCmd_errorInvalidParamValueStr, ret);
        return STRMPL_ERROR_WRONG_PARAM;
    }
    else if (ret < 0)
    {
        return ret;
    }

    /* IPv4 start address */
    ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_ARG,
                             ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                             ATCmd_excludeDelimArray);
    if (ret < 0)
    {
        return ret;
    }
    *startAddress = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);

    /* IPv4 end address */
    ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_TRM,
                             ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                             ATCmd_excludeDelimArray);
    if (ret < 0)
    {
        return ret;
    }
    *endAddress = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);

    return ret;
}

/*!
    \brief          Netcfg set DHCP server configurations callback.

    This routine allows setting the DHCP server configurations.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_setDhcpServerCallback(void *arg)
{
    int32_t ret = 0;
    uint32_t leaseTime = 0, startIp = 0, endIp = 0;
    uint32_t dhcp = 0;
    struct dhcps_lease lease = {0};

    /* In order to set DHCP server configurations AP role must
       be up and active, and hence DHCP server must be enabled. */
    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorApRoleNotUpStr, ret);
        return ret;
    }

    /* Check if DHCP is enabled */
    ret = network_stack_get_if_ip(WLAN_ROLE_AP,
                                  NULL, NULL, NULL,
                                  &dhcp);
    if (!dhcp || (ret < 0))
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        return -1;
    }

    ret = ATCmdNetcfg_setDhcpServerParse((char *)arg,
                                         &leaseTime,
                                         &startIp,
                                         &endIp);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    ret = network_stack_set_dhcp_server_if_ap(0);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        return ret;
    }

    lease.enable = TRUE;
    lease.start_ip.addr = htonl(startIp);
    lease.end_ip.addr = htonl(endIp);
    
    ret = wifi_softap_set_dhcps_lease(&lease);
    if (ret == TRUE) // Address config succeeded - now set lease time
    {
        ret = wifi_softap_set_dhcps_lease_time(leaseTime);
        if (ret == TRUE)  // Should always succeed (validated in parser)
        {
            Report("\n\rDHCP configuration applied successfully\n\r");
        }
    }

    else
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        Report("\n\rInvalid parameters - server running with previous settings\n\r");
    }

    ret = network_stack_set_dhcp_server_if_ap(1);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        return ret;
    }

    ATCmd_okResult();

    return ret;
}

/*!
    \brief          Return DHCP server configurations result.

    \param          arg       -   Points to list of arguments.
                    num       -   Token which can be used for this callback.
                    buff      -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdNetcfg_getDhcpServerResults(void *args, int32_t num, char *buff)
{    
    int32_t ret = 0;
    ATCmdNetCfgDhcpParams_t *params = (ATCmdNetCfgDhcpParams_t *)args;
    char hostAddress[INET_ADDRSTRLEN] = {0};
    const char *addr = NULL;

    StrMpl_setStr(ATCmd_netcfgGetDhcpServerStr, &buff, ATCMD_DELIM_EVENT);

    /* Set lease time value in minutes */
    StrMpl_setVal(&params->leaseTime, &buff, ATCMD_DELIM_ARG,
                  STRMPL_FLAG_PARAM_SIZE_32 | \
                  STRMPL_FLAG_PARAM_UNSIGNED | \
                  STRMPL_FLAG_PARAM_DEC);

    /* Set start IP address */
    addr = inet_ntop(AF_INET, &params->startAddress,
                     hostAddress, INET_ADDRSTRLEN);
    if (addr == NULL)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        os_free(params);
        return -1;
    }
    StrMpl_setStr(hostAddress, &buff, ATCMD_DELIM_ARG);

    /* Set end IP address */
    addr = inet_ntop(AF_INET, &params->endAddress,
                     hostAddress, INET_ADDRSTRLEN);
    if (addr == NULL)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        os_free(params);
        return -1;
    }
    StrMpl_setStr(hostAddress, &buff, ATCMD_DELIM_TRM);

    os_free(params);
    return 0;
}

/*!
    \brief          Netcfg get DHCP callback.

    This routine retrieves current DHCP server configurations.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_getDhcpServerCallback(void *arg)
{
    int32_t ret = 0;
    ATCmdNetCfgDhcpParams_t *params = NULL;
    struct dhcps_lease lease = {0};

    params = os_malloc(sizeof(ATCmdNetCfgDhcpParams_t));
    if (params == NULL)
    {
        ATCmd_errorResult(ATCmd_errorAllocStr, 0);
        os_free(params);
        return -1;
    }
    os_memset(params, 0, sizeof(ATCmdNetCfgDhcpParams_t));

    ret = wifi_softap_get_dhcps_lease(&lease);
    if (ret <= 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        os_free(params);
        return -1;
    }

    params->leaseTime = wifi_softap_get_dhcps_lease_time();
    params->startAddress = lease.start_ip.addr;
    params->endAddress = lease.end_ip.addr;

    ATCmd_okResult();
    ATCmd_commandResult(ATCmdNetcfg_getDhcpServerResults, params, 0);

    return 0;
}

static void ATCmdNetcfg_pingReportCallback(void *results)
{
    ATCmdEvent_compose(results, sizeof(ping_results_report_t),
                       ATCmdEvent_pingStopCallback, 0);
}

/*!
    \brief          Return result.

    \param          arg       -   Points to list of arguments.
                    num       -   Token which can be used for this callback.
                    buff      -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1;

*/
int32_t ATCmdNetcfg_pingStartResults(void *args, int32_t num, char *buff)
{    
    int32_t ret = 0;

    StrMpl_setStr(ATCmd_netcfgPingStr, &buff, ATCMD_DELIM_EVENT);

    /* Session ID */
    StrMpl_setVal(&num, &buff, ATCMD_DELIM_TRM,
                  STRMPL_FLAG_PARAM_SIZE_16 |
                  STRMPL_FLAG_PARAM_UNSIGNED |
                  STRMPL_FLAG_PARAM_DEC);

    return ret;
}

/*!
    \brief          Parse ping send command.

    \param          arg          -   Points to command line buffer.
    \param          pingParams   -   Points to ping parameters.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;
*/
int32_t ATCmdNetcfg_pingParse(char *arg, PingParams_t *pingParams)
{
    int32_t ret = 0;
    uint8_t ip[IPV4_ADDR_LEN] = {0};

    /* Fill ping parameters with default values */
    pingParams->interval_time_ms = PING_DEFAULT_INTERVAL_TIME_MS;
    pingParams->payload_size     = PING_DEFAULT_DATA_SIZE;
    pingParams->count            = PING_DEFAULT_COUNT;
    pingParams->flags            = 0;

    pingParams->ping_deinit_callback = ATCmdNetcfg_pingReportCallback;

    /* IPv4 target address */
    ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_ARG,
                             ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                             ATCmd_excludeDelimArray);
    if (ret < 0)
    {
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_TRM,
                                     ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                                     ATCmd_excludeDelimArray);
            if (ret == 0)
            {
                pingParams->target_ip.ipv4 = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);
            }
        }
        return ret;
    }
    pingParams->target_ip.ipv4 = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);

    /* Interval time */
    ret = StrMpl_getVal(&arg, &(pingParams->interval_time_ms), ATCMD_DELIM_ARG,
                        STRMPL_FLAG_PARAM_SIZE_32);
    if (ret < 0)
    {
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getVal(&arg, &(pingParams->interval_time_ms), ATCMD_DELIM_TRM,
                                STRMPL_FLAG_PARAM_SIZE_32);
            return ret;
        }

        if (ret != STRMPL_ERROR_PARAM_MISSING)
        {
            return ret;
        }
    }

    /* Ping count */
    ret = StrMpl_getVal(&arg, &(pingParams->count), ATCMD_DELIM_ARG,
                        STRMPL_FLAG_PARAM_SIZE_32);
    if (ret < 0)
    {
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getVal(&arg, &(pingParams->count), ATCMD_DELIM_TRM,
                                STRMPL_FLAG_PARAM_SIZE_32);
            return ret;
        }

        if (ret != STRMPL_ERROR_PARAM_MISSING)
        {
            return ret;
        }
    }

    /* Payload count */
    ret = StrMpl_getVal(&arg, &(pingParams->payload_size), ATCMD_DELIM_ARG,
                        STRMPL_FLAG_PARAM_SIZE_16);
    if (ret < 0)
    {
        if (ret == STRMPL_ERROR_DELIM_MISSING)
        {
            ret = StrMpl_getVal(&arg, &(pingParams->payload_size), ATCMD_DELIM_TRM,
                                STRMPL_FLAG_PARAM_SIZE_16);
            return ret;
        }

        if (ret != STRMPL_ERROR_PARAM_MISSING)
        {
            return ret;
        }
    }

    /* IPv4 source address */
    ret = StrMpl_getArrayVal(&arg, (void *)ip, IPV4_ADDR_LEN, ATCMD_DELIM_TRM,
                             ATCMD_DELIM_INTER, STRMPL_FLAG_PARAM_SIZE_8,
                             ATCmd_excludeDelimArray);
    if (ret == 0)
    {
        pingParams->source_ip.ipv4 = WLAN_IPV4_VAL(ip[0], ip[1], ip[2], ip[3]);
    }
    else if (ret == STRMPL_ERROR_PARAM_MISSING)
    {
        ret = 0;
    }

    return ret;
}

/*!
    \brief          Ping callback.

    This routine sends a ping to specified address.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_pingCallback(void *arg)
{
    int32_t ret = 0;
    PingParams_t pingParams = {0};
    uint16_t pingSessionId = 0;

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_IS_UP))
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorDeviceNotStartedStr, ret);
        return ret;
    }

    if (!IS_BIT_SET(ActiveNetIfBitMap, NET_IF_STA_BIT) &&
        !IS_BIT_SET(ActiveNetIfBitMap, NET_IF_AP_BIT))
    {
        ret = -1;
        ATCmd_errorResult(ATCmd_errorRoleNotUpStr, ret);
        return ret;
    }

    ret = ATCmdNetcfg_pingParse((char *)arg, &pingParams);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }

    ret = lwip_ping_start(&pingParams);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
        return ret;
    }
    pingSessionId = ret;

    ATCmd_okResult();
    ATCmd_commandResult(ATCmdNetcfg_pingStartResults, NULL, pingSessionId);

    return ret;
}

/*!
    \brief          Parse ping stop command.

    \param          arg          -   Points to command line buffer.
    \param          sessionId    -   Points to session id to stop.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;
*/
int32_t ATCmdNetcfg_pingStopParse(char *arg, int8_t *sessionId)
{
    int32_t ret = 0;

    ret = StrMpl_getVal(&arg, sessionId, ATCMD_DELIM_TRM,
                        STRMPL_FLAG_PARAM_SIZE_8);

    return ret;
}

/*!
    \brief          Ping stop callback.

    This routine stops ping sending.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_pingStopCallback(void *arg)
{
    int32_t ret = 0;
    int8_t sessionId = 0;

    ret = ATCmdNetcfg_pingStopParse((char *)arg, &sessionId);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorParseStr, ret);
        return ret;
    }
    
    ret = lwip_ping_stop(sessionId);
    if (ret < 0)
    {
        ATCmd_errorResult(ATCmd_errorCmdStr, ret);
    }

    return ret;
}
