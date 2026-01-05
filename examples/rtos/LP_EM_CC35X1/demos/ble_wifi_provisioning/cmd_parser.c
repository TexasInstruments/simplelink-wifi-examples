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
/* General includes */
#include <stdlib.h>
#include "string.h"

/* Application includes */
#include "lwip_iperf_examples.h"
#include "str.h"
#include "wlan_cmd.h"
#include "ble_cmd.h"
#include "uart_term.h"
#include "socket_examples.h"
#include "lwip_ping.h"

#include "network_terminal.h"
#include "cmd_parser.h"
#include "osi_kernel.h"

void* os_zalloc(size_t size);
void  os_free(void *ptr);
int32_t printSendUsage(void *arg);
int32_t printRecvUsage(void *arg);

LoadCertiCmd_t client_certi;
LoadCertiCmd_t ca_certi;
LoadCertiCmd_t private_key_certi;

#define DATE_TIME_STR_SIZE (22)


/* Application defines */
#define FRAME_TYPE_MASK                 (0x0C)
#define DEVICE_YEAR                     (2016)
#define DEVICE_MONTH                    (1)
#define DEVICE_DATE                     (6)
#define KEY_LEN_MAX                     (64)
#define MAX_PAYLOAD_SIZE                (1400)
#define FRAME_SUBTYPE_MASK              (0xF0)
#define DEF_ADVERTISE_PORT              (2525)
#define DEF_ADVERTISE_TTL               (2000)

#define SIX_BYTES_SIZE_MAC_ADDRESS      (17)

/******************************************************************************
                  Callback Functions
******************************************************************************/
/*!
    \brief          Parse Scan command.

    This routine takes a ScanCmd_t structure, and fill it's content with
    parameters taken from command line.
    In case of a parsing error or invalid parameters,
    this function sets default values.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          scanParams  -   Points to command structure provided
                                    by the scan callback.
                                    This structure will later be read
                                    by the scan callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the Scan command help menu.

    \sa             cmdScanCallback
 */
int32_t ParseScanCmd(void *arg, ScanCmd_t *scanParams)
{
    char            *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(NULL == token)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, n_optionStr))
        {
            token = strtok(NULL, space_str);

            if(token)
            {
                scanParams->numOfentries = (uint8_t)atoi(token);
            }

            if(scanParams->numOfentries > MAX_SSID_ENTRIES ||
               (0 == scanParams->numOfentries))
            {
                scanParams->numOfentries = MAX_SSID_ENTRIES;
            }
        }
        else
        {
            Report("\r\n using defaults");
            break;
        }
        token = strtok(NULL, " ");
    }

    if(help)
    {
        return(-1);
    }

    return(0);
}

/*!
    \brief          Parse Stop command.

    This routine takes a ProfileCmd_t structure, and fill it's content with
    parameters taken from command line.It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          stopParams  -      Points to StopCmd_t structure provided
                                       by the stop callbeck

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.


 */
int32_t ParseStopCmd(void *arg, StopCmd_t *stopParams)
{
    char            *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        stopParams->isRecovery = FALSE;
    }

    while(token)
    {
        if(!strcmp(token, r_optionStr))
        {
            stopParams->isRecovery = TRUE;
        }
        else
        {
            return -1;
        }
        break;
    }


    return(0);
}

/*!
    \brief          Parse Profile command.

    This routine takes a ProfileCmd_t structure, and fill it's content with
    parameters taken from command line.It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          ProfileParams  -   Points to command structure provided
                                       by the connect callback.
                                       This structure will later be read by
                                       the connect callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdAddProfileCallback
 */
int32_t ParseProfileCmd(void *arg, ProfileCmd_t *ProfileParams)
{
    char                 cmdStr[CMD_BUFFER_LEN + 1];
    char                 *token = NULL;
    char                 *ssid = NULL;
    char                 *password = NULL;
    char                 *security = NULL;
    char                 *priority = NULL;
    char                 *hidden = NULL;          
    uint8_t              help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    ProfileParams->mac = NULL;
    ProfileParams->priority = NULL;

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = 1;
        }
        else if(!strcmp(token, s_optionStr))
        {
            char *remainingCmd = strtok(NULL, "");
            int valid_format = 0;
            if (remainingCmd != NULL) 
            {
                while (*remainingCmd && isspace((unsigned char)*remainingCmd))// Skip leading spaces
                {
                    remainingCmd++;
                }
                if (*remainingCmd == '\"') 
                {
                    char *closing_quote = strchr(remainingCmd + 1, '\"');
                    
                    if (closing_quote != NULL) 
                    {
                        *closing_quote = '\0';
                        ssid = remainingCmd + 1;
                        token = strtok(closing_quote + 1, space_str);
                        valid_format = 1;
                        continue;
                    }
                }
            }
            
            if (!valid_format) 
            {
                SHOW_WARNING(-1, "Error: SSID must be enclosed in quotation marks.\n");
                help = 1;
                break;
            }

        }
        else if(!strcmp(token, p_optionStr))
        {
            password = strtok(NULL, "\"");
        }
        else if(!strcmp(token, t_optionStr))
        {
            security = strtok(NULL, space_str);
        }
        else if(!strcmp(token, priority_optionStr))
        {
            priority = strtok(NULL, space_str);
        }
        else if(!strcmp(token, h_optionStr))
        {
            hidden = strtok(NULL, space_str);
        }
#if 0 /* TODO: Add enterprise profile support */
        else if(!strcmp(token, ent_optionStr))
        {
            SHOW_WARNING(-1, CMD_ENT_ERROR);
            help = TRUE;
            break;
            /* For future use: process the enterprise user name
            char *entUserName = NULL;
            entUserName = strtok(NULL,  "\" ");
            */
        }
#endif
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        printAddProfileUsage(arg);
        return(-1);
    }

    if((ssid != NULL) && (strlen(ssid) <= WLAN_SSID_MAX_LENGTH))
    {   
        size_t len = strlen(ssid);
        ProfileParams->ssid = (uint8_t *)os_zalloc(len + 1);
        memcpy(ProfileParams->ssid, ssid, len + 1);
    }
    else
    {
        Report("\r\n[Cmd Parser] : Invalid SSID.\n\r");
        return(-1);
    }

    if(priority)
    {
        ProfileParams->priority = (uint32_t *)os_zalloc(4);
        if (atoi((const char*)priority) < 0 || atoi((const char*)priority) > 15)
        {
            Report("\r\n [Cmd Parser] : Invalid priority. Expected priority 0-15.\n\r");
            return(-1);
        }
        else
        {
            *(ProfileParams->priority) = atoi((const char*)priority);
        }
    }
    else
    {
        ProfileParams->priority = (uint32_t *)os_zalloc(4);
        *(ProfileParams->priority) = 0;
    }
    
    if(hidden)
    {
        ProfileParams->hidden = (uint32_t *)os_zalloc(4);
        if( atoi((const char*)hidden) < 0 || atoi((const char*)hidden) > 1)
        {
            Report("\r\n [Cmd Parser] : Invalid hidden. Expected 0 or 1. Setting default value (1) \n\r");
            *(ProfileParams->hidden) = 1;
        }
        else
        {
            *(ProfileParams->hidden) = atoi((const char*)hidden);
        }
    }
    else
    {
        // By default, the profile is hidden
        *(ProfileParams->hidden) = 1;
    }

    if(!security)
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_OPEN;
    }
    else if(!strcmp(security, WPA_str))
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_WPA_WPA2;
    }
    else if(!strcmp(security, WPA2_str))
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_WPA_WPA2;
    }
    else if(!strcmp(security, WPS_str) && !password)
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_WPS_PBC;
    }
    else if(!strcmp(security, WPS_str) && password)
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_WPS_PIN;
    }
    else if(!strcmp(security, OPEN_str))
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_OPEN;
    }
    else if(!strcmp(security, WPA3_str))
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_WPA3;
    }
    else if(!strcmp(security, WPA2_PLUS_str))
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_WPA2_PLUS;
    }
    else if (!strcmp(security, WPA2WPA3_str))
    {
        ProfileParams->secParams.Type = WLAN_SEC_TYPE_WPA2_WPA3;
    }
    else
    {
        Report(
            "\r\n [Cmd Parser] : Parser expected password "
            "parameters [OPEN, WPA, WPA2, WPA2_PLUS, WPA3, WPS].\n\r");
        return(-1);
    }

    ProfileParams->secParams.KeyLen = 0;

    if((password != NULL) && (strlen(password) <= PASSWD_LEN_MAX))
    {
        ProfileParams->secParams.KeyLen = strlen(password);
        ProfileParams->secParams.Key = (signed char *)os_zalloc(
                ProfileParams->secParams.KeyLen + 1);
        strncpy((char *)ProfileParams->secParams.Key, password,
                ProfileParams->secParams.KeyLen);
    }
    else if(ProfileParams->secParams.Type != WLAN_SEC_TYPE_OPEN)
    {
        Report("\r\n[Cmd Parser] : Invalid Password.\n\r");
        return(-1);
    }


    return(0);
}

/*!
    \brief          Parse Disconnect command.

    This routine parse Disconnect parameter (RoleID).
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          RoleId            -   Pointer to RoleId


    \return         Upon successful completion, the function shall return zero.
                    In case of failure,this function would print error,
                    and show the Disconnect command help menu.

    \sa
 */
int32_t ParseDisconnectCmd(void *arg, uint32_t *RoleId)
{
    char       cmdStr[CMD_BUFFER_LEN + 1];
    char       *token = NULL;
    char       *strRoleId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, i_optionStr))
        {
            strRoleId = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        return(-1);
    }

    *RoleId = atoi((const char*)strRoleId);

    //check if role id valid
    if ((*RoleId < 0 ) || (*RoleId > 3))
    {
        Report("\r\n[Cmd Parser] : Invalid RoleId. Range [0-3]\n\r");
        return(-1);
    }
    return(0);
}

/*!
    \brief          Parses command line arguments for early termination setting.

    \param          arg       -   Points to command line buffer.

    \param          pEnable   -   Pointer to store the parsed enable/disable value.

    \return         0 on successful parsing, negative value on error.
*/
int32_t parseSetEarlyTerminationArgs(void *arg, uint8_t *pEnable)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strValue = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';

    token = strtok(cmdStr, space_str);
    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, "-e"))
        {
            strValue = strtok(NULL, space_str);
            if (strValue == NULL)
            {
                SHOW_WARNING(-1, CMD_ERROR);
                help = TRUE;
                break;
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        // Print usage information
        printSetEarlyTermUsage(NULL);
        return (-1);
    }

    int value = atoi((const char*) strValue);

    // Check if value is valid
    if (value != 0 && value != 1)
    {
        Report("\n\r[Cmd Parser] : Invalid value. Must be 0 or 1\n\r");
        return (-1);
    }

    *pEnable = (uint8_t)value;
    return (0);
}

/*!
    \brief          Parse Get MacAdress command.

    This routine parse get mac address parameter (RoleID).
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          RoleId         -   Pointer to RoleId.



    \return         Upon successful completion, the function shall return the RoleId number.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdSetMacAddressCallback
 */
int32_t ParseGetMacAddressCmd(void *arg, uint32_t *RoleId)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strRoleId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, i_optionStr))
        {
            strRoleId = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        return (-1);
    }

    *RoleId = atoi((const char*) strRoleId);

    //check if role id valid
    if ((*RoleId < 0) || (*RoleId > 3))
    {
        Report("\r\n[Cmd Parser] : Invalid RoleId. Range [0-3]\n\r");
        return (-1);
    }
    return (0);
}

/*!
    \brief          Parse Set MacAdress command.

    This routine parse set mac address parameter (RoleID, macAddress).
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
     \param         pMacAddress    -   Points to array that represent the mac adrress

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdAddProfileCallback
 */
int32_t ParseSetMacAddressCmd(void *arg, uint8_t *pMacAddress, uint32_t*  RoleId)
{
    char       cmdStr[CMD_BUFFER_LEN + 1];
    char       *token = NULL;
    char       *strRoleId = NULL;
    char       *macAddressStr = NULL;
    int16_t    ret = 0;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
     {
         if(!strcmp(token, i_optionStr))
         {
             strRoleId = strtok(NULL, space_str);
         }
         else if(!strcmp(token, m_optionStr))
         {
             macAddressStr = strtok(NULL, space_str);
         }
         else
         {
             SHOW_WARNING(-1, CMD_ERROR);
             help = TRUE;
             break;
         }
         token = strtok(NULL, space_str);
     }

    if(help)
    {
        return(-1);
    }

    *RoleId = atoi((const char*)strRoleId);

    //check if role id valid
    if ((*RoleId < 0 ) || (*RoleId > 3))
    {
         Report("\r\n[Cmd Parser] : Invalid RoleId. Range [0-3]\n\r");
         return(-1);
    }
    ret = macAddressParse(macAddressStr, pMacAddress);
    if (ret < 0)
    {
        Report("\r\n[Cmd Parser] : Invalid MacAddress. Format xx:xx:xx:xx:xx:xx\n\r");
        return (-1);
    }

    return(0);
}



/*!
    \brief          Parse Get Profile command.

    This routine takes a command line buffer, and fill it's content with
    parameters taken from command line.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \return         Upon successful completion, the function shall return the index number.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdGetProfileCallback
 */
int32_t ParseGetProfileCmd(void *arg)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *index = NULL;
    int16_t index_int = 0;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, i_optionStr))
        {
            index = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printGetProfileUsage(arg);
        return (-1);
    }

    index_int = atoi((const char*) index);

    return (index_int);
}


/*!
    \brief          Parse Delete Profile command.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \return         Upon successful completion, the function shall return success.
                    In case of failure,this function would print error.

    \sa             
 */
int32_t ParseDelProfileCmd(void *arg)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *index = NULL;
    int16_t index_int = 0;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, i_optionStr))
        {
            index = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printDeleteProfileUsage(arg);
        return (-1);
    }

    index_int = atoi((const char*) index);

    return (index_int);
}

/*!
    \brief          Parse Profile Connect command.

    This routine takes a command line buffer, and fill it's content with
    parameters taken from command line.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \return         Upon successful completion, the function shall return the index number.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdWlanProfileConnectCallback
 */
int32_t ParseProfileConnectCmd(void *arg)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *index = NULL;
    int16_t index_int = 0;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, i_optionStr))
        {
            index = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printWlanProfileConnectUsage(arg);
        return (-1);
    }

    index_int = atoi((const char*) index);

    return (index_int);
}


/*!
    \brief          Parse Connect command.

    This routine takes a ConnectCmd_t structure, and fill it's content with
    parameters taken from command line.It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          ConnectParams  -   Points to command structure provided
                                       by the connect callback.
                                       This structure will later be read by
                                       the connect callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdWlanConnectCallback
 */
int32_t ParseConnectCmd(void *arg, ConnectCmd_t *ConnectParams,
        WlanEapConnectParams_t* eapConnectParams, uint8_t* isEnt)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char                 *token = NULL;
    char                 *ssid = NULL;
    char                 *password = NULL;
    char                 *security = NULL;
    char                 *eapPhaseVal = NULL;
    char                 *eapIdentity = NULL;
    int32_t              eapIdentityLen;

    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = 1;
        }
        else if(!strcmp(token, s_optionStr))
        {
            char *remainingCmd = strtok(NULL, "");
            int valid_format = 0;
            if (remainingCmd != NULL) 
            {
                while (*remainingCmd && isspace((unsigned char)*remainingCmd))// Skip leading spaces
                {
                    remainingCmd++;
                }
                if (*remainingCmd == '\"') 
                {
                    char *closing_quote = strchr(remainingCmd + 1, '\"');
                    
                    if (closing_quote != NULL) 
                    {
                        *closing_quote = '\0';
                        ssid = remainingCmd + 1;
                        token = strtok(closing_quote + 1, space_str);
                        valid_format = 1;
                        continue;
                    }
                }
            }
            
            if (!valid_format) 
            {
                SHOW_WARNING(-1, "Error: SSID must be enclosed in quotation marks.\n");
                help = 1;
                break;
            }
        }
        else if(!strcmp(token, p_optionStr))
        {
            password = strtok(NULL, "\"");
        }
#ifdef CC35XX
        else if(!strcmp(token, e_optionStr))
        {
            *isEnt = 1 ;//enterprise connection
            eapPhaseVal = strtok(NULL, space_str);

            if (!strcmp(eapPhaseVal, TLS_str))
            {
                eapConnectParams->eap_phase1_val = WLAN_TLS;
            }
            else if (!strcmp(eapPhaseVal, TTLS_MSCHAP_str))
            {
                eapConnectParams->eap_phase1_val = WLAN_TTLS;
                eapConnectParams->eap_phase2_val = WLAN_MSCHAPV2_TYPE;
            }
            else if (!strcmp(eapPhaseVal, PEAP0_MSCHAP_str))
            {
                eapConnectParams->eap_phase1_val = WLAN_PEAP0;
                eapConnectParams->eap_phase2_val = WLAN_MSCHAPV2_TYPE;
            }
            else
            {
                Report("\n\r ERROR ! Received enterprise method is invalid %s");
            }

        }
        else if(!strcmp(token, i_optionStr))
        {
            eapIdentity = strtok(NULL, "\"");
            //identity
            os_memset(eapConnectParams->eapIdentity,0, sizeof(eapConnectParams->eapIdentity));
            eapIdentityLen = os_strlen(eapIdentity);
            if (eapIdentityLen >= sizeof(eapConnectParams->eapIdentity)) {
                eapIdentityLen = sizeof(eapConnectParams->eapIdentity) - 1;
            }
            eapConnectParams->eapIdentityLen = eapIdentityLen;
            os_memcpy(eapConnectParams->eapIdentity,eapIdentity, eapIdentityLen);
            Report("\n\r eapIdentoty %s",eapConnectParams->eapIdentity);

        }
#endif
        else if(!strcmp(token, t_optionStr))
        {
            security = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        printWlanConnectUsage(arg);
        return(-1);
    }

    if(ssid != NULL)
    {
        if (!strcmp(security, WPS_str))
        {
            // If a WPS type is provided, we not allow the 'SSID' parameter
            Report("\r\n[Cmd Parser] : WPS type is provided, we not allow the 'SSID' parameter.\n\r");
            return(-1);
        }
        else if((strlen(ssid) > WLAN_SSID_MAX_LENGTH))
        {
            Report("\r\n[Cmd Parser] : Invalid SSID.\n\r");
            return(-1);
        }
        else
        {
            ConnectParams->ssid = (uint8_t *)os_zalloc(strlen(ssid) + 1);
            strcpy((char *)ConnectParams->ssid, ssid);
            ConnectParams->ssid[strlen(ssid)]=0;
        }
    }
    else if(strcmp(security, WPS_str))
    {
            Report("\r\n[Cmd Parser] : Invalid SSID.\n\r");
            return(-1);
    }


    if(!security)
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_OPEN;
    }
    else if(!strcmp(security, WPA_str))
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_WPA_WPA2;
    }
    else if(!strcmp(security, WPA2_str))
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_WPA_WPA2;
    }
    else if(!strcmp(security, WPS_str) && !password)
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_WPS_PBC;
    }
    else if(!strcmp(security, WPS_str) && password)
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_WPS_PIN;
    }
    else if(!strcmp(security, OPEN_str))
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_OPEN;
    }
    else if(!strcmp(security, WPA3_str))
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_WPA3;
    }
    else if(!strcmp(security, WPA2_PLUS_str))
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_WPA2_PLUS;
    }
#ifdef CC35XX
    else if(!strcmp(security, WPA2WPA3_str))
    {
        ConnectParams->secParams.Type = WLAN_SEC_TYPE_WPA2_WPA3;
    }
#endif
    else
    {
        Report(
            "\r\n [Cmd Parser] : Parser expects security type "
            "parameter [OPEN, WPA, WPA2, WPA3, WPS, WPA2/WPA3].\n\r");
        return(-1);
    }

    ConnectParams->secParams.KeyLen = 0;

    if((password != NULL) && (strlen(password) <= PASSWD_LEN_MAX))
    {
        ConnectParams->secParams.KeyLen = strlen(password);
#ifdef CC35XX		
        ConnectParams->secParams.Key = (int8_t *)os_zalloc(ConnectParams->secParams.KeyLen + 50 );//OSPREY_MX-14
#else
		ConnectParams->secParams.Key = (int8_t *)os_zalloc(ConnectParams->secParams.KeyLen + 1 );
#endif 		
        strncpy((char *)ConnectParams->secParams.Key, password,
                ConnectParams->secParams.KeyLen);
        ConnectParams->secParams.Key[ConnectParams->secParams.KeyLen] = 0;
    }
    else if((ConnectParams->secParams.Type != WLAN_SEC_TYPE_OPEN) &&
            (ConnectParams->secParams.Type != WLAN_SEC_TYPE_WPS_PBC) &&
            (!(*isEnt)))
    {
        Report("\r\n[Cmd Parser] : Invalid Password.\n\r");
        return(-1);
    }
    return(0);
}

/*!
    \brief          Free Connect command.

    This routine takes a ConnectCmd_t structure,
    and free all memory that was
    allocated in ParseConnectCmd.

    \param          ConnectParams  -
    Points to command structure provided by the connect callback.

    \return         void

    \sa             cmdWlanConnectCallback
 */
void FreeConnectCmd(ConnectCmd_t *ConnectParams)
{
    if(ConnectParams->ssid != NULL)
    {
        os_free(ConnectParams->ssid);
        ConnectParams->ssid = NULL;
    }
    if(ConnectParams->secParams.Key != NULL)
    {
        os_free(ConnectParams->secParams.Key);
        ConnectParams->secParams.Key = NULL;
    }
    return;
}

/*!
    \brief          Free Profile command.

    This routine takes a ProfileCmd_t structure,
    and free all memory that was
    allocated in ParseProfileCmd.

    \param          ProfileParams  -
    Points to command structure provided by the add profile callback.

    \return         void

    \sa             cmdWlanaddProfileCallback
 */
void FreeProfileCmd(ProfileCmd_t *ProfileParams)
{
    if(ProfileParams->ssid != NULL)
    {
        os_free(ProfileParams->ssid);
        ProfileParams->ssid = NULL;
    }

    if(ProfileParams->mac != NULL)
    {
        os_free(ProfileParams->mac);
        ProfileParams->mac = NULL;
    }

    if(ProfileParams->priority != NULL)
    {
        os_free(ProfileParams->priority);
        ProfileParams->priority = NULL;
    }

    if(ProfileParams->secParams.Key != NULL)
    {
        os_free(ProfileParams->secParams.Key);
        ProfileParams->secParams.Key = NULL;
    }


    return;
}

/*!
    \brief          Check if the AllowedChannelsFor5GhzAP array
                    contain the candidate channel

    This function takes the channel that received from user,
    in case of NWP generation compatible with cc3235,
    check if that channel is allowed according to
    pre-configured allowed channels at AllowedChannelsFor5GhzAP array.

    \param          *AllowedChannelsFor5GhzAP  -   Point to pre-configured
                                                 allowed channels.

    \param          length  -                    Length of
                                                 AllowedChannelsFor5GhzAP.

    \param          channel -                    The candidate channel.

    \return         If AllowedChannelsFor5GhzAP contain the
                    channel return 1 (true),
                    if AllowedChannelsFor5GhzAP does not contain
                    the channel return 0 (false).
 */
uint8_t isChannelAt(uint8_t *AllowedChannelsFor5GhzAP,
                    uint32_t length,
                    uint8_t channel                   )
{
    uint32_t i;
    uint8_t  ret = 0;

    for(i = 0; i < length; i++)
    {
        if(channel == AllowedChannelsFor5GhzAP[i])
        {
            ret = 1;
            return ret;
        }
    }
    return ret;
}


/*!
    \brief          Parse Role up STA command.

    This routine takes a RoleUpApCmd_t structure, and fill it's content with
    parameters taken from command line. It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          RoleUpApParams  -   Points to command structure provided
                                       by the connect callback.
                                       This structure will later be read by
                                       the Role Up STA callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the set Start STA command help menu.

    \sa             cmdWlanRoleUpStaCallback
 */
int32_t ParseRoleUpStaCmd(void *arg, RoleUpStaCmd_t *RoleUpStaParams)
{
    char             *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    char    *domain = NULL;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            return (-1);
        }
        else if(!strcmp(token, r_optionStr))
        {
            domain = strtok(NULL, "\"");
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            return (-1);
        }
        token = strtok(NULL, space_str);
    }

#ifdef CC33XX
    if((!strcmp(domain, "US")) || (!strcmp(domain, "EU")) || (!strcmp(domain, "JP")) || (!strcmp(domain, "CS")) ||(!strcmp(domain, "00")))
    {
        strncpy((char *)RoleUpStaParams->countryDomain, (char *)domain, 2);
        Report("\n\rChosen domain is %s", domain);
    }
    else
    {
        strncpy((char *)RoleUpStaParams->countryDomain, (char *)"00", 3);
        Report("\n\tChosen domain is WW\n");
    }
#elif defined (CC35XX)
    if (domain)
    {
        os_memcpy((char *)RoleUpStaParams->countryDomain, (char *)domain, 2);
        RoleUpStaParams->countryDomain[2] = '\0';
        Report("\n\rChosen country code is %s", domain);
    }
    else
    {
        os_memcpy((char *)RoleUpStaParams->countryDomain, "00", 2);
        RoleUpStaParams->countryDomain[2] = '\0';
        Report("\n\rChosen country code is WW");
    }
#endif
    RoleUpStaParams->countryDomain[2] = 'I'; //Indoor

    return(0);
}

/*!
    \brief          Parse Role up AP command.

    This routine takes a RoleUpApCmd_t structure, and fill it's content with
    parameters taken from command line. It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          RoleUpApParams  -   Points to command structure provided
                                       by the connect callback.
                                       This structure will later be read by
                                       the Role Up AP callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the set Start AP command help menu.

    \sa             cmdWlanRoleUpApCallback
 */
int32_t ParseRoleUpApCmd(void *arg, RoleUpApCmd_t *RoleUpApParams)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token;
    char    *ssid = NULL;
    char    *password = NULL;
    char    *security = NULL;
    uint8_t help = FALSE;
#ifdef CC35XX
    uint8_t sae_pwe = 2; 
#elif defined (CC33XX)
    uint8_t AllowedChannelsFor5GhzAP [4] = {36,40,44,48};
    uint8_t isSupportedChannel = FALSE;
#endif

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    /* Set default parameters */
    RoleUpApParams->sta_limit = 4;
    RoleUpApParams->hidden = FALSE;
    RoleUpApParams->tx_pow = 0;
    RoleUpApParams->channel = 1;
#ifdef CC33XX	
    RoleUpApParams->countryDomain[0] = '\0';
#endif // CC33XX	
#ifdef CC35XX
    RoleUpApParams->sae_pwe = 2;
    RoleUpApParams->countryDomain[0] = '0';
    RoleUpApParams->countryDomain[1] = '0';
    RoleUpApParams->countryDomain[2] = '\0';
#endif

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, s_optionStr))
        {
            ssid = strtok(NULL, "\"");
        }
        else if(!strcmp(token, p_optionStr))
        {
            password = strtok(NULL, "\"");
        }
        else if(!strcmp(token, t_optionStr))
        {
            security = strtok(NULL, space_str);
        }
        else if(!strcmp(token, h_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                if(!strcmp(token, "YES") || !strcmp(token, "yes"))
                {
                    RoleUpApParams->hidden = TRUE;
                }
                else if(!strcmp(token, "NO") || !strcmp(token, "no"))
                {
                    RoleUpApParams->hidden = FALSE;
                }
                else
                {
                    Report(
                        "\r\n [Cmd Parser] : invalid Parameter for hidden "
                        "AP option. Using default (Not hidden).\n\r");
                    RoleUpApParams->hidden = FALSE;
                }
            }
        }
        else if(!strcmp(token, c_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpApParams->channel = atoi(token);
            }
#ifdef CC33XX
            /* Check if the configured channel is supported
               (according to NWP generation) */
            if( ((0 < RoleUpApParams->channel) && (RoleUpApParams->channel < 13))
                || (
               (isChannelAt(AllowedChannelsFor5GhzAP,
                sizeof(AllowedChannelsFor5GhzAP)/sizeof(uint8_t),
                RoleUpApParams->channel))) )
            {
                isSupportedChannel = TRUE;
            }

            /* In case of wrong channel configuration
               go to default channel (1)  */
            if(isSupportedChannel == (FALSE))
            {
                Report("\r\n [Cmd Parser] : invalid Parameter for channel"
                 " option. Using default (1).\n\r");
                RoleUpApParams->channel = 1;
            }
#endif // CC33XX
        }
        else if(!strcmp(token, txpow_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpApParams->tx_pow = atoi(token);
            }

            if(RoleUpApParams->tx_pow > 15)
            {
                Report(
                   "\r\n [Cmd Parser] : invalid Parameter for tx power option."
                   " Using default (0).\n\r");
                RoleUpApParams->tx_pow = 0;
            }
        }
        else if(!strcmp(token, l_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpApParams->sta_limit = atoi(token);
            }

            if((RoleUpApParams->sta_limit < 1) ||
            (RoleUpApParams->sta_limit > 4))
            {
                Report(
                    "\r\n [Cmd Parser] : invalid Parameter for station limit "
                    "option. Using default (4).\n\r");
                RoleUpApParams->sta_limit = 4;
            }
        }
        else if(!strcmp(token, r_optionStr))
        {
            token = strtok(NULL, "\"");
            if(token)
            {
                strncpy((char *)RoleUpApParams->countryDomain, (char *)token, 2);
                RoleUpApParams->countryDomain[2] = '\0';
            }
        }
#ifdef CC35XX
        else if(!strcmp(token, w_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            { 
                sae_pwe = atoi(token);

                if((sae_pwe != 0) && (sae_pwe != 1) && (sae_pwe != 2))
                {
                    SHOW_WARNING(-1,
                        "Wrong SAE PWE value. Using default (2).\n\r");
                    sae_pwe = 2;
                }
            }

            RoleUpApParams->sae_pwe = sae_pwe;

        }
#endif
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

#ifdef CC33XX
    if ( (RoleUpApParams->tx_pow != 0)    &&
         ((RoleUpApParams->channel == 36) ||
          (RoleUpApParams->channel == 40) ||
          (RoleUpApParams->channel == 44) ||
          (RoleUpApParams->channel == 48)) )
    {
        SHOW_WARNING(-1,
           "tx power can be set for 2.4GHz channels only.");
        help = TRUE;
    }
#endif // CC33XX

    if(help)
    {
        return(-1);
    }

    if((NULL == ssid) || (strlen(ssid) >= WLAN_SSID_MAX_LENGTH))
    {
        Report(
            "\r\n [Cmd Parser] : invalid Parameter for SSID - Should be max"
            " 31 characters.\n\r");
        return(-1);
    }
    else
    {
        RoleUpApParams->ssid = (uint8_t *)os_zalloc(strlen(ssid)+1);
        strcpy((char *)RoleUpApParams->ssid, ssid);
    }

    if(password != NULL)
    {
        RoleUpApParams->secParams.KeyLen = strlen(password)+1;

        if((RoleUpApParams->secParams.KeyLen >= KEY_LEN_MAX) ||
           (RoleUpApParams->secParams.KeyLen < PASSWD_LEN_MIN))
        {
            Report(
                "\r\n [Cmd Parser] : Key length invalid - "
                "Should be in range: [8,63].\n\r");
            return(-1);
        }

        RoleUpApParams->secParams.Key =
            (signed char *)calloc(RoleUpApParams->secParams.KeyLen, sizeof(uint8_t));
        strcpy((char *)RoleUpApParams->secParams.Key, password);
    }

    if(!security)
    {
        RoleUpApParams->secParams.Type = WLAN_SEC_TYPE_OPEN;
    }
    else if(!strcmp(security, WPAWPA2_str))
    {
        RoleUpApParams->secParams.Type = WLAN_SEC_TYPE_WPA_WPA2;
    }
    else if(!strcmp(security, WPA2_str))
    {
        RoleUpApParams->secParams.Type = WLAN_SEC_TYPE_WPA_WPA2;
    }
    else if(!strcmp(security, OPEN_str))
    {
        RoleUpApParams->secParams.Type = WLAN_SEC_TYPE_OPEN;
    }
#ifdef CC35XX
    else if(!strcmp(security, WPA2_PLUS_str))
    {
        RoleUpApParams->secParams.Type = WLAN_SEC_TYPE_WPA2_PLUS;
    }
    else if (!strcmp(security, WPA3_str))
    {
        RoleUpApParams->secParams.Type = WLAN_SEC_TYPE_WPA3;
    }
    else if(!strcmp(security, WPA2WPA3_str))
    {
        RoleUpApParams->secParams.Type = WLAN_SEC_TYPE_WPA2_WPA3;
    }
#endif
    else
    {
        Report(
            "\r\n [Cmd Parser]: Ap Role UP Parser expected security Type parameter "
            "[OPEN, WPA/WPA2].\n\r");
        return(-1);
    }

    return(0);
}

/*!
    \brief          Free role up AP command.

    This routine takes a RoleUpApCmd_t structure, and free all memory that was
    allocated in ParseRoleUpApCmd.

    \param          StartApParams  -
                Points to command structure provided by the Start AP callback.

    \return         void

    \sa             cmdWlanStartApCallback
 */
void FreeRoleUpApCmd(RoleUpApCmd_t *RoleUpApParams)
{
    if(RoleUpApParams->ssid != NULL)
    {
        os_free(RoleUpApParams->ssid);
        RoleUpApParams->ssid = NULL;
    }

    if(RoleUpApParams->secParams.Key != NULL)
    {
        os_free(RoleUpApParams->secParams.Key);
        RoleUpApParams->secParams.Key = NULL;
    }

    return;
}

#ifdef CC35XX
/*!

    \brief          Parse Role up P2P command.

    This routine fills RoleUpApCmd_t structure,with parameters taken from command line.
    It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          RoleUpP2PParams  -   Points to command structure provided
                                       by the connect callback.
                                       This structure will later be read by
                                       the Role Up AP callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the set Start AP command help menu.

    \sa             cmdWlanRoleUpP2PCallback
 */
int32_t ParseRoleUpP2PCmd(void *arg, RoleUpStaCmd_t *RoleUpP2PParams)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    // when no params default is used
    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }

        /*--------  r ----------------------*/
        else if(!strcmp(token, r_optionStr))
        {
            token = strtok(NULL, "\"");
            if(token)
            {
                strncpy((char *)RoleUpP2PParams->countryDomain, (char *)token, 2);
            }
        }
        /*--------  c oper channel ----------------------*/
        else if(!strcmp(token, c_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpP2PParams->P2pParams.operChannel = atoi(token);
            }
        }
        /*--------  o , oper reg class  ----------------------*/
        else if(!strcmp(token, o_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpP2PParams->P2pParams.operReg = atoi(token);
            }
        }
        /*--------  s , listen channel  ----------------------*/
        else if(!strcmp(token, s_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpP2PParams->P2pParams.listenChannel = atoi(token);
            }
        }
        /*--------  m , listen reg class  ----------------------*/
        else if(!strcmp(token, m_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpP2PParams->P2pParams.listenReg = atoi(token);
            }
        }
        /*--------  -i <Go intent]  ----------------------*/
        else if(!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RoleUpP2PParams->P2pParams.goIntent = atoi(token);
                if(RoleUpP2PParams->P2pParams.goIntent > 15)
                {
                    Report(
                        "\r\n [Cmd Parser] : P2P Go intent can have value from 0 to 15 "
                        "option. Using (15).\n\r");
                    RoleUpP2PParams->P2pParams.goIntent = 15;
                }
            }
        }

        else
        {
            Report("\r\n using default P2P configuration !!!" );
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        return(-1);
    }

    return(0);
}

/*!

    \brief          Parse Set channel command.

    This routine fills WlanP2pCmd_t structure,with parameters taken from command line.
    It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          WlanP2pCmd_t  -   Points to command structure provided
                                       by the set channel callback.
                                       This structure will later be read by
                                       the set channel callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the set Start AP command help menu.

    \sa             cmdWlanRoleUpP2PCallback
 */
int32_t ParseSetChannelCmd(void *arg, WlanP2pCmd_t *P2PParams)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token;
    uint8_t help = FALSE;
    uint8_t isSupportedChannel = FALSE;
    uint8_t AllowedChannelsFor5GhzAP [4] = {36,40,44,48};

    /*
    char wlan_role_up_p2p_UsageStr_first[]   =  " [-help]";
    char  wlan_role_up_p2p_UsageStr_third[]  =   "[-c <oper channel>]"
                                                    "[-o <oper reg class>]"
                                                    "[-s <listen channel>]"
                                                    "[-m <listen reg class >]"
                                                    "[-i <Go intent]>]\n\r";
    */

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    // when no params default is used
    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }

        /*--------  c oper channel ----------------------*/
        else if(!strcmp(token, c_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                P2PParams->Data.cfgParams.operChannel = atoi(token);
            }

            /* Check if the configured channel is supported
               (according to NWP generation) */
            if( ((0 < P2PParams->Data.cfgParams.operChannel) && (P2PParams->Data.cfgParams.operChannel < 13))
                || (
               (isChannelAt(AllowedChannelsFor5GhzAP,
                sizeof(AllowedChannelsFor5GhzAP)/sizeof(uint8_t),
                P2PParams->Data.cfgParams.operChannel))) )
            {
                isSupportedChannel = TRUE;
            }

            /* In case of wrong channel configuration
               go to default channel (1)  */
            if(isSupportedChannel == (FALSE))
            {
                Report("\r\n [Cmd Parser] : invalid Parameter for channel"
                 " option. Using default (1).\n\r");
                P2PParams->Data.cfgParams.operChannel = 1;
            }
        }
        /*--------  o , oper reg class  ----------------------*/
        else if(!strcmp(token, o_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                P2PParams->Data.cfgParams.operClass = atoi(token);
            }
        }
        /*--------  s , listen channel  ----------------------*/
        else if(!strcmp(token, s_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                P2PParams->Data.cfgParams.listenChannel = atoi(token);
            }
        }
        /*--------  m , listen reg class  ----------------------*/
        else if(!strcmp(token, m_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                P2PParams->Data.cfgParams.listenClass = atoi(token);
            }
        }
        /*--------  -i <Go intent]  ----------------------*/
        else if(!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                P2PParams->Data.cfgParams.goIntent = atoi(token);
                if(P2PParams->Data.cfgParams.goIntent > 15)
                {
                    Report(
                        "\r\n [Cmd Parser] : P2P Go intent can have value from 0 to 15 "
                        "option. Using (15).\n\r");
                    P2PParams->Data.cfgParams.goIntent = 15;
                }
            }
        }

        else
        {
            Report("\r\n using default P2P configuration !!!" );
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        return(-1);
    }

    return(0);
}

/*!
    \brief          Parse p2p connect command.
=

    This routine fills the parameters for p2p connect from the command line
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          p2p connect params  -   Points to item  provided

                                       by the connect callback.
                                       This structure will later be read by
                                       the Role Up AP callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the set Start AP command help menu.

    \sa             cmdWlanRoleUpP2PCallback
 */
int32_t ParseP2PConnectCmd(void *arg,uint8_t* peer_mac, uint32_t* wps_method,char* pin)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    uint8_t help = FALSE;
    char    *pPin, *pMacAddress;

    /*
    char *wlan_p2p_connect_DetailsStr    = "P2P connect .\n\r";
    char wlan_p2p_connect_UsageStr_first[]   =   " [-help]";
    char wlan_p2p_connect_UsageStr_second[]  =   "[-m <peer_macAdress>]"
                                                 "[-w <wps_method [0 1 2]>] 0=PBC 1=PIN DISPLAY 2= PIN keypad"
                                                 "[-p <pin_code>]\n\r";

    */
    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }

        /*--------  m ----------------------*/
        else if(!strcmp(token, m_optionStr))
        {
            pMacAddress = strtok(NULL, space_str);
            macAddressParse(pMacAddress, peer_mac);
        }
        /*--------  w ----------------------*/
        else if(!strcmp(token, w_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                *wps_method = atoi(token);
            }
        }
        /*--------  p ----------------------*/
        else if(!strcmp(token, p_optionStr))
        {
            pPin = strtok(NULL, "\"");
            strcpy(pin,pPin);
        }

        else
        {
            Report("\r\n Error, need to enter configuration !!!" );
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        return(-1);
    }

    return(0);
}

/*!
    \brief          Parse Connection Policy Set command.

    This routine shows how to Set Connection Policy.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          ConnPolicySetParams  -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error


    \sa             cmdWlanSetConnPolicyCallback
 */
int32_t ParseConnPolicySetCmd(void *arg, WlanPolicySetGet_t *ConnPolicySetParams)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    // when no params default is used
    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }

        /*--------  a auto connect ----------------------*/
        else if(!strcmp(token, a_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                ConnPolicySetParams->autoPolicy = atoi(token);
            }

        }
        else if(!strcmp(token, f_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                ConnPolicySetParams->fastPolicy = atoi(token);
            }
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                ConnPolicySetParams->fastPersistant = atoi(token);
            }   
        }
        else
        {
            Report("\r\n Error, need to enter all configuration params !!!" );
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }
    
    if(help)
    {
        printWlanSetConnPolicyUsage(arg);
        return(-1);
    }
    return(0);
}



#endif

int32_t ParseSetWsocPrimaryCmd(void *arg, WlanConnectivityFWSlot_t *WsocSlot)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                WsocSlot->Connectivityslot = atoi((const char*) token);
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        return (-1);
    }

    return (0);
}

/*!
    \brief          Parse Cmd.

    This routine is a general command parser,
    used with commands that has no parameters to parse.
    such an example is 'wlandisconnect'.

    \param        arg         -   Points to command line buffer.
                                  Contains the command line typed by user.

    \return       Upon successful completion, the function shall return 0.
                  In case of failure, this function would return -1.
                  It's the callback's responsibility to check the return value,
                  and print the help for this command.

    \sa           cmdConnectCallback, cmdDisableFilterCallback,
                  cmdP2PModecallback, cmdP2PStopcallback
 */
int32_t ParseCmd(void *arg)
{
    char             *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            return (-1);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            return (-1);
        }
        token = strtok(NULL, " ");
    }
    return(0);
}

/*!
    \brief          Parse enable WowLan command.

    This routine takes a WoWLANEnableCmd_t structure,
    and fill it's content with
    parameters taken from command line.
    It checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg                 -   Points to command line buffer.
                                            Contains the command line
                                            typed by user.

    \param          WoWLANEnableParams  -   Points to command
                                            structure provided
                                            by the WowLan callback.
                                            This structure will later
                                            be read by the
                                            the WowLan callback.

    \return         Upon successful completion,
                    the function shall return 0.
                    In case of failure,
                    this function would print error, and show
                    the set enablewowlan command help menu.

    \sa             cmdEnableWoWLANCallback
 */
#if 0 // RazB - LWip implemntation
int32_t ParseEnableWoWLANCmd(void *arg,
                             WoWLANEnableCmd_t *WoWLANEnableParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char                          *token = NULL;
    uint16_t offset = 0;
    uint8_t actionID = 0;
    uint8_t help = FALSE;
    uint8_t wowStr = FALSE;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, (char*)help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, (char*)v_optionStr))
        {
            token = strtok(NULL, (char*)space_str);
            if(token)
            {
                if(*token != '"')
                {
                    Report(
                        "\n\r[cmd parser] : enablewowlan expects the pattern"
                        " value in quotation marks (/"
                        ").\r\n");
                    help = TRUE;
                    break;
                }

                token++;

                WoWLANEnableParams->rule.Header.Args.Value.Pattern.Length =
                    (strlen(token) - 1);
                if(WoWLANEnableParams->rule.Header.Args.Value.Pattern.Length <=
                   16)
                {
                    os_memcpy(
                        &WoWLANEnableParams->rule.Header.Args.Value.Pattern.
                        Value,
                        token,
                        WoWLANEnableParams->rule.Header.Args.Value.Pattern.
                        Length);
                }
                else
                {
                    SHOW_WARNING(-1, CMD_ERROR);
                    help = TRUE;
                    break;
                }
                /* This mask determines which filters would be triggered */
                memset(&WoWLANEnableParams->rule.Header.Args.Mask, 0xFF, 16);
                wowStr = TRUE;
            }
        }
        else if(!strcmp(token, (char*)i_optionStr))
        {
            token = strtok(NULL, (char*)space_str);

            if(token)
            {
                offset = (uint16_t)atol((const char*)token);
                WoWLANEnableParams->rule.Header.Args.Value.Pattern.Offset =
                    offset;
            }
            else
            {
                WoWLANEnableParams->rule.Header.Args.Value.Pattern.Offset = 0;
            }

            WoWLANEnableParams->rule.Header.Args.Value.Pattern.Reserved = 0;
        }
        else if(!strcmp(token, (char*)u_optionStr))
        {
            token = strtok(NULL, (char*)space_str);

            if(token)
            {
                /* this sets the action as Host event,
                and UserId sets the bit corresponding to the filter */
                WoWLANEnableParams->action.UserId = (uint8_t)atol(token);
                actionID = TRUE;
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, " ");
    }

    if(!wowStr || !actionID)
    {
        help = TRUE;
    }

    if(help)
    {
        printEnableWoWLANUsage(arg);
        return(-1);
    }

    return(0);
}
#endif

#ifdef CC35XX
/*!
    \brief          Parse Ping command.

    This routine takes a PingParams_t structure,
    and fill it's content with
    parameters taken from command line.
    It checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg         -   Points to command line buffer.
                                    Contains the command line typed by user.

    \param          pingParams  -   Points to command structure provided
                                    by the Ping callback.
                                    This structure will later be read
                                    by the the Ping callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,
                    this function would print error, and show
                    the set Ping command help menu.

    \sa             cmdPingStartCallback
 */
int32_t ParsePingCmd(void *arg,
                     PingParams_t *pingParams)
{
    int32_t n;
    char *token = NULL;
    char *targetIpAddress = NULL;
    char *sourceIpAddress = NULL;
    uint8_t help = FALSE;
    int8_t ret = 0;
    char cmdStr[CMD_BUFFER_LEN + 1];

    if (pingParams == NULL)
    {
        return -1;
    }

    /* Fill ping parameters with default values */
    pingParams->interval_time_ms = PING_DEFAULT_INTERVAL_TIME_MS;
    pingParams->payload_size     = PING_DEFAULT_DATA_SIZE;
    pingParams->count            = PING_DEFAULT_COUNT;
    pingParams->flags            = PING_PRINT_RESPONSES | PING_PRINT_REPORT;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (NULL == token)
    {
        printPingStartUsage(arg);
        return -1;
    }

    targetIpAddress = token;
    if (!targetIpAddress)
    {
        Report("\n\r[Cmd Parser]: Parser is expecting IP address\n\r");
        printPingStartUsage(arg);
        return -1;
    }
    else
    {
        ret = inet_pton(AF_INET, targetIpAddress, &pingParams->target_ip.ipv4);
        if (ret == 0)
        {
            Report("\n\r[Cmd Parser]: Invalid target IP address %s\n\r", targetIpAddress);
            printPingStartUsage(arg);
            return -1;
        }
        pingParams->target_ip.ipv4 = lwip_ntohl(pingParams->target_ip.ipv4);
    }

    token = strtok(NULL, " ");
    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if (!strcmp(token, h_optionStr))
        {
            targetIpAddress = strtok(NULL, space_str);
        }
        else if (!strcmp(token, c_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token != NULL)
            {
                n = atol(token);
                if (n >= 0)
                {
                    pingParams->count = n;
                }
                else
                {
                    Report("\n\r[Cmd Parser]: 'Num of packets' argument is "
                           "invalid. Using default: %d.\n\r",
                           PING_DEFAULT_COUNT);
                }
            }
        }
        else if (!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token != NULL)
            {
                n = atol(token);
                if ((n >= PING_INTERVAL_MIN_TIME_MS) &&
                    (n <= PING_INTERVAL_MAX_TIME_MS))
                {
                    pingParams->interval_time_ms = n;
                }
                else
                {
                    Report("\n\r[Cmd Parser]: 'delay interval' argument is "
                           "invalid. Using default: %d.\n\r",
                            PING_DEFAULT_INTERVAL_TIME_MS);
                }
            }
        }
        else if (!strcmp(token, s_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token != NULL)
            {
                n = atol(token);
                if ((n > 0) && (n <= PING_MAX_DATA_SIZE))
                {
                    pingParams->payload_size = n;
                }
                else
                {
                    Report("\n\r[Cmd Parser]: 'data size' argument is "
                           "invalid. Using default: %d.\n\r",
                           PING_DEFAULT_DATA_SIZE);
                }
            }
        }
        else if (!strcmp(token, I_optionStr))
        {
            sourceIpAddress = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (!sourceIpAddress)
    {
        /* Default behavior is to use any IP address */
        pingParams->source_ip.ipv4 = IPADDR_ANY;
    }
    else
    {
        ret = inet_pton(AF_INET, sourceIpAddress, &pingParams->source_ip.ipv4);
        if (ret == 0)
        {
            Report("\n\r[Cmd Parser]: Invalid source IP address\n\r");
            help = TRUE;
        }
        pingParams->source_ip.ipv4 = lwip_htonl(pingParams->source_ip.ipv4);
    }

    if (help)
    {
        printPingStartUsage(arg);
        return -1;
    }

    return 0;
}

/*!
    \brief          Parse Ping Stop command.

    This routine takes an integer, and fill it with
    parameters taken from command line. It sets some default values pre-parsing,
    and checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          session_id  -      Ping session id. 0 for printing all sessions.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.


 */
int32_t ParsePingStopCmd(void *arg, int8_t *sessionId)
{
    char *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if (!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token != NULL)
            {
                *sessionId = atoi(token);
                if (*sessionId < 0)
                {
                    Report("\n\r[Cmd Parser]: Invalid session id\n\r");
                    printPingStopUsage(arg);
                    return -1;
                }
            }
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }
    
    if (help)
    {
        printPingStopUsage(arg);
        return -1;
    }

    return(0);
}

/*!
    \brief          Parse wlan regulatory domain entry set command.

    This routine parses the regulatory domain set command line buffer.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          entryParams  -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error


    \sa             cmdWlanSetRegDomainEntryCallback
 */
int32_t ParseRegDomEntrySetCmd(void *arg, WlanSetRegDomainCustomEntry_t *entryParams)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    uint8_t help = FALSE;
    uint8_t indexSet = 0, txPowerSet = 0, reset = 0;
    uint8_t bmSet = 0, bandSet = 0, countSet = 0;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            help = TRUE;
            break;
        }
        else if (!strcmp(token, r_optionStr))
        {
            entryParams->resetEntry = 1;
            reset = 1;
        }
        else if (!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                entryParams->customIndex = atoi(token);
                indexSet = 1;
            }
        }
        else if (!strcmp(token, b_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                entryParams->band = atoi(token);
                bandSet = 1;
            }
        }
        else if (!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                entryParams->MaxTxPower = atoi(token);
                txPowerSet = 1;
            }
        }
        else if (!strcmp(token, d_optionStr))
        {
            entryParams->IsDfsChannel = atoi(token);
        }
        else if (!strcmp(token, n_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                entryParams->numOfChannels = atoi(token);
                countSet = 1;
            }
        }
        else if (!strcmp(token, bitmap_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                entryParams->chanBitmap = strtol(token, NULL, 16);
                bmSet = 1;
            }
        }
        else
        {
            Report("\n\r[Cmd Parser]: Unsupported option\n\r");
            return -1;
        }
        token = strtok(NULL, space_str);
    }
    
    if (help)
    {
        return -1;
    }

    if (!indexSet)
    {
        Report("\n\r[Cmd Parser]: Index parameter is mandatory\n\r");
        return -1;
    }

    /* Reset doesn't care about any other parameter other than index */
    if (reset)
    {
        return 0;
    }

    if (!txPowerSet)
    {
        Report("\n\r[Cmd Parser]: TX power parameter is mandatory\n\r");
        return -1;
    }

    if (!bmSet)
    {
        Report("\n\r[Cmd Parser]: Bitmap parameter is mandatory\n\r");
        return -1;
    }

    if (!bandSet)
    {
        Report("\n\r[Cmd Parser]: Band parameter is mandatory\n\r");
        return -1;
    }

    if (!countSet)
    {
        Report("\n\r[Cmd Parser]: Channel count parameter is mandatory\n\r");
        return -1;
    }

    return 0;
}

/*!
    \brief          Parse wlan regulatory domain entry get command.

    This routine parses the regulatory domain get command line buffer.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          entryParams  -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,this function would print error


    \sa             cmdWlanGetRegDomainEntryCallback
 */
int32_t ParseRegDomEntryGetCmd(void *arg, WlanSetRegDomainCustomEntry_t *entryParams)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, help_optionStr))
        {
            help = TRUE;
            break;
        }
        else if (!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                entryParams->customIndex = atoi(token);
            }
        }
        else
        {
            Report("\n\r[Cmd Parser]: Invalid switch used\n\r");
            return -1;
        }
        token = strtok(NULL, space_str);
    }
    
    if (help)
    {
        return (-1);
    }
    return(0);
}

#endif //CC35XX

#if 0
/*!
    \brief          Free Ping command.

    This routine takes a PingCmd_t structure, and free all memory that was
    allocated in ParsePingCmd.

    \param          pingParams  -   Points to command structure provided by the
                                    ping callback.

    \return         void

    \sa             cmdPingCallback
 */
void FreePingCmd(PingCmd_t *pingParams)
{
    if(pingParams->host != NULL)
    {
        os_free(pingParams->host);
        pingParams->host = NULL;
    }

    return;
}

/*!
    \brief          Parse mDNS advertise command.

    This routine takes a mDnsAdvertiseCmd_t structure,
    and fill it's content with
    parameters taken from command line. It checks the
    parameters validity.
    In case of a parsing error or invalid parameters,

    this function prints help menu.
    \param          arg                  -   Points to
                                             command line buffer.
                                             Contains the command
                                             line typed by user.

    \param          mDNSAdvertiseParams  -   Points to command structure
                                             provided by the mDNS
                                             advertise callback.
                                             This structure will later be
                                             read by the the mDNS
                                             advertise callback.

    \return         Upon successful completion,
                    the function shall return 0.
                    In case of failure, this function
                    would print error, and show
                    the set mDNS advertise command help menu.

    \sa             mDNSAdvertiseCallback
 */

int32_t ParsemDNSAdvertiseCmd(void *arg,
                              mDnsAdvertiseCmd_t *mDNSAdvertiseParams)
{
    int32_t length = -1;
    uint32_t help = FALSE;

    /* Fill default parameters */
    mDNSAdvertiseParams->service_port = DEF_ADVERTISE_PORT;
    mDNSAdvertiseParams->service_ttl = DEF_ADVERTISE_TTL;

    char            *service_name = NULL;
    char            *adv_text = NULL;
    char            *p_adv_service_over = NULL;
    char            *adv_service_type = NULL;
    char            *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, n_optionStr))
        {
            service_name = strtok(NULL, space_str);
        }
        else if(!strcmp(token, st_optionStr))
        {
            adv_service_type = strtok(NULL,space_str);
        }
        else if(!strcmp(token, so_optionStr))
        {
            p_adv_service_over = strtok(NULL, space_str);
        }
        else if(!strcmp(token, t_optionStr))
        {
            adv_text = strtok(NULL, "\"");
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                mDNSAdvertiseParams->service_port = (uint16_t)atoi(token);
            }
            else
            {
                Report(
                    "\n\r[Cmd Parser]: Service port was not set,"
                    " using default parameter: %d\n\r",
                    mDNSAdvertiseParams->service_port);
            }
        }
        else if(!strcmp(token, ttl_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                mDNSAdvertiseParams->service_ttl = (uint32_t)atoi(token);
            }
            else
            {
                Report(
                    "\n\r[Cmd Parser]: Service ttl was not set, "
                    "using default parameter: %d\n\r",
                    mDNSAdvertiseParams->service_ttl);
            }
            if(!mDNSAdvertiseParams->service_ttl)
            {
                mDNSAdvertiseParams->service_ttl = DEF_ADVERTISE_TTL;
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, " ");
    }

    /* Sanity check the parameters */
    if(help || (!p_adv_service_over)  || (!service_name) || (!adv_service_type))
    {
        printmDNSAdvertiseUsage(arg);
        return(-1);
    }

    /* Copy the text */
    if(adv_text)
    {
        mDNSAdvertiseParams->adv_text =
            (uint8_t *)os_zalloc(sizeof(uint8_t), strlen(
                                  adv_text));
        strcpy((char *)mDNSAdvertiseParams->adv_text, adv_text);
    }
    if(service_name)
    {
        mDNSAdvertiseParams->dev_name =
            (uint8_t *)os_zalloc(sizeof(uint8_t), strlen(
                                  service_name));
        strcpy((char *)mDNSAdvertiseParams->dev_name, service_name);
    }

    /* Compose the advertise address from it's parts:
    name, service type and protocol */
    length =
        mDNScreateServiceName(service_name, p_adv_service_over,
                              adv_service_type,
                              (char *)&mDNSAdvertiseParams->service_name);

    if(length < 0)
    {
        return(-1);
    }

    return(0);
}

/*!
    \brief          Free mDNS advertise command.

    This routine takes a mDnsAdvertiseCmd_t structure,
    and free all memory that was
    allocated in ParsemDNSAdvertiseCmd.

    \param          mDNSAdvertiseParams  -   Points to command structure
                                provided by the mDNSAdvertise callback.

    \return         void

    \sa             mDNSAdvertiseCallback
 */
void FreemDNSAdvertiseCmd(mDnsAdvertiseCmd_t *mDNSAdvertiseParams)
{
    if(mDNSAdvertiseParams->adv_text != NULL)
    {
        os_free(mDNSAdvertiseParams->adv_text);
        mDNSAdvertiseParams->adv_text = NULL;
    }

    if(mDNSAdvertiseParams->dev_name != NULL)
    {
        os_free(mDNSAdvertiseParams->dev_name);
        mDNSAdvertiseParams->dev_name = NULL;
    }

    return;
}

/*!
    \brief          Parse mDNS query command.

    This routine takes a mDnsQueryCmd_t structure, and fill it's content with
    parameters taken from command line. It checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg             -   Points to command line buffer.
                                        Contains the command line
                                        typed by user.

    \param          QueryCmdParams  -   Points to command structure provided
                                        by the mDNS query callback.
                                        This structure will later be read by
                                        the the mDNS query callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the mDNS query command help menu.

    \sa             mDNSQueryCallback
 */
int32_t ParsemDNSQueryCmd(void *arg,  mDnsQueryCmd_t *QueryCmdParams)
{
    char            *service_name[3] = {0};
    char            *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    int8_t help = FALSE;
    int32_t ret;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, n_optionStr))
        {
            service_name[0] = strtok(NULL, space_str);
        }
        else if(!strcmp(token, st_optionStr))
        {
            service_name[1] = strtok(NULL, space_str);
        }
        else if(!strcmp(token, so_optionStr))
        {
            service_name[2] = strtok(NULL, space_str);
        }
        else if(!strcmp(token, o_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                if(!strcmp(token, "YES") || !strcmp(token, "yes"))
                {
                    QueryCmdParams->OneShotFlag = TRUE;
                }
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }

        token = strtok(NULL, " ");
    }

    /* Perform input validation */
    if((help) || (service_name[1] == NULL) || (service_name[2] == NULL) ||
       (!IS_CONNECTED(app_CB.Status)))
    {
        printmDNSQueryUsage(arg);
        return(-1);
    }

    /* Compose the mDNS service name */
    ret =
        mDNScreateServiceName(
        service_name[0],
        service_name[2],
        service_name[1],
        (char *)&QueryCmdParams->ServiceName);

    if(ret <= 0)
    {
        printmDNSQueryUsage(arg);
        return(-1);
    }

    return(0);
}

/*!
    \brief          Compose mDNS advertise address.

    This routine takes individual strings, which represent
    each part of a mDNS advertise address
    and put them together in order to create one mDNS address.
    The Address is null terminated and allocated
    by the caller.

    \param          p_sevName           -   Points to a string
                                            containing device name. ex:'PC1'

    \param          p_servOver          -   Points to a string containing
                                            protocol for this service. ex:'tcp'

    \param          p_servType          -   Points to a string containing
                                            service type. ex: 'http'

    \param          p_adv_service_name  -   Container to store the
                                            advertise service name.
                                            mDNS service names
                                            are of the following form:
                                            'PC1._http._tcp.local'.

    \return         Upon successful completion, the function shall return
                    the length of the composed name,
                    and place it in the provided container. The caller is
                    responsible to maintain this container.

    \note           mDNS advertise length, is max MAX_SERVICE_NAME_LENGTH
                    (63) 64 including null termination character.
                    Also, this function doesn't check for input validity,
                    since all caller functions provide valid parameters.

    \sa             ParsemDNSAdvertiseCmd, ParsemDNSQueryCmd
 */
int32_t mDNScreateServiceName(char *p_sevName,
                              char *p_servOver,
                              char *p_servType,
                              char *p_adv_service_name)
{
    int32_t str_len = -1;
    char        *point = "._";
    char        *space = "_";
    char        *spoint = ".";
    char        *local = "local";

    /* query can be partial - without name.
     *  for example: _http._tcp.local -
     *  find available http servers over tcp
     */
    if(p_sevName != NULL)
    {
        str_len = strlen(p_sevName) + strlen(spoint);
    }

    str_len += (strlen(p_servOver) + strlen(p_servType) + strlen(local) + 5);

    memset(p_adv_service_name, 0x0, str_len);

    if(p_sevName != NULL)
    {
        strcat(p_adv_service_name, p_sevName);
        strcat(p_adv_service_name, (const char*)spoint);
    }

    strcat(p_adv_service_name, (const char*)space);
    strcat(p_adv_service_name, (const char*)p_servType);
    strcat(p_adv_service_name, (const char*)point);
    strcat(p_adv_service_name, (const char*)p_servOver);
    strcat(p_adv_service_name, (const char*)spoint);
    strcat(p_adv_service_name, (const char*)local);
    strcat(p_adv_service_name, (const char*)"\0");

    return(str_len);
}
#endif
/*!
    \brief          Parse Send command.

    This routine takes a SendCmd_t structure,
    and fill it's content with
    parameters taken from command line.
    It checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg             -   Points to command line buffer.
                                        Contains the command
                                        line typed by user.

    \param          SendCmdParams   -   Points to command structure
                                        provided by the Send callback.
                                        This structure will later be
                                        read by the the Send callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure,
                    this function would print error, and show
                    the Send command help menu.

    \sa             cmdSendCallback
 */

int32_t ParseSendCmd(void *arg,  SendCmd_t *SendCmdParams)
{
    int32_t ret = 0;
    int8_t help = FALSE;
    char                    *ip = NULL;
    char                    *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint32_t defaultBandwidth = 25/*Mbps*/;


    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    /* Fill default to structures */
    SendCmdParams->numberOfPackets = 1000;
    SendCmdParams->udpFlag = FALSE;
    SendCmdParams->server = FALSE;
    SendCmdParams->nb = FALSE;
    SendCmdParams->ipv6 = FALSE;
    SendCmdParams->bandwidth = defaultBandwidth;
    uint8_t *clientIp = NULL;

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, u_optionStr))
        {
            SendCmdParams->udpFlag = TRUE;
        }
        else if(!strcmp(token, V_optionStr))
        {
            SendCmdParams->ipv6 = TRUE;
        }
        else if(!strcmp(token, nb_optionStr))
        {
            SendCmdParams->nb = TRUE;
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                SendCmdParams->destOrLocalPortNumber = (unsigned short)atol(token);
            }
            else
            {
                Report(
                    "\n\r[cmd Parser] : Port number was not set, "
                    "using default parameter: %d\n\r",
                    SendCmdParams->destOrLocalPortNumber);
            }
        }
        else if(!strcmp(token, c_optionStr))
        {
            ip = (char*)strtok(NULL, space_str);
            if((ip != NULL) && (clientIp == NULL))
            {

                clientIp = os_malloc(strlen((const char*)ip));

                os_memcpy(clientIp, ip, strlen((const char*)ip));
            }
            else
            {
                Report("\n\r[cmd Parser] : Invalid IP\n\r");
                return(-1);
            }

            SendCmdParams->server = FALSE;
        }
        else if(!strcmp(token, n_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                SendCmdParams->numberOfPackets = atol(token);
            }
            else
            {
                Report(
                    "\n\r[cmd Parser] : Number Of Packets was not set, "
                    "using default parameter: %d\n\r",
                    SendCmdParams->numberOfPackets);
            }
        }
        else if(!strcmp(token, b_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                SendCmdParams->bandwidth = atol(token);
            }
            else
            {
                Report(
                    "\n\r[cmd Parser] : Bandwidth was not set, "
                    "using default parameter: %d\n\r",
                    SendCmdParams->bandwidth);
            }
        }

        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        ret = printSendUsage(arg);
        return(-1);
    }

    if(!SendCmdParams->server)
    {
        token = NULL;
        token = strchr((const char*)clientIp, ':');
        if(token)
        {
            ret = ipv6AddressParse((char*)clientIp,
                                   (uint8_t*)&SendCmdParams->ipAddr.ipv6);
            SendCmdParams->ipv6 = TRUE;
        }
        else
        {
            ret = ipv4AddressParse((char*)clientIp,
                                   (uint32_t*)&SendCmdParams->ipAddr.ipv4);
            SendCmdParams->ipv6 = FALSE;
        }
        os_free(clientIp);
    }

    if(ret < 0)
    {
        Report("\n\r[cmd Parser] : Invalid IP\n\r");
        return(-1);
    }

    /* Perform a sanity check, since the device has
    to be connected in order to send packets. */
    if(SendCmdParams->ipv6)
    {
        if(!IS_IPV6G_ACQUIRED(app_CB.Status) &&
           !IS_IPV6L_ACQUIRED(app_CB.Status) && 
           !IS_STA_CONNECTED(app_CB.Status)  &&
           !IS_AP_CONNECTED(app_CB.Status))
        {
            Report(
                "\n\r[cmd Parser] : (error) Cannot send data if device is "
                "disconnected from network.\n\r");
            return(-1);
        }
    }
    else
    {
        if(!IS_IP_ACQUIRED(app_CB.Status)   && 
           !IS_STA_CONNECTED(app_CB.Status) &&
           !IS_AP_CONNECTED(app_CB.Status))
        {
            Report(
                "\n\r[cmd Parser] : (error) Cannot send data if device is "
                "disconnected from network.\n\r");
            return(-1);
        }
    }

    if(SendCmdParams->numberOfPackets < 0)
    {
        SendCmdParams->numberOfPackets = 1000;
    }

    if(SendCmdParams->destOrLocalPortNumber <= 0)
    {
        SendCmdParams->destOrLocalPortNumber = 5001;
    }

    Report("\n\r");
    Report("------------------------------------------------------------\n\r");
    Report("Protocol: %s\n\r", SendCmdParams->udpFlag ? "udp" : "tcp");
    Report("Traffic: Tx\n\r");
    Report("Client\\Server: ");

    if(SendCmdParams->server)
    {
        Report("Server\n\r");
    }
    else
    {
        Report("Client\n\r");

        if(SendCmdParams->ipv6)
        {
            PrintIPAddress(SendCmdParams->ipv6,
                           (void*)&SendCmdParams->ipAddr.ipv6);
            Report(lineBreak);
        }
        else
        {
            PrintIPAddress(SendCmdParams->ipv6,
                           (void*)&SendCmdParams->ipAddr.ipv4);
            Report(lineBreak);
        }
    }

    Report("Port: %d\n\r", SendCmdParams->destOrLocalPortNumber);
    if(SendCmdParams->numberOfPackets)
    {
        Report("Number of Packets to send: %d\n\r",
                   SendCmdParams->numberOfPackets);
    }
    else
    {
        Report("Number of Packets to send: INFINITY \n\r");
    }
    Report("------------------------------------------------------------\n\r");
    Report("\n\r");

    return(0);
}


/*!
    \brief          Parse Receive command.

    This routine takes a RecvCmd_t structure, and fill it's content with
    parameters taken from command line. It checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg             -   Points to command line buffer.
                                        Contains the command line
                                        typed by user.

    \param          RecvCmdParams   -   Points to command structure provided
                                        by the Recv callback.
                                        This structure will later be read
                                        by the the Recv callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the Send command help menu.

    \sa             cmdRecvCallback
 */
int32_t ParseRecvCmd(void *arg,  RecvCmd_t *RecvCmdParams)
{
    int32_t ret = 0;
    int8_t help = FALSE;
    char                    *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);


    /* Fill default to structures */
    RecvCmdParams->numberOfPackets = 1000;
    RecvCmdParams->udpFlag = FALSE;
    RecvCmdParams->server = TRUE;
    RecvCmdParams->nb = FALSE;
    RecvCmdParams->ipv6 = FALSE;
    RecvCmdParams->destOrLocalPortNumber = 5001;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, u_optionStr))
        {
            RecvCmdParams->udpFlag = TRUE;
        }
        else if(!strcmp(token, s_optionStr))
        {
            RecvCmdParams->server = TRUE;
        }
        else if(!strcmp(token, V_optionStr))
        {
            RecvCmdParams->ipv6 = TRUE;
        }
        else if(!strcmp(token, nb_optionStr))
        {
            RecvCmdParams->nb = TRUE;
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RecvCmdParams->destOrLocalPortNumber = (unsigned short)atol(token);
            }
            else
            {
                Report(
                    "\n\r[cmd Parser] : Port number was not set, using default"
                    " parameter: %d\n\r",
                    RecvCmdParams->destOrLocalPortNumber);
            }
        }
        else if(!strcmp(token, n_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                RecvCmdParams->numberOfPackets = atol(token);
            }
            else
            {
                Report(
                    "\n\r[cmd Parser] : Number Of Packets was not set, using"
                    " default parameter: %d\n\r",
                    RecvCmdParams->numberOfPackets);
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        printRecvUsage(arg);
        return(-1);
    }


    /* Perform a sanity check,
       since the device has to be connected in order to send packets. */
    if(RecvCmdParams->ipv6)
    {
        if(!IS_IPV6G_ACQUIRED(app_CB.Status) &&
           !IS_IPV6L_ACQUIRED(app_CB.Status) && 
           !IS_STA_CONNECTED(app_CB.Status)  &&
           !IS_AP_CONNECTED(app_CB.Status))
        {
            Report(
                "\n\r[cmd Parser] : (error) Cannot receive data if device"
                " is disconnected from network.\n\r");
            return(-1);
        }
    }
    else
    {
        if(!IS_IP_ACQUIRED(app_CB.Status)   && 
           !IS_STA_CONNECTED(app_CB.Status) &&
           !IS_AP_CONNECTED(app_CB.Status))
        {
            Report(
                "\n\r[cmd Parser] : (error)Cannot send data if device"
                " is disconnected from network.\n\r");
            return(-1);
        }
    }

    if(ret < 0)
    {
        Report("\n\r[cmd Parser] : Invalid IP\n\r");
        return(-1);
    }

    if(RecvCmdParams->numberOfPackets < 0)
    {
        RecvCmdParams->numberOfPackets = 1000;
    }

    if(RecvCmdParams->destOrLocalPortNumber <= 0)
    {
        RecvCmdParams->destOrLocalPortNumber = 5001;
    }

    Report("\n\r");
    Report("------------------------------------------------------------\n\r");
    Report("Protocol: %s\n\r", RecvCmdParams->udpFlag ? "udp" : "tcp");
    Report("Traffic: Rx\n\r");
    Report("Client\\Server: ");

    if(RecvCmdParams->server)
    {
        Report("Server\n\r");
    }
    else
    {
        Report("Client\n\r");
        Report("Client IP: ");

        if(RecvCmdParams->ipv6)
        {
            PrintIPAddress(RecvCmdParams->ipv6,
                           (void*)&RecvCmdParams->ipAddr.ipv6);
        }
        else
        {
            PrintIPAddress(RecvCmdParams->ipv6,
                           (void*)&RecvCmdParams->ipAddr.ipv4);
        }

        Report(lineBreak);
    }

    Report("Port: %d\n\r", RecvCmdParams->destOrLocalPortNumber);
    if(RecvCmdParams->numberOfPackets)
    {
        Report("Number of Packets to receive: %d\n\r",
                   RecvCmdParams->numberOfPackets);
    }
    else
    {
        Report("Number of Packets to receive: INFINITY \n\r");
    }
    Report("------------------------------------------------------------\n\r");
    Report("\n\r");

    return(0);
}


/*!
    \brief          Parse Receive command.

    This routine takes a RecvCmd_t structure, and fill it's content with
    parameters taken from command line. It checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg             -   Points to command line buffer.
                                        Contains the command line
                                        typed by user.

    \param          RecvCmdParams   -   Points to command structure provided
                                        by the Recv callback.
                                        This structure will later be read
                                        by the the Recv callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the Send command help menu.

    \sa             cmdRecvCallback
 */
int32_t ParseTestIperfCmd(void *arg,  RecvCmd_t *IperfCmdParams)
{
    int32_t ret = 0;
    int8_t help = FALSE;
    char *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t *clientIp = NULL;
    uint8_t *serverIp = NULL;//default any ip address
    char *ip = NULL;


    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    /* Fill default to structures */
    IperfCmdParams->udpFlag = FALSE;
    IperfCmdParams->server = FALSE;
    IperfCmdParams->ipv6 = FALSE;
    IperfCmdParams->destOrLocalPortNumber = 5001;
    IperfCmdParams->period = 0;
    IperfCmdParams->timeout = 99999;//endless
    IperfCmdParams->bandwidth =0;


    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, s_optionStr))
        {
            IperfCmdParams->server = TRUE;
        }
        else if(!strcmp(token, c_optionStr))
        {
            ip = (char*)strtok(NULL, space_str);
            if((ip != NULL) && (clientIp == NULL))
            {

                clientIp = os_malloc(strlen((const char*)ip));

                os_memcpy(clientIp, ip, strlen((const char*)ip));
            }
            else
            {
                Report("\n\r[cmd Parser] : Invalid IP\n\r");
                return(-1);
            }

            IperfCmdParams->server = FALSE;
        }
        else if(!strcmp(token, B_optionStr))
        {
            ip = (char*)strtok(NULL, space_str);
            if((ip != NULL) && (serverIp == NULL))
            {

                serverIp = os_malloc(strlen((const char*)ip));

                os_memcpy(serverIp, ip, strlen((const char*)ip));
            }
            else
            {
                Report("\n\r[cmd Parser] : Invalid IP\n\r");
                return(-1);
            }
            IperfCmdParams->server = TRUE;

        }
        else if(!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                IperfCmdParams->period = (unsigned char)atoi(token);
            }
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                IperfCmdParams->destOrLocalPortNumber = (unsigned short)atoi(token);
            }
            else
            {
                Report(
                    "\n\r[cmd Parser] : Port number was not set, using default"
                    " parameter: %d\n\r",
                    IperfCmdParams->destOrLocalPortNumber);
            }
        }
        else if(!strcmp(token, t_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                IperfCmdParams->timeout = (uint32_t)atol(token);
            }
        }
        else if(!strcmp(token, u_optionStr))
        {
            IperfCmdParams->udpFlag = TRUE;
        }
        else if(!strcmp(token, b_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                IperfCmdParams->bandwidth = atol(token);
            }
        }

        token = strtok(NULL, space_str);
    }


    if(help)
    {
        printTestIperfUsage(arg);
        return(-1);
    }

    if(IperfCmdParams->ipv6)
    {
        if(!IS_IPV6G_ACQUIRED(app_CB.Status) &&
           !IS_IPV6L_ACQUIRED(app_CB.Status) &&
           !IS_STA_CONNECTED(app_CB.Status)  &&
           !IS_AP_CONNECTED(app_CB.Status))
        {
            Report(
                "\n\r[cmd Parser] : (error) Cannot send data if device is "
                "disconnected from network.\n\r");
            return(-1);
        }
    }
    else
    {
        if(!IS_IP_ACQUIRED(app_CB.Status)   &&
           !IS_STA_CONNECTED(app_CB.Status) &&
           !IS_AP_CONNECTED(app_CB.Status))
        {
            Report(
                "\n\r[cmd Parser] : (error)Cannot send data if device"
                " is disconnected from network.\n\r");
            return(-1);
        }
    }

    if(IperfCmdParams->server)
    {
        token = NULL;
        if(serverIp == NULL)
        {
            IperfCmdParams->ipAddr.ipv4 = IPADDR_ANY;
        }
        else
        {
            token = strchr((const char*)serverIp, ':');
            if(token)
            {
                ret = ipv6AddressParse((char*)serverIp,
                                       (uint8_t*)&IperfCmdParams->ipAddr.ipv6);
                IperfCmdParams->ipv6 = TRUE;
            }
            else
            {
                ret = ipv4AddressParse((char*)serverIp,
                                       (uint32_t*)&IperfCmdParams->ipAddr.ipv4);
                IperfCmdParams->ipv6 = FALSE;
            }
            os_free(serverIp);
        }
    }

    else
    {
        token = NULL;
        if(clientIp == NULL)
        {
            Report("\n\r[cmd Parser] : (error) client ip is not defined.\n\r");
            return -1;
        }
        token = strchr((const char*)clientIp, ':');
        if(token)
        {
            ret = ipv6AddressParse((char*)clientIp,
                                   (uint8_t*)&IperfCmdParams->ipAddr.ipv6);
            IperfCmdParams->ipv6 = TRUE;
        }
        else
        {
            ret = ipv4AddressParse((char*)clientIp,
                                   (uint32_t*)&IperfCmdParams->ipAddr.ipv4);
            IperfCmdParams->ipv6 = FALSE;
        }
        os_free(clientIp);
    }

    if(ret < 0)
    {
        Report("\n\r[cmd Parser] : Invalid IP\n\r");
        return(-1);
    }


    Report("\n\r");
    Report("------------------------------------------------------------\n\r");
    Report("Protocol: %s\n\r", IperfCmdParams->udpFlag ? "udp" : "tcp");
    Report("Client\\Server: ");

    if(IperfCmdParams->server)
    {
        Report("Server\n\r");
        Report("Server IP: ");

    }
    else
    {
        Report("Client\n\r");
        Report("Client IP: ");
    }


    if(IperfCmdParams->ipv6)
    {
        PrintIPAddress(IperfCmdParams->ipv6,
                       (void*)&IperfCmdParams->ipAddr.ipv6);
    }
    else
    {
        PrintIPAddress(IperfCmdParams->ipv6,
                       (void*)&IperfCmdParams->ipAddr.ipv4);
    }

    Report(lineBreak);
    Report("Dest Port: %d\n\r", IperfCmdParams->destOrLocalPortNumber);
    Report("timeout: %ld\n\r", IperfCmdParams->timeout);
    Report("period: %ld\n\r", IperfCmdParams->period);
    Report("bandwidth: %ld Mbps\n\r", IperfCmdParams->bandwidth);
    Report("------------------------------------------------------------\n\r");
    Report("\n\r");

    return(0);
}

/*!
    \brief          Parse Set Country Code command.

    This routine parse set country code parameters.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg             -   Points to command line buffer.
                                        Contains the command line typed by
                                        user.
                    InputPad        -   Pad number to assign for BLE input
                    OutputPad       -   Pad number to assign for RF switch
                                        control

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                     and show the CoexEnable command help menu.

    \sa             ParseCountycodeCmd
 */

int32_t ParseStopTestIperfCmd(void *arg, stopCmd_t *stopCmd)
{
    char               *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    stopCmd->processNum = 0;

    if(NULL == token)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, n_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                stopCmd->processNum = (uint32_t)atoi(token);;
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, " ");
    }

    if(help)
    {
        printStopTestIperfUsage(arg);
        return(-1);
    }

    return(0);
}





/*!
    \brief          Parse Set Country Code command.

    This routine parse set country code parameters.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg             -   Points to command line buffer.
                                        Contains the command line typed by
                                        user.
                    InputPad        -   Pad number to assign for BLE input
                    OutputPad       -   Pad number to assign for RF switch
                                        control

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                     and show the CoexEnable command help menu.

    \sa             ParseCountycodeCmd
 */

int32_t ParseCountycodeCmd(void *arg, uint8_t *country)
{
    char               *token = NULL;
    uint8_t            *country_input = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(NULL == token)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, g_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                country_input = (uint8_t *)token;
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, " ");
    }

    if(!country_input)
    {
        SHOW_WARNING(-1, CMD_ERROR);
        help = TRUE;
    }
    else
    {
        if(!((strlen((char *)country_input) == 2) &&
             (country_input[0] >= 'A') && (country_input[0] <= 'Z') &&
             (country_input[1] >= 'A') && (country_input[1] <= 'Z')))
        {
            Report("\n\rInvalid input \n\r");
            help = TRUE;
        }
    }

    if(help)
    {
        printCountrycodeeUsage(arg);
        return(-1);
    }

    os_memcpy(country, country_input, 2);
    return(0);
}


/*!
    \brief          Parse Triggered Roaming Enable command.

    This routine takes the RssiThreshold parameter, and fill it's content with
    parameters taken from command line. It checks the parameters validity.
    In case of a parsing error or invalid parameters, this function
    prints help menu.

    \param          arg             -   Points to command line buffer.
                                        Contains the command line
                                        typed by user.

    \param          RssiThreshold   -   Input event Threshold.
                                        Units: dBm / dB ; Range: (-85 .. 0)

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the TriggeredRoamingEnableEnable command help menu.

    \sa             cmdTriggeredRoamingEnablecallback
 */
int32_t ParseTriggeredRoamingEnableCmd(void *arg, int16_t *RssiThreshold)
{
    char    *token = NULL;
    char    cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    *RssiThreshold = -63;
    token = strtok(cmdStr, space_str);

    if(NULL == token)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, r_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                *RssiThreshold = (int32_t)atoi(token);
                if ( (*RssiThreshold < -85) ||
                     (*RssiThreshold > 0) )
                {
                    Report("\n\rInvalid input \n\r");
                    help = TRUE;
                }
            }
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, " ");
    }

    if(help)
    {
        printTriggeredRoamingEnableUsage(arg);
        return(-1);
    }

    return(0);
}

/*!
\brief          Parse Set Interface IP command.

   
\param          arg            -   Points to command line buffer.
                                   Contains the command line typed by user.

\param          SetInterfaceIpParams_t  -   Points to command structure provided
                                   by the set Interface IP callback.
                                   This structure will later be read by
                                   the set Interface IP callback.

\return         Upon successful completion, the function shall return 0.
                In case of failure, this function would print error,
                and show the command help menu.

\sa             cmdSetInterfaceIpCallback
*/
int32_t ParseSetInterfaceIpCmd(void *arg, SetInterfaceIpParams_t *params)
{
    char     cmdStr[CMD_BUFFER_LEN + 1];
    char     *token;
    char     *strRoleId = NULL;
    char     *ipMode = NULL;
    char     *ip = NULL;
    char     *gateway = NULL;
    char     *netmask = NULL;
    uint8_t help = FALSE;
    int32_t ret;
    params->setDhcpServerAddress = TRUE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);
    if(token == NULL)
    {
        help = TRUE;
    }
    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        /*  set roleID*/
        else if (!strcmp(token, i_optionStr))
        {
            strRoleId = strtok(NULL, space_str);
            params->roleType = atoi(strRoleId);
        }
        /*  set ipMode*/
        else if(!strcmp(token, ip_optionStr))
        {
            ipMode = strtok(NULL, space_str);
            params->ipMode =atoi(ipMode);
        }
        /*  set ipAddress  */
        else if(!strcmp(token, c_optionStr))
        {        
            ip = strtok(NULL, space_str);
        }
        
        /*  set Netmask  */
        else if(!strcmp(token, v_optionStr))
        {
        netmask = (char*)strtok(NULL, space_str);

        }
        /*  set Gateway  */
        else if(!strcmp(token, gw_optionStr))
        {
            gateway = strtok(NULL, space_str);
        }

        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    ret = ipv4AddressParse(ip, &params->ipAddress);
    if (ret == -1)
    {
        params->setDhcpServerAddress = FALSE;
    }

    ret = ipv4AddressParse(gateway, &params->gateway);
    if (ret == -1)
    {
        params->setDhcpServerAddress = FALSE;
    }

    ret = ipv4AddressParse(netmask, &params->netmask);
    if (ret == -1)
    {
        params->setDhcpServerAddress = FALSE;
    }

    if(help)
    {
        return -1;
    }
    return 0;
}
/*!
\brief          Parse Get Interface IP command.

\param          arg            -   Points to command line buffer.
                                   Contains the command line typed by user.

\param          SetInterfaceIpParams_t  -   Points to command structure provided
                                   by the get Interface IP callback.
                                   This structure will later be read by
                                   the set Interface IP callback.

\return         Upon successful completion, the function shall return 0.
                In case of failure, this function would print error,
                and show the command help menu.

\sa             cmdGetInterfaceIpCallback
*/
int32_t ParseGetInterfaceIpCmd(void *arg, WlanRole_e *RoleId)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strRoleId = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if (!strcmp(token, i_optionStr))
        {
            strRoleId = strtok(NULL, space_str);
        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        return (-1);
    }

    *RoleId = atoi((const char*) strRoleId);

    //check if role id valid
    if ((*RoleId != WLAN_ROLE_STA) && (*RoleId != WLAN_ROLE_AP))
    {
        Report("\r\n[Cmd Parser] : Invalid RoleId\n\r");
        return (-1);
    }
    return (0);
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
int32_t ParseSetDhcpServerCmd(char *arg,
                                uint32_t *leaseTime,
                                uint32_t *startAddress,
                                uint32_t *endAddress)
{
    char     cmdStr[CMD_BUFFER_LEN + 1];
    char     *token;
    char     *s_address = NULL;
    char     *e_address = NULL;
    uint8_t help = FALSE;
    int32_t ret;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);
    if(token == NULL)
    {
        help = TRUE;
    }
    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        /* lease time */
        else if (!strcmp(token, t_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token == NULL || token[0] == '-')
            {
                Report("\n\rError: Lease time (-t) parameter is required and must be greater than 0\n\r");
                return -1;
            }
            *leaseTime = atoi(token);
        }
        /* IPv4 start address */
        else if(!strcmp(token, s_optionStr))
        {
            s_address = strtok(NULL, space_str);
        }
        /* IPv4 end address */
        else if(!strcmp(token, e_optionStr))
        {
            e_address = strtok(NULL, space_str);
        }
        token = strtok(NULL, space_str);
    }

    if (*leaseTime == 0)
    {
        Report("\n\rError: Lease time (-t) parameter is required and must be greater than 0\n\r");
        return -1;
    }
    ret = ipv4AddressParse(s_address, startAddress);
    if (ret == -1)
    {
        return -1;
    }

    ret = ipv4AddressParse(e_address, endAddress);
    if (ret == -1)
    {
        return -1;
    }

    if(help)
    {
        return -1;
    }

    return 0;
}
/*!
    \brief          Parse Role Down AP command.

    This routine takes a RoleDownApCmd_t structure, and fill it's content with
    parameters taken from command line. It checks the parameters validity.
    In case of a parsing error or invalid parameters,
    this function prints help menu.

    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.

    \param          RoleDownApParams - Points to command structure provided
                                       by the connect callback.
                                       This structure will later be read by
                                       the Role Down AP callback.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would print error,
                    and show the set Start AP command help menu.

    \sa             printWlanRoleDownApUsage
 */
//int32_t ParseRoleDownApCmd(void *arg, RoleDownApCmd_t *RoleDownApParams)
//{
//    char    cmdStr[CMD_BUFFER_LEN + 1];
//    char    *token;
//    char    *ssid = NULL;
//
//    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
//    cmdStr[CMD_BUFFER_LEN] = '\0';
//    token = strtok(cmdStr, space_str);
//
//    if(token == NULL)
//    {
//        help = TRUE;
//    }
//
//    while(token)
//    {
//        if(!strcmp(token, help_optionStr))
//        {
//            help = TRUE;
//        }
//        else if(!strcmp(token, s_optionStr))
//        {
//            ssid = strtok(NULL, "\"");
//        }
//        else
//        {
//            SHOW_WARNING(-1, CMD_ERROR);
//            help = TRUE;
//            break;
//        }
//        token = strtok(NULL, space_str);
//    }
//    if(help)
//    {
//        printWlanRoleDownApUsage(arg);
//        return(-1);
//    }
//
//    if((NULL == ssid) || (strlen(ssid) >= SL_WLAN_SSID_MAX_LENGTH))
//    {
//        Report(
//            "\r\n [Cmd Parser] : invalid Parameter for SSID - Should be max"
//            " 31 characters.\n\r");
//        printWlanRoleDownApUsage(arg);
//        return(-1);
//    }
//    else
//    {
//        RoleDownApParams->ssid = (uint8_t *)os_zalloc(strlen(ssid)+1,sizeof(uint8_t));
//        strcpy((char *)StartApParams->ssid, ssid);
//    }
//
//    return(0);
//}


/*!
    \brief          Parse BLE advertise configure command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          advParams      -   Points to command structure provided
                                       by the AdvCfg callback.
                                       This structure will later be read
                                       by the AdvCfg callback.

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleAdvCfgCallback
 */
int32_t ParseBleAdvCfgCmd(void *arg, ExtAdvCfg_t *advParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    //Set Defaults
    advParams->instance = 0;
    advParams->legacy = 1;
    advParams->interval_ms = 100;
    advParams->prim_phy = 0x1;
    advParams->sec_phy = 0x1;

    while (token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advParams->instance = atoi(token);
            }
        }
        else if(!strcmp(token, l_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advParams->legacy = atoi(token);
            }
        }
        else if(!strcmp(token, n_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advParams->interval_ms = atoi(token);
            }
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advParams->prim_phy = atoi(token);
            }
        }
        else if(!strcmp(token, s_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advParams->sec_phy = atoi(token);
            }
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printBleAdvCfgUsage(NULL);
        return (-1);
    }

    return (0);
}

/*!
    \brief          Parse BLE advertise enable command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          advParams      -   Points to command structure provided
                                       by the AdvEnable callback.
                                       This structure will later be read
                                       by the AdvEnable callback.

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleAdvEnableCallback
 */
int32_t ParseBleAdvEnableCmd(void *arg, ExtAdvEnable_t *advEnable)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    //Set Defaults
    advEnable->enable = 1;
    advEnable->instance = 0;
    advEnable->duration = 0;
    advEnable->max_events = 0;

    while (token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, e_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advEnable->enable = atoi(token);
            }
        }
        else if(!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advEnable->instance = atoi(token);
            }
        }
        else if(!strcmp(token, d_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advEnable->duration = atoi(token);
            }
        }
        else if(!strcmp(token, m_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                advEnable->max_events = atoi(token);
            }
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printBleAdvEnableUsage(NULL);
        return (-1);
    }

    return (0);
}

/*!
    \brief          Parse BLE scan configure command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          scanParams     -   Points to command structure provided
                                       by the scanCfg callback.
                                       This structure will later be read
                                       by the scanCfg callback.

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleScanCfgCallback
 */
int32_t ParseBleScanCfgCmd(void *arg, ExtScanCfg_t *scanParams)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    //Set Defaults
    scanParams->scan_interval_ms = 100;
    scanParams->scan_window_ms = 50;
    scanParams->filter_policy = 0;
    scanParams->own_address_type = 0;
    scanParams->scan_phy = 0x1;
    scanParams->scan_type = 0x1;

    while (token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, i_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                scanParams->scan_interval_ms = atoi(token);
            }
        }
        else if(!strcmp(token, w_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                scanParams->scan_window_ms = atoi(token);
            }
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                scanParams->scan_phy = atoi(token);
            }
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printBleScanCfgUsage(NULL);
        return (-1);
    }

    return (0);
}

/*!
    \brief          Parse BLE scan enable command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          scanEnable     -   Points to command structure provided
                                       by the scanEnable callback.
                                       This structure will later be read
                                       by the sanEnable callback.

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleScanEnableCallback
 */
int32_t ParseBleScanEnableCmd(void *arg, ExtScanEnable_t *scanEnable)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    uint8_t help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    //Set Defaults
    scanEnable->enable = 1;
    scanEnable->filter_duplicate = 1;
    scanEnable->duration = 300; //3 seconds
    scanEnable->period = 0;

    while (token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, e_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                scanEnable->enable = atoi(token);
            }
        }
        else if(!strcmp(token, p_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                scanEnable->period = atoi(token);
            }
        }
        else if(!strcmp(token, f_optionStr))
        {
            token = strtok(NULL, space_str);
            if (token)
            {
                scanEnable->filter_duplicate = atoi(token);
            }
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        printBleScanEnableUsage(NULL);
        return (-1);
    }

    return (0);
}

/*!
    \brief          Parse BLE connect command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          bd_addr        -   Points to BD address to connect to
    \param          addr_type      -   Points to address type to connect to

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleConnectCallback
 */
int32_t ParseBleConnectCmd(void *arg, uint8_t *bd_addr, uint8_t* addr_type)
{
    char       cmdStr[CMD_BUFFER_LEN + 1];
    char       *token = NULL;
    char       *bdAddressStr = NULL;
    char       *addrTypeStr = NULL;
    int16_t    ret = 0;
    uint8_t    help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, b_optionStr))
        {
            bdAddressStr = strtok(NULL, space_str);
        }
        else if(!strcmp(token, t_optionStr))
        {
            addrTypeStr = strtok(NULL, space_str);
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        printBleConnectUsage(NULL);
        return(-1);
    }

    ret = macAddressParse(bdAddressStr, bd_addr);
    if (ret < 0)
    {
        UART_PRINT("\r\n[Cmd Parser] : Invalid BdAddress. Format xx:xx:xx:xx:xx:xx\n\r");
        return (-1);
    }

    if(!addrTypeStr)
    {
        UART_PRINT("\r\n[Cmd Parser] : Invalid address type. Format PUBLIC or RANDOM\n\r");
        return (-1);
    }
    else if(!strcmp(addrTypeStr, PUBLIC_str))
    {
        *addr_type = 0;
    }
    else if(!strcmp(addrTypeStr, RANDOM_str))
    {
        *addr_type = 1;
    }

    return(0);
}

/*!
    \brief          Parse BLE disconnect command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          bd_addr        -   Points to BD address to disconnect from
    \param          addr_type      -   Points to address type to disconnect from

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleDisconnectCallback
 */
int32_t ParseBleDisconnectCmd(void *arg, uint8_t *bd_addr, uint8_t* addr_type)
{
    char       cmdStr[CMD_BUFFER_LEN + 1];
    char       *token = NULL;
    char       *bdAddressStr = NULL;
    char       *addrTypeStr = NULL;
    int16_t    ret = 0;
    uint8_t    help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        //Allow no parameters as default action is to disconnect all peers
        return(0);
    }

    while(token)
    {
        if(!strcmp(token, b_optionStr))
        {
            bdAddressStr = strtok(NULL, space_str);
        }
        else if(!strcmp(token, t_optionStr))
        {
            addrTypeStr = strtok(NULL, space_str);
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        printBleDisconnectUsage(NULL);
        return(-1);
    }

    ret = macAddressParse(bdAddressStr, bd_addr);
    if (ret < 0)
    {
        UART_PRINT("\r\n[Cmd Parser] : Invalid BdAddress. Format xx:xx:xx:xx:xx:xx\n\r");
        return (-1);
    }

    if(!addrTypeStr)
    {
        UART_PRINT("\r\n[Cmd Parser] : Invalid address type. Format PUBLIC or RANDOM\n\r");
        return (-1);
    }
    else if(!strcmp(addrTypeStr, PUBLIC_str))
    {
        *addr_type = 0;
    }
    else if(!strcmp(addrTypeStr, RANDOM_str))
    {
        *addr_type = 1;
    }

    return(0);
}

/*!
    \brief          Parse BLE get BD address command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          addr_type      -   Points to address type to disconnect from

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleDisconnectCallback
 */
int32_t ParseBleGetBdAddressCmd(void *arg, uint8_t* addr_type)
{
    char       cmdStr[CMD_BUFFER_LEN + 1];
    char       *token = NULL;
    char       *addrTypeStr = NULL;
    uint8_t    help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        //Allow no parameters as default action is to disconnect all peers
        return(0);
    }

    while(token)
    {
        if(!strcmp(token, t_optionStr))
        {
            addrTypeStr = strtok(NULL, space_str);
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        printBleGetBdAddrUsage(NULL);
        return(-1);
    }

    if(!addrTypeStr)
    {
        UART_PRINT("\r\n[Cmd Parser] : Invalid address type. Format PUBLIC or RANDOM\n\r");
        return (-1);
    }
    else if(!strcmp(addrTypeStr, PUBLIC_str))
    {
        *addr_type = 0;
    }
    else if(!strcmp(addrTypeStr, RANDOM_str))
    {
        *addr_type = 1;
    }

    return(0);
}

/*!
    \brief          Parse BLE set BD address command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          addr_type      -   Points to address type to disconnect from

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdBleDisconnectCallback
 */
int32_t ParseBleSetBdAddressCmd(void *arg, uint8_t* addr_type)
{
    char       cmdStr[CMD_BUFFER_LEN + 1];
    char       *token = NULL;
    char       *addrTypeStr = NULL;
    uint8_t    help = FALSE;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(token == NULL)
    {
        //Allow no parameters as default action is to disconnect all peers
        return(0);
    }

    while(token)
    {
        if(!strcmp(token, t_optionStr))
        {
            addrTypeStr = strtok(NULL, space_str);
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        printBleSetBdAddrUsage(NULL);
        return(-1);
    }

    if(!addrTypeStr)
    {
        UART_PRINT("\r\n[Cmd Parser] : Invalid address type. Format PUBLIC or RANDOM\n\r");
        return (-1);
    }
    else if(!strcmp(addrTypeStr, PUBLIC_str))
    {
        *addr_type = 0;
    }
    else if(!strcmp(addrTypeStr, RANDOM_str))
    {
        *addr_type = 1;
    }

    return(0);
}

/*!
    \brief          Prints role_id  help menu.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.

    \sa             cmdScanCallback
*/

int32_t printRoleIdGetUsage(void *arg)
{
    Report(getRoleIdUsageStr);
    Report(lineBreak);
    return(0);
}


int32_t ParseGetRoleId(void *arg, WlanRole_e* role)
{
    char            *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t help = FALSE;
    *role = WLAN_ROLE_AP;
    char *roleIdSTR;


    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(NULL == token)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, r_optionStr))
        {
            roleIdSTR = strtok(NULL, "\"");
            if(os_memcmp(roleIdSTR ,"sta", 3) == 0)
            {
                *role = WLAN_ROLE_STA;
            }
            else
            {
                *role = WLAN_ROLE_AP;
            }
        }
        else
        {
            return -1;
            break;
        }
        token = strtok(NULL, " ");
    }

    if(help)
    {
        return(-1);
    }

    return(0);
}

#ifdef CC35XX
int32_t ParseSetpeerAgingTimeout(void *arg, uint32_t* timeOut)
{
    char            *token = NULL;
    char cmdStr[CMD_BUFFER_LEN + 1];
    uint8_t help = FALSE;


    strncpy(cmdStr, (char*)arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if(NULL == token)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, help_optionStr))
        {
            help = TRUE;
        }
        else if(!strcmp(token, t_optionStr))
        {
            token = strtok(NULL, "\"");

            if(token)
            {
                *timeOut = (uint32_t)atoi(token);
            }
        }
        else
        {
            return -1;
            break;
        }
        token = strtok(NULL, " ");
    }

    if(help)
    {
        return(-1);
    }

    return(0);
}
#endif

/*!
    \brief          ipv6 Address Parse

    This routine converts string representing IPv6 address
    in a colon notation, into an array of bytes,
    representing each IP address octate.

    \param          str         -   Points to string address.

    \param          ipv6ip      -   Points to a 16 byte long array.

    \return         Upon successful completion,
                    the function shall return 0.
                    In case of failure, this function would return -1.

 */
int32_t ipv6AddressParse(char *str,
                         uint8_t *ipv6ip)
{
    int32_t         i;
    int32_t         l;
    int32_t         zeroCompressPos;
    uint8_t        *t;
    uint8_t         tmp[16];
    uint16_t        value;
    uint8_t         hexDigit;

    i = 0;
    t = (uint8_t*)str;
    value = 0;
    hexDigit=0;
    zeroCompressPos=-1;
    memset(tmp, 0, sizeof(tmp));

    if(*t==':')
    {
        if(*++t!=':')
        {
            return(-1);
        }
    }

    while(*t && (i < 16))
    {
        if(*t >= '0' && *t <= '9')
        {
            value = (value << 4) | (*t - '0');
            hexDigit = 1;
        }
        else if(*t >= 'a' && *t <= 'f')
        {
            value = (value << 4) | ((*t - 'a') + 10);
            hexDigit = 1;
        }
        else if(*t >= 'A' && *t <= 'F')
        {
            value = (value << 4) | ((*t - 'A') + 10);
            hexDigit = 1;
        }
        else if((*t == ':') && (i < 14))
        {
            if(hexDigit)
            {
                tmp[i++] = (value >> 8) & 0xFF;
                tmp[i++] = (value) & 0xFF;
                hexDigit = 0;
                value = 0;
            }
            else
            {
                if(zeroCompressPos < 0)
                {
                    zeroCompressPos = i;
                }
                else
                {
                    return(-1);
                }
            }
        }
        t++;
    }

    if(i > 15)
    {
        return(-1);
    }
    else if(hexDigit && (zeroCompressPos < 0) && (i < 14))
    {
        return(-1);
    }
    else if((!hexDigit) && (zeroCompressPos < 0))
    {
        return(-1);
    }
    else if((!hexDigit) && (zeroCompressPos != i))
    {
        return(-1);
    }
    else if((zeroCompressPos >= 0) && i >= 14)
    {
        return(-1);
    }

    if((hexDigit) && (i < 15))
    {
        tmp[i++] = (value >> 8) & 0xFF;
        tmp[i++] = (value) & 0xFF;
        hexDigit = 0;
        value = 0;
    }

    if(zeroCompressPos>=0)
    {
        i--;
        l = 15;
        while(i >= zeroCompressPos)
        {
            if((l >= 0) && (i >= 0))
            {
                tmp[l] = tmp[i];
                tmp[i] = 0;
                l--;
                i--;
            }
        }
    }

    memcpy(ipv6ip, tmp, sizeof(tmp));

    return(0);
}

/*!
    \brief          ipv4 Address Parse

    This routine converts string representing IPv4 address
    in a dotted decimal notation, into a 32-bit
    integer representation of the IP address.

    \param          str         -   Points to string address.

    \param          ipv4ip      -   Points to a uint32_t where the parsed IP will be stored.

    \return         Upon successful completion,
                    the function shall return 0.
                    In case of failure, this function would return -1.

 */
int32_t ipv4AddressParse(char *str,
                         uint32_t *ipv4ip)
{
    volatile int32_t i = 0;
    uint32_t n;
    uint32_t ipv4Address = 0;
    char             *token;

    token = strtok(str, ".");
    if(token)
    {
        n = (int)strtoul(token, 0, 10);
    }
    else
    {
        return(-1);
    }

    while(i < 4)
    {
       /* Check Whether IP is valid */
       if((token != NULL) && (n < 256))
       {
           ipv4Address |= n;
           if(i < 3)
           {
               ipv4Address = ipv4Address << 8;
           }
           token=strtok(NULL,".");
           if (token)
           {
               n = (int)strtoul(token, 0, 10);
           }
           i++;
       }
       else
       {
           return -1;
       }
    }

    *ipv4ip = ipv4Address;

    return(0);
}
/*!
    \brief          hex byte string to ASCII

    This routine converts a hexadecimal base byte represented
    as a string, into the equivalent ASCII character.

    \param          str         -   Points to string Hex.

    \param          ascii       -   Points to a buffer
                                    containing the converted ASCII char.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1.

 */
int32_t hexbyteStrtoASCII(char *str,
                          uint8_t *ascii)
{
    char        *t = NULL;
    uint8_t hexchar = 0;

    /* Skip 0x if present */
    if(str[1] == 'x')
    {
        t = str + 2;
    }
    else
    {
        t = str;
    }

    while((hexchar < 2) && ((*t) != '\0'))
    {
        if(*t >= '0' && *t <= '9')
        {
            *ascii = (*ascii << (hexchar * 4)) | (*t - '0');
        }
        else if(*t >= 'a' && *t <= 'f')
        {
            *ascii = (*ascii << (hexchar * 4)) | ((*t - 'a') + 10);
        }
        else if(*t >= 'A' && *t <= 'F')
        {
            *ascii = (*ascii << (hexchar * 4)) | ((*t - 'A') + 10);
        }
        else
        {
            /* invalid entry */
            return(-1);
        }
        hexchar++;
        t++;
    }

    if(hexchar == 0)
    {
        return(-1);
    }

    return(0);
}

/*!
    \brief          Parse MAC address.

    This routine converts a MAC address given in a colon separated
    format in a string form, and converts it to six byte number format,
    representing the MAC address.
    Note: make sure that "mac" buffer input param is initialized to 0.

    \param          str       -   Points to string Hex.

    \param          mac       -   Points to a buffer containing
                                  the converted ASCII char.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return -1.

 */
int32_t macAddressParse(char *str,
                        uint8_t *mac)
{

    int32_t        count = 0;
    char           *t = NULL;
    uint8_t        tmp[3];
    uint8_t        byte = 0;
    size_t         MAC_length;

    t = (char*)str;

    MAC_length = strlen(t);

    if (MAC_length > SIX_BYTES_SIZE_MAC_ADDRESS)
    {
        /* invalid MAC size */
        return(-1);
    }

    memset(tmp, 0, sizeof(tmp));

    while(byte < 6)
    {
        count  = 0;
        while(*t != ':' && count < 2)
        {
            tmp[count] = *t;
            count++;
            t++;
        }

        tmp[count] = 0;

        if(hexbyteStrtoASCII((char*)&tmp[0], &mac[byte]) < 0)
        {
            return(-1);
        }

        byte++;

        if(*t != ':' && byte < 6)
        {
            /* invalid entry */
            return(-1);
        }

        /* Skip ':' */
        t++;
    }

    return(0);
}

#ifdef CC35XX
/*!
    \brief          Parse start AP WPS command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          wpsSession     -   Points to command structure provided
                                       by the startApWps callback.
                                       This structure will later be read
                                       by the startApWps callback.

    \return         Upon successful completion, the function shall return success status.
                    In case of failure,this function would print error.

    \sa             cmdStartApWpsCallback
 */
int32_t ParseStartApWpsSessionCmd(void *arg, wlanWpsSession_t *wpsSession)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    uint8_t help = FALSE;
    uint8_t wpsMethod = 0xFF;
    char    *pin = NULL;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, w_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                wpsMethod = atoi(token);
            }
        }
        else if(!strcmp(token, p_optionStr))
        {
            pin = strtok(NULL, "\"");
            if(strlen(pin) != 8)
            {
                Report("\r\n[Cmd Parser] : Invalid PIN. Expected 8 digits\n\r");
                return(-1);
            }
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        return(-1);
    }

    if ((wpsMethod == WPS_METHOD_PBC) || (wpsMethod == WPS_METHOD_PIN))
    {
        wpsSession->wpsMethod = wpsMethod;
    }
    else
    {
        Report("\r\n[Cmd Parser] : Invalid WPS method\n\r");
        return(-1);
    }

    if (pin)
    {
        strcpy(wpsSession->pin, pin);
    }
    else if (wpsMethod == WPS_METHOD_PIN)
    {
        Report("\r\n[Cmd Parser] : Parser expected PIN code\n\r");
        return(-1);
    }

    return(0);
}
#endif

#ifdef SNTP_SUPPORT

/*!
    \brief          Parse SntpConfigServers command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          id              -   Pointer to id.



    \return         Upon successful completion, the function shall return the RoleId number.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdKillCallback
 */
int32_t ParseSntpConfigServersCmd(void *arg, uint32_t *pNumOfServers,char* serverIp[])
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    uint8_t help = FALSE;

    *pNumOfServers = 0;
    char  *ip = NULL;


    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if(!strcmp(token, s_optionStr))
        {
            if(*pNumOfServers == 3)
            {
                Report("\n\r[cmd Parser] : up to 3 servers are supported\n\r");
                return(-1);
            }
            ip = (char*)strtok(NULL, space_str);
            if(ip != NULL)
            {
                serverIp[*pNumOfServers] = os_malloc(strlen((const char*)ip)+1);
                os_memset(serverIp[*pNumOfServers], 0, strlen((const char*)ip)+1);
                os_memcpy(serverIp[*pNumOfServers] , ip, strlen((const char*)ip));
                (*pNumOfServers)++;

            }
            else
            {
                Report("\n\r[cmd Parser] : Invalid IP\n\r");
                return(-1);
            }
        }

        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        return (-1);
    }

    return (0);
}


/*!
 \brief          Prints sntp server update menu.

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdShowCallback

 */
int32_t printSntpConfigServersUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(SntpConfigNTPServers);
    Report("\t");


    UART_PRINT(SntpConfigNTPServersUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(SntpConfigNTPServers_s_optionDetailsStr);
    UART_PRINT(SntpConfigNTPServers_s_optionDetailsStrExpand);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return (0);
}



int32_t printSntpUpdateDateTimeUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(SntpUpdateDateTime);
    Report("\t");

    UART_PRINT(SntpUpdateTimeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return (0);
}

#endif



int32_t ParseSetDateTimeCmd(void *arg, uint32_t* pYear, uint32_t* pMonth,
        uint32_t* pDay, uint32_t* pHour, uint32_t* pMinute,
        uint32_t* pSecond)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;

    uint8_t help = FALSE;

    char *pDateTime = NULL;
    char* dateTimeStr;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while (token)
    {
        if(!strcmp(token, t_optionStr))
        {
            pDateTime = (char*)strtok(NULL, space_str);
            if(pDateTime != NULL)
            {
                dateTimeStr = os_zalloc(DATE_TIME_STR_SIZE);
                os_memcpy(dateTimeStr , pDateTime, DATE_TIME_STR_SIZE);
                sscanf(dateTimeStr, "%d-%d-%dT%d:%d:%d", (int *)pYear, (int *)pMonth, (int *)pDay,
                       (int *)pHour, (int *)pMinute, (int *)pSecond);
                os_free(dateTimeStr);

            }
            else
            {
                Report("\n\r[cmd Parser] : Invalid IP\n\r");
                return(-1);
            }

        }
        else
        {
            SHOW_WARNING(-1, CMD_ERROR);
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if (help)
    {
        return (-1);
    }

    return (0);
}


/*!
 \brief          Prints menu for update the date and time manually

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdShowCallback

 */
int32_t printSetDateTimeUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(SetDateTime);
    Report("\t");

    UART_PRINT(SetDateTimeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(SetDateTime_t_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return (0);
}


/*!
 \brief          Prints menu for getting the time

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdShowCallback

 */
int32_t printGetDateTimeUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(GetDateTime);
    Report("\t");

    UART_PRINT(GetDateTimeUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(GetDateTime_t_optionDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);

    return (0);
}

int32_t ParseLoadCartificateCmd(void *arg, LoadCertiCmd_t *loadCertificate)
{
    char    cmdStr[CMD_BUFFER_LEN + 1];
    char    *token = NULL;
    uint8_t help = FALSE;
    uint8_t type;
    uint32_t size;

    strncpy(cmdStr, (char*) arg, CMD_BUFFER_LEN);
    cmdStr[CMD_BUFFER_LEN] = '\0';
    token = strtok(cmdStr, space_str);

    if (token == NULL)
    {
        help = TRUE;
    }

    while(token)
    {
        if(!strcmp(token, t_optionStr))
        {
            token = strtok(NULL, space_str);
            if(token)
            {
                type = atoi(token);
            }
        }
        else if(!strcmp(token, s_optionStr))
        {
            token = strtok(NULL, "\"");
            if (token){
                size = atoi(token);
            }
        }
        else
        {
            help = TRUE;
            break;
        }
        token = strtok(NULL, space_str);
    }

    if(help)
    {
        return(-1);
    }

    if (type == 0){
        client_certi.fileType = type;
        client_certi.size = size;
        //craet malloc for client_certi global variable
        client_certi.certi = os_malloc(client_certi.size);
        if (client_certi.certi != NULL){
            // send ACK
            UART_PRINT("ACK\n\r");
            // Get certi to client_certi global variable
            getstr(client_certi.size, client_certi.certi);

            UART_PRINT("client certificate Load successfully\n\r");
        }
        
    }
    else if (type == 1) {
        ca_certi.fileType = type;
        ca_certi.size = size;
        //craet malloc for ca_certi global variable
        ca_certi.certi = os_malloc(ca_certi.size);
        if(ca_certi.certi != NULL){
            // send ACK
            UART_PRINT("ACK\n\r");
            // Get certi to ca_certi global variable
            getstr(ca_certi.size, ca_certi.certi);

            UART_PRINT("ca certificate Load successfully\n\r");
        }
        
    }
    else if (type == 2) {
        private_key_certi.fileType = type;
        private_key_certi.size = size;
        //craet malloc for private_key_certi global variable
        private_key_certi.certi = os_malloc(private_key_certi.size);
        if(private_key_certi.certi != NULL){
            // send ACK
            UART_PRINT("ACK\n\r");
            // Get certi to ca_certi global variable
            getstr(private_key_certi.size, private_key_certi.certi);

            UART_PRINT("private key certificate Load successfully\n\r");
        }
    }
    else {
        // Error file type
        UART_PRINT("file type does not exist\n\r");
        return(1);
    }


    return(0);
}

int32_t printloadCartificateUsage(void *arg)
{
    UART_PRINT(lineBreak);
    UART_PRINT(usageStr);
    UART_PRINT(loadCertificate);
    UART_PRINT(loadCertificateUsageStr);
    UART_PRINT(descriptionStr);
    UART_PRINT(loadCertificateDetailsStr);
    UART_PRINT(help_optaionDetails);
    UART_PRINT(lineBreak);
    return(0);
}
