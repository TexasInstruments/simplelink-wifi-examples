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

#ifndef __ATCMD_NETCFG_H__
#define __ATCMD_NETCFG_H__

#include <stdint.h>

/*!
	\defgroup NetCfg 
	\short Controls the configuration of the device addresses (i.e. IP and MAC addresses)

*/

/*!

    \addtogroup NetCfg
    @{

*/

#if 0
/*!
    \brief          Netcfg Get callback.

    This routine gets network configurations
    
    \param          arg       -   Points to command line buffer.

    \return                     Zero on success, or negative error code on failure
*/
int32_t ATCmdNetcfg_getCallback(void *arg);

/*!
    \brief          Netcfg Set callback.

    This routine sets network configurations
    
    \param          arg       -   Points to command line buffer.

    \return                     Zero on success, or negative error code on failure
*/
int32_t ATCmdNetcfg_setCallback(void *arg);

#endif

/*!
    \brief          Netcfg Set Interface IP callback.

    This routine allows configuring an interface IP address.
    Whether it's an AP or STA and whether the IP is static or DHCP.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_setInterfaceIpCallback(void *arg);

/*!
    \brief          Netcfg get interface IP callback.

    This routine gets network configurations of a specific
    requested interface.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_getInterfaceIpCallback(void *arg);

/*!
    \brief          Netcfg set DHCP server configurations callback.

    This routine allows setting the DHCP server configurations.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_setDhcpServerCallback(void *arg);

/*!
    \brief          Netcfg get DHCP callback.

    This routine retrieves current DHCP server configurations.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_getDhcpServerCallback(void *arg);

/*!
    \brief          Ping callback.

    This routine sends a ping to specified address.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_pingCallback(void *arg);

/*!
    \brief          Ping stop callback.

    This routine stops ping sending.

    \param          arg       -   Points to command line buffer.

    \return         Upon successful completion, the function shall return 0.
                    In case of failure, this function would return an error;

*/
int32_t ATCmdNetcfg_pingStopCallback(void *arg);

/*!

 Close the Doxygen group.
 @}

 */

#endif /* __ATCMD_NETCFG_H__ */
