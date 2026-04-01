/*
 * Copyright (c) 2025, Texas Instruments Incorporated
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
#include <stdio.h>
#include <string.h>


#include "FreeRTOS.h"

/* Board Header files */
#include "ti_drivers_config.h"
#include "uart_term.h"
//LWIP
#include "network_lwip.h"
//ERRORS
#include "errors.h"
#include "osi_kernel.h"
#include "date_time_service.h"

#include "sntp_task.h"

#ifdef SNTP_SUPPORT

 struct sockaddr;
 struct sockaddr_in;
#ifdef LWIP_IPV6
 struct sockaddr_in6;
#endif

#define NTP_SERVER_PORT (123)

#define NTPSIZE (3 * sizeof(struct sockaddr))
#define OSI_WAIT_FOR_SNTP_TIME_UPDATE_MS   (50000)


uint8_t g_storedNtpServers[NTPSIZE] = {0};
uint8_t g_numOfStoredNtpServers = 0;

static OsiSyncObj_t g_sntpSemTimeUpdate = NULL;


/*
 *  ======== timeUpdateHook ========
 *  Called after NTP time sync
 */
void sntpWrapper_timeUpdateHook(void *p)
{
   osi_SyncObjSignal(&g_sntpSemTimeUpdate);
}

//defines the NTP servers ip addresses
void sntpWrapper_store_servers(uint32_t numOfServers,char* sntpServer1IP,char* sntpServer2Ip, char* sntpServer3Ip )
{
    struct sockaddr_in ntpAddr;
    int32_t currPos = 0;

    SNTP_PRINT_REPORT("\n\rConfigure the  NTP servers : num of servers %d : ", numOfServers);

    /* the test is performed on IPV$ socket) */
    if(numOfServers > 0)
    {
        /* NTP address 1 */
        ntpAddr.sin_family = AF_INET;
        ntpAddr.sin_port = NTP_SERVER_PORT;//htons(NTP_SERVER_PORT);
        inet_pton(AF_INET, sntpServer1IP, &ntpAddr.sin_addr);
        os_memcpy((g_storedNtpServers + currPos), &ntpAddr, sizeof(struct sockaddr_in));
        currPos += sizeof(struct sockaddr);
        SNTP_PRINT_REPORT("%s ",sntpServer1IP);
    }
    if(numOfServers > 1)
    {
        /* NTP address 2 */
        inet_pton(AF_INET, sntpServer2Ip, &ntpAddr.sin_addr);
        os_memcpy((g_storedNtpServers + currPos), &ntpAddr, sizeof(struct sockaddr_in));
        currPos += sizeof(struct sockaddr);
        SNTP_PRINT_REPORT("%s ",sntpServer2Ip);
    }
    if(numOfServers > 2)
    {
        /* NTP address 3 */
        inet_pton(AF_INET, sntpServer3Ip, &ntpAddr.sin_addr);
        os_memcpy((g_storedNtpServers + currPos), &ntpAddr, sizeof(struct sockaddr_in));
        currPos += sizeof(struct sockaddr);
        SNTP_PRINT_REPORT("%s ",sntpServer3Ip);

    }

    g_numOfStoredNtpServers = numOfServers;

}
/*
 *  ======== test ========
 */
int32_t sntpWrapper_updateDateTime()
{
    int32_t ret = 0;

    SNTP_PRINT_REPORT("\n\rSyncing time with NTP server...");
    /* the test is performed on IPV$ socket) */

    if(g_numOfStoredNtpServers == 0)
    {
        SNTP_PRINT_REPORT_ERROR("\n\rERROR! SNTP, no NTP servers are configured");
        return -1;
    }

    ret = osi_SyncObjCreate(&g_sntpSemTimeUpdate);
    if (ret != OSI_OK) {
        SNTP_PRINT_REPORT_ERROR("ERROR ! startNTPTime: Cannot create semaphore!");
        return -1;
    }


    if (sntpTask_start(datetime_secondsGet, datetime_SecondsSet, sntpWrapper_timeUpdateHook,
            (struct sockaddr *)g_storedNtpServers, g_numOfStoredNtpServers, 0)!= 0) {
        SNTP_PRINT_REPORT_ERROR("\n\rERROR ! startNTPTime: SNTP failed!");
        osi_SyncObjDelete(&g_sntpSemTimeUpdate);
        g_sntpSemTimeUpdate = 0;
        return -1;
    }


    sntpTask_forceTimeSync();
    ret = osi_SyncObjWait(&g_sntpSemTimeUpdate,OSI_WAIT_FOR_SNTP_TIME_UPDATE_MS);
    if(OSI_OK != ret)
    {
        SNTP_PRINT_REPORT_ERROR("\n\rStop sntp update time task(due to timeout), time was not updated");
    }
    else
    {
        datetime_printCurTime();
    }

    sntpTask_stop();
    osi_SyncObjDelete(&g_sntpSemTimeUpdate);
    g_sntpSemTimeUpdate = 0;

    return 0;
}



#endif
