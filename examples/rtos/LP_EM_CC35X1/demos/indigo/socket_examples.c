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
#include "socket_examples.h"
#include "lwip/opt.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmd_parser.h"
#include "uart_term.h"
#include "errors.h"
#include "FreeRTOSConfig.h"
#include "osi_kernel.h"
#ifdef CC35XX
#include "wlan_if.h"
#include "network_lwip.h"
#endif // CC35XX

#ifndef SOCK_TARGET_HOST4
#define SOCK_TARGET_HOST4  "192.168.0.1"
#endif

#ifndef SOCK_TARGET_HOST6
#define SOCK_TARGET_HOST6  "FE80::12:34FF:FE56:78AB"
#endif

#ifndef SOCK_TARGET_PORT
#define SOCK_TARGET_PORT  5001
#endif

#ifndef SOCK_TARGET_MAXHTTPPAGESIZE
#define SOCK_TARGET_MAXHTTPPAGESIZE 1024
#endif

#ifndef SOCKET_EXAMPLES_RUN_PARALLEL
#define SOCKET_EXAMPLES_RUN_PARALLEL 0
#endif

const u8_t cmpbuf[8] = { 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab };

const u8_t *trafic = (u8_t*) 0x70008000;

/* a helper struct to ensure memory before/after fd_set is not touched */
typedef struct _xx
{
    u8_t buf1[8];
    fd_set readset;
    u8_t buf2[8];
    fd_set writeset;
    u8_t buf3[8];
    fd_set errset;
    u8_t buf4[8];
} fdsets;

#define INIT_FDSETS(sets) do { \
  memset((sets)->buf1, 0xab, 8); \
  memset((sets)->buf2, 0xab, 8); \
  memset((sets)->buf3, 0xab, 8); \
  memset((sets)->buf4, 0xab, 8); \
}while(0)

#define CHECK_FDSETS(sets) do { \
  LWIP_ASSERT("buf1 fail", !memcmp((sets)->buf1, cmpbuf, 8)); \
  LWIP_ASSERT("buf2 fail", !memcmp((sets)->buf2, cmpbuf, 8)); \
  LWIP_ASSERT("buf3 fail", !memcmp((sets)->buf3, cmpbuf, 8)); \
  LWIP_ASSERT("buf4 fail", !memcmp((sets)->buf4, cmpbuf, 8)); \
}while(0)

typedef union
{
#if LWIP_IPV6
    sockaddr_in6 in6;       /* Socket info for Ipv6 */
#endif
#if LWIP_IPV4
    struct sockaddr_in in4; /* Socket info for Ipv4 */
#endif
} sockAddr_t;


extern uint64_t ClockP_getTimeUsec();

/****************************************************************************
 LOCAL FUNCTION PROTOTYPES
 ****************************************************************************/
int32_t Client(uint8_t nb, uint16_t portNumber, ip_t ipAddress, uint8_t ipv6,
               uint32_t numberOfPackets, uint8_t tx, uint8_t protocol,uint32 bandwidth);
int32_t Server(uint8_t nb, uint16_t portNumber, uint8_t ipv6,
               uint32_t numberOfPackets, uint8_t tx, uint8_t protocol);

/* Thread execution */
void socket_ClientExecute(void* pValue);
void socket_ServerExecute(void* pValue);


/* client/server helper function */
int32_t socket_ClientTransceiver(int32_t sock, int32_t numberOfPackets,
                                 BOOLEAN *pIsRunning, uint32_t Bandwidth/*Mbps*/);

int32_t socket_ServerRecevicer(int32_t sock, int32_t numberOfPackets,
                               BOOLEAN *pIsRunning, BOOLEAN isTCP, uint8_t nb);

int32_t socket_ServerTCP(int32_t sock, int32_t numberOfPackets,
                         BOOLEAN *pIsRunning, uint8_t nb, BOOLEAN isTCP,
                         int32_t addrSize, sockAddr_t *sAddr,
                         struct sockaddr_in *csa);

/****************************************************************************
 THREAD FUNCTION PROTOTYPES
 ****************************************************************************/
#define MAX_THREAD_ENTRY    (4)

typedef void  (*_SpawnEntryFunc_t)(void* pValue);
typedef struct SpawnThreadEntry_t
{
    int8_t                  id;                 // Index for socket process that running
    OsiThread_t             pThread;            // Thread control block
    OsiSyncObj_t            syncObj;            // sync object to kill the thread
    BOOLEAN                 bIsRunning;         // Flag to indicate the thread is running
    int32_t                 sock;               // saved the sock
    _SpawnEntryFunc_t       entryFunc;          // Thread function
    char*                   pName;              // Thread name
    void*                   pParam;             // Thread params
} SpawnThreadEntry_t;


SpawnThreadEntry_t gSpawThread[MAX_THREAD_ENTRY] = {0};



/****************************************************************************
 THREAD FUNCTION PROTOTYPES
 ****************************************************************************/
int socket_ThreadExecute(_SpawnEntryFunc_t pEntry, void* pValue , char*  pThreadName ,unsigned long flags)
{

    OsiReturnVal_e status;
    int id;

    for(id = 0; id < MAX_THREAD_ENTRY; id++)
    {
        if(gSpawThread[id].bIsRunning == FALSE)
        {
            break;
        }
    }
    if( id == MAX_THREAD_ENTRY)
    {
        // no free entry
        return -1;
    }

    osi_EnterCritical();
    if(pValue)
    {
        gSpawThread[id].pParam = os_malloc(sizeof(SendCmd_t));
        if(!gSpawThread[id].pParam){
            ASSERT_GENERAL(0);
            Report("\n\rWlan start failed: %d\n\r OSI_MEMORY_ALLOCATION_FAILURE");
            return OSI_MEMORY_ALLOCATION_FAILURE;
        }
        os_memcpy(gSpawThread[id].pParam , pValue, sizeof(SendCmd_t));

    }

    if(OSI_OK != osi_SyncObjCreate(&gSpawThread[id].syncObj))
    {
        os_free(gSpawThread[id].pParam);
        return -1;
    }

#ifdef CC35XX
    status = osi_ThreadCreate(&gSpawThread[id].pThread, // Thread control block
                              pThreadName,              // Thread name
                              4096,                     // STACK size
                              4,                        // Priority
                              (void*)pEntry,            // Execute function
                              &gSpawThread[id]);         // params
#elif defined(CC33XX)
    status = osi_ThreadCreate(&gSpawThread[id].pThread, // Thread control block
                              pThreadName,              // Thread name
                              4096,                     // STACK size
                              5,                        // Priority
                              (void*)pEntry,            // Execute function
                              &gSpawThread[id]);         // params
#endif							  
							  




    if(OSI_OK != status)
    {
        os_free(gSpawThread[id].pParam);
        gSpawThread[id].pParam = NULL;
        return status;
    } // failed

    gSpawThread[id].pName       = pThreadName;
    gSpawThread[id].entryFunc   = pEntry;
    gSpawThread[id].id          = id;

    // Mark thread in running
    gSpawThread[id].bIsRunning = TRUE;

    osi_ExitCritical(0);
    return OSI_OK;
}

int32_t socket_ThreadDestroy(int id)
{
    struct timeval opt;
    opt.tv_sec = 0;
    opt.tv_usec = 0;
    int32_t ret;

    if(MAX_THREAD_ENTRY > id && 0 <= id && gSpawThread[id].bIsRunning)
    {
        // Change flag
        gSpawThread[id].bIsRunning = FALSE;

        // Force timeout
        lwip_setsockopt(gSpawThread[id].sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&opt, sizeof(opt));

        // Waiting for thread will finish main loop
        ret = osi_SyncObjWait(&gSpawThread[id].syncObj, OSI_WAIT_FOR_SECOND * 60);
        if(OSI_OK != ret)
        {
            ASSERT_GENERAL(0);
            return ret;
        }

        // Delete the sync object
        ret = osi_SyncObjDelete(&gSpawThread[id].syncObj);
        if(OSI_OK != ret)
        {
            ASSERT_GENERAL(0);
            return ret;
        }

        // Now the thread finish and we can delete the thread
        ret = osi_ThreadDelete(&gSpawThread[id].pThread);
        if(OSI_OK != ret)
        {
            ASSERT_GENERAL(0);
            return ret;
        }

        // Remove all resources

        gSpawThread[id].entryFunc   = NULL;
        gSpawThread[id].pName       = NULL;
        gSpawThread[id].syncObj     = NULL;

        gSpawThread[id].id          = -1;

        if(gSpawThread[id].pParam)
        {
            os_free(gSpawThread[id].pParam);
        }
        gSpawThread[id].pParam      = NULL;

        return 0;
    }

    return OSI_OPERATION_FAILED;


}


void socket_PrintRunningThread()
{
    int id;
    Report("\n\r\t Process running: \n\r");
    for(id = 0; id < MAX_THREAD_ENTRY; id++)
    {
        if(gSpawThread[id].bIsRunning)
        {
            Report("\n\r\t---id %d \t %s", id, gSpawThread[id].pName);
        }
    }
}


void killAllProcess()
{
    int id;
    int ret;
    // Go over all the process
    for(id = 0; id < MAX_THREAD_ENTRY; id++)
    {
        // If process running
        if(gSpawThread[id].pParam != NULL)
        {
            ret = socket_ThreadDestroy(id);
            if(ret != OSI_OK)
            {
                Report("\r\n[KILL ERROR] process in id %d is not running! call show_socket for see the running id's \n\r", id);
                SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
            }
        }
    }
    return;
}
uint8 gBuffer[MAX_BUF_SIZE] = {0};
/*****************************************************************************
 Client/Server helper Functions Callback
 *****************************************************************************/
int32_t socket_ClientTransceiver(int32_t sock, int32_t numberOfPackets,
                                 BOOLEAN *pIsRunning, uint32_t Bandwidth/*Mbps*/)
{

#ifdef CLIENT_DEBUG
    uint64_t current_sample, diff;
    uint32_t current_data;
    volatile uint64_t last_sample_write = 0;
    volatile uint64_t accumulated_data_sent = 0;
#endif

    int32_t status = NS_ERROR_BSD_SOC_OK;
    int32_t i = 0;
    int32_t buflen      = MAX_BUF_SIZE + 2;//1470 + 42 bytes Ethernet headers
    uint8_t *pBuffer = gBuffer;
    uint32_t Bandwidth_byte_per_100_mili;
    uint32_t time_mili_sec,sec,usec;
    uint32_t packet_count = 0, count;
    uint32_t units_of_100 = 100;

    Bandwidth_byte_per_100_mili =  (uint32_t)((uint64_t)(Bandwidth*1000*100)/8);//bytes per 100 mili


    /* filling the buffer with data */
    for (i = 0; i < MAX_BUF_SIZE; i++)
    {
        pBuffer[i] = (char) (i % 10);
    }

    /* Sanity check */
    if(!pIsRunning && !numberOfPackets)
    {
        Report("\n\r[socket_ServerRecevicer]: cannot get here - assert ! \n\r");
        ASSERT_GENERAL(0);
        return -1;
    }

    {
        uint32 last_time_mili_sec , number_of_bytes_send_from_last_mili = 0;
        last_time_mili_sec = osi_GetTimeMS();
        packet_count = 1;

        while ( ((pIsRunning!= NULL) && (*pIsRunning)) || ((numberOfPackets > 0)&& (packet_count < numberOfPackets)))
        {
            /* Send packets to server */

            time_mili_sec = osi_GetTimeMS();

            /*Report("\r\n time_mili_sec:%d "
                    "number_of_bytes_send_from_last_mili:%d "
                    "Bandwidth_byte_per_100_mili:%d",
                    time_mili_sec,
                    number_of_bytes_send_from_last_mili,
                    Bandwidth_byte_per_100_mili);*/
            if(time_mili_sec-last_time_mili_sec>100)
            {
                /*Report("\r\n 100 mili passed, during it send:%d bytes ",number_of_bytes_send_from_last_mili);*/
                number_of_bytes_send_from_last_mili = 0;
                last_time_mili_sec = time_mili_sec;
            }
            else if(number_of_bytes_send_from_last_mili >= (Bandwidth_byte_per_100_mili-40))
            {
               uint32_t sleep_ms = units_of_100 - (time_mili_sec-last_time_mili_sec) ;
               os_sleep(sleep_ms/1000, (sleep_ms%1000)*1000 );
               /*Report("\r\n  after sleep, sleep_ms:%d", sleep_ms);*/
               continue;
            }

            sec = htonl(time_mili_sec/1000);
            usec = htonl((time_mili_sec%1000)*1000);
            count = htonl(packet_count);

            os_memcpy(pBuffer, &count, sizeof(count));
            os_memcpy(pBuffer+4, &sec, sizeof(sec));
            os_memcpy(pBuffer+8, &usec, sizeof(usec));

            buflen = MIN(Bandwidth_byte_per_100_mili - number_of_bytes_send_from_last_mili,
                    MAX_BUF_SIZE);
            if(osi_GetFreeHeapSize() > HEAP_THRESHOLD_FOR_TX)
            {
                status = lwip_write(sock, pBuffer, buflen);
                number_of_bytes_send_from_last_mili += buflen;
                if (status == NS_ERROR_BSD_EAGAIN)
                {
                    Report("[error:%d] %s\n\r", status, SOCKET_ERROR_MSG);
                    osi_Sleep(1);
                    continue;
                }
                else if (status < 0)
                {
                    status = NS_ERROR_BSD_SOC_ERROR;
                    Report("[error:%d] %s\n\r", status, SOCKET_ERROR_MSG);
                    goto exit;
                }
                packet_count++;
    #ifdef CLIENT_DEBUG
                accumulated_data_sent += status;

                current_sample = ClockP_getTimeUsec();

                if (last_sample_write != 0)
                {
                    diff = current_sample - last_sample_write;

                    if (diff > 1000000)
                    {
                        diff = diff / 1000000;
                        current_data = accumulated_data_sent / diff;
                        Report("\n\r+++++++++ client send is %d bytes +++++++", current_data);
                        accumulated_data_sent = 0;
                    }
                }

                last_sample_write = current_sample;
    #endif
            }
        }
    }


exit:
    return status;

}

void format_bps(double bps, char *output, size_t size) {
    const char *units[] = {"bps", "Kbps", "Mbps", "Gbps", "Tbps"};
    int unit_index = 0;

    while (bps >= 1000 && unit_index < 4) {
        bps /= 1000;
        unit_index++;
    }

    uint32_t integer_part = (uint32_t)bps;
    uint32_t decimal_part = (uint32_t)((bps - integer_part) * 100 + 0.5);

    // Handle rounding overflow
    if (decimal_part >= 100)
    {
        integer_part++;
        decimal_part = 0;
    }

    snprintf(output, size, "%lu.%02lu %s",
             (unsigned long)integer_part,
             (unsigned long)decimal_part,
             units[unit_index]);
}

#define WLAN_MAX_FORMAT_RATE_LENGTH 20
void printAvg(uint32_t numOfRecvBytes, uint32_t *lastPrintTime, uint32_t* total_rx_bytes_per_sec)
{
    double bps =0 ;
    *total_rx_bytes_per_sec += numOfRecvBytes;
    char ratestr[WLAN_MAX_FORMAT_RATE_LENGTH];

    if((osi_GetTimeMS() - (*lastPrintTime)) >=  1000)
    {
        bps = ((*total_rx_bytes_per_sec) * 8);

        Report("\r\nt:%020d --Rx ",*lastPrintTime);
        Report("Rate ");

        format_bps(bps,ratestr, sizeof(ratestr));
        Report("%s",ratestr);
        *lastPrintTime = osi_GetTimeMS();
        *total_rx_bytes_per_sec = 0;
    }

}

int32_t socket_ServerRecevicer(int32_t sock, int32_t numberOfPackets,
                           BOOLEAN *pIsRunning, BOOLEAN isTCP, uint8_t nb)
{
    int32_t statusOrNumOfByte      = NS_ERROR_BSD_SOC_OK;
    uint16_t buflen     = MAX_BUF_SIZE;
    uint8_t  pBuffer[MAX_BUF_SIZE] = {0};
    uint32_t total_rcvd_bytes_per_sec = 0;
    uint32_t lastPrintTime;
    uint64_t total_recv_bytes = 0;
    uint32_t total_recv_packets = 0;
    uint32_t startTime, endTime;


    /* clear the global data buffer */
    memset(pBuffer, 0x0, MAX_BUF_SIZE);

    /* Sanity check */
    if(!pIsRunning && !numberOfPackets)
    {
        Report("\n\r[socket_ServerRecevicer]: cannot get here - assert ! \n\r");
        ASSERT_GENERAL(0);
    }

    lastPrintTime = osi_GetTimeMS();
    startTime = osi_GetTimeMS();
    if(numberOfPackets == 0)//endless loop
    {
        /* If numberOfPackets is 0 we are in infinity mode -
         * keep receiving data until socket terminated
         */
        while (*pIsRunning)
        {
            statusOrNumOfByte = lwip_read(sock, pBuffer, buflen);
            if(statusOrNumOfByte < 0)
            {
                if(!statusOrNumOfByte && isTCP)
                {
                    statusOrNumOfByte = NS_ERROR_BSD_ECLOSE;
                    Report("\n\rTCP client closed the socket errno:%d \n\r",errno);
                    Report("\n\rReceived total %u bytes successfully ( number may be wrapped around)\n\r",
                            total_recv_bytes);
                    goto exit;
                }
                if(nb == TRUE)
                {
                    statusOrNumOfByte = NS_ERROR_BSD_SOC_ERROR;
                    Report("[lwip_read - error:%d] %s errno:%d\n\r", statusOrNumOfByte, SOCKET_ERROR_MSG, errno);
                    goto exit;
                }
                // nothing received from client
                Report("\n\rTimeout on lwip_read , keep waiting until closing the thread \n\r");
                osi_Sleep(1);
                continue;
            }
            total_recv_bytes += statusOrNumOfByte;
            total_recv_packets++;
            printAvg(statusOrNumOfByte, &lastPrintTime, &total_rcvd_bytes_per_sec);

        }

    }
    else
    {
        while (total_recv_packets < numberOfPackets)//limit the amount of recv packets
        {

            statusOrNumOfByte = lwip_read(sock, pBuffer, buflen);
            if(statusOrNumOfByte <= 0)
            {
                if(!statusOrNumOfByte && isTCP)
                {
                    statusOrNumOfByte = NS_ERROR_BSD_ECLOSE;
                    Report("\n\rTCP client closed the socket \n\r");
                    Report("\n\rRx total of %u bytes successfully \n\r",
                            total_recv_bytes);
                    goto exit;
                }
                if(nb == TRUE)
                {
                    statusOrNumOfByte = NS_ERROR_BSD_SOC_ERROR;
                    Report("[lwip_read - error:%d] %s\n\r", statusOrNumOfByte, SOCKET_ERROR_MSG);
                    goto exit;
                }
                if(numberOfPackets > 0)
                {
                    Report(
                        "\n\rTimeout expired before receiving all packets, received bytes:%d\n\r", total_recv_bytes);
                    statusOrNumOfByte = NS_ERROR_BSD_ETIMEDOUT;
                    goto exit;
                }
                // nothing received from client
                Report("\n\rTimeout expired keep waiting until closing the thread \n\r");
                osi_Sleep(1);
                continue;
            }
            total_recv_bytes += statusOrNumOfByte;
            total_recv_packets++;
            printAvg(statusOrNumOfByte,&lastPrintTime, &total_rcvd_bytes_per_sec);
        }
    }
    endTime = osi_GetTimeMS();
    Report("\r\nTotal Rx of %u bytes( %u packets) in %u sec successfully \r\n (note! number of bytes maybe be wrapped around) \n\r",
        total_recv_bytes, total_recv_packets, ((endTime - startTime)/1000));

    statusOrNumOfByte = NS_ERROR_BSD_SOC_OK;

exit:
    return statusOrNumOfByte;
}


int32_t socket_ServerTCP(int32_t sock, int32_t numberOfPackets, BOOLEAN *pIsRunning, uint8_t  nb,
                     BOOLEAN isTCP, int32_t addrSize, sockAddr_t *sAddr, struct sockaddr_in *csa)
{
    int32_t status = NS_ERROR_BSD_SOC_OK;
    int32_t newsock = -1;
    /* TCP ONLY 'Listen' signify that wer'e ready to receive connection's from clients */
    status = lwip_listen(sock, 0);

    if(status < 0)
    {
        Report("[lwip_listen error:%d] %s\n\r", status,
                   SOCKET_ERROR_MSG);
        return status;
    }

    while(newsock < 0 || *pIsRunning)
    {
        /* This call accepts incoming client's connection requests.
         * Accept returns a new socket descriptor, which is dedicated for
         * the accepted client's session. Accept takes the 'welcome' socket
         * descriptor as socket descriptor.
         */
        newsock = lwip_accept(sock, (struct sockaddr *)csa, (socklen_t*)&addrSize);

        if((newsock<0 ) && ((-errno) == NS_ERROR_BSD_EAGAIN) && (TRUE == nb))
        {
            osi_Sleep(1);
        }
        else if(newsock < 0)
        {
            status = -errno;
            Report("[lwip_accept error:%d] %s nb configuration:%d \n\r", status,
                       SOCKET_ERROR_MSG, nb);

            goto exit;
        }
        else
        {
            Report("\n\r[lwip_accept]:Connected to client:\n\r");
            break;
        }
    }

    sAddr->in4.sin_addr.s_addr = htonl(sAddr->in4.sin_addr.s_addr);
    PrintIPAddress(FALSE,(void*)&sAddr->in4.sin_addr);

    Report(lineBreak);

    status = socket_ServerRecevicer(newsock, numberOfPackets, pIsRunning, isTCP, nb);

exit:
    lwip_close(newsock);

    return status;
}

/*****************************************************************************
 Callback Functions
 *****************************************************************************/
/*!
 \brief          Send callback.

 This routine shows how to send data in several configurations,
 As client, over TCP or UDP and also with various
 blocking or non-blocking socket option.

 \param          arg       -   Points to command line buffer.
 This container would be passed
 to the parser module.

 \return         Upon successful completion, the function shall return 0.
 In case of failure, this function would return -1.

 \sa             UDPServer, TCPServer, UDPClient, TCPClient

 */
//uint8_t frame = {0x18,0x03,0x73,0xB1,0x0E,0xDC,0x08,0x00,0x28,0xCC,0xDD,0xAA,0x08,0x00,0x45,0x00,0x03,0x3C,0x00,0x08,0x00,0x00,0xFF,0x11,0x93,0x6C,0xC0,0xA8,0x51,0xFD,0xC0,0xA8,0x51,0xEE,0xC1,0xC7,0x13,0x89,0x03,0x28,0xB7,0x3A,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
#ifdef CC35XX
int32_t cmdSendCallback(void *arg)
{
    int32_t ret = 0;
    SendCmd_t SendCmdParams;

    /* Call the command parser */
    memset(&SendCmdParams, 0x0, sizeof(SendCmd_t));
    ret = ParseSendCmd(arg, &SendCmdParams);

    if(ret < 0)
    {
        return -1;
    }

    uint8_t protocol = SendCmdParams.udpFlag? SOCK_DGRAM : SOCK_STREAM ;


    if (!SendCmdParams.server)
    {
        if (!SendCmdParams.numberOfPackets)//send infinity number of packets
        {
            // if passing 0 in this argument starting infinity thread for sending packets
            char *pName = protocol == SOCK_DGRAM ? "UDP client" : "TCP client";

            if(OSI_OK != socket_ThreadExecute((_SpawnEntryFunc_t)socket_ClientExecute, &SendCmdParams, pName, 0))
            {
                ret = -1;
                goto free;
            }
        }
        else
        {
            /*
            Report("\n\rWait For Arp...");
            {
                ip4_addr_t ipAddr;
                ipAddr.addr =htonl(SendCmdParams.ipAddr.ipv4);
                ret = (int32_t)update_arp(&ipAddr);
            }
            if(ret != ERR_OK)
            {
                Report("\n\rArp reply was not received!!!");
                ret = -1;
                goto free;
            }
            Report("\n\rArp reply received");
            */
            Report("\n\rSending traffic..");

            ret = Client(SendCmdParams.nb, SendCmdParams.destOrLocalPortNumber,
                          SendCmdParams.ipAddr, SendCmdParams.ipv6,
                          SendCmdParams.numberOfPackets,
                          TRUE,
                          protocol, SendCmdParams.bandwidth);
        }
        goto free;


    }
    // Not support in server mode
    else
    {
        Report("\n\r[SEND ERROR] - only supporting client in TX mode\n\r");
        ret = -1;
        goto free;
    }

free:
    return (ret);
}
#elif defined(CC33XX)
int32_t cmdSendCallback(void *arg)
{
    int32_t ret = 0;
    SendCmd_t SendCmdParams;

    /* Call the command parser */
    memset(&SendCmdParams, 0x0, sizeof(SendCmd_t));
    ret = ParseSendCmd(arg, &SendCmdParams);
    if (ret < 0)
    {
        return (-1);
    }

    uint8_t protocol = SendCmdParams.udpFlag? SOCK_DGRAM : SOCK_STREAM ;


    if (!SendCmdParams.server)
    {
        if (!SendCmdParams.numberOfPackets)//send infinity number of packets
        {
            // if passing 0 in this argument starting infinity thread for sending packets
            char *pName = protocol == SOCK_DGRAM ? "UDP client" : "TCP client";

            if(OSI_OK != socket_ThreadExecute((_SpawnEntryFunc_t)socket_ClientExecute, &SendCmdParams, pName, 0))
            {
                ret = -1;
                goto free;
            }
            goto free;
        }
        else
        {
            ret = Client(SendCmdParams.nb, SendCmdParams.destOrLocalPortNumber,
                          SendCmdParams.ipAddr, SendCmdParams.ipv6,
                          SendCmdParams.numberOfPackets,
                          TRUE,
                          protocol, SendCmdParams.bandwidth);
            goto free;
        }
    }
    // Not support in server mode
    else
    {
        Report("\n\r[SEND ERROR] - only supporting client in TX mode\n\r");
        ret = -1;
        goto free;
    }

free:
    return (ret);
}
#endif

/*!
 \brief          Prints Send command help menu.

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion,
 the function shall return 0.

 \sa             cmdSendCallback

 */
int32_t printSendUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(sendStr);
    Report(sendUsage1Str);
    Report("\t");
    Report(sendStr);
    Report(sendUsage2Str);
    Report(descriptionStr);
    Report(sendDetailsStr);
    Report(send_c_optionDetailsStr);
    //Report(send_s_optionDetailsStr);
    Report(send_u_optionDetailsStr);
    Report(send_p_optionDetailsStr);
    Report(send_nb_optionDetailsStr);
    Report(send_n_optionDetailsStr);
    Report(send_6_optionDetailsStr);
    Report(send_7_optionDetailsStr);
    Report(help_optaionDetails);
    Report(lineBreak);
    return (0);
}

/*!
 \brief          Receive callback.

 This routine shows how to Receive data in several configurations,
 As server, over TCP or UDP and also with various
 blocking or non-blocking socket option.

 \param          arg       -   Points to command line buffer.
 This container would be
 passed to the parser module.

 \return         Upon successful completion,
 the function shall return 0.
 In case of failure, this function would return -1.

 \sa             UDPServer, TCPServer, UDPClient, TCPClient

 */
int32_t cmdRecvCallback(void *arg)
{
    int32_t ret = 0;
    RecvCmd_t RecvCmdParams;

    /* Call the command parser */
    memset(&RecvCmdParams, 0x0, sizeof(RecvCmd_t));
    ret = ParseRecvCmd(arg, &RecvCmdParams);

    if (ret < 0)
    {
        return (-1);
    }
    uint8_t protocol = RecvCmdParams.udpFlag? SOCK_DGRAM : SOCK_STREAM ;

    /* In order to have RX statistics, we invoke sl_WlanRxStatStart().
     * the Variable 'rx_stat' Signify the status of the statistics request.
        int16_t rx_stat;
        rx_stat = sl_WlanRxStatStart();
     */

    if (RecvCmdParams.server)
    {
        if (!RecvCmdParams.numberOfPackets)
        {
            // if passing 0 in this argument starting infinity thread for receiving packets

                char *pName = protocol == SOCK_DGRAM ? "UDP server" : "TCP server";

                if(OSI_OK != socket_ThreadExecute((_SpawnEntryFunc_t)socket_ServerExecute, &RecvCmdParams, pName, 0))
                {
                    ret = -1;
                }
                goto free;
        }
        else
        {
            ret = Server(RecvCmdParams.nb, RecvCmdParams.destOrLocalPortNumber,
                         RecvCmdParams.ipv6, RecvCmdParams.numberOfPackets,
                         FALSE,
                         protocol);
        }



    }
    else
    {
        // Not support in server mode
        Report("\n\r[SEND ERROR] - only supporting server in RX mode\n\r");
        ret = -1;
        goto free;
    }

free:

    return (0);
}

/*!
 \brief          Prints Receive command help menu.

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdRecvCallback

 */
int32_t printRecvUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(recvStr);
    Report(recvUsage1Str);
    Report("\t");
    Report(recvStr);
    Report(recvUsage2Str);
    Report(descriptionStr);
    Report(recvDetailsStr);
    //Report(recv_c_optionDetailsStr);
    Report(recv_s_optionDetailsStr);
    Report(recv_u_optionDetailsStr);
    Report(recv_p_optionDetailsStr);
    Report(recv_nb_optionDetailsStr);
    Report(send_n_r_optionDetailsStr);
    Report(recv_6_optionDetailsStr);
    Report(help_optaionDetails);
    Report(recv_Note_optionDetailsStr);
    Report(lineBreak);
    return (0);
}


/*!
 \brief          Prints socket running thread

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdShowCallback

 */
int32_t cmdShowCallback(void *arg)
{
    socket_PrintRunningThread();
    return 0;
}
/*!
 \brief          Prints show command help menu.

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdShowCallback

 */
int32_t printShowUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(showStr);
    Report("\t");
    Report(showStr);
    Report(descriptionStr);
    Report(showDetailsStr);
    Report(help_optaionDetails);
    Report(lineBreak);
    return (0);
}

/*!
    \brief          Parse kill command.


    \param          arg            -   Points to command line buffer.
                                       Contains the command line typed by user.
    \param          id              -   Pointer to id.



    \return         Upon successful completion, the function shall return the RoleId number.
                    In case of failure,this function would print error,
                    and show the set scan policy command help menu.

    \sa             cmdKillCallback
 */
int32_t ParseKillCmd(void *arg, uint32_t *id)
{
    char cmdStr[CMD_BUFFER_LEN + 1];
    char *token = NULL;
    char *strId = NULL;
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
            strId = strtok(NULL, space_str);
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

    *id = atoi((const char*) strId);

    //check if role id valid
    if ((*id < 0) || (*id > 3))
    {
        Report("\r\n[Cmd Parser] : Invalid id. Range [0-3]\n\r");
        return (-1);
    }
    return (0);
}

/*!
 \brief          Prints show command help menu.

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdShowCallback

 */
int32_t printKillUsage(void *arg)
{
    Report(lineBreak);
    Report(usageStr);
    Report(killStr);
    Report("\t");
    Report(killUsageStr);
    Report(descriptionStr);
    Report(killDetailsStr);
    Report(help_optaionDetails);
    Report(lineBreak);
    return (0);
}


/*!
 \brief          Kill socket running thread by id

 \param          arg       -   Points to command line buffer.

 \return         Upon successful completion, the function shall return 0.

 \sa             cmdShowCallback

 */
int32_t cmdKillCallback(void *arg)
{
    int32_t id;

    int32_t ret;
    ret = ParseKillCmd(arg, (uint32_t*)&id);

    if(ret < 0)
    {
        printKillUsage(arg);
        return(0);
    }

    ret = socket_ThreadDestroy(id);
    if(ret != OSI_OK)
    {
        Report("\r\n[KILL ERROR] process in id %d is not running! call show_socket for see the running id's \n\r", id);
        SHOW_WARNING(WLAN_OSI_ERROR_BASE - ret, OS_ERROR_MSG);
        return -1;

    }
    return 0;
}


/****************************************************************************
 SERVER
 ****************************************************************************/
/*!
 \brief          Server.

 This routine shows how to set up a simple UDP/TCP Server.
 It shows sending and receiving packets as well.

 \param          nb              -   Sets the socket type:
 blocking or non-blocking.

 \param          portNumber      -   Decides which port is
 affiliated with the server's socket.

 \param          ipv6            -   Sets the version of
 the L3 IP protocol, IPv4 or IPv6.

 \param          numberOfPackets -   Sets the Number of
 packets to send \ receive.

 \param          tx              -   Decides if the function
 would transmit data. If this flag
 is set to false,
 this function would receive.

 \param          protocol              -   Decides which protocol
 SOCK_STREAM     1
 SOCK_DGRAM      2

 \return         Upon successful completion, the function shall return 0.
 In case of failure, this function would return -1;

 \sa             cmdSendCallback, cmdRecvCallback

 */
int32_t Server(uint8_t nb, uint16_t portNumber, uint8_t ipv6,
               uint32_t numberOfPackets, uint8_t tx, uint8_t protocol)
{
    int32_t sock;
    int32_t status;

    /* Contains the local ip address and port */
    struct sockaddr_in *sa;
    /* Contains the ip address and port of the connected peer. */
    struct sockaddr_in *csa;
    int32_t addrSize;
    sockAddr_t sAddr;

    struct timeval TimeVal;

    if (ipv6)
    {
#if LWIP_IPV6
        sAddr.in6.sin6_family = SL_AF_INET6;
        sAddr.in6.sin6_port = sl_Htons(portNumber);
        sAddr.in6.sin6_flowinfo = 0;

        sAddr.in6.sin6_addr._S6_un._S6_u32[0] =
            ((unsigned long *)ipAddress.ipv6)[0];
        sAddr.in6.sin6_addr._S6_un._S6_u32[1] =
            ((unsigned long *)ipAddress.ipv6)[1];
        sAddr.in6.sin6_addr._S6_un._S6_u32[2] =
            ((unsigned long *)ipAddress.ipv6)[2];
        sAddr.in6.sin6_addr._S6_un._S6_u32[3] =
            ((unsigned long *)ipAddress.ipv6)[3];

        sa = (SlSockAddr_t*)&sAddr.in6;
        addrSize = sizeof(SlSockAddrIn6_t);
#else
        return 0;
#endif
    }
    else
    {
        /* filling the TCP server socket address */
        /* Set socket family according to L3 Protocol:
         Ipv4 or Ipv6 - this is IPv4 case */
        sAddr.in4.sin_family = AF_INET;
        /* Set the server's port:
           We'll receive connection requests on this port */

        sAddr.in4.sin_port = PP_HTONS((unsigned short )portNumber);
        sAddr.in4.sin_addr.s_addr = INADDR_ANY;

        /* Since this is the client's side,
         * we must know beforehand the IP address
         * and the port of the server wer'e trying to send/receive UDP data to.
         */
        sa  = (struct sockaddr_in*) &sAddr.in4;
        csa = (struct sockaddr_in*) &sAddr.in4;
        addrSize = sizeof(struct sockaddr_in);

    }

    /* Get UDP/TCP sock descriptor - This call opens the socket. */
    sock = lwip_socket(sa->sin_family, protocol, 0);
    ASSERT_ON_ERROR(sock, SOCKET_ERROR_MSG);

    /* Bind socket to server's port */
    status = lwip_bind(sock, (struct sockaddr *)sa, (socklen_t)addrSize);
    if(status < 0)
    {
        Report("[error:%d] %s\n\r", status,
                   SOCKET_ERROR_MSG);
        lwip_close(sock);
        return(-1);
    }

    if(FALSE == nb)
    {
       /* In case of blocking, a timeout for sl_RecvFrom will be set to TimeVal
       * When timeout is expired sl_RecvFrom will return NS_ERROR_BSD_EAGAIN */
        TimeVal.tv_sec = RECEIVE_TIMEOUT;
        TimeVal.tv_usec = 0;
        status =
                lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                               (uint8_t*) &TimeVal, sizeof(TimeVal));
        if(status < 0)
        {
            Report("[error:%d] %s\n\r", status,
                       SOCKET_ERROR_MSG);
            lwip_close(sock);
            return(-1);
        }
    }

    if(SOCK_STREAM == protocol)
    {
        /***********************************************************************************/
        /*                             TCP SERVER                                          */
        /***********************************************************************************/
        status = socket_ServerTCP(sock,               // sock id
                                  numberOfPackets,    // number of packets
                                  NULL,               // Thread indicate
                                  nb,                 // nonblocking mode
                                  TRUE,               // TCP indicate
                                  addrSize,
                                  &sAddr,
                                  csa);

    }
    else
    {
        /***********************************************************************************/
        /*                             UDP SERVER                                          */
        /***********************************************************************************/
        status = socket_ServerRecevicer(sock,             // sock id
                                        numberOfPackets,  // number of packets
                                        NULL,             // Thread indicate
                                        FALSE,            // TCP indicate
                                        nb);              // nonblocking mode
    }

    Report("\n\rServer stopped close sock %d receive status %d\n\r", sock, status);

    status = lwip_close(sock);
    ASSERT_ON_ERROR(status, SOCKET_ERROR_MSG);

    return 0;

}

/*!
    \brief          socket_ServerExecute

    socket server execute thread for receiving UDP/TCP packets

    \param          pValue     -   Points to RecvCmd_t

    \return         void

 */
void socket_ServerExecute(void* pValue)
{
    SpawnThreadEntry_t *pSocketParam = (SpawnThreadEntry_t*)pValue;
    RecvCmd_t          *pServerParam = (RecvCmd_t *)pSocketParam->pParam;

    int32_t sock        = -1;
    int32_t status      =  0;

    uint8_t  protocol   = pServerParam->udpFlag? SOCK_DGRAM: SOCK_STREAM;
    uint16_t portNumber = pServerParam->destOrLocalPortNumber;
    uint8_t  nb         = pServerParam->nb;

    /* Contains the local ip address and port */
    struct sockaddr_in *sa;
    /* Contains the ip address and port of the connected peer. */
    struct sockaddr_in *csa;
    struct timeval opt;
    int32_t addrSize;
    sockAddr_t sAddr;


    Report("\n\r==========Thread Start %s==========\n\r", pSocketParam->pName);
    /* filling the TCP server socket address */
    /* Set socket family according to L3 Protocol:
     Ipv4 or Ipv6 - this is IPv4 case */
    sAddr.in4.sin_family = AF_INET;
    /* Set the server's port:
       We'll receive connection requests on this port */

    sAddr.in4.sin_port = PP_HTONS((unsigned short )portNumber);
    sAddr.in4.sin_addr.s_addr = INADDR_ANY;

    /* Since this is the client's side,
     * we must know beforehand the IP address
     * and the port of the server wer'e trying to send/receive UDP data to.
     */
    sa  = (struct sockaddr_in*) &sAddr.in4;
    csa = (struct sockaddr_in*) &sAddr.in4;
    addrSize = sizeof(struct sockaddr_in);

    /* Get UDP/TCP sock descriptor - This call opens the socket. */
    sock = lwip_socket(sa->sin_family, protocol, 0);
    if(status < 0)
    {
        Report("[lwip_socket error:%d] %s\n\r",  status,SOCKET_ERROR_MSG);
        goto exit;
    }

    /* Bind socket to server's port */
    status = lwip_bind(sock, (struct sockaddr *)sa, (socklen_t)addrSize);
    if(status < 0)
    {
        Report("[lwip_bind error:%d] %s\n\r", status,
                   SOCKET_ERROR_MSG);
        lwip_close(sock);
        goto exit;
    }

    //always works in NB= false mode
    opt.tv_sec = RECEIVE_TIMEOUT;
    opt.tv_usec = 0;

    status = lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&opt, sizeof(opt));
    if(status < 0)
    {
        Report("[lwip_setsockopt error:%d] %s\n\r", status,
                   SOCKET_ERROR_MSG);
        lwip_close(sock);
        goto exit;
    }

    if(SOCK_STREAM == protocol)
    {
        /***********************************************************************************/
        /*                             TCP SERVER                                          */
        /***********************************************************************************/
        status = socket_ServerTCP(sock,                         // sock id
                                  0,                            // number of packets
                                  &pSocketParam->bIsRunning,    // Thread indicate
                                  nb,                           // nonblocking mode
                                  TRUE,                         // TCP indicate
                                  addrSize,
                                  &sAddr,
                                  csa);

    }
    else
    {
        /***********************************************************************************/
        /*                             UDP SERVER                                          */
        /***********************************************************************************/
        status = socket_ServerRecevicer(sock,                       // sock id
                                        0,                          // number of packets
                                        &pSocketParam->bIsRunning,  // Thread indicate
                                        FALSE,                      // TCP indicate
                                        nb);                        // nonblocking mode
    }

    Report("\n\rServer stopped close sock %d receive status %d\n\r", sock, status);

    status = lwip_close(sock);

    if(status < 0)
    {
        Report("[lwip_close error:%d] %s\n\r",  status,SOCKET_ERROR_MSG);
        goto exit;
    }

exit:

    Report("\n\rPlease close process %d\n\r", pSocketParam->id);
    while(pSocketParam->bIsRunning)
    {
        const TickType_t xDelay = 5 / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
    }
    Report("\n\r==========Thread Stop  %s==========\n\r", pSocketParam->pName);
    osi_SyncObjSignal(&pSocketParam->syncObj);
    while(1);

}

/****************************************************************************
 CLIENT
 ****************************************************************************/
/*!
    \brief          socket_ClientExecute

    socket client execute thread for sending UDP/TCP packets

    \param          pValue     -   Points to SendCmd_t

    \return         void

 */
void socket_ClientExecute(void* pValue)
{
    SpawnThreadEntry_t *pSocketParam = (SpawnThreadEntry_t*)pValue;

    SendCmd_t          *pClientParam = (SendCmd_t *)pSocketParam->pParam;

    int32_t sock;
    int32_t status = 0;

    int32_t addrSize;
    sockAddr_t sAddr;
    struct sockaddr_in *sa;
    struct timeval opt;

    uint8_t  protocol   = pClientParam->udpFlag? SOCK_DGRAM: SOCK_STREAM;
    uint16_t portNumber = pClientParam->destOrLocalPortNumber;
    ip_t     ipAddress  = pClientParam->ipAddr;

    /* Set socket family according to L3 Protocol:
     Ipv4 or Ipv6 - this is IPv4 case */
    sAddr.in4.sin_family = AF_INET;
    /* Change the port number and IP
     address byte ordering from Host order (little endian)
     * to network order (Big endian) */
    sAddr.in4.sin_port = PP_HTONS((unsigned short )portNumber);
    sAddr.in4.sin_addr.s_addr = htonl((unsigned int )ipAddress.ipv4);

    /* Since this is the client's side,
     * we must know beforehand the IP address
     * and the port of the server wer'e trying to send/receive UDP data to.
     */
    sa = (struct sockaddr_in*) &sAddr.in4;
    addrSize = sizeof(struct sockaddr_in);


    /* Get UDP sock descriptor - This call opens the socket. */
    sock = lwip_socket(sa->sin_family, protocol, 0);
    if (sock < 0)
    {
        Report("[lwip_socket error:%d] %s\n\r", status, SOCKET_ERROR_MSG);
        goto exit;
    }

    status = lwip_connect(sock, (struct sockaddr*)sa, addrSize);

    opt.tv_sec = RECEIVE_TIMEOUT;
    opt.tv_usec = 0;

    status = lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&opt, sizeof(opt));
    if(status < 0)
    {
        Report("[lwip_setsockopt error:%d] %s\n\r", status,
                SOCKET_ERROR_MSG);
            lwip_close(sock);
            goto exit;
   }

    Report("\n\r==========Thread Start %s==========\n\r", pSocketParam->pName);

    status = socket_ClientTransceiver(sock,                             // sock id
                                      0,                                // number of packets
                                      &pSocketParam->bIsRunning, pClientParam->bandwidth);       // Thread indicate

    Report("\n\rClient stopped close sock %d transceiver status %d\n\r", sock, status);

    /* Calling 'close' with the socket descriptor,
     * once operation is finished.
     */
    status = lwip_close(sock);
    if (status < 0)
    {
        Report("[lwip_close error:%d] %s\n\r", status, SOCKET_ERROR_MSG);
    }

exit:
    Report("\n\rPrepare to close process %d\n\r", pSocketParam->id);
    while(pSocketParam->bIsRunning)
    {
        const TickType_t xDelay = 5 / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
    }

    Report("\n\r==========Thread Stop  %s==========\n\r", pSocketParam->pName);
    osi_SyncObjSignal(&pSocketParam->syncObj);
    while(1);
}


/*!
 \brief          Client.

 This routine shows how to set up a simple TCP/UDP Client.
 It shows sending and receiving packets as well.

 \param          nb              -   Sets the socket type:
 blocking or non-blocking.

 \param          portNumber      -   Decides which port is
 affiliated with the server's socket.

 \param          ipv6            -   Sets the version of
 the L3 IP protocol, IPv4 or IPv6.

 \param          numberOfPackets -   Sets the Number of
 packets to send \ receive.

 \param          tx              -   Decides if the function
 would transmit data. If this flag
 is set to false,
 this function would receive.

 \param          protocol              -   Decides which protocol
 SOCK_STREAM     1
 SOCK_DGRAM      2

 \return         Upon successful completion, the function shall return 0.
 In case of failure, this function would return -1;

 \sa             cmdSendCallback, cmdRecvCallback

 */
int32_t Client(uint8_t nb, uint16_t portNumber, ip_t ipAddress, uint8_t ipv6,
                  uint32_t numberOfPackets, uint8_t tx, uint8_t protocol,
                  uint32 bandwidth)
{
    int32_t sock;
    int32_t status;
    struct sockaddr_in *sa;
    int32_t addrSize;
    sockAddr_t sAddr;
    struct timeval TimeVal;


    if (ipv6)
    {
#if LWIP_IPV6
        sAddr.in6.sin6_family = SL_AF_INET6;
        sAddr.in6.sin6_port = sl_Htons(portNumber);
        sAddr.in6.sin6_flowinfo = 0;

        sAddr.in6.sin6_addr._S6_un._S6_u32[0] =
            ((unsigned long *)ipAddress.ipv6)[0];
        sAddr.in6.sin6_addr._S6_un._S6_u32[1] =
            ((unsigned long *)ipAddress.ipv6)[1];
        sAddr.in6.sin6_addr._S6_un._S6_u32[2] =
            ((unsigned long *)ipAddress.ipv6)[2];
        sAddr.in6.sin6_addr._S6_un._S6_u32[3] =
            ((unsigned long *)ipAddress.ipv6)[3];

        sa = (SlSockAddr_t*)&sAddr.in6;
        addrSize = sizeof(SlSockAddrIn6_t);
#else
        return 0;
#endif
    }
    else
    {
        /* Set socket family according to L3 Protocol:
         Ipv4 or Ipv6 - this is IPv4 case */
        sAddr.in4.sin_family = AF_INET;
        /* Change the port number and IP
         address byte ordering from Host order (little endian)
         * to network order (Big endian) */
        sAddr.in4.sin_port = PP_HTONS((unsigned short )portNumber);
        sAddr.in4.sin_addr.s_addr = htonl((unsigned int )ipAddress.ipv4);

        /* Since this is the client's side,
         * we must know beforehand the IP address
         * and the port of the server wer'e trying to send/receive UDP data to.
         */
        sa = (struct sockaddr_in*) &sAddr.in4;
        addrSize = sizeof(struct sockaddr_in);

    }

    /* Get UDP/TCP sock descriptor - This call opens the socket. */
    sock = lwip_socket(sa->sin_family, protocol, 0);
    ASSERT_ON_ERROR(sock, SOCKET_ERROR_MSG);

    status = lwip_connect(sock, (struct sockaddr*)sa, addrSize);

    if(FALSE == nb)
    {
       /* In case of blocking, a timeout for will be set to TimeVal
       * When timeout is expired sl_RecvFrom will return NS_ERROR_BSD_EAGAIN */
        TimeVal.tv_sec = RECEIVE_TIMEOUT;
        TimeVal.tv_usec = 0;
        status =
                lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                               (uint8_t*) &TimeVal, sizeof(TimeVal));
        if(status < 0)
        {
            Report("[error:%d] %s\n\r", status,
                       SOCKET_ERROR_MSG);
            lwip_close(sock);
            return(-1);
        }
    }
    status = socket_ClientTransceiver(sock,                // sock id
                                      numberOfPackets,     // number of packets
                                      NULL, bandwidth/*Mbps*/);               // Thread indicate

    ASSERT_ON_ERROR(status, SOCKET_ERROR_MSG);

    Report("\n\r Sent %u packets successfully", numberOfPackets);

    /* Calling 'close' with the socket descriptor,
     * once operation is finished.
     */
    status = lwip_close(sock);
    ASSERT_ON_ERROR(status, SOCKET_ERROR_MSG);

    return (0);
}



/*****************************************************************************
 Example Functions
 *****************************************************************************/
/** This is an example function that tests
 blocking- and nonblocking connect. */
void socket_connect(uint32_t ip)
{

    int s;
    int ret;
    uint8_t buffer[1400] = "Test";

    struct sockaddr_in addr;
    struct sockaddr_in addr_s;

    fdsets sets;

    const ip_addr_t *ipaddr = (const ip_addr_t*) (&ip);

    INIT_FDSETS(&sets);

    /* set up address to connect to */
    memset(&addr, 0, sizeof(addr));

    addr.sin_len = sizeof(addr);
    addr.sin_family = AF_INET;
    addr.sin_port = PP_HTONS(SOCK_TARGET_PORT);
    inet_addr_from_ip4addr(&addr.sin_addr, ip_2_ip4(ipaddr));
    //addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    s = lwip_socket(AF_INET, SOCK_DGRAM, 0);

    LWIP_ASSERT("s >= 0", s >= 0);
    uint32_t myip = 0x500000a;
    const ip_addr_t *ipaddr1 = (const ip_addr_t*) (&myip);
    /* connect */

    addr_s.sin_len = sizeof(addr);
    addr_s.sin_family = AF_INET;
    addr_s.sin_port = PP_HTONS(1234);
    inet_addr_from_ip4addr(&addr_s.sin_addr, ip_2_ip4(ipaddr1));


    ret = lwip_connect(s, (struct sockaddr*) &addr, sizeof(addr));

    /* should succeed */
    LWIP_ASSERT("ret == 0", ret == 0);

    ret = lwip_write(s, buffer, (100));

    osi_Sleep(2);

    /* write something */
    while (1)
    {
        ret = lwip_write(s, trafic, (500));
    }

    /* close */
    ret = lwip_close(s);

}
