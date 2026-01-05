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
/*
 *  ======== sntp.c ========
 *  Contains the SNTP client 'daemon' (a Task that runs every 30 minutes to
 *  sync our time with time value received from an NTP server).
 */
#include "sntp_task.h"

#include <stdlib.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

/* Board Header files */
#include "ti_drivers_config.h"
#include "uart_term.h"
//LWIP
#include "network_lwip.h"
//ERRORS
#include "errors.h"
#include "osi_kernel.h"

#include "lwipopts.h"

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/inet.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/api.h"
#include "lwip/netbuf.h"



#include "date_time_service.h"

#ifdef SNTP_SUPPORT
 struct sockaddr;
 struct sockaddr_in;
#ifdef LWIP_IPV6
 struct sockaddr_in6;
#endif

#define SNTP_MSGQ_MAX_NUM_MSGS 5
/* Time to wait after receiving a valid server update (milliseconds) */
#define SNTP_WAIT_TIME 1800000
 /* Time to wait to the servers to be defined */
#define SNTP_WAIT_TIME_FOR_SERVERS_CONFIG 1800


/* Time to wait for reply from server (mili sec) */
#define SNTP_REPLY_WAIT_TIME 5000

/* Time to wait before retrying if socket operations fail (milliseconds) */
#define SNTP_SOCKFAIL_WAIT_TIME 30000

/* Max number of Kiss o' Death (KOD) packets to recv before switching server */
#define SNTP_KOD_MAX 4

/* Time to back off (wait) after receiving a KOD packet (milliseconds) */
#define SNTP_KOD_BACKOFF_TIME 15000

/* KOD error code: rate exceeded, server requesting NTP client to back off */
#define SNTP_KOD_RATE_STR "RATE"
#define SNTP_KOD_RATE_CODE 3

/* KOD error code: access denied, server requests client to end all comm */
#define SNTP_KOD_DENY_STR "DENY"
#define SNTP_KOD_DENY_CODE 2

/* KOD error code: access denied, server requests client to end all comm */
#define SNTP_KOD_RSTR_STR "RSTR"
#define SNTP_KOD_RSTR_CODE 1

/* Size of KOD error codes */
#define SNTP_KOD_ERROR_CODE_SIZE 4

#define SNTP_THRD_PRIORITY (11)

#define SNTP_LOCAL_PORT (5008)

 /*
  *  Time Base Conversion Macros
  *
  *  The NTP timebase is 00:00 Jan 1 1900.  The local
  *  time base is 00:00 Jan 1 1970.  Convert between
  *  these two by added or substracting 70 years
  *  worth of time.  Note that 17 of these years were
  *  leap years.
  */
 #define TIME_BASEDIFF        ((((unsigned int)70 * 365 + 17) * 24 * 3600))
 #define TIME_NTP_TO_LOCAL(t) ((t) - TIME_BASEDIFF)
 #define TIME_LOCAL_TO_NTP(t) ((t) + TIME_BASEDIFF)


 typedef enum
 {
     /* EVENT ID USED TO CAUSE THE SNTP DAEMON TO WAKE UP AND EXIT */
     SNTP_EVENT_STOP,

     /* EVENT ID USED TO CAUSE THE SNTP DAEMON TO WAKE UP AND UPDATE THE TIME */
     SNTP_EVENT_FORCEUPDATE,
}sntpTask_events_t;

/* Use NTP version 4 */
#define SNTP_VERSION 4

/* Flag value for unsync'ed leap indicator field, signifying server error */
#define SNTP_NOSYNC 3

/* NTP mode defined in RFC 4330 */
#define SNTP_MODE_CLIENT 3

/* Well known SNTP server port */
#define SNTP_SERVER_PORT 123

/* sntpTask_syncTime_task Task defaults */
#define SNTP_TASK_STACKSIZE (1200)

/* SNTP Header (as specified in RFC 4330) */
typedef struct _SNTP_Header {
    /*
     *  'flags' stores three values:
     *
     *    - 2 bit Leap Indicator (LI)
     *    - 3 bit Version Number (VN)
     *    - 3 bit Mode.
     */
    uint8_t flags;
    uint8_t stratum;
    uint8_t poll;
    int8_t   precision;
    int32_t           rootDelay;
    uint32_t  rootDispersion;
    uint32_t  referenceID;

    /* NTP time stamps */
    uint32_t referenceTS[2];
    uint32_t originateTS[2];
    uint32_t receiveTS[2];
    uint32_t transmitTS[2];
} _SNTP_Header;


/*
 *  Define types and global variables used for calling set and get time
 *  functions.
 */
typedef uint32_t (*GetTimeFxn)(void);
typedef void (*SetTimeFxn)(uint32_t newtime);
typedef void (*SntpCbFxn)(void *);

static GetTimeFxn g_sntpGetTimeCB = NULL;
static SetTimeFxn g_sntpSetTimeCB = NULL;
static SntpCbFxn g_sntpTimeUpdCb = NULL;

/* The list of NTP servers to communicate with */
static struct sockaddr *g_sntpServerList = NULL;

/* The number of NTP servers in the g_sntpServerList */
static uint32_t g_sntpNumservers = 0;

/* The byte offset of the current server in the g_sntpServerList */
static int32_t g_sntpCurrSrvBytePos = 0;

/* The current server number (used to know when last server in list reached) */
static int32_t g_sntpCurrSrvNum = 0;

/* Flag used to avoid re-initialization */
static int32_t g_sntpInitialized = 0;

/* Semaphore used for global variable protection */
OsiLockObj_t        g_sntpMutex = NULL;

/* Event used to signal sntpTask_syncTime_task Task to exit or force a time update */
static OsiMsgQ_t g_sntpMsgQueue = NULL;

/* Semaphore used to signal sntpTask_stop that sntpTask_syncTime_task has completed */
OsiSyncObj_t g_sntpTaskExitSignal = NULL;

/* Handle to the sntpTask_syncTime_task Task */
static TaskHandle_t  g_sntpSyncHandle = NULL;

static struct netconn* sntpConn = NULL;

/* Function prototypes */
static inline int32_t getSocketError(int32_t ret);
static void changeServer(uint8_t family);
static int32_t hasKissCode(char *str);
static int32_t sntpConnSetup(struct netconn **con, struct sockaddr *cs, uint32_t waitTime);
static void sntpTask_syncTime_task(void* arg);



/*
 *  ======== getSocketError ========
 */
static inline int32_t getSocketError(int32_t ret)
{
#if defined (NET_SL)
    return (ret);
#else
    return (errno);
#endif
}

/*
 *  ======== changeServer ========
 *
 *  Utility function to move the current server to point32_t to the next one in the
 *  list.  If we're currently at the last server in list, then reset back to
 *  the beginning of g_sntpServerList.
 *
 *  The 'family' parameter is the family type of the current server (AF_INET or
 *  AF_INET6).
 *
 *  Must be called with appropriate Semaphore protection (using g_sntpMutex)
 */
static void changeServer(uint8_t family)
{
    if (g_sntpCurrSrvNum == (g_sntpNumservers - 1)) {
        /*
         *  Edge case: current server is last server in g_sntpServerList, reset
         *  g_sntpCurrSrvNum and g_sntpCurrSrvBytePos back to first server in list.
         */
        g_sntpCurrSrvNum = 0;
        g_sntpCurrSrvBytePos = 0;
    }
    else {
        /* Update server number to the next server in our list */
        g_sntpCurrSrvNum++;

        /*
         *  Update byte offset to point32_t to next server. Number of bytes to
         *  move depends on the family type of the current server.
         */
        if (family == AF_INET) {
            g_sntpCurrSrvBytePos += sizeof(struct sockaddr);
        }
#ifdef LWIP_IPV6
        else if (family == AF_INET6) {
            g_sntpCurrSrvBytePos += sizeof(struct sockaddr);
        }
#endif
    }
}

/*
 *  ======== hasKissCode ========
 *
 *  Utility function to check if a string contains a Kiss O' Death code.
 *
 *  Returns:
 *      SNTP_KOD_RATE_CODE - str contains "RATE" KOD code
 *
 *      SNTP_KOD_DENY_CODE - str contains "DENY" KOD code
 *
 *      SNTP_KOD_RSTR_CODE - str contains "RSTR" KOD code
 *
 *      0 - str does not contain any of the above KOD codes
 */
static int32_t hasKissCode(char *str)
{
    if (strncmp((char *)SNTP_KOD_RATE_STR, str, SNTP_KOD_ERROR_CODE_SIZE)
            == 0) {

        return (SNTP_KOD_RATE_CODE);
    }
    else if (strncmp((char *)SNTP_KOD_DENY_STR, str, SNTP_KOD_ERROR_CODE_SIZE)
            == 0) {

        return (SNTP_KOD_DENY_CODE);
    }
    else if (strncmp((char *)SNTP_KOD_RSTR_STR, str, SNTP_KOD_ERROR_CODE_SIZE)
            == 0) {

        return (SNTP_KOD_RSTR_CODE);
    }
    else {
        return (0);
    }
}

/*
 *  ======== sntpConnSetup ========
 *
 *  Utility function to create and connect the SNTP client socket a socket.
 *  This socket can then be used to generically call send() and recv() for
 *  either IPv4 or IPv6.
 *
 *  Returns 0 on success, 1 on failure.
 */
static int32_t sntpConnSetup(struct netconn **con, struct sockaddr *cs, uint32_t waitTime)
{
    int32_t status;
    ip_addr_t remote_ip;
    uint16_t remote_port = 0;
    struct netconn* pCon = NULL;


    /* If socket already exists close it */
    if ( *con != NULL) {
        netconn_delete(pCon);
        *con = NULL;
    }

    /* Create a UDP socket to communicate with NTP server */
    // Create new UDP netconn
    pCon = netconn_new(NETCONN_UDP);
    if (pCon == NULL) {
        SNTP_PRINT_REPORT_ERROR("\n\r ERROR ! sntpConnSetup:failed to allocate  connection handle");
        *con = NULL;
        return (1);
    }

    /*
     *  Connect our UDP socket. We only want to recv replies from the NTP
     *  server on this socket:
     */
    netconn_bind(pCon, IP_ADDR_ANY, 0/*SNTP_LOCAL_PORT*/);//lwip will choose the local port

    /*though there is no UDP connect, this is a way to make lwip
     * to filter the recv frames according addr+port */

    if(cs->sa_family == AF_INET){
        os_memcpy(&remote_ip, &((struct sockaddr_in *)cs)->sin_addr, sizeof(struct ip4_addr));
        remote_port = ((struct sockaddr_in *)cs)->sin_port;
    }
#if LWIP_IPV6
    else{
        os_memcpy(remote_ip.u_addr.ip6, ((struct sockaddr_in6 *)cs)->sin6_addr, sizeof(struct ip6_addr));
        remote_port = ((struct sockaddr_in6 *)cs)->sin_port;
    }
#endif
    status = netconn_connect(pCon, &remote_ip, remote_port);
    if (status !=  ERR_OK) {
        SNTP_PRINT_REPORT_ERROR("\n\r sntpConnSetup:connect to IP: 0x%x port:%d failed", remote_ip,remote_port);
        netconn_delete(pCon);
        *con = NULL;
        return (1);
    }

    //set the timeout to *to seconds
    netconn_set_recvtimeout(pCon, waitTime);


    *con = pCon;
    /* Socket creation success */
    return (0);
}

/*
 *  ======== sntpTask_start ========
 */
int32_t sntpTask_start(uint32_t (*get)(void), void (*set)(uint32_t newtime),
        void (*timeUpdatedHook)(void *),struct sockaddr *storedServers,
        uint32_t numservers, size_t stacksize)
{
    int32_t ret = 0;

    /* Don't re-initialize before sntpTask_stop is called */
    if (g_sntpInitialized) {
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_start: must call sntpTask_stop first");
        goto EXIT_FAIL;
    }

    /* Validate parameters */
    if (!get || !set || !storedServers || numservers == 0) {
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_start: invalid parameters passed"
                "0x%x 0x%x 0x%x %d",
                get,set,storedServers, numservers);
        goto EXIT_FAIL;
    }

    /* Assign our time getter and setter to functions passed by user */
    g_sntpGetTimeCB = (GetTimeFxn)get;
    g_sntpSetTimeCB = (SetTimeFxn)set;
    /* Store time update callback function, if the user specified one */
    if (timeUpdatedHook) {
        g_sntpTimeUpdCb = timeUpdatedHook;
    }

    ret = osi_LockObjCreate(&g_sntpMutex);
    if (ret != OSI_OK) {
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_start: Failed to create g_sntpMutex Semaphore");
        goto EXIT_FAIL;
    }

    /* Event used to signal sntpTask_syncTime_task Task to exit or force a time update */
    ret = osi_MsgQCreate(&g_sntpMsgQueue,
                            "sntpQueue",
                            sizeof(sntpMsg_t),
                            SNTP_MSGQ_MAX_NUM_MSGS
                            );

    if(ret != OSI_OK)
    {
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_start: Failed to create g_sntpMsgQueue Event");
        goto EXIT_FAIL;
    }

    /* Semaphore used to signal sntpTask_stop that sntpTask_syncTime_task has completed */
    ret = osi_SyncObjCreate(&g_sntpTaskExitSignal);
    if (ret != OSI_OK) {
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_start: Failed to create g_sntpTaskExitSignal Semaphore");
        goto EXIT_FAIL;
    }

    /*
     *  Create SNTP client daemon
     *
     *  This task will communicate with an SNTP server and sync the system
     *  time to the time received by the server (which is accurately
     *  maintained).
     */
    int32_t rc = xTaskCreate(sntpTask_syncTime_task,
                            "sntp",
                            ((stacksize == 0) ? SNTP_TASK_STACKSIZE : stacksize),
                            NULL,
                            SNTP_THRD_PRIORITY,
                            &g_sntpSyncHandle);
    if (rc != TRUE ) {
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_start: Failed to create sntpTask_syncTime_task Task");
        goto EXIT_FAIL;
    }

    /* Set flag indicating that initialization has completed */
    g_sntpInitialized = 1;

    /* call set servers here */
    sntpTask_configServers(storedServers, numservers);

    /* Return success */
    return (0);

EXIT_FAIL:
    /* If any *_create() fails, clean up any successful previous allocations */
    if (g_sntpMutex) {
        osi_LockObjDelete(&g_sntpMutex);
        g_sntpMutex = NULL;
    }
    if (g_sntpMsgQueue) {
        osi_MsgQDelete(&g_sntpMsgQueue);
        g_sntpMsgQueue = NULL;
    }
    if (g_sntpTaskExitSignal) {
        osi_SyncObjDelete(&g_sntpTaskExitSignal);
        g_sntpTaskExitSignal = NULL;
    }

    /* Return failure */
    return (1);
}

/*
 *  ======== sntpTask_stop ========
 */
void sntpTask_stop(void)
{
    sntpMsg_t sntpMsg;
    int32_t ret;
    /* Don't free anything unless we've already initialized! */
    if (!g_sntpInitialized) {
        return;
    }

    /*
     *  Signal sntpTask_syncTime_task Task to exit
     *
     *  Posting the g_sntpMsgQueue Event (with SNTP_EVENT_STOP) will cause the
     *  Task to break out of its while loop and exit. The Task will then be
     *  cleaned up automatically in the BIOS idle loop (we know this will
     *  happen because NDK requires that Task.deleteTerminatedTasks == TRUE)
     *
     *  Before exiting, sntpTask_syncTime_task will post the g_sntpTaskExitSignal Semaphore to let
     *  this function know that it has completed and to continue clean up.
     */
    sntpMsg.msgId = SNTP_EVENT_STOP;
    osi_MsgQWrite(&g_sntpMsgQueue, (void *)&sntpMsg, OSI_NO_WAIT, OSI_FLAG_NOT_FROM_INTR);

    /* Wait for sntpTask_syncTime_task to complete/exit */
    ret = osi_SyncObjWait(&g_sntpTaskExitSignal,OSI_WAIT_FOREVER);
    if(ret != 0){
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_stop : error!!! failed to wait  syn object g_sntpTaskExitSignal");
    }


    /* Delete dynamically allocated objects */
    osi_SyncObjDelete(&g_sntpTaskExitSignal);
    osi_MsgQDelete(&g_sntpMsgQueue);
    osi_LockObjDelete(&g_sntpMutex);

    /* Reset all global variables */
    g_sntpTaskExitSignal = NULL;
    g_sntpMsgQueue = NULL;
    g_sntpMutex = NULL;
    g_sntpGetTimeCB = NULL;
    g_sntpSetTimeCB = NULL;
    g_sntpTimeUpdCb = NULL;
    g_sntpNumservers = 0;
    g_sntpServerList = NULL;
    g_sntpCurrSrvBytePos = 0;
    g_sntpCurrSrvNum = 0;

    /* Set flag indicating that de-initialization has completed */
    g_sntpInitialized = 0;
}

/*
 *  ======== sntpTask_configServers ========
 */
void sntpTask_configServers(struct sockaddr *storedServers, uint32_t numservers)
{
    int32_t ret;
    /* check for invalid args and ensure g_sntpMutex has been created */
    if ((!storedServers) || !g_sntpInitialized) {
        SNTP_PRINT_REPORT_ERROR("\r\n Error! servers can't be configured , servers addr:0x%x g_sntpInitialized:%d",
                (uint32_t)storedServers,g_sntpInitialized);
        return;
    }

    ret = osi_LockObjLock(&g_sntpMutex,OSI_WAIT_FOREVER);
    if(ret != 0){
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_configServers : error!!! failed to wait  syn object g_sntpMutex");
        return;
    }
    g_sntpServerList = storedServers;
    g_sntpNumservers = numservers;

    /* Reset current server tracking variables to beginning of g_sntpServerList */
    g_sntpCurrSrvBytePos = 0;
    g_sntpCurrSrvNum = 0;

    ret = osi_LockObjUnlock(&g_sntpMutex);
    if(ret != 0){
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_configServers : error!!! failed to signal  syn object g_sntpMutex");
    }

}

/*
 *  ======== sntpTask_forceTimeSync ========
 */
int32_t sntpTask_forceTimeSync(void)
{
    sntpMsg_t sntpMsg;

    if (!g_sntpInitialized) {
        return -1;
    }

    /*
     *  Disable tasks to ensure our event is valid before posting it (It's
     *  possible to be interrupted here and for sntpTask_stop() to delete the
     *  event before the program returns to this point32_t and tries posting it).
     */
    vTaskSuspend(g_sntpSyncHandle);
    /* Unblock the SNTP daemon to allow communication with NTP server */
    if (g_sntpMsgQueue) {
        sntpMsg.msgId = SNTP_EVENT_FORCEUPDATE;
        osi_MsgQWrite(&g_sntpMsgQueue, (void *)&sntpMsg, OSI_NO_WAIT, OSI_FLAG_NOT_FROM_INTR);
    }
    vTaskResume(g_sntpSyncHandle);

    return 0;
}

/*
 *  ======== sntpTask_syncTime_task ========
 *
 *  Task that runs every 30 minutes to synchronize the local time
 *  with the actual time received from the NTP servers.
 */
static void sntpTask_syncTime_task(void* arg)
{
    struct sockaddr *currServer;
    uint32_t timeout = SNTP_REPLY_WAIT_TIME;
    struct netbuf *send_buf,*recv_buf;
    _SNTP_Header sntpPkt;
    int32_t active = 1;
    int32_t numKod = 0;
    int32_t kodBackOffTime = 0;
    int32_t forceTimeSync = 0;
    int32_t status = 0;
    int32_t i = 0;
    sntpMsg_t sntpMsg;
    int32_t rc, ret = 0;
    uint16_t numRecvBytes;
    char *mem;

    /* Initial wait time before sending request to server */
    uint32_t waitTime = 0;

    while (active) {
        /* Sleep here in between server requests */
        rc= osi_MsgQRead(&g_sntpMsgQueue, &sntpMsg, waitTime);
        if(rc == OSI_OK){

            if (sntpMsg.msgId == SNTP_EVENT_STOP) {
                /*
                 *  If sntpTask_stop() is called, exit out of the loop. (Note that
                 *  this case takes precedence if both events were posted;
                 *  i.e. stop "wins").
                 */
                active = 0;
                goto CONTINUE;
            }
            else if (sntpMsg.msgId == SNTP_EVENT_FORCEUPDATE) {
                /* If sntpTask_forceTimeSync() is called, sync with NTP server */
                forceTimeSync = 1;
            }
        }
        if(g_sntpNumservers == 0)//no server is defined
        {
            waitTime = SNTP_WAIT_TIME_FOR_SERVERS_CONFIG;
            continue;//wait for the servers to be defined
        }

        /*
         *  Send our request to the current NTP server.  If our current server
         *  responds, then exit this loop, update the time and go back to
         *  sleep for SNTP_WAIT_TIME.
         *
         *  If our current server doesn't respond (e.g. server went down, no
         *  route to server, etc.) then try the next one.
         *
         *  Keep trying until one responds or until we've tried all of the
         *  servers in the g_sntpServerList. (We only want to try each server once
         *  per wake up cycle).
         *
         *  If we've tried all servers in our list, and did not receive a
         *  response from any of them (e.g. all servers are down!), then we
         *  sleep for SNTP_WAIT_TIME and try the process again upon waking up.
         *
         *  Entire for loop below is guarded by our Semaphore.  We can't allow
         *  changes to the server list while we are actively trying to
         *  communicate with an NTP server(s).
         */
        ret = osi_LockObjLock(&g_sntpMutex,OSI_WAIT_FOREVER);
        if(ret != 0){
            SNTP_PRINT_REPORT_ERROR("\n\rSync time : error!!! failed to wait  syn object g_sntpMutex");
        }

        for (i = 0; i < g_sntpNumservers; i++) {
            /* Set/update the current server */
            currServer = (struct sockaddr *)((uint8_t *)g_sntpServerList +
                    g_sntpCurrSrvBytePos);

            /* Skip invalid servers (it's invalid if it has a kiss code) */
            if (hasKissCode((char *)currServer->sa_data)) {
                changeServer(currServer->sa_family);
                continue;
            }

            /*
             *  (Re)create socket using family that matches current server.
             *  If the current server's family is IPv6, then this will create
             *  an IPv6 socket.  If family is IPv4, it creates an IPv4 socket.
             */
            if (sntpConnSetup(&sntpConn, currServer, timeout) != 0) {
                /*
                 *  Most likely, stack not up yet or not enough memory.
                 *  Wait and try again.
                 */
                waitTime = SNTP_SOCKFAIL_WAIT_TIME;
                SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_syncTime_task: socket create/init failed");
                goto CONTINUE;
            }

            /* Initialize the SNTP packet, setting version and mode = client */
            memset(&sntpPkt, 0, sizeof(_SNTP_Header));
            sntpPkt.flags = SNTP_VERSION << 3;
            sntpPkt.flags |= SNTP_MODE_CLIENT;

            /* Set packet's transmit time to the current time on our clock */
            sntpPkt.transmitTS[0] = htonl(TIME_LOCAL_TO_NTP((*g_sntpGetTimeCB)()));

            send_buf = netbuf_new();
            if(!send_buf)
            {
                SNTP_PRINT_REPORT_ERROR("\n\rERROR ! Could not allocate netbuf");
                goto CONTINUE;
            }
            mem = netbuf_alloc(send_buf, sizeof(_SNTP_Header));
            if (mem == NULL) {
                SNTP_PRINT_REPORT_ERROR("\n\rERROR !Could not allocate memory for sending NTP req");
               goto CONTINUE;
            }
            os_memcpy(mem, &sntpPkt, sizeof(_SNTP_Header));
            // Send to remote IP/port
            ret = netconn_send(sntpConn, send_buf);
            if (ret != ERR_OK) {
                netbuf_delete(send_buf);
                SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_syncTime_task:"
                        "warning send NTP req to server: 0x%x failed due to err :%d(err_t)"
                        " ,moving to the next NTP server",
                       (uint32_t) ((struct sockaddr_in *)currServer)->sin_addr.s_addr,/*ip is on reverse order*/
                        ret);
                if(ret == ERR_RTE){ // route was not found
                    changeServer(currServer->sa_family);
                    continue;
                }
                else {
                    goto CONTINUE;
                }
            }
            netbuf_delete(send_buf);

            /* Wait for the reply */
            ret = netconn_recv(sntpConn, &recv_buf);

            numRecvBytes = 0;
            if((ret == ERR_OK) && (recv_buf != NULL)){
                numRecvBytes = netbuf_len(recv_buf);
            }

            if ((numRecvBytes <= 0) || (numRecvBytes != sizeof(_SNTP_Header))) {
                SNTP_PRINT_REPORT_ERROR("\r\nsntpTask_syncTime_task: "
                        "recvfrom 0x%x failed (%d)(err_enum_t)"
                        ", moving to the next server",
                        (uint32_t) ((struct sockaddr_in *)currServer)->sin_addr.s_addr,
                        (numRecvBytes <= 0) ? ret
                        : numRecvBytes);

                if(recv_buf != NULL){
                    netbuf_delete(recv_buf);
                    recv_buf = NULL;
                }
                /* Try the next server in our list */
                changeServer(currServer->sa_family);
                continue;
            }

            if((ret == ERR_OK) && (recv_buf != NULL)) {
                /*
                 *  We got a response from the current server. Retrieve the NTP
                 *  packet from the socket and update our time.
                 */
                memset(&sntpPkt, 0, sizeof(_SNTP_Header));
                os_memcpy(&sntpPkt, recv_buf->p->payload, sizeof(_SNTP_Header));
                netbuf_delete(recv_buf);

                /* Check for errors in server response */
                if (sntpPkt.stratum == 0) {

                    /* Per RFC5905, we MUST handle Kiss O' Death packet */
                    if ((sntpPkt.flags >> 6) == SNTP_NOSYNC) {

                        /* KOD recv'd. Inspect kiss code & handle accordingly */
                        status = hasKissCode((char *)&sntpPkt.referenceID);

                        if (status == SNTP_KOD_RATE_CODE) {
                            /* Server requests that we reduce our send rate */
                            if (numKod++ < SNTP_KOD_MAX) {
                                /* Back off our wait time and retry server */
                                kodBackOffTime += SNTP_KOD_BACKOFF_TIME;
                                waitTime = kodBackOffTime;
                                goto CONTINUE;
                            }
                            else {
                                /* Too many KOD packets. Move to next server */
                                numKod = 0;
                                kodBackOffTime = 0;
                                waitTime = 0;
                                changeServer(currServer->sa_family);

                                continue;
                            }
                        }
                        /* Check for fatal kiss codes */
                        else if (status == SNTP_KOD_DENY_CODE ||
                                status == SNTP_KOD_RSTR_CODE) {
                            /*
                             *  Server requests we end all communication. Mark
                             *  current server as invalid (use kiss code to mark
                             *  it)
                             */
                            os_memcpy(&currServer->sa_data, &sntpPkt.referenceID,
                                    SNTP_KOD_ERROR_CODE_SIZE);
                            changeServer(currServer->sa_family);

                            continue;
                        }
                        /* Per RFC5905, other kiss codes are ignored */
                    }
                    else {
                        /*
                         *  A server response with stratum == 0, with no kiss
                         *  code, is a fatal error. Mark server as invalid
                         */
                        os_memcpy(&currServer->sa_data,
                                (char *)SNTP_KOD_DENY_STR,
                                SNTP_KOD_ERROR_CODE_SIZE);
                    }
                }

                /* Use server's transmit time to update our clock */
                (*g_sntpSetTimeCB)( TIME_NTP_TO_LOCAL(ntohl(sntpPkt.transmitTS[0])));

                /* If time update was forced, notify of success */
                if (forceTimeSync && g_sntpTimeUpdCb) {
                    (*g_sntpTimeUpdCb)(NULL);
                    forceTimeSync = 0;
                }

                /* Reset KOD count and time once a valid response is received */
                numKod = 0;
                kodBackOffTime = 0;

                /*
                 *  Successful update from our current server, break out and
                 *  sleep for SNTP_WAIT_TIME before updating the time again.
                 */
                break;
            }
            else {
                /* Our current server didn't respond */
                if (status < 0) {
                    SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_syncTime_task: waiting for server reply (%d)",
                            getSocketError(status));
                }

                /* Try the next server in our list */
                changeServer(currServer->sa_family);
            }
        }
        waitTime = SNTP_WAIT_TIME;
CONTINUE:
        ret = osi_LockObjUnlock(&g_sntpMutex);
        if(ret != 0){
            SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_syncTime_task : error!!! failed to signal  syn object g_sntpMutex");
        }


        /* Clean up and wait until it's time to sync with server again */
        if (sntpConn != NULL) {
            netconn_delete(sntpConn);
            sntpConn = NULL;
        }
    }

    /* Signal sntpTask_stop that sntpTask_syncTime_task Task has completed and exit */
    ret = osi_SyncObjSignal(&g_sntpTaskExitSignal);
    if(ret != 0){
        SNTP_PRINT_REPORT_ERROR("\n\rsntpTask_syncTime_task : error!!! failed to signal  syn object g_sntpTaskExitSignal");
    }

    vTaskDelete(NULL);

}
#endif
