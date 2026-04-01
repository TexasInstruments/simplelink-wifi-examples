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
#ifndef LWIP_SOCKET_EXAMPLES_H
#define LWIP_SOCKET_EXAMPLES_H
#include <stdint.h>


/* Network stack BSD SOCKET ERRORS CODES */
#define NS_ERROR_BSD_SOC_OK                             ( 0L)  /* Success.                                                             */
#define NS_ERROR_BSD_SOC_ERROR                          (-1L)  /* Failure */
#define NS_ERROR_BSD_EINTR                              (-4L)   /*Interrupted system call */
#define NS_ERROR_BSD_EMSGSIZE                           (-7L)  /* Message too big to be carried in a single transaction */
#define NS_ERROR_BSD_INEXE                              (-8L)   /* socket command in execution  */
#define NS_ERROR_BSD_EBADF                              (-9L)   /* Bad file number */
#define NS_ERROR_BSD_ENSOCK                             (-10L)  /* The system limit on the total number of open socket, has been reached */
#define NS_ERROR_BSD_EAGAIN                             (-11L)  /* Try again */
#define NS_ERROR_BSD_EWOULDBLOCK                        SL_ERROR_BSD_EAGAIN
#define NS_ERROR_BSD_ENOMEM                             (-12L)  /* Out of memory */
#define NS_ERROR_BSD_EACCES                             (-13L)  /* Permission denied */
#define NS_ERROR_BSD_EFAULT                             (-14L)  /* Bad address */
#define NS_ERROR_BSD_ECLOSE                             (-15L)  /* close socket operation failed to transmit all queued packets */
#define NS_ERROR_BSD_EALREADY_ENABLED                   (-21L)  /* Transceiver - Transceiver already ON. there could be only one */
#define NS_ERROR_BSD_EINVAL                             (-22L)  /* Invalid argument */
#define NS_ERROR_BSD_EAUTO_CONNECT_OR_CONNECTING        (-69L)  /* Transceiver - During connection, connected or auto mode started */
#define NS_ERROR_BSD_CONNECTION_PENDING                 (-72L)  /* Transceiver - Device is connected, disconnect first to open transceiver */
#define NS_ERROR_BSD_EUNSUPPORTED_ROLE                  (-86L)  /* Transceiver - Trying to start when WLAN role is AP or P2P GO */
#define NS_ERROR_BSD_EDESTADDRREQ                       (-89L)  /* Destination address required */
#define NS_ERROR_BSD_EPROTOTYPE                         (-91L)  /* Protocol wrong type for socket */
#define NS_ERROR_BSD_ENOPROTOOPT                        (-92L)  /* Protocol not available */
#define NS_ERROR_BSD_EPROTONOSUPPORT                    (-93L)  /* Protocol not supported */
#define NS_ERROR_BSD_ESOCKTNOSUPPORT                    (-94L)  /* Socket type not supported */
#define NS_ERROR_BSD_EOPNOTSUPP                         (-95L)  /* Operation not supported on transport endpoint */
#define NS_ERROR_BSD_EAFNOSUPPORT                       (-97L)  /* Address family not supported by protocol */
#define NS_ERROR_BSD_EADDRINUSE                         (-98L)  /* Address already in use */
#define NS_ERROR_BSD_EADDRNOTAVAIL                      (-99L)  /* Cannot assign requested address */
#define NS_ERROR_BSD_ENETUNREACH                        (-101L) /* Network is unreachable */
#define NS_ERROR_BSD_ENOBUFS                            (-105L) /* No buffer space available */
#define NS_ERROR_BSD_EOBUFF                             SL_ERROR_BSD_ENOBUFS
#define NS_ERROR_BSD_EISCONN                            (-106L) /* Transport endpoint is already connected */
#define NS_ERROR_BSD_ENOTCONN                           (-107L) /* Transport endpoint is not connected */
#define NS_ERROR_BSD_ETIMEDOUT                          (-110L) /* Connection timed out */
#define NS_ERROR_BSD_ECONNREFUSED                       (-111L) /* Connection refused */
#define NS_ERROR_BSD_EALREADY                           (-114L) /* Non blocking connect in progress, try again */



void socket_examples_init(void);

/* Function prototypes */

int32_t cmdSendCallback(void *arg);

int32_t printSendUsage(void *arg);

int32_t cmdRecvCallback(void *arg);

int32_t printRecvUsage(void *arg);

int32_t cmdShowCallback(void *arg);

int32_t printShowUsage(void *arg);

int32_t cmdKillCallback(void *arg);

int32_t printKillUsage(void *arg);


void killAllProcess();

#define HEAP_THRESHOLD_FOR_TX (20000)

#endif /* LWIP_SOCKET_EXAMPLES_H */
