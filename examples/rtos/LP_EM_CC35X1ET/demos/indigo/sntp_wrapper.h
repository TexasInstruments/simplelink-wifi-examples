#ifndef SNTP_WRAPPER_H
#define SNTP_WRAPPER_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>

#include <stdlib.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "osi_kernel.h"

void sntpWrapper_store_servers(uint32_t numOfServers,char* sntpServer1IP,char* sntpServer2Ip, char* sntpServer3Ip );
int32_t sntpWrapper_updateDateTime();



#ifdef __cplusplus
}
#endif
#endif
