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
#include <osi_kernel.h>

#ifdef RAM_CONTAINER
    #ifdef CONTAINERS_40MHZ
        #include "containers_40mhz.h"
    #else
        #include "containers.h"
    #endif
#endif

#include "ti/common/nv/nvintf.h"
#include "ti/common/nv/nvocmp.h"
#include "ti_flash_map_config.h"


/*!
    \brief the below functions are needed for file management of the upper mac.
           currently only supported file is the WiFi part firmware bin file.
           in the sitara AM243x launchpad, the firmware bin is saved in a specific flash offset.

           those functions are redirected from fxxx to osi_fxxx in the linker command (--symbol_map=fread=osi_fread).
           if already have system that use fopen fclose fread etc. this file should not be compiled in.
*/
#define ATTRIBUTE __attribute__ ((used))
#define NVID_BLE_OUR_SEC_OFFSET_IN_FLASH (0x1)
#define NVID_BLE_PEER_SEC_OFFSET_IN_FLASH (0x2)
#define NVID_BLE_CCCD_OFFSET_IN_FLASH (0x3)

#define NVID_WLAN_CONNECTION_POLICY_OFFSET_IN_FLASH (0x4)
#define NVID_WLAN_PROFILES_OFFSET_IN_FLASH (0x5)
#define NVID_WLAN_FAST_CONNECT_OFFSET_IN_FLASH (0x6)


typedef enum
{
    OSI_FILE_CONNECTIVITY_FW_SLOT_1,
    OSI_FILE_CONNECTIVITY_FW_SLOT_2,
    OSI_FILE_CONF,
    OSI_FILE_RAMBTLR,
    OSI_FILE_BLE_OUR_SEC,
    OSI_FILE_BLE_PEER_SEC,
    OSI_FILE_BLE_CCCD,
    OSI_FILE_WLAN_CONNECTION_POLICY,
    OSI_FILE_WLAN_PROFILE,
    OSI_FILE_WLAN_FAST_CONNECT,
}osiInternalType_e;

typedef struct
{
    osiInternalType_e ftype;
    void *ptr;
}osiFileP_t;

#define IS_OSI_FILE(osiFileType) ((osiFileType) == OSI_FILE_BLE_OUR_SEC || \
                                    (osiFileType) == OSI_FILE_BLE_PEER_SEC || \
                                    (osiFileType) == OSI_FILE_BLE_CCCD || \
                                    (osiFileType) == OSI_FILE_WLAN_CONNECTION_POLICY || \
                                    (osiFileType) == OSI_FILE_WLAN_PROFILE || \
                                    (osiFileType) == OSI_FILE_WLAN_FAST_CONNECT)

XMEM_Handle        fwHandle;
NVINTF_nvFuncts_t *nvFptrs  = NULL;
uint32_t           Fwslot   = OSI_FLASH_CONNECTIVITY_FW_SLOT_1;
uint32_t           FwGpeDataOffset = 0x101c;

int ATTRIBUTE osi_fset(osiFileSetType containerType, void *params)
{
    if(containerType == OSI_FILESYSTEM_SET_CONNECTIVITY_FW_CONTAINER)
    {
        osiFlashFwSlot_e osiFlashFwSlot = *(osiFlashFwSlot_e *)params;
        if(osiFlashFwSlot == OSI_FLASH_CONNECTIVITY_FW_SLOT_1)
        {
            Fwslot = OSI_FLASH_CONNECTIVITY_FW_SLOT_1;
        }
        else if(osiFlashFwSlot == OSI_FLASH_CONNECTIVITY_FW_SLOT_2)
        {
            Fwslot = OSI_FLASH_CONNECTIVITY_FW_SLOT_2;
        }
    }

    return 0;
}

int ATTRIBUTE osi_fget(osiFileGetType containerType, void *params)
{
    if(containerType == OSI_FILESYSTEM_GET_CONNECTIVITY_FW_CONTAINER)
    {
        osiFlashFwSlot_e *osiFlashFwSlot = (osiFlashFwSlot_e *)params;
        *osiFlashFwSlot = Fwslot;
    }

    return 0;
}

int ATTRIBUTE osi_fclose(FILE *_fp)
{
    osiFileP_t *osiFile;
    osiFile = (osiFileP_t *)_fp;

    if(NULL == osiFile)
    {
        return 0;
    }

    if((osiFile->ftype == OSI_FILE_CONNECTIVITY_FW_SLOT_1) || (osiFile->ftype == OSI_FILE_CONNECTIVITY_FW_SLOT_2))
    {
        XMEMWFF3_close((XMEM_Handle)osiFile->ptr);
    }
    else if((osiFile->ftype == OSI_FILE_BLE_OUR_SEC) || (osiFile->ftype == OSI_FILE_BLE_PEER_SEC) || (osiFile->ftype == OSI_FILE_BLE_CCCD))
    {
        if(osiFile->ptr != NULL)
        {
            nvFptrs->deleteItem((*(NVINTF_itemID_t *)osiFile->ptr));
            os_free(osiFile->ptr);
        }
    }
    else if((osiFile->ftype == OSI_FILE_WLAN_FAST_CONNECT) || (osiFile->ftype == OSI_FILE_WLAN_CONNECTION_POLICY) || (osiFile->ftype == OSI_FILE_WLAN_PROFILE))
    {
        if(osiFile->ptr != NULL)
        {
            os_free(osiFile->ptr);
        }
    }
    os_free(_fp);
    return 0;
}

FILE * ATTRIBUTE osi_fopen(const char *_fname, const char *_mode)
{
    osiFileP_t      *osiFile = NULL;
    NVINTF_itemID_t *nvItem = NULL;
    XMEM_Params      params;

    if(NULL == nvFptrs)
    {
        nvFptrs = os_malloc(sizeof(NVINTF_nvFuncts_t));
        NVOCMP_loadApiPtrs(nvFptrs);
        if (nvFptrs->initNV(NULL) != 0)
        {
            Report("\n\rInit NV failed\n\r");
            ASSERT_GENERAL(0);
        }
    }

    if(strcmp("rambtlr",_fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        osiFile->ftype = OSI_FILE_RAMBTLR;
        osiFile->ptr = NULL;
        return (FILE *)(osiFile);
    }
    else if(strcmp("fw",_fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        if(Fwslot == OSI_FLASH_CONNECTIVITY_FW_SLOT_1)
        {
            osiFile->ftype = OSI_FILE_CONNECTIVITY_FW_SLOT_1;
            params.regionBase =  wifi_connectivity_physical_slot_1_address;
            params.regionStartAddr = wifi_connectivity_logical_slot_1_address;
            params.regionSize = wifi_connectivity_slot_1_region_size;
            params.deviceNum = 0;
            fwHandle = XMEMWFF3_open(&params);
        }
        else if(Fwslot == OSI_FLASH_CONNECTIVITY_FW_SLOT_2)
        {
            osiFile->ftype = OSI_FILE_CONNECTIVITY_FW_SLOT_2;
            params.regionBase =  wifi_connectivity_physical_slot_2_address;
            params.regionStartAddr = wifi_connectivity_logical_slot_2_address;
            params.regionSize = wifi_connectivity_slot_2_region_size;
            params.deviceNum = 0;
            fwHandle = XMEMWFF3_open(&params);
        }
        osiFile->ptr = (void *)fwHandle;
        return (FILE *)(osiFile);
    }
    else if(strcmp("cc33xx-conf",_fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        osiFile->ftype = OSI_FILE_CONF;
        osiFile->ptr = (void *)gINIbuffer;
        return (FILE *)(osiFile);
    }
    else if(strcmp("our_sec",_fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        nvItem = os_malloc(sizeof(NVINTF_itemID_t));
        osiFile->ftype = OSI_FILE_BLE_OUR_SEC;
        nvItem->itemID = NVID_BLE_OUR_SEC_OFFSET_IN_FLASH;
        nvItem->systemID = NVINTF_SYSID_BLE;
        nvItem->subID = 0;
        osiFile->ptr = nvItem;
        return (FILE *)(osiFile);
    }
    else if(strcmp("peer_sec",_fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        nvItem = os_malloc(sizeof(NVINTF_itemID_t));
        osiFile->ftype = OSI_FILE_BLE_PEER_SEC;
        nvItem->itemID = NVID_BLE_PEER_SEC_OFFSET_IN_FLASH;
        nvItem->systemID = NVINTF_SYSID_BLE;
        nvItem->subID = 0;
        osiFile->ptr = nvItem;
        return (FILE *)(osiFile);
    }
    else if(strcmp("cccd", _fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        nvItem = os_malloc(sizeof(NVINTF_itemID_t));
        osiFile->ftype = OSI_FILE_BLE_CCCD;
        nvItem->itemID = NVID_BLE_CCCD_OFFSET_IN_FLASH;
        nvItem->systemID = NVINTF_SYSID_BLE;
        nvItem->subID = 0;
        osiFile->ptr = nvItem;
        return (FILE *)(osiFile);
    }
    else if(strcmp("conn_p", _fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        nvItem = os_malloc(sizeof(NVINTF_itemID_t));
        osiFile->ftype = OSI_FILE_WLAN_CONNECTION_POLICY;
        nvItem->itemID = NVID_WLAN_CONNECTION_POLICY_OFFSET_IN_FLASH;
        nvItem->systemID = NVINTF_SYSID_WIFI;
        nvItem->subID = 0;
        osiFile->ptr = nvItem;
        return (FILE *)(osiFile);
    }
    else if(strcmp("profiles", _fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        nvItem = os_malloc(sizeof(NVINTF_itemID_t));
        osiFile->ftype = OSI_FILE_WLAN_PROFILE;
        nvItem->itemID = NVID_WLAN_PROFILES_OFFSET_IN_FLASH;
        nvItem->systemID = NVINTF_SYSID_WIFI;
        nvItem->subID = 0;
        osiFile->ptr = nvItem;
        return (FILE *)(osiFile);
    }
    else if(strcmp(".fast", _fname) == 0)
    {
        osiFile = os_malloc(sizeof(osiFileP_t));
        nvItem = os_malloc(sizeof(NVINTF_itemID_t));
        osiFile->ftype = OSI_FILE_WLAN_FAST_CONNECT;
        nvItem->itemID = NVID_WLAN_FAST_CONNECT_OFFSET_IN_FLASH;
        nvItem->systemID = NVINTF_SYSID_WIFI;
        nvItem->subID = 0;
        osiFile->ptr = nvItem;
        return (FILE *)(osiFile);
    }

    return (FILE *)(NULL);
}

size_t ATTRIBUTE osi_fread(void *_ptr, size_t len, size_t offset, FILE *_fp)
{
    osiFileP_t *osiFile;
    osiFile = (osiFileP_t *)_fp;

    if(NULL == osiFile)
    {
        return 0;
    }

    if((osiFile->ftype == OSI_FILE_CONF) || (osiFile->ftype == OSI_FILE_RAMBTLR))
    {
        memcpy(_ptr,(void *)((uint32_t)(osiFile->ptr) + offset),len);
        return len;
    }
    else if((osiFile->ftype == OSI_FILE_CONNECTIVITY_FW_SLOT_1) || (osiFile->ftype == OSI_FILE_CONNECTIVITY_FW_SLOT_2))
    {

        XMEMWFF3_read((XMEM_Handle)osiFile->ptr, FwGpeDataOffset + offset, (void*)_ptr, len, 0);

    }
    else if(IS_OSI_FILE(osiFile->ftype))
    {
        if(nvFptrs->readItem((*(NVINTF_itemID_t *)osiFile->ptr), 0, len, _ptr) == 0)
        {
            return (len);
        }
        else
        {
            return 0;
        }
    }

    return 0;
}

size_t osi_fwrite(const void *_ptr, size_t _size, size_t _count, FILE *_fp)
{
    osiFileP_t *osiFile;
    osiFile = (osiFileP_t *)_fp;

    if(NULL == osiFile)
    {
        return 0;
    }

    if(IS_OSI_FILE(osiFile->ftype))
    {
        nvFptrs->deleteItem((*(NVINTF_itemID_t *)osiFile->ptr));
        if(nvFptrs->writeItem((*(NVINTF_itemID_t *)osiFile->ptr), _size, (void *)_ptr) == 0)
        {
            return (_size);
        }
        else
        {
            return (0);
        }
    }

    return 0;
}

int osi_fremove(FILE *_fp)
{
    osiFileP_t *osiFile;
    osiFile = (osiFileP_t *)_fp;

    if(NULL == osiFile)
    {
        return -1;
    }

    if(osiFile->ftype == OSI_FILE_WLAN_FAST_CONNECT)
    {
        if(nvFptrs->deleteItem((*(NVINTF_itemID_t *)osiFile->ptr)) == 0)
        {
            return 0;
        }
        else
        {
            return -1;
        }
    }

    return 0;

}

/*!
    \brief initialize the SPI module
    \param data - input buffer
    \param length - length in bytes to read
    \return 0 - success, -1 - failed
    \note
    1. if initializing all the MCU drivers at the beginning of the world, this function can stay empty.
       this function is called in the wlan_start and will stay initialized even if used wlan_stop
    2. the SPI must have the following parameters
       - master
       - 4 pin mode SPI
       - chip select active low
       - polarity 0 phase 0
       - max frequency 40000000Hz
       - data block 32bits
    \warning
*/
size_t ATTRIBUTE osi_filelength(const char * FileName)
{
    return 0;
}
//

