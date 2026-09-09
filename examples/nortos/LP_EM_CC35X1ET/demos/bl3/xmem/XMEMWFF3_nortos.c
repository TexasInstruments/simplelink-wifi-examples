/*
 * Copyright (c) 2024-2026, Texas Instruments Incorporated
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
 *  ========== XSPIWFF3.c ==========
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


#include "xmem/XMEMWFF3_nortos.h"
#include "xmem/flash/FlashWFF3_nortos.h"
/* IRQ masking — inline assembly, no CMSIS header needed */
static inline void __disable_irq(void) { __asm volatile("cpsid i" ::: "memory"); }
static inline void __enable_irq(void)  { __asm volatile("cpsie i" ::: "memory"); }

#include <ti/devices/DeviceFamily.h>

#define PHY_OFFSET             12
#define PHYSICAL_ADDR          0
#define LOGIC_ADDR             1
#define VERIFBUF_SIZE_IN_BYTES 256
#define WRITE_SIZE_IN_BYTES    256
#define READ_SIZE_IN_BYTES     256
#define PAGE_SIZE              0x100

static int_fast16_t checkEraseRange(XMEM_Handle_nortos handle, size_t offset, size_t size);
static int_fast16_t doErase(XMEM_Handle_nortos handle, size_t offset, size_t size);
static uint32_t ConvertOffsetToAddr(XMEM_Handle_nortos handle, size_t offset, size_t AddrType);
static void flashTypeSelection(void);

const uint8_t XMEM_count_nortos = XMEM_NUM_HANDLER_NORTOS;
static uint8_t xmem_num  = 0; /* number of open handlers */

/*
 *  Mutex to synchronize access to flash region.
 */

static size_t sectorBaseMask; /* for efficient argument checking */
static void *verifyBuf;       /*!< Write Pre/Post verify buffer */
static uint8_t verifyBuffer[VERIFBUF_SIZE_IN_BYTES];
static void *xmemDataToWriteBuf;                               /*!< Write  buffer */
static uint8_t xmemDataToWriteBuffer[WRITE_SIZE_IN_BYTES + 4]; /*!< handling unaligned write */
static void *xmemDataToReadBuf;                                /*!< Read  buffer */
static uint8_t xmemDataToReadBuffer[READ_SIZE_IN_BYTES + 4];   /*!< handling unaligned read */

extern uint32_t _Boot_report_address[];
#define BOOT_REPORT_ADDR ((uintptr_t)(_Boot_report_address))

static bool isInitialized = false;

XMEMWFF3_HWAttrs_nortos XMEMWFF3_hwAttrs_nortos;

XMEMWFF3_Object_nortos XMEMWFF3_objects_nortos[XMEM_NUM_HANDLER_NORTOS];

XMEM_Config_nortos XMEM_config_nortos[CONFIG_XMEM_COUNT_NORTOS] = {{.object = &(XMEMWFF3_objects_nortos), .hwAttrs = &(XMEMWFF3_hwAttrs_nortos)}};

/*
 *  ======== XMEMWFF3_close_nortos ========
 */
int_fast16_t XMEMWFF3_close_nortos(XMEM_Handle_nortos handle)
{
    if (handle == NULL)
    {
        return XMEM_STATUS_INVALID_PARAMS_NORTOS;
    }

    XMEMWFF3_Object_nortos *object;


    object = handle->object;
    memset(object, 0, sizeof(XMEMWFF3_Object_nortos));

    free(handle);

    xmem_num--;


    return XMEM_STATUS_SUCCESS_NORTOS;
}

/*
 *  ======== XMEMWFF3_erase_nortos ========
 */
int_fast16_t XMEMWFF3_erase_nortos(XMEM_Handle_nortos handle, size_t offset, size_t size)
{
    int_fast16_t status;

    if (handle == NULL)
    {
        return (XMEM_STATUS_ERROR_NORTOS);
    }

    XMEMWFF3_Object_nortos const *object;

    object = handle->object;

    /* STIG operation are not supported for PSRAM */
    if (object->deviceNum == XMEM_PSRAM_NORTOS)
    {
        return XMEM_STATUS_ERROR_NORTOS;
    }


    status = doErase(handle, offset, size);


    return (status);
}

/*
 *  ======== XMEMWFF3_getAttrs_nortos ========
 */
void XMEMWFF3_getAttrs_nortos(XMEM_Handle_nortos handle, XMEM_Attrs_nortos *attrs)
{
    if (handle == NULL || attrs == NULL)
    {
        return;
    }

    XMEMWFF3_HWAttrs_nortos const *hwAttrs;

    hwAttrs = handle->hwAttrs;

    attrs->flashType = hwAttrs->flashType;
}

/*
 *  ======== XMEMWFF3_getObject_nortos ========
 */
void XMEMWFF3_getObject_nortos(XMEM_Handle_nortos handle, XMEMWFF3_Object_nortos *object)
{
    if (handle == NULL || object == NULL)
    {
        return;
    }

    XMEMWFF3_Object_nortos const *obj;

    obj = handle->object;

    memcpy(object, obj, sizeof(XMEMWFF3_Object_nortos));
}

/*
 *  ======== XMEMWFF3_getActiveHandlers_nortos ========
 */
uint8_t XMEMWFF3_getActiveHandlers_nortos(void)
{
    return xmem_num;
}

/*
 *  ======== XMEMWFF3_init_nortos ========
 */
void XMEMWFF3_init_nortos(void)
{
    if (!isInitialized)
    {
        isInitialized = true;

        /* Initialize variables */
        verifyBuf = (void *)&verifyBuffer;

        /* Select flash type */
        flashTypeSelection();
    }
}

/*
 *  ======== XMEMWFF3_lock_nortos =======
 */
int_fast16_t XMEMWFF3_lock_nortos(XMEM_Handle_nortos handle, uint32_t timeout)
{
    if (handle == NULL)
    {
        return (XMEM_STATUS_ERROR_NORTOS);
    }

    XMEMWFF3_Object_nortos *object = handle->object;
    if (object->mutexKey == 0)
    {
        return (XMEM_STATUS_TIMEOUT_NORTOS);
    }

    return (XMEM_STATUS_SUCCESS_NORTOS);
}

/*
 *  ======== XMEMWFF3_open_nortos =======
 */
XMEM_Handle_nortos XMEMWFF3_open_nortos(XMEM_Params_nortos *params)
{
    XMEMWFF3_Object_nortos *object;
    const XMEMWFF3_HWAttrs_nortos *hwAttrs;
    XMEM_Handle_nortos handle = NULL;
    int i;

    if (params == NULL)
    {
        return (NULL);
    }

    XMEMWFF3_init_nortos();
    /* Confirm that 'init' has successfully completed */

    /* Verify max number of handlers */
    if (xmem_num >= XMEM_count_nortos)
    {
        return (NULL);
    }


    /* Allocate XMEM handle */
    handle = malloc(sizeof(XMEM_Config_nortos));
    if (handle == NULL)
    {
            return (NULL);
    }

    /* Pointing to hwAttrs*/
    handle->hwAttrs = XMEM_config_nortos[0].hwAttrs;

    /* Pointing to the beginning of array of objects */
    object = (XMEMWFF3_Object_nortos *)(XMEM_config_nortos[0].object);

    /* Check for available slot */
    for (i = 0; i < XMEM_count_nortos; i++)
    {
        /* Check if current object slot is available (not opened) */
        if (false == object->opened)
        {
            break;
        }

        /* Ther is not available slot */
        if (i == XMEM_count_nortos - 1)
        {
            free(handle);
                    return (NULL);
        }

        /* Current object is opened, move to the next object in the array */
        object = object + 1;
    }

    hwAttrs = handle->hwAttrs;

    object->deviceNum       = params->deviceNum;
    object->regionBase      = params->regionBase;
    object->regionStartAddr = params->regionStartAddr;
    object->regionSize      = params->regionSize;

    sectorBaseMask = ~(hwAttrs->flashType.sectorSize - 1);

    /* The region cannot be smaller than a sector size */
    if (object->regionSize < hwAttrs->flashType.sectorSize)
    {
        memset(object, 0, sizeof(XMEMWFF3_Object_nortos));
        free(handle);
            return (NULL);
    }

    /* The region size must be a multiple of sector size */
    if (object->regionSize != (object->regionSize & sectorBaseMask))
    {
        memset(object, 0, sizeof(XMEMWFF3_Object_nortos));
        free(handle);
            return (NULL);
    }

    /* Validate target device */
    if (object->deviceNum != XMEM_FLASH_NORTOS && object->deviceNum != XMEM_PSRAM_NORTOS)
    {
        memset(object, 0, sizeof(XMEMWFF3_Object_nortos));
        free(handle);
            return (NULL);
    }

    object->opened = true;

    /* object is ready, handle points to a found object */
    handle->object = object;

    xmem_num++;


    return (handle);
}

/*
 *  ======== XMEMWFF3_read_nortos =======
 */
int_fast16_t XMEMWFF3_read_nortos(XMEM_Handle_nortos handle, size_t offset, void *buffer, size_t bufferSize, uint_fast16_t flags)
{
    if (handle == NULL || buffer == NULL || bufferSize == 0)
    {
        return (XMEM_STATUS_ERROR_NORTOS);
    }

    /* Validate that flags only contain allowed bits */
    if (flags & ~(XMEM_READ_STIG_NORTOS))
    {
        return (XMEM_STATUS_INVALID_PARAMS_NORTOS);
    }

    XMEMWFF3_Object_nortos const *object;

    object = handle->object;

    /* Prepare Read buffer */
    xmemDataToReadBuf = (void *)&xmemDataToReadBuffer;

    if (flags & XMEM_READ_STIG_NORTOS)
    {
        /* STIG operation are not supported for PSRAM */
        if (object->deviceNum == XMEM_PSRAM_NORTOS)
        {
            return XMEM_STATUS_ERROR_NORTOS;
        }

        /*  Check if offset + word size exceeds page boundary */
        if (((offset & 0xFF) + XMEM_WORD_SIZE_NORTOS) > PAGE_SIZE)
        {
            return XMEM_STATUS_ERROR_NORTOS;
        }

        uint32_t status = 0;

        uint32_t startAddrPhy = ConvertOffsetToAddr(handle, offset, PHYSICAL_ADDR);

        if (bufferSize < XMEM_WORD_SIZE_NORTOS)
        {
            return XMEM_STATUS_ERROR_NORTOS;
        }
        else
        {
            bufferSize = XMEM_WORD_SIZE_NORTOS; /* length of reading from STIG is always 1 word which is 4 Bytes */
        }

        if (offset + bufferSize > object->regionSize)
        {
            return (XMEM_STATUS_INV_OFFSET_NORTOS);
        }

        /*
         *  Get exclusive access to the region.  We don't want someone
         *  else to erase the region while we are reading it.
         */

        __disable_irq();

        FlashSetTickPeriod_nortos(1); /* nortos: SYSTIM_O_TIME1U already in microseconds */

        status = FlashReadSTIG_nortos(startAddrPhy, xmemDataToReadBuf, object->deviceNum);

        /* Copy data to buffer */
        memcpy(buffer, xmemDataToReadBuf, XMEM_WORD_SIZE_NORTOS);

        __enable_irq();

        if (status != 0)
        {
                return XMEM_STATUS_ERROR_NORTOS;
        }

    }
    else
    {
        int remainingBytes = bufferSize;
        uint32_t flashlogicAddr;
        uint32_t alignment;

        if (offset + bufferSize > object->regionSize)
        {
            return (XMEM_STATUS_INV_OFFSET_NORTOS);
        }

        /*
         *  Get exclusive access to the region.  We don't want someone
         *  else to erase the region while we are reading it.
         */

        /* Handling the constraint of reading from 4 Bytes aligned address */
        alignment = offset % 4;

        /* Handle the constraint of reading multiples of 256 Bytes at a time. */
        while (remainingBytes >= READ_SIZE_IN_BYTES)
        {
            /* Set address to read from
             * If offset is not align, set the address to be align*/
            flashlogicAddr = ConvertOffsetToAddr(handle, offset, LOGIC_ADDR) - alignment;

            /* Read from flash
             * flashlogicAddr is align, read 256 bytes*/
            FlashRead_nortos((uint32_t *)flashlogicAddr, (uint32_t *)xmemDataToReadBuf, READ_SIZE_IN_BYTES);

            /* Copy data to buffer.
             * If offset is not aligned, we need to align the src address and read size*/
            memcpy(buffer, xmemDataToReadBuf + alignment, READ_SIZE_IN_BYTES - alignment);

            /* Advance buffer, subtract remain size, Advance offset */
            buffer += READ_SIZE_IN_BYTES - alignment;
            remainingBytes -= (READ_SIZE_IN_BYTES - alignment);
            offset += READ_SIZE_IN_BYTES - alignment;

            /* Now the offset is aligned, alignment should be 0 for the rest of the read loop*/
            alignment = 0;
        }

        /* Less than 256-bytes to read:
         * - Either after the loop for remaining bytes
         * - Or for first time read with less than 256-bytes length
         */
        if (remainingBytes > 0)
        {
            /* Handling the constraint of reading from addresses that exceed region size  */
            if (offset + READ_SIZE_IN_BYTES > object->regionSize)
            {
                alignment = offset & 0xFF;
            }

            /* Set address to read from
             * If offset is not align, set the address to be align*/
            flashlogicAddr = ConvertOffsetToAddr(handle, offset, LOGIC_ADDR) - alignment;

            /* Check if the true alignment + remainingBytes exceeds buffer size */
            if ((alignment + remainingBytes) > READ_SIZE_IN_BYTES)
            {
                /* Calculate how much we can read in first operation */
                size_t firstReadSize = READ_SIZE_IN_BYTES - alignment;

                /* First read operation */
                FlashRead_nortos((uint32_t *)flashlogicAddr, (uint32_t *)xmemDataToReadBuf, READ_SIZE_IN_BYTES);
                memcpy(buffer, xmemDataToReadBuf + alignment, firstReadSize);

                /* Second read operation for remaining bytes */
                uint8_t *nextBuffer = (uint8_t *)buffer + firstReadSize;
                size_t nextSize     = remainingBytes - firstReadSize;
                uint32_t nextAddr   = (uint32_t)flashlogicAddr + READ_SIZE_IN_BYTES;

                FlashRead_nortos((uint32_t *)nextAddr, (uint32_t *)xmemDataToReadBuf, READ_SIZE_IN_BYTES);
                memcpy((uint8_t *)nextBuffer, xmemDataToReadBuf, nextSize);
            }
            else
            {
                /* Read from flash
                 * flashlogicAddr is aligned, read 256 bytes.
                 */
                FlashRead_nortos((uint32_t *)flashlogicAddr, (uint32_t *)xmemDataToReadBuf, READ_SIZE_IN_BYTES);

                /* Copy data to buffer.
                 * If offset is not aligned, we need to align the src address and read size
                 */
                memcpy(buffer, xmemDataToReadBuf + alignment, remainingBytes);
            }
        }

    }

    return (XMEM_STATUS_SUCCESS_NORTOS);
}

/*
 *  ======== XMEMWFF3_unlock_nortos =======
 */
void XMEMWFF3_unlock_nortos(XMEM_Handle_nortos handle)
{
    if (handle == NULL)
    {
        return;
    }
}

/*
 *  ======== XMEMWFF3_write_nortos =======
 */
int_fast16_t XMEMWFF3_write_nortos(XMEM_Handle_nortos handle, size_t offset, void *buffer, size_t bufferSize, uint_fast16_t flags)
{
    if (handle == NULL || buffer == NULL || bufferSize == 0)
    {
        return (XMEM_STATUS_ERROR_NORTOS);
    }

    /* Validate that flags only contain allowed bits */
    if (flags & ~(XMEM_WRITE_ERASE_NORTOS | XMEM_WRITE_PRE_VERIFY_NORTOS | XMEM_WRITE_POST_VERIFY_NORTOS | XMEM_WRITE_STIG_NORTOS))
    {
        return (XMEM_STATUS_INVALID_PARAMS_NORTOS);
    }

    XMEMWFF3_HWAttrs_nortos const *hwAttrs;
    XMEMWFF3_Object_nortos const *object;

    hwAttrs = handle->hwAttrs;
    object  = handle->object;

    unsigned int size;
    uint32_t status              = 0;
    uint32_t flashlogicStartAddr = ConvertOffsetToAddr(handle, offset, LOGIC_ADDR);
    uint32_t flashlogicAddr      = flashlogicStartAddr;
    uint32_t startAddrPhy        = ConvertOffsetToAddr(handle, offset, PHYSICAL_ADDR);
    uint32_t *srcBuf, *dstBuf;
    uint8_t *_srcBuf, *_dstBuf;
    uint32_t totalSize;
    size_t originalOffset = offset;
    void *originalBuffer  = buffer;
    int i, j;
    int retval = XMEM_STATUS_SUCCESS_NORTOS;

    /* Validate offset and bufferSize */
    if (offset + bufferSize > object->regionSize)
    {
        return (XMEM_STATUS_INV_OFFSET_NORTOS);
    }

    /* Validate buffer size for STIG write */
    if (flags & XMEM_WRITE_STIG_NORTOS)
    {
        /* STIG operation are not supported for PSRAM */
        if (object->deviceNum == XMEM_PSRAM_NORTOS)
        {
            return XMEM_STATUS_ERROR_NORTOS;
        }

        /*  Check if offset + word size exceeds page boundary */
        if (((offset & 0xFF) + XMEM_WORD_SIZE_NORTOS) > PAGE_SIZE)
        {
            return XMEM_STATUS_ERROR_NORTOS;
        }

        if (bufferSize < XMEM_WORD_SIZE_NORTOS)
        {
            return XMEM_STATUS_ERROR_NORTOS;
        }
        else
        {
            bufferSize = XMEM_WORD_SIZE_NORTOS;
        }
    }

    /* Single-threaded BL3: no mutex needed */

    /* Prepare Write buffer */
    xmemDataToWriteBuf = (void *)&xmemDataToWriteBuffer;

    /* If erase is set, erase destination sector(s) first */
    if (flags & XMEM_WRITE_ERASE_NORTOS)
    {
        size = bufferSize & sectorBaseMask;
        if (bufferSize & (~sectorBaseMask))
        {
            size += hwAttrs->flashType.sectorSize;
        }

        /* Validate offset is not aligned on a sector boundary */
        if (offset != (offset & sectorBaseMask))
        {
                return (XMEM_STATUS_INV_ALIGNMENT_NORTOS);
        }

        retval = doErase(handle, offset, size);
        if (retval != XMEM_STATUS_SUCCESS_NORTOS)
        {
                return (retval);
        }
    }
    else if (flags & XMEM_WRITE_PRE_VERIFY_NORTOS)
    {
        /*
         *  If pre-verify, each destination byte must be able to be changed to the
         *  source byte (1s to 0s, not 0s to 1s).
         *  this is satisfied by the following test:
         *     src == (src & dst)
         */

        /* Validate that the verification buffer has been configured */
        if ((verifyBuf == NULL) || (hwAttrs->flashType.verifyBufSize == 0))
        {
                return (XMEM_STATUS_VERIFYBUFFER_NORTOS);
        }

        /* Start reading from the requested start address */
        flashlogicAddr = offset;

        /* Set the number of bytes to read */
        totalSize = bufferSize;

        j = 0;

        while (totalSize > 0)
        {
            if (totalSize > hwAttrs->flashType.verifyBufSize)
            {
                size = hwAttrs->flashType.verifyBufSize;
            }
            else
            {
                size = totalSize;
            }

            /* Read destination before the change for pre-verify expected change */
            _dstBuf = (uint8_t *)verifyBuf;

            XMEMWFF3_read_nortos(handle, flashlogicAddr, _dstBuf, size, 0);

            _srcBuf = (uint8_t *)buffer;

            for (i = 0; i < size; i++, j++)
            {
                if (_srcBuf[j] != (_srcBuf[j] & _dstBuf[i]))
                {
                                return (XMEM_STATUS_INV_WRITE_NORTOS);
                }
            }

            flashlogicAddr = flashlogicAddr + size;
            totalSize      = totalSize - size;
        }
    }

    if (flags & XMEM_WRITE_STIG_NORTOS)
    {
        /* Copy to Write buffer */
        memcpy(xmemDataToWriteBuffer, buffer, bufferSize);

        srcBuf = (uint32_t *)xmemDataToWriteBuf;
        dstBuf = (uint32_t *)startAddrPhy;

        __disable_irq();

        FlashSetTickPeriod_nortos(1); /* nortos: SYSTIM_O_TIME1U already in microseconds */

        status = FlashWriteSTIG_nortos(srcBuf, dstBuf, object->deviceNum);
        __enable_irq();
    }
    else
    {
        srcBuf             = (uint32_t *)xmemDataToWriteBuf;
        size               = bufferSize;
        dstBuf             = (uint32_t *)flashlogicStartAddr;
        int remainingBytes = size;
        uint32_t alignment;

        /* Handling the constraint of writing from 4 Bytes aligned address */
        alignment = offset % 4;

        /* Handle the constraint of writing multiples of 256 Bytes at a time. */
        while (remainingBytes >= WRITE_SIZE_IN_BYTES)
        {
            /* Set address to write
             * If offset is not align, set the address to be align*/
            dstBuf = (uint32_t *)(ConvertOffsetToAddr(handle, offset, LOGIC_ADDR) - alignment);

            /* Read from flash
             * dstBuf is align, read 256 bytes*/
            FlashRead_nortos((uint32_t *)dstBuf, (uint32_t *)srcBuf, READ_SIZE_IN_BYTES);

            /* Copy data to from read buffer to write buffer.
             * If offset is not aligned, copy data to correct place in write buffer*/
            memcpy((uint8_t *)srcBuf + alignment, buffer, READ_SIZE_IN_BYTES - alignment);

            /* Write to flash */
            FlashWrite_nortos((uint32_t *)srcBuf, (uint32_t *)dstBuf, WRITE_SIZE_IN_BYTES);

            /* Advance buffer, subtract remain size, Advance offset */
            buffer += READ_SIZE_IN_BYTES - alignment;
            remainingBytes -= (READ_SIZE_IN_BYTES - alignment);
            offset += READ_SIZE_IN_BYTES - alignment;

            /* Now the offset is aligned, alignment should be 0 for the rest of the read loop*/
            alignment = 0;
        }

        /* Less than 256-bytes to write:
         * - Either after the loop for remaining bytes
         * - Or for first time write with less than 256-bytes length
         */
        if (remainingBytes > 0)
        {
            /* Handling the constraint of writing to addresses that exceed region size  */
            if (offset + WRITE_SIZE_IN_BYTES > object->regionSize)
            {
                alignment = offset & 0xFF;
            }

            /* Set address to read from
             * If offset is not align, set the address to be align*/
            dstBuf = (uint32_t *)(ConvertOffsetToAddr(handle, offset, LOGIC_ADDR) - alignment);

            /* Check if the true alignment + remainingBytes exceeds buffer size */
            if (alignment + remainingBytes > WRITE_SIZE_IN_BYTES)
            {
                /* Calculate how much we can write in first operation */
                size_t firstWriteSize = WRITE_SIZE_IN_BYTES - alignment;

                /* First write operation */
                FlashRead_nortos((uint32_t *)dstBuf, (uint32_t *)srcBuf, READ_SIZE_IN_BYTES);
                memcpy((uint8_t *)srcBuf + alignment, buffer, firstWriteSize);
                FlashWrite_nortos((uint32_t *)srcBuf, (uint32_t *)dstBuf, WRITE_SIZE_IN_BYTES);

                /* Second write operation for remaining bytes */
                uint8_t *nextBuffer = (uint8_t *)buffer + firstWriteSize;
                size_t nextSize     = remainingBytes - firstWriteSize;
                uint32_t nextAddr   = (uint32_t)dstBuf + WRITE_SIZE_IN_BYTES;

                FlashRead_nortos((uint32_t *)nextAddr, (uint32_t *)srcBuf, READ_SIZE_IN_BYTES);
                memcpy((uint8_t *)srcBuf, nextBuffer, nextSize);
                FlashWrite_nortos((uint32_t *)srcBuf, (uint32_t *)nextAddr, WRITE_SIZE_IN_BYTES);
            }
            else
            {
                /* Read from flash
                 * dstBuf is aligned, read 256 bytes.
                 */
                FlashRead_nortos((uint32_t *)dstBuf, (uint32_t *)srcBuf, READ_SIZE_IN_BYTES);

                /* Copy data from read buffer to write buffer.
                 * If offset is not aligned, copy data to correct place in write buffer */
                memcpy((uint8_t *)srcBuf + alignment, buffer, remainingBytes);

                /* Write to flash */
                FlashWrite_nortos((uint32_t *)srcBuf, (uint32_t *)dstBuf, WRITE_SIZE_IN_BYTES);
            }
        }
    }

    if (status != 0)
    {
        retval = XMEM_STATUS_ERROR_NORTOS;
    }
    else if (flags & XMEM_WRITE_POST_VERIFY_NORTOS)
    {
        /*
         *  Note: This validates the entire bufferSize even on erase mode.
         */

        /* Validate that the verification buffer has been configured */
        if ((verifyBuf == NULL) || (hwAttrs->flashType.verifyBufSize == 0))
        {
                return (XMEM_STATUS_VERIFYBUFFER_NORTOS);
        }

        /* Start reading from the requested start address */
        flashlogicAddr = originalOffset;

        /* Set the number of bytes to read */
        totalSize = bufferSize;

        j = 0;

        while (totalSize > 0)
        {
            if (totalSize > hwAttrs->flashType.verifyBufSize)
            {
                size = hwAttrs->flashType.verifyBufSize;
            }
            else
            {
                size = totalSize;
            }

            _srcBuf = (uint8_t *)originalBuffer;
            _dstBuf = (uint8_t *)verifyBuf;

            XMEMWFF3_read_nortos(handle, flashlogicAddr, _dstBuf, size, 0);

            for (i = 0; i < size; i++, j++)
            {
                if (_srcBuf[j] != _dstBuf[i])
                {
                                return (XMEM_STATUS_ERROR_NORTOS);
                }
            }

            flashlogicAddr = flashlogicAddr + size;
            totalSize      = totalSize - size;
        }
    }

    /* (mutex unlock removed — single-threaded BL3) */

    return (retval);
}

/*
 *  ======== checkEraseRange ========
 */
static int_fast16_t checkEraseRange(XMEM_Handle_nortos handle, size_t offset, size_t size)
{
    if (handle == NULL)
    {
        return (XMEM_STATUS_ERROR_NORTOS);
    }

    XMEMWFF3_Object_nortos const *object;

    object = handle->object;

    if (offset != (offset & sectorBaseMask))
    {
        return (XMEM_STATUS_INV_ALIGNMENT_NORTOS); /* Start address is not aligned */
    }

    if (size != (size & sectorBaseMask))
    {
        return (XMEM_STATUS_INV_SIZE_NORTOS); /* Size is not a multiple of sector size */
    }

    if (offset >= object->regionSize)
    {
        return (XMEM_STATUS_INV_OFFSET_NORTOS); /* Offset is past end of region */
    }

    if (offset + size > object->regionSize)
    {
        return (XMEM_STATUS_INV_SIZE_NORTOS); /* Size is too big */
    }

    return (XMEM_STATUS_SUCCESS_NORTOS);
}

uint32_t ConvertOffsetToAddr(XMEM_Handle_nortos handle, size_t offset, size_t AddrType)
{
    if (handle == NULL)
    {
        return 0;
    }

    XMEMWFF3_Object_nortos const *object;

    object = handle->object;

    uint32_t calcAddr;

    /* Calculate logical address */
    if (AddrType == LOGIC_ADDR)
    {
        calcAddr = object->regionStartAddr + offset;
        return (calcAddr);
    }
    else
    {
        /* Calculate physical address */
        calcAddr = object->regionBase + offset;
        return (calcAddr);
    }
}

/*
 *  ======== doErase ========
 */
static int_fast16_t doErase(XMEM_Handle_nortos handle, size_t offset, size_t size)
{
    if (handle == NULL)
    {
        return (XMEM_STATUS_ERROR_NORTOS);
    }

    XMEMWFF3_HWAttrs_nortos const *hwAttrs;
    XMEMWFF3_Object_nortos const *object;

    hwAttrs = handle->hwAttrs;
    object  = handle->object;

    uint32_t status = 0;
    int_fast16_t rangeStatus;

    /* Sanity test the erase args */
    rangeStatus = checkEraseRange(handle, offset, size);

    if (rangeStatus != XMEM_STATUS_SUCCESS_NORTOS)
    {
        return (rangeStatus);
    }

    uint32_t startAddrPhy = ConvertOffsetToAddr(handle, offset, PHYSICAL_ADDR);

    while (size)
    {
        __disable_irq();

        FlashSetTickPeriod_nortos(1); /* nortos: SYSTIM_O_TIME1U already in microseconds */
        status = (uint32_t)FlashSectorErase_nortos(startAddrPhy, object->deviceNum);

        __enable_irq();

        if (status != FLASH_STATUS_SUCCESS_NORTOS)
        {
            break;
        }

        startAddrPhy += hwAttrs->flashType.sectorSize;
        size -= hwAttrs->flashType.sectorSize;
    }

    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return (XMEM_STATUS_ERROR_NORTOS);
    }

    return (XMEM_STATUS_SUCCESS_NORTOS);
}

/*
 *  ======== flashTypeSelection ========
 */
static void flashTypeSelection(void)
{
    const FlashType_nortos IS25WJ032F_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0402220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0402220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0200,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos IS25WJ064F_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0402220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0402220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0200,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos W25Q32JW_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0802220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0802220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0400,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos W25Q64JW_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0802220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0802220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0400,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos W25Q33PW_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0602220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0602220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0300,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos W25Q64PW_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0602220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0602220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0300,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos W25Q12PW_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0802220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0802220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0300,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos GD25LF32E_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0402220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0402220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0200,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos GD25LF64E_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0402220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0402220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0200,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos PY25Q32LB_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0a02220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0a02220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0500,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos PY25Q64LB_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0a02220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0a02220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0500,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos PY25Q128LA_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0a02220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0a02220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0500,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos PY25Q256LC_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0c02220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0c02220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bbb0600,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020bb000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200b0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos MX25U3235F_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0402220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0402220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0200,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos MX25U6435F_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0402220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0402220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0200,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos MX25U12843G_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x060222eb,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x060222eb,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0xebba0300,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    const FlashType_nortos IS25WJ128F_TYPE = {

        /* Operations */
        .writeStigCfg.preStigCfg  = 1,
        .writeStigCfg.postStigCfg = 0,
        .readStigCfg.preStigCfg   = 0,
        .readStigCfg.postStigCfg  = 0,
        .eraseStigCfg.preStigCfg  = 1,
        .eraseStigCfg.postStigCfg = 0,
        .eraseStigCfg.StigCfg     = 1,

        /* Enter STIG mode */
        .enterStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .enterStigCfg[0].data    = 0x82080089,
        .enterStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .enterStigCfg[1].data    = 0x0602220b,
        .enterStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .enterStigCfg[2].data    = 0x00022002,

        /* EXIT STIG mode */
        .exitStigCfg[0].address = OSPI_REGS_BASE + OSPI_O_CONFIG,
        .exitStigCfg[0].data    = 0x82080089,
        .exitStigCfg[1].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_RD_CONFIG,
        .exitStigCfg[1].data    = 0x0602220b,
        .exitStigCfg[2].address = OSPI_REGS_BASE + OSPI_O_DEV_INSTR_WR_CONFIG,
        .exitStigCfg[2].data    = 0x00022002,

        /* Pre STIG configuration */
        .writeStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.preStigOperation[0].data    = 0x06000001,
        .eraseStigCfg.preStigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.preStigOperation[0].data    = 0x06000001,

        /* Execute STIG operation */
        .readStigCfg.stigOperation[0].address  = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .readStigCfg.stigOperation[0].data     = 0x0bba0300,
        .writeStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .writeStigCfg.stigOperation[0].data    = 0x020ab000,
        .eraseStigCfg.stigOperation[0].address = OSPI_REGS_BASE + OSPI_O_FLASH_CMD_CTRL,
        .eraseStigCfg.stigOperation[0].data    = 0x200a0000,

        /* Polling operation */
        .pollingCfg.command        = 0x05900000,
        .pollingCfg.timeOut        = 200,
        .pollingCfg.NumOfIteration = 4,

        /* General */
        .sectorSize    = 0x1000,
        .verifyBufSize = 256};

    /* Read data from shared memory */
    if (BOOT_REPORT_ADDR != 0 && BOOT_REPORT_ADDR >= 0x28000000)
    {
        uint8_t *pSharedReport      = (uint8_t *)(BOOT_REPORT_ADDR);
        uint16_t flashTypeBitMap    = *((uint16_t *)(pSharedReport + 9)); /* Flash type offset is 9 bytes */
        XMEM_Flash_Idx_nortos flashTypeIdx = (XMEM_Flash_Idx_nortos)(flashTypeBitMap & 0xff);

        switch (flashTypeIdx)
        {
            case IS25WJ032F:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = IS25WJ032F_TYPE;
                }
                break;

            case IS25WJ064F:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = IS25WJ064F_TYPE;
                }
                break;

            case W25Q32JW:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = W25Q32JW_TYPE;
                }
                break;

            case W25Q64JW:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = W25Q64JW_TYPE;
                }
                break;

            case GD25LF32E:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = GD25LF32E_TYPE;
                }
                break;

            case GD25LF64E:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = GD25LF64E_TYPE;
                }
                break;

            case PY25Q32LB:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = PY25Q32LB_TYPE;
                }
                break;

            case PY25Q64LB:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = PY25Q64LB_TYPE;
                }
                break;
            case PY25Q128LA:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = PY25Q128LA_TYPE;
                }
                break;
            case PY25Q256LC:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = PY25Q256LC_TYPE;
                }
                break;
            case MX25U3235F:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = MX25U3235F_TYPE;
                }
                break;
            case MX25U6435F:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = MX25U6435F_TYPE;
                }
                break;
            case MX25U12843G:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = MX25U12843G_TYPE;
                }
                break;
            case IS25WJ128F:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = IS25WJ128F_TYPE;
                }
                break;
            case W25Q33PW:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = W25Q33PW_TYPE;
                }
                break;
            case W25Q64PW:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = W25Q64PW_TYPE;
                }
                break;
            case W25Q12PW:
                {
                    XMEMWFF3_hwAttrs_nortos.flashType = W25Q12PW_TYPE;
                }
                break;

            default:
                /* Default to a safe configuration */
                XMEMWFF3_hwAttrs_nortos.flashType = IS25WJ032F_TYPE;
                break;
        }
    }
}
