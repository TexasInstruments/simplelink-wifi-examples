/*
 *  Copyright (c) 2024-2025 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1) Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2) Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3) Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * ========== FlashWFF3.c ==========
 */

#include "assert.h"
#include "xmem/flash/FlashWFF3_nortos.h"
#include <ti/devices/DeviceFamily.h>
#include "xmem/XMEMWFF3_nortos.h"
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_systim.h)

/*
 * Globals
 */
static uint32_t flashClockPTickPeriod = 1U;

static uint32_t __attribute__((section(".TI.ramfunc"))) getClockPTickUs(void);
static bool __attribute__((section(".TI.ramfunc")))
FlashClockPCheckTimeout(uint32_t StartTick, uint32_t TimerInMicroSec);
static FlashStigStatus_nortos __attribute__((section(".TI.ramfunc"))) FlashStigConfigOp(uint8_t configOp);
static FlashStigStatus_nortos __attribute__((section(".TI.ramfunc"))) FlashStigPreConfigOp(uint8_t DeviceNum);
static FlashStigStatus_nortos __attribute__((section(".TI.ramfunc"))) FlashStigPostConfigOp(uint8_t DeviceNum);
static int __attribute__((section(".TI.ramfunc"))) FlashDeviceSwitchConfigOp(uint8_t DeviceNum, bool bEnter);
static uint32_t FlashGetBound(uint8_t directionMode);
static FlashUdmaJobStatus_nortos FlashUDMAOperationStatus(uint32_t channelSelect);
static FlashUdmaJobStatus_nortos FlashPartitioningUDMATransaction(uint32_t *srcAddr,
                                             uint32_t *dstAddr,
                                             uint32_t lengthBytes,
                                             uint8_t directionMode,
                                             uint32_t channelSelect);

extern XMEMWFF3_HWAttrs_nortos XMEMWFF3_hwAttrs_nortos;

uint32_t __attribute__((section(".TI.ramfunc"))) FlashIsXspiIdle_nortos()
{
    uint32_t idleXSPICoutnerBefore = 0;

    /* xSPI (accessible) idle check before STIG execution */
    while (OSPI_CONFIG_IDLE_DISABLE == (HWREG(OSPI_REGS_BASE + OSPI_O_CONFIG) & OSPI_CONFIG_IDLE_M) &&
                                        (idleXSPICoutnerBefore < OSPI_IDLE_XSPI_COUNTER_BEFORE_TIMEOUT_NORTOS))
    {
        idleXSPICoutnerBefore++;
        if (idleXSPICoutnerBefore > OSPI_IDLE_XSPI_COUNTER_BEFORE_TIMEOUT_NORTOS)
        {
            return FLASH_TIMEOUT_REACHED_ERROR_NORTOS;
        }
    }

    return FLASH_STATUS_SUCCESS_NORTOS;
}

FlashUdmaJobStatus_nortos FlashRead_nortos(uint32_t *readFromAddr, uint32_t *writeToAddr, uint32_t length)
{
    return FlashPartitioningUDMATransaction(readFromAddr,
                                            writeToAddr,
                                            length,
                                            XIP_UDMA_DIRECTION_EXT_TO_INT,
                                            XIP_UDMA_SECURE_CHANNEL);
}

FlashUdmaJobStatus_nortos FlashWrite_nortos(uint32_t *readFromAddr, uint32_t *writeToAddr, uint32_t length)
{
    return FlashPartitioningUDMATransaction(readFromAddr,
                                            writeToAddr,
                                            length,
                                            XIP_UDMA_DIRECTION_INT_TO_EXT,
                                            XIP_UDMA_SECURE_CHANNEL);
}

void __attribute__((section(".TI.ramfunc"))) FlashSetOTFDE_nortos(uint8_t setState)
{
    /* SDK assert here is inverted (fires on valid values).  Remove until SDK is fixed. */

    if (setState == FLASH_OTFDE_DISABLE_NORTOS)
    {
        XIPDisableOTFDE();
    }
    else if (setState == FLASH_OTFDE_ENABLE_NORTOS)
    {
        XIPEnableOTFDE();
    }
}

/*
 * ======== FlashExecutePolling_nortos ========
 */
uint32_t __attribute__((section(".TI.ramfunc"))) FlashExecutePolling_nortos(void)
{
#if 0
    OSPISTIGConfig_t XspiStigConifgReg = {0};
    XspiStigConifgReg.stigOpcode       = OSPI_COMMAND_RDSR;
    XspiStigConifgReg.stigRdEn         = OSPI_STIG_READ_DATA_ENABLE >> OSPI_FLASH_CMD_CTRL_ENB_READ_DATA_S;
    XspiStigConifgReg.stigRDNumBytes   = 1;
#endif    
    uint32_t startTime                 = (uint32_t)getClockPTickUs();
    uint32_t timerValMicroSec          = XMEMWFF3_hwAttrs_nortos.flashType.pollingCfg.timeOut * 1000;
    int pollingCount                   = 0;

    /* Check the maximum time in system ticks (microsecond) for the transaction
     to complete and Wait for command completion by polling WIP bit from flash status register */
    if (pollingCount < XMEMWFF3_hwAttrs_nortos.flashType.pollingCfg.NumOfIteration)
    {
        do
        {
            if (XMEMWFF3_hwAttrs_nortos.flashType.pollingCfg.timeOut != 0)
            {
                if (FlashClockPCheckTimeout(startTime, timerValMicroSec) != 1)
                {
                    return FLASH_TIMEOUT_REACHED_ERROR_NORTOS;
                }
            }

            /* Send Polling Command */
            OSPIStartSTIGCommand(XMEMWFF3_hwAttrs_nortos.flashType.pollingCfg.command);

            /* xSPI (accessible) idle check before STIG execution */
            if (FlashIsXspiIdle_nortos() != FLASH_STATUS_SUCCESS_NORTOS)
            {
                return FLASH_TIMEOUT_REACHED_ERROR_NORTOS;
            }

            /* Read Status Register */
            if (false == (OSPIGetSTIGDataRegister(OSPI_STIG_READ_DATA_LOWER) & OSPI_FLASH_STA_REG_WIP))
            {
                pollingCount++;
            }
            else
            {
                pollingCount = 0;
            }

        } while (pollingCount < XMEMWFF3_hwAttrs_nortos.flashType.pollingCfg.NumOfIteration);
    }

    return FLASH_STATUS_SUCCESS_NORTOS;
}

/*
 * ======== FlashExecuteReadSTIGCommand_nortos ========
 */
FlashStigStatus_nortos __attribute__((section(".TI.ramfunc")))
FlashExecuteReadSTIGCommand_nortos(uint32_t srcReadAddr, uint32_t *writeToAddr)
{
    FlashStigStatus_nortos status;

    /* Configure STIG address STIG execution */
    OSPISetCommandAddress(srcReadAddr);

    /* Enter STIG mode */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_ENTER_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Pre STIG configuration */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_PRE_READ_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Execute STIG operation */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_EXECUTE_READ_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Processing read data */
    *writeToAddr = OSPIGetSTIGDataRegister(OSPI_STIG_READ_DATA_LOWER);

    /* Post STIG configuration */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_POST_READ_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Exit STIG mode */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_EXIT_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    return FLASH_STIG_DONE_NORTOS;
}

/*
 * ======== FlashReadSTIG_nortos ========
 */
FlashStigStatus_nortos __attribute__((section(".TI.ramfunc")))
FlashReadSTIG_nortos(uint32_t readStartAddr, uint32_t *writeToAddr, uint8_t DeviceNum)
{
    FlashStigStatus_nortos status;

    /* Prepare system for STIG operations */
    status = FlashStigPreConfigOp(DeviceNum);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    /* Execute the read command */
    status = FlashExecuteReadSTIGCommand_nortos(readStartAddr, writeToAddr);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Restore system after STIG operations */
    status = FlashStigPostConfigOp(DeviceNum);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    return status;
}

/*
 * ======== FlashExecuteEraseSTIGCommand_nortos ========
 */
FlashStigStatus_nortos __attribute__((section(".TI.ramfunc")))
FlashExecuteEraseSTIGCommand_nortos(uint32_t EraseAddr_en, uint32_t srcEraseAddr)
{
    FlashStigStatus_nortos status;

    /* Set start address to erase */
    if (EraseAddr_en == 1)
    {
        OSPISetCommandAddress(srcEraseAddr);
    }

    /* Enter STIG mode */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_ENTER_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Pre STIG configuration */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_PRE_ERASE_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Execute STIG operation */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_EXECUTE_ERASE_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Post STIG configuration */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_POST_ERASE_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Exit STIG mode */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_EXIT_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    return FLASH_STIG_DONE_NORTOS;
}

/*
 * ======== FlashSectorErase_nortos ========
 */
FlashStigStatus_nortos __attribute__((section(".TI.ramfunc"))) FlashSectorErase_nortos(uint32_t eraseStartAddr, uint8_t DeviceNum)
{
    FlashStigStatus_nortos status;

    /* Prepare system for STIG operations */
    status = FlashStigPreConfigOp(DeviceNum);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    /* Execute the write command */
    status = FlashExecuteEraseSTIGCommand_nortos(1, eraseStartAddr);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Restore system after STIG operations */
    status = FlashStigPostConfigOp(DeviceNum);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    return status;
}

/*
 * ======== FlashExecuteWriteSTIGCommandExtend ========
 */
FlashStigStatus_nortos __attribute__((section(".TI.ramfunc")))
FlashExecuteWriteSTIGCommand_nortos(uint32_t srcWriteData, uint32_t srcWriteAddrPhy)
{
    uint32_t status;

    /* configure STIG address and STIG Write data before Write STIG execution */
    OSPISetCommandAddress(srcWriteAddrPhy);
    HWREG(OSPI_REGS_BASE + OSPI_O_FLASH_WR_DATA_LOWER) = srcWriteData;

    /* Enter STIG mode */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_ENTER_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Pre STIG configuration */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_PRE_WRITE_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Execute STIG operation */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_EXECUTE_WRITE_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Post STIG configuration */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_POST_WRITE_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Exit STIG mode */
    status = FlashStigConfigOp(FLASH_STIG_CONFIG_EXIT_NORTOS);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    return FLASH_STIG_DONE_NORTOS;
}

/*
 * ======== FlashWriteSTIG_nortos ========
 */
FlashStigStatus_nortos __attribute__((section(".TI.ramfunc")))
FlashWriteSTIG_nortos(uint32_t *readFromAddr, uint32_t *writeToAddr, uint8_t DeviceNum)
{
    FlashStigStatus_nortos status;

    /* Prepare system for STIG operations */
    status = FlashStigPreConfigOp(DeviceNum);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    /* Execute the write command */
    status = FlashExecuteWriteSTIGCommand_nortos((uint32_t)*readFromAddr, (uint32_t)writeToAddr);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return status;
    }

    /* Restore system after STIG operations */
    status = FlashStigPostConfigOp(DeviceNum);
    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    return status;
}

/*
 * ======== FlashSetTickPeriod_nortos ========
 */
void FlashSetTickPeriod_nortos(uint32_t TickPeriod)
{
    flashClockPTickPeriod = TickPeriod;
}

/*
 *  ======== FlashClockPCheckTimeout ========
 *
 * @brief Return indication if configurable timeout excedded.
 *
 * @param[in] StartTick The start Time from which we count #TimerInMicroSec Ticks in micro seconds,
 * if exceeds stop counting #getClockPTickUs.
 *
 * @param[in] TimerInMicroSec Configured time in micro seconds,
 * if exceeds stop counting #getClockPTickUs.
 *
 * @return bool
 * false - Timeout occur
 * true  - Timeout not occur
 *
 */
static bool __attribute__((section(".TI.ramfunc")))
FlashClockPCheckTimeout(uint32_t StartTick, uint32_t TimerInMicroSec)
{
    uint32_t currTick = getClockPTickUs();

    if ((currTick - StartTick) > TimerInMicroSec)
    {
        return false;
    }

    return true;
}

/*
 *  ======== getClockPTickUs ========
 *
 * @brief Return the current ClockP tick value.
 *
 * @return The current ClockP tick value.
 *
 * @note the global variable #flashClockPTickPeriod is set by FlashSetTickPeriod_nortos().
 *
 */
static uint32_t __attribute__((section(".TI.ramfunc"))) getClockPTickUs(void)
{
    return (HWREG(SYSTIM_BASE + SYSTIM_O_TIME1U) / flashClockPTickPeriod);
}

/*
 *  ======== FlashGetBound ========
 *
 * @brief Return a bound to obtain an efficient read/write commands through UDMA.
 *
 * @param[in] directionMode XIP_UDMA_DIRECTION_INT_TO_EXT or XIP_UDMA_DIRECTION_EXT_TO_INT
 *                          INT: internal RAM
 *                          EXT: external memory
 *
 * @return Bound value in bytes.
 *
 */
static uint32_t FlashGetBound(uint8_t directionMode)
{
    /* Set resolution, in bytes */
    if (directionMode == XIP_UDMA_DIRECTION_INT_TO_EXT)
    {
        return FLASH_WR_BOUND_SIZE_IN_BYTES_NORTOS;
    }
    else
    {
        return FLASH_RD_BOUND_SIZE_IN_BYTES_NORTOS;
    }
}

/*
 *  ======== FlashUDMAOperationStatus ========
 *
 * @brief return UDMA job status.
 *
 * @param[in] channelSelect check the status of one of the two DMA channels
 * - \ref XIP_UDMA_SECURE_CHANNEL
 * - \ref XIP_UDMA_NON_SECURE_CHANNEL
 *
 * @return status FLASH_UDMA_JOB_DONE_NORTOS or FLASH_UDMA_JOB_ERROR_NORTOS
 *         after checking if UDMA job was completed.
 */
static FlashUdmaJobStatus_nortos FlashUDMAOperationStatus(uint32_t channelSelect)
{
    uint32_t jobStatus;

    while (XIPGetUDMAChannelProgressingStatus(channelSelect) == XIP_UDMA_CHANNEL_STATUS_PROGRESS) {};

    while (XIPGetUDMAChannelWordsLeft(channelSelect) != 0) {};

    jobStatus = XIPGetUDMAIrqStatus(channelSelect);
    if (jobStatus == XIP_UDMA_JOB_IRQ_STATUS_DONE)
    {
        return FLASH_UDMA_JOB_DONE_NORTOS;
    }
    else
    {
        return FLASH_UDMA_JOB_ERROR_NORTOS;
    }
}

/*
 *  ======== FlashPartitioningUDMATransaction ========
 *
 * @brief drive UDMA transaction according to write/read jobs restrictions.
 *
 * @param[in] srcAddr the start address from where to read the data
 *
 * @param[in] dstAddr the start address where to place the data
 *
 * @param[in] length number of bytes to transfer
 *
 * @param[in] directionMode XIP_UDMA_DIRECTION_INT_TO_EXT or XIP_UDMA_DIRECTION_EXT_TO_INT
 *                          INT: internal RAM
 *                          EXT: external memory
 *
 * @param[in] channelSelect one of the two DMA channels (SECURED & NON-SECURED)
 * - \ref XIP_UDMA_SECURE_CHANNEL
 * - \ref XIP_UDMA_NON_SECURE_CHANNEL
 *
 * @note use only SECURED channel see Jira: LPRFXXWARE-979
 */
static FlashUdmaJobStatus_nortos FlashPartitioningUDMATransaction(uint32_t *srcAddr,
                                             uint32_t *dstAddr,
                                             uint32_t lengthBytes,
                                             uint8_t directionMode,
                                             uint32_t channelSelect)
{
    uint32_t JobStatus;
    uint32_t bound;
    uint32_t startAddr;

    if ((uint8_t)directionMode == XIP_UDMA_DIRECTION_EXT_TO_INT)
    {
        startAddr = (uint32_t)srcAddr;
    }
    else /* XIP_UDMA_DIRECTION_INT_TO_EXT */
    {
        startAddr = (uint32_t)dstAddr;
    };

    bound = FlashGetBound(directionMode); /* Bound should not equal '0'*/

    /* Calculate srcAddr offset from bound [Bytes]
     * assume bound is power of two
     */

    uint32_t sourceOffsetToGrid = startAddr & (bound - 1);

    /* AlignedJobLength in Bytes */
    uint32_t alignedJobLength = bound - sourceOffsetToGrid;

    /* Check if source address is not aligned to bound
     * and check the size to be copy is actually cross the bound
     */
    if (sourceOffsetToGrid != 0 && lengthBytes > alignedJobLength)
    {
        /* Align transaction - transact the first bytes between the source address and the followed bound
         * in order to align the original srcAddr to the bound.
         */
        JobStatus = XIPStartUDMATransaction(srcAddr, dstAddr, alignedJobLength / 4, directionMode, channelSelect);

        /* Return an error if UDMA is busy and is unable to start a new job */
        if (JobStatus != FLASH_STATUS_SUCCESS_NORTOS)
        {
            /* Return in case of an error.
             * The error check will be in the callback function.
             */
            return FLASH_UDMA_JOB_ERROR_NORTOS;
        }

        if (FlashUDMAOperationStatus(channelSelect) == FLASH_UDMA_JOB_DONE_NORTOS)
        {
            /* Update parameters */
            srcAddr = (uint32_t *)((uint32_t)srcAddr + alignedJobLength);
            dstAddr = (uint32_t *)((uint32_t)dstAddr + alignedJobLength);
            lengthBytes -= alignedJobLength;
        }
        else
        {
            /* Return in case of an error.
             * The error check will be in the callback function.
             */
            return FLASH_UDMA_JOB_ERROR_NORTOS;
        }
    }

    /* Transact the rest of lengthBytes (when previous align procedure has been executed)
     * or directly when original srcAddr aligned to bound.
     */
    while (lengthBytes > 0)
    {
        /* Condition for the optional last remaining bytes (when lengthBytes is not aligned to bound) */
        uint32_t transSizeBytes;
        if (lengthBytes > bound)
        {
            transSizeBytes = bound;
        }
        else
        {
            transSizeBytes = lengthBytes;
        }

        JobStatus = XIPStartUDMATransaction(srcAddr, dstAddr, transSizeBytes / 4, directionMode, channelSelect);

        /* Return an error if UDMA is busy and is unable to start a new job */
        if (JobStatus != FLASH_STATUS_SUCCESS_NORTOS)
        {
            /* Return in case of an error.
             * The error check will be in the callback function.
             */
            return FLASH_UDMA_JOB_ERROR_NORTOS;
        }

        if (FlashUDMAOperationStatus(channelSelect) == FLASH_UDMA_JOB_DONE_NORTOS)
        {
            /* Update parameters */
            srcAddr = (uint32_t *)((uint32_t)srcAddr + transSizeBytes);
            dstAddr = (uint32_t *)((uint32_t)dstAddr + transSizeBytes);
            lengthBytes -= transSizeBytes;
        }
        else
        {
            /* Return in case of an error.
             * The error check will be in the callback function
             */
            return FLASH_UDMA_JOB_ERROR_NORTOS;
        }
    }
    return FLASH_UDMA_JOB_DONE_NORTOS;
}

/**
 * @brief Generic function to handle all STIG configuration operations
 *
 * @param configOp One of the FLASH_STIG_CONFIG_* operation types
 * @return FlashStigStatus_nortos Status of the operation
 */
static FlashStigStatus_nortos __attribute__((section(".TI.ramfunc"))) FlashStigConfigOp(uint8_t configOp)
{
    FlashStigStatus_nortos status;
    int i;
    uint32_t configEnabled         = 0;
    FlashRegister_nortos *configOperation = NULL;
    size_t operationSize           = 0;
    bool isPollingStage            = false;

    /* Select the appropriate configuration based on operation type */
    switch (configOp)
    {
        case FLASH_STIG_CONFIG_ENTER_NORTOS:
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.enterStigCfg;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.enterStigCfg);
            configEnabled   = 1; /* Always enabled */
            break;

        case FLASH_STIG_CONFIG_EXIT_NORTOS:
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.exitStigCfg;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.exitStigCfg);
            configEnabled   = 1; /* Always enabled */
            break;

        case FLASH_STIG_CONFIG_PRE_READ_NORTOS:
            configEnabled   = XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.preStigCfg;
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.preStigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.preStigOperation);
            break;

        case FLASH_STIG_CONFIG_EXECUTE_READ_NORTOS:
            configEnabled   = 1; /* Assuming always enabled for the main operation */
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.stigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.stigOperation);
            isPollingStage  = true;
            break;

        case FLASH_STIG_CONFIG_POST_READ_NORTOS:
            configEnabled   = XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.postStigCfg;
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.postStigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.readStigCfg.postStigOperation);
            break;

        case FLASH_STIG_CONFIG_PRE_WRITE_NORTOS:
            configEnabled   = XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.preStigCfg;
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.preStigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.preStigOperation);
            break;

        case FLASH_STIG_CONFIG_EXECUTE_WRITE_NORTOS:
            configEnabled   = 1; /* Assuming always enabled for the main operation */
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.stigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.stigOperation);
            isPollingStage  = true;
            break;

        case FLASH_STIG_CONFIG_POST_WRITE_NORTOS:
            configEnabled   = XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.postStigCfg;
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.postStigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.writeStigCfg.postStigOperation);
            break;

        case FLASH_STIG_CONFIG_PRE_ERASE_NORTOS:
            configEnabled   = XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.preStigCfg;
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.preStigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.preStigOperation);
            break;

        case FLASH_STIG_CONFIG_EXECUTE_ERASE_NORTOS:
            configEnabled   = XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.StigCfg;
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.stigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.stigOperation);
            isPollingStage  = true;
            break;

        case FLASH_STIG_CONFIG_POST_ERASE_NORTOS:
            configEnabled   = XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.postStigCfg;
            configOperation = XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.postStigOperation;
            operationSize   = sizeof(XMEMWFF3_hwAttrs_nortos.flashType.eraseStigCfg.postStigOperation);
            break;

        default:
            return FLASH_STIG_ERROR_NORTOS;
    }

    /* Check if configuration is needed */
    if (configEnabled)
    {
        for (i = 0; i < operationSize / sizeof(FlashRegister_nortos); i++)
        {
            if (isPollingStage && (configOperation[i].address & OSPI_O_FLASH_CMD_CTRL))
            {
                /* For Polling stage, we need to poll before command */
                status = FlashExecutePolling_nortos();
                if (status != FLASH_STATUS_SUCCESS_NORTOS)
                {
                    return FLASH_STIG_TIMEOUT_NORTOS;
                }

                OSPIStartSTIGCommand(configOperation[i].data);

                /* For Polling stage, we need to poll after command only for
                 *  write/erase and not for read
                 */
                if (configOp != FLASH_STIG_CONFIG_EXECUTE_READ_NORTOS)
                {
                    status = FlashExecutePolling_nortos();
                    if (status != FLASH_STATUS_SUCCESS_NORTOS)
                    {
                        return FLASH_STIG_TIMEOUT_NORTOS;
                    }
                }
            }
            else if (configOperation[i].address & OSPI_O_FLASH_CMD_CTRL)
            {
                /* xSPI (accessible) idle check before STIG execution */
                status = FlashIsXspiIdle_nortos();
                if (status != FLASH_STATUS_SUCCESS_NORTOS)
                {
                    return FLASH_STIG_ERROR_NORTOS;
                }

                /* For other stages with STIG commands */
                OSPIStartSTIGCommand(configOperation[i].data);
            }
            else
            {
                /* xSPI (accessible) idle check before STIG execution */
                status = FlashIsXspiIdle_nortos();
                if (status != FLASH_STATUS_SUCCESS_NORTOS)
                {
                    return FLASH_STIG_ERROR_NORTOS;
                }

                /* Configuration of unprotected registers */
                HWREG(configOperation[i].address) = configOperation[i].data;
            }
        }
    }

    return FLASH_STATUS_SUCCESS_NORTOS;
}

/*!
 *  @brief  Pre-configuration for STIG operations
 *
 *  This function prepares the flash memory system for STIG (Serial Transfer Interface Gateway) operations.
 *  It performs two critical initialization steps:
 *  1. Disables OTFDE (On-The-Fly Decryption/Encryption) to allow direct STIG command execution
 *  2. Verifies that the xSPI interface is in an idle state and ready to accept commands
 *
 *  @param[in] DeviceNum  Device identifier (XMEM_FLASH_NORTOS or XMEM_PSRAM_NORTOS)
 *
 *  @return FlashStigStatus_nortos FLASH_STATUS_SUCCESS_NORTOS if successful, FLASH_STIG_ERROR_NORTOS if xSPI is not idle
 */
static FlashStigStatus_nortos __attribute__((section(".TI.ramfunc"))) FlashStigPreConfigOp(uint8_t DeviceNum)
{
    FlashStigStatus_nortos status;

    /* OTFDE Disable in order to use STIG command */
    FlashSetOTFDE_nortos(FLASH_OTFDE_DISABLE_NORTOS);

    /* Switch device in order to perform STIG command */
    status = FlashDeviceSwitchConfigOp(DeviceNum, true);

    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    /* xSPI (accessible) idle check before STIG execution */
    status = FlashIsXspiIdle_nortos();

    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    return FLASH_STATUS_SUCCESS_NORTOS;
}

/*!
 *  @brief  Post-configuration cleanup after STIG operations
 *
 *  This function performs cleanup operations after a STIG command has been executed.
 *  Its primary purpose is to re-enable the OTFDE (On-The-Fly Decryption/Encryption)
 *  functionality that was disabled during the STIG operation.
 *
 *  @param[in] DeviceNum  Device identifier (XMEM_FLASH_NORTOS or XMEM_PSRAM_NORTOS)
 *
 *  @return FlashStigStatus_nortos FLASH_STATUS_SUCCESS_NORTOS indicating successful completion
 */
static FlashStigStatus_nortos __attribute__((section(".TI.ramfunc"))) FlashStigPostConfigOp(uint8_t DeviceNum)
{
    FlashStigStatus_nortos status;

    /* Switch device to current active device */
    status = FlashDeviceSwitchConfigOp(DeviceNum, false);

    if (status != FLASH_STATUS_SUCCESS_NORTOS)
    {
        return FLASH_STIG_ERROR_NORTOS;
    }

    /* OTFDE Enable */
    FlashSetOTFDE_nortos(FLASH_OTFDE_ENABLE_NORTOS);

    return FLASH_STATUS_SUCCESS_NORTOS;
}

/*!
 * @brief Configure OSPI interface by reading configuration from XIP (OTFDE) registers.
 *
 * This function configures the OSPI (Octal SPI) peripheral for a specific device
 * (Flash or PSRAM) by reading configuration data from XIP (Execute-In-Place) registers.
 * The function supports both entering and exiting device switch mode.
 *
 * XIP Register Layout:
 * --------------------
 * The configuration data is stored in [data, address] pairs starting at the base address:
 *
 * - HOST_XIP_REGS_BASE + 0x0:  First OSPI configuration data value
 * - HOST_XIP_REGS_BASE + 0x4:  First address control register
 *                              Bits [5:0]: Offset value (must be shifted left by 2 to get byte offset)
 *                              Bit  [6]:   Stop bit (1 = last configuration pair)
 *                              Bits [7]:   Reserved
 *
 * - HOST_XIP_REGS_BASE + 0x8:  Second OSPI configuration data value
 * - HOST_XIP_REGS_BASE + 0xC:  Second address control register (same format as above)
 *
 * This pattern repeats every 8 bytes until the stop bit is encountered.
 *
 * Address Calculation:
 * --------------------
 * OSPI register address = OSPI_REGS_BASE + ((offset_bits[5:0]) << 2)
 *
 * Device Selection:
 * -----------------
 * - XMEM_FLASH_NORTOS: XIP base = HOST_XIP_REGS_BASE + HOST_XIP_O_OTOSMEM
 * - XMEM_PSRAM_NORTOS: XIP base = HOST_XIP_REGS_BASE + HOST_XIP_O_OTOSMEM + PSRAM_XIP_OFFSET_NORTOS
 *
 * Operation Modes:
 * ----------------
 * - ENTER (true):  Configure OSPI for DeviceNum
 *                  Tracks what device OSPI is configured for
 *                  Allows nested enters - always ensures correct configuration
 * - EXIT (false):  Configure OSPI for current DeviceIsActive (read from HW)
 *                  Can be called without prior ENTER
 *
 * State Tracking:
 * ---------------
 * The function tracks the current OSPI configuration state internally:
 * - 0xFF = uninitialized (first call)
 * - 0x00 = OSPI configured for XMEM_FLASH_NORTOS
 * - 0x01 = OSPI configured for XMEM_PSRAM_NORTOS
 *
 * This allows the function to skip redundant configurations and handle
 * nested enters gracefully.
 *
 * @param[in] DeviceNum  Device identifier (XMEM_FLASH_NORTOS or XMEM_PSRAM_NORTOS)
 *                       Used only in ENTER mode, ignored in EXIT mode
 * @param[in] bEnter     Operation mode:
 *                       - true:  Enter device switch (configure TO DeviceNum)
 *                       - false: Exit device switch (configure TO current DeviceIsActive from HW)
 *
 * @return int
 *         - FLASH_STATUS_SUCCESS_NORTOS: Configuration completed successfully or no action needed
 *         - DEVICE_SWITCH_ERROR_NORTOS:  Invalid device number or OSPI not idle
 *
 * @note DeviceIsActive is ALWAYS read from hardware OTFDE status register
 * @note Multiple consecutive ENTERs are allowed - function handles them gracefully
 * @note EXIT can be called without prior ENTER
 * @note Function tracks actual OSPI configuration state internally
 *
 *
 * @example
 * // Flexible usage - nested enters allowed
 * FlashDeviceSwitchConfigOp(XMEM_FLASH_NORTOS, true);   // ENTER Flash
 * // ... Flash operations ...
 * FlashDeviceSwitchConfigOp(XMEM_PSRAM_NORTOS, true);   // ENTER PSRAM (allowed, reconfigures)
 * // ... PSRAM operations ...
 * FlashDeviceSwitchConfigOp(XMEM_PSRAM_NORTOS, false);  // EXIT (configure to DeviceIsActive)
 */
static int __attribute__((section(".TI.ramfunc"))) FlashDeviceSwitchConfigOp(uint8_t DeviceNum, bool bEnter)
{
    static uint8_t CurrentOSPIConfig = 0xFF; // Track actual OSPI configuration (0xFF = uninitialized)
    uint8_t DeviceIsActive;
    uint8_t TargetDevice;
    uint8_t StopBit = 0;
    uint8_t status;
    uint32_t DeviceXIPBaseAddr;
    uint32_t DeviceXIPAddrOffset = 0;
    uint32_t DeviceOSPICfgAddrOffset;
    FlashRegister_nortos Register;

    /* Always read current device state from hardware OTFDE status register */
    DeviceIsActive = (HWREG(HOST_XIP_REGS_BASE + HOST_XIP_O_OTSTA) & HOST_XIP_OTSTA_TASKDEVICE) >>
                      HOST_XIP_OTSTA_TASKDEVICE_S;

    /* Determine target device based on enter/exit mode */
    if (bEnter)
    {
        /* ENTER mode: configure OSPI for DeviceNum */
        TargetDevice = DeviceNum;

        /* If OSPI is already configured for this device, do nothing */
        if (CurrentOSPIConfig == DeviceNum)
        {
            return FLASH_STATUS_SUCCESS_NORTOS;
        }
    }
    else
    {
        /* EXIT mode: configure OSPI for current DeviceIsActive from HW (ignore DeviceNum) */
        TargetDevice = DeviceIsActive;

        /* If OSPI is already configured for DeviceIsActive, do nothing */
        if (CurrentOSPIConfig == DeviceIsActive)
        {
            return FLASH_STATUS_SUCCESS_NORTOS;
        }
    }

    /* Validate target device */
    if (TargetDevice != XMEM_FLASH_NORTOS && TargetDevice != XMEM_PSRAM_NORTOS)
    {
        /* Invalid device number */
        return DEVICE_SWITCH_ERROR_NORTOS;
    }

    /* Determine XIP base address based on target device type */
    if (XMEM_FLASH_NORTOS == TargetDevice)
    {
        DeviceXIPBaseAddr = HOST_XIP_REGS_BASE + HOST_XIP_O_OTOSMEM;
    }
    else /* XMEM_PSRAM_NORTOS */
    {
        DeviceXIPBaseAddr = HOST_XIP_REGS_BASE + HOST_XIP_O_OTOSMEM + PSRAM_XIP_OFFSET_NORTOS;
    }

    /* Iterate through configuration pairs until stop bit is set */
    while (!StopBit)
    {
        /* Read 32-bit configuration data from XIP register */
        Register.data = HWREG(DeviceXIPBaseAddr + DeviceXIPAddrOffset);

        /* Read address control register */
        uint32_t addrControl = HWREG(DeviceXIPBaseAddr + DeviceXIPAddrOffset + 0x4);

        /* Extract offset value (bits [5:0]) and convert to byte offset */
        DeviceOSPICfgAddrOffset = (addrControl & 0x3F) << 2;
        Register.address        = OSPI_REGS_BASE + DeviceOSPICfgAddrOffset;

        /* Extract stop bit (bit [6]) */
        StopBit = (addrControl & 0x40) >> 6;

        /* xSPI (accessible) idle check before STIG execution */
        status = FlashIsXspiIdle_nortos();
        if (status != FLASH_STATUS_SUCCESS_NORTOS)
        {
            return DEVICE_SWITCH_ERROR_NORTOS;
        }

        /* Write configuration data to target OSPI register */
        HWREG(Register.address) = Register.data;

        /* Move to next configuration pair (8 bytes forward) */
        DeviceXIPAddrOffset += 0x8;
    }

    /* Update tracked OSPI configuration state after successful switch */
    CurrentOSPIConfig = TargetDevice;

    return FLASH_STATUS_SUCCESS_NORTOS;
}
