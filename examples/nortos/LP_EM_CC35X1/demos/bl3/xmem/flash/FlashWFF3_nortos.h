/*
 *  Copyright (c) 2024-2026 Texas Instruments Incorporated
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
 *  ========== FlashWFF3.h ==========
 */

#ifndef ti_drivers_nvs_flash_FlashWFF3_nortos__include
#define ti_drivers_nvs_flash_FlashWFF3_nortos__include

#include <stdint.h>
#include <string.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/xip.h)
#include DeviceFamily_constructPath(driverlib/ospi.h)
#include DeviceFamily_constructPath(driverlib/debug.h)

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

/*
 * Values that can be returned from the API functions
 */
#define FLASH_STATUS_SUCCESS_NORTOS         0x00000000 /*!< Function completed successfully */
#define FLASH_TIMEOUT_REACHED_ERROR_NORTOS  (-1300)    /*!< timeout status error */
#define PSRAM_XIP_OFFSET_NORTOS             (0x100)    /*!< Offset for PSRAM XIP Configuration */
#define DEVICE_SWITCH_ERROR_NORTOS          (-1)
/*
 * Configuration values that should be passed to #XIPStartUDMATransaction()
 * as the directionMode parameter.
 */
/*!< UDMA Data movement boundary for write command (page program) Internal memory to External memory */
#define FLASH_WR_BOUND_SIZE_IN_BYTES_NORTOS 64U
/*!< UDMA Data movement boundary for read command External memory to Internal memory */
#define FLASH_RD_BOUND_SIZE_IN_BYTES_NORTOS 256U

/* Defines supports bit manipulation macros #BITMASK_x_0() & #BITMASK_x_y() */
#define __IF__(x)   (x)
#define __THEN__(x) ? (x)
#define __ELSE__(x)     : (x)
#define __ELSE_IF__(x)  : (x)

#define OSPI_IDLE_XSPI_COUNTER_BEFORE_TIMEOUT_NORTOS 100
/*!
 * BITMASK_x_0() returns a bit mask of "1" from Bits[x:0]
 *
 * E.g., for x=5
 *
 * BITMASK_x_0(5) = ........111111     Bits[5:0] = "111111"
 * BITMASK_x_0(0) = .............1     Bits[0:0] = "000001"
 *
 */
#define BITMASK_x_0(x)                                                                 \
    (__IF__((x) == 31)               /* For x==31, we immediately return 0xFFFFFFFF */ \
     __THEN__(0xFFFFFFFF)            /* For other x, we return 2^(x+1) - 1.         */ \
     __ELSE__((1 << ((x) + 1)) - 1)) // MACRO BITMASK_x_0() //

/*!
 * BITMASK_x_y() returns a bit mask of "1" from Bits[x:y]
 * (naturally, x >= y should be enforces by the caller!!)
 *
 * E.g., for x=5, y=2
 *
 * BITMASK_x_y(5,5) = ......1.....     Bits[5:5] = "100000"
 * BITMASK_x_y(5,2) = ......1111..     Bits[5:0] = "111100"
 * BITMASK_x_0(5,1) = ......11111.     Bits[5:1] = "111110"
 * BITMASK_x_0(5,0) = ......111111     Bits[5:0] = "111111"
 *
 */
#define BITMASK_x_y(x, y)                                                                             \
    (__IF__((y) == 0)                               /* For y==0, we return Bits[x:0]               */ \
     __THEN__(BITMASK_x_0(x))                       /* For y> 0, we return Bits[x:0] - Bits[y-1:0] */ \
     __ELSE__(BITMASK_x_0(x) - BITMASK_x_0((y)-1))) // MACRO BITMASK_x_y(). //

typedef enum
{
    FLASH_OTFDE_DISABLE_NORTOS = 0, /*!< Bit configuration to disable OTFDE control over XSPI */
    FLASH_OTFDE_ENABLE_NORTOS  = 1  /*!< Bit configuration to enable OTFDE control over XSPI */
} FlashOtfdeStateSet_nortos;

typedef enum
{
    FLASH_UDMA_JOB_DONE_NORTOS  = 0, /*!< UDMA job completed successfully */
    FLASH_UDMA_JOB_ERROR_NORTOS = 1  /*!< UDMA job did not completed successfully */
} FlashUdmaJobStatus_nortos;

/* STIG status */
typedef enum
{
    FLASH_STIG_DONE_NORTOS    = 0, /*!< STIG command completed successfully */
    FLASH_STIG_TIMEOUT_NORTOS = 1, /*!< STIG command completed with timeout */
    FLASH_STIG_ERROR_NORTOS   = 2  /*!< STIG command did not completed successfully */
} FlashStigStatus_nortos;

/* STIG configuration operation types */
#define FLASH_STIG_CONFIG_ENTER_NORTOS         0
#define FLASH_STIG_CONFIG_EXIT_NORTOS          1
#define FLASH_STIG_CONFIG_PRE_READ_NORTOS      2
#define FLASH_STIG_CONFIG_EXECUTE_READ_NORTOS  3
#define FLASH_STIG_CONFIG_POST_READ_NORTOS     4
#define FLASH_STIG_CONFIG_PRE_WRITE_NORTOS     5
#define FLASH_STIG_CONFIG_EXECUTE_WRITE_NORTOS 6
#define FLASH_STIG_CONFIG_POST_WRITE_NORTOS    7
#define FLASH_STIG_CONFIG_PRE_ERASE_NORTOS     8
#define FLASH_STIG_CONFIG_EXECUTE_ERASE_NORTOS 9
#define FLASH_STIG_CONFIG_POST_ERASE_NORTOS    10

/*
 *Functions
 */

/*!
 * @brief Read from external memory.
 *
 * This function call to #XIPStartUDMATransaction() in order to execute
 * copy transaction from external memory to internal memory.
 *
 * @param[in] readFromAddr is a pointer to the external memory source data.
 * @param[in] writeToAddr is a pointer to the internal memory destination data.
 * @param[in] length is the number of bytes to transfer.
 *
 * @note Tested with flash only (did not test with PSRAM)
 */
FlashUdmaJobStatus_nortos FlashRead_nortos(uint32_t *readFromAddr, uint32_t *writeToAddr, uint32_t length);

/*!
 * @brief Read from external memory using STIG command.
 *
 * This function call to #FlashExecuteReadSTIGCommand_nortos() in order to
 * Execute read Flash based on STIG command.
 *
 * @param[in] readFromAddr is a pointer to the external memory source data.
 * @param[in] writeToAddr is a pointer to the internal memory destination data.
 * @param[in] DeviceNum  Device identifier (XMEM_MEM_FLASH_NORTOS or XMEM_MEM_PSRAM_NORTOS).
 *
 * @return #FlashStigStatus_nortos of STIG execution command:
 * #FLASH_STIG_DONE_NORTOS    - STIG command done.
 * #FLASH_STIG_TIMEOUT_NORTOS - STIG command not completed, finish with timeout.
 * #FLASH_STIG_ERROR_NORTOS   - STIG erase command failed.
 */
FlashStigStatus_nortos FlashReadSTIG_nortos(uint32_t readStartAddr, uint32_t *writeToAddr, uint8_t DeviceNum);

/*!
 * @brief Execute read of one word from Flash based on STIG write command.
 *
 * @param[in] srcReadAddr is the data to read from external memory.
 *
 * @param[in] writeToAddr is a pointer to the internal memory destination data.
 *
 * @return #FlashStigStatus_nortos of STIG execution command:
 * #FLASH_STIG_DONE_NORTOS    - STIG command done.
 * #FLASH_STIG_TIMEOUT_NORTOS - STIG command not completed, finish with timeout.
 * #FLASH_STIG_ERROR_NORTOS   - STIG command failed.
 *
 */
FlashStigStatus_nortos FlashExecuteReadSTIGCommand_nortos(uint32_t srcReadAddr, uint32_t *writeToAddr);

/*!
 * @brief Write to external memory.
 *
 * This function call to #XIPStartUDMATransaction() in order to execute
 * copy transaction from internal memory to external memory.
 *
 * @param[in] readFromAddr is a pointer to the internal memory source data.
 * @param[in] writeToAddr is a pointer to the external memory destination data.
 * @param[in] length is the number of bytes to transfer.
 *        Maximum configurable job size - 1 Mega byte (256K Words).
 *
 * @note Tested with flash only (did not test with PSRAM)
 */
FlashUdmaJobStatus_nortos FlashWrite_nortos(uint32_t *readFromAddr, uint32_t *writeToAddr, uint32_t length);

/*!
 * @brief Write to external memory using STIG command.
 *
 * This function call to #FlashExecuteWriteSTIGCommand_nortos() in order to
 * Execute write Flash based on STIG command.
 *
 * @param[in] readFromAddr is a pointer to the internal memory source data.
 * @param[in] writeToAddr is a pointer to the external memory destination data.
 * @param[in] DeviceNum  Device identifier (XMEM_MEM_FLASH_NORTOS or XMEM_MEM_PSRAM_NORTOS).
 *
 * @return #FlashStigStatus_nortos of STIG execution command:
 * #FLASH_STIG_DONE_NORTOS    - STIG command done.
 * #FLASH_STIG_TIMEOUT_NORTOS - STIG command not completed, finish with timeout.
 * #FLASH_STIG_ERROR_NORTOS   - STIG erase command failed.
 *
 * @note Tested with flash only (did not test with PSRAM)
 */
FlashStigStatus_nortos FlashWriteSTIG_nortos(uint32_t *readFromAddr, uint32_t *writeToAddr, uint8_t DeviceNum);

/*!
 * @brief Execute write to Flash based on STIG write command.
 *
 * @param[in] srcWriteAddrPhy is the address of the external memory destination data.
 *
 * @param[in] srcWriteData is the data to write to external memory.
 *
 * @return #FlashStigStatus_nortos of STIG execution command:
 * #FLASH_STIG_DONE_NORTOS    - STIG command done.
 * #FLASH_STIG_TIMEOUT_NORTOS - STIG command not completed, finish with timeout.
 * #FLASH_STIG_ERROR_NORTOS   - STIG command failed.
 *
 */
FlashStigStatus_nortos FlashExecuteWriteSTIGCommand_nortos(uint32_t srcWriteData, uint32_t srcWriteAddrPhy);

/*!
 * @brief Enable/Disable the OTFDE.
 *
 * @param[in] setState is the required state to configure #FlashOtfdeStateSet_nortos.
 */
void FlashSetOTFDE_nortos(uint8_t setState);

/*!
 * @brief read flash status register until WIP (write in process) indication is free.
 *
 * This function can be called by nay function that execute on going process
 * such as #FlashSectorErase_nortos(), #FlashReadSTIG_nortos(), #FlashWriteSTIG_nortos().
 *
 *  If transaction will not complete before timeout reatched
 *  @return status transaction complete:
 *  #FLASH_STATUS_SUCCESS_NORTOS        - Trasaction successfuly completed.
 *  #FLASH_TIMEOUT_REACHED_ERROR_NORTOS - Timeout, transaction failed.
 */
uint32_t FlashExecutePolling_nortos(void);

/*!
 * @brief Execute erase Flash based on STIG command.
 *
 * @param[in] EraseAddr_en set in case the STIG command needs to use #srcEraseAddr.
 *
 * @param[in] srcEraseAddr from where to start the flash erase.
 *
 * @return #FlashStigStatus_nortos of STIG execution command:
 * #FLASH_STIG_DONE_NORTOS    - STIG command done.
 * #FLASH_STIG_TIMEOUT_NORTOS - STIG command not completed, finish with timeout.
 * #FLASH_STIG_ERROR_NORTOS   - STIG command failed.
 *
 */
FlashStigStatus_nortos FlashExecuteEraseSTIGCommand_nortos(uint32_t EraseAddr_en, uint32_t srcEraseAddr);

/*!
 * @brief Execute erase Flash based on STIG command.
 *
 * @param[in] eraseStartAddr from where to start the sector erase.
 *
 * @param[in] DeviceNum  Device identifier (XMEM_MEM_FLASH_NORTOS or XMEM_MEM_PSRAM_NORTOS).
 *
 * @return #FlashStigStatus_nortos of STIG execution command:
 * #FLASH_STIG_DONE_NORTOS    - STIG command done.
 * #FLASH_STIG_TIMEOUT_NORTOS - STIG command not completed, finish with timeout.
 * #FLASH_STIG_ERROR_NORTOS   - STIG command failed.
 *
 */
FlashStigStatus_nortos FlashSectorErase_nortos(uint32_t eraseStartAddr, uint8_t DeviceNum);

/*!
 * @brief Store the ClockP period internally.
 *
 * @param TickPeriod Periodic interval in ClockP ticks
 *
 */
void FlashSetTickPeriod_nortos(uint32_t TickPeriod);

/*!
 * @brief xSPI (accessible) idle check before STIG execution.
 */
uint32_t FlashIsXspiIdle_nortos();
/*!
 * Mark the end of the C bindings section for C++ compilers.
 */
#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_nvs_flash_FlashWFF3_nortos__include */
