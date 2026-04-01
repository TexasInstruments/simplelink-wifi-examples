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
// Standard includes
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <osi_kernel.h>
#include <uart_npi.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/UART2.h>
#include <ti/drivers/uart2/UART2WFF3.h>


//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
int uartNpiRead(uint8_t* buf, uint16_t len);
int uartNpiWrite(uint8_t* buf, uint16_t len);

const hciTransport_t uartForNpi =
{
    &uartNpiRead,
    &uartNpiWrite
};

//*****************************************************************************
//                          LOCAL VARIABLES
//*****************************************************************************
UART2_Handle uartNpiHandle = 0;
extern const UART2_Config UART2_config[];


//*****************************************************************************
//                          LOCAL FUNCTIONS
//*****************************************************************************


//*****************************************************************************
//                          GLOBAL VARIABLES
//*****************************************************************************


//*****************************************************************************
//                          API FUNCTIONS
//*****************************************************************************
//*****************************************************************************
//
//! UartNpiOpen
//!
//! This function opens the UART2_1 to be used.
//! Note the UART2_2 is configured in the SySconfig
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
void UartNpiOpen(void)
{
    UART2_Params params;
    UART2WFF3_HWAttrs* config = (UART2WFF3_HWAttrs*)(UART2_config[CONFIG_UART2_1].hwAttrs);
    UART2_Params_init(&params);
    params.baudRate = 230400;
    Report("\n\rNPI UART Baudrate :: %d\n\r",params.baudRate);
    Report("\n\rNPI UART Flow Control :: %d\n\r",config->flowControl);
    uartNpiHandle = UART2_open(CONFIG_UART2_1, &params);
}

//*****************************************************************************
//
//! UartNpiClose
//!
//! This function closes the UART2_1.
//! Note the UART2_2 is configured in the SySconfig
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
void UartNpiClose(void)
{
    UART2_readCancel(uartNpiHandle);
    UART2_writeCancel(uartNpiHandle);
    UART2_close(uartNpiHandle);
}

//*****************************************************************************
//
//! uartNpiRead
//!
//! This function reads to a buffer with a given length from UART2_1.
//!
//! \param  buf - buffer to read
//!         len - len to read
//!
//! \return none
//
//*****************************************************************************
int uartNpiRead(uint8_t* buf, uint16_t len)
{
    size_t bytesRead;
    return UART2_read(uartNpiHandle, buf, len, &bytesRead);
}

//*****************************************************************************
//
//! uartNpiWrite
//!
//! This function writes buffer with a given length to UART2_1.
//!
//! \param  buf - buffer to write
//!         len - len to write
//!
//! \return none
//
//*****************************************************************************
int uartNpiWrite(uint8_t* buf, uint16_t len)
{
    size_t bytesWritten;
    return UART2_write(uartNpiHandle, buf, len, &bytesWritten);
}
