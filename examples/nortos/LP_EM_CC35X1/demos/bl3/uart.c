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
 *    its contributors may be used to endorse or reproduce products derived
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
 *  ======== uart.c ========
 */

/* Must be included first so BL3_DEBUG_ENABLE from bl3_config.h is visible
 * to the #ifdef guard below. */
#include "uart.h"

/*
 *
 *  Raw register UART init for BL3. No driver stack, no DPL, no Power manager.
 *
 *  Matches the hardware setup that ti_drivers_config.c performs for UART1
 *  on LP_EM_CC35X1 (XDS110 back-channel UART):
 *    - UARTLIN1, GPIO5=TX (IOMUX SEL 5), GPIO6=RX (IOMUX SEL 5)
 *    - 115200 8N1, UART clock = CPU/2 = 80 MHz
 *
 *  Register write sequence mirrors UART2WFF3.c + GPIOWFF3.c:
 *    1. Write IOMUX PORTCFG for TX and RX pins (mux sel in bits[4:0])
 *    2. Enable UARTLIN1 peripheral clock (UARTLIN_O_CLKCFG = 1)
 *    3. Configure baud rate (UARTConfigSetExpClk)
 *    4. Enable UART + FIFO + TX + RX (UARTEnable)
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

UART2_Handle uartHandle = 0;

#define BL3_UART_BAUD     (115200U)
#define BL3_UART_BUF_SIZE (256U)


void BL3_UART_init(void)
{
    UART2_Params params;    
    UART2_Params_init(&params);
    params.baudRate = BL3_UART_BAUD;
    uartHandle = UART2_open(CONFIG_UART2_0, &params);    
}


//*****************************************************************************
//
//! Outputs a character to the console
//!
//! \param[in]  char    - A character to be printed
//!
//! \return none
//
//*****************************************************************************
void putch(char ch)
{
    size_t bytesWritten;
    UART2_write(uartHandle, &ch, 1, &bytesWritten);
}

void BL3_UART_deinit(void)
{
    if (!uartHandle) return;

    UART2_close(uartHandle);
    uartHandle = 0;
}

#ifdef BL3_DEBUG_ENABLE

void BL3_DBG_print(const char *fmt, ...)
{
    char buf[BL3_UART_BUF_SIZE];
    va_list args;
    int len;

    if (!uartHandle) return;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if ((size_t)len >= sizeof(buf)) len = (int)(sizeof(buf) - 1);

    for (int i = 0; i < len; i++)
    {
        putch((uint8_t)buf[i]);
    }
}

#endif /* BL3_DEBUG_ENABLE */
