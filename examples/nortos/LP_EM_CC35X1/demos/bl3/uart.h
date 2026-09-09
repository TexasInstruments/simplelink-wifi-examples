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
 *  ======== uart.h ========
 *
 *  UART configuration reference for BL3.
 *
 *  Demonstrates how to configure UART directly via driverlib, without
 *  SysConfig or the TI driver stack. Vendors can use this as a starting
 *  point for debug logging or any other serial communication needed
 *  during the boot sequence.
 *
 *  Uncomment BL3_DEBUG_ENABLE in bl3_config.h to enable output.
 */

#ifndef BL3_UART_H_
#define BL3_UART_H_

#include "config.h"

/*
 * Initialize the debug UART. Must be called before BL3_DBG_print().
 * Configures UART1 at 115200 baud, 8N1, using direct driverlib register writes.
 */
void BL3_UART_init(void);


/*
 * Drain the TX FIFO, disable UART1, turn off its clock, and restore
 * GPIO5/GPIO6 pin mux to the default GPIO function. Call once, immediately
 * before handing off to the vendor application.
 */
void BL3_UART_deinit(void);

#ifdef BL3_DEBUG_ENABLE

/*
 * Print a formatted debug message. Supports the same format specifiers as
 * printf. Output is sent synchronously — the call blocks until all bytes
 * are transmitted.
 */
void BL3_DBG_print(const char *fmt, ...);

#else  /* BL3_DEBUG_ENABLE not defined */

#define BL3_DBG_print(...)      do {} while (0)

#endif /* BL3_DEBUG_ENABLE */

#endif /* BL3_UART_H_ */
