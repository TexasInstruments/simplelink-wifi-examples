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

#include "gpio.h"
#include "uart.h"

#include <stdbool.h>
#include <stdint.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#if defined(BL3_DEBUG_ENABLE) && defined(BL3_GPIO_LOOPBACK_ENABLE)

void BL3_GPIO_init(void)
{
    GPIO_init();
}

#define GPIO_HIGH (1)
#define GPIO_LOW  (0)

void BL3_GPIO_loopbackTest(void)
{
    bool pass = true;

    BL3_DBG_print("[BL3] GPIO loopback: BP.5 (out) -> BP.6 (in)\r\n");
 
    BL3_DBG_print("[BL3] GPIO initialized\r\n");

    GPIO_write(CONFIG_GPIO_WRITE, GPIO_HIGH);        
    
    if (GPIO_read(CONFIG_GPIO_READ) != GPIO_HIGH) {
        BL3_DBG_print("[BL3]   FAIL: expected HIGH on GPIO11\r\n");
        pass = false;
    }

    GPIO_write(CONFIG_GPIO_WRITE, GPIO_LOW);
    
    if (GPIO_read(CONFIG_GPIO_READ) != GPIO_LOW) {
        BL3_DBG_print("[BL3]   FAIL: expected LOW on GPIO11\r\n");
        pass = false;
    }

    if(true == pass)
    {
        BL3_DBG_print("[BL3] GPIO teste - PASSED\r\n");
    }
    else 
    {
        BL3_DBG_print("[BL3] GPIO teste - FAILED\r\n");
    }
}



#endif /* BL3_DEBUG_ENABLE && BL3_GPIO_LOOPBACK_ENABLE */
