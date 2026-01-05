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
/*
 *  Terminal
 */

// Standard includes
#include <ti/drivers/apps/LED.h>
#include "ti_drivers_config.h"
#include "led_if.h"

static struct 
{
    LED_Handle handle;
} gLeds[CONFIG_TI_DRIVERS_LED_COUNT];

int LED_IF_init()
{
    int i;
    LED_Params ledParams;
    LED_init();

    LED_Params_init(&ledParams);
    for(i=0; i<CONFIG_TI_DRIVERS_LED_COUNT; i++)
    {
        gLeds[i].handle = LED_open(i, &ledParams);
        if (gLeds[i].handle == NULL)
        {
            return -1;
        }
    }
    return 0;
}


int LED_IF_set(uint32_t index, uint8_t brightness)
{
    if(brightness)
    {
        if(LED_setOn(gLeds[index].handle, brightness))
            return 0;
    } 
    else if(LED_setOff(gLeds[index].handle))
    {
        return 0;
    }
    return -1;        
}

int LED_IF_toggle(uint32_t index, uint8_t brightness)
{
    LED_setBrightnessLevel(gLeds[index].handle, brightness);
    LED_toggle(gLeds[index].handle);
    return 0;
}

int LED_IF_blink(uint32_t index, uint16_t blinkPeriod, uint16_t blinkCount, uint8_t brightness)
{
    if(brightness)
    {
        LED_setBrightnessLevel(gLeds[index].handle, brightness);
        LED_startBlinking(gLeds[index].handle, blinkPeriod, blinkCount);
        return 0;
    }
    else
    {
        LED_stopBlinking(gLeds[index].handle);
    }
    return -1;
}
