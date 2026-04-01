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
#include <ti/drivers/apps/Button.h>
#include "ti_drivers_config.h"
#include "button_if.h"

static struct 
{
    Button_Handle           handle;
    uint32_t                gpioId;
    BUTTON_IF_callback_f    callback;
} gButtons[CONFIG_TI_DRIVERS_BUTTON_COUNT];


int BUTTON_IF_init()
{
    Button_init();
    return 0;
}

static void buttonIfCallback(Button_Handle handle, Button_EventMask events)
{
    int buttonId;
    for(buttonId=0; buttonId<CONFIG_TI_DRIVERS_BUTTON_COUNT; buttonId++)
    {
        if(gButtons[buttonId].handle == handle)
        {
            // Note the BUTTON IF used the simplelink events mask value,
            // Thus the following casting can be done.
            BUTTON_IF_events_bm eventMask = (BUTTON_IF_events_bm)events;
            gButtons[buttonId].callback(eventMask);
            return;
        }
    }
}
int BUTTON_IF_registertCallback(uint32_t buttonId, 
                                uint32_t  gpioId,
                                BUTTON_IF_callback_f callback, 
                                BUTTON_IF_events_bm eventMask, 
                                BUTTON_IF_durationMS_t duration)
{
    Button_Params buttonParams;
    Button_Params_init(&buttonParams);

    // Note the BUTTON IF used the simplelink events mask value,
    // Thus the following casting can be done.
    buttonParams.buttonEventMask   = (Button_EventMask)eventMask;

    buttonParams.longPressDuration = duration; // ms
    gButtons[buttonId].handle         = Button_open(buttonId, &buttonParams);
    gButtons[buttonId].gpioId         = gpioId;

    gButtons[buttonId].callback         = callback;
    Button_setCallback(gButtons[buttonId].handle, buttonIfCallback);
    return 0;
}

int BUTTON_IF_enable(uint32_t buttonId) 
{
    GPIO_clearInt(gButtons[buttonId].gpioId);
    GPIO_enableInt(gButtons[buttonId].gpioId);
    return 0;
}

int BUTTON_IF_disable(uint32_t buttonId)
{
    GPIO_disableInt(gButtons[buttonId].gpioId);
    return 0;
}


