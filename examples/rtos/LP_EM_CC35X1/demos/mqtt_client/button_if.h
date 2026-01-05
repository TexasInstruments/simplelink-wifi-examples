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
#ifndef __BUTTON_IF_H__
#define __BUTTON_IF_H__

// TI-Driver includes
#include <stdint.h>
//Defines

/* 
 * possible event values (set and received as bit mask).
 * The value are defined below.  
 * Note: Values are directly mapped to simplelink Button driver values.
 *       Don't change as it will impact the simplelink implementation!
 */
typedef unsigned long BUTTON_IF_events_bm;


#define BUTTON_IF_EV_PRESSED        0x01  // event upon pressing the button
#define BUTTON_IF_EV_LONG_PRESSED   0x02 // event upon pressing the button (duration [ms] > 0) 
#define BUTTON_IF_EV_CLICKED        0x08 // Event upon releasing the button
#define BUTTON_IF_EV_LONG_CLICKED   0x10 // event upon releasing the button (duration [ms] > 0) 
#define BUTTON_IF_EV_DOUBLE_CLICKED 0x20 // event upon double-cleckign the button

typedef void (*BUTTON_IF_callback_f)(BUTTON_IF_events_bm eventMask);
typedef unsigned long BUTTON_IF_durationMS_t; // Long Press/Click duration in MS

/*!
 *  @brief  BUTTON_IF module init
 *
 *  Need to be called before any other BUTTON_IF API
 *
 *  @return  0 - upon success, -1 upon error (or if platfrom has no Buttons support)
 *
 *  @sa      BUTTON_IF_registertCallback()
 */
int BUTTON_IF_init();

/*!
 *  @brief  Register a callback for specific button and set of events
 *
 *  @pre       BUTTON_IF_init() has to be called first
 *
 *  @param[in] button_id  Logical button number indexed into the Button_config table
 *
 *  @param[in] gpio_id    GPIO Index the corresponds to the button
 *
 *  @param[in] callback  A pointer to button event handler
 *
 *  @param[in] eventMask  A bit-mask of trigger events
 *
 *  @param[in] duration  In case Long press or click is defined, 
 *                        this defines the duration in milliseconds for the trigger
 *
 *  @return  0 upon success, -1 in case of param error (or in case functionality is not supported)
 *
 *  @sa      BUTTON_IF_init()
 *  @sa      BUTTON_IF_enable()
 *  @sa      BUTTON_IF_disable()
 */
int BUTTON_IF_registertCallback(uint32_t button_id, 
                               uint32_t  gpio_id,
                               BUTTON_IF_callback_f callback,
                               BUTTON_IF_events_bm eventMask, 
                               BUTTON_IF_durationMS_t duration);

/*!
 *  @brief  Enable Button Interrupt
 *
 *  @pre       BUTTON_IF_init() has to be called first
 *
 *  @param[in] button_id  Logical button number indexed into the Button_config table
 *
 *  @return  0 upon success, -1 in case of param error (or in case functionality is not supported)
 *
 *  @sa      BUTTON_IF_registertCallback()
 *  @sa      BUTTON_IF_disable()
 */
int BUTTON_IF_enable(uint32_t buttonId);

/*!
 *  @brief  Disable Button Interrupt
 *
 *  @pre       BUTTON_IF_init() has to be called first
 *
 *  @param[in] button_id  Logical button number indexed into the Button_config table
 *
 *  @return  0 upon success, -1 in case of param error (or in case functionality is not supported)
 *
 *  @sa      BUTTON_IF_registertCallback()
 *  @sa      BUTTON_IF_enable()
 */
int BUTTON_IF_disable(uint32_t buttonId);

#endif // __BUTTON_IF_H__
