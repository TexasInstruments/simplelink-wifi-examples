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
#ifndef __LED_IF_H__
#define __LED_IF_H__

// TI-Driver includes
#include <stdint.h>
//Defines


/*!
 *  @brief  LED_IF module init
 *
 *  Need to be called before any other LED_IF API
 *
 *  @return  0 - upon success, -1 upon error (or if platfrom has no Buttons support)
 *
 *  @sa      LED_IF_set()
 *  @sa      LED_IF_toggle()
 *  @sa      LED_IF_blink()
 */
int LED_IF_init();

/*!
 *  @brief  Turns a specifc LED ON or OFF
 *
 *  @pre    LED_IF_init() has to be called first
 *
 *  @param[in] index    Logical button number indexed into the LED_config table
 *
 *  @param[in] brightness  level (percentage) of brigtness: 0-100
 *                          0 turns the LED off. 
 *                          If brtiness is not supported - any non-zero value means ON.
 *
 *  @return  0 - upon success, -1 upon error (or if platfrom has no Buttons support)
 *
 *  @sa      LED_IF_toggle()
 *  @sa      LED_IF_blink()
 */
int LED_IF_set(uint32_t index, uint8_t brightness);

/*!
 *  @brief  Toggle the state of a specifc LED
 *
 *  @pre    LED_IF_init() has to be called first
 *
 *  @param[in] index    Logical button number indexed into the LED_config table
 *
 *  @param[in] brightness  level (percentage) of brigtness: 0-100 (when turned on) 
 *                          0 just turns the LED off 
 *
 *  @return  0 - upon success, -1 upon error (or if platfrom has no Buttons support)
 *
 *  @sa      LED_IF_toggle()
 *  @sa      LED_IF_blink()
 */
int LED_IF_toggle(uint32_t index, uint8_t brightness);

/*!
 *  @brief  Start or stop LED blinking
 *
 *  @pre    LED_IF_init() has to be called first
 *
 *  @param[in] index    Logical button number indexed into the LED_config table
 *
 *  @param[in] brightness  level (percentage) of brigtness: 0-100 (when turned on) 
 *                          0 will stop the blicking and turn the LED off 
 *
 *  @return  0 - upon success, -1 upon error (or if platfrom has no Buttons support)
 *
 *  @sa      LED_IF_toggle()
 *  @sa      LED_IF_blink()
 */
int LED_IF_blink(uint32_t index, uint16_t blinkPeriod, uint16_t blinkCount, uint8_t brightness);


#endif // __LED_IF_H__
