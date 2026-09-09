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
/*
 *  ======== gpio.h ========
 *
 *  GPIO loopback test for BL3.
 *
 *  Drives GPIO10 (BP.5) high and low and reads back the level on GPIO11 (BP.6).
 *  Requires a jumper wire between BP.5 and BP.6.
 *
 *  Enable by defining BL3_DEBUG_ENABLE and BL3_GPIO_LOOPBACK_ENABLE in bl3_config.h.
 */

#ifndef BL3_GPIO_H_
#define BL3_GPIO_H_

#include "config.h"

#if defined(BL3_DEBUG_ENABLE) && defined(BL3_GPIO_LOOPBACK_ENABLE)

void BL3_GPIO_init(void);
void BL3_GPIO_loopbackTest(void);

#else

#define BL3_GPIO_init() do {} while (0)
#define BL3_GPIO_loopbackTest() do {} while (0)

#endif /* BL3_DEBUG_ENABLE && BL3_GPIO_LOOPBACK_ENABLE */

#endif /* BL3_GPIO_H_ */
