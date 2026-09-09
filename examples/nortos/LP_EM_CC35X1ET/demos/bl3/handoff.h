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
 *  ======== handoff.h ========
 *
 *  BL3 handoff to vendor application.
 *
 *  Implements the final two handoff steps:
 *
 *    BL3_HANDOFF_setHostBootDone()
 *              Assert SECGP_HOST_BOOT_DONE (register 0xb0). Signals the
 *              hardware that the M33 host boot sequence is complete and locks
 *              Boot Configs Flexibility items 9-19. BL2 deliberately leaves
 *              this register unset when BL3 is present.
 *
 *    BL3_HANDOFF_launchApp()
 *              Read the initial SP and reset handler from the vendor app vector
 *              table at appBase, disable interrupts, load MSP, and BX to the
 *              reset handler. Does not return.
 *              SYSRESETREQ is NOT used — it resets VTOR to 0x0 and causes BL2
 *              to re-run, never reaching the vendor app.
 */

#ifndef BL3_HANDOFF_H_
#define BL3_HANDOFF_H_

#include <stdint.h>

/* Assert HOST_BOOT_DONE to lock Boot Configs Flexibility items 9-19 */
void BL3_HANDOFF_setHostBootDone(void);

/* Direct branch to vendor app vector table base — does not return */
void BL3_HANDOFF_launchApp(uint32_t appBase);

#endif /* BL3_HANDOFF_H_ */
