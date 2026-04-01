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

/*!
    \file   ota_fwu.h
    \brief  PSA FWU slot management — query, prepare, and display component
            status for the OTA example.

    This module wraps the PSA Firmware Update API to provide helpers for
    selecting target slots, preparing slots for writing, and displaying
    human-readable component status.
*/

#ifndef OTA_FWU_H
#define OTA_FWU_H

#include "ti/utils/FWU/psa_fwu.h"

/*****************************************************************************/
/*                           API Functions                                   */
/*****************************************************************************/

/*!
    \brief  Select the target (non-primary, READY) slot for a component pair.

    Given slot1 and slot2 IDs, queries both to find the one that is non-primary
    and in READY state. Also returns the primary slot's version for comparison.

    \param[in]  slot1Id         First slot component ID
    \param[in]  slot2Id         Second slot component ID
    \param[out] pTargetId       Selected target component ID for update
    \param[out] pPrimaryVersion Version of the primary (active) slot

    \return 0 on success, -1 if no suitable slot found
*/
int OTA_FWU_selectTargetSlot(int slot1Id, int slot2Id,
                             psa_fwu_component_t *pTargetId,
                             psa_fwu_image_version_t *pPrimaryVersion);

/*!
    \brief  Prepare a component slot for writing (clean/cancel if needed).

    Queries the component, checks state, and transitions it to READY.
    Rejects primary components.

    \param[in] componentId  PSA FWU component ID

    \return 0 on success, negative on error
*/
int OTA_FWU_prepareSlot(psa_fwu_component_t componentId);

/*!
    \brief  Display the full status table for all component slots.

    Queries every component via psa_fwu_query() and prints a table showing
    component ID, name, state, primary flag, active/loaded status, and version.
*/
void OTA_FWU_displayStatus(void);

/*!
    \brief  Get human-readable component name for a given component ID.

    \param[in] componentId  PSA FWU component ID (0 .. MAX_COMPONENT_ID-1)

    \return Component name string, or "Unknown" if ID is out of range
*/
const char *OTA_FWU_componentName(int componentId);

/*!
    \brief  Scan all components for pending OTA states after reboot.

    Queries every component and counts how many are in TRIAL, STAGED, or
    CANDIDATE state. Prints details for TRIAL and CANDIDATE components.

    \param[out] pTrialCount     Number of components in TRIAL state
    \param[out] pStagedCount    Number of components in STAGED state
    \param[out] pCandidateCount Number of components in CANDIDATE state
*/
void OTA_FWU_scanPendingStates(int *pTrialCount, int *pStagedCount,
                               int *pCandidateCount);

/*!
    \brief  Reject all staged/trial updates and clean resulting FAILED slots.

    Calls psa_fwu_reject(), then scans all components and cleans any
    non-primary slots that ended up in FAILED state.
*/
void OTA_FWU_rejectAndCleanAll(void);

/*!
    \brief  Cancel and clean all CANDIDATE components.

    Scans all components. For each non-primary CANDIDATE, calls
    psa_fwu_cancel() followed by psa_fwu_clean() to return it to READY.
*/
void OTA_FWU_cancelAndCleanCandidates(void);

#endif /* OTA_FWU_H */
