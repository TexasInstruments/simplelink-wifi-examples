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
    \file   ota_fwu.c
    \brief  PSA FWU slot management — query, prepare, and display component
            status for the OTA example.

    This module wraps the PSA Firmware Update API to provide helpers for
    selecting target slots, preparing slots for writing, and displaying
    human-readable component status.

    Dependencies: psa_fwu.h, uart_term.h (no dependency on other ota_* modules)
*/

#include "ota_fwu.h"
#include "uart_term.h"

/*****************************************************************************/
/*                           Component Name/State Tables                     */
/*****************************************************************************/

static const char *gComponentName[MAX_COMPONENT_ID] = {
    "BL2_Slot_1",
    "BL2_Slot_2",
    "Wireless_FW_Slot_1",
    "Wireless_FW_Slot_2",
    "Vendor_Image_Slot_1",
    "Vendor_Image_Slot_2"
};

static const char *gComponentState[] = {
    "READY",
    "WRITING",
    "CANDIDATE",
    "STAGED",
    "FAILED",
    "TRIAL",
    "REJECTED",
    "UPDATED"
};

/*****************************************************************************/
/*                           API Functions                                   */
/*****************************************************************************/

const char *OTA_FWU_componentName(int componentId)
{
    if (componentId >= 0 && componentId < MAX_COMPONENT_ID)
    {
        return gComponentName[componentId];
    }
    return "Unknown";
}

int OTA_FWU_selectTargetSlot(int slot1Id, int slot2Id,
                             psa_fwu_component_t *pTargetId,
                             psa_fwu_image_version_t *pPrimaryVersion)
{
    psa_fwu_component_info_t info1, info2;
    int ret1, ret2;

    ret1 = psa_fwu_query((psa_fwu_component_t)slot1Id, &info1);
    ret2 = psa_fwu_query((psa_fwu_component_t)slot2Id, &info2);

    if (ret1 != PSA_SUCCESS || ret2 != PSA_SUCCESS)
    {
        UART_PRINT("[OTA] Failed to query slots %d/%d\n\r", slot1Id, slot2Id);
        return -1;
    }

    /* Find the primary slot (for version) and the non-primary slot (target) */
    if (info1.impl.Primary && !info2.impl.Primary)
    {
        *pPrimaryVersion = info1.version;
        *pTargetId = (psa_fwu_component_t)slot2Id;
        return 0;
    }
    else if (info2.impl.Primary && !info1.impl.Primary)
    {
        *pPrimaryVersion = info2.version;
        *pTargetId = (psa_fwu_component_t)slot1Id;
        return 0;
    }

    UART_PRINT("[OTA] No suitable target slot for %d/%d\n\r", slot1Id, slot2Id);
    return -1;
}

int OTA_FWU_prepareSlot(psa_fwu_component_t componentId)
{
    psa_fwu_component_info_t compInfo;
    int ret;

    ret = psa_fwu_query(componentId, &compInfo);
    if (ret != PSA_SUCCESS)
    {
        UART_PRINT("[OTA] psa_fwu_query(%d) failed: %d\n\r", componentId, ret);
        return -1;
    }

    UART_PRINT("[OTA] Component %d (%s): state=%s, primary=%s, version=%d.%d.%d.%d\n\r",
                componentId,
                gComponentName[componentId],
                gComponentState[compInfo.state],
                compInfo.impl.Primary ? "Yes" : "No",
                compInfo.version.major,
                compInfo.version.minor,
                compInfo.version.patch,
                compInfo.version.build);

    if (compInfo.impl.Primary)
    {
        UART_PRINT("[OTA] Error: component %d is PRIMARY, cannot update\n\r",
                    componentId);
        return -1;
    }

    /* Transition component to READY based on its current state */
    if (compInfo.state == PSA_FWU_READY)
    {
        /* Already ready, nothing to do */
    }
    else if (compInfo.state == PSA_FWU_FAILED || compInfo.state == PSA_FWU_UPDATED)
    {
        UART_PRINT("[OTA] Component in %s state, erasing flash, please wait...\n\r",
                    gComponentState[compInfo.state]);
        ret = psa_fwu_clean(componentId);
        if (ret != PSA_SUCCESS)
        {
            UART_PRINT("[OTA] psa_fwu_clean failed: %d\n\r", ret);
            return -1;
        }
        UART_PRINT("[OTA] Done — component is now READY\n\r");
    }
    else if (compInfo.state == PSA_FWU_WRITING ||
             compInfo.state == PSA_FWU_CANDIDATE)
    {
        UART_PRINT("[OTA] Component in %s state, cancelling and erasing flash, please wait...\n\r",
                    gComponentState[compInfo.state]);
        ret = psa_fwu_cancel(componentId);
        if (ret != PSA_SUCCESS)
        {
            UART_PRINT("[OTA] psa_fwu_cancel failed: %d\n\r", ret);
            return -1;
        }
        ret = psa_fwu_clean(componentId);
        if (ret != PSA_SUCCESS)
        {
            UART_PRINT("[OTA] psa_fwu_clean failed: %d\n\r", ret);
            return -1;
        }
        UART_PRINT("[OTA] Done — component is now READY\n\r");
    }
    else if (compInfo.state == PSA_FWU_STAGED ||
             compInfo.state == PSA_FWU_TRIAL)
    {
        UART_PRINT("[OTA] Component in %s state, rejecting and erasing flash, please wait...\n\r",
                    gComponentState[compInfo.state]);
        ret = psa_fwu_reject(PSA_ERROR_NOT_PERMITTED);
        if (ret != PSA_SUCCESS && ret != PSA_SUCCESS_REBOOT)
        {
            UART_PRINT("[OTA] psa_fwu_reject failed: %d\n\r", ret);
            return -1;
        }
        ret = psa_fwu_clean(componentId);
        if (ret != PSA_SUCCESS)
        {
            UART_PRINT("[OTA] psa_fwu_clean failed: %d\n\r", ret);
            return -1;
        }
        UART_PRINT("[OTA] Done — component is now READY\n\r");
    }
    else if (compInfo.state == PSA_FWU_REJECTED)
    {
        UART_PRINT("[OTA] Component in %s state, erasing flash, please wait...\n\r",
                    gComponentState[compInfo.state]);
        /* REJECTED → need to move to FAILED first, then clean */
        ret = psa_fwu_clean(componentId);
        if (ret != PSA_SUCCESS)
        {
            UART_PRINT("[OTA] psa_fwu_clean failed: %d\n\r", ret);
            return -1;
        }
        UART_PRINT("[OTA] Done — component is now READY\n\r");
    }
    else
    {
        UART_PRINT("[OTA] Component in unexpected state %d, cannot prepare\n\r",
                    compInfo.state);
        return -1;
    }

    /* Verify component is now READY */
    ret = psa_fwu_query(componentId, &compInfo);
    if (ret != PSA_SUCCESS || compInfo.state != PSA_FWU_READY)
    {
        UART_PRINT("[OTA] Component not in READY state (state=%d), cannot proceed\n\r",
                    compInfo.state);
        return -1;
    }

    return 0;
}

void OTA_FWU_displayStatus(void)
{
    psa_fwu_component_info_t compInfo;
    int ci;
    int ret;

    UART_PRINT("\n\rComponents in flash:\n\r");
    UART_PRINT(" %-3s %-25s %-10s %-8s %-8s %-16s\n\r",
                "#", "Name", "State", "Primary", "Active", "Version");
    UART_PRINT("------------------------------------------------------------------------\n\r");

    for (ci = 0; ci < MAX_COMPONENT_ID; ci++)
    {
        ret = psa_fwu_query((psa_fwu_component_t)ci, &compInfo);
        if (ret == PSA_SUCCESS)
        {
            UART_PRINT(" %-3d %-25s %-10s %-8s %-8s %d.%d.%d.%d\n\r",
                        ci,
                        gComponentName[ci],
                        gComponentState[compInfo.state],
                        compInfo.impl.Primary ? "Yes" : "No",
                        compInfo.impl.running_status ? "Yes" : "No",
                        compInfo.version.major,
                        compInfo.version.minor,
                        compInfo.version.patch,
                        compInfo.version.build);
        }
    }
    UART_PRINT("\n\r");
}

void OTA_FWU_scanPendingStates(int *pTrialCount, int *pStagedCount,
                               int *pCandidateCount)
{
    psa_fwu_component_info_t compInfo;
    int ci;
    int ret;

    *pTrialCount = 0;
    *pStagedCount = 0;
    *pCandidateCount = 0;

    for (ci = 0; ci < MAX_COMPONENT_ID; ci++)
    {
        ret = psa_fwu_query((psa_fwu_component_t)ci, &compInfo);
        if (ret != PSA_SUCCESS)
        {
            continue;
        }
        if (compInfo.state == PSA_FWU_TRIAL)
        {
            if (*pTrialCount == 0)
            {
                UART_PRINT("\n\r[OTA] TRIAL firmware detected:\n\r");
            }
            UART_PRINT("  Component %d (%s): %d.%d.%d.%d [TRIAL]\n\r",
                        ci, gComponentName[ci],
                        compInfo.version.major, compInfo.version.minor,
                        compInfo.version.patch, compInfo.version.build);
            (*pTrialCount)++;
        }
        else if (compInfo.state == PSA_FWU_STAGED)
        {
            (*pStagedCount)++;
        }
        else if (compInfo.state == PSA_FWU_CANDIDATE)
        {
            if (*pCandidateCount == 0)
            {
                UART_PRINT("\n\r[OTA] Pending downloads found:\n\r");
            }
            UART_PRINT("  Component %d (%s): %d.%d.%d.%d [CANDIDATE]\n\r",
                        ci, gComponentName[ci],
                        compInfo.version.major, compInfo.version.minor,
                        compInfo.version.patch, compInfo.version.build);
            (*pCandidateCount)++;
        }
    }
}

void OTA_FWU_rejectAndCleanAll(void)
{
    psa_fwu_component_info_t compInfo;
    int ci;
    int ret;

    ret = psa_fwu_reject(PSA_ERROR_NOT_PERMITTED);
    if (ret == PSA_SUCCESS || ret == PSA_SUCCESS_REBOOT)
    {
        /* Clean all non-primary FAILED components */
        for (ci = 0; ci < MAX_COMPONENT_ID; ci++)
        {
            ret = psa_fwu_query((psa_fwu_component_t)ci, &compInfo);
            if (ret == PSA_SUCCESS && compInfo.state == PSA_FWU_FAILED &&
                !compInfo.impl.Primary)
            {
                psa_fwu_clean((psa_fwu_component_t)ci);
            }
        }
    }
}

void OTA_FWU_cancelAndCleanCandidates(void)
{
    psa_fwu_component_info_t compInfo;
    int ci;
    int ret;

    for (ci = 0; ci < MAX_COMPONENT_ID; ci++)
    {
        ret = psa_fwu_query((psa_fwu_component_t)ci, &compInfo);
        if (ret == PSA_SUCCESS &&
            compInfo.state == PSA_FWU_CANDIDATE &&
            !compInfo.impl.Primary)
        {
            psa_fwu_cancel((psa_fwu_component_t)ci);
            psa_fwu_clean((psa_fwu_component_t)ci);
        }
    }
}
