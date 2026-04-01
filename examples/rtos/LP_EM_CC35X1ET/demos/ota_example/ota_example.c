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

/*****************************************************************************

   Application Name     - OTA Example
   Application Overview - This application demonstrates how to perform a
                          network-based Over-The-Air (OTA) firmware update
                          on the CC35xx device. It connects to a WiFi network,
                          queries an HTTPS server for available updates via
                          auto-discovery, displays a summary of available
                          images, and lets the user choose which component
                          to download and install.

   Application Details  - Refer to 'OTA Example' README.md

 *****************************************************************************/

/* Standard includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  /* atoi */

/* RTOS Header files */
#include "FreeRTOS.h"
#include "task.h"

/* TI-DRIVERS Header files */
#include "ti_drivers_config.h"

/* Application headers */
#include "ota_example.h"
#include "uart_term.h"
#include "led_if.h"

/* OTA topic modules */
#include "ota_wifi.h"
#include "ota_fwu.h"
#include "ota_server.h"

/* PSA Firmware Update API */
#include "ti/utils/FWU/psa_fwu.h"

/* Hardware registers for cache configuration */
#ifndef HWREG
#define HWREG(x) (*((volatile unsigned long *)(x)))
#endif
#define ICACHE_BASE 0x41902000

/* mbedTLS threading and platform */
#define MBEDTLS_CONFIG_FILE "config-hsm.h"
#include <mbedtls/platform.h>
#include "osi_kernel.h"
extern void initialize_mbedtls_threading(void);

/*****************************************************************************/
/*                           Defines                                         */
/*****************************************************************************/

/* Hardcoded date for TLS certificate validation.
   The CC35xx has no RTC, so the system clock starts at epoch 0 (Jan 1, 1970).
   This must be set to a date within the server certificate's validity period
   before any HTTPS connection. Update this value if certificates are renewed. */
#define OTA_SYSTEM_TIME_EPOCH   1767225600UL  /* Jan 1, 2026 00:00:00 UTC */

/*****************************************************************************/
/*                           Main Thread                                     */
/*****************************************************************************/

void *mainThread(void *pvParameters)
{
    int ret;
    char serverUrl[MAX_URL_LEN];
    static OtaUpdateEntry_t entries[OTA_MAX_UPDATES];
    int entryCount = 0;

    /* Enable instruction cache */
    HWREG(ICACHE_BASE + 0x84) |= 0x00000001;
    HWREG(ICACHE_BASE + 0x4) |= 0xc0000000;

    /* Initialize UART terminal */
    InitTerm();

    /* Initialize LED driver */
    LED_IF_init();

    /* Initialize mbedTLS threading support */
    initialize_mbedtls_threading();

    /* Use thread-safe OS allocation wrappers for mbedTLS.
       Raw calloc/free are not thread-safe in this SDK. */
    mbedtls_platform_set_calloc_free(os_calloc, os_free);

    UART_PRINT("\n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("   CC35xx OTA Example Application\n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("\n\r");

    /* Initialize PSA FWU (reads boot report, sets up component states) */
    psa_fwu_init();
    UART_PRINT("[OTA] PSA FWU initialized\n\r");

    /* One-time WiFi stack initialization (sync objects, TCP/IP, DNS) */
    ret = OTA_WIFI_initStack();
    if (ret != 0)
    {
        UART_PRINT("[OTA] WiFi stack init failed\n\r");
        goto done;
    }

    /* Main application loop — WiFi disconnect returns here */
wifi_disconnected:

    /* Startup checks: detect pending OTA states after reboot */
    {
        int trialFound = 0;
        int stagedFound = 0;
        int candidateFound = 0;
        char promptBuf[CMD_BUFFER_LEN];

        /* Scan all components for pending states */
        OTA_FWU_scanPendingStates(&trialFound, &stagedFound, &candidateFound);

        /* TRIAL: test WiFi, accept or reject */
        if (trialFound > 0)
        {
            while (1)
            {
                UART_PRINT("\n\r[OTA] Trial menu:\n\r");
                UART_PRINT(" [1] Run self-test (WiFi connectivity)\n\r");
                UART_PRINT(" [2] Accept (commit and reboot)\n\r");
                UART_PRINT(" [3] Reject (rollback and reboot)\n\r");
                UART_PRINT(" [4] Show components in flash\n\r");
                UART_PRINT(" [5] Continue to main menu\n\r");
                ret = GetCmd(promptBuf, CMD_BUFFER_LEN, "Choice: ");
                UART_PRINT("\n\r");
                if (ret <= 0)
                {
                    continue;
                }

                switch (promptBuf[0])
                {
                case '1':
                    /* Self-test: WiFi connectivity — scan, connect, then decide */
                    UART_PRINT("[OTA] Running self-test (WiFi connectivity)...\n\r");
                    ret = OTA_WIFI_connect();
                    if (ret != 0)
                    {
                        UART_PRINT("\n\r[OTA] Self-test failed — try again or reject\n\r");
                        continue;
                    }
                    UART_PRINT("\n\r[OTA] Self-test passed!\n\r");

                    /* Post-test decision (WiFi stays connected — reboot kills it) */
                    while (1)
                    {
                        UART_PRINT("\n\r[OTA] Self-test successful. Select action:\n\r");
                        UART_PRINT(" [1] Accept (commit and reboot)\n\r");
                        UART_PRINT(" [2] Reject (rollback and reboot)\n\r");
                        UART_PRINT(" [3] Back to trial menu\n\r");
                        ret = GetCmd(promptBuf, CMD_BUFFER_LEN, "Choice: ");
                        UART_PRINT("\n\r");
                        if (ret <= 0)
                        {
                            continue;
                        }
                        if (promptBuf[0] == '1')
                        {
                            UART_PRINT("[OTA] Accepting trial firmware...\n\r");
                            ret = psa_fwu_accept();
                            if (ret == PSA_SUCCESS_REBOOT || ret == PSA_SUCCESS)
                            {
                                UART_PRINT("[OTA] Rebooting to commit update...\n\r");
                                psa_fwu_request_reboot();
                            }
                            UART_PRINT("[OTA] Accept failed: %d\n\r", ret);
                            continue;
                        }
                        if (promptBuf[0] == '2')
                        {
                            UART_PRINT("[OTA] Rejecting trial firmware...\n\r");
                            ret = psa_fwu_reject(PSA_ERROR_NOT_PERMITTED);
                            if (ret == PSA_SUCCESS_REBOOT)
                            {
                                UART_PRINT("[OTA] Rebooting to rollback...\n\r");
                                psa_fwu_request_reboot();
                            }
                            UART_PRINT("[OTA] Reject failed: %d\n\r", ret);
                            continue;
                        }
                        if (promptBuf[0] == '3')
                        {
                            if (OTA_WIFI_isConnected())
                            {
                                OTA_WIFI_disconnectAndStop();
                            }
                            break; /* back to trial menu */
                        }
                        UART_PRINT("[OTA] Invalid choice\n\r");
                    }
                    continue;

                case '2':
                    UART_PRINT("[OTA] Accepting trial firmware...\n\r");
                    ret = psa_fwu_accept();
                    if (ret == PSA_SUCCESS_REBOOT || ret == PSA_SUCCESS)
                    {
                        UART_PRINT("[OTA] Rebooting to commit update...\n\r");
                        psa_fwu_request_reboot();
                    }
                    UART_PRINT("[OTA] Accept failed: %d\n\r", ret);
                    continue;

                case '3':
                    UART_PRINT("[OTA] Rejecting trial firmware...\n\r");
                    ret = psa_fwu_reject(PSA_ERROR_NOT_PERMITTED);
                    if (ret == PSA_SUCCESS_REBOOT)
                    {
                        UART_PRINT("[OTA] Rebooting to rollback...\n\r");
                        psa_fwu_request_reboot();
                    }
                    else if (ret == PSA_SUCCESS)
                    {
                        UART_PRINT("[OTA] Rejected\n\r");
                    }
                    else
                    {
                        UART_PRINT("[OTA] Reject failed: %d\n\r", ret);
                    }
                    continue;

                case '4':
                    OTA_FWU_displayStatus();
                    continue;

                case '5':
                    break; /* exit trial menu */

                default:
                    UART_PRINT("[OTA] Invalid choice\n\r");
                    continue;
                }
                break; /* case '5' exits */
            }
        }

        /* STAGED after unexpected reboot: auto-reject */
        if (stagedFound > 0 && trialFound == 0)
        {
            UART_PRINT("\n\r[OTA] WARNING: STAGED updates found after unexpected reboot.\n\r");
            UART_PRINT("[OTA] These updates cannot be applied and will be removed.\n\r");
            UART_PRINT("[OTA] Rejecting and cleaning up, please wait...\n\r");
            OTA_FWU_rejectAndCleanAll();
            UART_PRINT("[OTA] Done. Please re-download and install the updates.\n\r");
        }

        /* CANDIDATE: downloaded updates waiting to be installed */
        if (candidateFound > 0 && trialFound == 0 && stagedFound == 0)
        {
            while (1)
            {
                UART_PRINT("\n\r[OTA] Downloaded updates ready to install. Select action:\n\r");
                UART_PRINT(" [1] Install and reboot\n\r");
                UART_PRINT(" [2] Cancel downloads\n\r");
                UART_PRINT(" [3] Continue to main menu\n\r");
                ret = GetCmd(promptBuf, CMD_BUFFER_LEN, "Choice: ");
                UART_PRINT("\n\r");
                if (ret <= 0)
                {
                    continue;
                }

                switch (promptBuf[0])
                {
                case '1':
                    UART_PRINT("[OTA] Installing and rebooting...\n\r");
                    ret = psa_fwu_install();
                    if (ret == PSA_SUCCESS_REBOOT)
                    {
                        UART_PRINT("[OTA] Install OK, rebooting...\n\r");
                        psa_fwu_request_reboot();
                        /* Should not reach here */
                    }
                    UART_PRINT("[OTA] Install failed: %d\n\r", ret);
                    continue;

                case '2':
                    UART_PRINT("[OTA] Cancelling downloads, please wait...\n\r");
                    OTA_FWU_cancelAndCleanCandidates();
                    UART_PRINT("[OTA] Done\n\r");
                    break; /* exit to Menu 1 */

                case '3':
                    break; /* exit to Menu 1 */

                default:
                    UART_PRINT("[OTA] Invalid choice\n\r");
                    continue;
                }
                break;
            }
        }
    }

    /* Menu 1: Local firmware management + WiFi connect */
    {
        char startBuf[CMD_BUFFER_LEN];

        while (1)
        {
            UART_PRINT("\n\r[OTA] Select action:\n\r");
            UART_PRINT(" [1] Connect to WiFi\n\r");
            UART_PRINT(" [2] Show components in flash\n\r");
            UART_PRINT(" [3] Clean component\n\r");
            ret = GetCmd(startBuf, CMD_BUFFER_LEN, "Choice: ");
            UART_PRINT("\n\r");
            if (ret <= 0)
            {
                continue;
            }

            switch (startBuf[0])
            {
            case '1':
                break; /* exit the while loop */

            case '2':
                OTA_FWU_displayStatus();
                continue;

            case '3':
                {
                    char cleanBuf[CMD_BUFFER_LEN];
                    char cleanPrompt[64];
                    int compId;

                    OTA_FWU_displayStatus();
                    snprintf(cleanPrompt, sizeof(cleanPrompt),
                             "Enter component ID to clean (0-%d): ",
                             MAX_COMPONENT_ID - 1);
                    ret = GetCmd(cleanBuf, CMD_BUFFER_LEN, cleanPrompt);
                    UART_PRINT("\n\r");
                    if (ret <= 0)
                    {
                        UART_PRINT("[OTA] Invalid input\n\r");
                        continue;
                    }

                    compId = atoi(cleanBuf);
                    if (compId < 0 || compId >= MAX_COMPONENT_ID)
                    {
                        UART_PRINT("[OTA] Invalid component ID\n\r");
                        continue;
                    }

                    ret = OTA_FWU_prepareSlot((psa_fwu_component_t)compId);
                    if (ret != 0)
                    {
                        UART_PRINT("[OTA] Failed to clean component %d\n\r", compId);
                        continue;
                    }
                }
                continue;

            default:
                UART_PRINT("[OTA] Invalid choice\n\r");
                continue;
            }
            break; /* case '1' breaks here to exit while loop */
        }
    }

    ret = OTA_WIFI_connect();
    if (ret != 0)
    {
        UART_PRINT("[OTA] WiFi connection failed. Cannot proceed.\n\r");
        goto wifi_disconnected;
    }

    /* Step 2: Set system clock for TLS certificate date validation.
       The CC35xx has no RTC — without this, all certificates appear
       "not yet valid" because the device thinks it's Jan 1, 1970. */
    osi_SetDateTimeS(OTA_SYSTEM_TIME_EPOCH);
    UART_PRINT("[OTA] System clock set for TLS validation\n\r");

    /* Server connection + OTA menu loop — disconnect returns here */
    while (1)
    {
        /* Prompt for server base URL */
        UART_PRINT("\n\r[OTA] Server Connection\n\r");
        ret = GetCmd(serverUrl, MAX_URL_LEN,
                     "Enter server base URL (e.g. https://192.168.50.100:8443): ");
        UART_PRINT("\n\r");
        if (ret <= 0)
        {
            UART_PRINT("[OTA] Invalid server URL, please try again\n\r");
            continue;
        }

        /* Strip trailing slash if present */
        {
            int len = (int)strlen(serverUrl);
            if (len > 0 && serverUrl[len - 1] == '/')
            {
                serverUrl[len - 1] = '\0';
            }
        }

        /* Auto-query server on entry */
        entryCount = OTA_SERVER_queryUpdates(serverUrl, entries, OTA_MAX_UPDATES);
        if (entryCount < 0)
        {
            UART_PRINT("[OTA] Initial query failed\n\r");
            entryCount = 0;
        }

        /* Menu 2a + 2b loop: reject in 2b returns to 2a */
        {
            char menuBuf[CMD_BUFFER_LEN];
            int disconnectServer = 0;
            int disconnectWifi = 0;

            while (1)
            {
                /* Menu 2a: Download phase */
                int installed = 0;

                while (1)
                {
                    UART_PRINT("\n\r[OTA] Download menu:\n\r");
                    UART_PRINT(" [1] Re-query server\n\r");
                    UART_PRINT(" [2] Download update\n\r");
                    UART_PRINT(" [3] Install (stage candidates)\n\r");
                    UART_PRINT(" [4] Show components in flash\n\r");
                    UART_PRINT(" [5] Connect to server\n\r");
                    UART_PRINT(" [6] Disconnect WiFi\n\r");
                    UART_PRINT(" [7] Quit\n\r");
                    ret = GetCmd(menuBuf, CMD_BUFFER_LEN, "Choice: ");
                    UART_PRINT("\n\r");
                    if (ret <= 0)
                    {
                        continue;
                    }

                    switch (menuBuf[0])
                    {
                    case '1':
                        entryCount = OTA_SERVER_queryUpdates(serverUrl, entries,
                                                         OTA_MAX_UPDATES);
                        if (entryCount < 0)
                        {
                            UART_PRINT("[OTA] Query failed\n\r");
                            entryCount = 0;
                        }
                        break;

                    case '2':
                        if (entryCount <= 0)
                        {
                            UART_PRINT("[OTA] Query server first\n\r");
                            break;
                        }
                        {
                            char selBuf[CMD_BUFFER_LEN];
                            char promptBuf[64];
                            int sel;
                            int idx;

                            UART_PRINT("Available updates:\n\r");
                            for (idx = 0; idx < entryCount; idx++)
                            {
                                UART_PRINT(" [%d] %s (version %s)\n\r",
                                            idx, entries[idx].component,
                                            entries[idx].version);
                            }

                            snprintf(promptBuf, sizeof(promptBuf),
                                     "Select update to download (0-%d): ",
                                     entryCount - 1);
                            ret = GetCmd(selBuf, CMD_BUFFER_LEN, promptBuf);
                            UART_PRINT("\n\r");
                            if (ret <= 0)
                            {
                                UART_PRINT("[OTA] Invalid input\n\r");
                                break;
                            }

                            sel = atoi(selBuf);
                            if (sel < 0 || sel >= entryCount)
                            {
                                UART_PRINT("[OTA] Invalid selection\n\r");
                                break;
                            }

                            ret = OTA_SERVER_downloadUpdate(serverUrl, &entries[sel]);
                            if (ret != 0)
                            {
                                UART_PRINT("[OTA] Download failed\n\r");
                            }
                        }
                        break;

                    case '3':
                        UART_PRINT("[OTA] Installing candidates...\n\r");
                        ret = psa_fwu_install();
                        if (ret == PSA_SUCCESS_REBOOT)
                        {
                            UART_PRINT("[OTA] Install OK — candidates are now STAGED\n\r");
                            installed = 1;
                        }
                        else
                        {
                            UART_PRINT("[OTA] Install failed: %d (no candidates or conflict)\n\r", ret);
                        }
                        break;

                    case '4':
                        OTA_FWU_displayStatus();
                        break;

                    case '5':
                        UART_PRINT("[OTA] Returning to server connection...\n\r");
                        entryCount = 0;
                        disconnectServer = 1;
                        break;

                    case '6':
                        UART_PRINT("[OTA] Disconnecting from WiFi...\n\r");
                        OTA_WIFI_disconnectAndStop();
                        entryCount = 0;
                        disconnectWifi = 1;
                        break;

                    case '7':
                        UART_PRINT("[OTA] Exiting...\n\r");
                        goto done;

                    default:
                        UART_PRINT("[OTA] Invalid choice\n\r");
                        break;
                    }

                    if (disconnectServer || disconnectWifi || installed)
                    {
                        break;
                    }
                } /* end Menu 2a */

                if (disconnectWifi || disconnectServer)
                {
                    break; /* exit 2a+2b loop */
                }

                /* Menu 2b: Post-install phase (STAGED components) */
                if (installed)
                {
                    int rejected = 0;

                    while (1)
                    {
                        UART_PRINT("\n\r[OTA] Updates staged. Select action:\n\r");
                        UART_PRINT(" [1] Request reboot\n\r");
                        UART_PRINT(" [2] Reject (undo install)\n\r");
                        UART_PRINT(" [3] Show components in flash\n\r");
                        ret = GetCmd(menuBuf, CMD_BUFFER_LEN, "Choice: ");
                        UART_PRINT("\n\r");
                        if (ret <= 0)
                        {
                            continue;
                        }

                        switch (menuBuf[0])
                        {
                        case '1':
                            UART_PRINT("[OTA] Requesting reboot...\n\r");
                            ret = psa_fwu_request_reboot();
                            if (ret != PSA_SUCCESS)
                            {
                                UART_PRINT("[OTA] Reboot failed: %d\n\r", ret);
                            }
                            /* If successful, device resets and we never get here */
                            continue;

                        case '2':
                            UART_PRINT("[OTA] Rejecting staged updates...\n\r");
                            ret = psa_fwu_reject(PSA_ERROR_NOT_PERMITTED);
                            if (ret == PSA_SUCCESS)
                            {
                                UART_PRINT("[OTA] Reject OK — returning to download menu\n\r");
                                rejected = 1;
                            }
                            else if (ret == PSA_SUCCESS_REBOOT)
                            {
                                UART_PRINT("[OTA] Reject OK — reboot needed for rollback\n\r");
                                psa_fwu_request_reboot();
                            }
                            else
                            {
                                UART_PRINT("[OTA] Reject failed: %d\n\r", ret);
                            }
                            break;

                        case '3':
                            OTA_FWU_displayStatus();
                            continue;

                        default:
                            UART_PRINT("[OTA] Invalid choice\n\r");
                            continue;
                        }

                        if (rejected)
                        {
                            break; /* exit Menu 2b, loop back to Menu 2a */
                        }
                    }
                    continue; /* back to top of 2a+2b loop → Menu 2a */
                }
            } /* end 2a+2b loop */

            if (disconnectWifi)
            {
                break; /* break server loop, go back to Menu 1 */
            }
            /* disconnectServer: falls through to server loop, re-prompts URL */
        }
    } /* end server connection loop */

    /* WiFi was disconnected — go back to Menu 1 */
    goto wifi_disconnected;

done:
    UART_PRINT("\n\r[OTA] Application ended.\n\r");

    /* Idle loop */
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return NULL;
}
