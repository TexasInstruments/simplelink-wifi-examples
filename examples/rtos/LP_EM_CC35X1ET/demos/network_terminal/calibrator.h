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
#ifndef __CALIBRATOR_H__
#define __CALIBRATOR_H__

enum cc33xx_manual_calibIDs_e
{
    /*  0x0    */   CALIBRATION_RX_IQMM_ID                  = 0x0,
    /*  0x1    */   CALIBRATION_RX_DC_CORRECTION_ID         = 0x1,
    /*  0x2    */   CALIBRATION_TX_AUX_RX_DC_ID             = 0x2,
    /*  0x3    */   CALIBRATION_TX_LOL_ID                   = 0x3,
    /*  0x4    */   CALIBRATION_TX_IQMM_ID                  = 0x4,
    /*  0x5    */   CALIBRATION_TX_RFNL_AND_DPD_ID          = 0x5,
    /*  0x6    */   CALIB_RX_SPUR_CANCELER_ID               = 0x6,
};


#define CALIB_RX_IQMM_BITWISE_MASK              (1 << CALIBRATION_RX_IQMM_ID)
#define CALIB_RX_DC_BITWISE_MASK                (1 << CALIBRATION_RX_DC_CORRECTION_ID)
#define CALIB_RX_SPUR_CANCELER_BITWISE_MASK     (1 << CALIB_RX_SPUR_CANCELER_ID)
#define CALIB_TX_AUX_RX_DC_BITWISE_MASK         (1 << CALIBRATION_TX_AUX_RX_DC_ID)
#define CALIB_TX_IQMM_BITWISE_MASK              (1 << CALIBRATION_TX_IQMM_ID)
#define CALIB_TX_LOL_BITWISE_MASK               (1 << CALIBRATION_TX_LOL_ID)
#define CALIB_TX_RFNL_AND_DPD_BITWISE_MASK      (1 << CALIBRATION_TX_RFNL_AND_DPD_ID)

typedef enum
{
    CALIBRATOR_ACTIVE_MODE_RX_TH  = 0,
    CALIBRATOR_ACTIVE_MODE_TX_TH  = 1,
    CALIBRATOR_ACTIVE_MODE_RX_TO  = 2,
    CALIBRATOR_ACTIVE_MODE_TX_TO  = 3,
    CALIBRATOR_FORCE_POWER_MODE   = 4,
} calibrator_traffic_parameters;


int32_t cmdCalibratorCallback(void *arg);
int32_t printCalibratorUsage(void *arg);

extern char calibratorStr[];


#endif /* __CALIBRATOR_H__ */
