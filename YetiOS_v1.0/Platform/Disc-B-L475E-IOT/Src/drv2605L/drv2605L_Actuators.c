/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//#############################################################################
//
//! \file   drv2605L_Actuators.c
//
//  Group:          MSP
//  Target Devices: MSP430FR2xx/4xx/5xx/6xx
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// TI Release: 1.00.00.30
// Release Date: Thu, Oct 01, 2015  5:05:49 PM
//#############################################################################

//*****************************************************************************
// Includes
//*****************************************************************************

#include "drv2605L_Actuators.h"

//*****************************************************************************
// Definitions
//*****************************************************************************

// \brief A macro to extract the lower 2 bits of a 4-bit constant.
//!
#define FOURBIT_LOWERHALF(input) (input & 0x03)

//! \brief A macro to extract the upper 2 bits of a 4-bit constant.
//!
#define FOURBIT_UPPERHALF(input) ((input >> 2) & 0x03)

//*****************************************************************************
// Global Variables
//*****************************************************************************

//! \brief Default Actuator Configuration: 0934W (Mplus) [LRA]
//!
const drv2605L_Actuator_t drv2605L_actuator_LRA0934W =
{
    // Basic Parameters
	.ratedVoltage.r = 0x52,
	.overdriveClampVoltage.r = 0x90,

	// Auto-Calibration Parameters
	.autoCal_Compensation.r = 0x0C,
	.autoCal_BackEMF.r = 0x71,

	// Feedback Control Parameters
	.feedbackCtrl.b.bBackEMFGain = drv2605L_backEMFgain_3,
	.feedbackCtrl.b.bLoopGain = drv2605L_loopGain_medium,
	.feedbackCtrl.b.bFBBrakeFactor = drv2605L_feedbackBrakeFactor_3x,
	.feedbackCtrl.b.bActuatorType = drv2605L_actuator_LRA,

	// Control Register 1 Parameters
	.ctrlReg1.b.bDriveTime = 0,
	.ctrlReg1.b.bACCoupling = false,
	.ctrlReg1.b.bStartupBoost = true,

	// Control Register 2 Parameters
	.ctrlReg2.b.bCurrDissTimeLSB = FOURBIT_LOWERHALF(1),
	.ctrlReg2.b.bBlankingTimeLSB = FOURBIT_LOWERHALF(1),
	.ctrlReg2.b.bSamplingTime = drv2605L_LRASamplingTime_300us,
	.ctrlReg2.b.bBrakeStabilizer = false,
	.ctrlReg2.b.bBidirectional = false,

	// ControL Register 3 Parameters
	.ctrlReg3.b.bLRAOpenLoop = false,
	.ctrlReg3.b.bInputTriggerMode = drv2605L_inputTriggerMode_PWM,
	.ctrlReg3.b.bLRADriveMode = drv2605L_LRADriveMode_oncePerCycle,
	.ctrlReg3.b.bRTPDataFormat = drv2605L_RTPDataFormat_signed,
	.ctrlReg3.b.bDisableSupplyComp = false,
	.ctrlReg3.b.bERMOpenLoop = false,
	.ctrlReg3.b.bNoiseGateThresh = drv2605L_noiseGateThresh_off,

	// Control Register 4 Parameters
	.ctrlReg4.b.bOTPProgram = false,
	.ctrlReg4.b.bAutoCalTime = drv2605L_autoCalTime_500ms,

	// Control Register 5 Parameters
	.ctrlReg5.b.bCurrDissTimeMSB = FOURBIT_UPPERHALF(1),
	.ctrlReg5.b.bBlankingTimeMSB = FOURBIT_UPPERHALF(1),
	.ctrlReg5.b.bPlaybackInterval = drv2605L_playbackInterval_5ms,
	.ctrlReg5.b.bLRAAutoOpenLoop = false,
	.ctrlReg5.b.bAutoOpenLoopCnt = drv2605L_autoOpenLoopAttempts_4,
};




//! \brief Default Actuator Configuration: 306-10H (Precision Microdrives) [ERM]
//!
const drv2605L_Actuator_t drv2605L_actuator_ERM30610H =
{
    // Basic Parameters
	.ratedVoltage.r = 0,
	.overdriveClampVoltage.r = 139,

	// Auto-Calibration Parameters
	.autoCal_Compensation.r = 0x0C,
	.autoCal_BackEMF.r = 0x71,

	// Feedback Control Parameters
	.feedbackCtrl.b.bBackEMFGain = drv2605L_backEMFgain_2,
	.feedbackCtrl.b.bLoopGain = drv2605L_loopGain_medium,
	.feedbackCtrl.b.bFBBrakeFactor = drv2605L_feedbackBrakeFactor_3x,
	.feedbackCtrl.b.bActuatorType = drv2605L_actuator_ERM,

	// Control Register 1 Parameters
	.ctrlReg1.b.bDriveTime = 0x13,
	.ctrlReg1.b.bACCoupling = false,
	.ctrlReg1.b.bStartupBoost = true,

	// Control Register 2 Parameters
	.ctrlReg2.b.bCurrDissTimeLSB = FOURBIT_LOWERHALF(1),
	.ctrlReg2.b.bBlankingTimeLSB = FOURBIT_LOWERHALF(1),
	.ctrlReg2.b.bSamplingTime = drv2605L_LRASamplingTime_250us,
	.ctrlReg2.b.bBrakeStabilizer = false,
	.ctrlReg2.b.bBidirectional = false,

	// ControL Register 3 Parameters
	.ctrlReg3.b.bLRAOpenLoop = false,
	.ctrlReg3.b.bInputTriggerMode = drv2605L_inputTriggerMode_PWM,
	.ctrlReg3.b.bLRADriveMode = drv2605L_LRADriveMode_oncePerCycle,
	.ctrlReg3.b.bRTPDataFormat = drv2605L_RTPDataFormat_signed,
	.ctrlReg3.b.bDisableSupplyComp = false,
	.ctrlReg3.b.bERMOpenLoop = true,
	.ctrlReg3.b.bNoiseGateThresh = drv2605L_noiseGateThresh_off,

	// Control Register 4 Parameters
	.ctrlReg4.b.bOTPProgram = false,
	.ctrlReg4.b.bAutoCalTime = drv2605L_autoCalTime_500ms,

	// Control Register 5 Parameters
	.ctrlReg5.b.bCurrDissTimeMSB = FOURBIT_UPPERHALF(1),
	.ctrlReg5.b.bBlankingTimeMSB = FOURBIT_UPPERHALF(1),
	.ctrlReg5.b.bPlaybackInterval = drv2605L_playbackInterval_5ms,
	.ctrlReg5.b.bLRAAutoOpenLoop = false,
	.ctrlReg5.b.bAutoOpenLoopCnt = drv2605L_autoOpenLoopAttempts_4,
};



//! \brief Default Actuator Configuration: 0825 (Mplus) [LRA]
//!
const drv2605L_Actuator_t drv2605L_actuator_LRA0825 =
{
    // Basic Parameters
	.ratedVoltage.r = 0x2F,
	.overdriveClampVoltage.r = 0x59,

	// Auto-Calibration Parameters
	.autoCal_Compensation.r = 0x0C,
	.autoCal_BackEMF.r = 0x71,

	// Feedback Control Parameters
	.feedbackCtrl.b.bBackEMFGain = drv2605L_backEMFgain_3,
	.feedbackCtrl.b.bLoopGain = drv2605L_loopGain_medium,
	.feedbackCtrl.b.bFBBrakeFactor = drv2605L_feedbackBrakeFactor_3x,
	.feedbackCtrl.b.bActuatorType = drv2605L_actuator_LRA,

	// Control Register 1 Parameters
	.ctrlReg1.b.bDriveTime = 0,
	.ctrlReg1.b.bACCoupling = false,
	.ctrlReg1.b.bStartupBoost = true,

	// Control Register 2 Parameters
	.ctrlReg2.b.bCurrDissTimeLSB = FOURBIT_LOWERHALF(1),
	.ctrlReg2.b.bBlankingTimeLSB = FOURBIT_LOWERHALF(1),
	.ctrlReg2.b.bSamplingTime = drv2605L_LRASamplingTime_300us,
	.ctrlReg2.b.bBrakeStabilizer = false,
	.ctrlReg2.b.bBidirectional = false,

	// ControL Register 3 Parameters
	.ctrlReg3.b.bLRAOpenLoop = false,
	.ctrlReg3.b.bInputTriggerMode = drv2605L_inputTriggerMode_PWM,
	.ctrlReg3.b.bLRADriveMode = drv2605L_LRADriveMode_oncePerCycle,
	.ctrlReg3.b.bRTPDataFormat = drv2605L_RTPDataFormat_signed,
	.ctrlReg3.b.bDisableSupplyComp = false,
	.ctrlReg3.b.bERMOpenLoop = false,
	.ctrlReg3.b.bNoiseGateThresh = drv2605L_noiseGateThresh_off,

	// Control Register 4 Parameters
	.ctrlReg4.b.bOTPProgram = false,
	.ctrlReg4.b.bAutoCalTime = drv2605L_autoCalTime_500ms,

	// Control Register 5 Parameters
	.ctrlReg5.b.bCurrDissTimeMSB = FOURBIT_UPPERHALF(1),
	.ctrlReg5.b.bBlankingTimeMSB = FOURBIT_UPPERHALF(1),
	.ctrlReg5.b.bPlaybackInterval = drv2605L_playbackInterval_5ms,
	.ctrlReg5.b.bLRAAutoOpenLoop = false,
	.ctrlReg5.b.bAutoOpenLoopCnt = drv2605L_autoOpenLoopAttempts_4,
};







