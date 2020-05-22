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
#ifndef drv2605L_ACTUATORS_H_
#define drv2605L_ACTUATORS_H_
//#############################################################################
//
//! \file   drv2605L_Actuators.h
//!
//! \brief The drv2605L_Actuators layer contains pre-populated drv2605L_Actuator_t
//!        containers for known, existing, haptic actuators.
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
#include "drv2605L_RegisterFile.h"

//*****************************************************************************
// Type Definitions
//*****************************************************************************

//! \brief drv2605L_Actuator_t defines a container for all of the properties
//!        associated with a particular haptic actuator (ERM or LRA).  The 
//!        structure is composed of tdrv2605L_Register instances, to allow for
//!        efficient reading and writing to the driver IC via I2C.  These 
//!        structures are used to configure the DRV26x driver for a specific 
//!        actuator.
//!        Default,out-of-box driver configurations are provided in the 
//!        drv2605L_Actuators.c/.h files.  If auto-calibration data does not
//!        need to be read back, instances of drv2605L_Actuator_t may be
//!        read-only (const), saving RAM.  This is the default configuration 
//!        of the drv2605L_Acuators layer.
//!
typedef union
{
	uint8_t actuatorData[10]; /* Register Array (Sequential) */
	struct /* Individual Register Access */
	{
		drv2605L_RatedVoltageReg_t ratedVoltage;
		drv2605L_OvdrClampVoltageReg_t overdriveClampVoltage;
		drv2605L_AutocalCompResultReg_t autoCal_Compensation;
		drv2605L_AutocalBackEMFResultReg_t autoCal_BackEMF;
		drv2605L_FeedbackCtrlReg_t feedbackCtrl;
		drv2605L_Ctrl1Reg_t ctrlReg1;
		drv2605L_Ctrl2Reg_t ctrlReg2;
		drv2605L_Ctrl3Reg_t ctrlReg3;
		drv2605L_Ctrl4Reg_t ctrlReg4;
		drv2605L_Ctrl5Reg_t ctrlReg5;
	};
} drv2605L_Actuator_t;

//*****************************************************************************
// Global Variables
//*****************************************************************************

//! \brief Default Actuator Configuration: 0934W (Mplus) [LRA]
//!
extern const drv2605L_Actuator_t drv2605L_actuator_LRA0934W;


//! \brief Default Actuator Configuration: 306-10H (Precision Microdrives) [ERM]
//!
extern const drv2605L_Actuator_t drv2605L_actuator_ERM30610H;


//! \brief Default Actuator Configuration: 0825 (Mplus) [LRA]
//!
extern const drv2605L_Actuator_t drv2605L_actuator_LRA0825;



#endif /* drv2605L_ACTUATORS_H_ */
