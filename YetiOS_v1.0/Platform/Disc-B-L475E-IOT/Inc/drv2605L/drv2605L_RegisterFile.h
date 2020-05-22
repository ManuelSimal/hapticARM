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
#ifndef drv2605L_REGISTERFILE_H_
#define drv2605L_REGISTERFILE_H_
//#############################################################################
//
//! \file   drv2605L_RegisterFile.h
//!
//! \brief The drv2605L_RegisterFile header file contains an enumeration of the
//!        DRV26xx register file, as well as bit field definitions for each 
//!        register where appropriate.  This can be thought of as the 
//!        definition of the DRV26xx I2C register map.
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

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Definitions
//*****************************************************************************

//! \brief A helpful reminder that the bit field is read only.
//!
#define __R

//! \brief A helpful reminder that the bit field is write only.
//!
#define __W

//! \brief A helpful reminder that the bit field is read/write.
//!
#define __RW

//*****************************************************************************
// Type Definitions
//*****************************************************************************

//! \brief An enumeration of the tdrv2605L I2C register file.
//!
typedef enum
{
	drv2605L_reg_status = 0x00,
	drv2605L_reg_mode,
	drv2605L_reg_realTimePlaybackInput,
	drv2605L_reg_librarySelection,
	drv2605L_reg_waveformSequence1,
	drv2605L_reg_waveformSequence2,
	drv2605L_reg_waveformSequence3,
	drv2605L_reg_waveformSequence4,
	drv2605L_reg_waveformSequence5,
	drv2605L_reg_waveformSequence6,
	drv2605L_reg_waveformSequence7,
	drv2605L_reg_waveformSequence8,
	drv2605L_reg_go,
	drv2605L_reg_overdriveTimeOffset,
	drv2605L_reg_sustainTimePositiveOffset,
	drv2605L_reg_sustainTimeNegativeOffset,
	drv2605L_reg_brakeTimeOffset,
	drv2605L_reg_audioToVibeControl,
	drv2605L_reg_audioToVibeMinInputLevel,
	drv2605L_reg_audioToVibeMaxInputLevel,
	drv2605L_reg_audioToVibeMinOutputDrive,
	drv2605L_reg_audioToVibeMaxOutputDrive,
	drv2605L_reg_ratedVoltage,
	drv2605L_reg_overdriveClampVoltage,
	drv2605L_reg_autoCalibrationCompResult,
	drv2605L_reg_autoCalibrationBackEMFResult,
	drv2605L_reg_feedbackControl,
	drv2605L_reg_control1,
	drv2605L_reg_control2,
	drv2605L_reg_control3,
	drv2605L_reg_control4,
	drv2605L_reg_control5,
	drv2605L_reg_LRAOpenLoopPeriodReg,
	drv2605L_reg_voltageMonitorReg,
	drv2605L_reg_LRAResonancePeriodReg
} drv2605L_Register_t;

//! \brief STATUS REGISTER (0x00) (drv2605L_reg_status)
//!
typedef union {
    __R  uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __R  uint8_t bOverCurrent         :	1;  /* Over Current Flag */
      __R  uint8_t bOverTemp            :	1;  /* Over Temperature Flag */
      __R  uint8_t bRESERVED0           :	1;  /* Reserved */
      __R  uint8_t bDiagResult          :   1;  /* Diagnostic Result */
      __R  uint8_t bRESERVED1           :	1;  /* Reserved */
      __R  uint8_t bDeviceID            :	3;  /* Device ID */
    } b;
} drv2605L_StatusReg_t;

typedef enum {
	drv2605L_deviceID_DRV2604,
	drv2605L_deviceID_drv2605,
	drv2605L_deviceID_DRV2604L,
	drv2605L_deviceID_drv2605L
} drv2605L_DeviceID_t;

//! \brief MODE REGISTER (0x01) (drv2605L_reg_mode)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bMode                :	3;  /* Mode */
      __R  uint8_t bRESERVED0           :	3;  /* Reserved */
      __RW uint8_t bStandby             :   1;  /* Standby */
      __RW uint8_t bDeviceReset         :	1;  /* Reset */
    } b;
} drv2605L_ModeReg_t;

//! \brief Mode bit options
//!
typedef enum {
	drv2605L_mode_internalTrigger = 0,
	drv2605L_mode_externalEdgeTrigger,
	drv2605L_mode_externalLevelTrigger,
	drv2605L_mode_PWMandAnalog,
	drv2605L_mode_audioToVibe,
	drv2605L_mode_realtimePlayback,
	drv2605L_mode_diagnostics,
	drv2605L_mode_autoCalibration
} drv2605L_Mode_t;

//! \brief REALTIME PLAYBACK INPUT REGISTER (0x02) (drv2605L_reg_realTimePlaybackInput)
//!
//! Insert RTP Data directly into the data field at this address
//! Open Loop (ERM Only), BiDir_Input = N/A, SignedUnsigned_RTP = 0
//! 0x7F, Full scale positive (forward rotation)
//! 0x00, No signal
//! 0x81, Full scale negative (reverse rotation or braking)
//!
//! Open Loop (ERM Only), BiDir_Input = N/A, SignedUnsigned_RTP = 1
//! 0xFF, Full scale positive (forward rotation)
//! 0x80, No signal
//! 0x00, Full scale negative (reverse rotation or braking)
//!
//! Closed Loop, BiDir_Input = 1, SignedUnsigned_RTP = 0
//! 0x7F, Full scale drive
//! Less than 0x00 interpreted as 0x81 (full-scale brake)
//!
//! Closed Loop, BiDir_Input = 1, SignedUnsigned_RTP = 1
//! 0xFF, Full scale drive
//! Less than 0x7F interpreted as 0x00 (full-scale brake)
//!
//! Closed Loop, BiDir_Input = 0, SignedUnsigned_RTP = N/A
//! 0xFF, Full scale drive
//! 0x80, Half scale drive
//! 0x00, No signal, braking happens when input is less than back-EMF
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bRTPInput            :	8;  /* Realtime Playback Input */
    } b;
} drv2605L_RealtimePlaybackInputReg_t;

//! \brief LIBRARY SELECTION REGISTER (0x03) (drv2605L_reg_librarySelection)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bLibrarySelection    :	3;  /* Library Selection */
      __R  uint8_t bRESERVED0           :	1;  /* Reserved */
      __RW uint8_t bHighZ               :   1;  /* High-Z */
      __R  uint8_t bRESERVED1           :	3;  /* Reserved */
    } b;
} drv2605L_LibSelectReg_t;

//! \brief Library field options.
//!
typedef enum
{
	drv2605L_lib_RAM = 0,
	drv2605L_lib_ROM_A,
	drv2605L_lib_ROM_B,
	drv2605L_lib_ROM_C,
	drv2605L_lib_ROM_D,
	drv2605L_lib_ROM_E,
	drv2605L_lib_ROM_LRA,
	drv2605L_lib_ROM_F
} drv2605L_Library_t;

//! \brief WAVEFORM SEQUENCE REGISTERS  (0x04 to 0x0B) (eWaveformSequenceXReg)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bParam              :	7;  /* Effect or Delay */
      __RW uint8_t bDelay              :	1;  /* Set as Delay */
    } b;
} drv2605L_WaveformSeqReg_t;

//! \brief GO REGISTER (0x0C) (drv2605L_reg_go)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bGo                 :	1;  /* Go Bit */
      __R  uint8_t bRESERVED0          :	7;  /* Reserved */
    } b;
} drv2605L_GoReg_t;

//! \brief OVERDRIVE TIME OFFSET REGISTER (0x0D) (drv2605L_reg_overdriveTimeOffset)
//!        OverDrive Time Offset (ms) = ODT[7:0] * 5
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bOvdrTimeOffset     :	8;  /* Overdrive Time Offset */
    } b;
} drv2605L_OvdrTimeOffsetReg_t;

//! \brief SUSTAIN TIME OFFSET POSITIVE REGISTER (0x0E) 
//!        (drv2605L_reg_sustainTimePositiveOffset)
//!        Sustain Time Positive Offset (ms) = SPT[7:0] * 5
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bSustTimePosOffset  :	8;  /* Sustain Time Pos Offset */
    } b;
} drv2605L_SustTimePosOffsetReg_t;

//! \brief SUSTAIN TIME OFFSET NEGATIVE REGISTER (0x0F) 
//!        (drv2605L_reg_sustainTimeNegativeOffset)
//!        Sustain Time Negative Offset (ms) = SNT[7:0] * 5
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bSustTimeNegOffset  :	8;  /* Sustain Time Neg Offset */
    } b;
} drv2605L_SustTimeNegOffsetReg_t;

//! \brief BRAKE TIME OFFSET REGISTER (0x10) (drv2605L_reg_brakeTimeOffset)
//!        Braking Time Offset (ms) = BRT[7:0] * 5
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bBrakeTimeOffset    :	8;  /* Brake Time Offset */
    } b;
} drv2605L_BrakeTimeOffsetReg_t;

//! \brief RATED VOLTAGE REGISTER (0x16) (drv2605L_reg_ratedVoltage)
//!        Rated Voltage (V) = RatedVoltage[7:0] / 255 * 5.3 V
//!        User should write this value before performing autocal
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bRatedVoltage       :	8;  /* Rated Voltage */
    } b;
} drv2605L_RatedVoltageReg_t;

//! \brief OVERDRIVE CLAMP VOLTAGE REGISTER (0x17) (drv2605L_reg_overdriveClampVoltage)
//!        Overdrive Clamp Voltage (V) = ODClamp[7:0] / 255 * 5.6 V
//!        User should write this value before performing autocal
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bOvdrClampVoltage   :	8;  /* Overdrive Clamp Voltage */
    } b;
} drv2605L_OvdrClampVoltageReg_t;

//! \brief AUTOCAL COMPENSATION RESULT REGISTER (0x18)
//!        (drv2605L_reg_autoCalibrationCompResult)
//!        Calibration Compensation Coefficient = 1 + ACalComp[7:0] / 255
//!        The AutoCal routine will populate this register
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bAutocalCompResult  :	8;  /* Auto-cal Compensation */
    } b;
} drv2605L_AutocalCompResultReg_t;

//! \brief AUTOCAL BACK EMF RESULT REGISTER (0x19)
//!        (drv2605L_reg_autoCalibrationBackEMFResult)
//!        Auto Calibration Back-EMF Coefficient = (Value/255)*2.44/BEMFGain
//!        The AutoCal routine will populate this register
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bAutocalBackEMFResult :	8;  /* Auto-cal Back EMF */
    } b;
} drv2605L_AutocalBackEMFResultReg_t;

//! \brief FEEDBACK CONTROL REGISTER (0x1A) (drv2605L_reg_feedbackControl)
//! 
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bBackEMFGain        :	2;  /* Back EMF Gain */
      __RW uint8_t bLoopGain           :	2;  /* Loop Gain */
      __RW uint8_t bFBBrakeFactor      :	3;  /* Feedback Brake Factor */
      __RW uint8_t bActuatorType       :	1;  /* ERM or LRA */
    } b;
} drv2605L_FeedbackCtrlReg_t;

//! \brief Back EMF options
//!
typedef enum
{
	drv2605L_backEMFgain_0 = 0,
	drv2605L_backEMFgain_1,
	drv2605L_backEMFgain_2,
	drv2605L_backEMFgain_3
} drv2605L_BackEMFGain_t;

//! \brief Loop Gain options
//!
typedef enum
{
	drv2605L_loopGain_slow = 0,
	drv2605L_loopGain_medium,
	drv2605L_loopGain_fast,
	drv2605L_loopGain_veryfast
} drv2605L_LoopGain_t;

//! \brief Feedback Brake Factor options
//!
typedef enum
{
	drv2605L_feedbackBrakeFactor_1x = 0,
	drv2605L_feedbackBrakeFactor_2x,
	drv2605L_feedbackBrakeFactor_3x,
	drv2605L_feedbackBrakeFactor_4x,
	drv2605L_feedbackBrakeFactor_6x,
	drv2605L_feedbackBrakeFactor_8x,
	drv2605L_feedbackBrakeFactor_16x,
	drv2605L_feedbackBrakeFactor_disabled
} drv2605L_FeedbackBrakeFactor_t;

//! \brief Actuator options
//!
typedef enum
{
	drv2605L_actuator_ERM = 0,
	drv2605L_actuator_LRA
} drv2605L_Actuator;

//! \brief CONTROL REGISTER 1 (0x1B) (drv2605L_reg_control1)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bDriveTime          :	5;  /* Drive Time */
      __RW uint8_t bACCoupling         :	1;  /* AC Coupling */
      __RW uint8_t bRESERVED0          :	1;  /* Reserved */
      __RW uint8_t bStartupBoost       :	1;  /* Startup Boost */
    } b;
} drv2605L_Ctrl1Reg_t;

//! \brief CONTROL REGISTER 2 (0x1C) (drv2605L_reg_control2)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bCurrDissTimeLSB    :	2;  /* I Dissapation Time */
      __RW uint8_t bBlankingTimeLSB    :	2;  /* Blanking Time */
      __RW uint8_t bSamplingTime         :	2;  /* Sample Time */
      __RW uint8_t bBrakeStabilizer    :	1;  /* Brake Stabilizer */
      __RW uint8_t bBidirectional      :	1;  /* Bidirectional Input */
    } b;
} drv2605L_Ctrl2Reg_t;

//! \brief Sampling Time options
//!
typedef enum
{
	drv2605L_LRASamplingTime_150us = 0,
	drv2605L_LRASamplingTime_200us,
	drv2605L_LRASamplingTime_250us,
	drv2605L_LRASamplingTime_300us
} drv2605L_LRASampleTime_t;

//! \brief CONTROL REGISTER 3 (0x1D) (drv2605L_reg_control3)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bLRAOpenLoop        :	1;  /* Open Loop LRA */
      __RW uint8_t bInputTriggerMode   :	1;  /* PWM or Analog Input Mode */
      __RW uint8_t bLRADriveMode       :	1;  /* LRA Drive Mode */
      __RW uint8_t bRTPDataFormat      :	1;  /* Realtime Playback Format */
      __RW uint8_t bDisableSupplyComp  :	1;  /* Supply Compensation */
      __RW uint8_t bERMOpenLoop        :	1;  /* Open Loop ERM */
      __RW uint8_t bNoiseGateThresh    :	2;  /* Noise Gate Thresh */
    } b;
} drv2605L_Ctrl3Reg_t;

//! \brief Input Trigger options
//!
typedef enum
{
	drv2605L_inputTriggerMode_PWM = 0,
	drv2605L_inputTriggerMode_analog
} drv2605L_InputTriggerMode_t;

//! \brief LRA Drive Mode options
//!
typedef enum
{
	drv2605L_LRADriveMode_oncePerCycle = 0,
	drv2605L_LRADriveMode_twicePerCycle
} drv2605L_LRADriveMode_t;

//! \brief RTP Data Format options
//!
typedef enum
{
	drv2605L_RTPDataFormat_signed = 0,
	drv2605L_RTPDataFormat_unsigned
} drv2605L_RTPDataFormat_t;

//! \brief Noise Gate Threshold Options
//!
typedef enum
{
	drv2605L_noiseGateThresh_off = 0,
	drv2605L_noiseGateThresh_2percent,
	drv2605L_noiseGateThresh_4percent,
	drv2605L_noiseGateThresh_8percent
} drv2605L_NoiseGateThresh_t;

//! \brief CONTROL REGISTER 4 (0x1E) (drv2605L_reg_control4)
//!        drv2605L__OTP_PROGRAM:
//!          Warning:  Only light this bit if you are happy with the settings
//!          VDD must be 4.2V +/- 5%
//!          This process can only be executed once
//!          Non-Volatile Memory will be burned for Registers 0x16 through 0x1A
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bOTPProgram         :	1;  /* Launch OTP Programming */
      __RW uint8_t bRESERVED0          :	1;  /* Reserved */
      __R  uint8_t bOTPStatus          :	1;  /* OTP Programming Status */
      __RW uint8_t bRESERVED1          :	1;  /* Reserved */
      __RW uint8_t bAutoCalTime        :	2;  /* Autocalibration Time */
      __RW uint8_t bZeroCrossTime      :	2;  /* Zero Cross Detect Time */
    } b;
} drv2605L_Ctrl4Reg_t;

//! \brief Auto Cal Time options
//!
typedef enum
{
	drv2605L_autoCalTime_150ms = 0,
	drv2605L_autoCalTime_250ms,
	drv2605L_autoCalTime_500ms,
	drv2605L_autoCalTime_1000ms
} drv2605L_AutoCalTime_t;

//! \brief Zero Crossing Time options
//!
typedef enum
{
	drv2605L_zeroCrossTime_100us = 0,
	drv2605L_zeroCrossTime_200us,
	drv2605L_zeroCrossTime_300us,
	drv2605L_zeroCrossTime_390us
} drv2605L_ZeroCrossTime_t;

//! \brief CONTROL REGISTER 5 (0x1F) (drv2605L_reg_control5)
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bCurrDissTimeMSB    :	2;  /* I Dissipation Time MSB*/
      __RW uint8_t bBlankingTimeMSB    :	2;  /* Blanking Time MSB */
      __RW uint8_t bPlaybackInterval   :	1;  /* Memory Playback Interval */
      __RW uint8_t bLRAAutoOpenLoop    :	1;  /* LRA Auto Open Loop */
      __RW uint8_t bAutoOpenLoopCnt    :	2;  /* Auto Open-Loop Attempts */
    } b;
} drv2605L_Ctrl5Reg_t;

//! \brief Playback Interval options
//!
typedef enum
{
	drv2605L_playbackInterval_5ms = 0,
	drv2605L_playbackInterval_1ms,
} drv2605L_PlaybackInterval_t;

//! \brief Auto Open Loop Attempt options
//!
typedef enum
{
	drv2605L_autoOpenLoopAttempts_3 = 0,
	drv2605L_autoOpenLoopAttempts_4,
	drv2605L_autoOpenLoopAttempts_5,
	drv2605L_autoOpenLoopAttempts_6
} drv2605L_AutoOpenLoopCnt_t;

//! \brief LRA OPEN LOOP PERIOD REGISTER (0x20) (drv2605L_reg_LRAOpenLoopPeriodReg)
//!        Period (us) = OL_LRA_Period[7:0] * 98.46
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bLRAOpenLoopPeriod  :	8;  /* LRA Open Loop Period*/
    } b;
} drv2605L_LRAOpenLoopReg_t;

//! \brief LRA Open Loop Period options
//!
typedef enum
{
	drv2605L_autoOpenLoopPeriod_240Hz = 42,
	drv2605L_autoOpenLoopPeriod_230Hz = 44,
	drv2605L_autoOpenLoopPeriod_220Hz = 46,
	drv2605L_autoOpenLoopPeriod_210Hz = 48,
	drv2605L_autoOpenLoopPeriod_205Hz = 50,
	drv2605L_autoOpenLoopPeriod_175Hz = 58
} drv2605L_LRAOpenLoopPeriod_t;

//! \brief VBAT VOLTAGE MONITOR REGISTER (0x21) (drv2605L_reg_voltageMonitorReg)
//!        VDD (V) = VBAT[7:0] * 5.6V / 255	--Read Only
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bVBatVoltage        :	8;  /* VBat Voltage */
    } b;
} drv2605L_VBatMonitorReg_t;

//! \brief LRA RESONANCE PERIOD REGISTER (0x22) (drv2605L_reg_LRAOpenLoopPeriodReg)
//!        LRA Period (us) = LRA_Period[7:0] * 196.2 us
//!
typedef union {
    __RW uint8_t r;                             /* Complete Register */
    struct {                                    /* Register Bits */
      __RW uint8_t bLRAResonancePeriod :	8;  /* LRA Resonance Period */
    } b;
} drv2605L_LRAResonancePeriodReg_t;

#endif /* drv2605L_REGISTERFILE_H_ */
