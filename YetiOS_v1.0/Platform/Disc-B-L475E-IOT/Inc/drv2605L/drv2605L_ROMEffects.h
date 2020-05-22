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
#ifndef DRV2605L_ROMEFFECTS_H_
#define DRV2605L_ROMEFFECTS_H_
//#############################################################################
//
//! \file   drv2605L_effect_ROMEffects.h
//!
//! \brief The drv2605Leffect_ROMEffects header file contains an enumeration of the
//!        DRV2605L ROM effects ID's, so that they may be accessed by name.
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
// Type Definitions
//*****************************************************************************

//! \brief Haptic effect options
//!
typedef enum
{
	// Playlist Stop
	drv2605L_effect_stop = 0x00,

	// Strong Clicks
	drv2605L_effect_strongClick_100P,
	drv2605L_effect_strongClick_60P,
	drv2605L_effect_strongClick_30P,

	// Sharp Clicks
	drv2605L_effect_sharpClick_100P,
	drv2605L_effect_sharpClick_60P,
	drv2605L_effect_sharpClick_30P,

	// Soft Bumps
	drv2605L_effect_softBump_100P,
	drv2605L_effect_softBump_60P,
	drv2605L_effect_softBump_30P,

	// Double/Triple Clicks
	drv2605L_effect_doubleClick_100P,
	drv2605L_effect_doubleClick_60P,
	drv2605L_effect_tripleClick_100P,

	// Soft Fuzz
	drv2605L_effect_softFuzz_100P,

	// Strong Buzz
	drv2605L_effect_strongBuzz_100P,

	// Alerts
	drv2605L_effect_750msAlert_100P,
	drv2605L_effect_1sAlert_100P,

	// Strong Clicks (Expanded)
	drv2605L_effect_strongClick1_100P,
	drv2605L_effect_strongClick2_80P,
	drv2605L_effect_strongClick3_60P,
	drv2605L_effect_strongClick4_30P,

	// Medium Clicks
	drv2605L_effect_mediumClick1_100P,
	drv2605L_effect_mediumClick2_80P,
	drv2605L_effect_mediumClick3_60P,

	// Sharp Ticks
	drv2605L_effect_sharpTick1_100P,
	drv2605L_effect_sharpTick2_80P,
	drv2605L_effect_sharpTick3_60P,

	// Short Double Clicks (Strong)
	drv2605L_effect_shortDoubleClickStrong1_100P,
	drv2605L_effect_shortDoubleClickStrong2_80P,
	drv2605L_effect_shortDoubleClickStrong3_60P,
	drv2605L_effect_shortDoubleClickStrong4_30P,

	// Short Double Clicks (Medium)
	drv2605L_effect_shortDoubleClickMedium1_100P,
	drv2605L_effect_shortDoubleClickMedium2_80P,
	drv2605L_effect_shortDoubleClickMedium3_60P,

	// Short Double Sharp Ticks
	drv2605L_effect_shortDoubleSharpTick1_100P,
	drv2605L_effect_shortDoubleSharpTick2_80P,
	drv2605L_effect_shortDoubleSharpTick3_60P,

	// Long Double Sharp Clicks (Strong)
	drv2605L_effect_longDoubleSharpClickStrong1_100P,
	drv2605L_effect_longDoubleSharpClickStrong2_80P,
	drv2605L_effect_longDoubleSharpClickStrong3_60P,
	drv2605L_effect_longDoubleSharpClickStrong4_30P,

	// Long Double Sharp Clicks (Medium)
	drv2605L_effect_longDoubleSharpClickMedium1_100P,
	drv2605L_effect_longDoubleSharpClickMedium2_100P,
	drv2605L_effect_longDoubleSharpClickMedium3_60P,

	// Long Double Sharp Ticks
	drv2605L_effect_longDoubleSharpTick1_100P,
	drv2605L_effect_longDoubleSharpTick2_80P,
	drv2605L_effect_longDoubleSharpTick3_60P,

	// Buzzes
	drv2605L_effect_buzz1_100P,
	drv2605L_effect_buzz2_80P,
	drv2605L_effect_buzz3_60P,
	drv2605L_effect_buzz4_40P,
	drv2605L_effect_buzz5_20P,

	// Strong Pulses
	drv2605L_effect_pulsingStrong1_100P,
	drv2605L_effect_pulsingStrong2_60P,

	// Medium Pulses
	drv2605L_effect_pulsingMedium1_100P,
	drv2605L_effect_pulsingMedium2_60P,

	// Sharp Pulses
	drv2605L_effect_pulsingSharp1_100P,
	drv2605L_effect_pulsingSharp2_60P,

	// Transition Clicks
	drv2605L_effect_transitionClick1_100P,
	drv2605L_effect_transitionClick2_80P,
	drv2605_effect_transitionClick3_60P,
	drv2605L_effect_transitionClick4_40P,
	drv2605L_effect_transitionClick5_20P,
	drv2605L_effect_transitionClick6_10P,

	// Transition Hums
	drv2605L_effect_transitionHum1_100P,
	drv2605L_effect_transitionHum2_80P,
	drv2605L_effect_transitionHum3_60P,
	drv2605L_effect_transitionHum4_40P,
	drv2605L_effect_transitionHum5_20P,
	drv2605L_effect_transitionHum6_10P,

	// Transition Ramp Downs (Smooth) Full Scale
	drv2605L_effect_transitionRampDownLongSmooth1_100Pto0P,
	drv2605L_effect_transitionRampDownLongSmooth2_100Pto0P,
	drv2605L_effect_transitionRampDownMediumSmooth1_100Pto0P,
	drv2605L_effect_transitionRampDownMediumSmooth2_100Pto0P,
	drv2605L_effect_transitionRampDownShortSmooth1_100Pto0P,
	drv2605L_effect_transitionRampDownShortSmooth2_100Pto0P,

	// Transition Ramp Downs (Sharp) Full Scale
	drv2605L_effect_transitionRampDownLongSharp1_100Pto0P,
	drv2605L_effect_transitionRampDownLongSharp2_100Pto0P,
	drv2605L_effect_transitionRampDownMediumSharp1_100Pto0P,
	drv2605L_effect_transitionRampDownMediumSharp2_100Pto0P,
	drv2605L_effect_transitionRampDownShortSharp1_100Pto0P,
	drv2605L_effect_transitionRampDownShortSharp2_100Pto0P,

	// Transition Ramp Ups (Smooth) Full Scale
	drv2605L_effect_transitionRampUpLongSmooth1_0Pto100P,
	drv2605L_effect_transitionRampUpLongSmooth2_0Pto100P,
	drv2605L_effect_transitionRampUpMediumSmooth1_0Pto100P,
	drv2605L_effect_transitionRampUpMediumSmooth2_0Pto100P,
	drv2605L_effect_transitionRampUpShortSmooth1_0Pto100P,
	drv2605L_effect_transitionRampUpShortSmooth2_0Pto100P,

	// Transition Ramp Ups (Sharp) Full Scale
	drv2605L_effect_transitionRampUpLongSharp1_0Pto100P,
	drv2605L_effect_transitionRampUpLongSharp2_0Pto100P,
	drv2605L_effect_transitionRampUpMediumSharp1_0Pto100P,
	drv2605L_effect_transitionRampUpMediumSharp2_0Pto100P,
	drv2605L_effect_transitionRampUpShortSharp1_0Pto100P,
	drv2605L_effect_transitionRampUpShortSharp2_0Pto100P,

	// Transition Ramp Downs (Smooth) Half Scale
	drv2605L_effect_transitionRampDownLongSmooth1_50Pto0P,
	drv2605L_effect_transitionRampDownLongSmooth2_50Pto0P,
	drv2605L_effect_transitionRampDownMediumSmooth1_50Pto0P,
	drv2605L_effect_transitionRampDownMediumSmooth2_50Pto0P,
	drv2605L_effect_transitionRampDownShortSmooth1_50Pto0P,
	drv2605L_effect_transitionRampDownShortSmooth2_50Pto0P,

	// Transition Ramp Downs (Sharp) Half Scale
	drv2605L_effect_transitionRampDownLongSharp1_50Pto0P,
	drv2605L_effect_transitionRampDownLongSharp2_50Pto0P,
	drv2605L_effect_transitionRampDownMediumSharp1_50Pto0P,
	drv2605L_effect_transitionRampDownMediumSharp2_50Pto0P,
	drv2605L_effect_transitionRampDownShortSharp1_50Pto0P,
	drv2605L_effect_transitionRampDownShortSharp2_50Pto0P,

	// Transition Ramp Ups (Smooth) Half Scale
	drv2605L_effect_transitionRampUpLongSmooth1_0Pto50P,
	drv2605L_effect_transitionRampUpLongSmooth2_0Pto50P,
	drv2605L_effect_transitionRampUpMediumSmooth1_0Pto50P,
	drv2605L_effect_transitionRampUpMediumSmooth2_0Pto50P,
	drv2605L_effect_transitionRampUpShortSmooth1_0Pto50P,
	drv2605L_effect_transitionRampUpShortSmooth2_0Pto50P,

	// Transition Ramp Ups (Sharp) Half Scale
	drv2605L_effect_transitionRampUpLongSharp1_0Pto50P,
	drv2605L_effect_transitionRampUpLongSharp2_0Pto50P,
	drv2605L_effect_transitionRampUpMediumSharp1_0Pto50P,
	drv2605L_effect_transitionRampUpMediumSharp2_0Pto50P,
	drv2605L_effect_transitionRampUpShortSharp1_0Pto50P,
	drv2605L_effect_transitionRampUpShortSharp2_0Pto50P,

	// Long Buzz for Programmatic Stopping
	drv2605L_effect_LongBuzzForProgrammaticstopping,

	// Smooth Hums (No Kick or Brake Pulses)
	drv2605L_effect_smoothHum1_50P,
	drv2605L_effect_smoothHum2_40P,
	drv2605L_effect_smoothHum3_30P,
	drv2605L_effect_smoothHum4_20P,
	drv2605L_effect_smoothHum5_10P

} drv2605L_Effect_t;

#endif /* DRV2605L_ROMEFFECTS_H_ */
