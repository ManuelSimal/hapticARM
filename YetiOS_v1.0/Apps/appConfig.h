/*
 * Copyright (c) 2019, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the B105 Electronic Systems Lab.
 * 4. Neither the name of the B105 Electronic Systems Lab nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY UNIVERSITY AND CONTRIBUTORS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * appConfig.h
 *
 *  Created on: 14 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file appConfig.h
 */

#ifndef YETIOS_APPS_APPCONFIG_H_
#define YETIOS_APPS_APPCONFIG_H_

/* ONLY ONE APPLICATION MAY BE SELECTED SIMULTANEOUSLY */

/* Enable STDIO Example configuration*/
#define STDIO_EXAMPLE_APP	0
#if STDIO_EXAMPLE_APP
#include "stdioExampleConf.h"
#endif

/* Enable Basic YetiOS configuration*/
#define BASIC_YETIOS_EXAMPLE_APP	0
#if BASIC_YETIOS_EXAMPLE_APP
#include "yetiOsExampleConf.h"
#endif

/* Enable Test App configuration*/
#define TEST_APP				0
#if TEST_APP
#include "testAppConf.h"
#endif

/* Enable sreal's Test App configuration*/
#define TEST_APP_SRV			0
#if TEST_APP
#include "testAppConf.h"
#endif

/* Enable sreal's Test App configuration*/
#define TEST_APP_SRV_2			0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable sreal's Test App configuration*/
#define TEST_APP_MOCAP_UNITY	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable sreal's Test App configuration*/
#define TEST_APP_PWM			0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable sreal's Test App configuration*/
#define TEST_APP_WIRELESS_MOCAP_UNITY	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable msimal's Test App configuration*/
#define TEST_APP_HAPTICS	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable msimal's Test App configuration*/
#define TEST_APP_HAPTICS2	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable msimal's Test App configuration*/
#define TEST_ROTATION_V1	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable msimal's Test App configuration*/
#define TEST_ROTATION_V2	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable msimal's Test App configuration*/
#define TEST_APP_FINAL	1
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable msimal's Test App configuration*/
#define SERIAL_IMU_APPLICATION	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable msimal's Test App configuration*/
#define SERIAL_IMU_UNITY_2	0
#if TEST_APPggpqqp
#include "testAppConf.h"
#endif

/* Enable DBS Experiments Interface Device Application*/
#define DBS_EXP_INTERFACE_APP	0
#if DBS_EXP_INTERFACE_APP
#include "dbsExpInterfaceConf.h"
#endif

/* Enable DBS Experiments Device Under Test Application*/
#define DBS_EXP_DUT_APP			0
#if DBS_EXP_DUT_APP
#include "dbsExpDutConf.h"
#endif

/* Enable DBS Experiments Device Under Test Application*/
#define CURRENT_MEAS_APP		0
#if CURRENT_MEAS_APP
#include "currentMeasConf.h"
#endif

/* Enable UDP Example Application*/
#define UDP_EXAMPLE_APP			0
#if UDP_EXAMPLE_APP
#include "udpExampleConf.h"
#endif

/* Enable UDP Example Application*/
#define TCP_EXAMPLE_APP			0
#if TCP_EXAMPLE_APP
#include "tcpExampleConf.h"
#endif

/* Enable EMG Sampling Board Application*/
#define EMG_SAMPLING_APP		0
#if EMG_SAMPLING_APP
#include "emgBoardSamplingConf.h"
#endif

#ifndef APP_CUSTOM_CONFIG_DEFINED
#warning "No Custom config selected. Aplying default configuration values"
#endif


#endif /* YETIOS_APPS_APPCONFIG_H_ */
