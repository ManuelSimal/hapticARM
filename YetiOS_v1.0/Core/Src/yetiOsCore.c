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
 * yetiOsCore.c
 *
 *  Created on: 15 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file yetiOsCore.c
 */

#include "FreeRTOSConfig.h"
#include "yetiOS.h"
#include "yetiOsCore.h"
#include "process.h"
#include "platform.h"
#include "adaptEngine.h"
#include "netstack.h"
#include "timestamp.h"

static uint32_t currentCorePeriod;

#if YETIOS_STATUS_LED_PERIOD && YETIOS_STATUS_LED_TIME_ON
#ifdef PLATFORM_STATUS_LED
uint16_t ledOn = 0;
osTimerId statusLedTimerId;
static void statusLedTimerFunc(void const * argument);
#endif
#endif
extern void MX_FREERTOS_Init(void);
/**
 * @brief	Starts the operating system
 * @return
 */
retval_t ytCoreInitOS(){
	MX_FREERTOS_Init();

	if(osKernelStart() != osOK){
		return RET_ERROR;
	}

	return RET_OK;
}

#include "platformGpio.h"

/**
 * @brief			Core Task of YetiOS. It has the lowest priority.
 * @param argument
 */
void yetiosCoreTaskFunc(void const * argument)
{
	uint32_t statusLedTime = 0;
	currentCorePeriod = YETIOS_DEFAULT_CORE_PERIOD;
	/* Initialize low power manager*/
	ytInitLowPowerManager();

	/*Initialize GPIOs*/
	if(ytGpioInit() != RET_OK){
		while(1);
	}

	/*Initialize timestamp timer*/
	if(ytInitTimestamp() != RET_OK){
		while(1);
	}

	/*Initialize Time measurement timer*/
	if(ytInitTimeMeas() != RET_OK){
		while(1);
	}

	/*Initialize adaptive engine*/
#if YETIOS_ENABLE_ADAPTIVE_ENGINE
	ytInitAdaptEngine();
#endif

	/*Init device drivers*/
	if(ytInitDevices() != RET_OK){
		while(1);
	}


	/*Init Stdio*/
#if YETIOS_ENABLE_STDIO
	if(ytStdioInit(PLATFORM_DEFAULT_STDIO_INTERFACE) != RET_OK){
		while(1);
	}
#endif

	/*Init YetiShell*/
#if YETIOS_ENABLE_YETISHELL
	if(ytShellInit() != RET_OK){
		while(1);
	}
#endif
	/*Initialize netstack*/
#if	YETIOS_ENABLE_NETSTACK
	ytNetstackSetStartupLayers();
	ytNetstackInit();
#endif

	/* Initialize Processes*/
	ytAutoInitThreads();

	/*Initialize timer for the status led*/
#if YETIOS_STATUS_LED_PERIOD && YETIOS_STATUS_LED_TIME_ON
#ifdef PLATFORM_STATUS_LED
	statusLedTimerId = ytTimerCreate(osTimerOnce, statusLedTimerFunc, NULL);
	osTimerStart (statusLedTimerId, YETIOS_STATUS_LED_PERIOD);
#endif
#endif
	while(1){
		/*Refresh the watchdog */
		ytpWatchdogRefresh();

		/*Wait*/
		osDelay(currentCorePeriod);
		statusLedTime += currentCorePeriod;

	}
  /* USER CODE END StartDefaultTask */
}

#if YETIOS_STATUS_LED_PERIOD && YETIOS_STATUS_LED_TIME_ON
#ifdef PLATFORM_STATUS_LED
static void statusLedTimerFunc(void const * argument){
	if(ledOn){
		ledOn--;
		ytLedOff(PLATFORM_STATUS_LED);
		osTimerStart (statusLedTimerId, YETIOS_STATUS_LED_PERIOD-YETIOS_STATUS_LED_TIME_ON);
	}
	else{
		ledOn++;
		ytLedOn(PLATFORM_STATUS_LED);
		osTimerStart (statusLedTimerId, YETIOS_STATUS_LED_TIME_ON);
	}
}
#endif
#endif
/**
 * @brief	Watchdog refresh weak function. It should be defined by the platform
 */
__weak void ytpWatchdogRefresh(){
	return;
}
