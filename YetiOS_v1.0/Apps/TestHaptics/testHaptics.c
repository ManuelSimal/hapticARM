/*
 * Copyright (c) 2020, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * testHaptics.c
 *
 *  Created on: Mar 22, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */


#include "yetiOS.h"
#include "drv2605L.h"


#if TEST_APP_HAPTICS


static osThreadId testAppThreadHandle;

drv2605L_t* drv2605L;

static void testAppFunc(void const * argument);

void buttonCallback(void const * argument);

YtAutoInitThread(TestApp, testAppFunc, osPriorityLow, 256, &testAppThreadHandle, NULL);


static void testAppFunc(void const * argument){

	//GPIO setup
	ytGpioInitPin(GPIO_PIN_C13, GPIO_PIN_INTERRUPT_FALLING_RISING, GPIO_PIN_NO_PULL);
	ytGpioPinSetCallback(GPIO_PIN_C13, buttonCallback, NULL);

	//Haptics setup
	drv2605L_create(&drv2605L);
	drv2605L_init(drv2605L, GPIO_PIN_C3, 0, 0, 0, 0);
	drv2605L_exitStandby(drv2605L);
	drv2605L_loadActuatorConfig(drv2605L, &DRV2605L_ACTUATOR1);

	//uint8_t compensation;
	//uint8_t compensationW = 1;
	/*bool result;
	drv2605L_runAutoCalibration(drv2605L, &result);
	drv2605L_register_read(drv2605L, drv2605L_reg_autoCalibrationCompResult, &compensation);*/
	/*drv2605L_register_write(drv2605L, drv2605L_reg_autoCalibrationCompResult, &compensationW);
	drv2605L_register_read(drv2605L, drv2605L_reg_autoCalibrationCompResult, &compensation);*/

	drv2605L_setMode(drv2605L, DRV2605L_MODE_SELECTION1);
	drv2605L_selectEffectLibrary(drv2605L, DRV2605L_LIBRARY1);

	while(1){

		/*if (platformPinGet(GPIO_PIN_C13) == 1){
			ytLedToggle(LED_YELLOW_1);
		}*/

		//drv2605L_fireROMLibraryEffect(drv2605L,drv2605L_effect_tripleClick_100P,true);

		osDelay(2000);
	}
}

void buttonCallback(void const * argument){

		ytLedToggle(LED_YELLOW_1);
		/*drv2605L_fireROMLibraryEffect(drv2605L,drv2605L_effect_tripleClick_100P,true);
		osDelay(1500);
		drv2605L_stop(drv2605L);*/

		drv2605L_fireROMLibraryEffect(drv2605L,drv2605L_effect_longDoubleSharpClickStrong1_100P,true);

}

#endif



