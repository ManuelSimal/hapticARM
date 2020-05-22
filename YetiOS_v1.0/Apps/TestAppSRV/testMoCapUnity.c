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
 * testMoCapUnity.c
 *
 *  Created on: Mar 4, 2020
 *      Author: Santiago Isidro Real <sreal@b105.upm.es>
 */


#include "yetiOS.h"
#include "lsm6dsl.h"
#include "lsm6dsl_arch.h"


#if TEST_APP_MOCAP_UNITY

typedef struct{
	LSM6DSL_Axes_t accValue;
	LSM6DSL_Axes_t gyroValue;
} imuPkg;


static osThreadId testAppThreadHandle;

static void testAppFunc(void const * argument);

YtAutoInitThread(TestApp, testAppFunc, osPriorityLow, 256, &testAppThreadHandle, NULL);


static void testAppFunc(void const * argument){
	deviceFileHandler_t* uartDevice;
	deviceFileHandler_t* gyroDevice;
	deviceFileHandler_t* accDevice;
	imuPkg imuDataPkg;
	uint8_t request = 0;

	LSM6DSL_Axes_t gyroValue;

	if((uartDevice = ytOpen(PLATFORM_UART1_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}


	if((gyroDevice = ytOpen(PLATFORM_GYRO_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}


	if((accDevice = ytOpen(PLATFORM_ACC_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}


	while(1){
		ytLedToggle(LED_GREEN_1);

		request = 0;
		/*while(request != 'a')
			uartDevice->device->ops->read(uartDevice, (uint8_t*)&request, 1);*/

		gyroDevice->device->ops->read(gyroDevice, (uint8_t*) &imuDataPkg.gyroValue, sizeof(LSM6DSL_Axes_t));
		//accDevice->device->ops->read(accDevice, (uint8_t*) &imuDataPkg.accValue, sizeof(LSM6DSL_Axes_t));
		//uartDevice->device->ops->write(uartDevice, (uint8_t*)&imuDataPkg, sizeof(imuPkg));

		osDelay(500);
	}
}



#endif
