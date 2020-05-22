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
 * testApp.c
 *
 *  Created on: 15 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>, Santiago Real <srealvaldes@gmail.com>
 *
 */
/**
 * @file testApp.c
 */

#include "yetiOS.h"
#include "lsm6dsl.h"
#include "lsm6dsl_arch.h"


#if TEST_APP_SRV


static osThreadId testAppThreadHandle;
static osThreadId testAppThreadHandle2;

static void testAppFunc(void const * argument);
static void testAppFunc2(void const * argument);

YtAutoInitThread(TestApp, testAppFunc, osPriorityLow, 256, &testAppThreadHandle, NULL);
YtAutoInitThread(TestApp2, testAppFunc2, osPriorityLow, 255, &testAppThreadHandle2, NULL);

LSM6DSL_IO_t i2c_io = {
  .Init = LSM6DSL_arch_Init,
  .DeInit = LSM6DSL_arch_DeInit,
  .BusType = 0,
  .Address = 0xD4,
  .WriteReg = LSM6DSL_arch_WriteReg,
  .ReadReg = LSM6DSL_arch_ReadReg,
  .GetTick = LSM6DSL_arch_GetTick,
};

static void testAppFunc(void const * argument){
	deviceFileHandler_t* uartDevice;
	char outputLine[100];
	uint32_t ok = 5;
	LSM6DSL_Object_t lsm6dsl_obj;
	LSM6DSL_Axes_t gyroValue;
	uint8_t id;

	if((uartDevice = ytOpen(PLATFORM_UART1_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	ok = LSM6DSL_RegisterBusIO(&lsm6dsl_obj, &i2c_io);
	ok = LSM6DSL_Init(&lsm6dsl_obj);
	ok = LSM6DSL_ReadID(&lsm6dsl_obj, &id);
	ok = LSM6DSL_GYRO_Enable(&lsm6dsl_obj);

	while(1){
		//ytLedToggle(LED_BLUE_1);
		ytLedToggle(LED_GREEN_1);
		ok = LSM6DSL_GYRO_GetAxes(&lsm6dsl_obj, &gyroValue);
		sprintf(outputLine, "Gyro: %ld, %ld, %ld\n\r", gyroValue.x, gyroValue.y, gyroValue.z);
		uartDevice->device->ops->write(uartDevice, (uint8_t*)outputLine, strlen(outputLine));
		//ytPrintf("HOLA1\r\n");
		osDelay(500);
	}
}

static void testAppFunc2(void const * argument){

	while(1){
		ytLedToggle(LED_BLUE_1);
		//ytLedToggle(LED_GREEN_1);
		osDelay(800);
		ytPrintf("HOLA2\r\n");
	}

}


#endif
