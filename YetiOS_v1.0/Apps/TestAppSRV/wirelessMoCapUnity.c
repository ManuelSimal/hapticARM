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
 * wirelessMoCapUnity.c
 *
 *  Created on: Mar 5, 2020
 *      Author: Santiago Isidro Real <sreal@b105.upm.es>
 */


#include "yetiOS.h"
#include "netYetiOS.h"
#include "lsm6dsl.h"
#include "lsm6dsl_arch.h"


#if TEST_APP_WIRELESS_MOCAP_UNITY

//#define NODE_1		1
#define NODE_2		2

#define UDP_TEST_PORT	0xAAAA
#define UDP_TEST_PORT_2	0xAAAB
#define DATA_SIZE	30
#define QUEUE_LENGTH 5

typedef struct{
	LSM6DSL_Axes_t accValue;
	LSM6DSL_Axes_t gyroValue;
} imuPkg;

static QueueHandle_t udpQueue;

/***************************************/
static uint8_t testBuff[DATA_SIZE];

static osThreadId udpExampleThreadHandle;

#if NODE_2
static void UdpReceiver(void const * argument);
static void UdpTransmitter(void const * argument);
//YtAutoInitThread(UdpExampleApp, UdpReceiver, osPriorityLow, 256, &udpExampleThreadHandle, NULL);

#endif

#if NODE_1

YtAutoInitThread(UdpExampleApp, UdpTransmitter, osPriorityLow, 256, &udpExampleThreadHandle, NULL);

#endif

/***************************************/


static osThreadId testAppThreadHandle;

static void testAppFunc(void const * argument);

YtAutoInitThread(TestApp, testAppFunc, osPriorityBelowNormal, 256, &testAppThreadHandle, NULL);

LSM6DSL_IO_t default_lsm6dsl_arch_2 = {
  .Init = LSM6DSL_arch_Init,
  .DeInit = LSM6DSL_arch_DeInit,
  .BusType = 0,
  .Address = SLAVE_ADDRESS,
  .WriteReg = LSM6DSL_arch_WriteReg,
  .ReadReg = LSM6DSL_arch_ReadReg,
  .GetTick = LSM6DSL_arch_GetTick,
};



static void testAppFunc(void const * argument){
	deviceFileHandler_t* uartDevice;
	deviceFileHandler_t* gyroDevice;
	imuPkg imuDataPkg;
	uint8_t request = 0;

	if((uartDevice = ytOpen(PLATFORM_UART1_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}


	if((gyroDevice = ytOpen(PLATFORM_GYRO_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	udpQueue = xQueueCreate(QUEUE_LENGTH, sizeof(imuPkg));

	/*Start The UDP Thread*/
	if(ytStartThread("udpReceiveTask", UdpReceiver, osPriorityBelowNormal, 256,
			&udpExampleThreadHandle, NULL) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}


	while(1){
		ytLedToggle(LED_GREEN_1);

		request = 0;
		while(request != 'a')
			uartDevice->device->ops->read(uartDevice, (uint8_t*)&request, 1);

		//gyroDevice->device->ops->read(gyroDevice, (uint8_t*) &imuDataPkg.gyroValue, sizeof(LSM6DSL_Axes_t));
		if(uxQueueSpacesAvailable(udpQueue) < QUEUE_LENGTH ){
			xQueueReceive(udpQueue, &imuDataPkg, portMAX_DELAY);
			uartDevice->device->ops->write(uartDevice, (uint8_t*)&imuDataPkg, sizeof(imuPkg));
		}
	}
}


#if NODE_2

static void UdpReceiver(void const * argument){
	int queueSpaces = 0;
	osDelay(1000);

	netAddr_t addr = ytNetNewAddr("2", 'D');
	char rcvAddrStr[8];
	ytNetAddNodeAddr(addr);

	if(ytUdpOpen(UDP_TEST_PORT) != RET_OK){
		ytLedOn(LED_RED_1);
		while(1){
			osDelay(1000);
		}
	}

	while(1){
		if (ytUdpRcv(addr, UDP_TEST_PORT, testBuff, DATA_SIZE) != 0){
			ytNetAddrToString(addr, rcvAddrStr, 'D');
			ytPrintf("Received: %s from %s\r\n", testBuff, rcvAddrStr);
			ytLedToggle(LED_BLUE_1);

			if((queueSpaces = uxQueueSpacesAvailable(udpQueue)) > 0){
				xQueueSend(udpQueue, testBuff, portMAX_DELAY);
			}

		}
	}

}



static void UdpTransmitter(void const * argument){

	osDelay(1000);
	imuPkg imuDataPkg = {
			.accValue = {
					.x = 1,
					.y = 2,
					.z = 3,
			},
			.gyroValue = {
					.x = 4,
					.y = 5,
					.z = 6,
			},
	};

	memcpy(testBuff, (uint8_t*) &imuDataPkg, sizeof(imuPkg));	/*Copy also the /0 */
	netAddr_t addr = ytNetNewAddr("1", 'D');

	ytNetAddNodeAddr(addr);
	ytNetDeleteNetAddr(addr);

	addr = ytNetNewAddr("2", 'D');

	if(ytUdpOpen(UDP_TEST_PORT) != RET_OK){
		ytLedOn(LED_RED_1);
		while(1){
			osDelay(1000);
		}
	}

	while(1){
		osDelay(200);
		while(ytUdpSend(addr, UDP_TEST_PORT,testBuff, DATA_SIZE) == 0){
			osDelay(2);
		}
		ytLedToggle(LED_BLUE_1);
	}

}


#endif



#if NODE_1

static void UdpTransmitter(void const * argument){

	osDelay(1000);
	imuPkg imuDataPkg = {
			.accValue = {
					.x = 1,
					.y = 2,
					.z = 3,
			},
			.gyroValue = {
					.x = 4,
					.y = 5,
					.z = 6,
			},
	};

	memcpy(testBuff, (uint8_t*) &imuDataPkg, sizeof(imuPkg));	/*Copy also the /0 */
	netAddr_t addr = ytNetNewAddr("1", 'D');

	ytNetAddNodeAddr(addr);
	ytNetDeleteNetAddr(addr);

	addr = ytNetNewAddr("2", 'D');

	if(ytUdpOpen(UDP_TEST_PORT) != RET_OK){
		ytLedOn(LED_RED_1);
		while(1){
			osDelay(1000);
		}
	}

	while(1){
		osDelay(200);
		while(ytUdpSend(addr, UDP_TEST_PORT,testBuff, DATA_SIZE) == 0){
			osDelay(2);
		}
		ytLedToggle(LED_BLUE_1);
	}

}
#endif


#endif
