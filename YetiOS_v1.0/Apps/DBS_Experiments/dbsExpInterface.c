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
 * dbsExpInterface.c
 *
 *  Created on: 8 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file dbsExpInterface.c
 */


#include "yetiOS.h"

#if DBS_EXP_INTERFACE_APP

#include "platformGpio.h"


#define DEFAULT_BUFFERED_READ_SAMPLE_NUM	1
#define MAX_BUFFERED_READ_SAMPLES			8*DEFAULT_BUFFERED_READ_SAMPLE_NUM

#define NO_CMD 							0
#define	CMD_SET_BUFFERED_READ_SAMPLES	1
#define	CMD_SET_SAMPLE_RATE				2
#define CMD_SET_STIM_AMP				3
#define CMD_SET_STIM_FREQ				4
#define CMD_SET_STIM_PULSE				5

typedef struct __packed rcvSpiData_{
	uint8_t rcvCmd;
	uint8_t rcvValue;
}rcvSpiData_t;

//static uint32_t bufferedReadSampleNum;
//static uint32_t newBufferedReadSampleNum;
static uint16_t* sampleBuffer;
static uint16_t* sampleBufferEndPtr;
static uint16_t* sampleBufferUsbPtr;
static rcvSpiData_t* spiRcvBuffer;

static deviceFileHandler_t* spiDevice;
static deviceFileHandler_t* usbDevice;

/*Threads handle declaration*/
static osThreadId dbsExpUsbInterfaceThreadHandle;
static osThreadId dbsExpSpiInterfaceThreadHandle;

/*Thread Functiosn declaration*/
static void dbsExpUsbInterfaceFunc(void const * argument);
static void dbsExpSpiInterfaceFunc(void const * argument);

/*Function used to configure the SPI device*/
static retval_t configSpiDevice(deviceFileHandler_t* spiDevice);
/*Process the received commands from SPI*/
static void processRcvSpiCmd(rcvSpiData_t* rcvSpiData);

/*Autostart Thread*/
YtAutoInitThread(dbsExpUsb, dbsExpUsbInterfaceFunc, osPriorityNormal, 180, &dbsExpUsbInterfaceThreadHandle, NULL);


/**
 * @brief			USB read thread function
 * @param argument
 */
static void dbsExpUsbInterfaceFunc(void const * argument){

	uint32_t numStoredSamples = 0;

//	bufferedReadSampleNum = DEFAULT_BUFFERED_READ_SAMPLE_NUM;	/*By default only use 1 sample, no buffered read*/
//	newBufferedReadSampleNum = bufferedReadSampleNum;


	/*Initialize sample buffer*/
	sampleBuffer = (uint16_t*)pvPortMalloc(sizeof(uint16_t)*MAX_BUFFERED_READ_SAMPLES);
	sampleBufferEndPtr = &sampleBuffer[MAX_BUFFERED_READ_SAMPLES];
	sampleBufferUsbPtr = sampleBuffer + 1;

	/*Open Usb Device*/
	if((usbDevice = ytOpen(PLATFORM_USB_DEVICE_ID, 0)) == NULL){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/*Open Spi Device*/
	if((spiDevice = ytOpen(PLATFORM_SPI2_DEVICE_ID, 0)) == NULL){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/*Configure the Spi device*/
	if(configSpiDevice(spiDevice) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/*Config and disable DRDY PIN*/
	if(ytGpioInitPin(GPIO_PIN_C1, GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_NO_PULL) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}
	ytGpioPinSet(GPIO_PIN_C1);

	osDelay(1);

	/*Start The Spi Thread*/
	if(ytStartThread("dbsExpSpi", dbsExpSpiInterfaceFunc, osPriorityBelowNormal, 180,
			&dbsExpSpiInterfaceThreadHandle, NULL) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/*Start The USB send commands Thread*/


	while(1){

		/*Read one sample each time in the usb*/
		ytRead(usbDevice, (uint8_t*) sampleBufferUsbPtr, sizeof (uint16_t));

		sampleBufferUsbPtr++;
		if(sampleBufferUsbPtr >= sampleBufferEndPtr){
			sampleBufferUsbPtr = sampleBuffer;
		}
		numStoredSamples++;

		if(numStoredSamples >= DEFAULT_BUFFERED_READ_SAMPLE_NUM){		/*Ready to send the samples through SPI. Toggle DRDY Pin*/
//			if(newBufferedReadSampleNum != bufferedReadSampleNum){
//				sampleBufferUsbPtr = sampleBuffer + 1;					/*Restart USB Read*/
//				numStoredSamples = 0;
//				bufferedReadSampleNum = newBufferedReadSampleNum;
//				uint32_t remainingCount = 0;
//				do{
//					ytIoctl(spiDevice, GET_CURRENT_REMAINING_COUNT, &remainingCount);
//				}while(remainingCount != (MAX_BUFFERED_READ_SAMPLES*sizeof(uint16_t)));
//			}
//			else{
				ytGpioPinSet(GPIO_PIN_C1);
				numStoredSamples = 0;
				ytGpioPinReset(GPIO_PIN_C1);
//			}
		}

	}
}

/**
 * @brief			SPI read write thread function
 * @param argument
 */
static void dbsExpSpiInterfaceFunc(void const * argument){

	uint16_t i;
	ytSpiReadWriteBuff_t spiReadWriteBuffs;
	spiRcvBuffer = (uint16_t*) pvPortMalloc(MAX_BUFFERED_READ_SAMPLES*sizeof(rcvSpiData_t));
	spiReadWriteBuffs.size = MAX_BUFFERED_READ_SAMPLES*sizeof(uint16_t);
	spiReadWriteBuffs.ptx = (uint8_t*) sampleBuffer;
	spiReadWriteBuffs.prx = (uint8_t*) spiRcvBuffer;

	ytIoctl(spiDevice, SPI_READ_WRITE, &spiReadWriteBuffs);	/*Read/Write samples in async mode*/

	while(1){
		osDelay(1);
		for(i=0; i < MAX_BUFFERED_READ_SAMPLES; i++){

			if(spiRcvBuffer[i].rcvCmd != NO_CMD){
				if(spiRcvBuffer[i].rcvCmd == CMD_SET_BUFFERED_READ_SAMPLES){
					spiRcvBuffer[i].rcvCmd = NO_CMD;
//					newBufferedReadSampleNum = ((uint32_t)(spiRcvBuffer[i].rcvValue))+1;
				}
				else{
					ytWrite(usbDevice, (uint8_t*) &spiRcvBuffer[i], sizeof(rcvSpiData_t));	/*Send the received command to reconfigure the simulator using the usb*/
					spiRcvBuffer[i].rcvCmd = NO_CMD;
				}
			}
		}



	}
}


/**
 *
 * @param spiDevice
 * @return
 */
static retval_t configSpiDevice(deviceFileHandler_t* spiDevice){

	/*Set SPI to 8000000 but it actually does not matter, since it is configured as slave, and the Master sets the speed with its clk*/
	uint32_t spiSpeed = 8000000;
	if(ytIoctl(spiDevice, SPI_SET_SPEED, &spiSpeed) != RET_OK){
		return RET_ERROR;
	}

	/*Set SPI Polarity High*/
	ytSpiPolarity_t spiPolarity = SPI_POL_HIGH;
	if(ytIoctl(spiDevice, SPI_SET_POLARITY, &spiPolarity) != RET_OK){
		return RET_ERROR;
	}

	/*Set SPI edge*/
	ytSpiEdge_t spiEdge = SPI_EDGE_SECOND;
	if(ytIoctl(spiDevice, SPI_SET_EDGE, &spiEdge) != RET_OK){
		return RET_ERROR;
	}

	/*Set Spi mode to slave*/
	ytSpiMode_t spiMode = SPI_SLAVE;
	if(ytIoctl(spiDevice, SPI_SET_MODE, &spiMode) != RET_OK){
		return RET_ERROR;
	}

	/*Set HW CS Management*/
	if(ytIoctl(spiDevice, SPI_SET_HW_CS, NULL) != RET_OK){
		return RET_ERROR;
	}

	/*Set Circular Async Mode*/
	if(ytIoctl(spiDevice, SET_ASYNC_CIRCULAR_READ_MODE, NULL) != RET_OK){
		return RET_ERROR;
	}

	return RET_OK;
}


/**
 *
 * @param spiReadWriteBuffs
 * @return
 */
static void processRcvSpiCmd(rcvSpiData_t* rcvSpiData){

	switch(rcvSpiData->rcvCmd){
	case NO_CMD:
		break;
	case CMD_SET_BUFFERED_READ_SAMPLES:
//		bufferedReadSampleNum = (uint32_t) rcvSpiData->rcvValue;
//		spiReadWriteBuffs->size = bufferedReadSampleNum*sizeof(uint16_t);
		break;
	default:
		ytWrite(usbDevice, (uint8_t*) rcvSpiData, sizeof(rcvSpiData_t));	/*Send the received command to reconfigure the simulator using the usb*/
		break;
	}
}


#endif
