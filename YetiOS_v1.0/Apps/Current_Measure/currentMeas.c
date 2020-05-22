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
 * currentMeas.c
 *
 *  Created on: 22 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file currentMeas.c
 */


#include "yetiOS.h"

#if CURRENT_MEAS_APP

#include "platformGpio.h"
#include "adc.h"

#define VSYS_ADC_PIN			GPIO_PIN_A0
#define VDIG_ADC_PIN			GPIO_PIN_A4

#define VSYS_CTL_PIN			GPIO_PIN_C0
#define VDIG_CTL_PIN			GPIO_PIN_C2

#define ADC_SAMPLE_NUM			2
#define OUTPUT_SAMPLE_NUM		32

#define V_ADC 					3.25f
#define ADC_TOP_VALUE  			65535
#define HIGH_CURRENT_RSENSE  	0.15f   //Includes the transistor estimated RDSON. This RDSON is different deppending on VGS
#define LOW_CURRENT_RSENSE  	62.1f
#define SENSE_AMP_GAIN  		50

#define MIN_HIGH_CURRENT_VALUE	0.9f
#define MAX_LOW_CURRENT_VALUE	0.95f

#define STREAM_ID_CODE			0xFFFDFEFA

typedef enum measMode_{
	HIGH_CURRENT_MODE = 0,
	LOW_CURRENT_MODE = 1,
}measMode_t;

typedef enum operatingMode_{
	FIXED_HIGH_CURRENT_MODE = 0,
	FIXED_LOW_CURRENT_MODE = 1,
	DYNAMIC_MODE = 2,
}operatingMode_t;

static measMode_t vsysMode = HIGH_CURRENT_MODE;
static measMode_t vdigMode = HIGH_CURRENT_MODE;
static operatingMode_t vsysOpMode = DYNAMIC_MODE;
static operatingMode_t vdigOpMode = DYNAMIC_MODE;

static float32_t currentVsysRsense = HIGH_CURRENT_RSENSE;
static float32_t currentVdigRsense = HIGH_CURRENT_RSENSE;

static uint16_t pendingInterrupt = 0;

static uint32_t readAdcBuffer[ADC_SAMPLE_NUM];
static uint16_t* currentAdcBufferPtr;
static uint16_t* endAdcBufferPtr;
static float32_t outputBuffer[OUTPUT_SAMPLE_NUM*2*2];
static float32_t* outputBufferEndPtr;
static uint16_t startSend = 0;

static osSemaphoreId adcSemaphore;
static osSemaphoreId outputSemaphore;

static osThreadId currentMeasThreadHandle;
static osThreadId outputThreadHandle;

static void currentMeasFunc(void const * argument);
static void samplesOutputFunc(void const * argument);
static retval_t configAdcDevice(deviceFileHandler_t* adcDevice);
static void halfCommpleteCb(void);
static void inputConfigCommand(uint32_t argc, char** argv);
static void startSendCommand(uint32_t argc, char** argv);
static void stopSendCommand(uint32_t argc, char** argv);

YtAutoInitThread(currentMeasApp, currentMeasFunc, osPriorityNormal, 180, &currentMeasThreadHandle, NULL);
YtAutoInitThread(outputThread, samplesOutputFunc, osPriorityBelowNormal, 180, &outputThreadHandle, NULL);

/**
 *
 * @param argument
 */
static void currentMeasFunc(void const * argument){
	deviceFileHandler_t* adcDevice;
	uint16_t vsysSample;
	uint16_t vdigSample;
	float32_t vsysValue;
	float32_t vdigValue;
	float32_t* currentOutputVsysSamplePtr = outputBuffer;
	float32_t* currentOutputVdigSamplePtr = outputBuffer + 1;
	uint16_t numSamples = 0;
	outputBufferEndPtr = &outputBuffer[OUTPUT_SAMPLE_NUM*2*2];
	osDelay(3000);

	adcSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(adcSemaphore, osWaitForever);
	outputSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(outputSemaphore, osWaitForever);

	/*Open ADC Device to obtain the samples*/
	if((adcDevice = ytOpen(PLATFORM_ADC_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}
	/*Config the ADC device*/
	if(configAdcDevice(adcDevice) != RET_OK){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}


	ytGpioInitPin(VSYS_CTL_PIN, GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_NO_PULL);		//CURRENT CONTROL PIN
	ytGpioPinSet(VSYS_CTL_PIN);
	ytGpioInitPin(VDIG_CTL_PIN, GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_NO_PULL);		//CURRENT CONTROL PIN
	ytGpioPinSet(VDIG_CTL_PIN);

	currentAdcBufferPtr = (uint16_t*) readAdcBuffer;
	endAdcBufferPtr = (uint16_t*) &readAdcBuffer[ADC_SAMPLE_NUM];

	ytShellRegisterCommand("set_op_mode", inputConfigCommand);
	ytShellRegisterCommand("start_send", startSendCommand);
	ytShellRegisterCommand("stop_send", stopSendCommand);
	/*Start reading ADC*/
	ytRead(adcDevice, (uint8_t*)readAdcBuffer, ADC_SAMPLE_NUM);

	while(1){

		/*Wait for new samples*/
		if(pendingInterrupt){
			if(pendingInterrupt > 1){
				ytLedToggle(PLATFORM_ERROR_LED);	/*Overrun. Reduce ADC sample rate*/
			}
			pendingInterrupt = 0;
		}
		osSemaphoreWait(adcSemaphore, osWaitForever);

		/*Get Samples value*/
//		/*Ignore the first of each two samples (each sample is uint32_t and the pointer is uint16_t)*/
//		currentAdcBufferPtr += 2;
		vsysSample = *currentAdcBufferPtr;
		currentAdcBufferPtr++;
		vdigSample = *currentAdcBufferPtr;
		currentAdcBufferPtr++;
		if(currentAdcBufferPtr >= endAdcBufferPtr){
			currentAdcBufferPtr = (uint16_t*) readAdcBuffer;
		}

		vsysValue = (((float32_t)vsysSample)*V_ADC*1000)/(ADC_TOP_VALUE*currentVsysRsense*SENSE_AMP_GAIN);
//		vsysValue -= 0.604f;
//		if((vsysValue > 0.025f) && (vsysValue < 3)){
//			vsysValue *= 1.1f;
//		}
//		else if((vsysValue >= 3)){
//			vsysValue *= 1.15f;
//		}
		vdigValue = (((float32_t)vdigSample)*V_ADC*1000)/(ADC_TOP_VALUE*currentVdigRsense*SENSE_AMP_GAIN);

		/*Check change range*/
//		if(vsysOpMode == DYNAMIC_MODE){
//			if(vsysMode == HIGH_CURRENT_MODE){
//				if(vsysValue < MIN_HIGH_CURRENT_VALUE){	/*Change op mode*/
//					vsysMode = LOW_CURRENT_MODE;			//Switch to low current mode
//					currentVsysRsense = LOW_CURRENT_RSENSE;
//					ytGpioPinReset(VSYS_CTL_PIN);
//				}
//			}
//			else{
//				if(vsysValue > MAX_LOW_CURRENT_VALUE){	/*Change op mode*/
//					vsysMode = HIGH_CURRENT_MODE;			//Switch to low current mode
//					currentVsysRsense = HIGH_CURRENT_RSENSE;
//					ytGpioPinSet(VSYS_CTL_PIN);
//				}
//			}
//		}
//
//		if(vdigOpMode == DYNAMIC_MODE){
//			if(vdigMode == HIGH_CURRENT_MODE){
//				if(vdigValue < MIN_HIGH_CURRENT_VALUE){	/*Change op mode*/
//					vdigMode = LOW_CURRENT_MODE;			//Switch to low current mode
//					currentVdigRsense = LOW_CURRENT_RSENSE;
//					ytGpioPinReset(VDIG_CTL_PIN);
//				}
//			}
//			else{
//				if(vdigValue > MAX_LOW_CURRENT_VALUE){	/*Change op mode*/
//					vdigMode = HIGH_CURRENT_MODE;			//Switch to low current mode
//					currentVdigRsense = HIGH_CURRENT_RSENSE;
//					ytGpioPinSet(VDIG_CTL_PIN);
//				}
//			}
//		}

		/*Store sample to be sent*/
		(*currentOutputVsysSamplePtr) = vsysValue;
		(*currentOutputVdigSamplePtr) = vdigValue;
		currentOutputVsysSamplePtr += 2;
		currentOutputVdigSamplePtr += 2;
		if(currentOutputVsysSamplePtr >= outputBufferEndPtr){
			currentOutputVsysSamplePtr = outputBuffer;
			currentOutputVdigSamplePtr = outputBuffer + 1;
		}
		numSamples++;
		if(numSamples >= OUTPUT_SAMPLE_NUM){	/*Release send function if there are enough samples*/
			numSamples = 0;
			osSemaphoreRelease(outputSemaphore);
		}
	}
}

/**
 *
 * @param argument
 */
static void samplesOutputFunc(void const * argument){
	float32_t* currentOutputPtr = outputBuffer;
	uint32_t streamId = STREAM_ID_CODE;
	while(1){

		osSemaphoreWait(outputSemaphore, osWaitForever);
		if(startSend){
			ytStdoutSend((uint8_t*)&streamId, 4);
			ytStdoutSend((uint8_t*)currentOutputPtr, OUTPUT_SAMPLE_NUM*sizeof(float32_t) * 2);
			ytLedToggle(LED_BLUE_1);
		}
		if(currentOutputPtr == outputBuffer){
			currentOutputPtr += (OUTPUT_SAMPLE_NUM*2);
		}
		else{
			currentOutputPtr = outputBuffer;
		}


	}
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
static void inputConfigCommand(uint32_t argc, char** argv){

	if(argc < 2){
		return;
	}
	else{
		if(argc == 3){
			if (argv[1][0] == 'S'){		//Op mode of Vsys
				if (argv[2][0] == 'H'){		//Fixed high current mode
					vsysOpMode = FIXED_HIGH_CURRENT_MODE;
					vsysMode = HIGH_CURRENT_MODE;			//Switch to high current mode
					currentVsysRsense = HIGH_CURRENT_RSENSE;
					ytGpioPinSet(VSYS_CTL_PIN);
				}
				else if (argv[2][0] == 'L'){//Fixed low current mode
					vsysOpMode = FIXED_LOW_CURRENT_MODE;
					vsysMode = LOW_CURRENT_MODE;			//Switch to low current mode
					currentVsysRsense = LOW_CURRENT_RSENSE;
					ytGpioPinReset(VSYS_CTL_PIN);
				}
				else if (argv[2][0] == 'D'){//Dynamic current mode
					vsysOpMode = DYNAMIC_MODE;
					vsysMode = HIGH_CURRENT_MODE;			//Switch to high current mode
					currentVsysRsense = HIGH_CURRENT_RSENSE;
					ytGpioPinSet(VSYS_CTL_PIN);
				}
			}
			else if (argv[1][0] == 'D'){//Op mode of Vdig
				if (argv[2][0] == 'H'){		//Fixed high current mode
					vdigOpMode = FIXED_HIGH_CURRENT_MODE;
					vdigMode = HIGH_CURRENT_MODE;			//Switch to high current mode
					currentVdigRsense = HIGH_CURRENT_RSENSE;
					ytGpioPinSet(VDIG_CTL_PIN);
				}
				else if (argv[2][0] == 'L'){//Fixed low current mode
					vdigOpMode = FIXED_LOW_CURRENT_MODE;
					vdigMode = LOW_CURRENT_MODE;			//Switch to high current mode
					currentVdigRsense = LOW_CURRENT_RSENSE;
					ytGpioPinReset(VDIG_CTL_PIN);
				}
				else if (argv[2][0] == 'D'){//Dynamic current mode
					vdigOpMode = DYNAMIC_MODE;
					vdigMode = HIGH_CURRENT_MODE;			//Switch to high current mode
					currentVdigRsense = HIGH_CURRENT_RSENSE;
					ytGpioPinSet(VDIG_CTL_PIN);
				}

			}
		}
	}

}

/**
 *
 * @param argc
 * @param argv
 */
static void startSendCommand(uint32_t argc, char** argv){
	startSend++;
}

/**
 *
 * @param argc
 * @param argv
 */
static void stopSendCommand(uint32_t argc, char** argv){
	startSend = 0;
}
/**
 *
 * @param adcDevice
 * @return
 */
static retval_t configAdcDevice(deviceFileHandler_t* adcDevice){
	adcChannelConfig_t adcChannel;

	/*Set Sample Rate to 3125 Samples/s*/
	uint32_t sampleRate = 3125;
	if(ytIoctl(adcDevice, SET_ADC_SAMPLE_RATE, &sampleRate) != RET_OK){
		return RET_ERROR;
	}
	/*Set Dual Mode*/
	if(ytIoctl(adcDevice, SET_ADC_DUAL_MODE, NULL) != RET_OK){
		return RET_ERROR;
	}

	/*Config first channel*/
	adcChannel.channelMode = CHANNEL_SINGLE_ENDED;
	adcChannel.gpioPin1 = VSYS_ADC_PIN;
	if(ytIoctl(adcDevice, CONFIG_ADC_FIRST_CHANNEL, &adcChannel) != RET_OK){
		return RET_ERROR;
	}

	/*Config second channel*/
	adcChannel.channelMode = CHANNEL_SINGLE_ENDED;
	adcChannel.gpioPin1 = VDIG_ADC_PIN;
	if(ytIoctl(adcDevice, CONFIG_ADC_SECOND_CHANNEL, &adcChannel) != RET_OK){
		return RET_ERROR;
	}

	/*Set async circular mode*/
	if(ytIoctl(adcDevice, SET_ASYNC_CIRCULAR_READ_MODE, NULL) != RET_OK){
		return RET_ERROR;
	}

	/*Set half complete callback*/
	if(ytIoctl(adcDevice, SET_HALF_COMPLETE_CB, halfCommpleteCb) != RET_OK){
		return RET_ERROR;
	}

	/*Set read complete callback*/
	if(ytIoctl(adcDevice, SET_READ_COMPLETE_CB, halfCommpleteCb) != RET_OK){
		return RET_ERROR;
	}

	/*Set half callback*/

	return RET_OK;
}

/**
 *
 */
static void halfCommpleteCb(void){
	pendingInterrupt++;
	osSemaphoreRelease(adcSemaphore);

}

#endif
