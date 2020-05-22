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
 * adcDriver.c
 *
 *  Created on: 16 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file adcDriver.c
 */


#include "yetiOS.h"
#ifdef USE_HEIMDALL_L4
#include "stm32l4xx_hal.h"
#endif

#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM
#if (PLATFORM_ADC_DEVICE_ID)	/*Do not compile the dirver if not defined any of the IDs in the Conf file*/
#include "deviceDriver.h"
#include "lowPower.h"
#include "platformGpio.h"
#include "adc.h"

#define NUM_AVAILABLE_CYCLES	8

#define ADC_DEFAULT_TIMEOUT		osWaitForever

typedef enum adcDeviceMode_{
	SINGLE_MODE,
	DUAL_MODE,
}adcDeviceMode_t;

typedef struct adcDevicePrivateData_{
	adcDeviceMode_t adcDeviceMode;
	ADC_ChannelConfTypeDef adcChannelConfig1;
	ADC_ChannelConfTypeDef adcChannelConfig2;
	uint32_t sampleRate;
	osMutexId openedFileMutex;
	osSemaphoreId adcReadSemaphore;
	uint8_t* readingBuff;
	void(* readCompleteCb)(void);
	void(* halfReadCompleteCb)(void);
	uint32_t readingBuffSize;
	uint16_t adcOpened;
	uint16_t adcReading;
	uint16_t asyncCircularRead;
}adcDevicePrivateData_t;

static devHandler_t* adcDev;
static adcDevicePrivateData_t* adcPrivateData = NULL;

static const uint32_t availableAdcCycles[NUM_AVAILABLE_CYCLES] = {15, 19, 25, 37, 60, 105, 260, 653};

/*ADC and DMA HAL handles*/
static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static DMA_HandleTypeDef hdma_adc1;
static DMA_HandleTypeDef hdma_adc2;
static ADC_MultiModeTypeDef adcMultimode;

/*SPI Driver Init and Exit*/
static retval_t adcInit(void);
static retval_t adcExit(void);

/*SPI DRIVER FUNCTIONS*/
static retval_t adcOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t adcClose(deviceFileHandler_t* devFile);
static uint32_t adcRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t adcWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t adcIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t adcChangedCpuFreq(devHandler_t* devHandler);
/*AUX Functions*/
static uint32_t getChannelFromGpio(uint32_t gpioPin);
static retval_t setAdcSampleRate(uint32_t sampleRate);
static void MX_ADC1_Init(void);
static void MX_ADC1_DeInit();
static void MX_ADC2_Init(void);
static void MX_ADC2_DeInit();


static deviceDriverOps_t adcDriverOps = {
	.open = adcOpen,
	.close = adcClose,
	.read = adcRead,
	.write = adcWrite,
	.ioctl = adcIoctl,
	.changedCpuFreq = adcChangedCpuFreq,
};

/**
 *
 * @return
 */
static retval_t adcInit(void){
	if(adcPrivateData != NULL){
		return RET_ERROR;
	}


	adcPrivateData = (adcDevicePrivateData_t*) pvPortMalloc(sizeof(adcDevicePrivateData_t));
	adcPrivateData->openedFileMutex = ytMutexCreate();
	adcPrivateData->adcReadSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(adcPrivateData->adcReadSemaphore, osWaitForever);
	adcPrivateData->readCompleteCb = NULL;
	adcPrivateData->halfReadCompleteCb = NULL;
	adcPrivateData->sampleRate = 0;
	adcPrivateData->adcDeviceMode = SINGLE_MODE;
	adcPrivateData->adcOpened = 0;
	adcPrivateData->adcChannelConfig2.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	adcPrivateData->adcChannelConfig2.Rank = 1;
	adcPrivateData->adcChannelConfig2.OffsetNumber = ADC_OFFSET_NONE;
	adcPrivateData->adcChannelConfig2.Offset = 0;

	adcPrivateData->adcChannelConfig1.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	adcPrivateData->adcChannelConfig1.Rank = 1;
	adcPrivateData->adcChannelConfig1.OffsetNumber = ADC_OFFSET_NONE;
	adcPrivateData->adcChannelConfig1.Offset = 0;
	adcPrivateData->asyncCircularRead = 0;
	adcPrivateData->adcReading = 0;
	adcPrivateData->readingBuff = NULL;
	adcPrivateData->readingBuffSize = 0;
	/*Initialize HAL ADC*/
	MX_ADC1_Init();
	MX_ADC2_Init();

	adcDev =  ytNewDevice(PLATFORM_ADC_DEVICE_ID, &adcDriverOps);
	/*Register the new device*/
	if(ytRegisterDevice(adcDev) != RET_OK){
		MX_ADC1_DeInit();
		MX_ADC2_DeInit();
		osMutexDelete(adcPrivateData->openedFileMutex);
		osSemaphoreDelete(adcPrivateData->adcReadSemaphore);
		vPortFree(adcPrivateData);
		adcPrivateData = NULL;
	}

	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t adcExit(void){

	if(adcPrivateData == NULL){
		return RET_ERROR;
	}
	ytUnregisterDevice(PLATFORM_ADC_DEVICE_ID);

	MX_ADC1_DeInit();
	MX_ADC2_DeInit();

	osMutexDelete(adcPrivateData->openedFileMutex);
	osSemaphoreDelete(adcPrivateData->adcReadSemaphore);

	vPortFree(adcPrivateData);
	adcPrivateData = NULL;
	ytDeleteDevice(adcDev);
	return RET_OK;
}


/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t adcOpen(deviceFileHandler_t* devFile, uint32_t flags){
	retval_t ret = RET_ERROR;
	if(adcPrivateData == NULL){
		return ret;
	}
	osMutexWait(adcPrivateData->openedFileMutex, osWaitForever);
	if(!adcPrivateData->adcOpened){
		adcPrivateData->adcOpened++;
		ret = RET_OK;
	}
	osMutexRelease(adcPrivateData->openedFileMutex);
	return ret;
}

/**
 *
 * @param devFile
 * @return
 */
static retval_t adcClose(deviceFileHandler_t* devFile){
	retval_t ret = RET_ERROR;
	if(adcPrivateData == NULL){
		return ret;
	}
	osMutexWait(adcPrivateData->openedFileMutex, osWaitForever);
	if(adcPrivateData->adcOpened){
		if(!adcPrivateData->adcReading){
			adcPrivateData->adcOpened = 0;
			ret = RET_OK;
		}
	}
	osMutexRelease(adcPrivateData->openedFileMutex);
	return ret;
}



/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t adcRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){

	uint32_t ret = 0;
	osMutexWait(adcPrivateData->openedFileMutex, osWaitForever);
	if(!adcPrivateData->adcReading){
		/*Single Mode*/
		if(adcPrivateData->adcDeviceMode == SINGLE_MODE){
			if(hdma_adc1.Init.Mode == DMA_CIRCULAR){	/*Async Circular Read*/
				ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
				adcPrivateData->readingBuff = buff;
				adcPrivateData->readingBuffSize = size;
				adcPrivateData->adcReading++;
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) buff, size);
			}
			else{	/*Normal Read*/
				adcPrivateData->adcReading++;
				ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) buff, size);

				osMutexRelease(adcPrivateData->openedFileMutex);

				osSemaphoreWait(adcPrivateData->adcReadSemaphore, osWaitForever);

				ytSetDeviceDefaultLowPowerMode();

				osMutexWait(adcPrivateData->openedFileMutex, osWaitForever);
				adcPrivateData->adcReading--;
				ret = size;
			}

		}
		/*Dual Mode*/
		else{
			if(hdma_adc1.Init.Mode == DMA_CIRCULAR){	/*Async Circular Read*/
				ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
				adcPrivateData->readingBuff = buff;
				adcPrivateData->readingBuffSize = size;
				adcPrivateData->adcReading++;
				HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) buff, size);
			}
			else{	/*Normal Read*/
				adcPrivateData->adcReading++;
				ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
				HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) buff, size);

				osMutexRelease(adcPrivateData->openedFileMutex);

				osSemaphoreWait(adcPrivateData->adcReadSemaphore, osWaitForever);

				ytSetDeviceDefaultLowPowerMode();

				osMutexWait(adcPrivateData->openedFileMutex, osWaitForever);
				adcPrivateData->adcReading--;
				ret = size;
			}
		}
	}
	osMutexRelease(adcPrivateData->openedFileMutex);
	return ret;
}


	/**
	 *
	 * @param devFile
	 * @param buff
	 * @param size
	 * @return
	 */
static uint32_t adcWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return 0;
}


/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t adcIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	retval_t ret = RET_OK;
	if(adcPrivateData == NULL){
		return RET_ERROR;
	}
	osMutexWait(adcPrivateData->openedFileMutex, osWaitForever);
	if(adcPrivateData->adcOpened){
		switch(command){
		case SET_ADC_SAMPLE_RATE:
			if(!adcPrivateData->adcReading){
				setAdcSampleRate(*((uint32_t*) args));
			}
			else{
				ret = RET_ERROR;
			}
			break;

		case SET_ASYNC_CIRCULAR_READ_MODE:
			if(!adcPrivateData->adcReading){
				hdma_adc1.Init.Mode = DMA_CIRCULAR;
				HAL_DMA_Init(&hdma_adc1);
				hdma_adc2.Init.Mode = DMA_CIRCULAR;
				HAL_DMA_Init(&hdma_adc2);
			}
			else{
				ret = RET_ERROR;
			}
			break;

		case STOP_ASYNC_READ:
			if(hdma_adc1.Init.Mode == DMA_CIRCULAR){
				if(adcPrivateData->adcReading){
					if(adcPrivateData->adcDeviceMode == SINGLE_MODE){
						HAL_ADC_Stop_DMA(&hadc1);
					}
					else{
						HAL_ADCEx_MultiModeStop_DMA(&hadc1);
					}
					ytSetDeviceDefaultLowPowerMode();
				}
				else{
					ret = RET_ERROR;
				}
			}
			else{
				ret = RET_ERROR;
			}
			break;
		case SET_READ_COMPLETE_CB:
			adcPrivateData->readCompleteCb = args;
			break;
		case SET_HALF_COMPLETE_CB:
			adcPrivateData->halfReadCompleteCb = args;
			break;
		case SET_STD_READ_MODE:
			if(!adcPrivateData->adcReading){
				hdma_adc1.Init.Mode = DMA_NORMAL;
				HAL_DMA_Init(&hdma_adc1);
				hdma_adc2.Init.Mode = DMA_NORMAL;
				HAL_DMA_Init(&hdma_adc2);
			}
			else{
				ret = RET_ERROR;
			}
			break;

		case GET_CURRENT_READ_PTR:
			if((!adcPrivateData->adcReading) || adcPrivateData->adcDeviceMode != DUAL_MODE){
				ret = RET_ERROR;
			}
			else{
				if(args == NULL){
					ret = RET_ERROR;
				}
				else{
					uint8_t** retPtr = (uint8_t**) args;
					(*retPtr) = adcPrivateData->readingBuff + (adcPrivateData->readingBuffSize - __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle));
				}
			}
			break;

		case SET_ADC_DUAL_MODE:
			if(!adcPrivateData->adcReading){
				adcMultimode.Mode = ADC_DUALMODE_REGSIMULT;
				adcMultimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
				adcMultimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
				HAL_ADCEx_MultiModeConfigChannel(&hadc1, &adcMultimode);
				adcPrivateData->adcDeviceMode = DUAL_MODE;

				/* COnfig DMA WORD TRANSFER*/
				hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
				hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
				HAL_DMA_Init(&hdma_adc1);
				hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
				hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
				HAL_DMA_Init(&hdma_adc2);
			}
			else{
				ret = RET_ERROR;
			}
			break;
		case SET_ADC_SINGLE_MODE:
			if(!adcPrivateData->adcReading){
				adcMultimode.Mode = ADC_MODE_INDEPENDENT;
				HAL_ADCEx_MultiModeConfigChannel(&hadc1, &adcMultimode);
				adcPrivateData->adcDeviceMode = SINGLE_MODE;

				/* COnfig DMA HALFWORD TRANSFER*/
				hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
				hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
				HAL_DMA_Init(&hdma_adc1);
				hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
				hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
				HAL_DMA_Init(&hdma_adc2);
			}
			else{
				ret = RET_ERROR;
			}
			break;
		case CONFIG_ADC_FIRST_CHANNEL:
			if(!adcPrivateData->adcReading){
				adcChannelConfig_t* adcChannelConfig = (adcChannelConfig_t*) args;
				if(ytGpioInitPin(adcChannelConfig->gpioPin1, GPIO_PIN_ANALOG_ADC, GPIO_PIN_NO_PULL) != RET_OK){
					ret = RET_ERROR;
					break;
				}
				if(adcChannelConfig->channelMode == CHANNEL_DIFFERENTIAL){
					if(ytGpioInitPin(adcChannelConfig->gpioPin2, GPIO_PIN_ANALOG_ADC, GPIO_PIN_NO_PULL) != RET_OK){
						ytGpioDeInitPin(adcChannelConfig->gpioPin1);
						ret = RET_ERROR;
						break;
					}
					adcPrivateData->adcChannelConfig1.SingleDiff = ADC_DIFFERENTIAL_ENDED;
				}
				else{
					adcPrivateData->adcChannelConfig1.SingleDiff = ADC_SINGLE_ENDED;
				}
				adcPrivateData->adcChannelConfig1.Channel = getChannelFromGpio(adcChannelConfig->gpioPin1);
				HAL_ADC_ConfigChannel(&hadc1, &adcPrivateData->adcChannelConfig1);
			}
			else{
				ret = RET_ERROR;
			}
			break;
		case CONFIG_ADC_SECOND_CHANNEL:
			if(!adcPrivateData->adcReading){
				if(adcPrivateData->adcDeviceMode == SINGLE_MODE){
					ret = RET_ERROR;
				}
				else{
					adcChannelConfig_t* adcChannelConfig = (adcChannelConfig_t*) args;
					if(ytGpioInitPin(adcChannelConfig->gpioPin1, GPIO_PIN_ANALOG_ADC, GPIO_PIN_NO_PULL) != RET_OK){
						ret = RET_ERROR;
						break;
					}
					if(adcChannelConfig->channelMode == CHANNEL_DIFFERENTIAL){
						if(ytGpioInitPin(adcChannelConfig->gpioPin2, GPIO_PIN_ANALOG_ADC, GPIO_PIN_NO_PULL) != RET_OK){
							ytGpioDeInitPin(adcChannelConfig->gpioPin1);
							ret = RET_ERROR;
							break;
						}
						adcPrivateData->adcChannelConfig2.SingleDiff = ADC_DIFFERENTIAL_ENDED;
					}
					else{
						adcPrivateData->adcChannelConfig2.SingleDiff = ADC_SINGLE_ENDED;
					}
					adcPrivateData->adcChannelConfig2.Channel = getChannelFromGpio(adcChannelConfig->gpioPin1);
					HAL_ADC_ConfigChannel(&hadc2, &adcPrivateData->adcChannelConfig2);
				}
			}
			else{
				ret = RET_ERROR;
			}
			break;
		default:
			ret = RET_ERROR;
			break;
		}
	}
	osMutexRelease(adcPrivateData->openedFileMutex);
	return ret;
}

/**
 *
 * @param devHandler
 * @return
 */
static retval_t adcChangedCpuFreq(devHandler_t* devHandler){
	return setAdcSampleRate(adcPrivateData->sampleRate);
}


/**
 *
 * @param gpioPin
 * @return
 */
static uint32_t getChannelFromGpio(uint32_t gpioPin){
	switch(gpioPin){
	case GPIO_PIN_A0:
		return ADC_CHANNEL_5;
		break;
	case GPIO_PIN_A1:
		return ADC_CHANNEL_6;
		break;
	case  GPIO_PIN_A4:
		return ADC_CHANNEL_9;
		break;
	case GPIO_PIN_C1:
		return ADC_CHANNEL_2;
		break;
	case GPIO_PIN_C3:
		return ADC_CHANNEL_4;
		break;
	case GPIO_PIN_C5:
		return ADC_CHANNEL_14;
		break;
	default:
		return 0xFFFFFFFF;
		break;
	}
}

/**
 *
 * @param sampleRate
 * @return
 */
static retval_t setAdcSampleRate(uint32_t sampleRate){
	uint32_t stimatedSpeed,  oversampling, bitShift, channelCycles;
	uint32_t diff = 0xFFFFFFFF;
	uint16_t i;
	uint16_t best_index = 0;
	uint32_t bestOversampling = 0;
	uint32_t bestShift = 0;
	uint32_t bestStimatedSpeed = 0;

	for(i=0; i<NUM_AVAILABLE_CYCLES; i++){

		oversampling = (uint32_t) HAL_RCC_GetHCLKFreq()/(sampleRate*availableAdcCycles[i]);

		if(oversampling > 128){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(256*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_256;
			bitShift = ADC_RIGHTBITSHIFT_4;
		}
		else if(oversampling > 64){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(128*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_128;
			bitShift = ADC_RIGHTBITSHIFT_3;
		}
		else if(oversampling > 32){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(64*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_64;
			bitShift = ADC_RIGHTBITSHIFT_2;
		}
		else if(oversampling > 16){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(32*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_32;
			bitShift = ADC_RIGHTBITSHIFT_1;
		}
		else if(oversampling > 8){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(16*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_16;
			bitShift = ADC_RIGHTBITSHIFT_NONE;
		}
		else if(oversampling > 4){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(8*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_8;
			bitShift = ADC_RIGHTBITSHIFT_NONE;
		}
		else if(oversampling > 2){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(4*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_4;
			bitShift = ADC_RIGHTBITSHIFT_NONE;
		}
		else if(oversampling > 0){
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(2*availableAdcCycles[i]);
			oversampling = ADC_OVERSAMPLING_RATIO_2;
			bitShift = ADC_RIGHTBITSHIFT_NONE;
		}
		else{
			stimatedSpeed = (uint32_t) HAL_RCC_GetHCLKFreq()/(availableAdcCycles[i]);
			oversampling = 0xFFFFFFFF;
			bitShift = 0xFFFFFFFF;
		}

		if(stimatedSpeed > sampleRate){

			if(stimatedSpeed-sampleRate < diff){
				diff = stimatedSpeed-sampleRate;
				best_index = i;
				bestOversampling = oversampling;
				bestStimatedSpeed = stimatedSpeed;
				bestShift = bitShift;
			}
		}
		else{
			if(sampleRate-stimatedSpeed < diff){
				diff = sampleRate-stimatedSpeed;
				best_index = i;
				bestOversampling = oversampling;
				bestStimatedSpeed = stimatedSpeed;
				bestShift = bitShift;
			}
		}
	}

	oversampling = bestOversampling;
	bitShift = bestShift;
	sampleRate = bestStimatedSpeed;

	switch(availableAdcCycles[best_index]){
		case 15:
			channelCycles =	ADC_SAMPLETIME_2CYCLES_5;
			break;
		case 19:
			channelCycles =	ADC_SAMPLETIME_6CYCLES_5;
			break;
		case 25:
			channelCycles =	ADC_SAMPLETIME_12CYCLES_5;
			break;
		case 37:
			channelCycles = ADC_SAMPLETIME_24CYCLES_5;
			break;
		case 60:
			channelCycles = ADC_SAMPLETIME_47CYCLES_5;
			break;
		case 105:
			channelCycles = ADC_SAMPLETIME_92CYCLES_5;
			break;
		case 260:
			channelCycles = ADC_SAMPLETIME_247CYCLES_5;
			break;
		case 653:
			channelCycles = ADC_SAMPLETIME_640CYCLES_5;
			break;
		default:
			channelCycles = ADC_SAMPLETIME_2CYCLES_5;
			break;
	}


	if(oversampling != 0xFFFFFFFF){

		hadc1.Init.OversamplingMode = ENABLE;
		hadc1.Init.Oversampling.Ratio = oversampling;
		hadc1.Init.Oversampling.RightBitShift = bitShift;

		hadc2.Init.OversamplingMode = ENABLE;
		hadc2.Init.Oversampling.Ratio = oversampling;
		hadc2.Init.Oversampling.RightBitShift = bitShift;
	}
	else{
		hadc1.Init.OversamplingMode = DISABLE;
		hadc2.Init.OversamplingMode = DISABLE;
	}

	if(adcPrivateData->adcDeviceMode == DUAL_MODE){
		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);

		adcPrivateData->adcChannelConfig1.SamplingTime = channelCycles;
		adcPrivateData->adcChannelConfig2.SamplingTime = channelCycles;
		HAL_ADC_ConfigChannel(&hadc1, &adcPrivateData->adcChannelConfig1);
		HAL_ADC_ConfigChannel(&hadc2, &adcPrivateData->adcChannelConfig2);
	}
	else{
		HAL_ADC_Init(&hadc1);

		adcPrivateData->adcChannelConfig1.SamplingTime = channelCycles;
		HAL_ADC_ConfigChannel(&hadc1, &adcPrivateData->adcChannelConfig1);
	}

	adcPrivateData->sampleRate = sampleRate;
	return RET_OK;
}

/**
 *
 * @param hadc
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(adcPrivateData->adcReading){
		if(hdma_adc1.Init.Mode == DMA_NORMAL){
			osSemaphoreRelease(adcPrivateData->adcReadSemaphore);
		}
		else{
			if(adcPrivateData->readCompleteCb != NULL){
				adcPrivateData->readCompleteCb();
			}
		}
	}
}

/**
 *
 * @param hadc
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
	if(adcPrivateData->adcReading && hdma_adc1.Init.Mode == DMA_CIRCULAR){
		if(adcPrivateData->halfReadCompleteCb != NULL){
			adcPrivateData->halfReadCompleteCb();
		}
	}
}

static void MX_ADC1_Init(void)
{


    /**Common config
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_32;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  HAL_ADC_Init(&hadc1);

    /**Configure the ADC multi-mode
    */
  adcMultimode.Mode = ADC_MODE_INDEPENDENT;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &adcMultimode);
}
/**
 *
 */
static void MX_ADC1_DeInit(){
	HAL_ADC_DeInit(&hadc1);
}


/**
 *
 */
static void MX_ADC2_Init(void)
{
    /*Common config */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.NbrOfDiscConversion = 1;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_32;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  HAL_ADC_Init(&hadc2);

}

/**
 *
 */
static void MX_ADC2_DeInit(){
	HAL_ADC_DeInit(&hadc2);
}

static uint32_t HAL_RCC_ADC_CLK_ENABLED=0;

/**
 *
 * @param adcHandle
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  if(adcHandle->Instance==ADC1)
  {
    /* ADC1 clock enable */
    HAL_RCC_ADC_CLK_ENABLED++;
    if(HAL_RCC_ADC_CLK_ENABLED==1){
      __HAL_RCC_ADC_CLK_ENABLE();
    }
    /* ADC1 DMA Init */
    /* DMA ENABLE*/
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);



    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }
  else if(adcHandle->Instance==ADC2)
  {
    /* ADC2 clock enable */
    HAL_RCC_ADC_CLK_ENABLED++;
    if(HAL_RCC_ADC_CLK_ENABLED==1){
      __HAL_RCC_ADC_CLK_ENABLE();
    }
    /* ADC2 DMA Init */
    __HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);

    /* ADC2 Init */
    hdma_adc2.Instance = DMA1_Channel2;
    hdma_adc2.Init.Request = DMA_REQUEST_0;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc2);

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }
}

/**
 *
 * @param adcHandle
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
    /* ADC1 DMA DeInit */
	  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	  HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
  else if(adcHandle->Instance==ADC2)
  {
	  /* ADC2 DMA DeInit */
	  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
	  HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
}



/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/**
* @brief This function handles DMA1 channel2 global interrupt.
*/
void DMA1_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc2);
}

/**
 *
 */
void ADC1_2_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
}


InitDevice(adcInit);
ExitDevice(adcExit);

#endif
#endif
