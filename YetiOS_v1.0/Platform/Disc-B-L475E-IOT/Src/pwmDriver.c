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
 *  Created on: 17 mar. 2020
 *      Author: Santiago Real Valdes  <sreal@b105.upm.es>
 *
 */
/**
 * @file pwmDriver.c
 */


#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#if (PLATFORM_PWM_DEVICE_ID)	/*Do not compile the driver if not defined any of the IDs in the Conf file*/
#include "stm32l4xx_hal.h"
#include "deviceDriver.h"
#include "lowPower.h"
#include "platformGpio.h"

#define PWM_DEFAULT_CARRIER_FREQ	1000
#define PWM_MIN_RESOLUTION		0x000F
#define PWM_MAX_RESOLUTION		0xFFFF

/*Channels available in the board connectors*/
typedef enum pwmChannel_{
	PWM_CHANNEL_1 = TIM_CHANNEL_1,
	PWM_CHANNEL_2 = TIM_CHANNEL_3,
	PWM_CHANNEL_3 = TIM_CHANNEL_4,
}pwmChannel_t;

typedef struct adcDevicePrivateData_{
	uint16_t channel_1_dc;
	uint16_t channel_2_dc;
	uint16_t channel_3_dc;
	uint32_t carrierFreq;
	osMutexId openedFileMutex;
	uint16_t pwmOpened;
}pwmDevicePrivateData_t;

static devHandler_t* pwmDev;
static pwmDevicePrivateData_t* pwmPrivateData = NULL;

/*TIM HAL handle*/
static TIM_HandleTypeDef htim3;
static TIM_OC_InitTypeDef sConfigOC;

/*PWM Driver Init and Exit*/
static retval_t pwmInit(void);
static retval_t pwmExit(void);

/*PWM DRIVER FUNCTIONS*/
static retval_t pwmOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t pwmClose(deviceFileHandler_t* devFile);
static uint32_t pwmRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t pwmWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t pwmIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t pwmChangedCpuFreq(devHandler_t* devHandler);
/*AUX Functions*/
static retval_t MX_PWM_Init(pwmDevicePrivateData_t* pwmPrivateData);
static void MX_PWM_DeInit(void);
//static retval_t validCarrierFreq(uint32_t value);
static inline retval_t validPWMPeriod(void);
static inline uint32_t getRawDutyCycle(uint16_t dcValue);
static retval_t setPWMDutyCycle(uint32_t value, pwmChannel_t channel);


static deviceDriverOps_t pwmDriverOps = {
	.open = pwmOpen,
	.close = pwmClose,
	.read = pwmRead,
	.write = pwmWrite,
	.ioctl = pwmIoctl,
	.changedCpuFreq = pwmChangedCpuFreq,
};

/**
 *
 * @return
 */
static retval_t pwmInit(void){
	if(pwmPrivateData != NULL)
		return RET_ERROR;

	pwmPrivateData = (pwmDevicePrivateData_t*) pvPortMalloc(sizeof(pwmDevicePrivateData_t));
	pwmPrivateData->openedFileMutex = ytMutexCreate();
	pwmPrivateData->pwmOpened = 0;
	pwmPrivateData->channel_1_dc = 0;
	pwmPrivateData->channel_2_dc = 0;
	pwmPrivateData->channel_3_dc = 0;

	/*Initialize HAL TIM*/
	if(MX_PWM_Init(pwmPrivateData))
		return RET_ERROR;

	pwmDev =  ytNewDevice(PLATFORM_PWM_DEVICE_ID, &pwmDriverOps);
	/*Register the new device*/
	if(ytRegisterDevice(pwmDev) != RET_OK){
		MX_PWM_DeInit();
		osMutexDelete(pwmPrivateData->openedFileMutex);
		vPortFree(pwmPrivateData);
		pwmPrivateData = NULL;
	}

	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t pwmExit(void){
	if(pwmPrivateData == NULL){
		return RET_ERROR;
	}
	ytUnregisterDevice(PLATFORM_PWM_DEVICE_ID);

	MX_PWM_DeInit();

	osMutexDelete(pwmPrivateData->openedFileMutex);

	vPortFree(pwmPrivateData);
	pwmPrivateData = NULL;
	ytDeleteDevice(pwmDev);
	return RET_OK;
}


/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t pwmOpen(deviceFileHandler_t* devFile, uint32_t flags){
	retval_t ret = RET_ERROR;
	if(pwmPrivateData == NULL){
		return ret;
	}
	osMutexWait(pwmPrivateData->openedFileMutex, osWaitForever);
	if(!pwmPrivateData->pwmOpened){
		pwmPrivateData->pwmOpened++;
		ret = RET_OK;
	}

	HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_3);

	osMutexRelease(pwmPrivateData->openedFileMutex);
	return ret;
}

/**
 *
 * @param devFile
 * @return
 */
static retval_t pwmClose(deviceFileHandler_t* devFile){
	retval_t ret = RET_ERROR;
	if(pwmPrivateData == NULL){
		return ret;
	}
	osMutexWait(pwmPrivateData->openedFileMutex, osWaitForever);
	if(pwmPrivateData->pwmOpened){
		pwmPrivateData->pwmOpened = 0;
		ret = RET_OK;
	}
	osMutexRelease(pwmPrivateData->openedFileMutex);
	return ret;
}



/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t pwmRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return RET_OK;
}


	/**
	 *
	 * @param devFile
	 * @param buff
	 * @param size
	 * @return
	 */
static uint32_t pwmWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return RET_OK;
}


/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t pwmIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	retval_t ret = RET_OK;
	uint32_t aux;
	if(pwmPrivateData == NULL){
		return RET_ERROR;
	}
	osMutexWait(pwmPrivateData->openedFileMutex, osWaitForever);
	if(pwmPrivateData->pwmOpened){
		switch(command){
		case PWM_SET_CARRIER_FREQ:
			HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_3);

			aux = htim3.Init.Period;
			htim3.Init.Period = HAL_RCC_GetHCLKFreq()/htim3.Init.Prescaler/(*((uint16_t*)args));
			if(validPWMPeriod() != RET_OK){
				ret = RET_ERROR;
				htim3.Init.Period = aux;
			}else{
				if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
					ret = RET_ERROR;
				if (setPWMDutyCycle(pwmPrivateData->channel_1_dc, PWM_CHANNEL_1) != RET_OK)
					ret = RET_ERROR;
				if (setPWMDutyCycle(pwmPrivateData->channel_2_dc, PWM_CHANNEL_2) != RET_OK)
					ret = RET_ERROR;
				if (setPWMDutyCycle(pwmPrivateData->channel_3_dc, PWM_CHANNEL_3) != RET_OK)
					ret = RET_ERROR;
			}

			HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_3);
			break;
		case PWM_SET_CHANNEL_1_DC:
			HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_1);
			if (setPWMDutyCycle(*((uint16_t*)args), PWM_CHANNEL_1) != RET_OK)
				ret = RET_ERROR;
			pwmPrivateData->channel_1_dc = *((uint16_t*)args);
			HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_1);
			break;
		case PWM_SET_CHANNEL_2_DC:
			HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_2);
			if (setPWMDutyCycle(*((uint16_t*)args), PWM_CHANNEL_2) != RET_OK)
				ret = RET_ERROR;
			pwmPrivateData->channel_2_dc = *((uint16_t*)args);
			HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_2);
			break;
		case PWM_SET_CHANNEL_3_DC:
			HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_3);
			if (setPWMDutyCycle(*((uint16_t*)args), PWM_CHANNEL_3) != RET_OK)
				ret = RET_ERROR;
			pwmPrivateData->channel_3_dc = *((uint16_t*)args);
			HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_3);
			break;

		default:
			ret = RET_ERROR;
			break;
		}
	}else{
		ret = RET_ERROR;
	}
	osMutexRelease(pwmPrivateData->openedFileMutex);
	return ret;
}

/**
 *
 * @param devHandler
 * @return
 */
static retval_t pwmChangedCpuFreq(devHandler_t* devHandler){
	return RET_OK;
}



static retval_t MX_PWM_Init(pwmDevicePrivateData_t* pwmPrivateData){
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_TIM3_CLK_ENABLE();


	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 32;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = HAL_RCC_GetHCLKFreq()/htim3.Init.Prescaler/PWM_DEFAULT_CARRIER_FREQ;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(validPWMPeriod() != RET_OK)
		return RET_ERROR;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
		return RET_ERROR;

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
		return RET_ERROR;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = getRawDutyCycle(pwmPrivateData->channel_1_dc);
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, PWM_CHANNEL_1) != HAL_OK)
		return RET_ERROR;
	sConfigOC.Pulse = getRawDutyCycle(pwmPrivateData->channel_2_dc);
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, PWM_CHANNEL_2) != HAL_OK)
		return RET_ERROR;
	sConfigOC.Pulse = getRawDutyCycle(pwmPrivateData->channel_3_dc);
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, PWM_CHANNEL_3) != HAL_OK)
		return RET_ERROR;


	/*MSP Init*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**TIM3 GPIO Configuration
	PB0     ------> TIM3_CH3
	PB1     ------> TIM3_CH4
	PB4 (NJTRST)     ------> TIM3_CH1
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	return RET_OK;
}
/**
 *
 */
static void MX_PWM_DeInit(){
	//HAL_ADC_DeInit(&hadc1);
}

//static retval_t validCarrierFreq(uint32_t value){
//	uint32_t aux = HAL_RCC_GetHCLKFreq()/htim3.Init.Prescaler/value;
//
//	if((aux < PWM_MIN_RESOLUTION) || (aux > PWM_MAX_RESOLUTION))
//		return RET_ERROR;
//
//	return RET_OK;
//}

static inline retval_t validPWMPeriod(void){
	return ((htim3.Init.Period < PWM_MIN_RESOLUTION) || (htim3.Init.Period > PWM_MAX_RESOLUTION)) ? RET_ERROR : RET_OK;
}

static inline uint32_t getRawDutyCycle(uint16_t dcValue){
	return (uint32_t)dcValue*htim3.Init.Period/(0xFFFF);
}

static retval_t setPWMDutyCycle(uint32_t value, pwmChannel_t channel){
	//sConfigOC.Pulse = HAL_RCC_GetHCLKFreq()/htim3.Init.Prescaler/value;
	sConfigOC.Pulse = getRawDutyCycle(value);
	return (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, channel) != HAL_OK) ? RET_ERROR : RET_OK;
}


InitDevice(pwmInit);
ExitDevice(pwmExit);

#endif
#endif
