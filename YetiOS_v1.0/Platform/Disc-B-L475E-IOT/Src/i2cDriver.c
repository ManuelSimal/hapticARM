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
 * i2cDriver.c
 *
 *  Created on: 4 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file i2cDriver.c
 */

#include "yetiOS.h"


#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#if (PLATFORM_I2C1_DEVICE_ID || PLATFORM_I2C2_DEVICE_ID)	/*Do not compile the dirver if not defined any of the IDs in the Conf file*/
#include "stm32l4xx_hal.h"
#include "platformLowPower.h"
#include "deviceDriver.h"
#include "lowPower.h"

#define I2C_DEFAULT_TIMEOUT		osWaitForever
#define MIN_NUM_BYTES_DMA		12		/*When reading or writing more than this nunmber of bytes, the DMA is used*/


#define STANDARD_MODE_100KHZ		0
#define FAST_MODE_400KHZ			1		/*Default Config 400 KHz*/
#define FAST_MODE_PLUS_1MHZ			2

/*This table contains the timing constants used by the I2C hardware. Each column corresponds to a speed
 * mode of the I2C (100 KHz, 400 KHz, 1 MHz). Each File corresponds to a CPU Frequency from 48 MHz to 1 MHz.
 * Values obtained from CubeMX*/
/*												100KHz		400KHz		1MHz				*/
static const uint32_t timingTable[8][3] = {	{0x20303E5D, 0x2010091A, 0x20000209},	/*48 MHz*/
											{0x00707CBB, 0x00300F38, 0x00100413},	/*32 MHz*/
											{0x00506682, 0x00200C28, 0x0010030D},	/*24 MHz*/
											{0x00303D5B, 0x0010061A, 0x00000107},	/*16 MHz*/
											{0x2000090E, 0x0000020B, 0x00000001},	/*8 MHz*/
											{0x00000E14, 0x00000004, 0x00000000},	/*4 MHz*/	/*I2C 1MHz not supported*/
											{0x00000509, 0x00000000, 0x00000000},	/*2 MHz*/	/*I2C 400KHz and 1MHz not supported*/
											{0x00000103, 0x00000000, 0x00000000}	/*1 MHz*/	/*I2C 400KHz and 1MHz not supported*/
										};
typedef struct i2cMemData_{
	uint16_t i2cMemAddress;
	uint16_t i2cMemAddSize;
	uint16_t bufSize;
}i2cMemRWData_t;

typedef struct i2cPrivateData_{
	I2C_HandleTypeDef* hi2c;
	DMA_HandleTypeDef* hdma_i2c_rx;
	DMA_HandleTypeDef* hdma_i2c_tx;
	osSemaphoreId i2cSemaphore;
	osMutexId i2cMutex;
	uint16_t i2cFreqMode;
	i2cMemRWData_t i2cRWdata;
	uint16_t i2cOpRunning;
}i2cDevicePrivateData_t;

typedef struct i2cFilePrivateData_{
	uint16_t slaveAddr;
}i2cFilePrivateData_t;

/*I2C and DMA HAL handles*/
#if PLATFORM_I2C1_DEVICE_ID
static devHandler_t* i2c1Dev;
#endif

#if PLATFORM_I2C2_DEVICE_ID
static devHandler_t* i2c2Dev;
#endif

/*I2C Driver Init and Exit*/
static retval_t i2cInit(void);
static retval_t i2cExit(void);

/*I2C DRIVER FUNCTIONS*/
static retval_t i2cOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t i2cClose(deviceFileHandler_t* devFile);
static uint32_t i2cRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t i2cWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t i2cIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t i2cChangedCpuFreq(devHandler_t* devHandler);

/*ADDITIONAL FUNCTIONS*/
static retval_t i2cSetFreqMode(devHandler_t* devHandler);
static uint32_t i2cGetTimingValue(I2C_TypeDef* i2cInstance, uint32_t currentCpuFreq, uint16_t i2cFreqMode);
static HAL_StatusTypeDef HAL_I2C_DMAStop(I2C_HandleTypeDef *hi2c);	/*This function should be implemented in the HAL but it is not. Implemented here*/
static uint32_t i2cMemRead(deviceFileHandler_t* devFile, uint16_t MemAddress, uint16_t MemAddSize, uint8_t* buff, uint16_t size);
static uint32_t i2cMemWrite(deviceFileHandler_t* devFile, uint16_t MemAddress, uint16_t MemAddSize, uint8_t* buff, uint16_t size);

static deviceDriverOps_t i2cDriverOps = {
	.open = i2cOpen,
	.close = i2cClose,
	.read = i2cRead,
	.write = i2cWrite,
	.ioctl = i2cIoctl,
	.changedCpuFreq = i2cChangedCpuFreq,
};


/**
 *
 * @return
 */
static retval_t i2cInit(void){
	i2cDevicePrivateData_t* i2cPrivateData;
#if PLATFORM_I2C1_DEVICE_ID
	i2c1Dev =  ytNewDevice(PLATFORM_I2C1_DEVICE_ID, &i2cDriverOps);
	i2cPrivateData = (i2cDevicePrivateData_t*)pvPortMalloc(sizeof(i2cDevicePrivateData_t));
	i2c1Dev->privateData = (void*) i2cPrivateData;
	i2cPrivateData->hi2c = (I2C_HandleTypeDef*)pvPortMalloc(sizeof(I2C_HandleTypeDef));
	i2cPrivateData->hdma_i2c_rx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));
	i2cPrivateData->hdma_i2c_tx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));

	/*Default I2C Configuration Init*/
	i2cPrivateData->hi2c->Instance = I2C1;
	i2cPrivateData->hi2c->Init.Timing = i2cGetTimingValue(i2cPrivateData->hi2c->Instance, platformGetCurrentRunMode(), FAST_MODE_400KHZ);
	i2cPrivateData->hi2c->Init.OwnAddress1 = 0;
	i2cPrivateData->hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2cPrivateData->hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2cPrivateData->hi2c->Init.OwnAddress2 = 0;
	i2cPrivateData->hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	i2cPrivateData->hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2cPrivateData->hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(i2cPrivateData->hi2c) != HAL_OK){
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
		return RET_ERROR;
	}
	if (HAL_I2CEx_ConfigAnalogFilter(i2cPrivateData->hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)	{
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
		return RET_ERROR;
	}
	if (HAL_I2CEx_ConfigDigitalFilter(i2cPrivateData->hi2c, 0) != HAL_OK){
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
		return RET_ERROR;
	}

	/*Store the references of the HAL handlers, mutexes and semaphores*/
	i2cPrivateData->i2cSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(i2cPrivateData->i2cSemaphore, osWaitForever);
	i2cPrivateData->i2cMutex = ytMutexCreate();
	i2cPrivateData->i2cOpRunning = 0;
	if(ytRegisterDevice(i2c1Dev) != RET_OK){
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		osMutexDelete(i2cPrivateData->i2cMutex);
		osSemaphoreDelete(i2cPrivateData->i2cSemaphore);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
		return RET_ERROR;
	}
#endif

	/* ***********************************************/

#if PLATFORM_I2C2_DEVICE_ID
	/*Default I2C2 Configuration Init*/
	i2c2Dev =  ytNewDevice(PLATFORM_I2C2_DEVICE_ID, &i2cDriverOps);
	i2cPrivateData = (i2cDevicePrivateData_t*)pvPortMalloc(sizeof(i2cDevicePrivateData_t));
	i2c2Dev->privateData = (void*) i2cPrivateData;
	i2cPrivateData->hi2c = (I2C_HandleTypeDef*)pvPortMalloc(sizeof(I2C_HandleTypeDef));
	i2cPrivateData->hdma_i2c_rx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));
	i2cPrivateData->hdma_i2c_tx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));

	/*Default I2C Configuration Init*/
	i2cPrivateData->hi2c->Instance = I2C2;
	i2cPrivateData->hi2c->Init.Timing = i2cGetTimingValue(i2cPrivateData->hi2c->Instance, platformGetCurrentRunMode(), FAST_MODE_400KHZ);
	i2cPrivateData->hi2c->Init.OwnAddress1 = 0;
	i2cPrivateData->hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2cPrivateData->hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2cPrivateData->hi2c->Init.OwnAddress2 = 0;
	i2cPrivateData->hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	i2cPrivateData->hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2cPrivateData->hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(i2cPrivateData->hi2c) != HAL_OK){
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c2Dev);
#if PLATFORM_I2C1_DEVICE_ID
		ytUnregisterDevice(PLATFORM_I2C1_DEVICE_ID);
		i2cPrivateData = (i2cDevicePrivateData_t*) i2c1Dev->privateData;
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		osMutexDelete(i2cPrivateData->i2cMutex);
		osSemaphoreDelete(i2cPrivateData->i2cSemaphore);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
#endif
		return RET_ERROR;
	}
	if (HAL_I2CEx_ConfigAnalogFilter(i2cPrivateData->hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)	{
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c2Dev);
#if PLATFORM_I2C1_DEVICE_ID
		ytUnregisterDevice(PLATFORM_I2C1_DEVICE_ID);
		i2cPrivateData = (i2cDevicePrivateData_t*) i2c1Dev->privateData;
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		osMutexDelete(i2cPrivateData->i2cMutex);
		osSemaphoreDelete(i2cPrivateData->i2cSemaphore);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
#endif
		return RET_ERROR;
	}
	if (HAL_I2CEx_ConfigDigitalFilter(i2cPrivateData->hi2c, 0) != HAL_OK){
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c2Dev);
#if PLATFORM_I2C1_DEVICE_ID
		ytUnregisterDevice(PLATFORM_I2C1_DEVICE_ID);
		i2cPrivateData = (i2cDevicePrivateData_t*) i2c1Dev->privateData;
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		osMutexDelete(i2cPrivateData->i2cMutex);
		osSemaphoreDelete(i2cPrivateData->i2cSemaphore);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
#endif
		return RET_ERROR;
	}

	/*Store the references of the HAL handlers, mutexes and semaphores*/
	i2cPrivateData->i2cSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(i2cPrivateData->i2cSemaphore, osWaitForever);
	i2cPrivateData->i2cMutex = ytMutexCreate();
	i2cPrivateData->i2cOpRunning = 0;
	if(ytRegisterDevice(i2c2Dev) != RET_OK){
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		osMutexDelete(i2cPrivateData->i2cMutex);
		osSemaphoreDelete(i2cPrivateData->i2cSemaphore);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c2Dev);
#if PLATFORM_I2C1_DEVICE_ID
		ytUnregisterDevice(PLATFORM_I2C1_DEVICE_ID);
		i2cPrivateData = (i2cDevicePrivateData_t*) i2c1Dev->privateData;
		HAL_I2C_DeInit(i2cPrivateData->hi2c);
		osMutexDelete(i2cPrivateData->i2cMutex);
		osSemaphoreDelete(i2cPrivateData->i2cSemaphore);
		vPortFree(i2cPrivateData->hdma_i2c_rx);
		vPortFree(i2cPrivateData->hdma_i2c_tx);
		vPortFree(i2cPrivateData->hi2c);
		vPortFree(i2cPrivateData);
		ytDeleteDevice(i2c1Dev);
#endif
		return RET_ERROR;
	}
#endif

	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t i2cExit(void){
	i2cDevicePrivateData_t* i2cDevicePrivateData;
	/*Unregister and Delete I2C1*/
#if PLATFORM_I2C1_DEVICE_ID
	i2cDevicePrivateData = (i2cDevicePrivateData_t*) i2c1Dev->privateData;
	ytUnregisterDevice(PLATFORM_I2C1_DEVICE_ID);

	HAL_I2C_DeInit(i2cDevicePrivateData->hi2c);
	vPortFree(i2cDevicePrivateData->hi2c);
	vPortFree(i2cDevicePrivateData->hdma_i2c_rx);
	vPortFree(i2cDevicePrivateData->hdma_i2c_tx);

	osMutexDelete(i2cDevicePrivateData->i2cMutex);
	osSemaphoreDelete(i2cDevicePrivateData->i2cSemaphore);
	vPortFree(i2cDevicePrivateData);
	ytDeleteDevice(i2c1Dev);
#endif

	/*Unregister and Delete I2C2*/
#if PLATFORM_I2C2_DEVICE_ID
	i2cDevicePrivateData = (i2cDevicePrivateData_t*) i2c2Dev->privateData;
	ytUnregisterDevice(PLATFORM_I2C2_DEVICE_ID);

	HAL_I2C_DeInit(i2cDevicePrivateData->hi2c);
	vPortFree(i2cDevicePrivateData->hi2c);
	vPortFree(i2cDevicePrivateData->hdma_i2c_rx);
	vPortFree(i2cDevicePrivateData->hdma_i2c_tx);

	osMutexDelete(i2cDevicePrivateData->i2cMutex);
	osSemaphoreDelete(i2cDevicePrivateData->i2cSemaphore);
	vPortFree(i2cDevicePrivateData);
	ytDeleteDevice(i2c2Dev);
#endif
	return RET_OK;
}



/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t i2cOpen(deviceFileHandler_t* devFile, uint32_t flags){
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	i2cFilePrivateData_t* i2cFilePrivateData = (i2cFilePrivateData_t*)pvPortMalloc(sizeof(i2cFilePrivateData_t));
	i2cFilePrivateData->slaveAddr = (uint16_t) flags;
	devFile->privateData = (void*) i2cFilePrivateData;
	osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return RET_OK;
}


static retval_t i2cClose(deviceFileHandler_t* devFile){
	i2cFilePrivateData_t* i2cFilePrivateData = (i2cFilePrivateData_t*) devFile->privateData;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	if(i2cDevicePrivateData->i2cOpRunning){
		if((i2cDevicePrivateData->hi2c->Instance == I2C1) &&  (i2cDevicePrivateData->hdma_i2c_rx->Init.Mode == DMA_CIRCULAR)){
			HAL_I2C_DMAStop(i2cDevicePrivateData->hi2c);
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			ytSetDeviceDefaultLowPowerMode();
			i2cDevicePrivateData->i2cOpRunning = 0;
		}
		else{
			osMutexRelease(i2cDevicePrivateData->i2cMutex);
			return RET_ERROR;
		}
	}
	vPortFree(i2cFilePrivateData);
	osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return RET_OK;
}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t i2cRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){

	i2cFilePrivateData_t* i2cFilePrivateData = (i2cFilePrivateData_t*) devFile->privateData;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	if(i2cDevicePrivateData->i2cOpRunning){
		osMutexRelease(i2cDevicePrivateData->i2cMutex);
		return 0;
	}
	i2cDevicePrivateData->i2cOpRunning++;

	if(size <= MIN_NUM_BYTES_DMA){
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		if (HAL_I2C_Master_Receive(i2cDevicePrivateData->hi2c, i2cFilePrivateData->slaveAddr, buff, size, I2C_DEFAULT_TIMEOUT) != HAL_OK){
			i2cDevicePrivateData->i2cOpRunning = 0;
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			osMutexRelease(i2cDevicePrivateData->i2cMutex);
			return 0;
		}
		__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
		i2cDevicePrivateData->i2cOpRunning = 0;
	}
	else{
		ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		HAL_I2C_Master_Receive_DMA(i2cDevicePrivateData->hi2c, i2cFilePrivateData->slaveAddr, buff, size);
		if(i2cDevicePrivateData->hdma_i2c_rx->Init.Mode == DMA_NORMAL){
			if(osSemaphoreWait(i2cDevicePrivateData->i2cSemaphore, I2C_DEFAULT_TIMEOUT) != RET_OK){
				ytSetDeviceDefaultLowPowerMode();
				i2cDevicePrivateData->i2cOpRunning = 0;
				__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
				osMutexRelease(i2cDevicePrivateData->i2cMutex);
				return 0;
			}
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			i2cDevicePrivateData->i2cOpRunning = 0;
			ytSetDeviceDefaultLowPowerMode();
		}
	}
	osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return size;
}


/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t i2cWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){

	i2cFilePrivateData_t* i2cFilePrivateData = (i2cFilePrivateData_t*) devFile->privateData;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	if(i2cDevicePrivateData->i2cOpRunning){
		osMutexRelease(i2cDevicePrivateData->i2cMutex);
		return 0;
	}
	i2cDevicePrivateData->i2cOpRunning++;

	if(size <= MIN_NUM_BYTES_DMA){
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		if (HAL_I2C_Master_Transmit(i2cDevicePrivateData->hi2c, 0x90, buff, size, I2C_DEFAULT_TIMEOUT) != HAL_OK){
			i2cDevicePrivateData->i2cOpRunning = 0;
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			osMutexRelease(i2cDevicePrivateData->i2cMutex);
			return 0;
		}
		__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
		i2cDevicePrivateData->i2cOpRunning = 0;
	}
	else{
		ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		HAL_I2C_Master_Transmit_DMA(i2cDevicePrivateData->hi2c, i2cFilePrivateData->slaveAddr, buff, size);
		if(i2cDevicePrivateData->hdma_i2c_rx->Init.Mode == DMA_NORMAL){
			if(osSemaphoreWait(i2cDevicePrivateData->i2cSemaphore, I2C_DEFAULT_TIMEOUT) != RET_OK){
				ytSetDeviceDefaultLowPowerMode();
				i2cDevicePrivateData->i2cOpRunning = 0;
				__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
				osMutexRelease(i2cDevicePrivateData->i2cMutex);
				return 0;
			}
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			i2cDevicePrivateData->i2cOpRunning = 0;
			ytSetDeviceDefaultLowPowerMode();
		}
	}

	osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return size;
}


/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t i2cIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	retval_t ret = RET_ERROR;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devFile->device->privateData;

	osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	switch(command){

	case SET_ASYNC_CIRCULAR_READ_MODE:
		if(i2cDevicePrivateData->i2cOpRunning){
			break;
		}
		i2cDevicePrivateData->hdma_i2c_rx->Init.Mode = DMA_CIRCULAR;
		HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_rx);
		i2cDevicePrivateData->hdma_i2c_tx->Init.Mode = DMA_CIRCULAR;
		HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_tx);
		ret = RET_OK;
		break;
	case STOP_ASYNC_READ:
		if(i2cDevicePrivateData->i2cOpRunning){
			if(i2cDevicePrivateData->hdma_i2c_rx->Init.Mode == DMA_CIRCULAR){
				HAL_I2C_DMAStop(i2cDevicePrivateData->hi2c);
				__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
				i2cDevicePrivateData->i2cOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
				ret = RET_OK;
			}
		}
		break;
	case GET_CURRENT_REMAINING_COUNT:
		if(i2cDevicePrivateData->i2cOpRunning){
			if(i2cDevicePrivateData->hdma_i2c_rx->Init.Mode == DMA_CIRCULAR){
				(*((uint32_t*)args)) =__HAL_DMA_GET_COUNTER(i2cDevicePrivateData->hdma_i2c_rx);
				ret = RET_OK;
			}
		}
		break;
	case SET_STD_READ_MODE:
		if(i2cDevicePrivateData->i2cOpRunning){
			break;
		}
		i2cDevicePrivateData->hdma_i2c_rx->Init.Mode = DMA_NORMAL;
		HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_rx);
		i2cDevicePrivateData->hdma_i2c_tx->Init.Mode = DMA_NORMAL;
		HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_tx);
		ret = RET_OK;
		break;

	case I2C_SET_FREQ_MODE:
		if(i2cDevicePrivateData->i2cOpRunning){
			break;
		}
		if(*((uint16_t*) args) > FAST_MODE_PLUS_1MHZ){
			ret = RET_ERROR;
			break;
		}
		i2cDevicePrivateData->i2cFreqMode = *((uint16_t*) args);	/*Store the desired speed*/
		ret = i2cSetFreqMode(devFile->device);						/*Set the new Speed*/
		break;

	case I2C_MEM_RW_SET_REG_ADD:
		i2cDevicePrivateData->i2cRWdata.i2cMemAddress = *((uint16_t*) args);
		break;

	case I2C_MEM_RW_SET_REG_ADD_SIZE:
		i2cDevicePrivateData->i2cRWdata.i2cMemAddSize = *((uint16_t*) args);
		break;

	case I2C_MEM_RW_SET_BUF_SIZE:
		i2cDevicePrivateData->i2cRWdata.bufSize = *((uint16_t*) args);
		break;

	case I2C_MEM_READ:
		i2cMemRead(devFile, i2cDevicePrivateData->i2cRWdata.i2cMemAddress, (uint16_t) i2cDevicePrivateData->i2cRWdata.i2cMemAddSize,
				((uint8_t*) args), i2cDevicePrivateData->i2cRWdata.bufSize);
		break;
	case I2C_MEM_WRITE:
		i2cMemWrite(devFile, i2cDevicePrivateData->i2cRWdata.i2cMemAddress, (uint16_t) i2cDevicePrivateData->i2cRWdata.i2cMemAddSize,
				((uint8_t*) args), i2cDevicePrivateData->i2cRWdata.bufSize);
		break;
	default:
		ret = RET_ERROR;
		break;
	}

	osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return ret;
}



/**
 *
 * @param devFile
 * @return
 */
static retval_t i2cChangedCpuFreq(devHandler_t* devHandler){
	retval_t ret = RET_OK;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devHandler->privateData;
	/*This function is called by the OS when the CPU clock frequency changes to keep an appropiate peripheral configuration*/
	osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	ret = i2cSetFreqMode(devHandler);	/*Use an adequate timing value for the new system frequency*/
	osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return ret;
}




/**
 *
 * @param devHandler
 * @return
 */
static retval_t i2cSetFreqMode(devHandler_t* devHandler){
	retval_t ret = RET_OK;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devHandler->privateData;
	i2cDevicePrivateData->hi2c->Init.Timing = i2cGetTimingValue(i2cDevicePrivateData->hi2c->Instance, platformGetCurrentRunMode(), i2cDevicePrivateData->i2cFreqMode);
	if (HAL_I2C_Init(i2cDevicePrivateData->hi2c) != HAL_OK){
		ret = RET_ERROR;
	}
	return ret;
}

/**
 *
 * @param currentCpuFreq
 * @param i2cFreqMode
 * @return
 */
static uint32_t i2cGetTimingValue(I2C_TypeDef* i2cInstance, uint32_t currentCpuFreq, uint16_t i2cFreqMode){
	if(i2cFreqMode > STANDARD_MODE_100KHZ){
		if(currentCpuFreq > RUN_MODE_4MHZ){	/*Only 100 KHz in this cases is supported*/
			if(i2cInstance == I2C1){
				HAL_I2CEx_DisableFastModePlus(I2C_FASTMODEPLUS_I2C1);
			}
			else if(i2cInstance == I2C2){
				HAL_I2CEx_DisableFastModePlus(I2C_FASTMODEPLUS_I2C2);
			}
			return timingTable[currentCpuFreq][0];
		}
	}

	if((i2cFreqMode == FAST_MODE_PLUS_1MHZ) && (currentCpuFreq == RUN_MODE_4MHZ)){	// Supported <= 400KHz
		if(i2cInstance == I2C1){
			HAL_I2CEx_DisableFastModePlus(I2C_FASTMODEPLUS_I2C1);
		}
		else if(i2cInstance == I2C2){
			HAL_I2CEx_DisableFastModePlus(I2C_FASTMODEPLUS_I2C2);
		}
		return timingTable[currentCpuFreq][1];
	}


	if(i2cFreqMode == FAST_MODE_PLUS_1MHZ){
		if(i2cInstance == I2C1){
			HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
		}
		else if(i2cInstance == I2C2){
			HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
		}
	}
	else{
		if(i2cInstance == I2C1){
			HAL_I2CEx_DisableFastModePlus(I2C_FASTMODEPLUS_I2C1);
		}
		else if(i2cInstance == I2C2){
			HAL_I2CEx_DisableFastModePlus(I2C_FASTMODEPLUS_I2C2);
		}
	}
	return timingTable[currentCpuFreq][i2cFreqMode];

}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t i2cMemRead(deviceFileHandler_t* devFile, uint16_t MemAddress, uint16_t MemAddSize, uint8_t* buff, uint16_t size){
	i2cFilePrivateData_t* i2cFilePrivateData = (i2cFilePrivateData_t*) devFile->privateData;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devFile->device->privateData;
	//osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	if(i2cDevicePrivateData->i2cOpRunning){
		//osMutexRelease(i2cDevicePrivateData->i2cMutex);
		return 0;
	}
	i2cDevicePrivateData->i2cOpRunning++;

	if(size <= MIN_NUM_BYTES_DMA){
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		if (HAL_I2C_Mem_Read(i2cDevicePrivateData->hi2c, i2cFilePrivateData->slaveAddr, MemAddress, MemAddSize, buff, size, I2C_DEFAULT_TIMEOUT) != HAL_OK){
			i2cDevicePrivateData->i2cOpRunning = 0;
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			//osMutexRelease(i2cDevicePrivateData->i2cMutex);
			return 0;
		}
		__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
		i2cDevicePrivateData->i2cOpRunning = 0;
	}
	else{
		ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		HAL_I2C_Mem_Read_DMA(i2cDevicePrivateData->hi2c, i2cFilePrivateData->slaveAddr, MemAddress, MemAddSize, buff, size);
		if(i2cDevicePrivateData->hdma_i2c_rx->Init.Mode == DMA_NORMAL){
			if(osSemaphoreWait(i2cDevicePrivateData->i2cSemaphore, I2C_DEFAULT_TIMEOUT) != RET_OK){
				ytSetDeviceDefaultLowPowerMode();
				i2cDevicePrivateData->i2cOpRunning = 0;
				__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
				//osMutexRelease(i2cDevicePrivateData->i2cMutex);
				return 0;
			}
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			i2cDevicePrivateData->i2cOpRunning = 0;
			ytSetDeviceDefaultLowPowerMode();
		}
	}
	//osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return size;
}


/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t i2cMemWrite(deviceFileHandler_t* devFile, uint16_t MemAddress, uint16_t MemAddSize, uint8_t* buff, uint16_t size){

	i2cFilePrivateData_t* i2cFilePrivateData = (i2cFilePrivateData_t*) devFile->privateData;
	i2cDevicePrivateData_t* i2cDevicePrivateData = (i2cDevicePrivateData_t*) devFile->device->privateData;
	//osMutexWait(i2cDevicePrivateData->i2cMutex, osWaitForever);
	if(i2cDevicePrivateData->i2cOpRunning){
		//osMutexRelease(i2cDevicePrivateData->i2cMutex);
		return 0;
	}
	i2cDevicePrivateData->i2cOpRunning++;

	if(size <= MIN_NUM_BYTES_DMA){
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		if (HAL_I2C_Mem_Write(i2cDevicePrivateData->hi2c, i2cFilePrivateData->slaveAddr, MemAddress, MemAddSize, buff, size, I2C_DEFAULT_TIMEOUT) != HAL_OK){
			i2cDevicePrivateData->i2cOpRunning = 0;
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			//osMutexRelease(i2cDevicePrivateData->i2cMutex);
			return 0;
		}
		__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
		i2cDevicePrivateData->i2cOpRunning = 0;
	}
	else{
		ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
		__HAL_I2C_ENABLE(i2cDevicePrivateData->hi2c);
		HAL_I2C_Mem_Write_DMA(i2cDevicePrivateData->hi2c, i2cFilePrivateData->slaveAddr, MemAddress, MemAddSize, buff, size);
		if(i2cDevicePrivateData->hdma_i2c_rx->Init.Mode == DMA_NORMAL){
			if(osSemaphoreWait(i2cDevicePrivateData->i2cSemaphore, I2C_DEFAULT_TIMEOUT) != RET_OK){
				ytSetDeviceDefaultLowPowerMode();
				i2cDevicePrivateData->i2cOpRunning = 0;
				__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
				//osMutexRelease(i2cDevicePrivateData->i2cMutex);
				return 0;
			}
			__HAL_I2C_DISABLE(i2cDevicePrivateData->hi2c);
			i2cDevicePrivateData->i2cOpRunning = 0;
			ytSetDeviceDefaultLowPowerMode();
		}
	}

	//osMutexRelease(i2cDevicePrivateData->i2cMutex);
	return size;
}


/**
 * @brief			This function is not implemented in the i2c HAL, so I implement it here
 * @param hspi
 * @return
 */
static HAL_StatusTypeDef HAL_I2C_DMAStop(I2C_HandleTypeDef *hi2c){
  HAL_StatusTypeDef errorcode = HAL_OK;

  /* Abort the I2C DMA tx Stream/Channel  */
  if (hi2c->hdmatx != NULL)
  {
    if (HAL_OK != HAL_DMA_Abort(hi2c->hdmatx))
    {
      SET_BIT(hi2c->ErrorCode, HAL_I2C_ERROR_DMA);
      errorcode = HAL_ERROR;
    }
  }
  /* Abort the SPI DMA rx Stream/Channel  */
  if (hi2c->hdmarx != NULL)
  {
    if (HAL_OK != HAL_DMA_Abort(hi2c->hdmarx))
    {
      SET_BIT(hi2c->ErrorCode, HAL_I2C_ERROR_DMA);
      errorcode = HAL_ERROR;
    }
  }

  /* Disable the SPI DMA Tx & Rx requests */
  CLEAR_BIT(hi2c->Instance->CR2, I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN);
  hi2c->State = HAL_I2C_STATE_READY;
  return errorcode;
}


/**
 *
 * @param hi2c
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
#if PLATFORM_I2C1_DEVICE_ID
		osSemaphoreRelease(((i2cDevicePrivateData_t*) i2c1Dev->privateData)->i2cSemaphore);
#endif
	}
	if(hi2c->Instance == I2C2){
#if PLATFORM_I2C2_DEVICE_ID
		osSemaphoreRelease(((i2cDevicePrivateData_t*) i2c2Dev->privateData)->i2cSemaphore);
#endif
	}
}


/**
 *
 * @param hi2c
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
#if PLATFORM_I2C1_DEVICE_ID
		osSemaphoreRelease(((i2cDevicePrivateData_t*) i2c1Dev->privateData)->i2cSemaphore);
#endif
	}
	if(hi2c->Instance == I2C2){
#if PLATFORM_I2C2_DEVICE_ID
		osSemaphoreRelease(((i2cDevicePrivateData_t*) i2c2Dev->privateData)->i2cSemaphore);
#endif
	}
}


/**
 *
 * @param i2cHandle
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  i2cDevicePrivateData_t* i2cDevicePrivateData;

  if(i2cHandle->Instance==I2C1)
  {
#if PLATFORM_I2C1_DEVICE_ID
    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* DMA ENABLE*/
	__HAL_RCC_DMA2_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA2_Channel6_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA2_Channel7_IRQn);

    /**I2C1 GPIO Configuration*/
    GPIO_InitStruct.Pin = PLATFORM_I2C1_SCL_PIN|PLATFORM_I2C1_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(PLATFORM_I2C1_PIN_PORT, &GPIO_InitStruct);

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    i2cDevicePrivateData = (i2cDevicePrivateData_t*) i2c1Dev->privateData;

    i2cDevicePrivateData->hdma_i2c_rx->Instance = DMA2_Channel6;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Request = DMA_REQUEST_5;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    i2cDevicePrivateData->hdma_i2c_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.MemInc = DMA_MINC_ENABLE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Mode = DMA_NORMAL;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_rx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,*(i2cDevicePrivateData->hdma_i2c_rx));

    /* I2C1_TX Init */
    i2cDevicePrivateData->hdma_i2c_tx->Instance = DMA2_Channel7;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Request = DMA_REQUEST_5;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    i2cDevicePrivateData->hdma_i2c_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.MemInc = DMA_MINC_ENABLE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Mode = DMA_NORMAL;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_tx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,*(i2cDevicePrivateData->hdma_i2c_tx));

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
#endif
  }
  else if(i2cHandle->Instance==I2C2)
  {											/*I2C2 does not support DMA*/
#if PLATFORM_I2C2_DEVICE_ID
    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* DMA ENABLE*/
	__HAL_RCC_DMA1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);

    GPIO_InitStruct.Pin = PLATFORM_I2C2_SCL_PIN|PLATFORM_I2C2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(PLATFORM_I2C2_PIN_PORT, &GPIO_InitStruct);

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    i2cDevicePrivateData = (i2cDevicePrivateData_t*) i2c2Dev->privateData;

    i2cDevicePrivateData->hdma_i2c_rx->Instance = DMA1_Channel5;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Request = DMA_REQUEST_5;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    i2cDevicePrivateData->hdma_i2c_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.MemInc = DMA_MINC_ENABLE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Mode = DMA_NORMAL;
    i2cDevicePrivateData->hdma_i2c_rx->Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_rx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,*(i2cDevicePrivateData->hdma_i2c_rx));

    /* I2C1_TX Init */
    i2cDevicePrivateData->hdma_i2c_tx->Instance = DMA1_Channel4;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Request = DMA_REQUEST_5;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    i2cDevicePrivateData->hdma_i2c_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.MemInc = DMA_MINC_ENABLE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Mode = DMA_NORMAL;
    i2cDevicePrivateData->hdma_i2c_tx->Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(i2cDevicePrivateData->hdma_i2c_tx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,*(i2cDevicePrivateData->hdma_i2c_tx));

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

#endif
  }
}

/**
 *
 * @param i2cHandle
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
#if PLATFORM_I2C1_DEVICE_ID
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    HAL_GPIO_DeInit(PLATFORM_I2C1_PIN_PORT, PLATFORM_I2C1_SCL_PIN|PLATFORM_I2C1_SDA_PIN);

    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);

    /* DMA DISABLE*/
	HAL_NVIC_DisableIRQ(DMA2_Channel6_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Channel7_IRQn);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

#endif
  }
  else if(i2cHandle->Instance==I2C2)
  {
#if PLATFORM_I2C2_DEVICE_ID
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    HAL_GPIO_DeInit(PLATFORM_I2C2_PIN_PORT, PLATFORM_I2C2_SCL_PIN|PLATFORM_I2C2_SDA_PIN);

    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);

    /* DMA DISABLE*/
	HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);
#endif
  }
}

#if PLATFORM_I2C1_DEVICE_ID
/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA2_Channel6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((i2cDevicePrivateData_t*) i2c1Dev->privateData)->hdma_i2c_rx);
}


void DMA2_Channel7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((i2cDevicePrivateData_t*) i2c1Dev->privateData)->hdma_i2c_tx);
}


void I2C1_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(((i2cDevicePrivateData_t*) i2c1Dev->privateData)->hi2c);
}
#endif

#if PLATFORM_I2C2_DEVICE_ID

void DMA1_Channel5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((i2cDevicePrivateData_t*) i2c2Dev->privateData)->hdma_i2c_rx);
}


void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((i2cDevicePrivateData_t*) i2c2Dev->privateData)->hdma_i2c_tx);
}

void I2C2_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(((i2cDevicePrivateData_t*) i2c2Dev->privateData)->hi2c);
}
#endif


InitDevice(i2cInit);
ExitDevice(i2cExit);
#endif

#endif
