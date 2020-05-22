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
 * spiDriver.c
 *
 *  Created on: 26 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file spiDriver.c
 */
#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "stm32l4xx_hal.h"

#if (PLATFORM_SPI1_DEVICE_ID || PLATFORM_SPI3_DEVICE_ID)	/*Do not compile the driver if not defined any of the IDs in the Conf file*/
#include "deviceDriver.h"
#include "lowPower.h"

#define SPI_DEFAULT_TIMEOUT		osWaitForever
#define MIN_NUM_BYTES_DMA		12		/*When reading or writing more than this number of bytes, the DMA is used*/

typedef struct spiPrivateData_{
	SPI_HandleTypeDef* hspi;
	DMA_HandleTypeDef* hdma_spi_rx;
	DMA_HandleTypeDef* hdma_spi_tx;
	osSemaphoreId spiSemaphore;
	osMutexId spiMutex;
	uint32_t spiClkFreq;
	uint16_t spiOpRunning;
}spiDevicePrivateData_t;

typedef struct spiFilePrivateData_{
	uint32_t gpioCs;
	spiSwCsFunc_t enableSwCs;
}spiFilePrivateData_t;

/*SPI and DMA HAL handles*/
#if PLATFORM_SPI1_DEVICE_ID
static devHandler_t* spi1Dev;
#endif

#if PLATFORM_SPI3_DEVICE_ID
static devHandler_t* spi3Dev;
#endif

/*SPI Driver Init and Exit*/
static retval_t spiInit(void);
static retval_t spiExit(void);

/*SPI DRIVER FUNCTIONS*/
static retval_t spiOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t spiClose(deviceFileHandler_t* devFile);
static uint32_t spiRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t spiWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t spiIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t spiChangedCpuFreq(devHandler_t* devHandler);

/*ADDITIONAL FUNCTIONS*/
static retval_t spiReadWrite(deviceFileHandler_t*  devFile, uint8_t* ptx, uint8_t* prx, uint32_t size);
static retval_t spiSetSpeed(devHandler_t*  devHandler);


static deviceDriverOps_t spiDriverOps = {
	.open = spiOpen,
	.close = spiClose,
	.read = spiRead,
	.write = spiWrite,
	.ioctl = spiIoctl,
	.changedCpuFreq = spiChangedCpuFreq,
};



/**
 *
 * @return
 */
static retval_t spiInit(void){
	spiDevicePrivateData_t* spiPrivateData;
#if PLATFORM_SPI1_DEVICE_ID
	spi1Dev =  ytNewDevice(PLATFORM_SPI1_DEVICE_ID, &spiDriverOps);
	spiPrivateData = (spiDevicePrivateData_t*)pvPortMalloc(sizeof(spiDevicePrivateData_t));
	spi1Dev->privateData = (void*) spiPrivateData;
	spiPrivateData->hspi = (SPI_HandleTypeDef*)pvPortMalloc(sizeof(SPI_HandleTypeDef));
	spiPrivateData->hdma_spi_rx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));
	spiPrivateData->hdma_spi_tx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));

	/*Default SPI1 Configuration Init*/
	spiPrivateData->hspi->Instance = SPI1;
	spiPrivateData->hspi->Init.Mode = SPI_MODE_MASTER;
	spiPrivateData->hspi->Init.Direction = SPI_DIRECTION_2LINES;
	spiPrivateData->hspi->Init.DataSize = SPI_DATASIZE_8BIT;
	spiPrivateData->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	spiPrivateData->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
	spiPrivateData->hspi->Init.NSS = SPI_NSS_SOFT;
	spiPrivateData->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	spiPrivateData->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	spiPrivateData->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
	spiPrivateData->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spiPrivateData->hspi->Init.CRCPolynomial = 7;
	spiPrivateData->hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	spiPrivateData->hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(spiPrivateData->hspi) != HAL_OK){
		vPortFree(spiPrivateData->hspi);
		vPortFree(spiPrivateData);
		ytDeleteDevice(spi1Dev);
		return RET_ERROR;
	}
	/*Store the references of the HAL handlers, mutexes and semaphores*/
	spiPrivateData->spiSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(spiPrivateData->spiSemaphore, osWaitForever);
	spiPrivateData->spiMutex = ytMutexCreate();
	spiPrivateData->spiOpRunning = 0;
	if(ytRegisterDevice(spi1Dev) != RET_OK){
		HAL_SPI_DeInit(spiPrivateData->hspi);
		osMutexDelete(spiPrivateData->spiMutex);
		osSemaphoreDelete(spiPrivateData->spiSemaphore);
		vPortFree(spiPrivateData->hdma_spi_rx);
		vPortFree(spiPrivateData->hdma_spi_tx);
		vPortFree(spiPrivateData->hspi);
		vPortFree(spiPrivateData);
		ytDeleteDevice(spi1Dev);
		return RET_ERROR;
	}
#endif

	/* ***********************************************/

#if PLATFORM_SPI3_DEVICE_ID
	/*Default SPI3 Configuration Init*/
	spi3Dev =  ytNewDevice(PLATFORM_SPI3_DEVICE_ID, &spiDriverOps);
	spiPrivateData = (spiDevicePrivateData_t*)pvPortMalloc(sizeof(spiDevicePrivateData_t));
	spi3Dev->privateData = (void*) spiPrivateData;
	spiPrivateData->hspi = (SPI_HandleTypeDef*)pvPortMalloc(sizeof(SPI_HandleTypeDef));
	spiPrivateData->hdma_spi_rx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));
	spiPrivateData->hdma_spi_tx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));

	spiPrivateData->hspi->Instance = SPI3;
	spiPrivateData->hspi->Init.Mode = SPI_MODE_MASTER;
	spiPrivateData->hspi->Init.Direction = SPI_DIRECTION_2LINES;
	spiPrivateData->hspi->Init.DataSize = SPI_DATASIZE_8BIT;
	spiPrivateData->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	spiPrivateData->hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
	spiPrivateData->hspi->Init.NSS = SPI_NSS_SOFT;
	spiPrivateData->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	spiPrivateData->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	spiPrivateData->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
	spiPrivateData->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spiPrivateData->hspi->Init.CRCPolynomial = 7;
	spiPrivateData->hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	spiPrivateData->hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(spiPrivateData->hspi) != HAL_OK)
	{
		vPortFree(spiPrivateData->hdma_spi_rx);
		vPortFree(spiPrivateData->hdma_spi_tx);
		vPortFree(spiPrivateData->hspi);
		vPortFree(spiPrivateData);
		ytDeleteDevice(spi3Dev);
#if PLATFORM_SPI1_DEVICE_ID
		ytUnregisterDevice(PLATFORM_SPI1_DEVICE_ID);
		spiPrivateData = (spiDevicePrivateData_t*) spi1Dev->privateData;
		HAL_SPI_DeInit(spiPrivateData->hspi);
		osMutexDelete(spiPrivateData->spiMutex);	/*Remove also SPI1*/
		osSemaphoreDelete(((spiDevicePrivateData_t*)(spi1Dev->privateData))->spiSemaphore);
		vPortFree(spiPrivateData->hdma_spi_rx);
		vPortFree(spiPrivateData->hdma_spi_tx);
		vPortFree(spiPrivateData->hspi);
		vPortFree(spiPrivateData);
		ytDeleteDevice(spi1Dev);
#endif
		return RET_ERROR;
	}
	/*Store the references of the HAL handlers, mutexes and semaphores*/
	spiPrivateData->spiSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(spiPrivateData->spiSemaphore, osWaitForever);
	spiPrivateData->spiMutex = ytMutexCreate();
	spiPrivateData->spiOpRunning = 0;
	if(ytRegisterDevice(spi3Dev) != RET_OK){
		HAL_SPI_DeInit(spiPrivateData->hspi);
		osMutexDelete(spiPrivateData->spiMutex);
		osSemaphoreDelete(spiPrivateData->spiSemaphore);
		vPortFree(spiPrivateData->hdma_spi_rx);
		vPortFree(spiPrivateData->hdma_spi_tx);
		vPortFree(spiPrivateData->hspi);
		vPortFree(spiPrivateData);
		ytDeleteDevice(spi3Dev);
#if PLATFORM_SPI1_DEVICE_ID
		ytUnregisterDevice(PLATFORM_SPI1_DEVICE_ID);
		spiPrivateData = (spiDevicePrivateData_t*) spi1Dev->privateData;
		HAL_SPI_DeInit(spiPrivateData->hspi);
		osMutexDelete(spiPrivateData->spiMutex);	/*Remove also SPI 1*/
		osSemaphoreDelete(((spiDevicePrivateData_t*)(spi1Dev->privateData))->spiSemaphore);
		vPortFree(spiPrivateData->hdma_spi_rx);
		vPortFree(spiPrivateData->hdma_spi_tx);
		vPortFree(spiPrivateData->hspi);
		vPortFree(spiPrivateData);
		ytDeleteDevice(spi1Dev);
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
static retval_t spiExit(void){
	spiDevicePrivateData_t* spiDevicePrivateData;
	/*Unregister and Delete SPI1*/
#if PLATFORM_SPI1_DEVICE_ID
	spiDevicePrivateData = (spiDevicePrivateData_t*) spi1Dev->privateData;
	ytUnregisterDevice(PLATFORM_SPI1_DEVICE_ID);

	HAL_SPI_DeInit(spiDevicePrivateData->hspi);
	vPortFree(spiDevicePrivateData->hspi);
	vPortFree(spiDevicePrivateData->hdma_spi_rx);
	vPortFree(spiDevicePrivateData->hdma_spi_tx);

	osMutexDelete(spiDevicePrivateData->spiMutex);
	osSemaphoreDelete(spiDevicePrivateData->spiSemaphore);
	vPortFree(spiDevicePrivateData);
	ytDeleteDevice(spi1Dev);
#endif

	/*Unregister and Delete SPI3*/
#if PLATFORM_SPI3_DEVICE_ID
	spiDevicePrivateData = (spiDevicePrivateData_t*) spi3Dev->privateData;
	ytUnregisterDevice(PLATFORM_SPI3_DEVICE_ID);

	HAL_SPI_DeInit(spiDevicePrivateData->hspi);
	vPortFree(spiDevicePrivateData->hspi);
	vPortFree(spiDevicePrivateData->hdma_spi_rx);
	vPortFree(spiDevicePrivateData->hdma_spi_tx);

	osMutexDelete(spiDevicePrivateData->spiMutex);
	osSemaphoreDelete(spiDevicePrivateData->spiSemaphore);
	vPortFree(spiDevicePrivateData);
	ytDeleteDevice(spi3Dev);
#endif
	return RET_OK;
}



/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t spiOpen(deviceFileHandler_t* devFile, uint32_t flags){
	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(spiDevicePrivateData->spiMutex, osWaitForever);
	spiFilePrivateData_t* spiFilePrivateData = (spiFilePrivateData_t*)pvPortMalloc(sizeof(spiFilePrivateData_t));
	spiFilePrivateData->enableSwCs = SW_CS_DISABLED;
	spiFilePrivateData->gpioCs = 0xFFFFFFFF;
	devFile->privateData = (void*) spiFilePrivateData;
	osMutexRelease(spiDevicePrivateData->spiMutex);
	return RET_OK;
}


static retval_t spiClose(deviceFileHandler_t* devFile){
	spiFilePrivateData_t* spiFilePrivateData = (spiFilePrivateData_t*) devFile->privateData;
	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(spiDevicePrivateData->spiMutex, osWaitForever);
	if(spiDevicePrivateData->spiOpRunning){
		if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_CIRCULAR){
			HAL_SPI_DMAStop(spiDevicePrivateData->hspi);
			__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
			ytSetDeviceDefaultLowPowerMode();
			spiDevicePrivateData->spiOpRunning = 0;
		}
		else{
			osMutexRelease(spiDevicePrivateData->spiMutex);
			return RET_ERROR;
		}
	}
	vPortFree(spiFilePrivateData);
	osMutexRelease(spiDevicePrivateData->spiMutex);
	return RET_OK;
}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t spiRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	spiFilePrivateData_t* spiFilePrivateData = (spiFilePrivateData_t*) devFile->privateData;
	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(spiDevicePrivateData->spiMutex, osWaitForever);
	if(spiDevicePrivateData->spiOpRunning){
		osMutexRelease(spiDevicePrivateData->spiMutex);
		return 0;
	}
	spiDevicePrivateData->spiOpRunning++;
	__HAL_SPI_ENABLE(spiDevicePrivateData->hspi);
	/*Dont use SW Cs*/
	if((spiFilePrivateData->enableSwCs == SW_CS_DISABLED) || (spiFilePrivateData->enableSwCs == SW_CS_ALWAYS_ACTIVE)){

		if((size <= MIN_NUM_BYTES_DMA) && (spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_MASTER)){
			if (HAL_SPI_Receive(spiDevicePrivateData->hspi, buff, size, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				spiDevicePrivateData->spiOpRunning = 0;
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				osMutexRelease(spiDevicePrivateData->spiMutex);
				return 0;
			}
			__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
			spiDevicePrivateData->spiOpRunning = 0;
		}
		else{
			ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
			HAL_SPI_Receive_DMA(spiDevicePrivateData->hspi, buff, size);
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_NORMAL){
				if(osSemaphoreWait(spiDevicePrivateData->spiSemaphore, SPI_DEFAULT_TIMEOUT) != RET_OK){
					ytSetDeviceDefaultLowPowerMode();
					__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
					spiDevicePrivateData->spiOpRunning = 0;
					osMutexRelease(spiDevicePrivateData->spiMutex);
					return 0;
				}
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
			}
		}


	}
	else if(spiFilePrivateData->enableSwCs == SW_CS_ENABLED){	/*Enable SW Chip Select*/

		if((size <= MIN_NUM_BYTES_DMA) && (spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_MASTER)){
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			if (HAL_SPI_Receive(spiDevicePrivateData->hspi, buff, size, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				osMutexRelease(spiDevicePrivateData->spiMutex);
				return 0;
			}
			ytGpioPinSet(spiFilePrivateData->gpioCs);
			__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
			spiDevicePrivateData->spiOpRunning = 0;
		}
		else{
			ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			HAL_SPI_Receive_DMA(spiDevicePrivateData->hspi, buff, size);
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_NORMAL){
				if(osSemaphoreWait(spiDevicePrivateData->spiSemaphore, SPI_DEFAULT_TIMEOUT) != RET_OK){
					ytGpioPinSet(spiFilePrivateData->gpioCs);
					__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
					ytSetDeviceDefaultLowPowerMode();
					spiDevicePrivateData->spiOpRunning = 0;
					osMutexRelease(spiDevicePrivateData->spiMutex);
					return 0;
				}
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
			}
		}
	}
	else if(spiFilePrivateData->enableSwCs == SW_CS_ENABLED_ALL_BYTES){/*Enable SW chip select for each byte*/
		uint32_t i;
		for(i = 0; i < size; i++){
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			if (HAL_SPI_Receive(spiDevicePrivateData->hspi, buff+i, 1, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				osMutexRelease(spiDevicePrivateData->spiMutex);
				return 0;
			}
			ytGpioPinSet(spiFilePrivateData->gpioCs);
		}
		__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
		spiDevicePrivateData->spiOpRunning = 0;
	}
	else{
		spiDevicePrivateData->spiOpRunning = 0;
		__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
		osMutexRelease(spiDevicePrivateData->spiMutex);
		return 0;
	}
	osMutexRelease(spiDevicePrivateData->spiMutex);
	return size;
}


/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t spiWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	spiFilePrivateData_t* spiFilePrivateData = (spiFilePrivateData_t*) devFile->privateData;
	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(spiDevicePrivateData->spiMutex, osWaitForever);
	if(spiDevicePrivateData->spiOpRunning){
		osMutexRelease(spiDevicePrivateData->spiMutex);
		return 0;
	}
	spiDevicePrivateData->spiOpRunning++;
	__HAL_SPI_ENABLE(spiDevicePrivateData->hspi);
	/*Dont use SW Cs*/
	if((spiFilePrivateData->enableSwCs == SW_CS_DISABLED) || (spiFilePrivateData->enableSwCs == SW_CS_ALWAYS_ACTIVE)){

		if((size <= MIN_NUM_BYTES_DMA) && (spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_MASTER)){
			if (HAL_SPI_Transmit(spiDevicePrivateData->hspi, buff, size, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				osMutexRelease(spiDevicePrivateData->spiMutex);
				return 0;
			}
			__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
			spiDevicePrivateData->spiOpRunning = 0;
		}
		else{
			ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
			HAL_SPI_Transmit_DMA(spiDevicePrivateData->hspi, buff, size);
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_NORMAL){
				if(osSemaphoreWait(spiDevicePrivateData->spiSemaphore, SPI_DEFAULT_TIMEOUT) != RET_OK){
					ytSetDeviceDefaultLowPowerMode();
					spiDevicePrivateData->spiOpRunning = 0;
					__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
					osMutexRelease(spiDevicePrivateData->spiMutex);
					return 0;
				}
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
			}
		}


	}
	else if(spiFilePrivateData->enableSwCs == SW_CS_ENABLED){	/*Enable SW Chip Select*/

		if((size <= MIN_NUM_BYTES_DMA) && (spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_MASTER)){
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			if (HAL_SPI_Transmit(spiDevicePrivateData->hspi, buff, size, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				osMutexRelease(spiDevicePrivateData->spiMutex);
				return 0;
			}
			ytGpioPinSet(spiFilePrivateData->gpioCs);
			__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
			spiDevicePrivateData->spiOpRunning = 0;
		}
		else{
			ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			HAL_SPI_Transmit_DMA(spiDevicePrivateData->hspi, buff, size);
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_NORMAL){
				if(osSemaphoreWait(spiDevicePrivateData->spiSemaphore, SPI_DEFAULT_TIMEOUT) != RET_OK){
					ytGpioPinSet(spiFilePrivateData->gpioCs);
					__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
					ytSetDeviceDefaultLowPowerMode();
					spiDevicePrivateData->spiOpRunning = 0;
					osMutexRelease(spiDevicePrivateData->spiMutex);
					return 0;
				}
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
			}
		}
	}
	else if(spiFilePrivateData->enableSwCs == SW_CS_ENABLED_ALL_BYTES){/*Enable SW chip select for each byte*/
		uint32_t i;
		for(i = 0; i < size; i++){
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			if (HAL_SPI_Transmit(spiDevicePrivateData->hspi, buff+i, 1, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				osMutexRelease(spiDevicePrivateData->spiMutex);
				return 0;
			}
			ytGpioPinSet(spiFilePrivateData->gpioCs);
		}
		__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
		spiDevicePrivateData->spiOpRunning = 0;
	}
	else{
		__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
		spiDevicePrivateData->spiOpRunning = 0;
		osMutexRelease(spiDevicePrivateData->spiMutex);
		return 0;
	}
	osMutexRelease(spiDevicePrivateData->spiMutex);
	return size;
}


/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t spiIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	retval_t ret = RET_ERROR;
	spiFilePrivateData_t* spiFilePrivateData = (spiFilePrivateData_t*) devFile->privateData;
	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devFile->device->privateData;
	ytSpiReadWriteBuff_t* spiReadWriteBuff;
	osMutexWait(spiDevicePrivateData->spiMutex, osWaitForever);
	switch(command){
	case SPI_READ_WRITE:
		spiReadWriteBuff = (ytSpiReadWriteBuff_t*) args;
		ret = spiReadWrite(devFile, spiReadWriteBuff->ptx, spiReadWriteBuff->prx, spiReadWriteBuff->size);
		break;
	case SET_ASYNC_CIRCULAR_READ_MODE:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		spiDevicePrivateData->hdma_spi_rx->Init.Mode = DMA_CIRCULAR;
		HAL_DMA_Init(spiDevicePrivateData->hdma_spi_rx);
		spiDevicePrivateData->hdma_spi_tx->Init.Mode = DMA_CIRCULAR;
		HAL_DMA_Init(spiDevicePrivateData->hdma_spi_tx);
		ret = RET_OK;
		break;
	case STOP_ASYNC_READ:
		if(spiDevicePrivateData->spiOpRunning){
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_CIRCULAR){
				HAL_SPI_DMAStop(spiDevicePrivateData->hspi);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
				ret = RET_OK;
			}
		}
		break;
	case GET_CURRENT_REMAINING_COUNT:
		if(spiDevicePrivateData->spiOpRunning){
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_CIRCULAR){
				(*((uint32_t*)args)) =__HAL_DMA_GET_COUNTER(spiDevicePrivateData->hdma_spi_rx);
				ret = RET_OK;
			}
		}
		break;
	case SET_STD_READ_MODE:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		spiDevicePrivateData->hdma_spi_rx->Init.Mode = DMA_NORMAL;
		HAL_DMA_Init(spiDevicePrivateData->hdma_spi_rx);
		spiDevicePrivateData->hdma_spi_tx->Init.Mode = DMA_NORMAL;
		HAL_DMA_Init(spiDevicePrivateData->hdma_spi_tx);
		ret = RET_OK;
		break;
	case SPI_SET_SW_CS_PIN:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}

		if((ret = ytGpioInitPin(*((uint32_t*) args), GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_PULLUP)) != RET_OK){
			break;
		}
		ytGpioPinSet(*((uint32_t*) args));
		spiFilePrivateData->gpioCs = *((uint32_t*) args);
		spiFilePrivateData->enableSwCs = SW_CS_ENABLED;
		break;
	case SPI_SET_SW_CS_EACH_BYTE:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		if(spiDevicePrivateData->hspi->Instance==SPI1){

		if((ret = ytGpioInitPin(*((uint32_t*) args), GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_PULLUP)) != RET_OK){
			break;
		}
		ytGpioPinSet(*((uint32_t*) args));
		spiFilePrivateData->gpioCs = *((uint32_t*) args);
		spiFilePrivateData->enableSwCs = SW_CS_ENABLED_ALL_BYTES;
		break;
	case SPI_SET_SW_CS_ALWAYS:
		if(spiDevicePrivateData->spiOpRunning){
			return 0;
		}
		}
		if((ret = ytGpioInitPin(*((uint32_t*) args), GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_NO_PULL)) != RET_OK){
			break;
		}
		ytGpioPinReset(*((uint32_t*) args));	/*SET LOW THIS PIN TO ENABLE IT ALWAYS*/
		spiFilePrivateData->gpioCs = *((uint32_t*) args);
		spiFilePrivateData->enableSwCs = SW_CS_ALWAYS_ACTIVE;
		break;

	/*Device commands. The effects of these commands affects to every device file opened of the device*/
	case SPI_SET_HW_CS:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		if(spiDevicePrivateData->hspi->Instance==SPI1){
#if PLATFORM_SPI1_DEVICE_ID
			GPIO_InitTypeDef GPIO_InitStruct;
			GPIO_InitStruct.Pin = PLATFORM_SPI1_NSSP_PIN;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
			HAL_GPIO_Init(PLATFORM_SPI1_PIN_PORT, &GPIO_InitStruct);
#endif

		}
		else if(spiDevicePrivateData->hspi->Instance==SPI3){
#if PLATFORM_SPI3_DEVICE_ID
			GPIO_InitTypeDef GPIO_InitStruct;
			GPIO_InitStruct.Pin = PLATFORM_SPI3_NSSP_PIN;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
			HAL_GPIO_Init(PLATFORM_SPI3_NSSP_PIN_PORT, &GPIO_InitStruct);
#endif
		}
		if(spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_SLAVE){
			spiDevicePrivateData->hspi->Init.NSS = SPI_NSS_HARD_INPUT;
		}
		else{
			spiDevicePrivateData->hspi->Init.NSS = SPI_NSS_HARD_OUTPUT;
		}
		spiDevicePrivateData->hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		if (HAL_SPI_Init(spiDevicePrivateData->hspi) != HAL_OK)
		{
			ret = RET_ERROR;
		}
		ret = RET_OK;
		break;
	case SPI_SET_HW_PULSE_CS:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		if(spiDevicePrivateData->hspi->Instance==SPI1){
#if PLATFORM_SPI1_DEVICE_ID
			GPIO_InitTypeDef GPIO_InitStruct;
			GPIO_InitStruct.Pin = PLATFORM_SPI1_NSSP_PIN;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
			HAL_GPIO_Init(PLATFORM_SPI1_PIN_PORT, &GPIO_InitStruct);
#endif

		}
		else if(spiDevicePrivateData->hspi->Instance==SPI3){
#if PLATFORM_SPI3_DEVICE_ID
			GPIO_InitTypeDef GPIO_InitStruct;
			GPIO_InitStruct.Pin = PLATFORM_SPI3_NSSP_PIN;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
			HAL_GPIO_Init(PLATFORM_SPI3_NSSP_PIN_PORT, &GPIO_InitStruct);
#endif
		}
		if(spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_SLAVE){
			spiDevicePrivateData->hspi->Init.NSS = SPI_NSS_HARD_INPUT;
		}
		else{
			spiDevicePrivateData->hspi->Init.NSS = SPI_NSS_HARD_OUTPUT;
		}
		spiDevicePrivateData->hspi->Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
		if (HAL_SPI_Init(spiDevicePrivateData->hspi) != HAL_OK)
		{
			ret = RET_ERROR;
		}
		ret = RET_OK;
		break;
	case SPI_SET_SPEED:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		spiDevicePrivateData->spiClkFreq = *((uint32_t*) args);	/*Store the desired speed*/
		ret = spiSetSpeed(devFile->device);	/*Set the new Speed*/
		break;

	case SPI_SET_POLARITY:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		if((*((ytSpiPolarity_t*) args)) == SPI_POL_HIGH){
			spiDevicePrivateData->hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
		}
		else{
			spiDevicePrivateData->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
		}
		if (HAL_SPI_Init(spiDevicePrivateData->hspi) != HAL_OK)
		{
			ret = RET_ERROR;
		}
		ret = RET_OK;
		break;

	case SPI_SET_EDGE:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		if((*((ytSpiEdge_t*) args)) == SPI_EDGE_FIRST){
			spiDevicePrivateData->hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
		}
		else{
			spiDevicePrivateData->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
		}
		if (HAL_SPI_Init(spiDevicePrivateData->hspi) != HAL_OK)
		{
			ret = RET_ERROR;
		}
		ret = RET_OK;
		break;

	case SPI_SET_MODE:
		if(spiDevicePrivateData->spiOpRunning){
			break;
		}
		if((*((ytSpiMode_t*) args)) == SPI_MASTER){
			spiDevicePrivateData->hspi->Init.Mode = SPI_MODE_MASTER;
		}
		else{
			spiDevicePrivateData->hspi->Init.Mode = SPI_MODE_SLAVE;
		}
		if (HAL_SPI_Init(spiDevicePrivateData->hspi) != HAL_OK)
		{
			ret = RET_ERROR;
		}
		ret = RET_OK;
		break;
	default:
		ret = RET_ERROR;
		break;
	}

	osMutexRelease(spiDevicePrivateData->spiMutex);
	return ret;
}

/**
 *
 * @param devFile
 * @return
 */
static retval_t spiChangedCpuFreq(devHandler_t* devHandler){
	retval_t ret = RET_OK;
	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devHandler->privateData;
	/*This function is called by the OS when the CPU clock frequency changes to keep an appropiate peripheral configuration*/
	osMutexWait(spiDevicePrivateData->spiMutex, osWaitForever);
	ret = spiSetSpeed(devHandler);	/*Use an adequate prescaler value for the new system frequency*/
	osMutexRelease(spiDevicePrivateData->spiMutex);
	return ret;
}

/**
 *
 * @param devFile
 * @param ptx
 * @param prx
 * @param size
 * @return
 */
static retval_t spiReadWrite(deviceFileHandler_t*  devFile, uint8_t* ptx, uint8_t* prx, uint32_t size){
	spiFilePrivateData_t* spiFilePrivateData = (spiFilePrivateData_t*) devFile->privateData;
	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devFile->device->privateData;
	/*This function does not need a mutex, since it is called only from Ioctl, that already has the mutex locked*/
	if(spiDevicePrivateData->spiOpRunning){
		return RET_ERROR;
	}
	spiDevicePrivateData->spiOpRunning++;
	__HAL_SPI_ENABLE(spiDevicePrivateData->hspi);
	/*Dont use SW Cs*/
	if((spiFilePrivateData->enableSwCs == SW_CS_DISABLED) || (spiFilePrivateData->enableSwCs == SW_CS_ALWAYS_ACTIVE)){

		if((size <= MIN_NUM_BYTES_DMA) && (spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_MASTER)){
			if (HAL_SPI_TransmitReceive(spiDevicePrivateData->hspi, ptx, prx, size, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				return RET_ERROR;
			}
			__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
			spiDevicePrivateData->spiOpRunning = 0;
		}
		else{
			ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
			HAL_SPI_TransmitReceive_DMA(spiDevicePrivateData->hspi, ptx, prx, size);
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_NORMAL){
				if(osSemaphoreWait(spiDevicePrivateData->spiSemaphore, SPI_DEFAULT_TIMEOUT) != RET_OK){
					ytSetDeviceDefaultLowPowerMode();
					__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
					spiDevicePrivateData->spiOpRunning = 0;
					return RET_ERROR;
				}
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
			}
		}


	}
	else if(spiFilePrivateData->enableSwCs == SW_CS_ENABLED){	/*Enable SW Chip Select*/

		if((size <= MIN_NUM_BYTES_DMA) && (spiDevicePrivateData->hspi->Init.Mode == SPI_MODE_MASTER)){
			__HAL_SPI_ENABLE(spiDevicePrivateData->hspi);
			ytGpioPinReset(spiFilePrivateData->gpioCs);
//			osDelay(1);
			if (HAL_SPI_TransmitReceive(spiDevicePrivateData->hspi, ptx, prx, size, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				return RET_ERROR;
			}
			ytGpioPinSet(spiFilePrivateData->gpioCs);
			__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
			spiDevicePrivateData->spiOpRunning = 0;
		}
		else{
			ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			HAL_SPI_TransmitReceive_DMA(spiDevicePrivateData->hspi, ptx, prx, size);
			if(spiDevicePrivateData->hdma_spi_rx->Init.Mode == DMA_NORMAL){
				if(osSemaphoreWait(spiDevicePrivateData->spiSemaphore, SPI_DEFAULT_TIMEOUT) != RET_OK){
					ytGpioPinSet(spiFilePrivateData->gpioCs);
					__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
					ytSetDeviceDefaultLowPowerMode();
					spiDevicePrivateData->spiOpRunning = 0;
					return RET_ERROR;
				}
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				ytSetDeviceDefaultLowPowerMode();
			}
		}
	}
	else if(spiFilePrivateData->enableSwCs == SW_CS_ENABLED_ALL_BYTES){/*Enable SW chip select for each byte*/
		uint32_t i;
		for(i = 0; i < size; i++){
			ytGpioPinReset(spiFilePrivateData->gpioCs);
			if (HAL_SPI_TransmitReceive(spiDevicePrivateData->hspi, ptx+i, prx+i, 1, SPI_DEFAULT_TIMEOUT) != HAL_OK){
				ytGpioPinSet(spiFilePrivateData->gpioCs);
				__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
				spiDevicePrivateData->spiOpRunning = 0;
				return RET_ERROR;
			}
			ytGpioPinSet(spiFilePrivateData->gpioCs);
		}
		__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
		spiDevicePrivateData->spiOpRunning = 0;
	}
	else{
		spiDevicePrivateData->spiOpRunning = 0;
		__HAL_SPI_DISABLE(spiDevicePrivateData->hspi);
		return RET_ERROR;
	}
	return RET_OK;
}

/**
 *
 * @param devFile
 * @param speed
 * @return
 */
static retval_t spiSetSpeed(devHandler_t*  devHandler){

	spiDevicePrivateData_t* spiDevicePrivateData = (spiDevicePrivateData_t*) devHandler->privateData;

	/*Find the closest available prescaler value for the selected speed */
	uint32_t prescaler = (uint32_t) HAL_RCC_GetHCLKFreq()/spiDevicePrivateData->spiClkFreq;
	if(prescaler > 128){	/*Minimum allowed SPI speed*/
		prescaler = SPI_BAUDRATEPRESCALER_256;
	}
	else if(prescaler > 64){
		prescaler = SPI_BAUDRATEPRESCALER_128;
	}
	else if(prescaler > 32){
		prescaler = SPI_BAUDRATEPRESCALER_64;
	}
	else if(prescaler > 16){
		prescaler = SPI_BAUDRATEPRESCALER_32;
	}
	else if(prescaler > 8){
		prescaler = SPI_BAUDRATEPRESCALER_16;
	}
	else if(prescaler > 4){
		prescaler = SPI_BAUDRATEPRESCALER_8;
	}
	else if(prescaler > 2){
		prescaler = SPI_BAUDRATEPRESCALER_4;
	}
	else if(prescaler >= 0){									/*Maximum allowed SPI clock frequency*/
		prescaler = SPI_BAUDRATEPRESCALER_2;
	}
	else{
		return RET_ERROR;
	}
	spiDevicePrivateData->hspi->Init.BaudRatePrescaler = prescaler;
	if(HAL_SPI_Init(spiDevicePrivateData->hspi) != HAL_OK){
		return RET_ERROR;
	}
	return RET_OK;
}

/**
 *
 * @param hspi
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SPI1){
#if PLATFORM_SPI1_DEVICE_ID
		osSemaphoreRelease(((spiDevicePrivateData_t*)(spi1Dev->privateData))->spiSemaphore);
#endif
	}
	else if(hspi->Instance == SPI3){
#if PLATFORM_SPI3_DEVICE_ID
		osSemaphoreRelease(((spiDevicePrivateData_t*)(spi3Dev->privateData))->spiSemaphore);
#endif
	}
}

/**
 *
 * @param hspi
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SPI1){
#if PLATFORM_SPI1_DEVICE_ID
		osSemaphoreRelease(((spiDevicePrivateData_t*)(spi1Dev->privateData))->spiSemaphore);
#endif
	}
	else if(hspi->Instance == SPI3){
#if PLATFORM_SPI3_DEVICE_ID
		osSemaphoreRelease(((spiDevicePrivateData_t*)(spi3Dev->privateData))->spiSemaphore);
#endif
	}
}

/**
 *
 * @param hspi
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SPI1){
#if PLATFORM_SPI1_DEVICE_ID
		osSemaphoreRelease(((spiDevicePrivateData_t*)(spi1Dev->privateData))->spiSemaphore);
#endif
	}
	else if(hspi->Instance == SPI3){
#if PLATFORM_SPI3_DEVICE_ID
		osSemaphoreRelease(((spiDevicePrivateData_t*)(spi3Dev->privateData))->spiSemaphore);
#endif
	}
}

/**
 *
 * @param spiHandle
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  spiDevicePrivateData_t* spiDevicePrivateData;


  if(spiHandle->Instance==SPI1)
  {
#if PLATFORM_SPI1_DEVICE_ID
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* DMA ENABLE*/
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA2_Channel3_IRQn);
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = PLATFORM_SPI1_CLK_PIN|PLATFORM_SPI1_MISO_PIN|PLATFORM_SPI1_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(PLATFORM_SPI1_PIN_PORT, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    spiDevicePrivateData = (spiDevicePrivateData_t*) spi1Dev->privateData;

    spiDevicePrivateData->hdma_spi_rx->Instance = DMA2_Channel3;
    spiDevicePrivateData->hdma_spi_rx->Init.Request = DMA_REQUEST_4;
    spiDevicePrivateData->hdma_spi_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    spiDevicePrivateData->hdma_spi_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    spiDevicePrivateData->hdma_spi_rx->Init.MemInc = DMA_MINC_ENABLE;
    spiDevicePrivateData->hdma_spi_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_rx->Init.Mode = DMA_NORMAL;
    spiDevicePrivateData->hdma_spi_rx->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(spiDevicePrivateData->hdma_spi_rx) != HAL_OK)
    {
      while(1);
    }

    __HAL_LINKDMA(spiHandle,hdmarx,*(spiDevicePrivateData->hdma_spi_rx));

    /* SPI1_TX Init */
    spiDevicePrivateData->hdma_spi_tx->Instance = DMA1_Channel3;
    spiDevicePrivateData->hdma_spi_tx->Init.Request = DMA_REQUEST_1;
    spiDevicePrivateData->hdma_spi_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    spiDevicePrivateData->hdma_spi_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    spiDevicePrivateData->hdma_spi_tx->Init.MemInc = DMA_MINC_ENABLE;
    spiDevicePrivateData->hdma_spi_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_tx->Init.Mode = DMA_NORMAL;
    spiDevicePrivateData->hdma_spi_tx->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(spiDevicePrivateData->hdma_spi_tx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(spiHandle,hdmatx,*(spiDevicePrivateData->hdma_spi_tx));
#endif
  }
  else if(spiHandle->Instance==SPI3)
  {
#if PLATFORM_SPI3_DEVICE_ID
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    /* DMA ENABLE*/
	__HAL_RCC_DMA2_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA2_Channel1_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */

    GPIO_InitStruct.Pin = PLATFORM_SPI3_MISO_PIN|PLATFORM_SPI3_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(PLATFORM_SPI3_PIN_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PLATFORM_SPI3_CLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(PLATFORM_SPI3_PIN_PORT, &GPIO_InitStruct);

    /* SPI3 DMA Init */
    /* SPI3_RX Init */
    spiDevicePrivateData = (spiDevicePrivateData_t*) spi3Dev->privateData;

    spiDevicePrivateData->hdma_spi_rx->Instance = DMA2_Channel1;
    spiDevicePrivateData->hdma_spi_rx->Init.Request = DMA_REQUEST_3;
    spiDevicePrivateData->hdma_spi_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    spiDevicePrivateData->hdma_spi_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    spiDevicePrivateData->hdma_spi_rx->Init.MemInc = DMA_MINC_ENABLE;
    spiDevicePrivateData->hdma_spi_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_rx->Init.Mode = DMA_NORMAL;
    spiDevicePrivateData->hdma_spi_rx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(spiDevicePrivateData->hdma_spi_rx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(spiHandle,hdmarx,*(spiDevicePrivateData->hdma_spi_rx));

    /* SPI3_TX Init */
    spiDevicePrivateData->hdma_spi_tx->Instance = DMA2_Channel2;
    spiDevicePrivateData->hdma_spi_tx->Init.Request = DMA_REQUEST_3;
    spiDevicePrivateData->hdma_spi_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    spiDevicePrivateData->hdma_spi_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    spiDevicePrivateData->hdma_spi_tx->Init.MemInc = DMA_MINC_ENABLE;
    spiDevicePrivateData->hdma_spi_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spiDevicePrivateData->hdma_spi_tx->Init.Mode = DMA_NORMAL;
    spiDevicePrivateData->hdma_spi_tx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(spiDevicePrivateData->hdma_spi_tx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(spiHandle,hdmatx,*(spiDevicePrivateData->hdma_spi_tx));

#endif
  }
}

/**
 *
 * @param spiHandle
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
#if PLATFORM_SPI1_DEVICE_ID
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(PLATFORM_SPI1_PIN_PORT, PLATFORM_SPI1_CLK_PIN|PLATFORM_SPI1_MISO_PIN|PLATFORM_SPI1_MOSI_PIN);

    /* DMA DISABLE*/
	HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Channel3_IRQn);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);

#endif
  }
  else if(spiHandle->Instance==SPI3)
  {
#if PLATFORM_SPI3_DEVICE_ID
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    HAL_GPIO_DeInit(PLATFORM_SPI3_PIN_PORT, PLATFORM_SPI3_CLK_PIN|PLATFORM_SPI3_MISO_PIN|PLATFORM_SPI3_MOSI_PIN);

    /* DMA DISABLE*/
	HAL_NVIC_DisableIRQ(DMA2_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Channel2_IRQn);

    /* SPI3 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
#endif
  }
}

#if PLATFORM_SPI3_DEVICE_ID
/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA2_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((spiDevicePrivateData_t*) spi3Dev->privateData)->hdma_spi_rx);
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA2_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((spiDevicePrivateData_t*) spi3Dev->privateData)->hdma_spi_tx);
}
#endif

#if PLATFORM_SPI1_DEVICE_ID
/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((spiDevicePrivateData_t*) spi1Dev->privateData)->hdma_spi_tx);
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA2_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((spiDevicePrivateData_t*) spi1Dev->privateData)->hdma_spi_rx);
}
#endif
/*Load the Spi Driver module on startup*/

InitDevice(spiInit);
ExitDevice(spiExit);
#endif

#endif
