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
 * uartDriver.c
 *
 *  Created on: 26 jul. 2019
 *      Author: Santiago Real Valdes <sreal@b105.upm.es>
 *
 */
/**
 * @file uartDriver.c
 */
#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "stm32l4xx_hal.h"


#if PLATFORM_UART1_DEVICE_ID	/*Do not compile the driver if not defined any of the IDs in the Conf file*/
#include "deviceDriver.h"
#include "lowPower.h"

#define UART_DEFAULT_TIMEOUT	osWaitForever
#define MIN_NUM_BYTES_DMA		PLATFORM_I2C1_DEVICE_ID ? 0XFFFFFFFF : 12		/*When reading or writing more than this number of bytes, the DMA is used*/

typedef struct uartPrivateData_{
	UART_HandleTypeDef* huart;
	DMA_HandleTypeDef* hdma_uart_rx;
	DMA_HandleTypeDef* hdma_uart_tx;
	osSemaphoreId uartSemaphore;
	osMutexId uartMutex;
	uint32_t uartClkFreq;
	uint16_t uartOpRunning;
}uartDevicePrivateData_t;


/*UART and DMA HAL handles*/
#if PLATFORM_UART1_DEVICE_ID
static devHandler_t* uart1Dev;
#endif

/*UART Driver Init and Exit*/
static retval_t uartInit(void);
static retval_t uartExit(void);

/*UART DRIVER FUNCTIONS*/
static retval_t uartOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t uartClose(deviceFileHandler_t* devFile);
static uint32_t uartRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t uartWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t uartIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t uartChangedCpuFreq(devHandler_t* devHandler);

/*ADDITIONAL FUNCTIONS*/


static deviceDriverOps_t uartDriverOps = {
	.open = uartOpen,
	.close = uartClose,
	.read = uartRead,
	.write = uartWrite,
	.ioctl = uartIoctl,
	.changedCpuFreq = uartChangedCpuFreq,
};


/**
 *
 * @return
 */
static retval_t uartInit(void){
	uartDevicePrivateData_t* uartPrivateData;

	/*Default UART1 Configuration Init*/
	uart1Dev =  ytNewDevice(PLATFORM_UART1_DEVICE_ID, &uartDriverOps);
	uartPrivateData = (uartDevicePrivateData_t*)pvPortMalloc(sizeof(uartDevicePrivateData_t));
	uart1Dev->privateData = (void*) uartPrivateData;
	uartPrivateData->huart = (UART_HandleTypeDef*)pvPortMalloc(sizeof(UART_HandleTypeDef));
	uartPrivateData->hdma_uart_rx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));
	uartPrivateData->hdma_uart_tx = (DMA_HandleTypeDef*)pvPortMalloc(sizeof(DMA_HandleTypeDef));
	uartPrivateData->huart->Instance = USART1;
	uartPrivateData->huart->Init.Mode = UART_MODE_TX_RX;
	uartPrivateData->huart->Init.BaudRate = 115200;
	uartPrivateData->huart->Init.WordLength = UART_WORDLENGTH_8B;
	uartPrivateData->huart->Init.StopBits = UART_STOPBITS_1;
	uartPrivateData->huart->Init.Parity = UART_PARITY_NONE;
	uartPrivateData->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uartPrivateData->huart->Init.OverSampling = UART_OVERSAMPLING_16;
	uartPrivateData->huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uartPrivateData->huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(uartPrivateData->huart) != HAL_OK)
	{
		vPortFree(uartPrivateData->hdma_uart_rx);
		vPortFree(uartPrivateData->hdma_uart_tx);
		vPortFree(uartPrivateData->huart);
		vPortFree(uartPrivateData);
		ytDeleteDevice(uart1Dev);
		return RET_ERROR;
	}
	/*Store the references of the HAL handlers, mutexes and semaphores*/
	uartPrivateData->uartSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(uartPrivateData->uartSemaphore, osWaitForever);
	uartPrivateData->uartMutex = ytMutexCreate();
	uartPrivateData->uartOpRunning = 0;
	if(ytRegisterDevice(uart1Dev) != RET_OK){
		HAL_UART_DeInit(uartPrivateData->huart);
		osMutexDelete(uartPrivateData->uartMutex);
		osSemaphoreDelete(uartPrivateData->uartSemaphore);
		vPortFree(uartPrivateData->hdma_uart_rx);
		vPortFree(uartPrivateData->hdma_uart_tx);
		vPortFree(uartPrivateData->huart);
		vPortFree(uartPrivateData);
		ytDeleteDevice(uart1Dev);
		return RET_ERROR;
	}
	return RET_OK;
}


/**
 *
 * @return
 */
static retval_t uartExit(void){
	uartDevicePrivateData_t* uartDevicePrivateData;

	/*Unregister and Delete UART1*/
	uartDevicePrivateData = (uartDevicePrivateData_t*) uart1Dev->privateData;
	ytUnregisterDevice(PLATFORM_UART1_DEVICE_ID);

	HAL_UART_DeInit(uartDevicePrivateData->huart);
	vPortFree(uartDevicePrivateData->huart);
	vPortFree(uartDevicePrivateData->hdma_uart_rx);
	vPortFree(uartDevicePrivateData->hdma_uart_tx);

	osMutexDelete(uartDevicePrivateData->uartMutex);
	osSemaphoreDelete(uartDevicePrivateData->uartSemaphore);
	vPortFree(uartDevicePrivateData);
	ytDeleteDevice(uart1Dev);
	return RET_OK;
}



/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t uartOpen(deviceFileHandler_t* devFile, uint32_t flags){
	/*INCOMPLETE*/
	uartDevicePrivateData_t* uartDevicePrivateData = (uartDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(uartDevicePrivateData->uartMutex, osWaitForever);
	osMutexRelease(uartDevicePrivateData->uartMutex);
	return RET_OK;
}


static retval_t uartClose(deviceFileHandler_t* devFile){
	/*INCOMPLETE*/
	uartDevicePrivateData_t* uartDevicePrivateData = (uartDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(uartDevicePrivateData->uartMutex, osWaitForever);
	if(uartDevicePrivateData->uartOpRunning){
		if(uartDevicePrivateData->hdma_uart_rx->Init.Mode == DMA_CIRCULAR){
			HAL_UART_DMAStop(uartDevicePrivateData->huart);
			__HAL_UART_DISABLE(uartDevicePrivateData->huart);
			ytSetDeviceDefaultLowPowerMode();
			uartDevicePrivateData->uartOpRunning = 0;
		}
		else{
			osMutexRelease(uartDevicePrivateData->uartMutex);
			return RET_ERROR;
		}
	}
	osMutexRelease(uartDevicePrivateData->uartMutex);
	return RET_OK;
}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t uartRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	uartDevicePrivateData_t* uartDevicePrivateData = (uartDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(uartDevicePrivateData->uartMutex, osWaitForever);
	if(uartDevicePrivateData->uartOpRunning){
		osMutexRelease(uartDevicePrivateData->uartMutex);
		return 0;
	}
	uartDevicePrivateData->uartOpRunning++;
	__HAL_UART_ENABLE(uartDevicePrivateData->huart);

	if((size <= MIN_NUM_BYTES_DMA) && ((uartDevicePrivateData->huart->Init.Mode == UART_MODE_TX_RX)||(uartDevicePrivateData->huart->Init.Mode == UART_MODE_RX))){
		if (HAL_UART_Receive(uartDevicePrivateData->huart, buff, size, UART_DEFAULT_TIMEOUT) != HAL_OK){
			uartDevicePrivateData->uartOpRunning = 0;
			__HAL_UART_DISABLE(uartDevicePrivateData->huart);
			osMutexRelease(uartDevicePrivateData->uartMutex);
			return 0;
		}
		__HAL_UART_DISABLE(uartDevicePrivateData->huart);
		uartDevicePrivateData->uartOpRunning = 0;
	}
	else if((uartDevicePrivateData->huart->Init.Mode == UART_MODE_TX_RX)||(uartDevicePrivateData->huart->Init.Mode == UART_MODE_RX)){
		ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
		HAL_UART_Receive_DMA(uartDevicePrivateData->huart, buff, size);
		if(uartDevicePrivateData->hdma_uart_rx->Init.Mode == DMA_NORMAL){
			if(osSemaphoreWait(uartDevicePrivateData->uartSemaphore, UART_DEFAULT_TIMEOUT) != RET_OK){
				ytSetDeviceDefaultLowPowerMode();
				__HAL_UART_DISABLE(uartDevicePrivateData->huart);
				uartDevicePrivateData->uartOpRunning = 0;
				osMutexRelease(uartDevicePrivateData->uartMutex);
				return 0;
			}
			__HAL_UART_DISABLE(uartDevicePrivateData->huart);
			uartDevicePrivateData->uartOpRunning = 0;
			ytSetDeviceDefaultLowPowerMode();
		}
	}
	else{
		uartDevicePrivateData->uartOpRunning = 0;
		__HAL_UART_DISABLE(uartDevicePrivateData->huart);
		osMutexRelease(uartDevicePrivateData->uartMutex);
		return 0;
	}
	osMutexRelease(uartDevicePrivateData->uartMutex);
	return size;
}


/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t uartWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	uartDevicePrivateData_t* uartDevicePrivateData = (uartDevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(uartDevicePrivateData->uartMutex, osWaitForever);
	if(uartDevicePrivateData->uartOpRunning){
		osMutexRelease(uartDevicePrivateData->uartMutex);
		return 0;
	}
	uartDevicePrivateData->uartOpRunning++;
	__HAL_UART_ENABLE(uartDevicePrivateData->huart);

	if((size <= MIN_NUM_BYTES_DMA) && ((uartDevicePrivateData->huart->Init.Mode == UART_MODE_TX_RX)||(uartDevicePrivateData->huart->Init.Mode == UART_MODE_TX))){
		if (HAL_UART_Transmit(uartDevicePrivateData->huart, buff, size, UART_DEFAULT_TIMEOUT) != HAL_OK){
			__HAL_UART_DISABLE(uartDevicePrivateData->huart);
			uartDevicePrivateData->uartOpRunning = 0;
			osMutexRelease(uartDevicePrivateData->uartMutex);
			return 0;
		}
		__HAL_UART_DISABLE(uartDevicePrivateData->huart);
		uartDevicePrivateData->uartOpRunning = 0;
	}
	else if((uartDevicePrivateData->huart->Init.Mode == UART_MODE_TX_RX)||(uartDevicePrivateData->huart->Init.Mode == UART_MODE_TX)){
		ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);
		HAL_UART_Transmit_DMA(uartDevicePrivateData->huart, buff, size);
		if(uartDevicePrivateData->hdma_uart_rx->Init.Mode == DMA_NORMAL){
			if(osSemaphoreWait(uartDevicePrivateData->uartSemaphore, UART_DEFAULT_TIMEOUT) != RET_OK){
				ytSetDeviceDefaultLowPowerMode();
				uartDevicePrivateData->uartOpRunning = 0;
				__HAL_UART_DISABLE(uartDevicePrivateData->huart);
				osMutexRelease(uartDevicePrivateData->uartMutex);
				return 0;
			}
			__HAL_UART_DISABLE(uartDevicePrivateData->huart);
			uartDevicePrivateData->uartOpRunning = 0;
			ytSetDeviceDefaultLowPowerMode();
		}
	}


	osMutexRelease(uartDevicePrivateData->uartMutex);
	return size;
}


/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t uartIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	retval_t ret = RET_OK;
	uartDevicePrivateData_t* uartDevicePrivateData = (uartDevicePrivateData_t*) devFile->device->privateData;

	osMutexWait(uartDevicePrivateData->uartMutex, osWaitForever);

	switch(command){
	case SET_UART_SPEED:
		uartDevicePrivateData->huart->Init.BaudRate = *((uint32_t*) args);
		if(HAL_UART_Init((UART_HandleTypeDef*)(uartDevicePrivateData->huart)) != HAL_OK){
			ret = RET_ERROR;
		}
		break;
	case UART_SET_NO_PARITY:
		uartDevicePrivateData->huart->Init.Parity = UART_PARITY_NONE;
		if(HAL_UART_Init(uartDevicePrivateData->huart) != HAL_OK){
			ret = RET_ERROR;
		}
		break;
	case UART_SET_PARITY_EVEN:
		uartDevicePrivateData->huart->Init.Parity = UART_PARITY_EVEN;
		if(HAL_UART_Init(uartDevicePrivateData->huart) != HAL_OK){
			ret = RET_ERROR;
		}
		break;
	case UART_SET_PARITY_ODD:
		uartDevicePrivateData->huart->Init.Parity = UART_PARITY_ODD;
		if(HAL_UART_Init(uartDevicePrivateData->huart) != HAL_OK){
			ret = RET_ERROR;
		}
		break;



//	case SET_ASYNC_CIRCULAR_READ_MODE:
//		osMutexWait(uartOpMutex, osWaitForever);
//		if(readingFlag){	/*Not allowed changing this configuration while reading*/
//			ret = RET_ERROR;
//			osMutexRelease(usbOpMutex);
//		}
//		else{
//			configReadMode = USB_READ_CIRCULAR_BUF;
//			osMutexRelease(usbOpMutex);
//		}
//		break;
//	case STOP_ASYNC_READ:
//		osMutexWait(usbOpMutex, osWaitForever);
//		if(readingFlag){
//			readingFlag = 0;
//			ret = RET_OK;
//		}
//		osMutexRelease(usbOpMutex);
//		break;
//	case GET_CURRENT_READ_PTR:
//		if(args == NULL){
//			ret = RET_ERROR;
//		}
//		else{
//			osMutexWait(usbOpMutex, osWaitForever);
//			if(readingFlag){	/*Only return a valid value when reading*/
//				uint8_t** retPtr = (uint8_t**) args;
//				(*retPtr) = currentReadBufferPtr;
//				osMutexRelease(usbOpMutex);
//			}
//			else{
//				ret = RET_ERROR;
//				osMutexRelease(usbOpMutex);
//			}
//		}
//
//		break;
//	case WAIT_FOR_NEW_DATA:
//		osSemaphoreWait(usbDataAvailableSemph, *((uint32_t*)args));	/*The timeout is passed in args*/
//		break;
//	case SET_READ_COMPLETE_CB:
//		readCompleteCb = args;
//		break;
//	case SET_STD_READ_MODE:
//		if(readingFlag){	/*Not allowed changing this configuration while reading*/
//			ret = RET_ERROR;
//		}
//		else{
//			configReadMode = USB_READ_STANDARD;
//		}
//		break;


	default:
		ret = RET_ERROR;
	}

	osMutexRelease(uartDevicePrivateData->uartMutex);
	return ret;
}


/**
 *
 * @param devFile
 * @return
 */
static retval_t uartChangedCpuFreq(devHandler_t* devHandler){
	retval_t ret = RET_OK;
	/*NOT IMPLEMENTED*/
	//uartDevicePrivateData_t* uartDevicePrivateData = (uartDevicePrivateData_t*) devHandler->privateData;
	/*This function is called by the OS when the CPU clock frequency changes to keep an appropiate peripheral configuration*/
	//osMutexWait(uartDevicePrivateData->uartMutex, osWaitForever);
	//ret = uartSetSpeed(devHandler);	/*Use an adequate prescaler value for the new system frequency*/
	//osMutexRelease(uartDevicePrivateData->uartMutex);
	return ret;
}



/**
 *
 * @param huart
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
#if PLATFORM_UART1_DEVICE_ID
		osSemaphoreRelease(((uartDevicePrivateData_t*)(uart1Dev->privateData))->uartSemaphore);
#endif
	}
}


/**
 *
 * @param huart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
#if PLATFORM_UART1_DEVICE_ID
		osSemaphoreRelease(((uartDevicePrivateData_t*)(uart1Dev->privateData))->uartSemaphore);
#endif
	}
}


/**
 *
 * @param uartHandle
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uartDevicePrivateData_t* uartDevicePrivateData;


  if(uartHandle->Instance==USART1)
  {
#if PLATFORM_UART1_DEVICE_ID
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    /* DMA ENABLE*/
	__HAL_RCC_DMA2_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA2_Channel6_IRQn);

	HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);
	HAL_NVIC_ClearPendingIRQ(DMA2_Channel7_IRQn);


    /**USART1 GPIO Configuration
    PB7     ------> ST_LINK_UART1_RX
    PB6     ------> ST_LINK_UART1_TX
    */

	GPIO_InitStruct.Pin = PLATFORM_UART1_TX_PIN|PLATFORM_UART1_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(PLATFORM_UART1_PIN_PORT, &GPIO_InitStruct);


    uartDevicePrivateData = (uartDevicePrivateData_t*) uart1Dev->privateData;

    uartDevicePrivateData->hdma_uart_tx->Instance = DMA2_Channel6;
    uartDevicePrivateData->hdma_uart_tx->Init.Request = DMA_REQUEST_2;
    uartDevicePrivateData->hdma_uart_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    uartDevicePrivateData->hdma_uart_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    uartDevicePrivateData->hdma_uart_tx->Init.MemInc = DMA_MINC_ENABLE;
    uartDevicePrivateData->hdma_uart_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uartDevicePrivateData->hdma_uart_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    uartDevicePrivateData->hdma_uart_tx->Init.Mode = DMA_NORMAL;
    uartDevicePrivateData->hdma_uart_tx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(uartDevicePrivateData->hdma_uart_tx) != HAL_OK)
    {
    	while(1);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,*(uartDevicePrivateData->hdma_uart_tx));



    /* UART1_TX Init */
	uartDevicePrivateData->hdma_uart_rx->Instance = DMA2_Channel7;
	uartDevicePrivateData->hdma_uart_rx->Init.Request = DMA_REQUEST_2;
	uartDevicePrivateData->hdma_uart_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
	uartDevicePrivateData->hdma_uart_rx->Init.PeriphInc = DMA_PINC_DISABLE;
	uartDevicePrivateData->hdma_uart_rx->Init.MemInc = DMA_MINC_ENABLE;
	uartDevicePrivateData->hdma_uart_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	uartDevicePrivateData->hdma_uart_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	uartDevicePrivateData->hdma_uart_rx->Init.Mode = DMA_NORMAL;
	uartDevicePrivateData->hdma_uart_rx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
	if (HAL_DMA_Init(uartDevicePrivateData->hdma_uart_rx) != HAL_OK)
	{
		while(1);
	}

	__HAL_LINKDMA(uartHandle,hdmarx,*(uartDevicePrivateData->hdma_uart_rx));

    /* UART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
  }
}

/**
 *
 * @param uartHandle
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    HAL_GPIO_DeInit(PLATFORM_UART1_PIN_PORT, PLATFORM_UART1_TX_PIN|PLATFORM_UART1_RX_PIN);

    HAL_NVIC_DisableIRQ(USART1_IRQn);

    /* DMA DISABLE*/
	HAL_NVIC_DisableIRQ(DMA2_Channel6_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Channel7_IRQn);

    /* UART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);


  }
}

#if !PLATFORM_I2C1_DEVICE_ID
/**
* @brief This function handles DMA2 channel6 global interrupt.
*/
void DMA2_Channel6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((uartDevicePrivateData_t*) uart1Dev->privateData)->hdma_uart_tx);
}

/**
* @brief This function handles DMA2 channel7 global interrupt.
*/
void DMA2_Channel7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(((uartDevicePrivateData_t*) uart1Dev->privateData)->hdma_uart_rx);
}
#endif

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(((uartDevicePrivateData_t*)uart1Dev->privateData)->huart);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}


/*Load the Uart Driver module on startup*/

InitDevice(uartInit);
ExitDevice(uartExit);
#endif


#endif
