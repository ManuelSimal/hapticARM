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
 * usbDriver.c
 *
 *  Created on: 1 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file usbDriver.c
 */

#include "yetiOS.h"

#if !CONFIG_USE_TRACEALYZER_SW
#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM

#if PLATFORM_USB_DEVICE_ID 		/*Do not compile the dirver if not defined its ID in the Conf file*/
/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  256
#define APP_TX_DATA_SIZE  256

#define DEFAULT_USB_WRITE_TIMEOUT		500

typedef enum configReadMode_{
	USB_READ_STANDARD,
	USB_READ_CIRCULAR_BUF,
}configReadMode_t;

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
/** Data to send over USB CDC are stored in this buffer   */
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

static volatile uint16_t readingFlag = 0;	/*Flag to check if someone is currently using read function*/
static volatile uint16_t openedFlag = 0;	/*Flag to indicate if the device is already opened. Only one USB device opened allowed*/
static osMutexId usbOpMutex;				/*Mutex to lock reading and opened Flags. These variables are independent but can share the same mutex*/
static osMutexId usbWriteMutex;				/*Mutex to allow only one simultaneous writing*/

static configReadMode_t configReadMode;		/*READ MODE CURRENTLY CONFIGURED*/
static osSemaphoreId readEndSemaphore;		/*Semaphore to indicate a standard read operation is ended*/
static osSemaphoreId usbDataAvailableSemph;	/*Semaphore indicating new data is available when using async circular buffer read*/

static uint8_t* readBufferPtr;				/*Pointer to the read operation buffer*/
static uint8_t* currentReadBufferPtr;		/*Current position in the read buffer*/
static uint8_t* endReadBufferPtr;			/*End Position of the read buffer*/

static void(* readCompleteCb)(void);		/*Callback function called when the circular buffer reaches its end*/

/*Low layer CDC USB functions*/
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);


/*USB Driver Init and Exit*/
static retval_t usbInit(void);
static retval_t usbExit(void);

/*USB DRIVER FUNCTIONS*/
static retval_t usbOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t usbClose(deviceFileHandler_t* devFile);
static uint32_t usbRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t usbWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t usbIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t usbChangedCpuFreq(devHandler_t* devHandler);


/*USB Driver Ops*/
static deviceDriverOps_t usbDriverOps = {
	.open = usbOpen,
	.close = usbClose,
	.read = usbRead,
	.write = usbWrite,
	.ioctl = usbIoctl,
	.changedCpuFreq = usbChangedCpuFreq,
};



/*CDC Low layer Ops*/
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};


/**
 *
 * @return
 */
static retval_t usbInit(void){
	devHandler_t* usbDev;				/*Usb device handler used*/
	if((platformRunMode_t)ytGetCurrentRunMode() != RUN_MODE_48MHZ){
		return RET_ERROR;
	}
	ytPeripheralFreqChangeLock();						/*CPU Frequency Changes not allowed while using the USB Driver. It must run at 48 MHz*/
	ytSetDeviceLowPowerMode((uint32_t) SLEEP_MODE);	/*SLEEP MODE is the only low power mode allowed*/
	readingFlag = 0;
	openedFlag = 0;
	readCompleteCb = NULL;
	usbOpMutex = ytMutexCreate();
	usbWriteMutex = ytMutexCreate();
	readEndSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(readEndSemaphore, osWaitForever);
	usbDataAvailableSemph = ytSemaphoreCreate(1);
	osSemaphoreWait(usbDataAvailableSemph, osWaitForever);
	configReadMode = USB_READ_STANDARD;
	MX_USB_DEVICE_Init();
	osDelay(100);	/*Some delay is required after initializing the usb hw*/

	usbDev =  ytNewDevice(PLATFORM_USB_DEVICE_ID, &usbDriverOps);
	if(ytRegisterDevice(usbDev) != RET_OK){
		osMutexDelete(usbOpMutex);
		osMutexDelete(usbWriteMutex);
		osSemaphoreDelete(readEndSemaphore);
		osSemaphoreDelete(usbDataAvailableSemph);
		ytSetDeviceDefaultLowPowerMode();
		ytPeripheralFreqChangeUnlock();
		return RET_ERROR;
	}
	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t usbExit(void){
	devHandler_t* usbDev;				/*Usb device handler used*/
	if((platformRunMode_t)ytGetCurrentRunMode() != RUN_MODE_48MHZ){
		return RET_ERROR;
	}
	osMutexWait(usbOpMutex, osWaitForever);
	usbDev = ytUnregisterDevice(PLATFORM_USB_DEVICE_ID);
	MX_USB_DEVICE_DeInit();
	openedFlag = 0;
	if(readingFlag){
		if(configReadMode == USB_READ_STANDARD){
			osSemaphoreRelease(readEndSemaphore);
		}
		else{
			osSemaphoreRelease(usbDataAvailableSemph);
		}
		readingFlag = 0;
	}
	readCompleteCb = NULL;
	osMutexRelease(usbOpMutex);

	osMutexDelete(usbOpMutex);
	osMutexDelete(usbWriteMutex);
	osSemaphoreDelete(readEndSemaphore);
	osSemaphoreDelete(usbDataAvailableSemph);
	ytSetDeviceDefaultLowPowerMode();
	ytPeripheralFreqChangeUnlock();
	ytDeleteDevice(usbDev);
	return RET_OK;
}

/*USB DRIVER FUNCTIONS*/
/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t usbOpen(deviceFileHandler_t* devFile, uint32_t flags){
	retval_t ret = RET_OK;
	osMutexWait(usbOpMutex, osWaitForever);
	if(openedFlag){
		ret = RET_ERROR;
	}
	else{
		openedFlag++;
	}
	osMutexRelease(usbOpMutex);
	return ret;
}

/**
 *
 * @param devFile
 * @return
 */
static retval_t usbClose(deviceFileHandler_t* devFile){
	retval_t ret = RET_OK;
	osMutexWait(usbOpMutex, osWaitForever);
	if(!openedFlag){
		ret = RET_ERROR;
	}
	else{
		if(readingFlag){
			if(configReadMode == USB_READ_STANDARD){
				osSemaphoreRelease(readEndSemaphore);
			}
			readingFlag = 0;
		}
		openedFlag--;
	}
	readCompleteCb = NULL;
	osMutexRelease(usbOpMutex);
	return ret;
}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t usbRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	if((size == 0) || (size > APP_RX_DATA_SIZE)){
		return 0;
	}

	osMutexWait(usbOpMutex, osWaitForever);
	if(openedFlag){
		if(readingFlag){	/*Someone else is reading*/
			osMutexRelease(usbOpMutex);
			return 0;
		}
		else{
			readBufferPtr = buff;
			currentReadBufferPtr = buff;
			endReadBufferPtr = &buff[size];
			readingFlag++;
			osMutexRelease(usbOpMutex);

			if(configReadMode == USB_READ_STANDARD){
				osSemaphoreWait(readEndSemaphore, osWaitForever);
				return size;
			}

		}
	}
	else{
		osMutexRelease(usbOpMutex);
	}
	return 0;
}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t usbWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	if((size == 0) || (size > APP_TX_DATA_SIZE)){
		return 0;
	}

	if(openedFlag){	/*Not need to use mutex. A race condition may happen but it doesnt matters since the usb will terminate sending
	 	 	 	 	 *anyway (even if it has been closed)*/

		osMutexWait(usbWriteMutex, osWaitForever);	/*Only one simultaneous write allowed*/

		USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
		if (hcdc->TxState != 0){
		  osMutexRelease(usbWriteMutex);
		  return 0;
		}
		/*Return ERROR if the USB device is not actually connected*/
		if((hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) && (hUsbDeviceFS.dev_state == USBD_STATE_DEFAULT)){
		  osMutexRelease(usbWriteMutex);
		  return 0;
		}

		USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buff, size);
		USBD_CDC_TransmitPacket(&hUsbDeviceFS);

		uint32_t startTime = osKernelSysTick();
		while(hcdc->TxState == 1){					/*Block until write is done*/
			if((osKernelSysTick() - startTime) > DEFAULT_USB_WRITE_TIMEOUT){
				osMutexRelease(usbWriteMutex);
				return 0;
			}
		}
		osMutexRelease(usbWriteMutex);
	}
	else{
		return 0;
	}
	  return size;
}

/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t usbIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	retval_t ret = RET_OK;
	osMutexWait(usbOpMutex, osWaitForever);
	if(openedFlag){
		osMutexRelease(usbOpMutex);
		switch(command){
		case SET_ASYNC_CIRCULAR_READ_MODE:
			osMutexWait(usbOpMutex, osWaitForever);
			if(readingFlag){	/*Not allowed changing this configuration while reading*/
				ret = RET_ERROR;
				osMutexRelease(usbOpMutex);
			}
			else{
				configReadMode = USB_READ_CIRCULAR_BUF;
				osMutexRelease(usbOpMutex);
			}
			break;
		case STOP_ASYNC_READ:
			osMutexWait(usbOpMutex, osWaitForever);
			if(readingFlag){
				readingFlag = 0;
				ret = RET_OK;
			}
			osMutexRelease(usbOpMutex);
			break;
		case GET_CURRENT_READ_PTR:
			if(args == NULL){
				ret = RET_ERROR;
			}
			else{
				osMutexWait(usbOpMutex, osWaitForever);
				if(readingFlag){	/*Only return a valid value when reading*/
					uint8_t** retPtr = (uint8_t**) args;
					(*retPtr) = currentReadBufferPtr;
					osMutexRelease(usbOpMutex);
				}
				else{
					ret = RET_ERROR;
					osMutexRelease(usbOpMutex);
				}
			}

			break;
		case WAIT_FOR_NEW_DATA:
			osSemaphoreWait(usbDataAvailableSemph, *((uint32_t*)args));	/*The timeout is passed in args*/
			break;
		case SET_READ_COMPLETE_CB:
			readCompleteCb = args;
			break;
		case SET_STD_READ_MODE:
			osMutexWait(usbOpMutex, osWaitForever);
			if(readingFlag){	/*Not allowed changing this configuration while reading*/
				ret = RET_ERROR;
				osMutexRelease(usbOpMutex);
			}
			else{
				configReadMode = USB_READ_STANDARD;
				osMutexRelease(usbOpMutex);
			}
			break;
		default:
			ret = RET_ERROR;
		}
	}
	else{
		ret = RET_ERROR;
		osMutexRelease(usbOpMutex);
	}
	return ret;
}
static retval_t usbChangedCpuFreq(devHandler_t* devHandler){
	return RET_OK;
}


/* *******************************************/
/* **********LOW LAYER USB FUNCTIONS**********/
/**
 *
 * @return
 */
static int8_t CDC_Init_FS(void)
{
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

/**
 *
 * @return
 */
static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

/**
 *
 * @param cmd
 * @param pbuf
 * @param length
 * @return
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  return (USBD_OK);
}

/**
 *
 * @param Buf
 * @param Len
 * @return
 */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
	uint32_t rcvLen = (*Len);
	uint8_t* pBuf = Buf;

	if(readingFlag && openedFlag){	/*Check if someone is reading. Dont use mutex since this function is called from ISR*/

		if(configReadMode == USB_READ_CIRCULAR_BUF){
			while(rcvLen){	/*Copy received data in the circular buffer*/
				if(currentReadBufferPtr >= endReadBufferPtr){
					currentReadBufferPtr = readBufferPtr;
					if(readCompleteCb != NULL){	/*Notify when the end of the circular buffer is reached*/
						readCompleteCb();
					}
				}
				(*currentReadBufferPtr) = (*pBuf);
				pBuf++;
				currentReadBufferPtr++;
				rcvLen--;
			}
			/*Notify when new data are stored*/
			osSemaphoreRelease(usbDataAvailableSemph);
		}
		else if(configReadMode == USB_READ_STANDARD){

			while(rcvLen > 0){
				(*currentReadBufferPtr) = (*pBuf);
				currentReadBufferPtr++ ;
				rcvLen--;
				pBuf++;
				if(currentReadBufferPtr >= endReadBufferPtr){	/*The required bytes have been read*/
					readingFlag = 0;
					osSemaphoreRelease(readEndSemaphore);
					break;
				}
			}
		}
	}


	/*Continue reading*/
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	return (USBD_OK);

}

/**
 * @brief		This function isnt used in yetiOS. Used usbWrite instead
 * @param Buf
 * @param Len
 * @return
 */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  return result;
}


/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}


InitDevice(usbInit);
ExitDevice(usbExit);

#endif

#endif
#endif
