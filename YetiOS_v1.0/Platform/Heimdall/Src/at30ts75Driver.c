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
 * at30ts75Driver.c
 *
 *  Created on: 5 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file at30ts75Driver.c
 */

#include "yetiOS.h"
#ifdef USE_HEIMDALL_L4
#include "stm32l4xx_hal.h"
#endif

#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM
#if PLATFORM_AT30TS75_DEVICE_ID && PLATFORM_I2C1_DEVICE_ID	/*Do not compile the dirver if not defined the ID in the Conf file, or the I2C driver is not loaded*/
#include "deviceDriver.h"

#define TEMP_RESOLUTION		0.0039f
#define AT30TS75_I2C_ADDR	0x90
//#define AT30TS75_DEFAULT_TIMEOUT		osWaitForever


typedef struct at30ts75PrivateData_{
	deviceFileHandler_t* i2cDev;
	osMutexId at30ts75Mutex;
}at30ts75DevicePrivateData_t;


/*AT30TS75 and DMA HAL handles*/
#if PLATFORM_AT30TS75_DEVICE_ID
static devHandler_t* at30ts75Dev;
#endif

/*AT30TS75 Driver Init and Exit*/
static retval_t at30ts75Init(void);
static retval_t at30ts75Exit(void);

/*AT30TS75 DRIVER FUNCTIONS*/
static retval_t at30ts75Open(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t at30ts75Close(deviceFileHandler_t* devFile);
static uint32_t at30ts75Read(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t at30ts75Write(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t at30ts75Ioctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t at30ts75ChangedCpuFreq(devHandler_t* devHandler);


static deviceDriverOps_t at30ts75DriverOps = {
	.open = at30ts75Open,
	.close = at30ts75Close,
	.read = at30ts75Read,
	.write = at30ts75Write,
	.ioctl = at30ts75Ioctl,
	.changedCpuFreq = at30ts75ChangedCpuFreq,
};



/**
 *
 * @return
 */
static retval_t at30ts75Init(void){
	at30ts75DevicePrivateData_t* at30ts75PrivateData;
	uint8_t writeBuff[2];

	at30ts75Dev =  ytNewDevice(PLATFORM_AT30TS75_DEVICE_ID, &at30ts75DriverOps);
	at30ts75PrivateData = (at30ts75DevicePrivateData_t*)pvPortMalloc(sizeof(at30ts75DevicePrivateData_t));
	at30ts75Dev->privateData = (void*) at30ts75PrivateData;

	if( (at30ts75PrivateData->i2cDev = ytOpen(PLATFORM_I2C1_DEVICE_ID, (uint32_t) AT30TS75_I2C_ADDR)) == NULL){
		vPortFree(at30ts75PrivateData);
		ytDeleteDevice(at30ts75Dev);
		return RET_ERROR;
	}

	writeBuff[0] = 0x01;
	writeBuff[1] = 0x31;		//High resolution and Shutdown mode
	ytWrite(at30ts75PrivateData->i2cDev, writeBuff, 2);


	writeBuff[0] = 0x01;
	ytWrite(at30ts75PrivateData->i2cDev, writeBuff, 2);			//Set pointer to register 0x01

	at30ts75PrivateData->at30ts75Mutex = ytMutexCreate();
	if(ytRegisterDevice(at30ts75Dev) != RET_OK){
		osMutexDelete(at30ts75PrivateData->at30ts75Mutex);
		vPortFree(at30ts75PrivateData);
		ytDeleteDevice(at30ts75Dev);
		return RET_ERROR;
	}
	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t at30ts75Exit(void){
	at30ts75DevicePrivateData_t* at30ts75DevicePrivateData;
	uint8_t writeBuff[2];
	/*Unregister and Delete AT30TS751*/

	at30ts75DevicePrivateData = (at30ts75DevicePrivateData_t*) at30ts75Dev->privateData;
	ytUnregisterDevice(PLATFORM_AT30TS75_DEVICE_ID);

	writeBuff[0] = 0x01;
	writeBuff[1] = 0x01;		//Shutdown mode
	ytWrite(at30ts75DevicePrivateData->i2cDev, writeBuff, 2);
	ytClose(at30ts75DevicePrivateData->i2cDev);

	osMutexDelete(at30ts75DevicePrivateData->at30ts75Mutex);
	vPortFree(at30ts75DevicePrivateData);
	ytDeleteDevice(at30ts75Dev);

	return RET_OK;
}



/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t at30ts75Open(deviceFileHandler_t* devFile, uint32_t flags){
	at30ts75DevicePrivateData_t* at30ts75DevicePrivateData = (at30ts75DevicePrivateData_t*) devFile->device->privateData;
	uint8_t i2cBuff[2];
	osMutexWait(at30ts75DevicePrivateData->at30ts75Mutex, osWaitForever);
	devFile->privateData = NULL;

	i2cBuff[0] = 0x01;
	ytWrite(at30ts75DevicePrivateData->i2cDev, i2cBuff, 1);			//Set pointer to register 0x01

	ytRead(at30ts75DevicePrivateData->i2cDev, &i2cBuff[1], 1);		//Read register value

	i2cBuff[1] = i2cBuff[1] & 0xFE;
	ytWrite(at30ts75DevicePrivateData->i2cDev, i2cBuff, 2);			//Enable sensor

	i2cBuff[0] = 0x00;
	ytWrite(at30ts75DevicePrivateData->i2cDev, i2cBuff, 1);			//Set pointer to register 0x00

	osMutexRelease(at30ts75DevicePrivateData->at30ts75Mutex);
	return RET_OK;
}


static retval_t at30ts75Close(deviceFileHandler_t* devFile){
	at30ts75DevicePrivateData_t* at30ts75DevicePrivateData = (at30ts75DevicePrivateData_t*) devFile->device->privateData;
	uint8_t i2cBuff[2];
	osMutexWait(at30ts75DevicePrivateData->at30ts75Mutex, osWaitForever);
	i2cBuff[0] = 0x01;
	ytWrite(at30ts75DevicePrivateData->i2cDev, i2cBuff, 1);			//Set pointer to register 0x01

	ytRead(at30ts75DevicePrivateData->i2cDev, &i2cBuff[1], 1);		//Read register value

	i2cBuff[1] = i2cBuff[1] | 0x01;
	ytWrite(at30ts75DevicePrivateData->i2cDev, i2cBuff, 2);			//Disable sensor

	osMutexRelease(at30ts75DevicePrivateData->at30ts75Mutex);
	return RET_OK;
}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t at30ts75Read(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	at30ts75DevicePrivateData_t* at30ts75DevicePrivateData = (at30ts75DevicePrivateData_t*) devFile->device->privateData;
	float32_t* outVal = (float32_t*) buff;
	uint8_t i2cBuff[4];
	int16_t* readTemp;
	if(size != 1){
		return 0;
	}

	osMutexWait(at30ts75DevicePrivateData->at30ts75Mutex, osWaitForever);

	ytRead(at30ts75DevicePrivateData->i2cDev, i2cBuff, 2);		//Read temp register

	i2cBuff[2] = i2cBuff[1];
	i2cBuff[3] = i2cBuff[0];
	readTemp = (int16_t*) &i2cBuff[2];

	(*outVal) = TEMP_RESOLUTION * ((float32_t) (*readTemp));

	osMutexRelease(at30ts75DevicePrivateData->at30ts75Mutex);
	return size;
}


/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t at30ts75Write(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return 0;
}


/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t at30ts75Ioctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	return RET_ERROR;
}



/**
 *
 * @param devFile
 * @return
 */
static retval_t at30ts75ChangedCpuFreq(devHandler_t* devHandler){
	return RET_OK;
}


InitDevice_1(at30ts75Init);
ExitDevice_1(at30ts75Exit);
#endif

#endif
