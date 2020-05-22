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
 * gyroDriver.c
 *
 *  Created on: 28 feb. 2020
 *      Author: Santiago Isidro Real <sreal@b105.upm.es>
 */




#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#if (PLATFORM_GYRO_DEVICE_ID)	/*Do not compile the driver if not defined any of the IDs in the Conf file*/
#include "stm32l4xx_hal.h"
#include "deviceDriver.h"
#include "lowPower.h"
#include "lsm6dsl.h"


typedef struct gyroDevicePrivateData_{
	LSM6DSL_Object_t* lsm6dsl_obj;
}gyroDevicePrivateData_t;

static devHandler_t* gyroDev;

/*SPI Driver Init and Exit*/
static retval_t gyroInit(void);
static retval_t gyroExit(void);

/*SPI DRIVER FUNCTIONS*/
static retval_t gyroOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t gyroClose(deviceFileHandler_t* devFile);
static uint32_t gyroRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t gyroWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t gyroIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t gyroChangedCpuFreq(devHandler_t* devHandler);

static deviceDriverOps_t gyroDriverOps = {
	.open = gyroOpen,
	.close = gyroClose,
	.read = gyroRead,
	.write = gyroWrite,
	.ioctl = gyroIoctl,
	.changedCpuFreq = gyroChangedCpuFreq,
};


static retval_t gyroInit(void){
	gyroDevicePrivateData_t* gyroPrivateData;
	uint8_t id = 0;

	if(gyroDev != NULL){
		return RET_ERROR;
	}

	gyroDev = ytNewDevice(PLATFORM_GYRO_DEVICE_ID, &gyroDriverOps);
	gyroPrivateData = (gyroDevicePrivateData_t*)pvPortMalloc(sizeof(gyroDevicePrivateData_t));
	gyroDev->privateData = (void*) gyroPrivateData;
	gyroPrivateData->lsm6dsl_obj = LSM6DSL_GetDefaultObj();

	if(ytRegisterDevice(gyroDev) != RET_OK){
		vPortFree(gyroDev->privateData);
	}

	if(LSM6DSL_Init(gyroPrivateData->lsm6dsl_obj) != LSM6DSL_OK){
		return RET_ERROR;
	}

	if((LSM6DSL_ReadID(gyroPrivateData->lsm6dsl_obj, &id) != LSM6DSL_OK) || (id != LSM6DSL_WHO_AM_I_ID)){
		return RET_ERROR;
	}

	return RET_OK;
}


static retval_t gyroExit(void){
	gyroDevicePrivateData_t* gyroPrivateData = (gyroDevicePrivateData_t*)gyroDev->privateData;

	if(gyroPrivateData == NULL){
		return RET_ERROR;
	}
	ytUnregisterDevice(PLATFORM_GYRO_DEVICE_ID);

	vPortFree(gyroPrivateData);
	gyroPrivateData = NULL;
	ytDeleteDevice(gyroDev);
	return RET_OK;
}

static retval_t gyroOpen(deviceFileHandler_t* devFile, uint32_t flags){
	gyroDevicePrivateData_t* gyroPrivateData = (gyroDevicePrivateData_t*) devFile->device->privateData;

	if(LSM6DSL_GYRO_Enable(gyroPrivateData->lsm6dsl_obj) != LSM6DSL_OK){
		return RET_ERROR;
	}

	return RET_OK;
}

static retval_t gyroClose(deviceFileHandler_t* devFile){
	gyroDevicePrivateData_t* gyroPrivateData = (gyroDevicePrivateData_t*) devFile->device->privateData;
	if(LSM6DSL_GYRO_Disable(gyroPrivateData->lsm6dsl_obj) != LSM6DSL_OK){
		return RET_ERROR;
	}
	//LSM6DSL_DeInit(gyroPrivateData->lsm6dsl_obj);
	return RET_OK;
}

static uint32_t gyroRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	gyroDevicePrivateData_t* gyroPrivateData = (gyroDevicePrivateData_t*) devFile->device->privateData;

	if(LSM6DSL_GYRO_GetAxes(gyroPrivateData->lsm6dsl_obj, (LSM6DSL_Axes_t*) buff) != LSM6DSL_OK){
		return RET_ERROR;
	}

	return RET_OK;
}

static uint32_t gyroWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return RET_OK;
}


static retval_t gyroIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	return RET_OK;
}


static retval_t gyroChangedCpuFreq(devHandler_t* devHandler){
	return RET_OK;
}


InitDevice_1(gyroInit);
ExitDevice_1(gyroExit);

#endif
#endif




