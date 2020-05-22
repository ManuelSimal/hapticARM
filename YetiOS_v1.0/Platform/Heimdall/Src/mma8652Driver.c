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
 * mma8652Driver.c
 *
 *  Created on: 10 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file mma8652Driver.c
 */

#include "yetiOS.h"
#ifdef USE_HEIMDALL_L4
#include "stm32l4xx_hal.h"
#endif

/*TODO: THIS DRIVER IS NOT DONE! Only implemented the Init function to set the accelerometer to sleep mode*/

#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM
#if PLATFORM_MMA8652_DEVICE_ID && PLATFORM_I2C1_DEVICE_ID	/*Do not compile the dirver if not defined the ID in the Conf file, or the I2C driver is not loaded*/
#include "deviceDriver.h"

#define MMA8652_I2C_ADDR	0x3A

#define MMA8652_CTRL_REG1	0x2A


typedef struct mma8652PrivateData_{
	deviceFileHandler_t* i2cDev;
	osMutexId mma8652Mutex;
}mma8652DevicePrivateData_t;


/*MMA8652 and DMA HAL handles*/
#if PLATFORM_MMA8652_DEVICE_ID
static devHandler_t* mma8652Dev;
#endif

/*MMA8652 Driver Init and Exit*/
static retval_t mma8652Init(void);
static retval_t mma8652Exit(void);

/*MMA8652 DRIVER FUNCTIONS*/
static retval_t mma8652Open(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t mma8652Close(deviceFileHandler_t* devFile);
static uint32_t mma8652Read(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t mma8652Write(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t mma8652Ioctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t mma8652ChangedCpuFreq(devHandler_t* devHandler);


static deviceDriverOps_t mma8652DriverOps = {
	.open = mma8652Open,
	.close = mma8652Close,
	.read = mma8652Read,
	.write = mma8652Write,
	.ioctl = mma8652Ioctl,
	.changedCpuFreq = mma8652ChangedCpuFreq,
};



/**
 *
 * @return
 */
static retval_t mma8652Init(void){
	mma8652DevicePrivateData_t* mma8652PrivateData;
	uint8_t writeBuff[2];

	mma8652Dev =  ytNewDevice(PLATFORM_MMA8652_DEVICE_ID, &mma8652DriverOps);
	mma8652PrivateData = (mma8652DevicePrivateData_t*)pvPortMalloc(sizeof(mma8652DevicePrivateData_t));
	mma8652Dev->privateData = (void*) mma8652PrivateData;

	if( (mma8652PrivateData->i2cDev = ytOpen(PLATFORM_I2C1_DEVICE_ID, (uint32_t) MMA8652_I2C_ADDR)) == NULL){
		vPortFree(mma8652PrivateData);
		ytDeleteDevice(mma8652Dev);
		return RET_ERROR;
	}

	writeBuff[0] = MMA8652_CTRL_REG1;
	writeBuff[1] = 0x00;								//Set standby mode
	ytWrite(mma8652PrivateData->i2cDev, writeBuff, 2);



	mma8652PrivateData->mma8652Mutex = ytMutexCreate();
	if(ytRegisterDevice(mma8652Dev) != RET_OK){
		osMutexDelete(mma8652PrivateData->mma8652Mutex);
		vPortFree(mma8652PrivateData);
		ytDeleteDevice(mma8652Dev);
		return RET_ERROR;
	}
	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t mma8652Exit(void){
	mma8652DevicePrivateData_t* mma8652DevicePrivateData;
	uint8_t writeBuff[2];
	/*Unregister and Delete MMA86521*/

	mma8652DevicePrivateData = (mma8652DevicePrivateData_t*) mma8652Dev->privateData;
	ytUnregisterDevice(PLATFORM_MMA8652_DEVICE_ID);

	writeBuff[0] = MMA8652_CTRL_REG1;
	writeBuff[1] = 0x00;								//Set standby mode
	ytWrite(mma8652DevicePrivateData->i2cDev, writeBuff, 2);
	ytClose(mma8652DevicePrivateData->i2cDev);

	osMutexDelete(mma8652DevicePrivateData->mma8652Mutex);
	vPortFree(mma8652DevicePrivateData);
	ytDeleteDevice(mma8652Dev);

	return RET_OK;
}



/**
 *
 * @param devFile
 * @param flags
 * @return
 */
static retval_t mma8652Open(deviceFileHandler_t* devFile, uint32_t flags){
	mma8652DevicePrivateData_t* mma8652DevicePrivateData = (mma8652DevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(mma8652DevicePrivateData->mma8652Mutex, osWaitForever);

	osMutexRelease(mma8652DevicePrivateData->mma8652Mutex);
	return RET_ERROR;
}


static retval_t mma8652Close(deviceFileHandler_t* devFile){
	mma8652DevicePrivateData_t* mma8652DevicePrivateData = (mma8652DevicePrivateData_t*) devFile->device->privateData;

	osMutexWait(mma8652DevicePrivateData->mma8652Mutex, osWaitForever);

	osMutexRelease(mma8652DevicePrivateData->mma8652Mutex);
	return RET_ERROR;
}

/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t mma8652Read(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	mma8652DevicePrivateData_t* mma8652DevicePrivateData = (mma8652DevicePrivateData_t*) devFile->device->privateData;
	osMutexWait(mma8652DevicePrivateData->mma8652Mutex, osWaitForever);

	osMutexRelease(mma8652DevicePrivateData->mma8652Mutex);
	return 0;
}


/**
 *
 * @param devFile
 * @param buff
 * @param size
 * @return
 */
static uint32_t mma8652Write(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return 0;
}


/**
 *
 * @param devFile
 * @param command
 * @param args
 * @return
 */
static retval_t mma8652Ioctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	return RET_ERROR;
}



/**
 *
 * @param devFile
 * @return
 */
static retval_t mma8652ChangedCpuFreq(devHandler_t* devHandler){
	return RET_OK;
}


InitDevice_1(mma8652Init);
ExitDevice_1(mma8652Exit);
#endif

#endif
