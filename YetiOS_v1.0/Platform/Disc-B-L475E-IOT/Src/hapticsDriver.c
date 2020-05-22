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
 * hapticsDriver.c
 *
 *  Created on: Mar 27, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */


#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#if (PLATFORM_HAPTICS_DEVICE_ID)	/*Do not compile the driver if not defined any of the IDs in the Conf file*/

#include "stm32l4xx_hal.h"
#include "deviceDriver.h"
#include "lowPower.h"
#include "drv2605L.h"


typedef struct hapticsDevicePrivateData_{
	drv2605L_t* drv2605L;
}hapticsDevicePrivateData_t;

static devHandler_t* hapticsDev;

/*Haptics Driver Init and Exit*/
static retval_t hapticsInit(void);
static retval_t hapticsExit(void);

/*HAPTICS DRIVER FUNCTIONS*/
static retval_t hapticsOpen(deviceFileHandler_t* devFile, uint32_t flags);
static retval_t hapticsClose(deviceFileHandler_t* devFile);
static uint32_t hapticsRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static uint32_t hapticsWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size);
static retval_t hapticsIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);
static retval_t hapticsChangedCpuFreq(devHandler_t* devHandler);


static deviceDriverOps_t hapticsDriverOps = {
	.open = hapticsOpen,
	.close = hapticsClose,
	.read = hapticsRead,
	.write = hapticsWrite,
	.ioctl = hapticsIoctl,
	.changedCpuFreq = hapticsChangedCpuFreq,
};


static retval_t hapticsInit(void){
	hapticsDevicePrivateData_t* hapticsPrivateData;

	if(hapticsDev != NULL){
		return RET_ERROR;
	}

	hapticsDev = ytNewDevice(PLATFORM_HAPTICS_DEVICE_ID, &hapticsDriverOps);
	hapticsPrivateData = (hapticsDevicePrivateData_t*)pvPortMalloc(sizeof(hapticsDevicePrivateData_t));
	hapticsDev->privateData = (void*) hapticsPrivateData;

	if(ytRegisterDevice(hapticsDev) != RET_OK){
		vPortFree(hapticsDev->privateData);
	}

	return RET_OK;
}


static retval_t hapticsExit(void){
	hapticsDevicePrivateData_t* hapticsPrivateData = (hapticsDevicePrivateData_t*)hapticsDev->privateData;

	if(hapticsPrivateData == NULL){
		return RET_ERROR;
	}
	ytUnregisterDevice(PLATFORM_HAPTICS_DEVICE_ID);

	vPortFree(hapticsPrivateData);
	hapticsPrivateData = NULL;
	ytDeleteDevice(hapticsDev);
	return RET_OK;
}

static retval_t hapticsOpen(deviceFileHandler_t* devFile, uint32_t flags){
	hapticsDevicePrivateData_t* hapticsPrivateData = (hapticsDevicePrivateData_t*) devFile->device->privateData;

	switch(flags){

	case 1:
		drv2605L_create(&(hapticsPrivateData->drv2605L));
		drv2605L_init(hapticsPrivateData->drv2605L, I2C_ACTUATOR_1, GPIO_PIN_C3);
		drv2605L_exitStandby(hapticsPrivateData->drv2605L);
		drv2605L_loadActuatorConfig(hapticsPrivateData->drv2605L, &DRV2605L_ACTUATOR1);
		drv2605L_setMode(hapticsPrivateData->drv2605L, DRV2605L_MODE_SELECTION1);
		drv2605L_selectEffectLibrary(hapticsPrivateData->drv2605L, DRV2605L_LIBRARY1);
		break;

	case 2:
		drv2605L_create(&(hapticsPrivateData->drv2605L));
		drv2605L_init(hapticsPrivateData->drv2605L, I2C_ACTUATOR_2, GPIO_PIN_C3);
		drv2605L_exitStandby(hapticsPrivateData->drv2605L);
		drv2605L_loadActuatorConfig(hapticsPrivateData->drv2605L, &DRV2605L_ACTUATOR2);
		drv2605L_setMode(hapticsPrivateData->drv2605L, DRV2605L_MODE_SELECTION2);
		drv2605L_selectEffectLibrary(hapticsPrivateData->drv2605L, DRV2605L_LIBRARY2);
		break;

	case 3:
		drv2605L_create(&(hapticsPrivateData->drv2605L));
		drv2605L_init(hapticsPrivateData->drv2605L, I2C_ACTUATOR_2, GPIO_PIN_C3);
		drv2605L_exitStandby(hapticsPrivateData->drv2605L);
		drv2605L_loadActuatorConfig(hapticsPrivateData->drv2605L, &DRV2605L_ACTUATOR3);
		drv2605L_setMode(hapticsPrivateData->drv2605L, DRV2605L_MODE_SELECTION3);
		drv2605L_selectEffectLibrary(hapticsPrivateData->drv2605L, DRV2605L_LIBRARY3);
		break;

	}

	return RET_OK;
}

static retval_t hapticsClose(deviceFileHandler_t* devFile){
	hapticsDevicePrivateData_t* hapticsPrivateData = (hapticsDevicePrivateData_t*) devFile->device->privateData;

	if(drv2605L_delete (hapticsPrivateData->drv2605L) != RET_OK){
		return RET_ERROR;
	}

	return RET_OK;
}

static uint32_t hapticsRead(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return RET_OK;
}

static uint32_t hapticsWrite(deviceFileHandler_t* devFile, uint8_t* buff, uint32_t size){
	return RET_OK;
}


static retval_t hapticsIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	hapticsDevicePrivateData_t* hapticsPrivateData = (hapticsDevicePrivateData_t*) devFile->device->privateData;

	switch(command){

	case FIRE_HAPTIC_EFFECT:
		drv2605L_fireROMLibraryEffect(hapticsPrivateData->drv2605L, *((drv2605L_Effect_t*) args),true);
		break;

	default:
		return RET_ERROR;
		break;
	}

	return RET_OK;
}


static retval_t hapticsChangedCpuFreq(devHandler_t* devHandler){
	return RET_OK;
}


InitDevice(hapticsInit);
ExitDevice(hapticsExit);

#endif
#endif

