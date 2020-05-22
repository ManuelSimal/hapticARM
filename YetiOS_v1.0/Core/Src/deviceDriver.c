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
 * deviceDriver.c
 *
 *  Created on: 26 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file deviceDriver.c
 */

#include "types.h"
#include "genList.h"
#include "yetiOS.h"
#include "deviceDriver.h"

#define DEVICE_DRIVER_WAIT_TIMEOUT 			200

/*List containing all the device drivers registered*/
static genList_t * deviceList = NULL;

static uint16_t checkDeviceId(uint32_t devId);
static devHandler_t* getDeviceFromId(uint32_t devId);
static uint16_t checkDeviceInList(devHandler_t* device);

/**
 * @brief	Call the init functions of the compiled devices
 * @return	Return status
 */
retval_t ytInitDevices(void){
	/*Defined by the linker*/
	extern uint32_t (*_dev_initcall_start_0)(void);
	extern uint32_t (*_dev_initcall_end_0)(void);

	extern uint32_t (*_dev_initcall_start_1)(void);
	extern uint32_t (*_dev_initcall_end_1)(void);

	extern uint32_t (*_dev_initcall_start_2)(void);
	extern uint32_t (*_dev_initcall_end_2)(void);

	extern uint32_t (*_dev_initcall_start_3)(void);
	extern uint32_t (*_dev_initcall_end_3)(void);

	/*Initialize devices list*/
	deviceList = genListInit();

	uint32_t* start = (uint32_t*) &_dev_initcall_start_0;
	uint32_t* end = (uint32_t*) &_dev_initcall_end_0;


	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	start = (uint32_t*) &_dev_initcall_start_1;
	end = (uint32_t*) &_dev_initcall_end_1;

	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	start = (uint32_t*) &_dev_initcall_start_2;
	end = (uint32_t*) &_dev_initcall_end_2;

	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	start = (uint32_t*) &_dev_initcall_start_3;
	end = (uint32_t*) &_dev_initcall_end_3;

	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	return RET_OK;
};


/**
 * @brief	Call the exit functions of the devices compiled
 * @return	Return status
 */
retval_t ytExitDevices(void){
	/*Defined by the linker*/
	extern uint32_t (*_dev_exitcall_start_0)(void);
	extern uint32_t (*_dev_exitcall_end_0)(void);

	extern uint32_t (*_dev_exitcall_start_1)(void);
	extern uint32_t (*_dev_exitcall_end_1)(void);

	extern uint32_t (*_dev_exitcall_start_2)(void);
	extern uint32_t (*_dev_exitcall_end_2)(void);

	extern uint32_t (*_dev_exitcall_start_3)(void);
	extern uint32_t (*_dev_exitcall_end_3)(void);

	uint32_t* start = (uint32_t*) &_dev_exitcall_start_0;
	uint32_t* end = (uint32_t*) &_dev_exitcall_end_0;

	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	start = (uint32_t*) &_dev_exitcall_start_1;
	end = (uint32_t*) &_dev_exitcall_end_1;

	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	start = (uint32_t*) &_dev_exitcall_start_2;
	end = (uint32_t*) &_dev_exitcall_end_2;

	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	start = (uint32_t*) &_dev_exitcall_start_3;
	end = (uint32_t*) &_dev_exitcall_end_3;

	genListRemoveAndDeleteAll(deviceList);
	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func != NULL){
			func();
		}
		start++;
	}

	return RET_OK;
};

/**
 *
 * @param deviceId
 * @param ops
 * @return
 */
devHandler_t* ytNewDevice(uint32_t deviceId, deviceDriverOps_t* ops){
	devHandler_t* newDevice = (devHandler_t*) pvPortMalloc(sizeof(devHandler_t));
	newDevice->devId = deviceId;
	newDevice->devOpened = 0;
	newDevice->deviceLock = 0;
	newDevice->ops = ops;
	newDevice->deviceFileHandlerList = genListInit();
	return newDevice;
}

/**
 *
 * @param dev
 * @return
 */
retval_t ytDeleteDevice(devHandler_t* dev){
	if(dev == NULL){
		return RET_ERROR;
	}
	genListRemoveAll(dev->deviceFileHandlerList);
	vPortFree(dev);
	return RET_OK;

}

/**
 *
 * @param deviceId
 * @param ops
 * @return
 */
retval_t ytRegisterDevice(devHandler_t* dev){
	osThreadSuspendAll();	/*Enter critical*/

	if(checkDeviceId(dev->devId)){	/*If the ID already exists, return ERROR*/
		osThreadResumeAll();
		return RET_ERROR;
	}


	genListAdd(deviceList, (void*) dev);

	osThreadResumeAll();
	return RET_OK;
}

/**
 *
 * @param device_id
 * @param name
 * @return
 */
devHandler_t* ytUnregisterDevice(uint32_t deviceId){
	devHandler_t* device;
	uint32_t initTick;
	osThreadSuspendAll();	/*Enter critical*/

	if((device = getDeviceFromId(deviceId)) == NULL){	/*The device does not exist*/
		osThreadResumeAll();
		return NULL;
	}
	else{
		initTick = osKernelSysTick();
		while(device->deviceLock){	/*Securely wait until the driver is not used by anyone*/
			osThreadResumeAll();
			osDelay(1);
			osThreadSuspendAll();
			if((osKernelSysTick() - initTick) >= DEVICE_DRIVER_WAIT_TIMEOUT){	/*Timeout reached*/
				osThreadResumeAll();
				return NULL;
			}
		}

		while(device->devOpened){	/*First, properly close the device files that are opened*/
			deviceFileHandler_t* devFile = (deviceFileHandler_t*) genListGetLast(device->deviceFileHandlerList);
			device->ops->close(devFile);
			genListRemove(devFile->device->deviceFileHandlerList, devFile);
			vPortFree(devFile);
			device->devOpened--;
		}

		/*Finally remove it*/
		genListRemove(deviceList, (void*) device);
		osThreadResumeAll();
	}
	return device;
}

/**
 *
 * @param deviceId
 * @param flags
 * @return
 */
deviceFileHandler_t* ytOpen(uint32_t deviceId, uint32_t flags){
	devHandler_t* device;
	deviceFileHandler_t* devFile;
	osThreadSuspendAll();/*Secure checks if the device exists;*/
	if((device = getDeviceFromId(deviceId)) == NULL){	/*The device does not exist*/
		osThreadResumeAll();
		return NULL;
	}
	else{
		/*Create the device file to be opened*/
		if(device->devOpened < MAX_DEVICE_FILE_HANDLERS){
			devFile = (deviceFileHandler_t*)pvPortMalloc(sizeof(deviceFileHandler_t));
			devFile->device = device;
			devFile->devFileReady = 0;
			genListAdd(device->deviceFileHandlerList, (void*) devFile);
			device->devOpened++;
		}
		else{
			osThreadResumeAll();
			return NULL;
		}
		device->deviceLock++;
		ytPeripheralFreqChangeLock();	/*Prevents changing the cpu clock frequency while using a device driver*/
		osThreadResumeAll();
	}

	/*Call device Open function*/
	if(device->ops->open(devFile, flags) == RET_ERROR){	/*An error happened*/
		genListRemove(device->deviceFileHandlerList, devFile);
		vPortFree(devFile);
		device->devOpened--;
		ytPeripheralFreqChangeUnlock();
		device->deviceLock--;
		return NULL;
	}

	/*Set the device as opened and free the lock*/
	devFile->devFileReady++;
	ytPeripheralFreqChangeUnlock();
	device->deviceLock--;

	return devFile;

}

/**
 *
 * @param dev
 * @return
 */
retval_t ytClose(deviceFileHandler_t* devFile){

	osThreadSuspendAll();/*Secure checks if the device exists;*/
	if(!checkDeviceInList(devFile->device)){	/*The device does not exist*/
		osThreadResumeAll();
		return RET_ERROR;
	}
	else{		/*The device exists. Check if it is opened*/
		if(!devFile->devFileReady){
			osThreadResumeAll();
			return RET_ERROR;
		}
		/*If opened capture the locks*/
		devFile->device->deviceLock++;
		ytPeripheralFreqChangeLock();	/*Prevents changing the cpu clock frequency while using a device driver*/
		devFile->devFileReady = 0;/*Device not ready anymore*/
		osThreadResumeAll();
	}
	/*Call device Close function*/
	if(devFile->device->ops->close(devFile) == RET_ERROR){
		ytPeripheralFreqChangeUnlock();
		devFile->device->deviceLock--;
		devFile->devFileReady++;	/*Remains ready if an error happened closing*/
		return RET_ERROR;
	}

	/*Decrease the opened counter and release locks*/
	genListRemove(devFile->device->deviceFileHandlerList, devFile);
	devFile->device->devOpened--;
	ytPeripheralFreqChangeUnlock();
	devFile->device->deviceLock--;
	vPortFree(devFile);

	return RET_OK;

}

/**
 *
 * @param dev
 * @param buf
 * @param bytes
 * @return
 */
uint32_t ytRead(deviceFileHandler_t* devFile, uint8_t* buf, uint32_t bytes){
	uint32_t retBytes;
	osThreadSuspendAll();/*Securelly checks if the device exists;*/
	if(checkDeviceInList(devFile->device)){	/*The device is on the list. Capture a lock. This prevents deleting the driver while it is being used*/
		if(!devFile->devFileReady){	/*The device must be opened to use it*/
			osThreadResumeAll();
			return 0;
		}
		devFile->device->deviceLock++;
		ytPeripheralFreqChangeLock();	/*Prevents changing the cpu clock frequency while using a device driver*/
	}
	else{
		osThreadResumeAll();
		return 0;
	}
	osThreadResumeAll();

	/*Call the device read function*/
	retBytes = devFile->device->ops->read(devFile, buf, bytes);
	/*Free the lock*/
	devFile->device->deviceLock--;
	ytPeripheralFreqChangeUnlock();

	return retBytes;

}

/**
 *
 * @param dev
 * @param buf
 * @param bytes
 * @return
 */
uint32_t ytWrite(deviceFileHandler_t* devFile, uint8_t* buf, uint32_t bytes){
	uint32_t retBytes;
	osThreadSuspendAll();/*Securelly checks if the device exists;*/
	if(checkDeviceInList(devFile->device)){	/*The device is on the list. Capture a lock. This prevents deleting the driver while it is being used*/
		if(!devFile->devFileReady){	/*The device must be opened to use it*/
			osThreadResumeAll();
			return 0;
		}
		devFile->device->deviceLock++;
		ytPeripheralFreqChangeLock();	/*Prevents changing the cpu clock frequency while using a device driver*/
	}
	else{
		osThreadResumeAll();
		return 0;
	}
	osThreadResumeAll();

	/*Call the device read function*/
	retBytes = devFile->device->ops->write(devFile, buf, bytes);
	/*Free the lock*/
	devFile->device->deviceLock--;
	ytPeripheralFreqChangeUnlock();

	return retBytes;
}

/**
 *
 * @param dev
 * @param command
 * @param args
 * @return
 */
retval_t ytIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args){
	retval_t retVal;
	osThreadSuspendAll();/*Securelly checks if the device exists;*/
	if(checkDeviceInList(devFile->device)){	/*The device is on the list. Capture a lock. This prevents deleting the driver while it is being used*/
		if(!devFile->devFileReady){	/*The device must be opened to use it*/
			osThreadResumeAll();
			return 0;
		}
		devFile->device->deviceLock++;
		ytPeripheralFreqChangeLock();	/*Prevents changing the cpu clock frequency while using a device driver*/
	}
	else{
		osThreadResumeAll();
		return 0;
	}
	osThreadResumeAll();

	/*Call the device read function*/
	retVal = devFile->device->ops->ioctl(devFile, command, args);
	/*Free the lock*/
	devFile->device->deviceLock--;
	ytPeripheralFreqChangeUnlock();

	return retVal;
}

/**
 * @brief 	Calls the changedCpuFreq of every Device Driver loaded to update the peripheral clocks if necessary
 * @return
 */
retval_t ytUpdateDevicesCpuFreq(){
	if(deviceList == NULL){
		return RET_ERROR;
	}
	devHandler_t* storedDevice;
	genListElement_t* currentDev = deviceList->tailElement;
	while(currentDev != NULL){
		storedDevice = (devHandler_t*) currentDev->item;
		storedDevice->ops->changedCpuFreq(storedDevice);
		currentDev = currentDev->next;
	}
	return RET_OK;
}
/**
 * @brief			Checks if the id selected exists in the device list. Private function
 * @param dev_id	The device id to be checked
 * @return			Return distint to zero if the device id exists
 */
static uint16_t checkDeviceId(uint32_t devId){
	devHandler_t* device;
	genListElement_t* current = deviceList->tailElement;
	while(current != NULL){
		device = (devHandler_t*) current->item;
		if(device->devId == devId){
			return 1;
			break;
		}
		current = current->next;
	}
	return 0;
}

/**
 * @brief			Gets the device struct handler from a device id. Private function
 * @param dev_id	Device id to be obtained
 * @return			Device struct handler. Null if the device id does not exist
 */
static devHandler_t* getDeviceFromId(uint32_t devId){

	devHandler_t* device;
	genListElement_t* current = deviceList->tailElement;
	while(current != NULL){
		device = (devHandler_t*) current->item;
		if(device->devId == devId){
			return device;
			break;
		}
		current = current->next;
	}
	return NULL;
}

/**
 *
 * @param device
 * @return
 */
static uint16_t checkDeviceInList(devHandler_t* device){

	devHandler_t* storedDevice;
	genListElement_t* current = deviceList->tailElement;
	while(current != NULL){
		storedDevice = (devHandler_t*) current->item;
		if(storedDevice == device){
			return 1;
			break;
		}
		current = current->next;
	}
	return 0;
}
