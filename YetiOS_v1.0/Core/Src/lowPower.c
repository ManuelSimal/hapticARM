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
 * lowPower.c
 *
 *  Created on: 17 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file lowPower.c
 */

#include "yetiOS.h"
#include "lowPower.h"

#define SET_RUN_MODE_TIMEOUT	3000

#ifndef PLATFORM_NUM_LOW_POWER_MODES
#define PLATFORM_NUM_LOW_POWER_MODES	0
#endif

#ifndef PLATFORM_NUM_RUN_MODES
#define PLATFORM_NUM_RUN_MODES			0
#endif

static uint32_t currentLowPowerModeConsumption = 0;
static uint32_t currentLowPowerMode = 0;
static uint16_t lowPowerBlockingPeripherals = 0;
static uint32_t defaultLowPowerMode = 0;
static uint32_t devicePowerModeChanged = 0;

static uint32_t currentRunModeConsumption = 0;
static uint32_t currentRunMode = 0;
static uint32_t usingAnyPeripheral = 0;

static osMutexId lowPowerModeMutex;

/*Variables used for current consumption estimation*/
static uint32_t estimatedAcumulatedConsumption = 0;
static uint32_t estimationInitUs = 0;
static uint32_t estimationAcumulatedLowPowerUs = 0;
static uint32_t estimationLapsedUs = 0;

static uint32_t microsecondsForOneTick = 0;

/*Platform functions declarations that might be defined. If not, use default __weak definitions*/
extern void platformInitLowPowerManager();
extern uint32_t platformUpdateLowPowerModeConsumption();
extern retval_t platformSetCurrentLowPowerMode(uint32_t newLowPowerMode);
extern uint32_t platformGetCurrentLowPowerMode(void);

extern uint32_t platformUpdateRunModeConsumption();
extern retval_t platformSetCurrentRunMode(uint32_t newRunMode);
extern uint32_t platformGetCurrentRunMode(void);
/**
 *
 * @return
 */
retval_t ytInitLowPowerManager(){
	estimatedAcumulatedConsumption = 0;
	estimationInitUs = 0;
	estimationAcumulatedLowPowerUs = 0;
	currentLowPowerMode = 0;
	estimationLapsedUs = 0;
	usingAnyPeripheral = 0;
	devicePowerModeChanged = 0;
	defaultLowPowerMode = 0;
	microsecondsForOneTick = (1000000/configTICK_RATE_HZ);
	lowPowerModeMutex = ytMutexCreate();
	/*Initialize Run and low power modes to platform defaults*/
	platformInitLowPowerManager();
	currentRunModeConsumption = platformUpdateRunModeConsumption();
	currentRunMode = platformGetCurrentRunMode();

	currentLowPowerMode = platformGetCurrentLowPowerMode();
	currentLowPowerModeConsumption = platformUpdateLowPowerModeConsumption();
	defaultLowPowerMode = currentLowPowerMode;

	/*Change the running mode clock to the selected platform default value*/
	ytSetCurrentRunMode(PLATFORM_DEFAULT_INIT_RUN_MODE);

	return RET_OK;
}

/**
 * @brief	Some low power modes may change its current consumption depending on the system frequency
 * @return
 */
void ytUpdateCurrentLowPowerModeConsumption(){
	currentLowPowerModeConsumption = platformUpdateLowPowerModeConsumption();
}

/**
 *
 * @return
 */
uint32_t ytGetCurrentLowPowerMode(){
	return currentLowPowerMode;
}


/**
 *
 * @param newLowPowerMode
 * @return
 */
retval_t ytSetCurrentLowPowerMode(uint32_t newLowPowerMode){
	if(newLowPowerMode < PLATFORM_NUM_LOW_POWER_MODES){
		if(newLowPowerMode != currentLowPowerMode){
			/*Estimate the current consumption till this moment*/
			uint32_t lapsedTime = (osKernelSysTick()*microsecondsForOneTick) - estimationInitUs;
			estimationInitUs = (osKernelSysTick()*microsecondsForOneTick);
			estimationLapsedUs += lapsedTime;
			uint32_t runUs = lapsedTime - estimationAcumulatedLowPowerUs;
			estimatedAcumulatedConsumption += ((runUs*currentRunModeConsumption) +
								(estimationAcumulatedLowPowerUs*currentLowPowerModeConsumption));
			estimationAcumulatedLowPowerUs = 0;
			/*Set the new Low power mode*/
			currentLowPowerMode = newLowPowerMode;
			platformSetCurrentLowPowerMode(currentLowPowerMode);
			currentLowPowerModeConsumption = platformUpdateLowPowerModeConsumption();
		}
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

uint32_t ytGetCurrentLowPowerConsumption(){
	return currentLowPowerModeConsumption;
}

/**
 *
 */
void ytPeripheralBlockLowPower(){
	lowPowerBlockingPeripherals++;
}

/**
 *
 */
void ytPeripheralReleaseLowPower(){
	lowPowerBlockingPeripherals--;
}

/**
 *
 * @return
 */
uint16_t ytIsPeripheralBlockingSleep(){
	return lowPowerBlockingPeripherals;
}


/**
 * @brief			Acumulates the low current consumption for the specified ticks
 * @param ticks
 */
void ytAcumulateLowPowerTicks(uint32_t ticks){
	estimationAcumulatedLowPowerUs += ticks;
}

/**
 * @brief	Resets and start estimating the current consumption
 */
void ytStartEstimatingConsumption(){
	estimationAcumulatedLowPowerUs = 0;
	estimatedAcumulatedConsumption = 0;
	estimationLapsedUs = 0;
	estimationInitUs = osKernelSysTick()*microsecondsForOneTick;
}

/**
 * @brief	Returns the average	 estimated current consumption since the last startEstimatingConsumption call
 * @return
 */
uint32_t ytGetLastEstimatedConsumption(){
	uint32_t lapsedTime = (osKernelSysTick()*microsecondsForOneTick) - estimationInitUs;
	estimationInitUs = osKernelSysTick()*microsecondsForOneTick;
	estimationLapsedUs += lapsedTime;
	uint32_t runTicks = lapsedTime - estimationAcumulatedLowPowerUs;
	estimatedAcumulatedConsumption += ((runTicks*currentRunModeConsumption) +
			(estimationAcumulatedLowPowerUs*currentLowPowerModeConsumption));
	return estimatedAcumulatedConsumption/estimationLapsedUs;
}


/**
 *
 * @return
 */
uint32_t ytGetCurrentRunMode(){
	return currentRunMode;
}


/**
 *
 * @param newRunMode
 * @return
 */
retval_t ytSetCurrentRunMode(uint32_t newRunMode){
	uint32_t timeoutCount = 0;
	if(newRunMode < PLATFORM_NUM_RUN_MODES){
		if(newRunMode != currentRunMode){

			/*Set the new Run mode*/
			/*Suspend the scheduler while changing the clock*/
			taskENTER_CRITICAL();		/*Evaluate usingAnyPeripheral in a critical region to prevent race conditions*/
			while(usingAnyPeripheral){	/*If any peripheral is being used, the clock cannot be changed*/
				taskEXIT_CRITICAL();
				osDelay(1);
				timeoutCount++;
				if(timeoutCount > SET_RUN_MODE_TIMEOUT){
					return RET_ERROR;
				}
				taskENTER_CRITICAL();
			}
			platformSetCurrentRunMode(newRunMode);
			microsecondsForOneTick = (1000000/configTICK_RATE_HZ);
			ytUpdateDevicesCpuFreq();		/*Update the clock config of the loaded peripheral devices*/
			taskEXIT_CRITICAL();		/*Exit the critical region*/


			/*Estimate the current consumption till this moment*/
			uint32_t lapsedTime = (osKernelSysTick()*microsecondsForOneTick) - estimationInitUs;
			estimationInitUs = osKernelSysTick()*microsecondsForOneTick;
			estimationLapsedUs += lapsedTime;
			uint32_t runTicks = lapsedTime - estimationAcumulatedLowPowerUs;
			estimatedAcumulatedConsumption += ((runTicks*currentRunModeConsumption) +
								(estimationAcumulatedLowPowerUs*currentLowPowerModeConsumption));

			estimationAcumulatedLowPowerUs = 0;
			currentRunMode = newRunMode;
			currentRunModeConsumption = platformUpdateRunModeConsumption();
			/*Some low power modes may change its current consumption depending on the system frequency*/
			currentLowPowerModeConsumption = platformUpdateLowPowerModeConsumption();
		}
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

uint32_t ytGetCurrentRunConsumption(){
	return currentRunModeConsumption;
}

/**
 *
 */
void ytPeripheralFreqChangeLock(){
	usingAnyPeripheral++;
}

/**
 *
 */
void ytPeripheralFreqChangeUnlock(){
	usingAnyPeripheral--;
}

/**
 * @param nextMode
 */
void ytSetDeviceLowPowerMode(uint32_t selectedLowPowerMode){
	osMutexWait(lowPowerModeMutex, osWaitForever);
	devicePowerModeChanged++;
	if(selectedLowPowerMode < currentLowPowerMode){	/*A more restrictive mode selected*/
		ytSetCurrentLowPowerMode(selectedLowPowerMode);
	}
	osMutexRelease(lowPowerModeMutex);
}

/**
 *
 */
void ytSetDeviceDefaultLowPowerMode(){
	osMutexWait(lowPowerModeMutex, osWaitForever);
	devicePowerModeChanged--;
	if(!devicePowerModeChanged){
		ytSetCurrentLowPowerMode(defaultLowPowerMode);
	}
	osMutexRelease(lowPowerModeMutex);
}

/**
 *
 * @param newLowPowerMode
 */
void ytUpdateDefaultLowPowerMode(uint32_t newLowPowerMode){
	osMutexWait(lowPowerModeMutex, osWaitForever);
	if(newLowPowerMode != defaultLowPowerMode){
		ytSetCurrentLowPowerMode(newLowPowerMode);
	}
	osMutexRelease(lowPowerModeMutex);
}

/* Weak implementations in case the platform does not implement low power modes*/
__weak void platformInitLowPowerManager(){
	return;
}
__weak uint32_t platformGetCurrentLowPowerMode(){
	return 0;
}
__weak uint32_t platformUpdateLowPowerModeConsumption(){
	return 0;
}
__weak retval_t platformSetCurrentLowPowerMode(uint32_t newLowPowerMode){
	return RET_OK;
}

__weak uint32_t platformGetCurrentRunMode(){
	return 0;
}
__weak uint32_t platformUpdateRunModeConsumption(){
	return 0;
}
__weak retval_t platformSetCurrentRunMode(uint32_t newRunMode){
	return RET_OK;
}
/* Config a device driver to set its state (enabled or disabled) in a specific powerMode */
//retval_t configDeviceLowPowerState(device_t device, lowPowerState_t state, platformLowPowerMode_t lowPowerMode);	//Each device struct has a table with the state of each power mode TODO


