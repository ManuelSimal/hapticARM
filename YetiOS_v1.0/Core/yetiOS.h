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
 * yetiOs.h
 *
 *  Created on: 13 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file yetiOs.h
 * @brief	YetiOS Api Header File. Must be imported by all the apps that uses YetiOS
 */

#ifndef YETIOS_CORE_YETIOS_H_
#define YETIOS_CORE_YETIOS_H_

#include "types.h"
#include "platformConf.h"
#include "yetiOSConf.h"
#include "process.h"
#include "leds.h"
#include "deviceDriver.h"
#include "gpio.h"
#include "spi.h"
#include "ytStdio.h"
#include "ytShell.h"
#include "timeMeas.h"

/*CMSIS OS API IS INHERITED BY YETIOS */
#include "cmsis_os.h"


typedef void (*ytThreadFunc_t)(void const * argument);


/**
 * @brief 					MACRO to create a proccess at startup
 * @param name				Process name. For memory saving avoid large names. (char*)
 * @param threadFunc		Function to be launched when the process starts. (ytProcessFunc_t) => void (*)(const void* args)
 * @param threadPriority	Process Priority.
 * @param stackSize			Amount of stack to reserve for this process. (uint32_t)
 * @param processHandle		Returned value of the process handle created. Use a pointer to NULL if is not desired to return the id.
 * @param arg				Argument to the process function. (void*)
 */
#define YtAutoInitThread(name, threadFunc, threadPriority, stackSize, threadHandle, arg) INIT_THREAD(name, threadFunc, threadPriority, stackSize, threadHandle, arg)

/*Create and Start a new thread*/
retval_t ytStartThread(char* name, ytThreadFunc_t func, osPriority threadPriority, uint32_t stackSize,
		osThreadId* pThreadHandle, void* arg);

/*Create a new timer instance. Returned osTimerId compatible with cmsis_os API */
osTimerId ytTimerCreate(os_timer_type timerType, ytThreadFunc_t timerFunc, void *argument);

/*Create a new mutex instance. Returned osMutexId compatible with cmsis_os API */
osMutexId ytMutexCreate(void);

/*Create a new semaphore instance. Returned osSemaphoreId compatible with cmsis_os API */
osSemaphoreId ytSemaphoreCreate(int32_t count);

/*Create a new message queue instance. Returned osMessageQId compatible with cmsis_os API */
osMessageQId ytMessageqCreate(uint32_t q_size);
/*For any reason there is not a function in cmsis_os.h to release resources of a message queue, so it is implemented with this function*/
void ytMessageqDelete(osMessageQId messageqId);

/*Turn ON an specified LED*/
INLINE_FUNC retval_t ytLedOn(ytLed_t led);

/*Turn OFF an specified LED*/
INLINE_FUNC retval_t ytLedOff(ytLed_t led);

/*Toggle an specified LED*/
INLINE_FUNC retval_t ytLedToggle(ytLed_t led);

/*Device driver functions*/
deviceFileHandler_t* ytOpen(uint32_t deviceId, uint32_t flags);
retval_t ytClose(deviceFileHandler_t* devFile);
uint32_t ytRead(deviceFileHandler_t* devFile, uint8_t* buf, uint32_t bytes);
uint32_t ytWrite(deviceFileHandler_t* devFile, uint8_t* buf, uint32_t bytes);
retval_t ytIoctl(deviceFileHandler_t* devFile, ytIoctlCmd_t command, void* args);

/*GPIO Functions*/
retval_t ytGpioInitPin(uint32_t gpio, ytGpioMode_t gpioMode, ytGpioPull_t gpioPull);
retval_t ytGpioDeInitPin(uint32_t gpio);
retval_t ytGpioPinSet(uint32_t gpio);
retval_t ytGpioPinReset(uint32_t gpio);
retval_t ytGpioPinToggle(uint32_t gpio);
uint16_t ytGpioPinGet(uint32_t gpio);
retval_t ytGpioPinSetCallback(uint32_t gpio, ytThreadFunc_t gpioInterruptCallbackFunc, void* args);

/*STDIO Functions*/
/*Stdio Output Functions*/
void ytPrintf(char* format, ...);
retval_t ytStdoutSend(uint8_t* data, uint32_t size);

/*Stdio Input functions*/
retval_t ytStdioRead(uint8_t* data, uint32_t size);
retval_t ytStdioReadLine(uint8_t* data, uint32_t max_size);

/*Functions to enable or disable echo output when using stdio read Line*/
void ytEnableStdioReadLineEcho(void);
void ytDisableStdioReadLineEcho(void);

/*Register and Unregister yetishell commands*/
retval_t ytShellRegisterCommand(char* commandName, ytShellFunc_t ytShellFunc);
retval_t ytShellUnregisterCommand(char* commandName);

/*Time measurement functions*/
retval_t ytStartTimeMeasure(timeMeas_t* timeMeas);
retval_t ytStopTimeMeasure(timeMeas_t* timeMeas);

/*Timestamp functions*/
uint64_t ytGetTimestamp(void);
retval_t ytSetTimestamp(uint64_t newTimestamp);

/*Unique Id Functions*/
#if PLATFORM_USE_16_BITS_UNIQUE_ID
uint16_t ytGetUniqueId16Bits(void);
#endif
#if PLATFORM_USE_32_BITS_UNIQUE_ID
uint32_t ytGetUniqueId32Bits(void);
#endif
#if PLATFORM_USE_64_BITS_UNIQUE_ID
uint64_t ytGetUniqueId64Bits(void);
#endif
#if PLATFORM_USE_96_BITS_UNIQUE_ID
uint32_t* ytGetUniqueId96Bits(void);
#endif



#endif /* YETIOS_CORE_YETIOS_H_ */
