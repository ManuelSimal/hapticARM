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
 * process.c
 *
 *  Created on: 15 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file process.c
 */

#include "yetiOS.h"
#include "process.h"



typedef struct ytThreadFuncArgs_{	//Estructura que guarda los argumentos necesarios para ejecutar un proceso.
	ytThreadFunc_t function;
	void* arg;
}ytThreadFuncArgs_t;

static void initializeThreadFunc(void const * argument);


/**
 * @brief	Init all User Processes on startup.
 * @return	Return status
 */
retval_t ytAutoInitThreads(void){

	extern uint32_t (*_process_init_start)(void); // Defined by the linker
	extern uint32_t (*_process_init_end)(void); // Defined by the linker

	uint32_t* start = (uint32_t*) &_process_init_start;
	uint32_t* end = (uint32_t*) &_process_init_end;

	while(((*start) != (uint32_t)NULL) && (start < end)){
		retval_t (*func)(void)=(retval_t(*)(void))(*start);
		if(func() != RET_OK){
#ifdef ERROR_LED
			leds_on(ERROR_LED);	// If there is any problem initializing a process, block the system
#endif
			osThreadSuspendAll();
			while(1);
		}
		start++;
	}

	return RET_OK;
}


/**
 * @brief					Create a new thread and starts it.
 * @param name				Thread name. For memory saving avoid large names
 * @param func				Function to be launched when the thread starts
 * @param processPriority	Priority of the Thread
 * @param stackSize			Amount of stack to reserve for this thread
 * @param processHandle		Returned value of the thread handle. Use a pointer to NULL if is not desired to return the id
 * @param arg				Argument to the thread function
 * @return					Return status
 */
retval_t ytStartThread(char* name, ytThreadFunc_t func, osPriority threadPriority, uint32_t stackSize,
		osThreadId* pThreadHandle, void* arg){


	if((func == NULL) || (stackSize < configMINIMAL_STACK_SIZE)){
		return RET_ERROR;
	}


	ytThreadFuncArgs_t* funcArgs = (ytThreadFuncArgs_t*) pvPortMalloc(sizeof(ytThreadFuncArgs_t));
	funcArgs->function = func;
	funcArgs->arg = arg;

	const osThreadDef_t os_thread_def = 	{ name, initializeThreadFunc, threadPriority, 0, stackSize};

	if (pThreadHandle == NULL){
		if (osThreadCreate(&os_thread_def, (void*) funcArgs) == NULL){
			return RET_ERROR;
		}
	}
	else{
		if (((*pThreadHandle) = osThreadCreate(&os_thread_def, (void*) funcArgs)) == NULL){
			return RET_ERROR;
		}
	}


	return RET_OK;
}


/**
 *
 * @param timerType
 * @param timerFunc
 * @param argument
 * @return
 */
osTimerId ytTimerCreate(os_timer_type timerType, ytThreadFunc_t timerFunc, void *argument){

	if(timerFunc == NULL){
		return NULL;
	}

	osTimerDef(ytTimer, timerFunc);
	return osTimerCreate(osTimer(ytTimer), timerType, argument);

}

/**
 *
 * @return
 */
osMutexId ytMutexCreate(void){
	osMutexDef(ytMutex);
	return osMutexCreate(osMutex(ytMutex));
}


/**
 *
 * @param count
 * @return
 */
osSemaphoreId ytSemaphoreCreate(int32_t count){
	osSemaphoreDef(ytSemaphore);
	return osSemaphoreCreate(osSemaphore(ytSemaphore), count);
}

/**
 *
 * @param q_size
 * @return
 */
osMessageQId ytMessageqCreate(uint32_t q_size){

	osMessageQDef(yt_messageq, q_size, uint32_t);
	return  osMessageCreate(osMessageQ(yt_messageq), NULL);
}

/**
 *
 * @param messageqId
 */
void ytMessageqDelete(osMessageQId messageqId){
	vQueueDelete( ( QueueHandle_t ) ( messageqId ) );
}

/**
 * @brief			Initialization function called when every process is started
 * @param argument	Argument to the callback function defined by the user in startProcess func
 */
static void initializeThreadFunc(void const * argument){
	ytThreadFuncArgs_t* funcArgs = (ytThreadFuncArgs_t*) argument;

	ytThreadFunc_t function = funcArgs->function;
	void* arg = funcArgs->arg;
	vPortFree(funcArgs);

	if(function != NULL){
		function(arg);	/*Call the process Function */
	}

	/*At this point the process have reach its end by exiting the function. If not, terminate the process*/

	osThreadTerminate(osThreadGetId());	/*Terminate this process adequately */

	while(1);	/*The process should be terminated before reaching this point*/
}


