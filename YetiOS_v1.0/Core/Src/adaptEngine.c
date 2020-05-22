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
 * adaptEngine.c
 *
 *  Created on: 9 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file adaptEngine.c
 */

#include "yetiOS.h"
#include "adaptEngine.h"

#if YETIOS_ENABLE_ADAPTIVE_ENGINE

static genList_t* samplingBlocksList;
static genList_t* adaptTasksList;

static osMutexId adaptTaskListMutex;		/*Mutex to protect access to adaptTasks list*/


static ytSamplingBlock_t* getSamplingBlockFromId(uint32_t samplingBlockId);
/*Common CB func for all adaptive tasks*/
static void adaptTaskCbFunc(void const * argument);

/* ************************/

/**
 *
 */
void ytInitAdaptEngine(void){
	adaptTaskListMutex = ytMutexCreate();
	samplingBlocksList = genListInit();
	adaptTasksList = genListInit();
}


/**
 *
 * @param samplingBlockId
 * @param bufferSize
 * @return
 */
ytSamplingBlock_t* ytNewSamplingBlock(uint32_t samplingBlockId, uint32_t bufferSize){
	if((bufferSize == 0) || (samplingBlockId == 0)){
		return NULL;
	}
	ytSamplingBlock_t* samplingBlock = (ytSamplingBlock_t*) pvPortMalloc(sizeof(ytSamplingBlock_t));
	samplingBlock->sampleBuffer = (uint8_t*) pvPortMalloc(bufferSize*2);	/*Double size*/
	samplingBlock->currentBufferPtr = samplingBlock->sampleBuffer;
	samplingBlock->samplingBlockId = samplingBlockId;
	samplingBlock->bufferSize = bufferSize;
	samplingBlock->endBufferPtr = &samplingBlock->sampleBuffer[bufferSize*2];

	return samplingBlock;
}

/**
 *
 * @param samplingBlock
 * @return
 */
retval_t ytDeleteSamplingBlock(ytSamplingBlock_t* samplingBlock){
	if(samplingBlock == NULL){
		return RET_ERROR;
	}
	vPortFree(samplingBlock->sampleBuffer);
	vPortFree(samplingBlock);
	return RET_OK;
}

/**
 *
 * @param samplingBlock
 * @return
 */
retval_t ytRegisterSamplingBlock(ytSamplingBlock_t* samplingBlock){
	if(getSamplingBlockFromId(samplingBlock->samplingBlockId) != NULL) {	/*This samplingBlock is already registered*/
		return RET_ERROR;
	}
	return genListAdd(samplingBlocksList, samplingBlock);
}

/**
 *
 * @param samplingBlock
 * @return
 */
retval_t ytUnregisterSamplingBlock(ytSamplingBlock_t* samplingBlock){
	return genListRemove(samplingBlocksList, samplingBlock);
}

/**
 *
 * @param samplingBlock
 */
void ytUpdateSamplingBlockBuffPtr(ytSamplingBlock_t* samplingBlock){
	if(samplingBlock->currentBufferPtr != samplingBlock->sampleBuffer){
		samplingBlock->currentBufferPtr = samplingBlock->sampleBuffer;
	}
	else{
		samplingBlock->currentBufferPtr+= samplingBlock->bufferSize;
	}
}


/**
 * @brief				This task loops the adaptive tasks listed and release their execution if required
 * @param samplingBlock
 * @return
 */
retval_t ytReleaseAdaptTasks(ytSamplingBlock_t* samplingBlock){
	osMutexWait(adaptTaskListMutex, osWaitForever);
	genListElement_t* current = adaptTasksList->tailElement;

	while(current != NULL){

		ytAdaptTaskBlock_t* adaptTaskElement = (ytAdaptTaskBlock_t*) current->item;

		/*Check if the adaptive task contains this sampling block*/
		if(genListContains(adaptTaskElement->ytSamplingBlocksList, (void*)samplingBlock)){

			/*Release the adaptive task passing the samplingBlock instance that releases it*/
			if(osMessagePut (adaptTaskElement->adaptTaskMessageQ, (uint32_t)samplingBlock, 0) != osOK){
				/*The algorithm queue buffer is full, indicating a possible overrun in this algorithm*/
				adaptTaskElement->overrunFlag++;
			}
		}
		current = current->next;
	}
	osMutexRelease(adaptTaskListMutex);
	return RET_OK;
}

/**
 *
 * @param samplingBlockId
 * @return
 */
static ytSamplingBlock_t* getSamplingBlockFromId(uint32_t samplingBlockId){
	genListElement_t* current = samplingBlocksList->tailElement;
	ytSamplingBlock_t* samplingBlock;
	while(current != NULL){

		samplingBlock = (ytSamplingBlock_t*) current->item;
		if(samplingBlock->samplingBlockId == samplingBlockId){
			return samplingBlock;
		}
		current = current->next;
	}
	return NULL;
}

/*Adaptive Block functions*/
/**
 *
 * @param adaptTaskFunc
 * @param stopAdaptTaskFunc
 * @return
 * @note					Do not creates the message queue until the start function is called
 */
ytAdaptTaskBlock_t* ytNewAdaptTaskBlock(adaptTaskFunc_t adaptTaskFunc, exitAdaptTaskFunc_t exitAdaptTaskFunc){
	ytAdaptTaskBlock_t* adaptTaskBlock = (ytAdaptTaskBlock_t*)pvPortMalloc(sizeof(ytAdaptTaskBlock_t));
	adaptTaskBlock->adaptTaskFunc = adaptTaskFunc;
	adaptTaskBlock->exitAdaptTaskFunc = exitAdaptTaskFunc;
	adaptTaskBlock->ytSamplingBlocksList = genListInit();
	adaptTaskBlock->waitingStopSemph = ytSemaphoreCreate(1);
	osSemaphoreWait(adaptTaskBlock->waitingStopSemph, osWaitForever);
	adaptTaskBlock->adaptTaskRunning = 0;
	adaptTaskBlock->overrunFlag = 0;
	adaptTaskBlock->privateData = NULL;
	return adaptTaskBlock;
}

/**
 *
 * @param ytAdaptTaskBlock
 * @return
 */
retval_t ytDeleteAdaptTaskBlock(ytAdaptTaskBlock_t* adaptTaskBlock){
	if(adaptTaskBlock == NULL){
		return RET_ERROR;
	}
	/*First stop the task*/
	if(adaptTaskBlock->adaptTaskRunning){
		ytStopAdaptTask(adaptTaskBlock);
	}

	/*Free resources*/
	genListRemoveAll(adaptTaskBlock->ytSamplingBlocksList);
	osSemaphoreDelete(adaptTaskBlock->waitingStopSemph);
	vPortFree(adaptTaskBlock);

	return RET_OK;
}

/**
 *
 * @param adaptTaskBlock
 * @param samplingBlockId
 * @return
 */
retval_t ytAdaptTaskAddSamplingBlock(ytAdaptTaskBlock_t* adaptTaskBlock, uint32_t samplingBlockId){
	retval_t ret = RET_ERROR;
	if(adaptTaskBlock == NULL){
		return RET_ERROR;
	}
	osMutexWait(adaptTaskListMutex, osWaitForever);
	if(!adaptTaskBlock->adaptTaskRunning){/*Only allowed adding sampling blocks if the algorithm is not running*/
		if(samplingBlockId){	/*A value of 0 is not allowed*/
			ytSamplingBlock_t* samplingBlock = getSamplingBlockFromId(samplingBlockId);
			if(samplingBlock == NULL){	/*If this sampling block is not found in the system return error*/
				ret = RET_ERROR;
			}
			else{
				ret = genListAdd(adaptTaskBlock->ytSamplingBlocksList, (void*) samplingBlock);
			}
		}
	}
	osMutexRelease(adaptTaskListMutex);
	return ret;
}

/**
 *
 * @param adaptTaskBlock
 * @param samplingBlockId
 * @return
 */
retval_t ytAdaptTaskRemoveSamplingBlock(ytAdaptTaskBlock_t* adaptTaskBlock, uint32_t samplingBlockId){
	retval_t ret = RET_ERROR;
	if(adaptTaskBlock == NULL){
		return RET_ERROR;
	}
	osMutexWait(adaptTaskListMutex, osWaitForever);
	if(!adaptTaskBlock->adaptTaskRunning){/*Only allowed removing sampling blocks if the algorithm is not running*/
		if(samplingBlockId){	/*A value of 0 is not allowed*/
			ytSamplingBlock_t* samplingBlock = getSamplingBlockFromId(samplingBlockId);
			if(samplingBlock == NULL){	/*If this sampling block is not found in the system return error*/
				osMutexRelease(adaptTaskListMutex);
				ret = RET_ERROR;
			}
			else{
				ret = genListRemove(adaptTaskBlock->ytSamplingBlocksList, (void*) samplingBlock);
			}
		}
	}
	osMutexRelease(adaptTaskListMutex);
	return ret;
}

/**
 *
 * @param adaptTaskBlock
 * @return
 */
osThreadId ytStartAdaptTask(ytAdaptTaskBlock_t* adaptTaskBlock, char* name, osPriority threadPriority, uint32_t stackSize){
	if(adaptTaskBlock == NULL){
		return NULL;
	}
	osMutexWait(adaptTaskListMutex, osWaitForever);
	if((!adaptTaskBlock->ytSamplingBlocksList->numElements) || (adaptTaskBlock->adaptTaskRunning)){	/*If there are not sampling blocks associated this adapt task should not run*/
		osMutexRelease(adaptTaskListMutex);
		return NULL;
	}
	adaptTaskBlock->adaptTaskRunning++;
	/*Create the message queue with the same number of slots as associated sampling blocks*/
	adaptTaskBlock->adaptTaskMessageQ = ytMessageqCreate(adaptTaskBlock->ytSamplingBlocksList->numElements);

	/*Add this adaptive task to the system list*/
	genListAdd(adaptTasksList, (void*) adaptTaskBlock);

	/*Execute the thread*/
	osThreadId pThreadHandle;
	ytStartThread(name, adaptTaskCbFunc, threadPriority, stackSize, &pThreadHandle, (void*)adaptTaskBlock);

	osMutexRelease(adaptTaskListMutex);
	return pThreadHandle;
}

/**
 *
 * @param adaptTaskBlock
 * @return
 */
retval_t ytStopAdaptTask(ytAdaptTaskBlock_t* adaptTaskBlock){
	if(adaptTaskBlock == NULL){
		return RET_ERROR;
	}
	osMutexWait(adaptTaskListMutex, osWaitForever);
	if(!adaptTaskBlock->adaptTaskRunning){
		osMutexRelease(adaptTaskListMutex);
		return RET_ERROR;
	}
	adaptTaskBlock->adaptTaskRunning = 0;

	/*Sned the stop message to the task*/
	osMessagePut (adaptTaskBlock->adaptTaskMessageQ, (uint32_t)NULL, osWaitForever);

	/*Wait until the thread is actually stopped*/
	osSemaphoreWait(adaptTaskBlock->waitingStopSemph, osWaitForever);

	/*Delete the queue*/
	ytMessageqDelete(adaptTaskBlock->adaptTaskMessageQ);

	/*Remove this element from the system running list*/
	genListRemove(adaptTasksList, adaptTaskBlock);

	osMutexRelease(adaptTaskListMutex);
	return RET_OK;
}


/**
 *
 * @param argument
 */
static void adaptTaskCbFunc(void const * argument){
	ytAdaptTaskBlock_t* ytAdaptTaskBlock = (ytAdaptTaskBlock_t*) argument;
	ytSamplingBlock_t* currentSampleBlock;
	osEvent ev;

	while(1){

		/*Wait for new sample data to arrive*/
		ev = osMessageGet (ytAdaptTaskBlock->adaptTaskMessageQ, osWaitForever);
		currentSampleBlock = (ytSamplingBlock_t*) ev.value.p;

		if(currentSampleBlock == NULL){	/*The task must finish at this point if received NULL*/
			if(ytAdaptTaskBlock->exitAdaptTaskFunc != NULL){
				ytAdaptTaskBlock->exitAdaptTaskFunc(ytAdaptTaskBlock);
			}
			osSemaphoreRelease(ytAdaptTaskBlock->waitingStopSemph);
			osThreadTerminate(osThreadGetId());
			while(1);/*Should never reach this point*/
		}

		/*Process these new samples*/
		if(ytAdaptTaskBlock->adaptTaskFunc != NULL){
			ytAdaptTaskBlock->adaptTaskFunc(ytAdaptTaskBlock, currentSampleBlock);
		}
	}
}

#endif

/*Example of a sample Thread*/
//void samplingThread_1(){
//	ytSamplingBlock_t* samplingBlock = ytNewSamplingBlock(LFP_ADC_SAMPLE_ID, LFP_ADC_BUFFER_SIZE);
//	ytRegisterSamplingBlock(samplingBlock);
//
//	/*Config Spi*/
//
//	/*Config DRDY interrupt gpio pin*/
//	while(1){
//
//		{/*Do sampling:*/
//
//		/*waitSemaphore(drdySemaphore)*/
//		/*readSpi()*/
//		}
//		/*Update the current buffer pointer*/
//		ytUpdateSamplingBlockBuffPtr(samplingBlock);
//
//		/*Release registered adaptive tasks that use this samplingBlock*/
//		ytReleaseAdaptTasks(samplingBlock);
//
//
//	}
//
//}
//
//
///*Example of an adaptive algorithm*/
//void adaptTask1(){
//	/*Create the adaptive task instance*/
//	ytAdaptTaskBlock_t* adaptTaskBlock = ytNewAdaptTask();
//	/*Add the sampling block required by this adaptive task. If they do not exist return error*/
//	if(ytAdaptTaskAddSamplingBlock(adaptTask, LFP_ADC_SAMPLE_ID) != RET_OK){
//		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
//		while(1);
//	}
//	if(ytAdaptTaskAddSamplingBlock(adaptTask, LFP_ADC_SAMPLE_ID2) != RET_OK){
//		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
//		while(1);
//	}
//	/*Register this adaptive task*/
//	ytRegisterAdaptTask(adaptTask);
//
//	/*Prepare algorithm Buffers*/
//
//	while(1){
//		ytSamplingBlock_t* currentSampleBlock =  ytAdaptTaskWaitData(ytAdaptTask, osWaitForever);
//		/*Process these samples*/
//	}
//}
