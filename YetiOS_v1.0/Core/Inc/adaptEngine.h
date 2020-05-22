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
 * adaptEngine.h
 *
 *  Created on: 9 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file adaptEngine.h
 */

#ifndef YETIOS_CORE_INC_ADAPTENGINE_H_
#define YETIOS_CORE_INC_ADAPTENGINE_H_

#include "yetiOS.h"

#if YETIOS_ENABLE_ADAPTIVE_ENGINE

/*Forward structs declaration*/
struct ytSamplingBlock_;
struct ytAdaptTaskBlock_;

typedef void (*exitAdaptTaskFunc_t)(struct ytAdaptTaskBlock_* adaptTaskBlock);
typedef void (*adaptTaskFunc_t)(struct ytAdaptTaskBlock_* adaptTaskBlock, struct ytSamplingBlock_* currentSamplingBlock);

typedef struct ytSamplingBlock_{
	uint32_t samplingBlockId;		/*Unique ID of a sampling block*/
	uint8_t* sampleBuffer;			/*Buffer to store the samples. It always have double size of bufferSize to allow processing previous samples without overflowing*/
	uint8_t* currentBufferPtr;		/*Pointer to the position of the data currently being stored*/
	uint8_t* endBufferPtr;			/*End position of the sample Buffer*/
	uint32_t bufferSize;			/*Number of samples read in each sampling loop*/
}ytSamplingBlock_t;


typedef struct ytAdaptTaskBlock_{
	genList_t* ytSamplingBlocksList;		/*List containing the sampling blocks references used by this adaptive task block*/
	adaptTaskFunc_t adaptTaskFunc;			/*Function called from the adaptive task to process the incoming samples*/
	exitAdaptTaskFunc_t exitAdaptTaskFunc;	/*Callback function called when an adaptive task block is stopped to allow the user releasing resources*/
	osMessageQId adaptTaskMessageQ;			/*Message queue to schedule execution of this task indicating the sampling block that has scheduled it*/
	osSemaphoreId waitingStopSemph;			/*Semaphore used to wait when the stop function is called*/
	uint16_t adaptTaskRunning;				/*Flag indicating the adaptTask is Running or not*/
	uint16_t overrunFlag;					/*Flag indicating an overrun may have happen in this algorithm. Data arrives faster than it is capable to process it*/
	void* privateData;						/*Specific private data reference that may be used by any specific adaptive task*/
}ytAdaptTaskBlock_t;


/*Adaptive engine initialization function. Must be called by system startup*/
void ytInitAdaptEngine(void);

/*Sampling Block functions**/
ytSamplingBlock_t* ytNewSamplingBlock(uint32_t samplingBlockId, uint32_t bufferSize);
retval_t ytDeleteSamplingBlock(ytSamplingBlock_t* samplingBlock);

retval_t ytRegisterSamplingBlock(ytSamplingBlock_t* samplingBlock);
retval_t ytUnregisterSamplingBlock(ytSamplingBlock_t* samplingBlock);

void ytUpdateSamplingBlockBuffPtr(ytSamplingBlock_t* samplingBlock);

retval_t ytReleaseAdaptTasks(ytSamplingBlock_t* samplingBlock);
/* ************************/

/*Adaptive Block functions*/
ytAdaptTaskBlock_t* ytNewAdaptTaskBlock(adaptTaskFunc_t adaptTaskFunc, exitAdaptTaskFunc_t exitAdaptTaskFunc);
retval_t ytDeleteAdaptTaskBlock(ytAdaptTaskBlock_t* adaptTaskBlock);

retval_t ytAdaptTaskAddSamplingBlock(ytAdaptTaskBlock_t* adaptTaskBlock, uint32_t samplingBlockId);
retval_t ytAdaptTaskRemoveSamplingBlock(ytAdaptTaskBlock_t* adaptTaskBlock, uint32_t samplingBlockId);

osThreadId ytStartAdaptTask(ytAdaptTaskBlock_t* adaptTaskBlock, char* name, osPriority threadPriority, uint32_t stackSize);
retval_t ytStopAdaptTask(ytAdaptTaskBlock_t* adaptTaskBlock);

#endif

#endif /* YETIOS_CORE_INC_ADAPTENGINE_H_ */
