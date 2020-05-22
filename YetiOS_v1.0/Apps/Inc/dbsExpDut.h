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
 * dbsExpDut.h
 *
 *  Created on: 15 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file dbsExpDut.h
 */

#ifndef YETIOS_APPS_INC_DBSEXPDUT_H_
#define YETIOS_APPS_INC_DBSEXPDUT_H_

#include "yetiOS.h"

#if DBS_EXP_DUT_APP


#define BUFFERED_READ_MODE			0


#define SAMPLE_NUM_RATE_1000		256
#define SAMPLE_NUM_RATE_500			128
#define SAMPLE_NUM_RATE_250			64
#define SAMPLE_NUM_RATE_100			32
#define SAMPLE_NUM_TEST_RATE_100	25		/* 256/100 */

#define MAX_SAMPLING_BLOCK_ACQ_SAMPLES	256	/*The maximum of the above sample nums. The sampling block will allocate
 	 	 	 	 	 	 	 	 	 	 	  the double of this sample num to acquire samples without overruning*/


#define ENERGY_SAMPLES_1000_RATE		1024
#define ENERGY_SAMPLES_500_RATE			512
#define ENERGY_SAMPLES_250_RATE			256
#define ENERGY_SAMPLES_100_RATE			128


#define NO_CMD 							0
#define	CMD_SET_BUFFERED_READ_SAMPLES	1
#define	CMD_SET_SAMPLE_RATE				2
#define CMD_SET_STIM_AMP				3
#define CMD_SET_STIM_FREQ				4
#define CMD_SET_STIM_PULSE				5

typedef struct __packed rcvSpiData_{
	uint8_t rcvCmd;
	uint8_t rcvValue;
}rcvSpiData_t;

typedef enum lfpFilterRate_{
	FILTER_RATE_1000 = 0,
	FILTER_RATE_500 = 1,
	FILTER_RATE_250 = 2,
	FILTER_RATE_100 = 3,
}lfpFilterRate_t;


retval_t setSampleRate(lfpFilterRate_t lfpFilterRate, uint32_t* sampleNum, uint32_t* numSamplesEnergyWindow);
retval_t runLfpFilter(float32_t* inputSignal, float32_t* outputSignal);

retval_t setTestSampleRate(lfpFilterRate_t lfpFilterRate, uint32_t* testSampleNum, uint32_t* testNumSamplesEnergyWindow);
retval_t runTestLfpFilter(float32_t* inputSignal, float32_t* outputSignal);

void spiCmdInit(rcvSpiData_t* dstSpiCmd);
retval_t updateSpiCmd(rcvSpiData_t* dstSpiCmd);
retval_t prepareSendCmd(uint8_t cmd, uint8_t value);

#endif

#endif /* YETIOS_APPS_INC_DBSEXPDUT_H_ */
