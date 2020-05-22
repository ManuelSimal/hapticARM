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
 * dbsExpDutFilters.c
 *
 *  Created on: 14 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file dbsExpDutFilters.c
 */

#include "yetiOS.h"

#if DBS_EXP_DUT_APP

#include "dbsExpDut.h"

#define MAX_PENDING_SPI_CMD		4

static float32_t* currentFilterCoefficients;
static float32_t* firStateF32 = NULL;
static uint32_t currentBlockSize;
static uint32_t currentNumTaps;
static arm_fir_instance_f32 filterInstance;

static float32_t* testFilterCoefficients;
static float32_t* testFirStateF32 = NULL;
static uint32_t testBlockSize;
static uint32_t testNumTaps;
static arm_fir_instance_f32 testFilterInstance;

static uint16_t pendingSpiCmd = 0;
static osMutexId currentSpiCmdMutex;
static rcvSpiData_t spiCmdBuff[MAX_PENDING_SPI_CMD];
static rcvSpiData_t* lastSentSpiCmdBuffPtr;
static rcvSpiData_t* currentSpiCmdBuffPtr;
static rcvSpiData_t* endSpiCmdBuffPtr;


/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using MATLAB function.
** ------------------------------------------------------------------- */
#define SAMPLE_NUM
#define NUM_TAPS_RATE_1000	64

static float32_t filterCoefficientsRate1000Taps44[NUM_TAPS_RATE_1000] =
{
		-0.118309210385856858f, 0.0415694827087173396f, 0.0271205111861624049f, 0.0117437272770734014f,
		-0.00298016558604680126f, -0.0160130810880310948f, -0.0264903156252368488f, -0.0341788319520782563f,
		-0.0388941670578587667f, -0.0408663720165800826f, -0.0403572144310992975f, -0.0378655526399434839f,
		-0.033826724562643129f, -0.0287420919215010118f, -0.0229308787429059191f, -0.0167874499609057259f,
		-0.0104535126082037633f, -0.0043097567503065648f, 0.00186409371275930713f, 0.00756019110270297889f,
		0.0133281012530918341f, 0.0186521266100724559f, 0.0234894836379380134f, 0.0284014132559658738f,
		0.0328328791249762747f, 0.0369186268822459668f, 0.040568964254506415f, 0.0438349657428257627f,
		0.0466276460200138465f, 0.0487505238521272477f, 0.0501553834121984538f, 0.0508301368376691404f,
		0.0508301368376691404f, 0.0501553834121984538f, 0.0487505238521272477f, 0.0466276460200138465f,
		0.0438349657428257627f, 0.040568964254506415f, 0.0369186268822459668f, 0.0328328791249762747f,
		0.0284014132559658738f, 0.0234894836379380134f, 0.0186521266100724559f, 0.0133281012530918341f,
		0.00756019110270297889f, 0.00186409371275930713f, -0.0043097567503065648f, -0.0104535126082037633f,
		-0.0167874499609057259f, -0.0229308787429059191f, -0.0287420919215010118f, -0.033826724562643129f,
		-0.0378655526399434839f, -0.0403572144310992975f, -0.0408663720165800826f, -0.0388941670578587667f,
		-0.0341788319520782563f, -0.0264903156252368488f, -0.0160130810880310948f, -0.00298016558604680126f,
		0.0117437272770734014f, 0.0271205111861624049f, 0.0415694827087173396f, -0.118309210385856858f
};

#define NUM_TAPS_RATE_500	28

//static float32_t filterCoefficientsRate500Taps26[NUM_TAPS_RATE_500] =
//{
//		-0.175457084364402244f, -0.0481066329231248646f, -0.0478962625770383843f, -0.0425466778005980373f,
//		-0.0318418073715156513f, -0.0162522905400347417f, 0.00312562531032181679f, 0.0249222001750168676f,
//		0.0471807132482820746f, 0.0681601753876631411f, 0.0858137851238242044f, 0.0989890346432161505f,
//		0.10533998640765016f, 0.10533998640765016f, 0.0989890346432161505f, 0.0858137851238242044f,
//		0.0681601753876631411f, 0.0471807132482820746f, 0.0249222001750168676f, 0.00312562531032181679f,
//		-0.0162522905400347417f, -0.0318418073715156513f, -0.0425466778005980373f, -0.0478962625770383843f,
//		-0.0481066329231248646f, -0.175457084364402244f
//
//};

static float32_t filterCoefficientsRate500Taps20[NUM_TAPS_RATE_500] =
{
		-0.157819403113556356f, -0.0261947280582899844f, -0.030682628071137974f, -0.0352988978418564137f,
		-0.0380286343958836925f, -0.0363323080505791318f, -0.0284393999079314008f, -0.0132796403000211838f,
		0.00849651515986962758f, 0.0351108235176906813f, 0.0631932034046326502f, 0.0891835465008781608f,
		0.108977825487056673f, 0.119649833712123038f, 0.119649833712123038f, 0.108977825487056673f,
		0.0891835465008781608f, 0.0631932034046326502f, 0.0351108235176906813f, 0.00849651515986962758f,
		-0.0132796403000211838f, -0.0284393999079314008f, -0.0363323080505791318f, -0.0380286343958836925f,
		-0.0352988978418564137f, -0.030682628071137974f, -0.0261947280582899844f, -0.157819403113556356f
};


#define NUM_TAPS_RATE_250	18

static float32_t filterCoefficientsRate250Taps14[NUM_TAPS_RATE_250] =
{
		-0.0548820830451645855f, 0.0423184423209735719f, -0.126866436336125765f, -0.145931670641473887f,
		-0.0894084459621177635f, -0.00920355328747179957f, 0.0764721528012179824f, 0.15429719723019425f,
		0.202011938284761911f, 0.202011938284761911f, 0.15429719723019425f, 0.0764721528012179824f,
		-0.00920355328747179957f, -0.0894084459621177635f, -0.145931670641473887f, -0.126866436336125765f,
		0.0423184423209735719f, -0.0548820830451645855f
};

#define NUM_TAPS_RATE_100	12

static float32_t filterCoefficientsRate100Taps10[NUM_TAPS_RATE_100] =
{
		-0.0517169702371483922f, -0.0397932445976323032f, -0.00403207188179304822f, -0.284718340243033829f,
		-0.122131839789903412f, 0.408768430150923012f, 0.408768430150923012f, -0.122131839789903412f,
		-0.284718340243033829f, -0.00403207188179304822f, -0.0397932445976323032f, -0.0517169702371483922f
};


/**
 *
 * @param lfpFilterRate
 * @param sampleNum
 * @return
 */
retval_t setSampleRate(lfpFilterRate_t lfpFilterRate, uint32_t* sampleNum, uint32_t* numSamplesEnergyWindow){

	switch(lfpFilterRate){
	case FILTER_RATE_1000:
		currentFilterCoefficients = filterCoefficientsRate1000Taps44;
		currentNumTaps = NUM_TAPS_RATE_1000;
		(*sampleNum) = SAMPLE_NUM_RATE_1000;
		(*numSamplesEnergyWindow) = ENERGY_SAMPLES_1000_RATE;
		prepareSendCmd(CMD_SET_SAMPLE_RATE, 100);
		#if BUFFERED_READ_MODE
		setCurrentRunMode(RUN_MODE_24MHZ);	/*24 MHz*/
		#else
		setCurrentRunMode(RUN_MODE_8MHZ);	/*8 MHz*/
		#endif
		break;
	case FILTER_RATE_500:
		currentFilterCoefficients = filterCoefficientsRate500Taps20;
		currentNumTaps = NUM_TAPS_RATE_500;
		(*sampleNum) = SAMPLE_NUM_RATE_500;
		(*numSamplesEnergyWindow) = ENERGY_SAMPLES_500_RATE;
		prepareSendCmd(CMD_SET_SAMPLE_RATE, 50);
		#if BUFFERED_READ_MODE
		setCurrentRunMode(RUN_MODE_24MHZ);	/*24 MHz*/
		#else
		setCurrentRunMode(RUN_MODE_8MHZ);	/*8 MHz*/
		#endif
		break;
	case FILTER_RATE_250:
		currentFilterCoefficients = filterCoefficientsRate250Taps14;
		currentNumTaps = NUM_TAPS_RATE_250;
		(*sampleNum) = SAMPLE_NUM_RATE_250;
		(*numSamplesEnergyWindow) = ENERGY_SAMPLES_250_RATE;
		prepareSendCmd(CMD_SET_SAMPLE_RATE, 25);
		#if BUFFERED_READ_MODE
		setCurrentRunMode(RUN_MODE_16MHZ);	/*16 MHz*/
		#else
		setCurrentRunMode(RUN_MODE_2MHZ);	/*2 MHz*/
		#endif
		break;
	case FILTER_RATE_100:
		currentFilterCoefficients = filterCoefficientsRate100Taps10;
		currentNumTaps = NUM_TAPS_RATE_100;
		(*sampleNum) = SAMPLE_NUM_RATE_100;
		(*numSamplesEnergyWindow) = ENERGY_SAMPLES_100_RATE;
		prepareSendCmd(CMD_SET_SAMPLE_RATE, 10);
		#if BUFFERED_READ_MODE
		setCurrentRunMode(RUN_MODE_16MHZ);	/*16 MHz*/
		#else
		setCurrentRunMode(RUN_MODE_4MHZ);	/*4 MHz*/
		#endif
		break;
	default:
		(*numSamplesEnergyWindow) = 0;
		(*sampleNum) = 0;
		return RET_ERROR;
		break;
	}
	currentBlockSize = (*sampleNum);
	if(firStateF32 == NULL){
		firStateF32 = (float32_t*) pvPortMalloc((currentBlockSize + currentNumTaps - 1)*sizeof(float32_t));
	}
	else{
		vPortFree(firStateF32);
		firStateF32 = (float32_t*) pvPortMalloc((currentBlockSize + currentNumTaps - 1)*sizeof(float32_t));
	}
	arm_fir_init_f32(&filterInstance, currentNumTaps, (float32_t *)currentFilterCoefficients, firStateF32, currentBlockSize);


	return RET_OK;
}

/**
 *
 * @param inputSignal
 * @param outputSignal
 * @return
 */
retval_t runLfpFilter(float32_t* inputSignal, float32_t* outputSignal){
	if(firStateF32 == NULL){
		return RET_ERROR;
	}
	arm_fir_f32(&filterInstance, (float*) inputSignal, (float*) outputSignal, currentBlockSize);
	return RET_OK;
}


/**
 *
 * @param lfpFilterRate
 * @return
 */
retval_t setTestSampleRate(lfpFilterRate_t lfpFilterRate, uint32_t* testSampleNum, uint32_t* testNumSamplesEnergyWindow){
	switch(lfpFilterRate){
		case FILTER_RATE_1000:
			testFilterCoefficients = filterCoefficientsRate1000Taps44;
			testNumTaps = NUM_TAPS_RATE_1000;
			testBlockSize = SAMPLE_NUM_RATE_1000;
			(*testNumSamplesEnergyWindow) = ENERGY_SAMPLES_1000_RATE;
			break;
		case FILTER_RATE_500:
			testFilterCoefficients = filterCoefficientsRate500Taps20;
			testNumTaps = NUM_TAPS_RATE_500;
			testBlockSize = SAMPLE_NUM_RATE_500;
			(*testNumSamplesEnergyWindow) = ENERGY_SAMPLES_500_RATE;
			break;
		case FILTER_RATE_250:
			testFilterCoefficients = filterCoefficientsRate250Taps14;
			testNumTaps = NUM_TAPS_RATE_250;
			testBlockSize = SAMPLE_NUM_RATE_250;
			(*testNumSamplesEnergyWindow) = ENERGY_SAMPLES_250_RATE;
			break;
		case FILTER_RATE_100:
			testFilterCoefficients = filterCoefficientsRate100Taps10;
			testNumTaps = NUM_TAPS_RATE_100;
			testBlockSize = SAMPLE_NUM_TEST_RATE_100;
			(*testNumSamplesEnergyWindow) = ENERGY_SAMPLES_100_RATE;
			break;
		default:
			return RET_ERROR;
			break;
	}
	(*testSampleNum) = testBlockSize;
	if(testFirStateF32 == NULL){
		testFirStateF32 = (float32_t*) pvPortMalloc((testBlockSize + testNumTaps - 1)*sizeof(float32_t));
	}
	else{
		vPortFree(testFirStateF32);
		testFirStateF32 = (float32_t*) pvPortMalloc((testBlockSize + testNumTaps - 1)*sizeof(float32_t));
	}
	arm_fir_init_f32(&testFilterInstance, testNumTaps, (float32_t *)testFilterCoefficients, testFirStateF32, testBlockSize);
	return RET_OK;
}

/**
 *
 * @param inputSignal
 * @param outputSignal
 * @return
 */
retval_t runTestLfpFilter(float32_t* inputSignal, float32_t* outputSignal){
	if(testFirStateF32 == NULL){
		return RET_ERROR;
	}
	arm_fir_f32(&testFilterInstance, (float*) inputSignal, (float*) outputSignal, testBlockSize);
	return RET_OK;
}



/**
 *
 * @param dstSpiCmd
 */
void spiCmdInit(rcvSpiData_t* dstSpiCmd){
	memset(spiCmdBuff, 0, MAX_PENDING_SPI_CMD*sizeof(rcvSpiData_t));
	currentSpiCmdBuffPtr = spiCmdBuff;
	lastSentSpiCmdBuffPtr = spiCmdBuff;
	endSpiCmdBuffPtr = &spiCmdBuff[MAX_PENDING_SPI_CMD];

	memcpy(dstSpiCmd, currentSpiCmdBuffPtr, sizeof(rcvSpiData_t));
	currentSpiCmdMutex = ytMutexCreate();
}

/**
 *
 * @param dstSpiCmd
 * @return
 */
retval_t updateSpiCmd(rcvSpiData_t* dstSpiCmd){
	osMutexWait(currentSpiCmdMutex, osWaitForever);
	if(pendingSpiCmd){
		memcpy(dstSpiCmd, lastSentSpiCmdBuffPtr, sizeof(rcvSpiData_t));
		lastSentSpiCmdBuffPtr++;
		if(lastSentSpiCmdBuffPtr >= endSpiCmdBuffPtr){
			lastSentSpiCmdBuffPtr = spiCmdBuff;
		}
		pendingSpiCmd--;
	}
	else{
		dstSpiCmd->rcvCmd = NO_CMD;
		dstSpiCmd->rcvValue = 0;
	}
	osMutexRelease(currentSpiCmdMutex);
	return RET_OK;

}


/**
 *
 * @param cmd
 * @param value
 * @return
 */
retval_t prepareSendCmd(uint8_t cmd, uint8_t value){
	if (cmd > CMD_SET_STIM_PULSE){
		return RET_ERROR;
	}
	osMutexWait(currentSpiCmdMutex, osWaitForever);
	if(pendingSpiCmd >= MAX_PENDING_SPI_CMD){
		osMutexRelease(currentSpiCmdMutex);		/*Only MAX_PENDING_SPI_CMD commands can be prepared to be sent at each time*/
		return RET_ERROR;
	}
	(*currentSpiCmdBuffPtr).rcvCmd = cmd;
	(*currentSpiCmdBuffPtr).rcvValue = value;
	currentSpiCmdBuffPtr++;
	if(currentSpiCmdBuffPtr >= endSpiCmdBuffPtr){
		currentSpiCmdBuffPtr = spiCmdBuff;
	}
	pendingSpiCmd++;
	osMutexRelease(currentSpiCmdMutex);

	return RET_OK;
}
#endif
