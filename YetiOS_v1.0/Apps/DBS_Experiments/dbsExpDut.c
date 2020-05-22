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
 * dbsExpDut.c
 *
 *  Created on: 8 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file dbsExpDut.c
 */


#include "yetiOS.h"

#if DBS_EXP_DUT_APP

#include "platformGpio.h"
#include "adaptEngine.h"
#include "dbsExpDut.h"

#define LFP_SPI_ADC_SAMPLING_BLOCK_ID		1


#define USE_NIGHT_MODE						0
#define USE_DYNAMIC_SAMPLE_RATE				0

#if BUFFERED_READ_MODE
#define BUFFERED_READ_NUM_SAMPLES			32
#else
#define BUFFERED_READ_NUM_SAMPLES			1
#endif

#define DEFAULT_SAMPLE_RATE					FILTER_RATE_500

#if !YETIOS_ENABLE_ADAPTIVE_ENGINE
#error	"The adaptive engine must be enabled to use this application"
#endif

/* Double threshold parameters*/
#define DOUBLE_THR_TOP					0.008f
#define DOUBLE_THR_BOTTOM				0.003f

#define MAX_READ_SAMPLES				256

#define MAX_ENERGY_WINDOW_SAMPLES 		ENERGY_SAMPLES_1000_RATE
#define MIN_ENERGY_WINDOW_SAMPLES 		ENERGY_SAMPLES_100_RATE

#define SAMPLE_RATE_CHECK_PERIOD		95000

#define SAMPLE_RATE_TEST_TIME			5000
#define	MAX_TH_ERRORS					5
#define	MIN_TH_ERRORS					2

#define NUM_DISCARDED_BUFFER_COUNT		4

#define NIGHT_NO_READ_DATA_TIME			95000
#define NIGHT_READ_DATA_TIME			5000

typedef enum stimState_{
	STIM_STATE_S0,
	STIM_STATE_S1,
	STIM_STATE_S2,
}stimState_t;

//static uint16_t acqOn = 1;
static stimState_t currentStimState;
static uint16_t nightMode = USE_NIGHT_MODE;

static float32_t firInputSamples[MAX_SAMPLING_BLOCK_ACQ_SAMPLES];
static float32_t energyWindowSamples[MAX_ENERGY_WINDOW_SAMPLES];
static float32_t* energyWindowSamplesPtr;
static float32_t* energyWindowSamplesEndPtr;
static uint32_t numSamplesEnergyWindow;
static uint32_t currentBufferCount;

static float32_t testFirInputBuffer[MAX_SAMPLING_BLOCK_ACQ_SAMPLES];
static float32_t testEnergyWindowSamples[MAX_ENERGY_WINDOW_SAMPLES];
static float32_t* testEnergyWindowSamplesPtr;
static float32_t* testEnergyWindowSamplesEndPtr;
static uint32_t numSamplesTestEnergyWindow;
static uint32_t currentTestBufferCount;

static uint32_t defaultSampleRate = 100;
static lfpFilterRate_t defaultLfpFilterRate = DEFAULT_SAMPLE_RATE;
static uint16_t testSampleRateMode = 0;
static uint32_t testErrorCount = 0;

static uint16_t changeSampleRate = 0;
static lfpFilterRate_t newSampleRate;

static uint32_t doubleThresholdFactor;
static uint32_t testDoubleThresholdFactor;

static uint32_t energyOutputFactor;

#define MAX_STIM_AMP	100

/*Double threshold variables*/
static uint8_t currentStimAmp = 0;

/*Tx Buffer used for read write operations. Its first two bytes are the SPI command of type rcvSpiData_t*/
static uint8_t* txBuffer;

static uint32_t numSamplesProc;
static uint32_t testSamplesProc;

/*Spi Device*/
static deviceFileHandler_t* spiDevice;

/*Threads handle declaration*/
static osThreadId dbsExpDutThreadHandle;
static osThreadId pwrEstThreadHandle;
static osThreadId dynRateThreadHandle;

/*Semaphore used to indicate new data is ready*/
static osSemaphoreId drdySemaphore;
static uint32_t interruptPending = 0;	/*Indicates the number of data ready interrupts pending. A value larger than one may indicate an overrun*/

/*Thread Functiosn declaration*/
static void dbsExpDutFunc(void const * argument);
static void pwrEstFunc(void const * argument);
static void dynamicRateFunc(void const * argument);
/*Config Spi Function declaration*/
static retval_t configSpiDevice(deviceFileHandler_t* spiDevice);
/*Data ready pin callback function*/
static void dataReadyCallback(void const * argument);
/*Used to dynamically change sampleRate*/
static void updateSampleRate(lfpFilterRate_t lfpFilterRate);
/*PID adaptive tasks*/
static void exitAdaptTaskFunc(ytAdaptTaskBlock_t* adaptTaskBlock);
static void adaptTaskFunc(ytAdaptTaskBlock_t* adaptTaskBlock, ytSamplingBlock_t* currentSamplingBlock);
/*On Off algorithm function*/
//static void onOffAlg(float32_t energy);
/*Double threshold algorithm function*/
static void doubleThresholdAlg(float32_t energy);
/*PID algorithm function*/
//static void pidAlg(float32_t energy);
//static arm_pid_instance_f32 pidInstance;
static void testSampleRate(float32_t* samplePtr);
static retval_t increaseDefaultSamplerate(void);
static retval_t reduceDefaultSamplerate(void);
/*Autostart Thread*/
YtAutoInitThread(dbsExpDut, dbsExpDutFunc, osPriorityNormal, 180, &dbsExpDutThreadHandle, NULL);
YtAutoInitThread(pwrEst, pwrEstFunc, osPriorityLow, 180, &pwrEstThreadHandle, NULL);
YtAutoInitThread(dynRate, dynamicRateFunc, osPriorityLow, 180, &dynRateThreadHandle, NULL);

//static float32_t* firInputSamples;


static void pwrEstFunc(void const * argument){
	volatile uint32_t estimation = 0;	/*Volatile to debug with optimizations*/
	while(1){
		startEstimatingConsumption();
		osDelay(20000);
		estimation = getLastEstimatedConsumption();
//		updateSampleRate(FILTER_RATE_100);
//		osDelay(20000);
//		updateSampleRate(FILTER_RATE_1000);

	}
}

static uint32_t samplingOverflowFlag = 0;
static osMutexId sampleRateMutex;
/**
 * @brief			USB read thread function
 * @param argument
 */
static void dbsExpDutFunc(void const * argument){
	ytSamplingBlock_t* samplingBlock;
	ytAdaptTaskBlock_t* clDbsAdaptTask;
	uint32_t samplingBlockReadSamples = 0;

	/* *************Initialization*******************/
	/*Initialize data ready semaphore*/
	drdySemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(drdySemaphore, osWaitForever);

	/*Initialize change sample rate mutex*/
	sampleRateMutex = ytMutexCreate();

	/*Initialize the txBuffer and the command sent to NO Command*/
	txBuffer = (uint8_t*) pvPortMalloc(BUFFERED_READ_NUM_SAMPLES*sizeof(uint16_t));
	memset(txBuffer, 0 , BUFFERED_READ_NUM_SAMPLES*sizeof(uint16_t));
	spiCmdInit((rcvSpiData_t*)txBuffer);



	/*Initialize FIR filter structures*/
	if(setSampleRate(DEFAULT_SAMPLE_RATE, &numSamplesProc, &numSamplesEnergyWindow) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	doubleThresholdFactor = numSamplesEnergyWindow/MIN_ENERGY_WINDOW_SAMPLES;
	energyOutputFactor = MAX_ENERGY_WINDOW_SAMPLES/numSamplesEnergyWindow;

	energyWindowSamplesPtr = energyWindowSamples;
	energyWindowSamplesEndPtr = &energyWindowSamples[MAX_ENERGY_WINDOW_SAMPLES];
	currentBufferCount = 0;

	testEnergyWindowSamplesPtr = testEnergyWindowSamples;
	testEnergyWindowSamplesEndPtr = &testEnergyWindowSamples[MAX_ENERGY_WINDOW_SAMPLES];
	currentTestBufferCount = 0;

	/*PID intance initialization*/
//	pidInstance.Kd = 1;
//	pidInstance.Ki = 0.5f;
//	pidInstance.Kp = 1;
//	arm_pid_init_f32(&pidInstance, 1);

	/*Open Spi Device to obtain the samples*/
	if((spiDevice = ytOpen(PLATFORM_SPI2_DEVICE_ID, 0)) == NULL){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/*Configure the Spi device*/
	if(configSpiDevice(spiDevice) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/*Create and configure the sampling block*/
	if((samplingBlock = ytNewSamplingBlock(LFP_SPI_ADC_SAMPLING_BLOCK_ID, MAX_READ_SAMPLES*sizeof(uint16_t))) == NULL){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}
	/*Register this sampling block*/
	if(ytRegisterSamplingBlock(samplingBlock) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/*Create the Adaptive tasks*/
	if((clDbsAdaptTask = ytNewAdaptTaskBlock(adaptTaskFunc, exitAdaptTaskFunc)) == NULL){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}
	/*Link the adaptive tasks to their sampling sources*/
	if(ytAdaptTaskAddSamplingBlock(clDbsAdaptTask, LFP_SPI_ADC_SAMPLING_BLOCK_ID) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}
	/*Start the Adaptive Task*/
	ytStartAdaptTask(clDbsAdaptTask, "clDbsTask", osPriorityLow, 300);

	/*Configure the SPI Read Write buffers*/
	ytSpiReadWriteBuff_t spiReadWriteBuffs;
	spiReadWriteBuffs.ptx = (uint8_t*) txBuffer;
	spiReadWriteBuffs.prx = (uint8_t*) samplingBlock->sampleBuffer;
	spiReadWriteBuffs.size = BUFFERED_READ_NUM_SAMPLES*sizeof(uint16_t);

	/*Configure Data Ready Pin as a falling edge interrupt*/
	if(ytGpioInitPin(GPIO_PIN_C1, GPIO_PIN_INTERRUPT_FALLING, GPIO_PIN_NO_PULL) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}
	if(ytGpioPinSetCallback(GPIO_PIN_C1, dataReadyCallback, NULL) != RET_OK){
		ytLedToggle(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	/* ****************Sampling Loop***********************/
	while(1){

		/*SemaphoreWait until Data Ready*/
		if(!interruptPending){
			osSemaphoreWait(drdySemaphore, osWaitForever);
		}
		else if(interruptPending > 1){		/*Only one pending interrupt allowed as maximum*/
			samplingOverflowFlag = 1;
		}
		interruptPending = 0;

		/*If pending TX command->Add it to TX buffer*/
		updateSpiCmd((rcvSpiData_t*)txBuffer);

		/*Read-Write with SPI*/
		ytIoctl(spiDevice, SPI_READ_WRITE, &spiReadWriteBuffs);	/*Wait until we end Sending/Receiving data*/
		samplingBlockReadSamples += BUFFERED_READ_NUM_SAMPLES;
		spiReadWriteBuffs.prx += spiReadWriteBuffs.size;
		if(spiReadWriteBuffs.prx >= samplingBlock->endBufferPtr){
			spiReadWriteBuffs.prx = samplingBlock->sampleBuffer;
		}

		/*When there are enough samples, release the adaptive tasks*/
		if(samplingBlockReadSamples >= numSamplesProc){
			osMutexWait(sampleRateMutex, osWaitForever);
			if(changeSampleRate){	/*Sample Rate change required*/
				changeSampleRate = 0;
				currentBufferCount = 0;
				currentTestBufferCount = 0;
				spiReadWriteBuffs.prx = samplingBlock->sampleBuffer;
				samplingBlock->currentBufferPtr = spiReadWriteBuffs.prx;
				energyWindowSamplesPtr = energyWindowSamples;
				testEnergyWindowSamplesPtr = testEnergyWindowSamples;
				setSampleRate(newSampleRate, &numSamplesProc, &numSamplesEnergyWindow);
				doubleThresholdFactor = numSamplesEnergyWindow/MIN_ENERGY_WINDOW_SAMPLES;
				energyOutputFactor = MAX_ENERGY_WINDOW_SAMPLES/numSamplesEnergyWindow;
			}
			else{
				/*Update the currentBuffer Pointer to the position of the new data to be acquired*/
				samplingBlock->currentBufferPtr = spiReadWriteBuffs.prx;
				/*Release the adaptive tasks linked to this sampling Block*/
				ytReleaseAdaptTasks(samplingBlock);
			}
			samplingBlockReadSamples = 0;
			osMutexRelease(sampleRateMutex);
		}
	}
}



/**
 * @brief						Function called each time new samples are available
 * @param adaptTaskBlock
 * @param currentSamplingBlock
 */
static void adaptTaskFunc(ytAdaptTaskBlock_t* adaptTaskBlock, ytSamplingBlock_t* currentSamplingBlock){

	uint16_t i;
	uint16_t* buffPtr;
	float32_t* currentCalcEnergyPtr;
	volatile timeMeas_t timeMeas; /*Volatile to debug with optimizations*/
	ytStartTimeMeasure(&timeMeas);
	osMutexWait(sampleRateMutex, osWaitForever);
	if(currentSamplingBlock->currentBufferPtr == currentSamplingBlock->sampleBuffer){
		buffPtr = ((uint16_t*) currentSamplingBlock->endBufferPtr) - numSamplesProc;
	}
	else{
		buffPtr = ((uint16_t*) currentSamplingBlock->currentBufferPtr) - numSamplesProc;
	}
	/*Samples to Float32*/
	for(i=0; i<numSamplesProc; i++){
		firInputSamples[i] = (((float32_t)buffPtr[i])*1.2f)/65535;
	}

	runLfpFilter(firInputSamples, energyWindowSamplesPtr);	/*Obtain the FIR of numSamplesProc samples*/

	float32_t avg = 0;						/*Calculate the mean value and subtract it*/
	for(i=0; i<numSamplesProc; i++){
		avg += energyWindowSamplesPtr[i];
	}
	avg /=numSamplesProc;
	for(i=0; i<numSamplesProc; i++){
		energyWindowSamplesPtr[i] -= avg;
	}

	energyWindowSamplesPtr += numSamplesProc;				/*Update stored samples in energy buffer*/
	if(energyWindowSamplesPtr >= energyWindowSamplesEndPtr){
		energyWindowSamplesPtr = energyWindowSamples;
	}

	uint32_t buffPosition = energyWindowSamplesPtr - energyWindowSamples;
	if(buffPosition >= numSamplesEnergyWindow){
		currentCalcEnergyPtr = energyWindowSamplesPtr - numSamplesEnergyWindow;
	}
	else{
		currentCalcEnergyPtr = energyWindowSamplesEndPtr - numSamplesEnergyWindow + buffPosition;
	}

	float32_t energy = 0;
	currentBufferCount++;
	if(currentBufferCount > NUM_DISCARDED_BUFFER_COUNT){		/*Only calculate the algorithm when the buffer is filled.*/
		currentBufferCount--;

		for(i=0; i<numSamplesEnergyWindow; i++){
			energy += (*currentCalcEnergyPtr)*(*currentCalcEnergyPtr);
			currentCalcEnergyPtr++;
			if(currentCalcEnergyPtr >= energyWindowSamplesEndPtr){
				currentCalcEnergyPtr = energyWindowSamples;
			}
		}

//		if(acqOn){
		doubleThresholdAlg(energy);
//		}
	}

	if(testSampleRateMode){	/*If the sample rate is in the test state*/
		testSampleRate(firInputSamples);
	}

	ytStopTimeMeasure(&timeMeas);
//	energy *= energyOutputFactor;
	osMutexRelease(sampleRateMutex);
//	ytStdoutSend((uint8_t*) &energy, sizeof(float32_t));
//	ytPrintf("Enrg:%.5f;%d\r\n", energy, timeMeas.elapsedTime);
//	ytLedToggle(LED_BLUE_1);
	/*Do nothing!*/


}

/**
 * @brief					Function called when the PID task is stopped to release its resources
 * @param adaptTaskBlock
 */
static void exitAdaptTaskFunc(ytAdaptTaskBlock_t* adaptTaskBlock){
	/*Do nothing!*/
}



/**
 *
 * @param samplePtr
 */
static void testSampleRate(float32_t* samplePtr){
	uint32_t i;
	float32_t* currentCalcEnergyPtr;
	float32_t* auxTestEnergyWindowPtr;
	uint32_t downsampleFactor = 1000/defaultSampleRate;

	for(i=0; i<testSamplesProc; i++){
		testFirInputBuffer[i] = samplePtr[i*downsampleFactor];			/*Obtain the downsampled buffer*/
	}

	runTestLfpFilter(testFirInputBuffer, testEnergyWindowSamplesPtr);	/*Obtain the FIR of numSamplesProc samples*/

	float32_t avg = 0;						/*Calculate the mean value and subtract it*/
	auxTestEnergyWindowPtr = testEnergyWindowSamplesPtr;
	for(i=0; i<testSamplesProc; i++){
		avg += (*auxTestEnergyWindowPtr);
		auxTestEnergyWindowPtr++;
		if(auxTestEnergyWindowPtr >= testEnergyWindowSamplesEndPtr){
			auxTestEnergyWindowPtr = testEnergyWindowSamples;
		}
	}
	avg /=testSamplesProc;
	auxTestEnergyWindowPtr = testEnergyWindowSamplesPtr;
	for(i=0; i<testSamplesProc; i++){
		(*auxTestEnergyWindowPtr) -= avg;
		auxTestEnergyWindowPtr++;
		if(auxTestEnergyWindowPtr >= testEnergyWindowSamplesEndPtr){
			auxTestEnergyWindowPtr = testEnergyWindowSamples;
		}
	}

	testEnergyWindowSamplesPtr = auxTestEnergyWindowPtr;	/*Update stored samples in energy buffer*/


	uint32_t buffPosition = testEnergyWindowSamplesPtr - testEnergyWindowSamples;
	if(buffPosition >= numSamplesTestEnergyWindow){
		currentCalcEnergyPtr = testEnergyWindowSamplesPtr - numSamplesTestEnergyWindow;
	}
	else{
		currentCalcEnergyPtr = testEnergyWindowSamplesEndPtr - numSamplesTestEnergyWindow + buffPosition;
	}

	currentTestBufferCount++;
	if(currentTestBufferCount > NUM_DISCARDED_BUFFER_COUNT){		/*Only calculate the algorithm when the buffer is filled.*/
		currentTestBufferCount--;
		float32_t energy = 0;

		for(i=0; i<numSamplesTestEnergyWindow; i++){
			energy += (*currentCalcEnergyPtr)*(*currentCalcEnergyPtr);
			currentCalcEnergyPtr++;
			if(currentCalcEnergyPtr >= testEnergyWindowSamplesEndPtr){
				currentCalcEnergyPtr = testEnergyWindowSamples;
			}
		}
		stimState_t testStimState;
		if(energy > (DOUBLE_THR_TOP*testDoubleThresholdFactor)){
			testStimState = STIM_STATE_S0;
		}
		else if(energy < (DOUBLE_THR_BOTTOM*testDoubleThresholdFactor)){
			testStimState = STIM_STATE_S2;
		}
		else{
			testStimState = STIM_STATE_S1;
		}

		if(testStimState != currentStimState){
			testErrorCount++;
		}
//		ytPrintf("Test:%.5f\r\n", energy);
	}


}


/**
 *
 * @param argument
 */
static void dynamicRateFunc(void const * argument){

	while(1){
		if(!nightMode){	/*Standard mode*/
			/*Wait for the rate check period*/
			osDelay(SAMPLE_RATE_CHECK_PERIOD);
#if USE_DYNAMIC_SAMPLE_RATE
			/*Change to the maximum sample rate 1000 Samples/s */
			updateSampleRate(FILTER_RATE_1000);

			osMutexWait(sampleRateMutex, osWaitForever);
			testEnergyWindowSamplesPtr = testEnergyWindowSamples;
			currentTestBufferCount = 0;
			setTestSampleRate(defaultLfpFilterRate, &testSamplesProc, &numSamplesTestEnergyWindow);
			testDoubleThresholdFactor = numSamplesTestEnergyWindow/MIN_ENERGY_WINDOW_SAMPLES;

			testSampleRateMode++;
			osMutexRelease(sampleRateMutex);

			osDelay(SAMPLE_RATE_TEST_TIME);		/*Wait a time to evaluate the currently default sample rate*/

			osMutexWait(sampleRateMutex, osWaitForever);
			testSampleRateMode = 0;
			osMutexRelease(sampleRateMutex);
			/*Check Error Count*/
			if(testErrorCount > MAX_TH_ERRORS){	/*To many errors in the current default sample rate, increase it*/
				increaseDefaultSamplerate();
			}
			else if(testErrorCount < MIN_TH_ERRORS){/*Reduce current default sample rate*/
				reduceDefaultSamplerate();
			}
			else{	/*Maintain current default sample rate*/

			}
			updateSampleRate(defaultLfpFilterRate);	/*Go to the default sample rate selected*/
			testErrorCount = 0;
#endif
		}
		else{
			updateSampleRate(FILTER_RATE_250);	/*Go to 250 Samples/s*/
			osDelay(5000);
			while(nightMode){
//				acqOn = 0;
				ytGpioPinSetCallback(GPIO_PIN_C1, NULL, NULL);	/*Enable interrupts to read data for a short time*/
				ytGpioDeInitPin(GPIO_PIN_C1);
				ytGpioInitPin(GPIO_PIN_C1, GPIO_PIN_ANALOG, GPIO_PIN_NO_PULL);
				osDelay(NIGHT_NO_READ_DATA_TIME);
				ytGpioDeInitPin(GPIO_PIN_C1);
				ytGpioInitPin(GPIO_PIN_C1, GPIO_PIN_INTERRUPT_FALLING, GPIO_PIN_NO_PULL);
				ytGpioPinSetCallback(GPIO_PIN_C1, dataReadyCallback, NULL);	/*Disable interrupts, so no new data is read*/
//				acqOn++;
				osDelay(NIGHT_READ_DATA_TIME);
			}
		}
	}
}

/**
 *
 * @return
 */
static retval_t increaseDefaultSamplerate(void){
	switch(defaultLfpFilterRate){
	case FILTER_RATE_1000:	/*Cannot be increased*/
		defaultLfpFilterRate = FILTER_RATE_1000;
		defaultSampleRate = 1000;
		break;
	case FILTER_RATE_500:
		defaultLfpFilterRate = FILTER_RATE_1000;
		defaultSampleRate = 1000;
		break;
	case FILTER_RATE_250:
		defaultLfpFilterRate = FILTER_RATE_500;
		defaultSampleRate = 500;
		break;
	case FILTER_RATE_100:
		defaultLfpFilterRate = FILTER_RATE_250;
		defaultSampleRate = 250;
		break;
	default:
		return RET_ERROR;
		break;
	}

	return RET_OK;
}


/**
 *
 * @return
 */
static retval_t reduceDefaultSamplerate(void){
	switch(defaultLfpFilterRate){
	case FILTER_RATE_1000:
		defaultLfpFilterRate = FILTER_RATE_500;
		defaultSampleRate = 500;
		break;
	case FILTER_RATE_500:
		defaultLfpFilterRate = FILTER_RATE_250;
		defaultSampleRate = 250;
		break;
	case FILTER_RATE_250:
		defaultLfpFilterRate = FILTER_RATE_100;
		defaultSampleRate = 100;
		break;
	case FILTER_RATE_100:/*Cannot be reduced*/
		defaultLfpFilterRate = FILTER_RATE_100;
		defaultSampleRate = 100;
		break;
	default:
		return RET_ERROR;
		break;
	}

	return RET_OK;
}

/**
 *
 * @param energy
 */
static void doubleThresholdAlg(float32_t energy){

	if(energy > (DOUBLE_THR_TOP*doubleThresholdFactor)){
		currentStimAmp++;
		if(currentStimAmp > MAX_STIM_AMP){
			currentStimAmp = MAX_STIM_AMP;
		}
		prepareSendCmd(CMD_SET_STIM_AMP, currentStimAmp);
		currentStimState = STIM_STATE_S0;
	}
	else if(energy < (DOUBLE_THR_BOTTOM*doubleThresholdFactor)){
		if(currentStimAmp){
			currentStimAmp--;
			prepareSendCmd(CMD_SET_STIM_AMP, currentStimAmp);
		}
		currentStimState = STIM_STATE_S2;
	}
	else{
		currentStimState = STIM_STATE_S1;
	}
}


/**
 *
 * @param spiDevice
 * @return
 */
static retval_t configSpiDevice(deviceFileHandler_t* spiDevice){

	/*Set Spi SPEED TO 8000000*/
	uint32_t spiSpeed = 8000000;
	if(ytIoctl(spiDevice, SPI_SET_SPEED, &spiSpeed) != RET_OK){
		return RET_ERROR;
	}

	/*Set SPI Polarity High*/
	ytSpiPolarity_t spiPolarity = SPI_POL_HIGH;
	if(ytIoctl(spiDevice, SPI_SET_POLARITY, &spiPolarity) != RET_OK){
		return RET_ERROR;
	}

	/*Set SPI edge*/
	ytSpiEdge_t spiEdge = SPI_EDGE_SECOND;
	if(ytIoctl(spiDevice, SPI_SET_EDGE, &spiEdge) != RET_OK){
		return RET_ERROR;
	}

	/*Set Spi mode to MASTER*/
	ytSpiMode_t spiMode = SPI_MASTER;
	if(ytIoctl(spiDevice, SPI_SET_MODE, &spiMode) != RET_OK){
		return RET_ERROR;
	}

	/*Set HW CS Management*/
	if(ytIoctl(spiDevice, SPI_SET_HW_CS, NULL) != RET_OK){
		return RET_ERROR;
	}

	return RET_OK;
}

/**
 *
 * @param argument
 */
static void dataReadyCallback(void const * argument){
	interruptPending++;
	osSemaphoreRelease(drdySemaphore);
}

/**
 *
 * @param lfpFilterRate
 */
static void updateSampleRate(lfpFilterRate_t lfpFilterRate){
	osMutexWait(sampleRateMutex, osWaitForever);
	changeSampleRate++;
	newSampleRate = lfpFilterRate;
	osMutexRelease(sampleRateMutex);
}


/* **********************Not Used********************/
//#define ON_OFF_STIM_TIME_THR	2000
//
//#define ON_OFF_THR_1000_RATE	0.9800f
//#define ON_OFF_THR_500_RATE		0.6380f
//#define ON_OFF_THR_250_RATE		0.2230f
//#define ON_OFF_THR_100_RATE		0.4045f
//
//static uint32_t stimStartTime = 0;
//static float32_t onOffThr = ON_OFF_THR_100_RATE;
///**
// *
// * @param energy
// */
//static void onOffAlg(float32_t energy){
//	if(energy > onOffThr){
//		if((osKernelSysTick() - stimStartTime) > ON_OFF_STIM_TIME_THR){
//			prepareSendCmd(CMD_SET_STIM_AMP, 100);
//			stimStartTime = osKernelSysTick();
//		}
//	}
//	else{
//		if((osKernelSysTick() - stimStartTime) > ON_OFF_STIM_TIME_THR){
//			prepareSendCmd(CMD_SET_STIM_AMP, 0);
//			stimStartTime = osKernelSysTick();
//		}
//	}
//}
//#define PID_TARGET_1000_RATE		0.9800f
//#define PID_TARGET_THR_500_RATE		0.6290f
//#define PID_TARGET_THR_250_RATE		0.2230f
//#define PID_TARGET_THR_100_RATE		0.4045f
//
//static float32_t pidEnergyTarget = PID_TARGET_THR_500_RATE;
///**
// *
// * @param energy
// */
//static void pidAlg(float32_t energy){
//	float32_t out;
//	out = arm_pid_f32(&pidInstance, (energy - pidEnergyTarget)*100);
//	prepareSendCmd(CMD_SET_STIM_AMP, (uint8_t) out);
//}
#endif
