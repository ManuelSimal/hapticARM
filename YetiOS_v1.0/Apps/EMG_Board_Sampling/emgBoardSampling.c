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
 * emgBoardSampling.c
 *
 *  Created on: 7 feb. 2020
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file emgBoardSampling.c
 */


#include "yetiOs.h"
#include "ad7124.h"
#include "ad7124Regs.h"
#include <stdlib.h>


#if EMG_SAMPLING_APP


#define AD7124_SPI_SPEED		6000000
#define AD7124_GPIO_CS_PIN 		GPIO_PIN_A4
#define AD7124_SPI_DEV			PLATFORM_SPI1_DEVICE_ID


#define AD7124_GPIO_DRY_PIN 	GPIO_PIN_C5

#define ADC_GAIN				1

#define NUMBER_OF_CHANNELS		4
#define CHANNEL_BUFFER_SIZE		512

#define NUM_2_POW_23			8388608

#define ENABLE_OUTPUT_DATA		1
#define INIT_FRAME_ID			0xFFFFEFAD
#define END_FRAME_ID			0xFFFFFADE

#define DEFAULT_FFT_SAMPLE_NUM	256				//Must be a 2s power
//Channel type
typedef struct channelBuff_{
	float32_t sampleBuffer[CHANNEL_BUFFER_SIZE];
	float32_t* buffPtr;
}channelBuff_t;


//Processing modes
typedef enum processingMode_{
	PROC_TIME_DOMAIN = 0,
	PROC_FFT = 1,
	//MULTICHANNEL POST PROC
}processingMode_t;

//Processing data configuration type
typedef struct processingConfig_{
	processingMode_t processingMode;
	//Window
	uint16_t fftSampleNum;
	//Etc
}processingConfig_t;

//Channel buffers
static channelBuff_t* channelBuff;

//Channel output buffers
static float32_t channelOutputBuf[NUMBER_OF_CHANNELS][CHANNEL_BUFFER_SIZE];

//Processing configuration
static processingConfig_t processingConfig;
//Processing configuration used to update the current values from command interface
static processingConfig_t updateProcessingConfig;

//Processes instances
static osThreadId ad7124AcqProcId;
static osThreadId processingProcId;
static osThreadId dataOutProcId;

static osSemaphoreId ad7124ReadSemaphore;
static osSemaphoreId allChannelsSampledSemaphore;
static osSemaphoreId outputDataSemaphore;

//Process FFT Function
static retval_t processFft(float32_t* inData, float32_t* outData, uint16_t fftSampleNum);

//Data ready Pin Callback
static void dataReadyPinCallback(void const * argument);

//Configuration command function for the acquisition module
static void acqConfigCommand(uint32_t argc, char** argv);

//AD7124 Acquisition process
static void ad7124AcqFunc(void const * argument);
//Signals Processing process
static void processingFunc(void const * argument);
//Data output process
static void dataOutFunc(void const * argument);

YtAutoInitThread(ad7124_acq_proc, ad7124AcqFunc, osPriorityAboveNormal, 256, &ad7124AcqProcId, NULL);
YtAutoInitThread(processing_proc, processingFunc, osPriorityNormal, 256, &processingProcId, NULL);
YtAutoInitThread(data_out_proc, dataOutFunc, osPriorityBelowNormal, 256, &dataOutProcId, NULL);

#define BLOCK_SIZE           256
#define NUM_TAPS 			 65

static arm_fir_instance_f32 S1;
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

static float32_t filterCoefficients[NUM_TAPS] =
{
		0.0606467253536945722f, -0.0314183213878071535f, -0.00852681395914876156f, 0.011526883911874692f,
		0.0242251070547044624f, 0.0273471078870842592f, 0.0210370620252575136f, 0.00743938709260434736f,
		-0.00947297562820596448f, -0.0248110303640633016f, -0.0336954522736558765f, -0.032876389196422294f,
		-0.0217982870365519624f, -0.00339653119240760455f, 0.0167611524925375503f, 0.031977518605460005f,
		0.0365893564198917567f, 0.0281388222882209688f, 0.00844680046341009932f, -0.0165406165699413209f,
		-0.0384871302898627046f, -0.049594202439858695f, -0.0454732679720572797f, -0.0266465819451935648f,
		0.0013560839830527719f, 0.0292522658663128404f, 0.0474616479531444757f, 0.0497310286994902817f,
		0.0344935200828566726f, 0.00678786852775527423f, -0.0245091872884221733f, -0.0487863691581232745f,
		0.942044393667672786f, -0.0487863691581232745f, -0.0245091872884221733f, 0.00678786852775527423f,
		0.0344935200828566726f, 0.0497310286994902817f, 0.0474616479531444757f, 0.0292522658663128404f,
		0.0013560839830527719f, -0.0266465819451935648f, -0.0454732679720572797f, -0.049594202439858695f,
		-0.0384871302898627046f, -0.0165406165699413209f, 0.00844680046341009932f, 0.0281388222882209688f,
		0.0365893564198917567f, 0.031977518605460005f, 0.0167611524925375503f, -0.00339653119240760455f,
		-0.0217982870365519624f, -0.032876389196422294f, -0.0336954522736558765f, -0.0248110303640633016f,
		-0.00947297562820596448f, 0.00743938709260434736f, 0.0210370620252575136f, 0.0273471078870842592f,
		0.0242251070547044624f, 0.011526883911874692f, -0.00852681395914876156f, -0.0314183213878071535f,
		0.0606467253536945722f
};

/**
 *
 * @param argument
 */
static void ad7124AcqFunc(void const * argument){
	ad7124Dev_t* ad7124Dev;
//	ytTimeMeas_t time_meas;
	int32_t sample;
	float64_t sampleDoubleVolts;
	uint16_t currentChannel;
	uint16_t expectedChannel;
	uint16_t i;
	uint8_t currentStatus;
	channelBuff = (channelBuff_t*) pvPortMalloc(sizeof(channelBuff_t)*NUMBER_OF_CHANNELS);
	//Initialize buffer pointers
	for(i=0; i<NUMBER_OF_CHANNELS; i++){
		channelBuff[i].buffPtr = &(channelBuff[i].sampleBuffer[0]);
	}

	arm_fir_init_f32(&S1, NUM_TAPS, (float32_t *)&filterCoefficients[0], &firStateF32[0], BLOCK_SIZE);

	//Initialize semaphores
	ad7124ReadSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(ad7124ReadSemaphore, osWaitForever);
	allChannelsSampledSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(allChannelsSampledSemaphore, osWaitForever);
	outputDataSemaphore = ytSemaphoreCreate(1);
	osSemaphoreWait(outputDataSemaphore, osWaitForever);

	//Register Config command
	osDelay(100);
	ytShellRegisterCommand("ACQ_CONFIG", acqConfigCommand);

	//Setup AD7124
	ad7124Setup(&ad7124Dev, ad7124Regs);		//DEFAULT CONFIGURATION
	osDelay(100);

	//Initialize DRDY PIN
	ytGpioInitPin(AD7124_GPIO_DRY_PIN, GPIO_PIN_INTERRUPT_FALLING, GPIO_PIN_NO_PULL);		//CONFIG DRDY PIN
	ytGpioInitPin(GPIO_PIN_C1, GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_NO_PULL);		//TEST PIN

	expectedChannel = 0;

	//Main acquisition Loop
	while(1){

//		ytStartTimeMeasure(&time_meas);	//Start Measure time

		//DISABLE DRDY PIN WHILE READING THE DATA
		ytGpioPinSetCallback(AD7124_GPIO_DRY_PIN, NULL, NULL);

		//Read the data from ADC
		ad7124ReadDataStatusAppend(ad7124Dev, &sample, &currentStatus);

		//ENABLE DRDY PIN TO INTERRUPT WHEN NEW DATA IS AVAILABLE
		ytGpioPinSetCallback(AD7124_GPIO_DRY_PIN, dataReadyPinCallback, NULL);

		//Calculate Volts
		sampleDoubleVolts = ((((float64_t)sample)  / ((float64_t)NUM_2_POW_23)) - 1) *3.3;

		//Get current Channel
		currentChannel = (uint16_t) AD7124_STATUS_REG_CH_ACTIVE(currentStatus);

		//Some data in any channel has been lost. Fill these data with zeros
		if(currentChannel != expectedChannel){		//Missed samples
			if (currentChannel > expectedChannel){
				for(i=expectedChannel; i<currentChannel; i++){
					*(channelBuff[i].buffPtr) = 0;					//Fill with 0 the missed samples
					if(channelBuff[i].buffPtr < (&(channelBuff[i].sampleBuffer[CHANNEL_BUFFER_SIZE]))){
						channelBuff[i].buffPtr++;
					}
					else{
						channelBuff[i].buffPtr = (&(channelBuff[i].sampleBuffer[0]));
					}
				}
			}
			else{
				for(i=expectedChannel; i<NUMBER_OF_CHANNELS; i++){
					*(channelBuff[i].buffPtr) = 0;					//Fill with 0 the missed samples
					if(channelBuff[i].buffPtr < (&(channelBuff[i].sampleBuffer[CHANNEL_BUFFER_SIZE]))){
						channelBuff[i].buffPtr++;
					}
					else{
						channelBuff[i].buffPtr = (&(channelBuff[i].sampleBuffer[0]));
					}
				}
				for(i=0; i<currentChannel; i++){
					*(channelBuff[i].buffPtr) = 0;					//Fill with 0 the missed samples
					if(channelBuff[i].buffPtr < (&(channelBuff[i].sampleBuffer[CHANNEL_BUFFER_SIZE]))){
						channelBuff[i].buffPtr++;
					}
					else{
						channelBuff[i].buffPtr = (&(channelBuff[i].sampleBuffer[0]));
					}
				}
			}
		}

		//Store the current chanel sample in the buffer
		*(channelBuff[currentChannel].buffPtr) = (float32_t) sampleDoubleVolts;			//Store sample in Channel Buffer

		//Update the channel buffer pointer
		if(channelBuff[currentChannel].buffPtr < (&(channelBuff[currentChannel].sampleBuffer[CHANNEL_BUFFER_SIZE]))){
			channelBuff[currentChannel].buffPtr++;
		}
		else{
			channelBuff[currentChannel].buffPtr = (&(channelBuff[currentChannel].sampleBuffer[0]));
		}

		//Update the next expected channel
		if(currentChannel >= (NUMBER_OF_CHANNELS-1)){
			expectedChannel = 0;
			 osSemaphoreRelease(allChannelsSampledSemaphore);//A new sample have been obtained in all channels. Free processing thread
		}
		else{
			expectedChannel = currentChannel +1;
		}

//		ytStopTimeMeasure(&time_meas);		//Stop measuring time

		//Wait till the DRDY interrupt is toggled
	    osSemaphoreWait(ad7124ReadSemaphore, osWaitForever);
	    ytGpioPinToggle(GPIO_PIN_C1);
	}
}
/* *************************************************/


//Signals Processing process
/**
 *
 * @param argument
 */
void static processingFunc(void const * argument){
	uint16_t currentFftSampleIndex = 0;
	uint16_t i;

	//Initialize processing configuration to time domain
	processingConfig.processingMode = PROC_TIME_DOMAIN;
	processingConfig.fftSampleNum = DEFAULT_FFT_SAMPLE_NUM;
	updateProcessingConfig.processingMode = processingConfig.processingMode;
	updateProcessingConfig.fftSampleNum = processingConfig.fftSampleNum;

	while(1){

		//Wait till the acquisition process signals that it has acquire a new sample in all channels
	    osSemaphoreWait(allChannelsSampledSemaphore, osWaitForever);

		switch(processingConfig.processingMode){

		case PROC_TIME_DOMAIN: //Nothing to do. Just output the signals. Entered each time a sample is acquired in every channel
			 osSemaphoreRelease(outputDataSemaphore); //Free the semaphore to output the acquired data
			break;

		case PROC_FFT:		//Do FFT when fft_samples_num are available and output the signal

			//Check if there are enough stored samples to perform the FFT
			if((channelBuff[NUMBER_OF_CHANNELS-1].buffPtr >= &(channelBuff[NUMBER_OF_CHANNELS-1].sampleBuffer[currentFftSampleIndex + processingConfig.fftSampleNum]))
					|| (channelBuff[NUMBER_OF_CHANNELS-1].buffPtr < &(channelBuff[NUMBER_OF_CHANNELS-1].sampleBuffer[currentFftSampleIndex]))){

				for(i=0; i<NUMBER_OF_CHANNELS; i++){
					if(i == 0){
						processFft(&(channelBuff[i].sampleBuffer[currentFftSampleIndex]), &channelOutputBuf[i][0], processingConfig.fftSampleNum);
					}
					else{
//						processFft(&(channelBuff[i].sampleBuffer[currentFftSampleIndex]), &channelOutputBuf[i][0], processingConfig.fftSampleNum);
					}
				}

				currentFftSampleIndex += processingConfig.fftSampleNum;
				if(currentFftSampleIndex >= CHANNEL_BUFFER_SIZE){
					currentFftSampleIndex = 0;
				}
				osSemaphoreRelease(outputDataSemaphore); //Free the semaphore to output the fft processed data of all channels
			}

			break;
		default:
			break;
		}

	}
}
/* *************************************************/


//Data output process
/**
 *
 * @param argument
 */
static void dataOutFunc(void const * argument){
	uint32_t frameId;
	float32_t* outDataPtr;
	uint16_t i, j;

	//ATENTION!! This process has the lowest priority. Output data may overrun if other processes uses much CPU
	while(1){

		//Wait till the proccessing thread signals to output some data
		osSemaphoreWait(outputDataSemaphore, osWaitForever);

		switch(processingConfig.processingMode){
			case PROC_TIME_DOMAIN: //Entered each time a sample is acquired in every channel

				frameId = INIT_FRAME_ID;
				ytStdoutSend((uint8_t*) &frameId, sizeof(float32_t));	//First send the frame ID

				for(i= 0; i<NUMBER_OF_CHANNELS; i++){	//For each channel output the last obtained value

					if(channelBuff[i].buffPtr != (&(channelBuff[i].sampleBuffer[0]))){
						outDataPtr = channelBuff[i].buffPtr - 1;
					}
					else{
						outDataPtr = &(channelBuff[i].sampleBuffer[CHANNEL_BUFFER_SIZE-1]);
					}

					ytStdoutSend((uint8_t*) outDataPtr, sizeof(float32_t));
				}
				break;

			case PROC_FFT:		//Send FFT data output when it is processed
				frameId = INIT_FRAME_ID;
				ytStdoutSend((uint8_t*) &frameId, sizeof(float32_t));	//First send the init frame ID

				for(i= 0; i<NUMBER_OF_CHANNELS; i++){	//For each channel output all the processed FFT samples
					for(j=0; j<(processingConfig.fftSampleNum/2); j++){
						ytStdoutSend((uint8_t*) &channelOutputBuf[i][j], sizeof(float32_t));
					}

				}

				frameId = END_FRAME_ID;
				ytStdoutSend((uint8_t*) &frameId, sizeof(float32_t));	//Send the end frame ID
				break;
			default:
				break;
			}

		//Update config if necessary. This is the lowest priority thread, so it will be updated only when the processing thread is done
		if(updateProcessingConfig.fftSampleNum != processingConfig.fftSampleNum){
			processingConfig.fftSampleNum = updateProcessingConfig.fftSampleNum;
		}
		if(updateProcessingConfig.processingMode != processingConfig.processingMode){
			processingConfig.processingMode = updateProcessingConfig.processingMode;
		}

	}
}
/* *************************************************/


//AD7124 data ready pin callback. Release the semaphore to read the data
/**
 *
 * @param argument
 */
static void dataReadyPinCallback(void const * argument){
	osSemaphoreRelease(ad7124ReadSemaphore);
}

//Configuration command function for the acquisition module
/**
 *
 * @param argc
 * @param argv
 */
static void acqConfigCommand(uint32_t argc, char** argv){

	if(argc < 2){
		return;
	}
	else{
		if(argc == 3){
			if(argv[1][0] == 'M'){		//Change mode command
				updateProcessingConfig.processingMode = (processingMode_t) atoi(argv[2]);
			}
			if(argv[1][0] == 'F'){		//Change FFT samples command
				updateProcessingConfig.fftSampleNum = (uint16_t) atoi(argv[2]);
			}
			else{
				return;
			}
		}
		else{
			return;
		}
	}
	return;

}



//
///* -------------------------------------------------------------------
// * Declare State buffer of size (numTaps + blockSize - 1)
// * ------------------------------------------------------------------- */
//static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
//
///* ----------------------------------------------------------------------
//** FIR Coefficients buffer generated using fir1() MATLAB function.
//** ------------------------------------------------------------------- */
//static float32_t filterCoefficients[NUM_TAPS] =
//{
//		-3.27414835903473e-18,	-0.000224482338935084,	-0.000560146034538294,	-0.00110613172163582,
//		-0.00196260956910152,	-0.00322178433050895,	-0.00495892992267924,	-0.00722419322271586,
//		-0.0100358703230005,	-0.0133757629896308,	-0.0171870751701219,	-0.0213751202448101,
//		-0.0258108941069969,	-0.0303373443243691,	-0.0347779498345095,	-0.0389470366146975,
//		-0.0426611082987929,	-0.0457503792143622,	-0.0480696688008289,	-0.0495078537571182,
//		0.949908326603572,	-0.0495078537571182,	-0.0480696688008289,	-0.0457503792143622,
//		-0.0426611082987929,	-0.0389470366146975,	-0.0347779498345095,	-0.0303373443243691,
//		-0.0258108941069969,	-0.0213751202448101,	-0.0171870751701219,	-0.0133757629896308,
//		-0.0100358703230005,	-0.00722419322271586,	-0.00495892992267924,	-0.00322178433050895,
//		-0.00196260956910152,	-0.00110613172163582,	-0.000560146034538294,	-0.000224482338935084,
//		-3.27414835903473e-18
//};


float32_t post_filter[256];

//Process FFT function
/**
 *
 * @param inData
 * @param outData
 * @param fftSampleNum
 * @return
 */
static retval_t processFft(float32_t* inData, float32_t* outData, uint16_t fftSampleNum){


	//FIR FILTER 50 HZ
	//ARM FIR FILTER (TIME 2 MS)
//	ytTimeMeas_t time_meas;
//	ytStartTimeMeasure(&time_meas);
//
//	arm_fir_instance_f32 S1;
//	arm_fir_init_f32(&S1, NUM_TAPS, (float32_t *)&filterCoefficients[0], &firStateF32[0], NUM_TAPS);
//
//	arm_fir_f32(&S1, (float*) inData, (float*) outData, BLOCK_SIZE);
//	uint16_t i;
//	float32_t avg = 0;
//	for(i=0; i<256; i++){
//		avg += outData[i];
//	}
//
//	avg /= BLOCK_SIZE;
//
//	for(i=0; i<256; i++){
//		outData[i] -= avg;
//	}

//	arm_fir_f32(&S1, (float*) inData, (float*) post_filter, BLOCK_SIZE);

//	ytStopTimeMeasure(&time_meas);

    arm_rfft_fast_instance_f32 S;

    arm_rfft_fast_init_f32(&S, fftSampleNum);
    arm_rfft_fast_f32(&S, post_filter, outData, 0);


    //The real FFT output has fftSampleNum/2 complex samples, since the other half are their conjugate
    arm_cmplx_mag_f32(outData, outData, fftSampleNum/2);
    return RET_OK;
}


/**
 *
 * @param ad7124Dev
 * @return
 */
int32_t ad7124ConfigSpiDevice(ad7124Dev_t* ad7124Dev){

	ytIoctlCmd_t spiIoctlCmd;

	ytSpiPolarity_t spiPol = SPI_POL_HIGH;
	ytSpiEdge_t spiEdge = SPI_EDGE_SECOND;
	ytSpiMode_t spiMode = SPI_MASTER;
	uint32_t spiSpeed = AD7124_SPI_SPEED;
	uint32_t csPin = AD7124_GPIO_CS_PIN;

	if ((ad7124Dev->spiFd = ytOpen(AD7124_SPI_DEV, 0)) == NULL){
		return -1;
	}
	spiIoctlCmd = SPI_SET_SW_CS_ALWAYS;
	if(ytIoctl(ad7124Dev->spiFd, spiIoctlCmd, &(csPin)) != RET_OK){
		return -1;
	}
	spiIoctlCmd = SPI_SET_POLARITY;
	if(ytIoctl(ad7124Dev->spiFd, spiIoctlCmd, &spiPol) != RET_OK){
		return -1;
	}
	spiIoctlCmd = SPI_SET_EDGE;
	if(ytIoctl(ad7124Dev->spiFd, spiIoctlCmd, &spiEdge) != RET_OK){
		return -1;
	}
	spiIoctlCmd = SPI_SET_MODE;
	if(ytIoctl(ad7124Dev->spiFd, spiIoctlCmd, &spiMode) != RET_OK){
		return -1;
	}
	spiIoctlCmd = SPI_SET_SPEED;
	if(ytIoctl(ad7124Dev->spiFd, spiIoctlCmd, &spiSpeed) != RET_OK){
		return -1;
	}

	return 0;


}


#endif
