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
 * ytStdio.c
 *
 *  Created on: 30 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file ytStdio.c
 */


#include "yetiOS.h"

#if YETIOS_ENABLE_STDIO

typedef enum stdioReadOp_{
	STDIO_NO_READ = 0,
	STDIO_READ,
	STDIO_READ_LINE,
}stdioReadOp_t;

static uint8_t* stdioReadBuffer;			/*Base address of the STDIO Input Buffer*/
static uint8_t* stdioEndReadBufferPtr;		/*End Address of the STDIO Input Buffer*/
static uint16_t stdioBufferCompleteFlag;	/*Flag that indicates that the Circular STDIO Input buffer has reached its end and starts filling from the beginning */

static uint8_t* stdioBufferHeadPtr;			/*Pointer to the current position of the STDIO buffer*/
static uint8_t* stdioNewBufferHeadPtr;		/*Pointer to the new current position to be set in the STDIO buffer*/

static volatile stdioReadOp_t stdioUsingReadOp;		/*Flag indicating a read operation is running. Only one read Operation can run simultaneously
 	 	 	 	 	 	 	 	 	 	 	 	 	 *This flag can be STDIO_NO_READ (=0), STDIO_READ, STDIO_READ_LINE*/

static uint8_t* readOpBufferPtr;			/*Base address of the buffer of the currently using the read operation*/
static uint8_t* readOpEndBufferPtr;			/*End address of the buffer of the currently using the read operation*/
static uint8_t* currentReadOpBufferPtr;		/*Current pointer in the buffer of the currently using the read operation*/

static osMutexId stdioUsingReadOpMutex;		/*Mutex to lock stdioUsingReadOp so only one simultaneous read operation is allowed*/
static osSemaphoreId stdioReadOpSemph;		/*Semaphore to notify a Read operation when it finish*/

static uint16_t stdioReadLineEchoEnabled; 			/*Flag to enable or disable Read Line echo*/
static volatile uint16_t stdioInitialized = 0;		/*Flag indicating the stdio has been initialized*/
static volatile uint16_t opRunningFlag = 0;			/*Flag indicating that a stdio operation is running, preventing deinitializing simultaneously*/
static volatile uint16_t endTaskFlag = 0;			/*Flag to indicate the stdio thread that it should end*/

static osThreadId stdioTaskHandle;			/*Stdio task handle*/

static deviceFileHandler_t* devFile;		/*Device File Hanlder of the stdio interface used*/

/*Task used for read from stdio Interface*/
static void ytStdioTask(void const * argument);
/*Callback function called when circular buffer is complete*/
static void readBufferFullCb(void);

/**
 *
 * @param deviceId
 * @return
 */
retval_t ytStdioInit(uint32_t deviceId){
	/*First try to open the HW interface. It must be done before entering the critical region, since open function
	 * could have Mutexes or Semaphores waiting inside which are not allowed when the scheduler is suspended*/
	if((devFile = ytOpen(deviceId, 0)) == NULL){	/*Unable to open HW stdio interface*/
		return RET_ERROR;
	}

	osThreadSuspendAll();	/*Init on critical region*/
	if(!stdioInitialized){


		stdioUsingReadOp = STDIO_NO_READ;
		/*Initialize stdio buffer pointers*/
		stdioReadBuffer = (uint8_t*)pvPortMalloc(YETIOS_STDIO_READ_BUFF_SIZE);
		stdioEndReadBufferPtr = &stdioReadBuffer[YETIOS_STDIO_READ_BUFF_SIZE];
		stdioBufferCompleteFlag = 0;

		stdioBufferHeadPtr = stdioReadBuffer;
		stdioNewBufferHeadPtr = stdioReadBuffer;
		stdioReadLineEchoEnabled = STDIO_READ_LINE_ECHO_ENABLED;

		stdioUsingReadOpMutex = ytMutexCreate();
		stdioReadOpSemph = ytSemaphoreCreate(1);

		stdioInitialized++;
		endTaskFlag = 0;
		ytStartThread("StdioTask", ytStdioTask, osPriorityLow, 256,	&stdioTaskHandle, NULL);

		osThreadResumeAll();

		osSemaphoreWait(stdioReadOpSemph, osWaitForever);	/*The first wait of the semaphore outside the critical region*/
		return RET_OK;
	}
	osThreadResumeAll();
	ytClose(devFile);
	return RET_ERROR;
}

/**
 *
 * @return
 */
retval_t ytStdioDeInit(void){
	osThreadSuspendAll();	/*DeInit on critical region*/
	if(stdioInitialized){
		endTaskFlag++;	/*Notify to securely end the stdio task*/
		while(endTaskFlag){	/*Let the stdio task end itself to securely call ytClose, and do not end the read while an ytIoctl is running*/
			osThreadResumeAll();
			osDelay(1);
			osThreadSuspendAll();
		}

		while(opRunningFlag){
			osSemaphoreRelease(stdioReadOpSemph);	/*Unblock in case any read operation is running*/
			osThreadResumeAll();
			osDelay(1);
			osThreadSuspendAll();
		}
		stdioUsingReadOp = STDIO_NO_READ;

		osSemaphoreDelete(stdioReadOpSemph);
		osMutexDelete(stdioUsingReadOpMutex);
		vPortFree(stdioReadBuffer);
		stdioInitialized--;
		osThreadResumeAll();

		/*Close the peripheral outside the critical region*/
		ytClose(devFile);

		return RET_OK;
	}
	osThreadResumeAll();
	return RET_ERROR;
}

/*Output Functions*/
/**
 *
 * @param format
 */
void ytPrintf(char* format, ...){
	opRunningFlag++;
	if(stdioInitialized){
		uint8_t* buff = (uint8_t* ) pvPortMalloc(YETIOS_PRINTF_BUFFER_SIZE);
		va_list argptr;
		va_start(argptr, format);
		vsnprintf((char*)buff, YETIOS_PRINTF_BUFFER_SIZE-1, format, argptr);
		va_end(argptr);

		ytWrite(devFile, buff, strlen((char*)buff));
		vPortFree(buff);
	}
	opRunningFlag--;
}

/**
 *
 * @param data
 * @param size
 * @param timeout
 * @return
 */
retval_t ytStdoutSend(uint8_t* data, uint32_t size){
	opRunningFlag++;
	if(stdioInitialized){
		if(ytWrite(devFile, data, size) == size){
			opRunningFlag--;
			return RET_OK;
		}
		else{
			opRunningFlag--;
			return RET_ERROR;
		}
	}
	opRunningFlag--;
	return RET_ERROR;
}



/*Input functions*/
/**
 *
 * @param data
 * @param size
 * @return
 */
retval_t ytStdioRead(uint8_t* data, uint32_t size){
	opRunningFlag++;
	if((size == 0) || (size > YETIOS_STDIO_READ_BUFF_SIZE)){
		opRunningFlag--;
		return RET_ERROR;
	}

	if(stdioInitialized){
		osMutexWait(stdioUsingReadOpMutex, osWaitForever);
		if(stdioUsingReadOp == STDIO_NO_READ){
			readOpBufferPtr = data;
			readOpEndBufferPtr = &data[size];
			currentReadOpBufferPtr = readOpBufferPtr;
			stdioUsingReadOp = STDIO_READ;
			osMutexRelease(stdioUsingReadOpMutex);

			if(osSemaphoreWait(stdioReadOpSemph, osWaitForever) != osOK){	/*Wait till the read is done*/

				if(stdioInitialized){
					osMutexWait(stdioUsingReadOpMutex, osWaitForever);	/*If Timeout reached exit read and return ERROR*/
					stdioUsingReadOp = STDIO_NO_READ;
					osMutexRelease(stdioUsingReadOpMutex);
				}
				opRunningFlag--;
				return RET_ERROR;
			}
		}
		else{
			osMutexRelease(stdioUsingReadOpMutex);
			opRunningFlag--;
			return RET_ERROR;
		}
		opRunningFlag--;
		return RET_OK;
	}
	opRunningFlag--;
	return RET_ERROR;
}


/**
 *
 * @param data
 * @param maxSize
 * @return
 */
retval_t ytStdioReadLine(uint8_t* data, uint32_t maxSize){
	opRunningFlag++;
	if((maxSize == 0) || (maxSize > YETIOS_STDIO_READ_BUFF_SIZE)){
		opRunningFlag--;
		return RET_ERROR;
	}
	if(stdioInitialized){
		osMutexWait(stdioUsingReadOpMutex, osWaitForever);
		if(stdioUsingReadOp == STDIO_NO_READ){
			readOpBufferPtr = data;
			readOpEndBufferPtr = &data[maxSize];
			currentReadOpBufferPtr = readOpBufferPtr;
			stdioUsingReadOp = STDIO_READ_LINE;
			osMutexRelease(stdioUsingReadOpMutex);

			if(osSemaphoreWait(stdioReadOpSemph, osWaitForever) != osOK){	/*Wait till the read is done*/

				osMutexWait(stdioUsingReadOpMutex, osWaitForever);
				stdioUsingReadOp = STDIO_NO_READ;
				osMutexRelease(stdioUsingReadOpMutex);
				opRunningFlag--;
				return RET_ERROR;
			}
			if(stdioInitialized){
				osMutexWait(stdioUsingReadOpMutex, osWaitForever);
				if(currentReadOpBufferPtr >= readOpEndBufferPtr){	/*Buffer end reached before the end of line character is found*/
					stdioUsingReadOp = STDIO_NO_READ;
					osMutexRelease(stdioUsingReadOpMutex);
					opRunningFlag--;
					return RET_ERROR;
				}
				osMutexRelease(stdioUsingReadOpMutex);
			}
		}
		else{
			osMutexRelease(stdioUsingReadOpMutex);
			opRunningFlag--;
			return RET_ERROR;
		}
		opRunningFlag--;
		return RET_OK;
	}
	opRunningFlag--;
	return RET_ERROR;
}

/**
 *
 */
void ytEnableStdioReadLineEcho(void){
	stdioReadLineEchoEnabled++;
}

/**
 *
 */
void ytDisableStdioReadLineEcho(void){
	stdioReadLineEchoEnabled = 0;
}

/**
 *
 * @param argument
 */
static void ytStdioTask(void const * argument){			/*Low Priority*/

	if(ytIoctl(devFile, SET_ASYNC_CIRCULAR_READ_MODE, NULL) != RET_OK){	/*Set read async circular mode*/
		ytStdioDeInit();	/*This will terminate the thread*/
	}
	if(ytIoctl(devFile, SET_READ_COMPLETE_CB, (void*) readBufferFullCb) != RET_OK){	/*Set the buffer complete callback*/
		ytStdioDeInit();	/*This will terminate the thread*/
	}
	ytRead(devFile, stdioReadBuffer, YETIOS_STDIO_READ_BUFF_SIZE);	/*Continuous non blocking read*/

	uint32_t numReadBytes = 0;
	uint32_t taskPeriod = YETIOS_STDIO_TASK_PERIOD;
	while(1){

		if(endTaskFlag){		/*Terminate the thread in an adequate point*/
			endTaskFlag = 0;
			osThreadTerminate(stdioTaskHandle);
		}
		ytIoctl(devFile, WAIT_FOR_NEW_DATA, (void*) &taskPeriod);	/*Wait for new data or timeout*/

		/*Update the new head pointer*/
		ytIoctl(devFile, GET_CURRENT_READ_PTR, (void*) &stdioNewBufferHeadPtr);

		/*Get the number of new bytes*/
		if(stdioNewBufferHeadPtr >= stdioBufferHeadPtr){
			numReadBytes = stdioNewBufferHeadPtr - stdioBufferHeadPtr;

			/*Check if a HW overflow happened. It could happen when the new pointer has a larger value than the previous one
			 * and the buffer has been marked as completed at least once*/
			if(stdioBufferCompleteFlag){
				numReadBytes =	YETIOS_STDIO_READ_BUFF_SIZE;
				stdioBufferHeadPtr = stdioNewBufferHeadPtr;
			}
		}
		else{
			numReadBytes = (stdioNewBufferHeadPtr - stdioReadBuffer) + (stdioEndReadBufferPtr- stdioBufferHeadPtr);

			/*Check if a HW overflow happened. It could happen when the new pointer has a lower value than the previous one
			 * and the buffer has been marked as completed at least twice*/
			if(stdioBufferCompleteFlag >= 2){
				numReadBytes =	YETIOS_STDIO_READ_BUFF_SIZE;
				stdioBufferHeadPtr = stdioNewBufferHeadPtr;
			}
		}

		/*Store the new bytes in the reading buffers*/
		if(numReadBytes > 0){
			stdioBufferCompleteFlag = 0;			/*The buffer complete flag is reset when any byte is read, so there is new room in the buffer*/
			/*If the operation is Read Line, store the bytes in the read buffer until the end line characters are found*/
			osMutexWait(stdioUsingReadOpMutex, osWaitForever);
			if(stdioUsingReadOp == STDIO_READ_LINE){
				osMutexRelease(stdioUsingReadOpMutex);
				while(numReadBytes > 0){/*Store all the bytes and check for new line characters*/
					if(currentReadOpBufferPtr < readOpEndBufferPtr){	/*Check if there is enough room in the read buffer*/

						if(*stdioBufferHeadPtr == '\b'){	/*Backspace processing*/
							if(currentReadOpBufferPtr > readOpBufferPtr){
								currentReadOpBufferPtr--;
								if(stdioReadLineEchoEnabled){
									ytWrite(devFile, (uint8_t*)"\b \b", 3);
								}
							}
						}
						else{

							*currentReadOpBufferPtr = *stdioBufferHeadPtr;	/*Store each new byte*/

							if(stdioReadLineEchoEnabled){
								ytWrite(devFile, currentReadOpBufferPtr, 1);
							}
							/*Check End of Line character*/
							if(((*currentReadOpBufferPtr)) == '\r' || ((*currentReadOpBufferPtr) == '\n')){
								/*Update the head pointer, release the waiting semaphore and break*/
								(*currentReadOpBufferPtr) = '\0';
								stdioBufferHeadPtr++;
								if(stdioBufferHeadPtr >= stdioEndReadBufferPtr){
									stdioBufferHeadPtr = stdioReadBuffer;
								}
								osMutexWait(stdioUsingReadOpMutex, osWaitForever);
								stdioUsingReadOp = STDIO_NO_READ;
								osMutexRelease(stdioUsingReadOpMutex);
								osSemaphoreRelease(stdioReadOpSemph);
								break;
							}
							currentReadOpBufferPtr++;
						}
					}
					else{	/*Read line buffer reached its end without finding an end of line.
							The read Line function should process this as an error
							Dont reset stdioUsingReadOp to STDIO_NO_READ, let error handling in readLine function do it*/
						osSemaphoreRelease(stdioReadOpSemph);
						break;
					}

					/*Update the head pointer*/
					stdioBufferHeadPtr++;
					if(stdioBufferHeadPtr >= stdioEndReadBufferPtr){
						stdioBufferHeadPtr = stdioReadBuffer;
					}
					numReadBytes--;
				}
			}
			else if(stdioUsingReadOp == STDIO_READ){/*In a standard bytes read operation the bytes are stored until the indicated size is reached*/
				osMutexRelease(stdioUsingReadOpMutex);
				while(numReadBytes > 0){/*Store all the bytes and check for new line characters*/

					*currentReadOpBufferPtr = *stdioBufferHeadPtr;	/*Store each new byte*/

					currentReadOpBufferPtr++;
					/*Update the head pointer*/
					stdioBufferHeadPtr++;
					if(stdioBufferHeadPtr >= stdioEndReadBufferPtr){
						stdioBufferHeadPtr = stdioReadBuffer;
					}
					numReadBytes--;

					if(currentReadOpBufferPtr >= readOpEndBufferPtr){		/*Check if the end has been reached*/
						osMutexWait(stdioUsingReadOpMutex, osWaitForever);
						stdioUsingReadOp = STDIO_NO_READ;
						osMutexRelease(stdioUsingReadOpMutex);
						osSemaphoreRelease(stdioReadOpSemph);
						break;
					}

				}
			}
			else{
				osMutexRelease(stdioUsingReadOpMutex);
			}
		}

	}

}

/**
 *
 * @param devFile
 */
static void readBufferFullCb(void){
	stdioBufferCompleteFlag++;
}

#endif
