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
 * ytShell.c
 *
 *  Created on: 30 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file ytShell.c
 */

#include "yetiOS.h"

#if YETIOS_ENABLE_YETISHELL

#define AVAILABLE_MEM_CMD			"mem"
#define LIST_COMMANDS_CMD			"lcmd"

typedef struct ytShellStoredCommand_{
	char* commandName;
	ytShellFunc_t commandFunc;
}ytShellStoredCommand_t;


static genList_t* ytShellCommandsList;
static osMutexId ytShellListMutex;
static osThreadId ytShellTaskHandle;

static volatile uint16_t endTaskFlag = 0;
static volatile uint16_t ytShellInitialized = 0;
static volatile uint16_t ytShellOpRunning = 0;

static void ytShellTask(void const * argument);
static char** strSplit( char* str, char delim, uint32_t* numSplits);

static void printCommandsListCmd(uint32_t argc, char** argv);
static void availableMemCmd(uint32_t argc, char** argv);

/**
 *
 * @return
 */
retval_t ytShellInit(){
	osThreadSuspendAll();	/*Init on critical region*/
	if(!ytShellInitialized){
		ytShellCommandsList = genListInit();
		ytShellListMutex = ytMutexCreate();
		endTaskFlag = 0;
		ytShellInitialized++;
		ytStartThread("YetiShellTask", ytShellTask, osPriorityLow, 256,	&ytShellTaskHandle, NULL);

		osThreadResumeAll();

		ytShellRegisterCommand(LIST_COMMANDS_CMD, printCommandsListCmd);
		ytShellRegisterCommand(AVAILABLE_MEM_CMD, availableMemCmd);
		return RET_OK;
	}
	osThreadResumeAll();
	return RET_ERROR;
}

/**
 *
 * @return
 */
retval_t ytShellDeInit(){
	genListElement_t* currentCommand;
	ytShellStoredCommand_t* storedCommand;
	osThreadSuspendAll();	/*DeInit on critical region*/
	if(ytShellInitialized){
		endTaskFlag++;	/*Notify to securely end the yetishell task*/
		while(endTaskFlag){
			osThreadResumeAll();
			osDelay(1);
			osThreadSuspendAll();
		}

		while(ytShellOpRunning){
			osThreadResumeAll();
			osDelay(1);
			osThreadSuspendAll();
		}
		ytShellInitialized = 0;

		/*Remove all commands from List*/
		currentCommand = ytShellCommandsList->tailElement;
		while(currentCommand != NULL){	/*Loop the commands list*/

			storedCommand = (ytShellStoredCommand_t*) currentCommand->item;
			vPortFree(storedCommand->commandName);
			vPortFree(storedCommand);

			currentCommand = currentCommand->next;
		}
		/*Remove the list*/
		genListRemoveAll(ytShellCommandsList);

		osMutexDelete(ytShellListMutex);
		osThreadResumeAll();
		return RET_OK;
	}

	osThreadResumeAll();
	return RET_ERROR;
}

/**
 *
 * @param commandName
 * @param ytShellFunc
 * @return
 */
retval_t ytShellRegisterCommand(char* commandName, ytShellFunc_t ytShellFunc){
	genListElement_t* currentCommand;
	ytShellStoredCommand_t* storedCommand;
	ytShellOpRunning++;
	if(ytShellInitialized){
		osMutexWait(ytShellListMutex, osWaitForever);/*Lock the command list*/

		currentCommand = ytShellCommandsList->tailElement;
		while(currentCommand != NULL){	/*Find the command in the list*/

			storedCommand = (ytShellStoredCommand_t*) currentCommand->item;

			if(strcmp(storedCommand->commandName, commandName) == 0){	/*Command already exist*/

				osMutexRelease(ytShellListMutex);	/*Unlock the command list*/
				ytShellOpRunning--;
				return RET_ERROR;		/*Do not register duplicated commands*/
			}

			currentCommand = currentCommand->next;
		}
		/*Create the new command*/
		storedCommand = (ytShellStoredCommand_t*) pvPortMalloc(sizeof(ytShellStoredCommand_t));
		storedCommand->commandName =(char*) pvPortMalloc(strlen(commandName));
		memcpy(storedCommand->commandName, commandName, strlen(commandName));
		storedCommand->commandFunc = ytShellFunc;
		/*Store it in the commands list*/
		genListAdd(ytShellCommandsList, (void*) storedCommand);

		osMutexRelease(ytShellListMutex);	/*Unlock the command list*/
		ytShellOpRunning--;
		return RET_OK;
	}
	ytShellOpRunning--;
	return RET_ERROR;

}

/**
 *
 * @param commandName
 * @return
 */
retval_t ytShellUnregisterCommand(char* commandName){
	genListElement_t* currentCommand;
	ytShellStoredCommand_t* storedCommand;
	ytShellOpRunning++;
	if(ytShellInitialized){
		osMutexWait(ytShellListMutex, osWaitForever);/*Lock the command list*/
		currentCommand = ytShellCommandsList->tailElement;
		while(currentCommand != NULL){	/*Find the command in the list*/

			storedCommand = (ytShellStoredCommand_t*) currentCommand->item;

			if(strcmp(storedCommand->commandName, commandName) == 0){	/*Command found*/

				genListRemove(ytShellCommandsList, currentCommand);
				/*Free command allocated memory*/
				vPortFree(storedCommand->commandName);
				vPortFree(storedCommand);

				osMutexRelease(ytShellListMutex);	/*Unlock the command list*/
				ytShellOpRunning--;
				return RET_OK;
			}

			currentCommand = currentCommand->next;
		}
		osMutexRelease(ytShellListMutex);	/*Unlock the command list*/
	}
	ytShellOpRunning--;		/*Command not found*/
	return RET_ERROR;
}

/**
 *
 * @param argument
 */
static void ytShellTask(void const * argument){

	char** argv;
	uint32_t argc;
	genListElement_t* currentCommand;
	ytShellStoredCommand_t* storedCommand;
	uint8_t* lineBuff = pvPortMalloc(YETSHELL_LINE_BUFFER_SIZE);
	uint16_t commandFound = 0;
	uint16_t i;

	while(1){
		if(endTaskFlag){		/*Securely close this thread*/
			endTaskFlag = 0;
			vPortFree(lineBuff);
			osThreadTerminate(ytShellTaskHandle);
		}

		*lineBuff = '\0';	/*Mark the first character*/
		if(ytStdioReadLine(lineBuff, YETSHELL_LINE_BUFFER_SIZE) == RET_OK){	/*If the stdio is not running it will return ERROR*/

			if(*lineBuff != '\0'){	/*If the first character is still end of string, nothing has been read*/

				/*Memory for argv is reserved inside str_split function. It is necessary to free it later*/
				argv = strSplit((char*)lineBuff, ' ', &(argc));

				/*A value of 0 in argc should not be possible due to strSplit implementation. However maintain this check in case other implementations are used */
				if(argc > 0){
					commandFound = 0;
					osMutexWait(ytShellListMutex, osWaitForever);/*Lock the command list*/
					currentCommand = ytShellCommandsList->tailElement;
					while(currentCommand != NULL){	/*Find the received command in the list*/

						storedCommand = (ytShellStoredCommand_t*) currentCommand->item;

						if(strcmp(storedCommand->commandName, argv[0]) == 0){
							if(storedCommand->commandFunc != NULL){
								storedCommand->commandFunc(argc, argv);	/*Execute the command function if the command is found*/
								commandFound++;
								break;
							}
						}

						currentCommand = currentCommand->next;
					}
					osMutexRelease(ytShellListMutex);	/*Unlock the command list*/
					if(!commandFound){
						ytPrintf("Command not found\r\n");
					}
					ytPrintf(">");
				}

				/*Free argv Memory previously reserved*/
				for (i=0; i<argc; i++){
					vPortFree(argv[i]);
				}
				vPortFree(argv);
			}

		}
		else{
			osDelay(5);
		}

	}

}


static void printCommandsListCmd(uint32_t argc, char** argv){
	genListElement_t* currentCommand;
	ytShellStoredCommand_t* storedCommand;
	currentCommand = ytShellCommandsList->tailElement;
	while(currentCommand != NULL){	/*Loop the commands list*/

		storedCommand = (ytShellStoredCommand_t*) currentCommand->item;
		ytPrintf("-%s\r\n", storedCommand->commandName);

		currentCommand = currentCommand->next;
	}
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
static void availableMemCmd(uint32_t argc, char** argv){
	size_t freeMEM = xPortGetFreeHeapSize();
	ytPrintf( "-Available Memory: %.3f KB\r\n", (float32_t) ((float32_t)freeMEM/1000));
}

/**
 *
 * @param str
 * @param delim
 * @param numSplits
 * @return
 */
static char** strSplit( char* str, char delim, uint32_t* numSplits ){
	char** ret;
	uint32_t retLen;
	char* c = str;
	uint32_t prevDelim = 0;
	uint32_t i = 0;
	char *strStart = c;
	uint32_t strlen = 0;
	uint32_t j = 0;

	retLen = 0;

	/* Pre-calculate number of elements */
	do
	{
		if ( (*c == delim)  && (prevDelim == 0))
		{
			prevDelim = 1;
		}
		if ( (*c != delim)  && (prevDelim == 1)){
			prevDelim = 0;
			retLen++;
		}

		c++;
	} while ( *c != '\0' );

	retLen++;

	ret = pvPortMalloc( ( retLen ) * sizeof( *ret ) );

	c = str;

	for (i=0; i<retLen;i++){

LOOP:
		while(*c != delim){
			c++;
			strlen ++;
		}

		if(strlen != 0){
			ret[i] = pvPortMalloc( ( strlen+1) * sizeof(char) );
			for(j=0; j<strlen;j++){
				ret[i][j] = strStart[j];
			}
			ret[i][j] = '\0';
		}
		else{
			c++;
			strlen = 0;
			strStart = c;
			goto LOOP;
		}

		c++;
		strlen = 0;
		strStart = c;
	}

	*numSplits = retLen;
	return ret;

}

#endif
