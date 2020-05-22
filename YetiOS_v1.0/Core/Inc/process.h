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
 * process.h
 *
 *  Created on: 15 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file process.h
 */

#include "types.h"

#ifndef YETIOS_CORE_INC_PROCESS_H_
#define YETIOS_CORE_INC_PROCESS_H_

/* **************Auto init process function *****************/

/**
 * @brief 					MACRO to create processes at startup
 * @param name				Process name. For memory saving avoid large names. (char*)
 * @param threadFunc		Function to be launched when the process starts. (ytProcessFunc_t) => void (*)(const void* args)
 * @param threadPriority	Process Priority.
 * @param stackSize			Amount of stack to reserve for this process. (uint32_t)
 * @param processHandle		Returned value of the process handle created. Use a pointer to NULL if is not desired to return the id.
 * @param arg				Argument to the process function. (void*)
 */
#define INIT_THREAD(name, threadFunc, threadPriority, stackSize, threadHandle, arg)\
	\
	static retval_t __init_process_##name(void){\
		return ytStartThread(#name, threadFunc, threadPriority, stackSize, threadHandle, arg);\
		}\
	static retval_t (*__initcall_process_##name)(void) __attribute__((__used__)) \
			__attribute__((__section__(".process_initcall"))) = __init_process_##name;\
			 void fooProcFunc##name(void){ __initcall_process_##name();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/



retval_t ytAutoInitThreads(void);
#endif /* YETIOS_CORE_INC_PROCESS_H_ */
