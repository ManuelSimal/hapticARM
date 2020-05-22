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
 * yetiOsConf.h
 *
 *  Created on: 13 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file yetiOsConf.h
 * @brief Platform Independent YetiOS Configuration
 */

#ifndef YETIOS_CORE_YETIOSCONF_H_
#define YETIOS_CORE_YETIOSCONF_H_


#include "appConfig.h"			/* This file selects the config for the current app */

/* Blinking Period (ms) for the Status LED. A value of 0 disables the status LED*/
#ifndef YETIOS_STATUS_LED_PERIOD
#define	YETIOS_STATUS_LED_PERIOD	2000
#endif

/* The time (ms) the status LED is on each blinking period. A value of 0 disables the status LED */
#ifndef YETIOS_STATUS_LED_TIME_ON
#define	YETIOS_STATUS_LED_TIME_ON	40
#endif

/* Default Period of the YetiOS core thread. The longer it is the higher the consumption will be.*/
#ifndef YETIOS_DEFAULT_CORE_PERIOD
#define YETIOS_DEFAULT_CORE_PERIOD	10000
#endif

/*STDIO and YetiShell Defines*/
#ifndef PLATFORM_DEFAULT_STDIO_INTERFACE		/*An stdio interface must be defined by the platform to use the STDIO*/
#ifdef YETIOS_ENABLE_STDIO
#undef YETIOS_ENABLE_STDIO
#endif
#else
#ifndef YETIOS_ENABLE_STDIO
#define	YETIOS_ENABLE_STDIO				1
#endif
#endif

#if YETIOS_ENABLE_STDIO						/*The STDIO must be enabled to use the yetishell*/
#ifndef YETIOS_ENABLE_YETISHELL
#define YETIOS_ENABLE_YETISHELL			1
#endif
#endif

#ifndef STDIO_READ_LINE_ECHO_ENABLED
#define STDIO_READ_LINE_ECHO_ENABLED	1
#endif

#ifndef	YETIOS_STDIO_READ_BUFF_SIZE
#define YETIOS_STDIO_READ_BUFF_SIZE		256	/*Maximum Stdio throughput depends on the buffer size and the read task period*/
#endif

#ifndef	YETIOS_STDIO_TASK_PERIOD
#define  YETIOS_STDIO_TASK_PERIOD		5	/*Higher Buffer sizes and lower read periods increases the throughput*/
#endif

#ifndef YETIOS_PRINTF_BUFFER_SIZE
#define YETIOS_PRINTF_BUFFER_SIZE		256
#endif

#ifndef	YETSHELL_LINE_BUFFER_SIZE
#define YETSHELL_LINE_BUFFER_SIZE		128	/*This buffer should be lower or equal to STDIO READ Buffer size*/
#endif

#ifndef YETIOS_ENABLE_ADAPTIVE_ENGINE
#define YETIOS_ENABLE_ADAPTIVE_ENGINE	1
#endif

#ifndef	YETIOS_ENABLE_NETSTACK
#define YETIOS_ENABLE_NETSTACK			1
#endif

#if YETIOS_ENABLE_NETSTACK
	#ifndef YETIOS_PHY_DEFAULT_LAYER
	#define	YETIOS_PHY_DEFAULT_LAYER			PLATFORM_PHY_DEFAULT_LAYER
	#endif

	#ifndef YETIOS_MAC_DEFAULT_LAYER
	#define	YETIOS_MAC_DEFAULT_LAYER			basicYetiMacLayer
	#endif

	#ifndef YETIOS_ROUTING_DEFAULT_LAYER
	#define	YETIOS_ROUTING_DEFAULT_LAYER		yetiRoutingLayer
	#endif

	#ifndef YETIOS_TRANSPORT_DEFAULT_LAYER
	#define	YETIOS_TRANSPORT_DEFAULT_LAYER		yetiTransportLayer
	#endif

	#ifndef PHY_LAYER_PROCESS_PRIORITY
	#define	PHY_LAYER_PROCESS_PRIORITY			osPriorityHigh
	#endif
#endif


#endif /* YETIOS_CORE_YETIOSCONF_H_ */
