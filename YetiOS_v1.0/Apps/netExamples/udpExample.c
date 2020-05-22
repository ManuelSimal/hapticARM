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
 * udpExample.c
 *
 *  Created on: 15 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file udpExample.c
 */

#include "yetiOS.h"
#include "netYetiOS.h"

#if UDP_EXAMPLE_APP

#define NODE_1		1
//#define NODE_2		2

#define UDP_TEST_PORT	0xAAAA

#define DATA_SIZE	24

static uint8_t testBuff[DATA_SIZE];

static osThreadId udpExampleThreadHandle;

static void UdpExampleFunc(void const * argument);

YtAutoInitThread(UdpExampleApp, UdpExampleFunc, osPriorityLow, 256, &udpExampleThreadHandle, NULL);

static void UdpExampleFunc(void const * argument){

	osDelay(1000);
#ifdef NODE_1
	memcpy(testBuff, "TEST_PACKET", strlen("TEST_PACKET") + 1);	/*Copy also the /0 */
	netAddr_t addr = ytNetNewAddr("1", 'D');
#endif
#ifdef NODE_2
	netAddr_t addr = ytNetNewAddr("2", 'D');
	char rcvAddrStr[8];
#endif
	ytNetAddNodeAddr(addr);
	ytNetDeleteNetAddr(addr);

	addr = ytNetNewAddr("2", 'D');

	if(ytUdpOpen(UDP_TEST_PORT) != RET_OK){
		ytLedOn(LED_RED_1);
		while(1){
			osDelay(1000);
		}
	}

	while(1){
#ifdef NODE_1
		osDelay(200);
		while(ytUdpSend(addr, UDP_TEST_PORT,testBuff, DATA_SIZE) == 0){
			osDelay(2);
		}
		ytLedToggle(LED_BLUE_1);
#endif
#ifdef NODE_2
		if (ytUdpRcv(addr, UDP_TEST_PORT, testBuff, DATA_SIZE) != 0){
			ytNetAddrToString(addr, rcvAddrStr, 'D');
#if YETIOS_ENABLE_STDIO
			ytPrintf("Received: %s from %s\r\n", testBuff, rcvAddrStr);
#endif
			ytLedToggle(LED_BLUE_1);
		}
#endif

	}

}

#endif

