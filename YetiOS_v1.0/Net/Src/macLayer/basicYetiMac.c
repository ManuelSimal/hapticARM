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
 * basicYetiMac.c
 *
 *  Created on: 23 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file basicYetiMac.c
 */

#include "netstack.h"
#include "yetiOS.h"

#define DEFAULT_NODE_MAC_ADDR			0xB105B105
#define FLOOD_PACKET_RESEND_TIMEOUT		1000				/*Resending flood packets is not allowed for this time*/

#define FLOOD_MAX_RANDOM_DELAY			100


typedef struct __attribute__((__packed__))  basicYetiMacHdr_{
	uint32_t destMacAddr;
	uint32_t srcMacAddr;
}basicYetiMacHdr_t;

typedef uint32_t basicYetiMacAddr_t;


static basicYetiMacAddr_t basicYetiMacNodeAddr = DEFAULT_NODE_MAC_ADDR;
static osMutexId basicYetiMacMutex = NULL;
static uint16_t basicYetiMacStarted = 0;

static osTimerId floodDisableTimer;
static uint16_t floodPacketsDisabled = 0;

static osTimerId floodResendTimer;
static netPacket_t* reSendPacket = NULL;

static uint16_t macOpRunning = 0;

/* ********* MAC LAYER OPS ***********/
static retval_t basicYetiMacLayerInit(void);
static retval_t basicYetiMacLayerDeInit(void);
static retval_t basicYetiMacSendPacket(macAddr_t macAddrDest, netPacket_t* packet);
static retval_t basicYetiMacRcvPacket(void);
static retval_t basicYetiMacPacketSent(netPacket_t* packet, uint16_t size);
static retval_t basicYetiMacPacketReceived(netPacket_t* packet, uint16_t size);
static retval_t basicYetiMacSetAddr(macAddr_t macAddr);
static macAddr_t basicYetiMacNewAddr(char* macAddrStr, char format);
static retval_t basicYetiMacDeleteAddr(macAddr_t macAddr);
static retval_t basicYetiMacAddrToString(macAddr_t macAddr, char* macAddrStr, char format);
static retval_t basicYetiMacAddrCpy(macAddr_t destAddr, macAddr_t fromAddr);
static uint16_t basicYetiMacAddrCmp(macAddr_t aAddr, macAddr_t bAddr);
static retval_t basicYetiMacSetNodeAsGw(void);
static uint16_t basicYetiMacIsNodeLinked(void);
static retval_t basicYetiMacSetDutyCycle(uint32_t dutyCycle);
static retval_t basicYetiMacSetProcessPriority(osPriority priority);
static osPriority basicYetiMacGetProcessPriority(void);

/* ********* AUX FUNCS ***********/
static void floodDisableCbFunc(void const * argument);
static void floodResendCbFunc(void const * argument);

macLayerOps_t basicYetiMacLayer = {
	.macLayerInit = basicYetiMacLayerInit,
	.macLayerDeInit = basicYetiMacLayerDeInit,
	.macSendPacket = basicYetiMacSendPacket,
	.macRcvPacket = basicYetiMacRcvPacket,
	.macPacketSent = basicYetiMacPacketSent,
	.macPacketReceived = basicYetiMacPacketReceived,
	.macSetAddr = basicYetiMacSetAddr,
	.macNewAddr = basicYetiMacNewAddr,
	.macDeleteAddr = basicYetiMacDeleteAddr,
	.macAddrToString = basicYetiMacAddrToString,
	.macAddrCpy = basicYetiMacAddrCpy,
	.macAddrCmp = basicYetiMacAddrCmp,
	.macSetNodeAsGw = basicYetiMacSetNodeAsGw,
	.macIsNodeLinked = basicYetiMacIsNodeLinked,
	.macSetDutyCycle = basicYetiMacSetDutyCycle,
	.macSetProcessPriority = basicYetiMacSetProcessPriority,
	.macGetProcessPriority = basicYetiMacGetProcessPriority,
};




/**
 *
 * @return
 */
static retval_t basicYetiMacLayerInit(void){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	basicYetiMacStarted++;
	basicYetiMacMutex = ytMutexCreate();
#if PLATFORM_USE_32_BITS_UNIQUE_ID
	basicYetiMacNodeAddr = ytGetUniqueId32Bits();
#else
	basicYetiMacNodeAddr = DEFAULT_NODE_MAC_ADDR;
#endif
	floodPacketsDisabled = 0;
	reSendPacket = NULL;
	setMacHdrSize(rxPacketbuffer, (uint16_t) sizeof(basicYetiMacHdr_t));
	setMacHdrSize(txPacketbuffer, (uint16_t) sizeof(basicYetiMacHdr_t));

	floodDisableTimer = ytTimerCreate(osTimerOnce, floodDisableCbFunc, NULL);
	floodResendTimer = ytTimerCreate(osTimerOnce, floodResendCbFunc, NULL);

	macOpRunning--;
	taskEXIT_CRITICAL();

	netstack.phyLayerOps->phySetModeReceiving();	/*Always receive packets*/

	return RET_OK;

}

/**
 *
 * @return
 */
static retval_t basicYetiMacLayerDeInit(void){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	basicYetiMacStarted = 0;

	while(macOpRunning){								/*Wait until all operations are finished before closing*/
		taskEXIT_CRITICAL();
		osDelay(1);
		taskENTER_CRITICAL();
	}

	osTimerStop(floodDisableTimer);
	osTimerDelete(floodDisableTimer);
	osTimerStop(floodResendTimer);
	osTimerDelete(floodResendTimer);
	if(reSendPacket != NULL){
		packetbufferReleasePacket(txPacketbuffer, reSendPacket);
		reSendPacket = NULL;
	}
	osMutexDelete(basicYetiMacMutex);

	taskEXIT_CRITICAL();

	return RET_OK;
}

/**
 *
 * @param macAddrDest
 * @param packet
 * @return
 */
static retval_t basicYetiMacSendPacket(macAddr_t macAddrDest, netPacket_t* packet){
	retval_t ret = RET_ERROR;
	basicYetiMacAddr_t yetiDestMacAddr;
	basicYetiMacHdr_t* basicYetiMacHdr;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	macOpRunning++;
	taskEXIT_CRITICAL();
	osMutexWait(basicYetiMacMutex, osWaitForever);

	yetiDestMacAddr = *((basicYetiMacAddr_t*) macAddrDest);
	basicYetiMacHdr = (basicYetiMacHdr_t*) getPacketMacHeaderPtr(packet);

	if(yetiDestMacAddr == MAC_FLOOD_ADDR){
		floodPacketsDisabled++;
		osTimerStart (floodDisableTimer, FLOOD_PACKET_RESEND_TIMEOUT);
	}
	osMutexRelease(basicYetiMacMutex);
	basicYetiMacHdr->destMacAddr = yetiDestMacAddr;
	basicYetiMacHdr->srcMacAddr = basicYetiMacNodeAddr;
	packet->macExtraInfo = (void*)0;	/*Use this flag to indicate that this packet is (1) or not (0) a resend packet*/
	ret = netstack.phyLayerOps->phySendPacket(packet, FIXED_PACKET_SIZE);
	macOpRunning--;
	return ret;
}

/**
 *
 * @return
 */
static retval_t basicYetiMacRcvPacket(void){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	macOpRunning++;
	taskEXIT_CRITICAL();

	ret = netstack.phyLayerOps->phySetModeReceiving();

	macOpRunning--;
	return ret;
}


/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t basicYetiMacPacketSent(netPacket_t* packet, uint16_t size){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	macOpRunning++;
	taskEXIT_CRITICAL();

	if(!((uint32_t)packet->macExtraInfo)){
		ret = netstack.routingLayerOps->rtPacketSent(packet);
	}
	else{/*If it is a resend packet, release it*/
		ret = packetbufferReleasePacket(txPacketbuffer, packet);
	}
	ret = netstack.phyLayerOps->phySetModeReceiving();

	macOpRunning--;

	return ret;
}


/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t basicYetiMacPacketReceived(netPacket_t* packet, uint16_t size){
	retval_t ret = RET_ERROR;
	basicYetiMacHdr_t* basicYetiMacHdr;
	basicYetiMacAddr_t basicYetiMacFromAddr;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	macOpRunning++;
	taskEXIT_CRITICAL();

	netstack.phyLayerOps->phySetModeReceiving();
	basicYetiMacHdr = (basicYetiMacHdr_t*) getPacketMacHeaderPtr(packet);

	/*Check if the packet is for me. If it is process it*/
	if((basicYetiMacHdr->destMacAddr == MAC_FLOOD_ADDR)){	/*If it is a flood message try to resend*/
		basicYetiMacFromAddr = basicYetiMacHdr->srcMacAddr;
		ret = netstack.routingLayerOps->rtPacketReceived(packet, (macAddr_t) &basicYetiMacFromAddr);
		osMutexWait(basicYetiMacMutex, osWaitForever);
		if((!floodPacketsDisabled) && (reSendPacket == NULL)){	/*When a flood packet is sent no more resends are allowed until the timeout is reached*/
			if((reSendPacket = packetbufferGetFreePacket(txPacketbuffer))!= NULL){
				basicYetiMacHdr->srcMacAddr = basicYetiMacNodeAddr;
				basicYetiMacHdr->destMacAddr = MAC_FLOOD_ADDR;
				memcpy(reSendPacket, packet, FIXED_PACKET_SIZE);	/*Copy the packet to be sent*/
				reSendPacket->macExtraInfo = (void*)1;		/*Mark this packet as resend packet*/
				floodPacketsDisabled++;
				osTimerStart (floodDisableTimer, FLOOD_PACKET_RESEND_TIMEOUT);	/*Disable resending more packets for the timeout*/

				osTimerStart(floodResendTimer, ((uint32_t)rand())%FLOOD_MAX_RANDOM_DELAY);	/*Random delay before resend to prevent packet colisions*/
			}
		}
		osMutexRelease(basicYetiMacMutex);
	}
	else if((basicYetiMacHdr->destMacAddr == MAC_BROADCAST_ADDR) || (basicYetiMacHdr->destMacAddr == basicYetiMacNodeAddr)){/*To the routing layer*/
		basicYetiMacFromAddr = basicYetiMacHdr->srcMacAddr;
		ret = netstack.routingLayerOps->rtPacketReceived(packet, (macAddr_t) &basicYetiMacFromAddr);
	}
	macOpRunning--;

	return ret;
}


/**
 *
 * @param macAddr
 * @return
 */
static retval_t basicYetiMacSetAddr(macAddr_t macAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	macOpRunning++;
	taskEXIT_CRITICAL();
	osMutexWait(basicYetiMacMutex, osWaitForever);
	basicYetiMacNodeAddr = *((basicYetiMacAddr_t*)macAddr);
	osMutexRelease(basicYetiMacMutex);

	macOpRunning--;
	return RET_OK;
}

/**
 *
 * @param macAddrStr
 * @param format
 * @return
 */
static macAddr_t basicYetiMacNewAddr(char* macAddrStr, char format){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return NULL;
	}
	taskEXIT_CRITICAL();
	basicYetiMacAddr_t* newMacAddr = NULL;
	uint32_t addrVal;

	if(format == '0'){	//Empty new address
		newMacAddr = (basicYetiMacAddr_t*) pvPortMalloc(sizeof(basicYetiMacAddr_t));
		*newMacAddr = 0;
	}

	else{
		if((format == 'D') || (format == 'd')){
			if(strlen(macAddrStr)>10){	/*Max addr value 4294967296*/
				return NULL;
			}
			addrVal = strtoul(macAddrStr, NULL, 10);

			newMacAddr = (basicYetiMacAddr_t*) pvPortMalloc(sizeof(basicYetiMacAddr_t));
			*newMacAddr = addrVal;
		}
		else if((format == 'H') || (format == 'h')){
			if(strlen(macAddrStr)>10){	/*Max addr value 0xFFFFFFFF*/
				return NULL;
			}
			if((macAddrStr[0] != '0') || (macAddrStr[1] != 'x')){
				return NULL;
			}
			addrVal = strtoul(macAddrStr+2, NULL, 16);

			newMacAddr = (basicYetiMacAddr_t*) pvPortMalloc(sizeof(basicYetiMacAddr_t));
			*newMacAddr = addrVal;
		}
		else{
			return NULL;
		}

	}
	return newMacAddr;
}

/**
 *
 * @param macAddr
 * @return
 */
static retval_t basicYetiMacDeleteAddr(macAddr_t macAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	taskEXIT_CRITICAL();
	vPortFree(macAddr);
	return RET_OK;
}

/**
 *
 * @param macAddr
 * @param macAddrStr
 * @param format
 * @return
 */
static retval_t basicYetiMacAddrToString(macAddr_t macAddr, char* macAddrStr, char format){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	taskEXIT_CRITICAL();
	basicYetiMacAddr_t* yetiMacAddr = (basicYetiMacAddr_t*) macAddr;
	if((format == 'D') || (format == 'd')){
		sprintf(macAddrStr, "%u", (unsigned int) *yetiMacAddr);
	}
	else if((format == 'H') || (format == 'h')){
		sprintf(macAddrStr, "0x%08X",(unsigned int) *yetiMacAddr);
	}
	else{
		macAddrStr[0] = 'N';
		macAddrStr[1] = 'a';
		macAddrStr[2] = 'N';
		return RET_ERROR;
	}

	return RET_OK;
}

/**
 *
 * @param destAddr
 * @param fromAddr
 * @return
 */
static retval_t basicYetiMacAddrCpy(macAddr_t destAddr, macAddr_t fromAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	taskEXIT_CRITICAL();

	basicYetiMacAddr_t* yetiDestAddr = (basicYetiMacAddr_t*) destAddr;
	basicYetiMacAddr_t* yetiFromAddr = (basicYetiMacAddr_t*) fromAddr;

	*yetiDestAddr = *yetiFromAddr;
	return RET_OK;
}

/**
 *
 * @param aAddr
 * @param bAddr
 * @return
 */
static uint16_t basicYetiMacAddrCmp(macAddr_t aAddr, macAddr_t bAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	taskEXIT_CRITICAL();

	if((*((basicYetiMacAddr_t*)aAddr)) == (*((basicYetiMacAddr_t*)bAddr))){
		return 1;
	}
	return 0;
}

/**
 *
 * @return
 */
static retval_t basicYetiMacSetNodeAsGw(void){
	return RET_OK;
}

/**
 *
 * @return
 */
static uint16_t basicYetiMacIsNodeLinked(void){
	return 1;
}

/**
 *
 * @param dutyCycle
 * @return
 */
static retval_t basicYetiMacSetDutyCycle(uint32_t dutyCycle){
	return RET_ERROR;
}

/**
 *
 * @param priority
 * @return
 */
static retval_t basicYetiMacSetProcessPriority(osPriority priority){
	return RET_ERROR;
}

/**
 *
 * @return
 */
static osPriority basicYetiMacGetProcessPriority(void){
	return osPriorityError;
}

/**
 *
 * @param argument
 */
static void floodDisableCbFunc(void const * argument){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return;
	}
	macOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(basicYetiMacMutex, osWaitForever);
	floodPacketsDisabled = 0;
	osMutexRelease(basicYetiMacMutex);

	macOpRunning--;
}

/**
 *
 * @param argument
 */
static void floodResendCbFunc(void const * argument){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!basicYetiMacStarted){
		taskEXIT_CRITICAL();
		return;
	}
	macOpRunning++;
	taskEXIT_CRITICAL();


	osMutexWait(basicYetiMacMutex, osWaitForever);
	netPacket_t* tempReSendPacket = reSendPacket;
	reSendPacket = NULL;
	osMutexRelease(basicYetiMacMutex);

	if(netstack.phyLayerOps->phySendPacket(tempReSendPacket, FIXED_PACKET_SIZE) != RET_OK){
		packetbufferReleasePacket(txPacketbuffer, tempReSendPacket);
	}

	macOpRunning--;
}
