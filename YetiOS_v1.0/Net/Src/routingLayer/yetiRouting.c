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
 * yetiRouting.c
 *
 *  Created on: 20 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file yetiRouting.c
 */


#include "yetiOS.h"
#include "netstack.h"
#include <string.h>


#define NB_DISCOVERY_PERIOD		6000
#define ROUTE_DISCOVERY_PERIOD	8000


#define PACKET_TYPE_RT_DEFAULT	0x00
#define PACKET_TYPE_NB_DISC		0x01
#define PACKET_TYPE_RT_DISC		0x02

typedef uint16_t yetiNetAddr_t;

typedef struct __attribute__((__packed__))  yetiRoutingHdr_{
	uint16_t destAddr;
	uint16_t srcAddr;
	uint8_t rtPacketType;
}yetiRoutingHdr_t;

typedef struct yetiRoute_{
	yetiNetAddr_t destAddr;
	yetiNetAddr_t nextAddr;
	uint16_t numHops;
}yetiRoute_t;

typedef struct yetiNeighbour_{
	macAddr_t nbMacAddr;
	yetiNetAddr_t yetiNetAddr;
}yetiNeighbour_t;

static osMutexId yetiRoutingMutex = NULL;
static uint16_t yetiRoutingStarted = 0;

static yetiNetAddr_t nodeAddr;
static genList_t* routesList;
static genList_t* neighboursList;

static osTimerId nbDiscoveryTimer;
static osTimerId routeDiscoveryTimer;

static uint16_t routingOpRunning = 0;

/*Routing Layer functions*/
static retval_t yetiRtLayerInit(void);
static retval_t yetiRtLayerDeInit(void);
static retval_t yetiRtSendPacket(netAddr_t addr, netPacket_t* packet);
static retval_t yetiRtRcvPacket(void);
static retval_t yetiRtPacketSent(netPacket_t* packet);
static retval_t yetiRtPacketReceived(netPacket_t* packet, macAddr_t macFromAddr);
static netAddr_t yetiRtGetNodeAddr(uint16_t index);
static retval_t yetiRtAddNodeAddr(netAddr_t newAddr);
static retval_t yetiRtRemoveNodeAddr(netAddr_t addr);
static retval_t yetiRtAddRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops);
static retval_t yetiRtRemoveRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops);
static netAddr_t yetiRtNewNetAddr(char* addrStr, char format);
static retval_t yetiRtDeleteNetAddr(netAddr_t netAddr);
static retval_t yetiRtNetAddrToString(netAddr_t netAddr, char* addrStr, char format);
static retval_t yetiRtNetAddrCpy(netAddr_t destAddr, netAddr_t fromAddr);
static uint16_t yetiRtNetAddrCmp(netAddr_t aAddr, netAddr_t bAddr);
static retval_t yetiRtSetNodeAsGw(void);

/*Local Functions*/
static void nbDiscoveryCbFunc(void const * argument);
static void routeDiscoveryCbFunc(void const * argument);
static retval_t getNextAddrFromDestRoute(yetiNetAddr_t destAddr, yetiNetAddr_t* nextAddr);
static retval_t yetiAddNeighbour(yetiNetAddr_t nbNetAddr, macAddr_t nbMacAddr);
static macAddr_t getNeighbourMacAddr(yetiNetAddr_t netAddr);
static uint16_t checkExistingMacInNbTable(macAddr_t nbMacAddr);
static uint16_t checkExistingNetAddrInNbTable(yetiNetAddr_t nbNetAddr);


routingLayerOps_t yetiRoutingLayer ={
		.rtLayerInit = yetiRtLayerInit,
		.rtLayerDeInit = yetiRtLayerDeInit,
		.rtSendPacket = yetiRtSendPacket,
		.rtRcvPacket = yetiRtRcvPacket,
		.rtPacketSent = yetiRtPacketSent,
		.rtPacketReceived = yetiRtPacketReceived,
		.rtGetNodeAddr = yetiRtGetNodeAddr,
		.rtAddNodeAddr = yetiRtAddNodeAddr,
		.rtRemoveNodeAddr = yetiRtRemoveNodeAddr,
		.rtAddRoute = yetiRtAddRoute,
		.rtRemoveRoute = yetiRtRemoveRoute,
		.rtNewNetAddr = yetiRtNewNetAddr,
		.rtDeleteNetAddr = yetiRtDeleteNetAddr,
		.rtNetAddrToString = yetiRtNetAddrToString,
		.rtNetAddrCpy = yetiRtNetAddrCpy,
		.rtNetAddrCmp = yetiRtNetAddrCmp,
		.rtSetNodeAsGw = yetiRtSetNodeAsGw,
};



/*Routing Layer functions*/

/**
 *
 * @return
 */
static retval_t yetiRtLayerInit(void){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	yetiRoutingStarted++;
	nodeAddr = 0;
	yetiRoutingMutex = ytMutexCreate();

	routesList = genListInit();
	neighboursList = genListInit();

	setRoutingHdrSize(rxPacketbuffer, (uint16_t) sizeof(yetiRoutingHdr_t));
	setRoutingHdrSize(txPacketbuffer, (uint16_t) sizeof(yetiRoutingHdr_t));
	nbDiscoveryTimer = ytTimerCreate(osTimerPeriodic, nbDiscoveryCbFunc, NULL);
	routeDiscoveryTimer = ytTimerCreate(osTimerPeriodic, routeDiscoveryCbFunc, NULL);

	osTimerStart (nbDiscoveryTimer, NB_DISCOVERY_PERIOD);
	osTimerStart (routeDiscoveryTimer, ROUTE_DISCOVERY_PERIOD);
	routingOpRunning--;

	taskEXIT_CRITICAL();

	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t yetiRtLayerDeInit(void){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	yetiRoutingStarted = 0;

	while(routingOpRunning){		/*Wait until all operations are finished before closing*/
		taskEXIT_CRITICAL();
		osDelay(1);
		taskENTER_CRITICAL();
	}

	osMutexWait(yetiRoutingMutex, osWaitForever);
	osTimerStop (nbDiscoveryTimer);
	osTimerStop (routeDiscoveryTimer);
	osTimerDelete (nbDiscoveryTimer);
	osTimerDelete (routeDiscoveryTimer);
	genListRemoveAndDeleteAll(neighboursList);
	genListRemoveAndDeleteAll(routesList);
	osMutexDelete(yetiRoutingMutex);

	taskEXIT_CRITICAL();
	return RET_OK;
}

/**
 *
 * @param addr
 * @param packet
 * @return
 */
static retval_t yetiRtSendPacket(netAddr_t addr, netPacket_t* packet){
	yetiNetAddr_t destAddr = *((yetiNetAddr_t*) addr);
	yetiRoutingHdr_t* routingHeader;
	yetiNetAddr_t nextAddr;
	macAddr_t destMacAddr;
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();

	if(getNextAddrFromDestRoute(destAddr, &nextAddr) != RET_OK){	/*Look for the route to the destination in the route list*/
		routingOpRunning--;
		return ret;
	}
	else{	/*Next hop address found in the route table*/
		routingHeader = (yetiRoutingHdr_t*) getPacketRoutingHeaderPtr(packet);
		routingHeader->srcAddr = nodeAddr;
		routingHeader->destAddr = destAddr;
		routingHeader->rtPacketType = PACKET_TYPE_RT_DEFAULT;

		if(nextAddr == RT_BROADCAST_ADDR){
			destMacAddr = netstack.macLayerOps->macNewAddr(MAC_FLOOD_ADDR_STR, 'H');
		}
		else{
			if((destMacAddr = getNeighbourMacAddr(nextAddr)) == NULL){	/*Look for a the MAC address of next node in the negihbour table*/
				routingOpRunning--;
				return ret;
			}
		}

		ret = netstack.macLayerOps->macSendPacket(destMacAddr, packet);

		if(destAddr == RT_BROADCAST_ADDR){	/*In this case a broadcast mac address has been created, so it has to be removed*/
			netstack.macLayerOps->macDeleteAddr(destMacAddr);
		}

	}

	routingOpRunning--;
	return ret;
}

/**
 *
 * @return
 */
static retval_t yetiRtRcvPacket(void){
	retval_t ret;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();

	ret = netstack.macLayerOps->macRcvPacket();

	routingOpRunning--;
	return ret;
}

/**
 *
 * @param packet
 * @return
 */
static retval_t yetiRtPacketSent(netPacket_t* packet){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();
	retval_t ret = RET_ERROR;

	yetiRoutingHdr_t* routingHeader = (yetiRoutingHdr_t*) getPacketRoutingHeaderPtr(packet);
	if(routingHeader->rtPacketType  == PACKET_TYPE_RT_DEFAULT){	/*If I am the originator of a Default packet, raise it up to the transport layer*/
		osMutexWait(yetiRoutingMutex, osWaitForever);
		if (routingHeader->srcAddr == nodeAddr){
			osMutexRelease(yetiRoutingMutex);
			ret = netstack.transportLayerOps->transportPacketSent(packet);
		}
		else{	/*I am not the originator, it is a resend packet so free it*/
			osMutexRelease(yetiRoutingMutex);
			ret = packetbufferReleasePacket(txPacketbuffer, packet);
		}
	}
	else{	/*It is not a default packet, free it here*/
		ret = packetbufferReleasePacket(txPacketbuffer, packet);
	}

	routingOpRunning--;
	return ret;
}

/**
 *
 * @param packet
 * @param macFromAddr
 * @return
 */
static retval_t yetiRtPacketReceived(netPacket_t* packet, macAddr_t macFromAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();
	retval_t ret = RET_ERROR;
	yetiRoutingHdr_t* routingHeader = (yetiRoutingHdr_t*) getPacketRoutingHeaderPtr(packet);
	netPacket_t* reSendPacket;
	yetiNetAddr_t resendDestAddr, nextAddr;
	yetiNetAddr_t netFromAddr;
	macAddr_t destMacAddr;

	if(routingHeader->rtPacketType  == PACKET_TYPE_RT_DEFAULT){
		osMutexWait(yetiRoutingMutex, osWaitForever);
		if((nodeAddr == routingHeader->destAddr) || (routingHeader->destAddr == RT_BROADCAST_ADDR)){	/*The packet is for me (or it is broadcast)*/
			netFromAddr = routingHeader->srcAddr;
			if(nodeAddr != netFromAddr){	/*Dont raise the packet if I am the originator*/
				osMutexRelease(yetiRoutingMutex);
				ret = netstack.transportLayerOps->transportPacketReceived(packet, (netAddr_t) &netFromAddr);	/*Raise it to the upper layer*/
			}
			else{
				osMutexRelease(yetiRoutingMutex);
			}
			routingOpRunning--;
			return ret;
		}
		else{	/*The packet is not for me. Resend it*/
			if(nodeAddr == routingHeader->srcAddr){/*If I am the originator dont resend it*/
				osMutexRelease(yetiRoutingMutex);
				routingOpRunning--;
				return ret;
			}
			else{
				osMutexRelease(yetiRoutingMutex);
				resendDestAddr = routingHeader->destAddr;
				if((reSendPacket = packetbufferGetFreePacket(txPacketbuffer))!= NULL){				/*Get a packet to transmit*/
					/*Re send the packet*/
					if(getNextAddrFromDestRoute(resendDestAddr, &nextAddr) != RET_OK){	/*Look for the route to the destination in the route list*/
						packetbufferReleasePacket(txPacketbuffer, reSendPacket);
						routingOpRunning--;
						return ret;
					}
					else{	/*Next hop address found in the route table*/
						memcpy(reSendPacket, packet, FIXED_PACKET_SIZE);						/*Copy the received packet contents*/
						/*It is not necessary to fill the routing header, since it has been copied just before*/

						if(nextAddr == RT_BROADCAST_ADDR){
							destMacAddr = netstack.macLayerOps->macNewAddr(MAC_FLOOD_ADDR_STR, 'H');
						}
						else{
							if((destMacAddr = getNeighbourMacAddr(nextAddr)) == NULL){	/*Look for a the MAC address of next node in the negihbour table*/
								packetbufferReleasePacket(txPacketbuffer, reSendPacket);
								routingOpRunning--;
								return ret;
							}
						}
						if((ret = netstack.macLayerOps->macSendPacket(destMacAddr, reSendPacket)) != RET_OK){
							packetbufferReleasePacket(txPacketbuffer, reSendPacket);
						}
						if(nextAddr == RT_BROADCAST_ADDR){	/*In this case a broadcast mac address has been created, so it has to be removed*/
							netstack.macLayerOps->macDeleteAddr(destMacAddr);
						}
						routingOpRunning--;
						return ret;
					}
				}
				else{
					routingOpRunning--;
					return ret;
				}
			}

		}
	}

	else if(routingHeader->rtPacketType  == PACKET_TYPE_NB_DISC){
		if((ret = yetiAddNeighbour(routingHeader->srcAddr, macFromAddr)) != RET_OK){
			routingOpRunning--;
			return ret;
		}
		/*If it is a neighbour add also the direct route to this node*/
		netFromAddr = routingHeader->srcAddr;
		ret = yetiRtAddRoute((netAddr_t) &netFromAddr, (netAddr_t) &netFromAddr, 0);
	}

	else if(routingHeader->rtPacketType == PACKET_TYPE_RT_DISC){
		netFromAddr = routingHeader->srcAddr;
		uint8_t* dataPtr = getPacketDataPtr(packet);
		memcpy(&nextAddr, dataPtr, sizeof(uint16_t));	/*Get the next address from the data payload*/

		/*THE ROUTING Discovery packet is sent using the MAC Flood. Update the packet here so the mac layer could resend it adequately*/
		memcpy(dataPtr, &nodeAddr, sizeof(uint16_t));
		ret = yetiRtAddRoute((netAddr_t) &netFromAddr, (netAddr_t) &nextAddr, 0);

	}
	routingOpRunning--;
	return ret;

}

/**
 *
 * @param index
 * @return
 */
static netAddr_t yetiRtGetNodeAddr(uint16_t index){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return NULL;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();
	yetiNetAddr_t* retNodeAddr = (yetiNetAddr_t*) pvPortMalloc(sizeof(yetiNetAddr_t));
	osMutexWait(yetiRoutingMutex, osWaitForever);
	*retNodeAddr = nodeAddr;
	osMutexRelease(yetiRoutingMutex);

	routingOpRunning--;

	return (netAddr_t) retNodeAddr;
}

/**
 *
 * @param newAddr
 * @return
 */
static retval_t yetiRtAddNodeAddr(netAddr_t newAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();
	yetiNetAddr_t* newNodeAddr = (yetiNetAddr_t*) newAddr;
	osMutexWait(yetiRoutingMutex, osWaitForever);
	nodeAddr = *newNodeAddr;
	osMutexRelease(yetiRoutingMutex);

	routingOpRunning--;
	return RET_OK;
}

/**
 *
 * @param addr
 * @return
 */
static retval_t yetiRtRemoveNodeAddr(netAddr_t addr){
	return RET_ERROR;	/*Operation not allowed for this routing layer, since only one address per node is allowed*/
}

/**
 *
 * @param destAddr
 * @param nextAddr
 * @param hops
 * @return
 */
static retval_t yetiRtAddRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops){
	yetiNetAddr_t checkNextAddr;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();

	if(getNextAddrFromDestRoute(*((yetiNetAddr_t*)destAddr), &checkNextAddr) == RET_OK){	/*Check the route already exists in the list*/
		if(checkNextAddr == *((yetiNetAddr_t*)nextAddr)){
			routingOpRunning--;
			return RET_ERROR;
		}
	}

	yetiRoute_t* newRoute = (yetiRoute_t*)pvPortMalloc(sizeof(yetiRoute_t));
	newRoute->destAddr = *((yetiNetAddr_t*)destAddr);
	newRoute->nextAddr = *((yetiNetAddr_t*)nextAddr);
	newRoute->numHops = hops;

	osMutexWait(yetiRoutingMutex, osWaitForever);
	genListAdd(routesList, (void*)newRoute);
	osMutexRelease(yetiRoutingMutex);

	routingOpRunning--;
	return RET_OK;
}

/**
 *
 * @param destAddr
 * @param nextAddr
 * @param hops
 * @return
 */
static retval_t yetiRtRemoveRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(yetiRoutingMutex, osWaitForever);

	genListElement_t* current = routesList->tailElement;
	yetiRoute_t* yetiRoute;

	while(current != NULL){
		yetiRoute = (yetiRoute_t*) current->item;
		if(yetiRoute->destAddr == *((yetiNetAddr_t*)destAddr)){
			if(yetiRoute->nextAddr == *((yetiNetAddr_t*)nextAddr)){
				genListRemove(routesList, (void*) yetiRoute);
				vPortFree(yetiRoute);
				osMutexRelease(yetiRoutingMutex);

				routingOpRunning--;
				return RET_OK;
			}
		}
		current = current->next;
	}

	osMutexRelease(yetiRoutingMutex);

	routingOpRunning--;
	return RET_ERROR;
}

/**
 *
 * @param addrStr
 * @param format
 * @return
 */
static netAddr_t yetiRtNewNetAddr(char* addrStr, char format){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return NULL;
	}
	taskEXIT_CRITICAL();
	yetiNetAddr_t* newNetAddr = NULL;
	uint32_t addrVal;

	if(format == '0'){	/*Empty new address*/
		newNetAddr = (yetiNetAddr_t*) pvPortMalloc(sizeof(yetiNetAddr_t));
		*newNetAddr = 0x0000;
	}

	else{
		if((format == 'D') || (format == 'd')){
			if(strlen(addrStr)>5){	/*Max addr value 65535*/
				return NULL;
			}
			addrVal = strtoul(addrStr, NULL, 10);

			newNetAddr = (yetiNetAddr_t*) pvPortMalloc(sizeof(yetiNetAddr_t));
			*newNetAddr = (uint16_t)(addrVal & 0x0000FFFF);
		}
		else if((format == 'H') || (format == 'h')){
			if(strlen(addrStr)>6){	/*Max addr value 0xFFFF*/
				return NULL;
			}
			if((addrStr[0] != '0') || (addrStr[1] != 'x')){
				return NULL;
			}
			addrVal = strtoul(addrStr+2, NULL, 16);

			newNetAddr = (yetiNetAddr_t*) pvPortMalloc(sizeof(yetiNetAddr_t));
			*newNetAddr = (uint16_t)(addrVal & 0x0000FFFF);
		}
		else{
			return NULL;
		}

	}
	return newNetAddr;

}

/**
 *
 * @param netAddr
 * @return
 */
static retval_t yetiRtDeleteNetAddr(netAddr_t netAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	taskEXIT_CRITICAL();
	vPortFree((void*) netAddr);
	return RET_OK;
}


/**
 *
 * @param netAddr
 * @param addrStr
 * @param format
 * @return
 */
static retval_t yetiRtNetAddrToString(netAddr_t netAddr, char* addrStr, char format){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	taskEXIT_CRITICAL();
	yetiNetAddr_t* yetiNetAddr = (yetiNetAddr_t*) netAddr;
	if((format == 'D') || (format == 'd')){
		sprintf(addrStr, "%u", (unsigned int) *yetiNetAddr);
	}
	else if((format == 'H') || (format == 'h')){
		sprintf(addrStr, "0x%04X",(unsigned int) *yetiNetAddr);
	}
	else{
		addrStr[0] = 'N';
		addrStr[1] = 'a';
		addrStr[2] = 'N';
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
static retval_t yetiRtNetAddrCpy(netAddr_t destAddr, netAddr_t fromAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	taskEXIT_CRITICAL();

	yetiNetAddr_t* yetiDestAddr = (yetiNetAddr_t*) destAddr;
	yetiNetAddr_t* yetiFromAddr = (yetiNetAddr_t*) fromAddr;

	*yetiDestAddr = *yetiFromAddr;
	return RET_OK;
}

/**
 *
 * @param aAddr
 * @param bAddr
 * @return
 */
static uint16_t yetiRtNetAddrCmp(netAddr_t aAddr, netAddr_t bAddr){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	taskEXIT_CRITICAL();

	if((*((yetiNetAddr_t*)aAddr)) == (*((yetiNetAddr_t*)bAddr))){
		return 1;
	}
	return 0;
}

/**
 *
 * @return
 */
static retval_t yetiRtSetNodeAsGw(void){
	retval_t ret;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();

	ret = netstack.macLayerOps->macSetNodeAsGw();

	routingOpRunning--;
	return ret;
}

/**
 *
 * @param argument
 */
static void nbDiscoveryCbFunc(void const * argument){
	netPacket_t* packet;
	macAddr_t macAddr;
	yetiRoutingHdr_t* routingHeader;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();


	if((packet = packetbufferGetFreePacket(txPacketbuffer)) == NULL){/*Send Neighbour discovery packet*/
		routingOpRunning--;
		return;
	}
	if((macAddr = netstack.macLayerOps->macNewAddr(MAC_BROADCAST_ADDR_STR, 'H')) == NULL){	/*MAC broadcast address*/
		packetbufferReleasePacket(txPacketbuffer, packet);
		routingOpRunning--;
		return;
	}
	routingHeader = (yetiRoutingHdr_t*) getPacketRoutingHeaderPtr(packet);
	osMutexWait(yetiRoutingMutex, osWaitForever);
	routingHeader->srcAddr = nodeAddr;
	osMutexRelease(yetiRoutingMutex);
	routingHeader->destAddr = RT_BROADCAST_ADDR;
	routingHeader->rtPacketType  = (uint8_t) PACKET_TYPE_NB_DISC;
	if(netstack.macLayerOps->macSendPacket(macAddr, packet) != RET_OK){
		packetbufferReleasePacket(txPacketbuffer, packet);
	}
	netstack.macLayerOps->macDeleteAddr(macAddr);
	routingOpRunning--;
}

/**
 *
 * @param argument
 */
static void routeDiscoveryCbFunc(void const * argument){
	netPacket_t* packet;
	macAddr_t macAddr;
	yetiRoutingHdr_t* routingHeader;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiRoutingStarted){
		taskEXIT_CRITICAL();
		return;
	}
	routingOpRunning++;
	taskEXIT_CRITICAL();

	if((packet = packetbufferGetFreePacket(txPacketbuffer)) == NULL){	/*Send Route discovery packet*/
		routingOpRunning--;
		return;
	}
	if((macAddr = netstack.macLayerOps->macNewAddr(MAC_FLOOD_ADDR_STR, 'H')) == NULL){	/*MAC flood address*/
		packetbufferReleasePacket(txPacketbuffer, packet);
		routingOpRunning--;
		return;
	}
	routingHeader = (yetiRoutingHdr_t*) getPacketRoutingHeaderPtr(packet);
	osMutexWait(yetiRoutingMutex, osWaitForever);
	routingHeader->srcAddr = nodeAddr;
	osMutexRelease(yetiRoutingMutex);
	routingHeader->destAddr = RT_BROADCAST_ADDR;
	routingHeader->rtPacketType  = (uint8_t) PACKET_TYPE_RT_DISC;
	uint8_t* dataPtr = getPacketDataPtr(packet);
	memcpy(dataPtr, &(nodeAddr), sizeof(uint16_t));

	if(netstack.macLayerOps->macSendPacket(macAddr, packet) != RET_OK){
		packetbufferReleasePacket(txPacketbuffer, packet);
	}
	netstack.macLayerOps->macDeleteAddr(macAddr);

	routingOpRunning--;
}


/**
 *
 * @param destAddr
 * @param nextAddr
 * @return
 */
static retval_t getNextAddrFromDestRoute(yetiNetAddr_t destAddr, yetiNetAddr_t* nextAddr){

	if(destAddr == RT_BROADCAST_ADDR){
		*nextAddr = RT_BROADCAST_ADDR;
		return RET_OK;
	}
	osMutexWait(yetiRoutingMutex, osWaitForever);
	genListElement_t* current = routesList->tailElement;
	yetiRoute_t* yetiRoute;

	while(current != NULL){
		yetiRoute = (yetiRoute_t*) current->item;
		if(yetiRoute->destAddr == destAddr){

			*nextAddr = yetiRoute->nextAddr;
			osMutexRelease(yetiRoutingMutex);
			return RET_OK;
		}
		current = current->next;
	}
	osMutexRelease(yetiRoutingMutex);
	return RET_ERROR;
}

/**
 *
 * @param nbAddr
 * @param nbMacAddr
 * @return
 */
static retval_t yetiAddNeighbour(yetiNetAddr_t nbNetAddr, macAddr_t nbMacAddr){
	yetiNeighbour_t* newYetiNeighbour;
	macAddr_t newMacAddr;
	osMutexWait(yetiRoutingMutex, osWaitForever);

	if(checkExistingMacInNbTable(nbMacAddr)){
		osMutexRelease(yetiRoutingMutex);
		return RET_ERROR;
	}
	if(checkExistingNetAddrInNbTable(nbNetAddr)){
		osMutexRelease(yetiRoutingMutex);
		return RET_ERROR;
	}

	newYetiNeighbour = (yetiNeighbour_t*) pvPortMalloc(sizeof(yetiNeighbour_t));
	newMacAddr = netstack.macLayerOps->macNewAddr(MAC_BROADCAST_ADDR_STR, 'H');
	netstack.macLayerOps->macAddrCpy(newMacAddr, nbMacAddr);
	newYetiNeighbour->nbMacAddr = newMacAddr;
	newYetiNeighbour->yetiNetAddr = nbNetAddr;

	genListAdd(neighboursList, (void*)newYetiNeighbour);
	osMutexRelease(yetiRoutingMutex);

	return RET_OK;
}

/**
 *
 * @param netAddr
 * @return
 */
static macAddr_t getNeighbourMacAddr(yetiNetAddr_t netAddr){
	macAddr_t retMacAddr;
	osMutexWait(yetiRoutingMutex, osWaitForever);
	genListElement_t* current = neighboursList->tailElement;
	yetiNeighbour_t* yetiNeighbour;

	while(current != NULL){
		yetiNeighbour = (yetiNeighbour_t*) current->item;
		if(netAddr == yetiNeighbour->yetiNetAddr){
			retMacAddr = yetiNeighbour->nbMacAddr;
			osMutexRelease(yetiRoutingMutex);
			return retMacAddr;
		}
		current = current->next;
	}
	osMutexRelease(yetiRoutingMutex);
	return NULL;
}

/**
 *
 * @param nbMacAddr
 * @return
 */
static uint16_t checkExistingMacInNbTable(macAddr_t nbMacAddr){

	genListElement_t* current = neighboursList->tailElement;
	yetiNeighbour_t* yetiNeighbour;

	while(current != NULL){
		yetiNeighbour = (yetiNeighbour_t*) current->item;
		if(netstack.macLayerOps->macAddrCmp(nbMacAddr, yetiNeighbour->nbMacAddr)){
			return 1;
		}
		current = current->next;
	}

	return 0;
}

/**
 *
 * @param nbNetAddr
 * @return
 */
static uint16_t checkExistingNetAddrInNbTable(yetiNetAddr_t nbNetAddr){
	genListElement_t* current = neighboursList->tailElement;
	yetiNeighbour_t* yetiNeighbour;

	while(current != NULL){
		yetiNeighbour = (yetiNeighbour_t*) current->item;
		if(nbNetAddr == yetiNeighbour->yetiNetAddr){
			return 1;
		}
		current = current->next;
	}

	return 0;
}
