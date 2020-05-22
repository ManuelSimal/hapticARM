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
 * yetiTransport.c
 *
 *  Created on: 13 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file yetiTransport.c
 */


#include "yetiOS.h"
#include "netstack.h"

#define MAX_TCP_RETRANSMISSIONS					2
#define CONNECT_ATTEMP_TIMEOUT					500
#define RETRANSMISSION_TIMEOUT					500
#define DEFAULT_INNACTIVE_CONNECTION_TIMEOUT	10000
#define MIN_INNACTIVE_CONNECTION_TIMEOUT		5000

#define	PACKET_TYPE_UDP_SEND					0x01
#define	PACKET_TYPE_TCP_SEND					0x02
#define	PACKET_TYPE_TCP_SEND_ACK				0x03
#define	PACKET_TYPE_TCP_CONNECT					0x04
#define	PACKET_TYPE_TCP_CONNECT_ACK				0x05
#define	PACKET_TYPE_TCP_DISCONNECT				0x06


typedef struct __attribute__((__packed__)) yetiTransportHdr_{
	uint32_t connectionId;			/*The connection ID is the ptr value of the connection handle*/
	uint16_t portNum;
	uint16_t dataSize;
	uint8_t tpPacketType;
}yetiTransportHdr_t;

typedef struct yetiTcpServer_{
	genList_t* tcpConnectionsList;
	osMutexId serverMutex;
	osMessageQId rcvMessageQId;
	uint16_t portNumber;
	uint16_t serverListening;
}yetiTcpServer_t;

typedef struct yetiTcpConnection_{
	uint8_t* rxDataPtr;
	netAddr_t rxFromAddr;
	osMutexId connMutex;
	osMessageQId rcvMessageQId;
	osMessageQId sendMessageQId;
	osTimerId inactiveConnTimer;
	osTimerId txTimeoutTimer;
	uint32_t innactiveConnectionTimeout;
	netReadCbFunc_t asynCbFunc;
	yetiTcpServer_t* yetiTcpLocalServer;
	uint32_t remoteConnectionId;
	uint16_t portNumber;
	uint16_t connected;
	uint16_t connectionReceiving;
	uint16_t waitingAck;
	uint16_t rxSize;
}yetiTcpConnection_t;


typedef struct yetiUdpPort_{
	uint8_t* rxDataPtr;
	netAddr_t rxFromAddr;
	osMutexId portMutex;
	osMessageQId sendMessageQId;
	osMessageQId rcvMessageQId;
	netReadCbFunc_t asynCbFunc;
	uint16_t portSending;
	uint16_t portReceiving;
	uint16_t portNumber;
	uint16_t rxSize;
}yetiUdpPort_t;

static genList_t* udpPortsList;

static genList_t* tcpServersList;
static genList_t* tcpConnectionsList;

static osMutexId udpPortsListMutex;

static osMutexId tcpServersListMutex;
static osMutexId tcpConnectionsListMutex;

static uint16_t transportOpRunning = 0;
static uint16_t yetiTransportStarted = 0;

static retval_t yetiTransportLayerInit(void);
static retval_t yetiTransportLayerDeInit(void);

/* UDP like funcs */
static retval_t yetiUdpOpenPort(uint16_t portNumber);
static retval_t yetiUdpClosePort(uint16_t portNumber);
static uint32_t yetiUdpSend(netAddr_t destAddr, uint16_t portNumber, uint8_t* data, uint32_t size);
static uint32_t yetiUdpRcv(netAddr_t fromAddr, uint16_t portNumber, uint8_t* data, uint32_t size);
static retval_t yetiUdpRcvAsync(uint16_t portNumber, netReadCbFunc_t readCbFunc);
static retval_t yetiUdpStopRcvAsync(uint16_t portNumber);
/* TCP like funcs */
static uint32_t yetiTcpCreateServer(uint16_t portNumber);
static retval_t yetiTcpDeleteServer(uint32_t serverHandle);
static uint32_t yetiTcpListenConnection(uint32_t serverHandle, netAddr_t fromAddr);
static uint32_t yetiTcpConnectServer(netAddr_t destAddr, uint16_t portNumber);
static uint32_t yetiTcpSend(uint32_t connectionHandle, uint8_t* data, uint32_t size);
static uint32_t yetiTcpRcv(uint32_t connectionHandle, uint8_t* data, uint32_t size);
static retval_t yetiTcpRcvAsync(uint32_t connectionHandle, netReadCbFunc_t readCbFunc);
static retval_t yetiTcpStopRcvAsync(uint32_t connectionHandle);
static retval_t yetiTcpCloseConnection(uint32_t connectionHandle);
static uint32_t yetiTcpCheckConnection(uint32_t connectionHandle);
static retval_t yetiTcpSetConnectionTimeout(uint32_t connectionHandle, uint32_t timeout);

/* Send and receive packet callbacks (from routing layer)*/
static retval_t yetiTransportPacketSent(netPacket_t* packetSent);
static retval_t yetiTransportPacketReceived(netPacket_t* rcvPacket, netAddr_t fromAddr);


/*Local aux functions*/
static void innactiveConnTimeoutCb(void const * argument);
static void txTimeoutCb(void const * argument);
static void closeConnThreadCb(void const * argument);

static yetiUdpPort_t* getUdpPortFromList(uint16_t portNum);
static 	yetiTcpServer_t* getTcpServerFromList(uint16_t portNumber);
static uint16_t checkTcpServerFromList(yetiTcpServer_t* tcpServer);
static uint16_t checkTcpConnectionFromList(yetiTcpConnection_t* tcpConnection);
static yetiTcpConnection_t* getTcpConnectionWithRemoteId(uint32_t remoteConnectionId);



transportLayerOps_t yetiTransportLayer = {
	.transportLayerInit = yetiTransportLayerInit,
	.transportLayerDeInit = yetiTransportLayerDeInit,
	.udpOpenPort = yetiUdpOpenPort,
	.udpClosePort = yetiUdpClosePort,
	.udpSend = yetiUdpSend,
	.udpRcv = yetiUdpRcv,
	.udpRcvAsync = yetiUdpRcvAsync,
	.udpStopRcvAsync = yetiUdpStopRcvAsync,
	.tcpCreateServer = yetiTcpCreateServer,
	.tcpDeleteServer = yetiTcpDeleteServer,
	.tcpListenConnection = yetiTcpListenConnection,
	.tcpConnectServer = yetiTcpConnectServer,
	.tcpSend = yetiTcpSend,
	.tcpRcv = yetiTcpRcv,
	.tcpRcvAsync = yetiTcpRcvAsync,
	.tcpStopRcvAsync = yetiTcpStopRcvAsync,
	.tcpCloseConnection = yetiTcpCloseConnection,
	.tcpCheckConnection = yetiTcpCheckConnection,
	.tcpSetConnectionTimeout = yetiTcpSetConnectionTimeout,
	.transportPacketSent = yetiTransportPacketSent,
	.transportPacketReceived = yetiTransportPacketReceived,
};


/**
 *
 * @return
 */
static retval_t yetiTransportLayerInit(void){

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(yetiTransportStarted || transportOpRunning){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	yetiTransportStarted++;

	setTransportHdrSize(rxPacketbuffer, (uint16_t) sizeof(yetiTransportHdr_t));
	setTransportHdrSize(txPacketbuffer, (uint16_t) sizeof(yetiTransportHdr_t));

	udpPortsList = genListInit();
	tcpConnectionsList = genListInit();
	tcpServersList = genListInit();

	udpPortsListMutex = ytMutexCreate();
	tcpConnectionsListMutex = ytMutexCreate();
	tcpServersListMutex = ytMutexCreate();

	transportOpRunning--;
	taskEXIT_CRITICAL();

	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t yetiTransportLayerDeInit(void){
	yetiTransportHdr_t* packetTransportHeader;
	yetiUdpPort_t* udpPort;
	netPacket_t* packet;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	yetiTransportStarted = 0;

	while(transportOpRunning){								/*Wait until all operations are finished before closing*/
		taskEXIT_CRITICAL();
		osDelay(1);
		taskENTER_CRITICAL();
	}

	transportOpRunning++;
	taskEXIT_CRITICAL();

	/*First close all opened UDP ports*/
	osMutexWait(udpPortsListMutex, osWaitForever);
	genListElement_t* current = udpPortsList->tailElement;
	while(current != NULL){
		udpPort = (yetiUdpPort_t*) current->item;
		genListRemove(udpPortsList, udpPort);
		while(udpPort->portSending){
			osMessagePut (udpPort->sendMessageQId, 0, 0);		/*Exit send function returning error (0)*/
			osMutexRelease(udpPort->portMutex);
			osDelay(1);		/*Leave some time to end send function*/
			osMutexWait(udpPortsListMutex, osWaitForever);
		}
		while(udpPort->portReceiving){
			if(udpPort->asynCbFunc == NULL){
				osMessagePut (udpPort->rcvMessageQId, 0, 0);	/*Exit rcv function returning error (0)*/
				osMutexRelease(udpPort->portMutex);
				osDelay(1);	/*Leave some time to end rcv function*/
				osMutexWait(udpPortsListMutex, osWaitForever);
			}
			else{
				break;
			}
		}
		udpPort->portReceiving = 0;
		osMutexDelete(udpPort->portMutex);
		osMessageDelete(udpPort->sendMessageQId);
		osMessageDelete(udpPort->rcvMessageQId);

		vPortFree(udpPort);
		current = current->next;
	}
	genListRemoveAll(udpPortsList);
	osMutexDelete(udpPortsListMutex);

	/*Close all tcp connections and servers*/
	/*First Close All TCP Connections*/
	current = tcpConnectionsList->tailElement;
	yetiTcpConnection_t* tcpConnection;
	osMutexWait(tcpServersListMutex, osWaitForever);
	osMutexWait(tcpConnectionsListMutex, osWaitForever);
	while(current != NULL){
		tcpConnection = (yetiTcpConnection_t*) current->item;
		genListRemove(tcpConnectionsList, tcpConnection);

		/*DELETE ALSO FROM SERVER CONNECTIONS LIST*/
		if(tcpConnection->yetiTcpLocalServer != NULL){
			if(checkTcpServerFromList(tcpConnection->yetiTcpLocalServer)){		/*Check the server exists in the list.*/
				osMutexWait(tcpConnection->yetiTcpLocalServer->serverMutex, osWaitForever);
				genListRemove(tcpConnection->yetiTcpLocalServer->tcpConnectionsList, (void*) tcpConnection);	/*Also remove the connection from the server connections list*/
				osMutexRelease(tcpConnection->yetiTcpLocalServer->serverMutex);
			}
		}
		/*Create the disconnect packet to be sent*/
		while((packet = packetbufferGetFreePacket(txPacketbuffer)) == NULL){
			osDelay(1);
		}

		osMutexWait(tcpConnection->connMutex, osWaitForever);
		/*Reset the inactive timer*/
		osTimerStop (tcpConnection->inactiveConnTimer);
		osTimerStart (tcpConnection->inactiveConnTimer, tcpConnection->innactiveConnectionTimeout);

		packetTransportHeader = (yetiTransportHdr_t*)getPacketTransportHeaderPtr(packet);
		packetTransportHeader->dataSize = 0;
		packetTransportHeader->portNum = tcpConnection->portNumber;
		packetTransportHeader->tpPacketType = PACKET_TYPE_TCP_DISCONNECT;
		packetTransportHeader->connectionId = (uint32_t)tcpConnection;


		if((netstack.routingLayerOps->rtSendPacket(tcpConnection->rxFromAddr, packet)) != RET_OK){	/*Send the disconnect packet*/
			packetbufferReleasePacket(txPacketbuffer, packet);
		}
		/*Dont wait for anything and delete the connection immediatelly*/
		while(!tcpConnection->connected){
			osMessagePut (tcpConnection->rcvMessageQId, 0, 0);
			osMutexRelease(tcpConnection->connMutex);
			osDelay(1);
			osMutexWait(tcpConnection->connMutex, osWaitForever);
		}

		while(tcpConnection->waitingAck){
			osMessagePut (tcpConnection->sendMessageQId, 0, 0);
			osMutexRelease(tcpConnection->connMutex);
			osDelay(1);
			osMutexWait(tcpConnection->connMutex, osWaitForever);
		}

		while(tcpConnection->connectionReceiving){
			if(tcpConnection->asynCbFunc == NULL){
				osMessagePut (tcpConnection->rcvMessageQId, 0, 0);
				osMutexRelease(tcpConnection->connMutex);
				osDelay(1);
				osMutexWait(tcpConnection->connMutex, osWaitForever);
			}
			else{
				break;
			}
		}
		tcpConnection->connectionReceiving = 0;
		tcpConnection->asynCbFunc = NULL;

		osTimerStop (tcpConnection->inactiveConnTimer);
		osTimerStop (tcpConnection->txTimeoutTimer);
		osTimerDelete(tcpConnection->inactiveConnTimer);
		osTimerDelete(tcpConnection->txTimeoutTimer);
		osMessageDelete(tcpConnection->sendMessageQId);
		osMessageDelete(tcpConnection->rcvMessageQId);
		netstack.routingLayerOps->rtDeleteNetAddr(tcpConnection->rxFromAddr);

		osMutexDelete(tcpConnection->connMutex);

		vPortFree(tcpConnection);

		current = current->next;
	}
	/*Now close all the servers*/
	current = tcpServersList->tailElement;
	yetiTcpServer_t* tcpServer;
	while(current != NULL){
		tcpServer = (yetiTcpServer_t*) current->item;
		osMutexWait(tcpServer->serverMutex, osWaitForever);
		genListRemove(tcpServersList, tcpServer);
		tcpServer->serverListening = 0;

		genListRemoveAll(tcpServer->tcpConnectionsList);
		osMessageDelete(tcpServer->rcvMessageQId);

		osMutexDelete(tcpServer->serverMutex);

		vPortFree(tcpServer);
		current = current->next;
	}

	genListRemoveAll(tcpServersList);
	genListRemoveAll(tcpConnectionsList);
	osMutexDelete(tcpConnectionsListMutex);
	osMutexDelete(tcpServersListMutex);

	transportOpRunning--;

	return RET_OK;
}


/* UDP like funcs */
/**
 *
 * @param portNumber
 * @return
 */
static retval_t yetiUdpOpenPort(uint16_t portNumber){
	yetiUdpPort_t* newUdpPort;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(udpPortsListMutex, osWaitForever);
	if(getUdpPortFromList(portNumber) != NULL){		/*Port already opened*/
		osMutexRelease(udpPortsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}

	newUdpPort = (yetiUdpPort_t*) pvPortMalloc(sizeof(yetiUdpPort_t));

	newUdpPort->portNumber = portNumber;
	newUdpPort->rxDataPtr = NULL;
	newUdpPort->portReceiving = 0;
	newUdpPort->portSending = 0;
	newUdpPort->portMutex = ytMutexCreate();
	newUdpPort->sendMessageQId = ytMessageqCreate(1);
	newUdpPort->rcvMessageQId = ytMessageqCreate(1);
	newUdpPort->rxSize = 0;

	genListAdd(udpPortsList, newUdpPort);
	osMutexRelease(udpPortsListMutex);

	transportOpRunning--;
	return RET_OK;
}

/**
 *
 * @param portNumber
 * @return
 */
static retval_t yetiUdpClosePort(uint16_t portNumber){
	yetiUdpPort_t* udpPort;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();
	osMutexWait(udpPortsListMutex, osWaitForever);
	if((udpPort = getUdpPortFromList(portNumber)) == NULL){		/*Port not opened*/
		osMutexRelease(udpPortsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	genListRemove(udpPortsList, udpPort);
	osMutexWait(udpPort->portMutex, osWaitForever);
	osMutexRelease(udpPortsListMutex);

	while(udpPort->portSending){
		osMessagePut (udpPort->sendMessageQId, 0, 0);		/*Exit send function returning error (0)*/
		osMutexRelease(udpPort->portMutex);
		osDelay(1);		/*Leave some time to end send function*/
		osMutexWait(udpPort->portMutex, osWaitForever);
	}
	while(udpPort->portReceiving){
		if(udpPort->asynCbFunc == NULL){
			osMessagePut (udpPort->rcvMessageQId, 0, 0);	/*Exit rcv function returning error (0)*/
			osMutexRelease(udpPort->portMutex);
			osDelay(1);	/*Leave some time to end rcv function*/
			osMutexWait(udpPort->portMutex, osWaitForever);
		}
		else{
			break;
		}

	}
	udpPort->portReceiving = 0;
	udpPort->asynCbFunc = NULL;
	osMessageDelete(udpPort->sendMessageQId);
	osMessageDelete(udpPort->rcvMessageQId);
	osMutexDelete(udpPort->portMutex);

	vPortFree(udpPort);

	transportOpRunning--;
	return RET_OK;
}


/**
 *
 * @param destAddr
 * @param portNumber
 * @param data
 * @param size
 * @return
 */
static uint32_t yetiUdpSend(netAddr_t destAddr, uint16_t portNumber, uint8_t* data, uint32_t size){
	yetiUdpPort_t* udpPort;
	netPacket_t* packet;
	yetiTransportHdr_t* packetTransportHeader;
	uint8_t* packetData;
	osEvent event;
	if(size > txPacketbuffer->packetDataSize){	/*Not enough space in a single packet. TODO: Fragmentation*/
		return 0;
	}

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(udpPortsListMutex, osWaitForever);
	if((udpPort = getUdpPortFromList(portNumber)) == NULL){	/*Port not opened*/
		osMutexRelease(udpPortsListMutex);
		transportOpRunning--;
		return 0;
	}
	osMutexWait(udpPort->portMutex, osWaitForever);
	osMutexRelease(udpPortsListMutex);

	if(udpPort->portSending){
		osMutexRelease(udpPort->portMutex);
		transportOpRunning--;
		return 0;
	}



	if((packet = packetbufferGetFreePacket(txPacketbuffer)) == NULL){
		osMutexRelease(udpPort->portMutex);
		transportOpRunning--;
		return 0;
	}
	packetTransportHeader = (yetiTransportHdr_t*)getPacketTransportHeaderPtr(packet);
	packetData = (uint8_t*)getPacketDataPtr(packet);

	packetTransportHeader->dataSize = (uint16_t) size;
	packetTransportHeader->portNum = portNumber;
	packetTransportHeader->tpPacketType = PACKET_TYPE_UDP_SEND;
	packetTransportHeader->connectionId = 0;

	memcpy(packetData, data, size);

	udpPort->portSending++;
	osMutexRelease(udpPort->portMutex);

	if((netstack.routingLayerOps->rtSendPacket(destAddr, packet)) != RET_OK){
		packetbufferReleasePacket(txPacketbuffer, packet);
		osMutexWait(udpPort->portMutex, osWaitForever);
		udpPort->portSending--;
		osMutexRelease(udpPort->portMutex);
		transportOpRunning--;
		return 0;
	}
	event = osMessageGet (udpPort->sendMessageQId, osWaitForever);

	packetbufferReleasePacket(txPacketbuffer, packet);

	osMutexWait(udpPort->portMutex, osWaitForever);
	udpPort->portSending--;
	osMutexRelease(udpPort->portMutex);

	transportOpRunning--;
	return event.value.v;
}

/**
 *
 * @param fromAddr
 * @param portNumber
 * @param data
 * @param size
 * @return
 */
static uint32_t yetiUdpRcv(netAddr_t fromAddr, uint16_t portNumber, uint8_t* data, uint32_t size){
	yetiUdpPort_t* udpPort;
	osEvent event;
	if(size > rxPacketbuffer->packetDataSize){	/*Not enough space in a single packet. TODO: Fragmentation*/
		return 0;
	}

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(udpPortsListMutex, osWaitForever);
	if((udpPort = getUdpPortFromList(portNumber)) == NULL){	/*Port not opened*/
		osMutexRelease(udpPortsListMutex);
		transportOpRunning--;
		return 0;
	}
	osMutexWait(udpPort->portMutex, osWaitForever);
	osMutexRelease(udpPortsListMutex);

	if(udpPort->portReceiving){
		osMutexRelease(udpPort->portMutex);
		transportOpRunning--;
		return 0;
	}
	udpPort->portReceiving++;
	udpPort->rxDataPtr = data;
	udpPort->rxFromAddr = fromAddr;
	udpPort->asynCbFunc = NULL;
	udpPort->rxSize = (uint16_t) size;
	osMutexRelease(udpPort->portMutex);

	if((netstack.routingLayerOps->rtRcvPacket()) != RET_OK){
		osMutexWait(udpPort->portMutex, osWaitForever);
		udpPort->portReceiving--;
		osMutexRelease(udpPort->portMutex);
		transportOpRunning--;
		return 0;
	}
	event = osMessageGet (udpPort->rcvMessageQId, osWaitForever);

	osMutexWait(udpPort->portMutex, osWaitForever);
	udpPort->portReceiving--;
	osMutexRelease(udpPort->portMutex);

	transportOpRunning--;
	return event.value.v;
}

/**
 *
 * @param portNumber
 * @param data
 * @param size
 * @param readCbFunc
 * @param args
 * @return
 */
static retval_t yetiUdpRcvAsync(uint16_t portNumber, netReadCbFunc_t readCbFunc){
	yetiUdpPort_t* udpPort;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(udpPortsListMutex, osWaitForever);
	if((udpPort = getUdpPortFromList(portNumber)) == NULL){	/*Port not opened*/
		osMutexRelease(udpPortsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	osMutexWait(udpPort->portMutex, osWaitForever);
	osMutexRelease(udpPortsListMutex);

	if(udpPort->portReceiving){
		osMutexRelease(udpPort->portMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	udpPort->portReceiving++;
	udpPort->asynCbFunc = readCbFunc;
	osMutexRelease(udpPort->portMutex);

	retval_t ret = netstack.routingLayerOps->rtRcvPacket();

	transportOpRunning--;
	return ret;
}

/**
 *
 * @param portNumber
 * @return
 */
static retval_t yetiUdpStopRcvAsync(uint16_t portNumber){
	yetiUdpPort_t* udpPort;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(udpPortsListMutex, osWaitForever);
	if((udpPort = getUdpPortFromList(portNumber)) == NULL){	/*Port not opened*/
		osMutexRelease(udpPortsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	osMutexWait(udpPort->portMutex, osWaitForever);
	osMutexRelease(udpPortsListMutex);

	if(!udpPort->portReceiving){
		osMutexRelease(udpPort->portMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	udpPort->portReceiving--;
	udpPort->asynCbFunc = NULL;
	osMutexRelease(udpPort->portMutex);

	transportOpRunning--;
	return RET_OK;
}


/* TCP like funcs */
/**
 *
 * @param portNumber
 * @return
 */
static uint32_t yetiTcpCreateServer(uint16_t portNumber){
	yetiTcpServer_t* tcpServer;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(tcpServersListMutex, osWaitForever);
	if(getTcpServerFromList(portNumber) != NULL){		/*Server already listening in this port*/
		osMutexRelease(tcpServersListMutex);
		transportOpRunning--;
		return 0;
	}
	tcpServer = (yetiTcpServer_t*) pvPortMalloc(sizeof(yetiTcpServer_t));
	tcpServer->portNumber = portNumber;
	tcpServer->tcpConnectionsList = genListInit();
	tcpServer->serverMutex = ytMutexCreate();
	tcpServer->rcvMessageQId = ytMessageqCreate(1);
	tcpServer->serverListening = 0;
	genListAdd(tcpServersList, tcpServer);
	osMutexRelease(tcpServersListMutex);

	transportOpRunning--;

	return (uint32_t) tcpServer;
}

/**
 *
 * @param serverHandle
 * @return
 */
static retval_t yetiTcpDeleteServer(uint32_t serverHandle){
	yetiTcpServer_t* tcpServer = (yetiTcpServer_t*) serverHandle;
	yetiTcpConnection_t* tcpConnection;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(tcpServersListMutex, osWaitForever);
	if(!checkTcpServerFromList(tcpServer)){		/*Check the server exists in the list*/
		osMutexRelease(tcpServersListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	osMutexWait(tcpServer->serverMutex, osWaitForever);
	genListRemove(tcpServersList, (void*) tcpServer);
	osMutexRelease(tcpServersListMutex);

	tcpServer->serverListening = 0;

	genListElement_t* current = tcpServer->tcpConnectionsList->tailElement;

	while(current != NULL){
		tcpConnection = (yetiTcpConnection_t*) current->item;

		genListRemove(tcpServer->tcpConnectionsList, (void*) tcpConnection);
		yetiTcpCloseConnection((uint32_t) tcpConnection);

		current = current->next;
	}

	genListRemoveAll(tcpServer->tcpConnectionsList);
	osMessageDelete(tcpServer->rcvMessageQId);

	osMutexDelete(tcpServer->serverMutex);

	vPortFree(tcpServer);

	transportOpRunning--;
	return RET_OK;
}

/**
 *
 * @param serverHandle
 * @param fromAddr
 * @return
 */
static uint32_t yetiTcpListenConnection(uint32_t serverHandle, netAddr_t fromAddr){
	yetiTcpServer_t* tcpServer = (yetiTcpServer_t*) serverHandle;
	yetiTcpConnection_t* tcpConnection;
	osEvent event;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();
	osMutexWait(tcpServersListMutex, osWaitForever);
	if(!checkTcpServerFromList(tcpServer)){		/*Check the server exists in the list*/
		osMutexRelease(tcpServersListMutex);
		transportOpRunning--;
		return 0;
	}
	osMutexWait(tcpServer->serverMutex, osWaitForever);
	osMutexRelease(tcpServersListMutex);

	tcpServer->serverListening++;
	osMutexRelease(tcpServer->serverMutex);

	event = osMessageGet (tcpServer->rcvMessageQId, osWaitForever);		/*Wait until a connect message arrives*/

	tcpConnection = (yetiTcpConnection_t*) event.value.p;
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	netstack.routingLayerOps->rtNetAddrCpy(fromAddr, tcpConnection->rxFromAddr);
	osMutexRelease(tcpConnection->connMutex);

	transportOpRunning--;
	return event.value.v;	/*Return the connection handle*/
}


/**
 *
 * @param destAddr
 * @param portNumber
 * @return
 */
static uint32_t yetiTcpConnectServer(netAddr_t destAddr, uint16_t portNumber){
	osEvent event;
	netPacket_t* packet;
	yetiTransportHdr_t* packetTransportHeader;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(tcpServersListMutex, osWaitForever);
	if(getTcpServerFromList(portNumber) != NULL){		/*Check if there is a server locally created for this port*/
		osMutexRelease(tcpServersListMutex);
		transportOpRunning--;
		return 0;
	}
	osMutexRelease(tcpServersListMutex);

	/*Create a packet to connect to the server*/
	if((packet = packetbufferGetFreePacket(txPacketbuffer)) == NULL){
		transportOpRunning--;
		return 0;
	}

	/*Create the new connection and store it in the global connections list*/
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) pvPortMalloc(sizeof(yetiTcpConnection_t));
	tcpConnection->connMutex = ytMutexCreate();
	tcpConnection->rxDataPtr = NULL;
	tcpConnection->sendMessageQId = ytMessageqCreate(1);
	tcpConnection->rcvMessageQId = ytMessageqCreate(1);
	tcpConnection->inactiveConnTimer = ytTimerCreate(osTimerOnce, innactiveConnTimeoutCb, (void*) tcpConnection);
	tcpConnection->txTimeoutTimer = ytTimerCreate(osTimerOnce, txTimeoutCb, (void*) tcpConnection);
	tcpConnection->connected = 0;
	tcpConnection->portNumber = portNumber;
	tcpConnection->remoteConnectionId = 0;
	tcpConnection->connectionReceiving = 0;
	tcpConnection->waitingAck = 0;
	tcpConnection->rxSize = 0;
	tcpConnection->asynCbFunc = NULL;
	tcpConnection->yetiTcpLocalServer = NULL;	/*This connection does not belong to a local server*/
	tcpConnection->innactiveConnectionTimeout = DEFAULT_INNACTIVE_CONNECTION_TIMEOUT;
	tcpConnection->rxFromAddr = netstack.routingLayerOps->rtNewNetAddr("0", '0');	/*Empty address*/
	netstack.routingLayerOps->rtNetAddrCpy(tcpConnection->rxFromAddr, destAddr);
	osMutexWait(tcpConnection->connMutex, osWaitForever);

	osMutexWait(tcpConnectionsListMutex, osWaitForever);
	genListAdd(tcpConnectionsList, (void*) tcpConnection);
	osMutexRelease(tcpConnectionsListMutex);

	/*Start the inactive timer*/
	osTimerStart (tcpConnection->inactiveConnTimer, tcpConnection->innactiveConnectionTimeout);

	packetTransportHeader = (yetiTransportHdr_t*)getPacketTransportHeaderPtr(packet);
	packetTransportHeader->connectionId = (uint32_t)tcpConnection;	/*Append this connectionId*/

	packetTransportHeader->dataSize = 0;
	packetTransportHeader->portNum = portNumber;
	packetTransportHeader->tpPacketType = PACKET_TYPE_TCP_CONNECT;

	osTimerStart (tcpConnection->txTimeoutTimer, CONNECT_ATTEMP_TIMEOUT);
	osMutexRelease(tcpConnection->connMutex);

	if((netstack.routingLayerOps->rtSendPacket(destAddr, packet)) != RET_OK){	/*Send the packet*/
		packetbufferReleasePacket(txPacketbuffer, packet);
		/*Delete the created connection*/
		osMutexWait(tcpConnection->connMutex, osWaitForever);

		osMutexWait(tcpConnectionsListMutex, osWaitForever);
		genListRemove(tcpConnectionsList, (void*) tcpConnection);
		osMutexRelease(tcpConnectionsListMutex);

		osMessageDelete(tcpConnection->sendMessageQId);
		osMessageDelete(tcpConnection->rcvMessageQId);
		osTimerStop(tcpConnection->inactiveConnTimer);
		osTimerDelete(tcpConnection->inactiveConnTimer);
		osTimerStop(tcpConnection->txTimeoutTimer);
		osTimerDelete(tcpConnection->txTimeoutTimer);
		netstack.routingLayerOps->rtDeleteNetAddr(tcpConnection->rxFromAddr);
		osMutexDelete(tcpConnection->connMutex);
		vPortFree(tcpConnection);

		transportOpRunning--;
		return 0;
	}

	event = osMessageGet (tcpConnection->rcvMessageQId, osWaitForever);	/*Wait for the ACK*/

	osMutexWait(tcpConnection->connMutex, osWaitForever);
	osTimerStop(tcpConnection->txTimeoutTimer);
	tcpConnection->connected++;

	osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list. If not, someone is clossing the connection*/
	if(checkTcpConnectionFromList(tcpConnection) && (!event.value.v)){	/*Only delete the connection if it still exists in the list and a error (0) is received in the queue*/

		genListRemove(tcpConnectionsList, (void*) tcpConnection);

		osMessageDelete(tcpConnection->sendMessageQId);
		osMessageDelete(tcpConnection->rcvMessageQId);
		osTimerStop(tcpConnection->inactiveConnTimer);
		osTimerDelete(tcpConnection->inactiveConnTimer);
		osTimerStop(tcpConnection->txTimeoutTimer);
		osTimerDelete(tcpConnection->txTimeoutTimer);
		netstack.routingLayerOps->rtDeleteNetAddr(tcpConnection->rxFromAddr);
		osMutexDelete(tcpConnection->connMutex);
		vPortFree(tcpConnection);
	}
	else{
		osMutexRelease(tcpConnection->connMutex);
	}
	osMutexRelease(tcpConnectionsListMutex);

	transportOpRunning--;
	return event.value.v;	/*Return the connection handle*/
}

/**
 *
 * @param connectionHandle
 * @param data
 * @param size
 * @return
 */
static uint32_t yetiTcpSend(uint32_t connectionHandle, uint8_t* data, uint32_t size){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) connectionHandle;
	yetiTransportHdr_t* packetTransportHeader;
	osEvent event;
	netPacket_t* packet;
	uint8_t* packetData;
	uint32_t retSize = 0xFFFFFFFF;
	uint32_t numRetransmission = 0;

	if(size > txPacketbuffer->packetDataSize){	/*Not enough space in a single packet. TODO: Fragmentation*/
		return 0;
	}

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	while((retSize == 0xFFFFFFFF) && (numRetransmission <= MAX_TCP_RETRANSMISSIONS)){	/*0xFFFFFFFF is sent by timeout function to indicate the ack has not been received*/
		osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list*/
		if(!checkTcpConnectionFromList(tcpConnection)){
			osMutexRelease(tcpConnectionsListMutex);
			transportOpRunning--;
			return 0;
		}
		osMutexWait(tcpConnection->connMutex, osWaitForever);
		osMutexRelease(tcpConnectionsListMutex);

		/*Check if the connection is connected and it is not already waiting for an ack (sending a packet)*/
		if((!tcpConnection->connected) || (tcpConnection->waitingAck)){
			osMutexRelease(tcpConnection->connMutex);
			transportOpRunning--;
			return 0;
		}

		/*Create the packet to be sent*/
		if((packet = packetbufferGetFreePacket(txPacketbuffer)) == NULL){
			osMutexRelease(tcpConnection->connMutex);
			transportOpRunning--;
			return 0;
		}
		packetTransportHeader = (yetiTransportHdr_t*)getPacketTransportHeaderPtr(packet);
		packetTransportHeader->dataSize = size;
		packetTransportHeader->portNum = tcpConnection->portNumber;
		packetTransportHeader->tpPacketType = PACKET_TYPE_TCP_SEND;
		packetTransportHeader->connectionId = (uint32_t)tcpConnection;

		packetData = (uint8_t*)getPacketDataPtr(packet);
		memcpy(packetData, data, size);

		tcpConnection->waitingAck++;

		netAddr_t tempRxFromAddr = netstack.routingLayerOps->rtNewNetAddr("0", '0');	/*Empty address*/
		netstack.routingLayerOps->rtNetAddrCpy(tempRxFromAddr, tcpConnection->rxFromAddr);

		osTimerStart (tcpConnection->txTimeoutTimer, RETRANSMISSION_TIMEOUT);
		osMutexRelease(tcpConnection->connMutex);

		if((netstack.routingLayerOps->rtSendPacket(tempRxFromAddr, packet)) != RET_OK){	/*Send the packet*/
			netstack.routingLayerOps->rtDeleteNetAddr(tempRxFromAddr);
			packetbufferReleasePacket(txPacketbuffer, packet);
			osMutexWait(tcpConnection->connMutex, osWaitForever);
			tcpConnection->waitingAck--;
			osMutexRelease(tcpConnection->connMutex);
			transportOpRunning--;
			return 0;
		}
		netstack.routingLayerOps->rtDeleteNetAddr(tempRxFromAddr);
		event = osMessageGet (tcpConnection->sendMessageQId, osWaitForever);	/*Wait for the ACK*/

		osMutexWait(tcpConnection->connMutex, osWaitForever);
		osTimerStop (tcpConnection->txTimeoutTimer);
		tcpConnection->waitingAck--;
		osMutexRelease(tcpConnection->connMutex);
		numRetransmission++;
		retSize = event.value.v;
	}

	if(retSize == 0xFFFFFFFF){	/*Max retransmissions*/
		retSize = 0;
	}

	transportOpRunning--;
	return retSize;	/*Return the size*/

}

/**
 *
 * @param connectionHandle
 * @param data
 * @param size
 * @return
 */
static uint32_t yetiTcpRcv(uint32_t connectionHandle, uint8_t* data, uint32_t size){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) connectionHandle;
	osEvent event;

	if(size > rxPacketbuffer->packetDataSize){	/*Not enough space in a single packet. TODO: Fragmentation*/
		return 0;
	}

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return 0;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list*/
	if(!checkTcpConnectionFromList(tcpConnection)){
		osMutexRelease(tcpConnectionsListMutex);
		transportOpRunning--;
		return 0;
	}
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	osMutexRelease(tcpConnectionsListMutex);

	/*Check if the connection is connected and it is not already reading packets*/
	if((!tcpConnection->connected) || (tcpConnection->connectionReceiving)){
		osMutexRelease(tcpConnection->connMutex);
		transportOpRunning--;
		return 0;
	}
	tcpConnection->connectionReceiving++;
	tcpConnection->rxDataPtr = data;
	tcpConnection->asynCbFunc = NULL;
	tcpConnection->rxSize = (uint16_t) size;
	osMutexRelease(tcpConnection->connMutex);

	if((netstack.routingLayerOps->rtRcvPacket()) != RET_OK){
		osMutexWait(tcpConnection->connMutex, osWaitForever);
		tcpConnection->connectionReceiving--;
		osMutexRelease(tcpConnection->connMutex);
		transportOpRunning--;
		return 0;
	}
	event = osMessageGet (tcpConnection->rcvMessageQId, osWaitForever);

	osMutexWait(tcpConnection->connMutex, osWaitForever);
	tcpConnection->connectionReceiving--;
	osMutexRelease(tcpConnection->connMutex);

	transportOpRunning--;
	return event.value.v;	/*Return the size*/
}

/**
 *
 * @param connectionFd
 * @param data
 * @param size
 * @param readCbFunc
 * @return
 */
static retval_t yetiTcpRcvAsync(uint32_t connectionHandle, netReadCbFunc_t readCbFunc){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) connectionHandle;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list*/
	if(!checkTcpConnectionFromList(tcpConnection)){
		osMutexRelease(tcpConnectionsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	osMutexRelease(tcpConnectionsListMutex);

	/*Check if the connection is connected and it is not already reading packets*/
	if((!tcpConnection->connected) || (tcpConnection->connectionReceiving)){
		osMutexRelease(tcpConnection->connMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	tcpConnection->connectionReceiving++;
	tcpConnection->asynCbFunc = readCbFunc;
	osMutexRelease(tcpConnection->connMutex);

	if((netstack.routingLayerOps->rtRcvPacket()) != RET_OK){
		transportOpRunning--;
		return RET_ERROR;
	}

	transportOpRunning--;
	return RET_OK;
}


static retval_t yetiTcpStopRcvAsync(uint32_t connectionHandle){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) connectionHandle;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list*/
	if(!checkTcpConnectionFromList(tcpConnection)){
		osMutexRelease(tcpConnectionsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	osMutexRelease(tcpConnectionsListMutex);/*Always capture the connection mutex before releasing the connections list mutex. Prevent race condition when closing connections*/

	if((!tcpConnection->connected) || (!tcpConnection->connectionReceiving) || (tcpConnection->asynCbFunc == NULL)){
		osMutexRelease(tcpConnection->connMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	tcpConnection->connectionReceiving--;
	tcpConnection->asynCbFunc = NULL;
	osMutexRelease(tcpConnection->connMutex);

	transportOpRunning--;
	return RET_OK;
}

/**
 *
 * @param connectionHandle
 * @return
 */
static retval_t yetiTcpCloseConnection(uint32_t connectionHandle){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) connectionHandle;
	yetiTransportHdr_t* packetTransportHeader;
	netPacket_t* packet;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	/*Create the disconnect packet to be sent*/
	if((packet = packetbufferGetFreePacket(txPacketbuffer)) == NULL){
		transportOpRunning--;
		return RET_ERROR;
	}

	osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list*/
	if(!checkTcpConnectionFromList(tcpConnection)){
		packetbufferReleasePacket(txPacketbuffer, packet);
		osMutexRelease(tcpConnectionsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	genListRemove(tcpConnectionsList, (void*) tcpConnection);	/*Remove connection from connections list*/
	/*DELETE ALSO FROM SERVER CONNECTIONS LIST*/
	if(tcpConnection->yetiTcpLocalServer != NULL){
		osMutexWait(tcpServersListMutex, osWaitForever);
		if(checkTcpServerFromList(tcpConnection->yetiTcpLocalServer)){		/*Check the server exists in the list.*/

			osMutexWait(tcpConnection->yetiTcpLocalServer->serverMutex, osWaitForever);
			genListRemove(tcpConnection->yetiTcpLocalServer->tcpConnectionsList, (void*) tcpConnection);	/*Also remove the connection from the server connections list*/
			osMutexRelease(tcpConnection->yetiTcpLocalServer->serverMutex);

		}
		osMutexRelease(tcpServersListMutex);
	}

	osMutexRelease(tcpConnectionsListMutex);

	/*Reset the inactive timer*/
	packetTransportHeader = (yetiTransportHdr_t*)getPacketTransportHeaderPtr(packet);
	packetTransportHeader->dataSize = 0;
	packetTransportHeader->portNum = tcpConnection->portNumber;
	packetTransportHeader->tpPacketType = PACKET_TYPE_TCP_DISCONNECT;
	packetTransportHeader->connectionId = (uint32_t)tcpConnection;

	netAddr_t tempRxFromAddr = netstack.routingLayerOps->rtNewNetAddr("0", '0');	/*Empty address*/
	netstack.routingLayerOps->rtNetAddrCpy(tempRxFromAddr, tcpConnection->rxFromAddr);
	osMutexRelease(tcpConnection->connMutex);

	if((netstack.routingLayerOps->rtSendPacket(tempRxFromAddr, packet)) != RET_OK){	/*Send the disconnect packet*/
		packetbufferReleasePacket(txPacketbuffer, packet);
	}
	netstack.routingLayerOps->rtDeleteNetAddr(tempRxFromAddr);
	/*Dont wait for anything and delete the connection immediatelly*/
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	while(!tcpConnection->connected){
		osMessagePut (tcpConnection->rcvMessageQId, 0, 0);
		osMutexRelease(tcpConnection->connMutex);
		osDelay(1);
		osMutexWait(tcpConnection->connMutex, osWaitForever);
	}

	while(tcpConnection->waitingAck){
		osMessagePut (tcpConnection->sendMessageQId, 0, 0);
		osMutexRelease(tcpConnection->connMutex);
		osDelay(1);
		osMutexWait(tcpConnection->connMutex, osWaitForever);
	}

	while(tcpConnection->connectionReceiving){
		if(tcpConnection->asynCbFunc == NULL){
			osMessagePut (tcpConnection->rcvMessageQId, 0, 0);
			osMutexRelease(tcpConnection->connMutex);
			osDelay(1);
			osMutexWait(tcpConnection->connMutex, osWaitForever);
		}
		else{
			break;
		}
	}
	tcpConnection->connectionReceiving = 0;
	tcpConnection->asynCbFunc = NULL;

	osTimerStop (tcpConnection->inactiveConnTimer);
	osTimerStop (tcpConnection->txTimeoutTimer);
	osTimerDelete(tcpConnection->inactiveConnTimer);
	osTimerDelete(tcpConnection->txTimeoutTimer);
	osMessageDelete(tcpConnection->sendMessageQId);
	osMessageDelete(tcpConnection->rcvMessageQId);
	netstack.routingLayerOps->rtDeleteNetAddr(tcpConnection->rxFromAddr);

	osMutexDelete(tcpConnection->connMutex);

	vPortFree(tcpConnection);

	transportOpRunning--;
	return RET_OK;
}


/**
 *
 * @param connectionHandle
 * @return
 */
static uint32_t yetiTcpCheckConnection(uint32_t connectionHandle){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) connectionHandle;
	uint32_t ret = 0;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();
	osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list*/
	if(!checkTcpConnectionFromList(tcpConnection)){
		osMutexRelease(tcpConnectionsListMutex);
		return ret;
	}
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	osMutexRelease(tcpConnectionsListMutex);

	ret = (uint32_t) tcpConnection->connected;

	osMutexRelease(tcpConnection->connMutex);
	transportOpRunning--;
	return ret;
}

/**
 *
 * @param connectionHandle
 * @param timeout
 * @return
 */
static retval_t yetiTcpSetConnectionTimeout(uint32_t connectionHandle, uint32_t timeout){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*) connectionHandle;
	if(timeout < MIN_INNACTIVE_CONNECTION_TIMEOUT){
		return RET_ERROR;
	}

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(tcpConnectionsListMutex, osWaitForever);	/*Check the connection exists in the list*/
	if(!checkTcpConnectionFromList(tcpConnection)){
		osMutexRelease(tcpConnectionsListMutex);
		transportOpRunning--;
		return RET_ERROR;
	}
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	osMutexRelease(tcpConnectionsListMutex);
	tcpConnection->innactiveConnectionTimeout = timeout;

	osMutexRelease(tcpConnection->connMutex);
	transportOpRunning--;
	return RET_OK;
}

/* Send and receive packet callbacks (from routing layer)*/
/**
 *
 * @param packetSent
 * @return
 */
static retval_t yetiTransportPacketSent(netPacket_t* packetSent){
	yetiUdpPort_t* udpPort;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	yetiTransportHdr_t* packetTransportHeader =  (yetiTransportHdr_t*)getPacketTransportHeaderPtr(packetSent);

	if(packetTransportHeader->tpPacketType == PACKET_TYPE_UDP_SEND){

		osMutexWait(udpPortsListMutex, osWaitForever);
		if((udpPort = getUdpPortFromList(packetTransportHeader->portNum)) == NULL){	/*Sent message in a wrong port. This should not happen since is previously checked in send function*/
			osMutexRelease(udpPortsListMutex);
			transportOpRunning--;
			return RET_ERROR;
		}
		osMutexRelease(udpPortsListMutex);
		if(packetSent->packetErrorFlag){
			osMessagePut (udpPort->sendMessageQId, 0, 0);
		}
		else{
			osMessagePut (udpPort->sendMessageQId, packetTransportHeader->dataSize, 0);
		}
	}
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_CONNECT){
		packetbufferReleasePacket(txPacketbuffer, packetSent);
	}
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_CONNECT_ACK){
		packetbufferReleasePacket(txPacketbuffer, packetSent);
	}
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_SEND){
		packetbufferReleasePacket(txPacketbuffer, packetSent);
	}
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_SEND_ACK){
		packetbufferReleasePacket(txPacketbuffer, packetSent);
	}
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_DISCONNECT){
		packetbufferReleasePacket(txPacketbuffer, packetSent);
	}

	transportOpRunning--;
	return RET_OK;
}

/**
 *
 * @param rcvPacket
 * @param fromAddr
 * @return
 */
static retval_t yetiTransportPacketReceived(netPacket_t* rcvPacket, netAddr_t fromAddr){

	yetiUdpPort_t* udpPort;
	yetiTcpServer_t* tcpServer;
	yetiTcpConnection_t* tcpConnection;
	netPacket_t* ackPacket;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	yetiTransportHdr_t* packetTransportHeader =  (yetiTransportHdr_t*)getPacketTransportHeaderPtr(rcvPacket);

	/*Received packet type UDP Send*/
	if(packetTransportHeader->tpPacketType == PACKET_TYPE_UDP_SEND){

		osMutexWait(udpPortsListMutex, osWaitForever);
		if((udpPort = getUdpPortFromList(packetTransportHeader->portNum)) == NULL){	/*Receive message in a wrong port*/
			osMutexRelease(udpPortsListMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		osMutexRelease(udpPortsListMutex);

		osMutexWait(udpPort->portMutex, osWaitForever);
		if(!udpPort->portReceiving){
			osMutexRelease(udpPort->portMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		if(udpPort->asynCbFunc == NULL){	/*Sync read*/
			if(packetTransportHeader->dataSize <= udpPort->rxSize){
				memcpy(udpPort->rxDataPtr, (uint8_t*)getPacketDataPtr(rcvPacket), packetTransportHeader->dataSize);	/*Copy the data from packet*/
			}
			else{
				memcpy(udpPort->rxDataPtr, (uint8_t*)getPacketDataPtr(rcvPacket), udpPort->rxSize);	/*Copy the data from packet*/
			}
			netstack.routingLayerOps->rtNetAddrCpy(udpPort->rxFromAddr, fromAddr);			/*Copy the Origin Address*/
			osMessagePut (udpPort->rcvMessageQId, packetTransportHeader->dataSize, 0);
			osMutexRelease(udpPort->portMutex);
		}
		else{								/*Async read*/
			udpPort->asynCbFunc(fromAddr, udpPort->portNumber, (uint8_t*)getPacketDataPtr(rcvPacket),
								packetTransportHeader->dataSize, 0);
			osMutexRelease(udpPort->portMutex);
		}
	}
	/*Received packet type TCP Connect to Server*/
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_CONNECT){
		/*Check if there is a server listening in this port*/
		osMutexWait(tcpServersListMutex, osWaitForever);
		if((tcpServer = getTcpServerFromList(packetTransportHeader->portNum)) == NULL){
			osMutexRelease(tcpServersListMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		osMutexWait(tcpServer->serverMutex, osWaitForever);
		osMutexRelease(tcpServersListMutex);

		if(!tcpServer->serverListening){
			osMutexRelease(tcpServer->serverMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}

		if((ackPacket = packetbufferGetFreePacket(txPacketbuffer)) == NULL){		/*Prepare the ACK packet*/
			osMutexRelease(tcpServer->serverMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}

		tcpConnection = (yetiTcpConnection_t*) pvPortMalloc(sizeof(yetiTcpConnection_t));	/*Create the new connection and store it in the server connections list*/
		tcpConnection->connMutex = ytMutexCreate();
		tcpConnection->rxDataPtr = NULL;
		tcpConnection->sendMessageQId = ytMessageqCreate(1);
		tcpConnection->rcvMessageQId = ytMessageqCreate(1);
		tcpConnection->inactiveConnTimer = ytTimerCreate(osTimerOnce, innactiveConnTimeoutCb, (void*) tcpConnection);
		tcpConnection->txTimeoutTimer = ytTimerCreate(osTimerOnce, txTimeoutCb, (void*) tcpConnection);
		tcpConnection->portNumber =	tcpServer->portNumber;
		tcpConnection->connected = 1;
		tcpConnection->remoteConnectionId = packetTransportHeader->connectionId;
		tcpConnection->connectionReceiving = 0;
		tcpConnection->waitingAck = 0;
		tcpConnection->asynCbFunc = NULL;
		tcpConnection->rxSize = 0;
		tcpConnection->innactiveConnectionTimeout = DEFAULT_INNACTIVE_CONNECTION_TIMEOUT;
		tcpConnection->yetiTcpLocalServer = tcpServer;	/*This connection belong to this local server*/
		tcpConnection->rxFromAddr = netstack.routingLayerOps->rtNewNetAddr("0", '0');	/*Empty address*/
		netstack.routingLayerOps->rtNetAddrCpy(tcpConnection->rxFromAddr, fromAddr);
		osMutexWait(tcpConnection->connMutex, osWaitForever);

		genListAdd(tcpServer->tcpConnectionsList, (void*) tcpConnection);

		osMutexWait(tcpConnectionsListMutex, osWaitForever);
		genListAdd(tcpConnectionsList, (void*) tcpConnection);	/*Also add the connection to the global connections list*/
		osMutexRelease(tcpConnectionsListMutex);

		/*Start the inactive timer*/
		osTimerStart (tcpConnection->inactiveConnTimer, tcpConnection->innactiveConnectionTimeout);

		packetTransportHeader = (yetiTransportHdr_t*)getPacketTransportHeaderPtr(ackPacket);
		packetTransportHeader->dataSize = sizeof(uint32_t);
		packetTransportHeader->portNum = tcpServer->portNumber;
		packetTransportHeader->tpPacketType = PACKET_TYPE_TCP_CONNECT_ACK;
		packetTransportHeader->connectionId = (uint32_t) tcpConnection;				/*Append my connection Id*/
		uint8_t* packetData = (uint8_t*)getPacketDataPtr(ackPacket);
		memcpy(packetData, &(tcpConnection->remoteConnectionId), sizeof(uint32_t));		/*Send in the ack packet the remote conn Id*/
		osMutexRelease(tcpConnection->connMutex);
		osMutexRelease(tcpServer->serverMutex);

		if((netstack.routingLayerOps->rtSendPacket(fromAddr, ackPacket)) != RET_OK){	/*Send the ACK packet*/
			packetbufferReleasePacket(txPacketbuffer, ackPacket);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			/*Delete the created connection*/
			osMutexWait(tcpConnection->connMutex, osWaitForever);

			osMutexWait(tcpServer->serverMutex, osWaitForever);
			genListRemove(tcpServer->tcpConnectionsList, (void*) tcpConnection);
			osMutexRelease(tcpServer->serverMutex);

			osMutexWait(tcpConnectionsListMutex, osWaitForever);
			genListRemove(tcpConnectionsList, (void*) tcpConnection);
			osMutexRelease(tcpConnectionsListMutex);

			osMessageDelete(tcpConnection->sendMessageQId);
			osMessageDelete(tcpConnection->rcvMessageQId);
			osTimerStop(tcpConnection->inactiveConnTimer);
			osTimerDelete(tcpConnection->inactiveConnTimer);
			osTimerStop(tcpConnection->txTimeoutTimer);
			osTimerDelete(tcpConnection->txTimeoutTimer);
			netstack.routingLayerOps->rtDeleteNetAddr(tcpConnection->rxFromAddr);
			osMutexDelete(tcpConnection->connMutex);
			vPortFree(tcpConnection);

			transportOpRunning--;
			return RET_ERROR;
		}

		osMutexWait(tcpServer->serverMutex, osWaitForever);
		osMutexWait(tcpConnection->connMutex, osWaitForever);
		tcpServer->serverListening--;
		osMessagePut (tcpServer->rcvMessageQId, (uint32_t) tcpConnection, 0);

		osMutexRelease(tcpConnection->connMutex);
		osMutexRelease(tcpServer->serverMutex);
	}
	/*Received packet type TCP Connect to Server ACK*/
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_CONNECT_ACK){
		/*Check if there is a connection in the connections list*/
		yetiTcpConnection_t** tcpConnectionPtr = (yetiTcpConnection_t**)getPacketDataPtr(rcvPacket);
		tcpConnection = *(tcpConnectionPtr);	/*The ack message should contain my connection id*/
		osMutexWait(tcpConnectionsListMutex, osWaitForever);
		if(!checkTcpConnectionFromList(tcpConnection)){
			osMutexRelease(tcpConnectionsListMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
		}
		osMutexWait(tcpConnection->connMutex, osWaitForever);
		osMutexRelease(tcpConnectionsListMutex);

		/*Check if the connection is already connected or the port is different (should not happen)*/
		if((tcpConnection->connected) || (tcpConnection->portNumber != packetTransportHeader->portNum)){
			osMutexRelease(tcpConnection->connMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}

		/*Reset the inactive timer*/
		osTimerStop (tcpConnection->inactiveConnTimer);
		osTimerStart (tcpConnection->inactiveConnTimer, tcpConnection->innactiveConnectionTimeout);

		tcpConnection->connected++;
		tcpConnection->remoteConnectionId = packetTransportHeader->connectionId;	/*Store the remote connection id*/

		osMessagePut (tcpConnection->rcvMessageQId, (uint32_t) tcpConnection, 0);
		osMutexRelease(tcpConnection->connMutex);
	}
	/*Received packet type TCP Send*/
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_SEND){
		/*Check if there is a connection in the connections list*/
		osMutexWait(tcpConnectionsListMutex, osWaitForever);
		if((tcpConnection = getTcpConnectionWithRemoteId(packetTransportHeader->connectionId)) == NULL){
			osMutexRelease(tcpConnectionsListMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		osMutexWait(tcpConnection->connMutex, osWaitForever);
		osMutexRelease(tcpConnectionsListMutex);

		/*Check if the connection is connected, the port is not correct or it is not reading*/
		if((!tcpConnection->connected) || (tcpConnection->portNumber != packetTransportHeader->portNum)|| (!tcpConnection->connectionReceiving)){
			osMutexRelease(tcpConnection->connMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}

		/*Reset the inactive timer*/
		osTimerStop (tcpConnection->inactiveConnTimer);
		osTimerStart (tcpConnection->inactiveConnTimer, tcpConnection->innactiveConnectionTimeout);

		if((ackPacket = packetbufferGetFreePacket(txPacketbuffer)) == NULL){		/*Prepare the ACK packet*/
			osMutexRelease(tcpConnection->connMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		uint16_t rcvPacketDataSize = packetTransportHeader->dataSize;
		packetTransportHeader = (yetiTransportHdr_t*)getPacketTransportHeaderPtr(ackPacket);
		packetTransportHeader->dataSize = rcvPacketDataSize;
		packetTransportHeader->portNum = tcpConnection->portNumber;
		packetTransportHeader->tpPacketType = PACKET_TYPE_TCP_SEND_ACK;
		packetTransportHeader->connectionId = (uint32_t) tcpConnection;
		if((netstack.routingLayerOps->rtSendPacket(fromAddr, ackPacket)) != RET_OK){	/*Send the ACK packet*/
			packetbufferReleasePacket(txPacketbuffer, ackPacket);
			osMutexRelease(tcpConnection->connMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		if(tcpConnection->asynCbFunc == NULL){
			if(rcvPacketDataSize <= tcpConnection->rxSize){
				memcpy(tcpConnection->rxDataPtr, (uint8_t*)getPacketDataPtr(rcvPacket), rcvPacketDataSize);	/*Copy the data from packet*/
			}
			else{
				memcpy(tcpConnection->rxDataPtr, (uint8_t*)getPacketDataPtr(rcvPacket), tcpConnection->rxSize);	/*Copy the data from packet*/
			}
			osMessagePut (tcpConnection->rcvMessageQId, (uint32_t) rcvPacketDataSize, 0);
			osMutexRelease(tcpConnection->connMutex);
		}
		else{
			tcpConnection->asynCbFunc(fromAddr, tcpConnection->portNumber, (uint8_t*)getPacketDataPtr(rcvPacket),
					rcvPacketDataSize, (uint32_t) tcpConnection);
			osMutexRelease(tcpConnection->connMutex);
		}

	}
	/*Received ACK packet*/
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_SEND_ACK){
		/*Check if there is a connection in the connections list*/
		osMutexWait(tcpConnectionsListMutex, osWaitForever);
		if((tcpConnection = getTcpConnectionWithRemoteId(packetTransportHeader->connectionId)) == NULL){
			osMutexRelease(tcpConnectionsListMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		osMutexWait(tcpConnection->connMutex, osWaitForever);
		osMutexRelease(tcpConnectionsListMutex);

		/*Check if the connection is connected, the port is not correct or it is not waiting for an ack*/
		if((!tcpConnection->connected) || (tcpConnection->portNumber != packetTransportHeader->portNum)|| (!tcpConnection->waitingAck)){
			osMutexRelease(tcpConnection->connMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}

		/*Reset the inactive timer*/
		osTimerStop (tcpConnection->inactiveConnTimer);
		osTimerStart (tcpConnection->inactiveConnTimer, tcpConnection->innactiveConnectionTimeout);

		osMutexRelease(tcpConnection->connMutex);

		osMessagePut (tcpConnection->sendMessageQId, (uint32_t) packetTransportHeader->dataSize, 0);	/*In an ACK message the data size of the header contains the data size of the original packet*/
	}

	/*Received DISCONNECT packet*/
	else if(packetTransportHeader->tpPacketType == PACKET_TYPE_TCP_DISCONNECT){
		osMutexWait(tcpConnectionsListMutex, osWaitForever);
		if((tcpConnection = getTcpConnectionWithRemoteId(packetTransportHeader->connectionId)) == NULL){
			osMutexRelease(tcpConnectionsListMutex);
			netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/
			transportOpRunning--;
			return RET_ERROR;
		}
		osMutexRelease(tcpConnectionsListMutex);

		yetiTcpCloseConnection((uint32_t) tcpConnection);
	}

	netstack.routingLayerOps->rtRcvPacket(); /*Continue reading packets*/

	transportOpRunning--;
	return RET_OK;
}

/**
 *
 * @param argument
 */
static void innactiveConnTimeoutCb(void const * argument){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*)argument;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	/*Create a thread to close the connection, instead of doing it in the timer callback*/
	/*The timers cannot be deleted from a timer function.*/
	/*In addition, the close connection func may take some time, so it is better to do it in another thread instead of the timer thread */
	ytStartThread("CloseConnThread", closeConnThreadCb, osPriorityLow, 200,	NULL, (void*) tcpConnection);

	transportOpRunning--;
}


/**
 *
 * @param argument
 */
static void txTimeoutCb(void const * argument){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*)argument;

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	/*Check if there is a connection in the connections list*/
	osMutexWait(tcpConnectionsListMutex, osWaitForever);
	if(!checkTcpConnectionFromList(tcpConnection)){
		osMutexRelease(tcpConnectionsListMutex);
		transportOpRunning--;
		return;
	}
	osMutexWait(tcpConnection->connMutex, osWaitForever);
	osMutexRelease(tcpConnectionsListMutex);

	if((!tcpConnection->connected)){						/*If not connected it is waiting for connect to server ack*/
		osMessagePut (tcpConnection->rcvMessageQId, 0, 0);	/*Return error in the connect to server function that is waiting an ack*/
		osMutexRelease(tcpConnection->connMutex);
		transportOpRunning--;
		return ;
	}
	else{	/*Check we are waitting for a message ack*/
		if(tcpConnection->waitingAck){
			osMessagePut (tcpConnection->sendMessageQId, 0xFFFFFFFF, 0);	/*Return 0xFFFFFFFF to indicate the ack has not been received so the packet is sent again*/
			osMutexRelease(tcpConnection->connMutex);
			transportOpRunning--;
			return;
		}
	}
}

/**
 *
 * @param argument
 */
static void closeConnThreadCb(void const * argument){
	yetiTcpConnection_t* tcpConnection = (yetiTcpConnection_t*)argument;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!yetiTransportStarted){
		taskEXIT_CRITICAL();
		return;
	}
	transportOpRunning++;
	taskEXIT_CRITICAL();

	yetiTcpCloseConnection((uint32_t) tcpConnection);

	transportOpRunning--;

	osThreadTerminate(osThreadGetId());
}

/**
 *
 * @param portNum
 * @return
 */
static yetiUdpPort_t* getUdpPortFromList(uint16_t portNum){
	genListElement_t* current = udpPortsList->tailElement;
	yetiUdpPort_t* udpPort;

	while(current != NULL){
		udpPort = (yetiUdpPort_t*) current->item;
		if(udpPort->portNumber == portNum){
			return udpPort;
		}
		current = current->next;
	}

	return NULL;
}

/**
 *
 * @param portNumber
 * @return
 */
static 	yetiTcpServer_t* getTcpServerFromList(uint16_t portNumber){
	genListElement_t* current = tcpServersList->tailElement;
	yetiTcpServer_t* tcpServer;

	while(current != NULL){
		tcpServer = (yetiTcpServer_t*) current->item;
		if(tcpServer->portNumber == portNumber){
			return tcpServer;
		}
		current = current->next;
	}

	return NULL;
}

/**
 *
 * @param tcpServer
 * @return
 */
static uint16_t checkTcpServerFromList(yetiTcpServer_t* tcpServer){
	genListElement_t* current = tcpServersList->tailElement;
	if(tcpServer == NULL){
		return 0;
	}
	while(current != NULL){
		if(tcpServer ==  ((yetiTcpServer_t*) current->item)){
			return 1;
		}
		current = current->next;
	}

	return 0;
}

/**
 *
 * @param tcpConnection
 * @return
 */
static uint16_t checkTcpConnectionFromList(yetiTcpConnection_t* tcpConnection){
	genListElement_t* current = tcpConnectionsList->tailElement;
	if(tcpConnection == NULL){
		return 0;
	}

	while(current != NULL){
		if(tcpConnection ==  ((yetiTcpConnection_t*) current->item)){
			return 1;
		}
		current = current->next;
	}

	return 0;
}

/**
 *
 * @param remoteConnectionId
 * @return
 */
static yetiTcpConnection_t* getTcpConnectionWithRemoteId(uint32_t remoteConnectionId){
	genListElement_t* current = tcpConnectionsList->tailElement;
	yetiTcpConnection_t* tcpConnection;

	while(current != NULL){
		tcpConnection = (yetiTcpConnection_t*) current->item;
		if(tcpConnection->remoteConnectionId == remoteConnectionId){
			return tcpConnection;
		}
		current = current->next;
	}

	return NULL;
}
