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
 * netstack.c
 *
 *  Created on: 10 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file netstack.c
 */

#include "yetiOS.h"
#include "netstack.h"


netstack_t netstack;

packetbuffer_t* rxPacketbuffer;
packetbuffer_t* txPacketbuffer;


static osMutexId netstackMutex;
static uint16_t opRunning = 0;
static uint16_t netstackStarted = 0;

extern phyLayerOps_t YETIOS_PHY_DEFAULT_LAYER;
extern macLayerOps_t YETIOS_MAC_DEFAULT_LAYER;
extern routingLayerOps_t YETIOS_ROUTING_DEFAULT_LAYER;
extern transportLayerOps_t YETIOS_TRANSPORT_DEFAULT_LAYER;


/**
 *
 * @return
 */
retval_t ytNetstackSetStartupLayers(void){

#ifndef YETIOS_PHY_DEFAULT_LAYER
	return RET_ERROR;
#endif
#ifndef YETIOS_MAC_DEFAULT_LAYER
	return RET_ERROR;
#endif
#ifndef YETIOS_ROUTING_DEFAULT_LAYER
	return RET_ERROR;
#endif
#ifndef YETIOS_TRANSPORT_DEFAULT_LAYER
	return RET_ERROR;
#endif

	netstack.phyLayerOps = &YETIOS_PHY_DEFAULT_LAYER;
	netstack.macLayerOps = &YETIOS_MAC_DEFAULT_LAYER;
	netstack.routingLayerOps = &YETIOS_ROUTING_DEFAULT_LAYER;
	netstack.transportLayerOps = &YETIOS_TRANSPORT_DEFAULT_LAYER;

	netstackMutex = ytMutexCreate();
	return RET_OK;
}

/**
 *
 * @return
 */
retval_t ytNetstackInit(){
	osMutexWait(netstackMutex, osWaitForever);
	if(netstackStarted){
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	rxPacketbuffer = newPacketbuffer();
	txPacketbuffer = newPacketbuffer();

	if(netstack.phyLayerOps->phyLayerInit() != RET_OK){
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
#if YETIOS_ENABLE_STDIO
		ytPrintf(">ERROR: Unable to init PHY layer");
#endif
		return RET_ERROR;
	}

	if(netstack.macLayerOps->macLayerInit() != RET_OK){
		netstack.phyLayerOps->phyLayerDeInit();
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
#if YETIOS_ENABLE_STDIO
		ytPrintf(">ERROR: Unable to init MAC layer");
#endif
		return RET_ERROR;
	}

	if(netstack.routingLayerOps->rtLayerInit() != RET_OK){
		netstack.macLayerOps->macLayerDeInit();
		netstack.phyLayerOps->phyLayerDeInit();
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
#if YETIOS_ENABLE_STDIO
		ytPrintf(">ERROR: Unable to init ROUTING layer");
#endif
		return RET_ERROR;
	}

	if(netstack.transportLayerOps->transportLayerInit() != RET_OK){
		netstack.routingLayerOps->rtLayerDeInit();
		netstack.macLayerOps->macLayerDeInit();
		netstack.phyLayerOps->phyLayerDeInit();
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
#if YETIOS_ENABLE_STDIO
		ytPrintf(">ERROR: Unable to init TRANSPORT layer");
#endif
		return RET_ERROR;
	}

	netstackStarted++;
	opRunning = 0;
	osMutexRelease(netstackMutex);

	return RET_OK;
}

/**
 *
 * @return
 */
retval_t ytNetstackDeInit(void){
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	while(opRunning){					/*Not allowed while any op is running*/
		osMutexRelease(netstackMutex);
		osDelay(1);
		osMutexWait(netstackMutex, osWaitForever);
	}


	netstack.transportLayerOps->transportLayerDeInit();
	netstack.routingLayerOps->rtLayerDeInit();
	netstack.macLayerOps->macLayerDeInit();
	netstack.phyLayerOps->phyLayerDeInit();
	deletePacketbuffer(rxPacketbuffer);
	deletePacketbuffer(txPacketbuffer);

	netstackStarted = 0;
	osMutexRelease(netstackMutex);

	return RET_OK;
}

/**
 *
 * @param transportLayerOps
 * @return
 */
retval_t ytRegisterTransportLayer(transportLayerOps_t* transportLayerOps){
	osMutexWait(netstackMutex, osWaitForever);

	while(opRunning){			/*Not allowed while any op is running*/
		osMutexRelease(netstackMutex);
		osDelay(1);
		osMutexWait(netstackMutex, osWaitForever);
	}
	if(!netstackStarted){		/*Only link the layer to the netstack if it is not initialized*/
		netstack.transportLayerOps = transportLayerOps;
		osMutexRelease(netstackMutex);
		return RET_OK;
	}

	/*De init the previous layer*/
	netstack.transportLayerOps->transportLayerDeInit();

	/*Set the new layer*/
	netstack.transportLayerOps = transportLayerOps;

	/*Init the new Layer. If not able the full netstack is deinit*/
	if(netstack.transportLayerOps->transportLayerInit() != RET_OK){
		netstack.routingLayerOps->rtLayerDeInit();
		netstack.macLayerOps->macLayerDeInit();
		netstack.phyLayerOps->phyLayerDeInit();
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
		ytPrintf(">ERROR: Unable to init TRANSPORT layer");
		return RET_ERROR;
	}

	osMutexRelease(netstackMutex);
	return RET_OK;
}

/**
 *
 * @param routingLayerOps
 * @return
 */
retval_t ytRegisterRoutingLayer(routingLayerOps_t* routingLayerOps){
	osMutexWait(netstackMutex, osWaitForever);

	while(opRunning){			/*Not allowed while any op is running*/
		osMutexRelease(netstackMutex);
		osDelay(1);
		osMutexWait(netstackMutex, osWaitForever);
	}
	if(!netstackStarted){		/*Only link the layer to the netstack if it is not initialized*/
		netstack.routingLayerOps = routingLayerOps;
		osMutexRelease(netstackMutex);
		return RET_OK;
	}

	/*De init the previous layer*/
	netstack.routingLayerOps->rtLayerDeInit();

	/*Set the new layer*/
	netstack.routingLayerOps = routingLayerOps;

	/*Init the new Layer. If not able the full netstack is deinit*/
	if(netstack.routingLayerOps->rtLayerInit() != RET_OK){
		netstack.macLayerOps->macLayerDeInit();
		netstack.phyLayerOps->phyLayerDeInit();
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
		ytPrintf(">ERROR: Unable to init ROUTING layer");
		return RET_ERROR;
	}

	osMutexRelease(netstackMutex);
	return RET_OK;
}


/**
 *
 * @param macLayerOps
 * @return
 */
retval_t ytRegisterMacLayer(macLayerOps_t* macLayerOps){
	osMutexWait(netstackMutex, osWaitForever);

	while(opRunning){			/*Not allowed while any op is running*/
		osMutexRelease(netstackMutex);
		osDelay(1);
		osMutexWait(netstackMutex, osWaitForever);
	}
	if(!netstackStarted){		/*Only link the layer to the netstack if it is not initialized*/
		netstack.macLayerOps = macLayerOps;
		osMutexRelease(netstackMutex);
		return RET_OK;
	}

	/*De init the previous layer*/
	netstack.macLayerOps->macLayerDeInit();

	/*Set the new layer*/
	netstack.macLayerOps = macLayerOps;

	/*Init the new Layer. If not able the full netstack is deinit*/
	if(netstack.macLayerOps->macLayerInit() != RET_OK){
		netstack.phyLayerOps->phyLayerDeInit();
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
		ytPrintf(">ERROR: Unable to init MAC layer");
		return RET_ERROR;
	}

	osMutexRelease(netstackMutex);
	return RET_OK;
}


/**
 *
 * @param phyLayerOps
 * @return
 */
retval_t ytRegisterPhyLayer(phyLayerOps_t* phyLayerOps){
	osMutexWait(netstackMutex, osWaitForever);

	while(opRunning){			/*Not allowed while any op is running*/
		osMutexRelease(netstackMutex);
		osDelay(1);
		osMutexWait(netstackMutex, osWaitForever);
	}
	if(!netstackStarted){		/*Only link the layer to the netstack if it is not initialized*/
		netstack.phyLayerOps = phyLayerOps;
		osMutexRelease(netstackMutex);
		return RET_OK;
	}

	/*De init the previous layer*/
	netstack.phyLayerOps->phyLayerDeInit();

	/*Set the new layer*/
	netstack.phyLayerOps = phyLayerOps;

	/*Init the new Layer. If not able the full netstack is deinit*/
	if(netstack.phyLayerOps->phyLayerInit() != RET_OK){
		deletePacketbuffer(rxPacketbuffer);
		deletePacketbuffer(txPacketbuffer);
		osMutexRelease(netstackMutex);
		ytPrintf(">ERROR: Unable to init MAC layer");
		return RET_ERROR;
	}

	osMutexRelease(netstackMutex);
	return RET_OK;
}


/**
 *
 * @param port_number
 * @return
 */
retval_t ytUdpOpen(uint16_t portNumber){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->udpOpenPort(portNumber);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;

}

/**
 *
 * @param port_number
 * @return
 */
retval_t ytUdpClose(uint16_t portNumber){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->udpClosePort(portNumber);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}



/**
 *
 * @param portNumber
 * @param destAddr
 * @param data
 * @param size
 * @return
 */
uint32_t ytUdpSend(netAddr_t destAddr, uint16_t portNumber, uint8_t* data, uint32_t size){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->udpSend(destAddr, portNumber, data, size);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}


/**
 *
 * @param portNumber
 * @param fromAddr
 * @param data
 * @param size
 * @return
 */
uint32_t ytUdpRcv(netAddr_t fromAddr, uint16_t portNumber, uint8_t* data, uint32_t size){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->udpRcv(fromAddr, portNumber, data, size);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
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
retval_t ytUdpRcvAsync(uint16_t portNumber, netReadCbFunc_t readCbFunc){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->udpRcvAsync(portNumber, readCbFunc);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param portNumber
 * @return
 */
retval_t ytUdpStopRcvAsync(uint16_t portNumber){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->udpStopRcvAsync(portNumber);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/*TCP like RELIABLE CONNECTION FUNCS */
/**
 *
 * @param portNumber
 * @return
 */
uint32_t ytTcpCreateServer(uint16_t portNumber){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpCreateServer(portNumber);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param serverFd
 * @return
 */
retval_t ytTcpDeleteServer(uint32_t serverFd){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpDeleteServer(serverFd);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param serverFd
 * @param fromAddr
 * @return
 */
uint32_t ytTcpListenConnection(uint32_t serverFd, netAddr_t fromAddr){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpListenConnection(serverFd, fromAddr);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param destAddr
 * @param portNumber
 * @return
 */
uint32_t ytTcpConnectServer(netAddr_t destAddr, uint16_t portNumber){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpConnectServer(destAddr, portNumber);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param connectionFd
 * @return
 */
retval_t ytTcpCloseConnection(uint32_t connectionFd){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpCloseConnection(connectionFd);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param connectionFd
 * @param data
 * @param size
 * @return
 */
uint32_t ytTcpSend(uint32_t connectionFd, uint8_t* data, uint32_t size){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpSend(connectionFd, data, size);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param connectionFd
 * @param data
 * @param size
 * @return
 */
uint32_t ytTcpRcv(uint32_t connectionFd, uint8_t* data, uint32_t size){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpRcv(connectionFd, data, size);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param connectionFd
 * @param data
 * @param size
 * @param readCbFunc
 * @param args
 * @return
 */
retval_t ytTcpRcvAsync(uint32_t connectionFd, netReadCbFunc_t readCbFunc){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpRcvAsync(connectionFd, readCbFunc);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param connectionFd
 * @return
 */
retval_t ytTcpStopRcvAsync(uint32_t connectionFd){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpStopRcvAsync(connectionFd);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param connectionFd
 * @return
 */
uint32_t ytTcpCheckConnection(uint32_t connectionFd){
	uint32_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpCheckConnection(connectionFd);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param connectionFd
 * @param timeout
 * @return
 */
retval_t ytTcpSetConnectionTimeout(uint32_t connectionFd, uint32_t timeout){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.transportLayerOps->tcpSetConnectionTimeout(connectionFd, timeout);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/*Network Addresses Functions */
/**
 *
 * @param netAddrStr
 * @param format
 * @return
 */
netAddr_t ytNetNewAddr(char* netAddrStr, char format){
	netAddr_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return NULL;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtNewNetAddr(netAddrStr, format);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @return
 */
netAddr_t ytNetNewEmptyAddr(void){
	netAddr_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return NULL;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtNewNetAddr("0", '0');	/*Indicates empty address*/

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param netAddr
 * @return
 */
retval_t ytNetDeleteNetAddr(netAddr_t netAddr){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtDeleteNetAddr(netAddr);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param netAddr
 * @param netAddrStr
 * @param format
 * @return
 */
retval_t ytNetAddrToString(netAddr_t netAddr, char* netAddrStr, char format){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtNetAddrToString(netAddr, netAddrStr, format);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param destAddr
 * @param fromAddr
 * @return
 */
retval_t ytNetAddrCpy(netAddr_t destAddr, netAddr_t fromAddr){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtNetAddrCpy(destAddr, fromAddr);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param aAddr
 * @param bAddr
 * @return
 */
uint16_t ytNetAddrCmp(netAddr_t aAddr, netAddr_t bAddr){
	uint16_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtNetAddrCpy(aAddr, bAddr);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param index
 * @return
 */
netAddr_t ytNetGetNodeAddr(uint16_t index){
	netAddr_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return NULL;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtGetNodeAddr(index);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param nodeAddr
 * @return
 */
retval_t ytNetAddNodeAddr(netAddr_t nodeAddr){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtAddNodeAddr(nodeAddr);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param nodeAddr
 * @return
 */
retval_t ytNetRemoveNodeAddr(netAddr_t nodeAddr){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtRemoveNodeAddr(nodeAddr);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param destAddr
 * @param nextAddr
 * @param hops
 * @return
 */
retval_t ytNetAddRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtAddRoute(destAddr, nextAddr, hops);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @param destAddr
 * @param nextAddr
 * @param hops
 * @return
 */
retval_t ytNetRemoveRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtRemoveRoute(destAddr, nextAddr, hops);

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

/**
 *
 * @return
 */
retval_t ytNetSetNodeAsGw(void){
	retval_t ret;
	osMutexWait(netstackMutex, osWaitForever);
	if(!netstackStarted){		/*The netstack must be initialized*/
		osMutexRelease(netstackMutex);
		return RET_ERROR;
	}
	opRunning++;
	osMutexRelease(netstackMutex);

	ret = netstack.routingLayerOps->rtSetNodeAsGw();

	osMutexWait(netstackMutex, osWaitForever);
	opRunning--;
	osMutexRelease(netstackMutex);
	return ret;
}

