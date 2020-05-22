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
 * netstack.h
 *
 *  Created on: 10 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file netstack.h
 */
#ifndef YETIOS_NET_INC_NETSTACK_H_
#define YETIOS_NET_INC_NETSTACK_H_

#include "packetbuffer.h"


#define RT_BROADCAST_ADDR		0xFFFF
#define RT_BROADCAST_ADDR_STR	"0xFFFF"

#define MAC_BROADCAST_ADDR_STR	"0xFFFFFFFF"	/*These addresses are defined for all MAC layers*/
#define MAC_FLOOD_ADDR_STR		"0xFFFFFFFE"
#define MAC_BROADCAST_ADDR		0xFFFFFFFF
#define MAC_FLOOD_ADDR			0xFFFFFFFE


typedef void* macAddr_t;
typedef void* netAddr_t;

typedef retval_t (*netReadCbFunc_t)(netAddr_t fromAddr, uint16_t portNumber, uint8_t* data, uint32_t size, uint32_t connectionHandle);

typedef struct transportLayerOps{
	retval_t (*transportLayerInit)(void);
	retval_t (*transportLayerDeInit)(void);

	/* UDP like funcs */
	retval_t (*udpOpenPort)(uint16_t portNumber);
	retval_t (*udpClosePort)(uint16_t portNumber);
	uint32_t (*udpSend)(netAddr_t destAddr, uint16_t portNumber, uint8_t* data, uint32_t size);
	uint32_t (*udpRcv)(netAddr_t fromAddr, uint16_t portNumber, uint8_t* data, uint32_t size);
	retval_t (*udpRcvAsync)(uint16_t portNumber, netReadCbFunc_t readCbFunc);
	retval_t (*udpStopRcvAsync)(uint16_t portNumber);
	/* TCP like funcs */
	uint32_t (*tcpCreateServer)(uint16_t portNumber);
	retval_t (*tcpDeleteServer)(uint32_t serverHandle);
	uint32_t (*tcpListenConnection)(uint32_t serverHandle, netAddr_t fromAddr);
	uint32_t (*tcpConnectServer)(netAddr_t destAddr, uint16_t portNumber);
	uint32_t (*tcpSend)(uint32_t connectionHandle, uint8_t* data, uint32_t size);
	uint32_t (*tcpRcv)(uint32_t connectionHandle, uint8_t* data, uint32_t size);
	retval_t (*tcpRcvAsync)(uint32_t connectionHandle, netReadCbFunc_t readCbFunc);
	retval_t (*tcpStopRcvAsync)(uint32_t connectionHandle);
	retval_t (*tcpCloseConnection)(uint32_t connectionHandle);
	uint32_t (*tcpCheckConnection)(uint32_t connectionHandle);
	retval_t (*tcpSetConnectionTimeout)(uint32_t connectionHandle, uint32_t timeout);

	/* Send and receive packet callbacks (from routing layer)*/
	retval_t (*transportPacketSent)(netPacket_t* packetSent);
	retval_t (*transportPacketReceived)(netPacket_t* rcvPacket, netAddr_t fromAddr);

}transportLayerOps_t;



typedef struct routingLayerOps_{
	retval_t (*rtLayerInit)(void);
	retval_t (*rtLayerDeInit)(void);

	retval_t (*rtSendPacket)(netAddr_t addr, netPacket_t* packet);
	retval_t (*rtRcvPacket)(void);

	retval_t (*rtPacketSent)(netPacket_t* packet);
	retval_t (*rtPacketReceived)(netPacket_t* packet, macAddr_t macFromAddr);

	netAddr_t (*rtGetNodeAddr)(uint16_t index);
	retval_t (*rtAddNodeAddr)(netAddr_t newAddr);
	retval_t (*rtRemoveNodeAddr)(netAddr_t addr);
	retval_t (*rtAddRoute)(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops);
	retval_t (*rtRemoveRoute)(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops);

	netAddr_t (*rtNewNetAddr)(char* addrStr, char format);
	retval_t (*rtDeleteNetAddr)(netAddr_t netAddr);
	retval_t (*rtNetAddrToString)(netAddr_t netAddr, char* addrStr, char format);
	retval_t (*rtNetAddrCpy)(netAddr_t destAddr, netAddr_t fromAddr);
	uint16_t (*rtNetAddrCmp)(netAddr_t aAddr, netAddr_t bAddr);

	retval_t (*rtSetNodeAsGw)(void);
}routingLayerOps_t;


typedef struct macLayerOps_{
	retval_t (*macLayerInit)(void);
	retval_t (*macLayerDeInit)(void);

	retval_t (*macSendPacket)(macAddr_t macAddrDest, netPacket_t* packet);
	retval_t (*macRcvPacket)(void);

	retval_t (*macPacketSent)(netPacket_t* packet, uint16_t size);
	retval_t (*macPacketReceived)(netPacket_t* packet, uint16_t size);

	retval_t (*macSetAddr)(macAddr_t macAddr);

	macAddr_t (*macNewAddr)(char* macAddrStr, char format);
	retval_t (*macDeleteAddr)(macAddr_t macAddr);
	retval_t (*macAddrToString)(macAddr_t macAddr, char* macAddrStr, char format);
	retval_t (*macAddrCpy)(macAddr_t destAddr, macAddr_t fromAddr);
	uint16_t (*macAddrCmp)(macAddr_t aAddr, macAddr_t bAddr);
	retval_t (*macSetNodeAsGw)(void);
	uint16_t (*macIsNodeLinked)(void);
	retval_t (*macSetDutyCycle)(uint32_t dutyCycle);
	retval_t (*macSetProcessPriority)(osPriority priority);
	osPriority (*macGetProcessPriority)(void);
}macLayerOps_t;


typedef struct phyLayerOps_{
	retval_t (*phyLayerInit)(void);
	retval_t (*phyLayerDeInit)(void);

	retval_t (*phySendPacket)(netPacket_t* packet, uint16_t size);

	retval_t (*phyPacketSent)(netPacket_t* packet, uint16_t size);
	retval_t (*phyPacketReceived)(netPacket_t* packet, uint16_t size);

	retval_t (*phySetModeReceiving)(void);
	retval_t (*phySetModeIdle)(void);
	retval_t (*phySetModeSleep)(void);

	retval_t (*phyCheckChannelRssi)(float32_t* rssi);
	retval_t (*phyGetLastRssi)(float32_t* rssi);

	retval_t (*phySetBaseFreq)(uint32_t baseFreq);
	retval_t (*phyGetBaseFreq)(uint32_t* baseFreq);

	retval_t (*phyGetChannelNum)(uint32_t* channelNum);
	retval_t (*phySetFreqChannel)(uint32_t channelNum);
	retval_t (*phyGetFreqChannel)(uint32_t* channelNum);

	retval_t (*phySetBaudRate)(uint32_t baudRate);
	retval_t (*phySetOutPower)(int16_t outPower);

	retval_t (*phyEncryptPacket)(netPacket_t* packet, uint8_t* key, uint16_t size);
	retval_t (*phyDecryptPacket)(netPacket_t* packet, uint8_t* key, uint16_t size);

	retval_t (*phySetProcessPriority)(osPriority priority);
	osPriority (*phyGetProcessPriority)(void);
}phyLayerOps_t;


typedef struct netstack_{
	transportLayerOps_t* transportLayerOps;
	routingLayerOps_t* routingLayerOps;
	macLayerOps_t* macLayerOps;
	phyLayerOps_t* phyLayerOps;
}netstack_t;


netstack_t netstack;

packetbuffer_t* rxPacketbuffer;
packetbuffer_t* txPacketbuffer;

retval_t ytNetstackSetStartupLayers(void);

retval_t ytNetstackInit(void);
retval_t ytNetstackDeInit(void);

retval_t ytRegisterTransportLayer(transportLayerOps_t* transportLayerOps);
retval_t ytUnregisterTransportLayer(transportLayerOps_t* transportLayerOps);

retval_t ytRegisterRoutingLayer(routingLayerOps_t* routingLayerOps);
retval_t ytUnregisterRoutingLayer(routingLayerOps_t* routingLayerOps);

retval_t ytRegisterMacLayer(macLayerOps_t* macLayerOps);
retval_t ytUnregisterMacLayer(macLayerOps_t* macLayerOps);

retval_t ytRegisterPhyLayer(phyLayerOps_t* phyLayerOps);
retval_t ytUnregisterPhyLayer(phyLayerOps_t* phyLayerOps);

#endif /* YETIOS_NET_INC_NETSTACK_H_ */
