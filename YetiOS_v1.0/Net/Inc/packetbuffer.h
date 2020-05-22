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
 * packetbuffer.h
 *
 *  Created on: 10 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file packetbuffer.h
 */
#ifndef YETIOS_NET_INC_PACKETBUFFER_H_
#define YETIOS_NET_INC_PACKETBUFFER_H_

#include "yetiOS.h"

#define FIXED_PACKET_SIZE	63

#define PACKETBUFFER_SIZE				3			/*Number of packets in each packet buffer: 4 packets => 256 bytes*/

struct  packetbuffer_;

typedef struct netPacket_{
	/* Exra data used internally. This wont be sent in a packet. These data can be used to implement cross-layer techniques */
	void* tpExtraInfo;
	void* rtExtraInfo;
	void* macExtraInfo;
	void* phyExtraInfo;
	/*Tick time when this packet was received*/
	uint32_t rcvTickTime;
	uint16_t usedPacketFlag;
	uint16_t packetErrorFlag;
	/*Packet data actually sent*/
	uint8_t packetData[FIXED_PACKET_SIZE];	/*The packetData is organized as follows: MAC_HDR + ROUTING_HDR + TRANSPORT_HDR + APP_DATA*/
	/*Pointer to the packetbuffer this packet belongs to*/
	struct  packetbuffer_* packetbuffer;
}netPacket_t;

typedef struct  packetbuffer_{
	netPacket_t packetbuffer[PACKETBUFFER_SIZE];
	osMutexId packetbufferMutex;
	uint16_t currentNumPackets;
	uint16_t packetDataSize;

	uint16_t dataIndex;
	uint16_t routingHeaderIndex;
	uint16_t transportHeaderIndex;
	/*The mac header is always the first byte in the packet data array*/

}packetbuffer_t;


packetbuffer_t* newPacketbuffer(void);
retval_t deletePacketbuffer(packetbuffer_t* packetbuffer);

netPacket_t* packetbufferGetFreePacket(packetbuffer_t* packetbuffer);
retval_t packetbufferReleasePacket(packetbuffer_t* packetbuffer, netPacket_t* packet);

retval_t setMacHdrSize(packetbuffer_t* packetbuffer, uint16_t macHdrSize);
retval_t setRoutingHdrSize(packetbuffer_t* packetbuffer, uint16_t routingHdrSize);
retval_t setTransportHdrSize(packetbuffer_t* packetbuffer, uint16_t transportHdrSize);

void* getPacketDataPtr(netPacket_t* packet);
void* getPacketMacHeaderPtr(netPacket_t* packet);
void* getPacketRoutingHeaderPtr(netPacket_t* packet);
void* getPacketTransportHeaderPtr(netPacket_t* packet);


#endif /* YETIOS_NET_INC_PACKETBUFFER_H_ */
