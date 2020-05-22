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
 * packetbuffer.c
 *
 *  Created on: 10 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file packetbuffer.c
 */

#include "yetiOS.h"
#include "packetbuffer.h"

/**
 *
 * @return
 */
packetbuffer_t* newPacketbuffer(void){
	uint16_t i;
	packetbuffer_t* newPacketbuffer = (packetbuffer_t*)pvPortMalloc(sizeof(packetbuffer_t));
	newPacketbuffer->packetbufferMutex = ytMutexCreate();
	newPacketbuffer->currentNumPackets = 0;
	newPacketbuffer->packetDataSize = 0;

	newPacketbuffer->dataIndex = 0xFFFF;	/*The headers indexes are not initialized yet*/
	newPacketbuffer->routingHeaderIndex = 0xFFFF;
	newPacketbuffer->transportHeaderIndex = 0xFFFF;

	/*Set the packetbuffer reference to each packet*/
	for(i=0; i < PACKETBUFFER_SIZE; i++){
		newPacketbuffer->packetbuffer[i].packetbuffer = newPacketbuffer;
	}

	/*The mac header is always the first byte in the packet data array*/
	return newPacketbuffer;
}

/**
 *
 * @param packetbuffer
 * @return
 */
retval_t deletePacketbuffer(packetbuffer_t* packetbuffer){
	osMutexWait(packetbuffer->packetbufferMutex, osWaitForever);
	osMutexDelete(packetbuffer->packetbufferMutex);
	vPortFree(packetbuffer);
	return RET_OK;
}

/**
 *
 * @param packetbuffer
 * @return
 */
netPacket_t* packetbufferGetFreePacket(packetbuffer_t* packetbuffer){
	uint16_t i;
	osMutexWait(packetbuffer->packetbufferMutex, osWaitForever);
	if((packetbuffer->currentNumPackets > PACKETBUFFER_SIZE) || (packetbuffer->dataIndex == 0xFFFF)){
		osMutexRelease(packetbuffer->packetbufferMutex);
		return NULL;
	}

	for(i=0; i<PACKETBUFFER_SIZE; i++){
		if(!packetbuffer->packetbuffer[i].usedPacketFlag){
			packetbuffer->packetbuffer[i].usedPacketFlag++;
			packetbuffer->currentNumPackets++;

			packetbuffer->packetbuffer[i].packetErrorFlag = 0;
			osMutexRelease(packetbuffer->packetbufferMutex);

			return &(packetbuffer->packetbuffer[i]);
		}
	}

	osMutexRelease(packetbuffer->packetbufferMutex);
	return NULL;
}

/**
 *
 * @param packetbuffer
 * @param packet
 * @return
 */
retval_t packetbufferReleasePacket(packetbuffer_t* packetbuffer, netPacket_t* packet){
	uint16_t i;
	osMutexWait(packetbuffer->packetbufferMutex, osWaitForever);
	if(packetbuffer->dataIndex == 0xFFFF){
		osMutexRelease(packetbuffer->packetbufferMutex);
		return RET_ERROR;
	}
	for(i=0; i<PACKETBUFFER_SIZE; i++){
		if((&(packetbuffer->packetbuffer[i])) == packet){
			if(packetbuffer->packetbuffer[i].usedPacketFlag){
				packetbuffer->packetbuffer[i].usedPacketFlag--;
				packetbuffer->currentNumPackets--;

				osMutexRelease(packetbuffer->packetbufferMutex);
				return RET_OK;
			}
		}
	}
	osMutexRelease(packetbuffer->packetbufferMutex);
	return RET_ERROR;
}

/**
 *
 * @param packetbuffer
 * @param macHdrSize
 * @return
 */
retval_t setMacHdrSize(packetbuffer_t* packetbuffer, uint16_t macHdrSize){
	uint16_t oldRoutingHeaderIndex;
	osMutexWait(packetbuffer->packetbufferMutex, osWaitForever);
	if(macHdrSize >= (FIXED_PACKET_SIZE/2)){
		osMutexRelease(packetbuffer->packetbufferMutex);
		return RET_ERROR;
	}
	oldRoutingHeaderIndex = packetbuffer->routingHeaderIndex;
	packetbuffer->routingHeaderIndex = macHdrSize;

	/*Update the other indexes if they are already initialized*/
	if(packetbuffer->transportHeaderIndex != 0xFFFF){
		packetbuffer->transportHeaderIndex = (packetbuffer->transportHeaderIndex - oldRoutingHeaderIndex) + packetbuffer->routingHeaderIndex;
		if(packetbuffer->transportHeaderIndex >= FIXED_PACKET_SIZE){	/*Exceeded packet size. Not allowed.*/
			/*Return indexes to their original values*/
			packetbuffer->transportHeaderIndex = oldRoutingHeaderIndex + (packetbuffer->transportHeaderIndex - packetbuffer->routingHeaderIndex);
			packetbuffer->routingHeaderIndex = oldRoutingHeaderIndex;
			osMutexRelease(packetbuffer->packetbufferMutex);
			return RET_ERROR;
		}

		if(packetbuffer->dataIndex != 0xFFFF){
			packetbuffer->dataIndex = (packetbuffer->dataIndex - oldRoutingHeaderIndex) + packetbuffer->routingHeaderIndex;

			if(packetbuffer->dataIndex >= FIXED_PACKET_SIZE){	/*Exceeded packet size. Not allowed.*/
				/*Return indexes to their original values*/
				packetbuffer->dataIndex = oldRoutingHeaderIndex + (packetbuffer->dataIndex - packetbuffer->routingHeaderIndex);
				packetbuffer->transportHeaderIndex = oldRoutingHeaderIndex + (packetbuffer->transportHeaderIndex - packetbuffer->routingHeaderIndex);
				packetbuffer->routingHeaderIndex = oldRoutingHeaderIndex;
				osMutexRelease(packetbuffer->packetbufferMutex);
				return RET_ERROR;
			}

			packetbuffer->packetDataSize = FIXED_PACKET_SIZE - packetbuffer->dataIndex;
		}
	}

	osMutexRelease(packetbuffer->packetbufferMutex);
	return RET_OK;
}

/**
 *
 * @param packetbuffer
 * @param routingHdrSize
 * @return
 */
retval_t setRoutingHdrSize(packetbuffer_t* packetbuffer, uint16_t routingHdrSize){
	uint16_t oldTransportHeaderIndex;
	osMutexWait(packetbuffer->packetbufferMutex, osWaitForever);
	if((routingHdrSize >= (FIXED_PACKET_SIZE/2)) || (packetbuffer->routingHeaderIndex == 0xFFFF)){	/*The MAC Header size (which sets routingheadrindex) must have been initialized before*/
		osMutexRelease(packetbuffer->packetbufferMutex);
		return RET_ERROR;
	}
	oldTransportHeaderIndex = packetbuffer->transportHeaderIndex;
	packetbuffer->transportHeaderIndex = packetbuffer->routingHeaderIndex + routingHdrSize;
	if(packetbuffer->transportHeaderIndex >= FIXED_PACKET_SIZE){	/*Exceeded packet size. Not allowed.*/
		/*Return indexes to their original values*/
		packetbuffer->transportHeaderIndex = oldTransportHeaderIndex;
		osMutexRelease(packetbuffer->packetbufferMutex);
		return RET_ERROR;
	}


	/*Update the other index if they are already initialized*/
	if(packetbuffer->dataIndex != 0xFFFF){
		packetbuffer->dataIndex = (packetbuffer->dataIndex - oldTransportHeaderIndex) + packetbuffer->transportHeaderIndex;

		if(packetbuffer->dataIndex >= FIXED_PACKET_SIZE){	/*Exceeded packet size. Not allowed.*/
			/*Return indexes to their original values*/
			packetbuffer->dataIndex = oldTransportHeaderIndex + (packetbuffer->dataIndex - packetbuffer->transportHeaderIndex);
			packetbuffer->transportHeaderIndex = oldTransportHeaderIndex;
			osMutexRelease(packetbuffer->packetbufferMutex);
			return RET_ERROR;
		}

		packetbuffer->packetDataSize = FIXED_PACKET_SIZE - packetbuffer->dataIndex;
	}

	osMutexRelease(packetbuffer->packetbufferMutex);
	return RET_OK;
}

/**
 *
 * @param packetbuffer
 * @param transportHdrSize
 * @return
 */
retval_t setTransportHdrSize(packetbuffer_t* packetbuffer, uint16_t transportHdrSize){
	uint16_t oldDataIndex;
	osMutexWait(packetbuffer->packetbufferMutex, osWaitForever);
	if((transportHdrSize >= (FIXED_PACKET_SIZE/2)) || (packetbuffer->transportHeaderIndex == 0xFFFF)){	/*The Routing Header size (which sets transportheadrindex) must have been initialized before*/
		osMutexRelease(packetbuffer->packetbufferMutex);
		return RET_ERROR;
	}

	oldDataIndex = packetbuffer->dataIndex;
	packetbuffer->dataIndex = packetbuffer->transportHeaderIndex + transportHdrSize;

	if(packetbuffer->dataIndex >= FIXED_PACKET_SIZE){	/*Exceeded packet size. Not allowed.*/
		/*Return indexes to their original values*/
		packetbuffer->dataIndex = oldDataIndex;
		osMutexRelease(packetbuffer->packetbufferMutex);
		return RET_ERROR;
	}

	packetbuffer->packetDataSize = FIXED_PACKET_SIZE - packetbuffer->dataIndex;

	osMutexRelease(packetbuffer->packetbufferMutex);
	return RET_OK;
}


/**
 *
 * @param netPacket
 */
void* getPacketDataPtr(netPacket_t* packet){
	if(packet->packetbuffer->dataIndex == 0xFFFF){
		return NULL;
	}
	else{
		return (void*) &(packet->packetData[packet->packetbuffer->dataIndex]);
	}
}


/**
 *
 * @param packet
 */
void* getPacketMacHeaderPtr(netPacket_t* packet){
	return (void*) &(packet->packetData[0]);
}


/**
 *
 * @param packet
 */
void* getPacketRoutingHeaderPtr(netPacket_t* packet){
	if(packet->packetbuffer->routingHeaderIndex == 0xFFFF){
		return NULL;
	}
	else{
		return (void*) &(packet->packetData[packet->packetbuffer->routingHeaderIndex]);
	}
}


/**
 *
 * @param packet
 */
void* getPacketTransportHeaderPtr(netPacket_t* packet){
	if(packet->packetbuffer->transportHeaderIndex == 0xFFFF){
		return NULL;
	}
	else{
		return (void*) &(packet->packetData[packet->packetbuffer->transportHeaderIndex]);
	}
}
