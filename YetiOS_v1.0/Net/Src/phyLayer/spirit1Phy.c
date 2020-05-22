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
 * spirit1Phy.c
 *
 *  Created on: 3 feb. 2020
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file spirit1Phy.c
 */


#include "netstack.h"
#include "yetiOS.h"
#include "spirit1Core.h"
#include "cc2500Core.h"

#if PLATFORM_USE_SPIRIT1_PHY_LAYER

#define SEND_PACKET_TIMEOUT		200		//Un paquete de 64 bytes no deberia tardar mas de 5ms en enviarse a 250 kbps

#define SPIRIT1_DEFAULT_PACKET_LENGTH	FIXED_PACKET_SIZE


#define PROCESS_CHECK_TIMEOUT			2000

#define INTERRUPT_QUEUE_SIZE			4

#define SPIRIT1_CHANNEL					0

#define CERBERUS_SEND_ERROR_CMD	0xFFFFFF00


static spirit1Data_t* spirit1Data;


static uint32_t lastInterruptTime;
static osMessageQId interruptMessageQId;
static osMutexId spirit1MutexId;
static netPacket_t* sendingPacket;
static uint16_t lastSendPcktLength;
static osThreadId spirit1ProcId;
static uint16_t phyChannel;
static osTimerId spirit1SendTimer;

static uint16_t spirit1PhyStarted = 0;
static uint16_t spirit1OpRunning = 0;


/*Cerberus Board Phy Functions*/
static retval_t spirit1PhyLayerInit(void);
static retval_t spirit1PhyLayerDeInit(void);
static retval_t spirit1PhySendPacket(netPacket_t* packet, uint16_t size);
static retval_t spirit1PhyPacketSent(netPacket_t* packet, uint16_t size);
static retval_t spirit1PhyPacketReceived(netPacket_t* packet, uint16_t size);
static retval_t spirit1PhySetModeReceiving(void);
static retval_t spirit1PhySetModeIdle(void);
static retval_t spirit1PhySetModeSleep(void);
static retval_t spirit1PhyCheckChannelRssi(float32_t* rssi);
static retval_t spirit1PhyGetLastRssi(float32_t* rssi);
static retval_t spirit1PhySetBaseFreq(uint32_t baseFreq);
static retval_t spirit1PhyGetBaseFreq(uint32_t* baseFreq);
static retval_t spirit1PhyGetChannelNum(uint32_t* channelNum);
static retval_t spirit1PhySetFreqChannel(uint32_t channelNum);
static retval_t spirit1PhyGetFreqChannel(uint32_t* channelNum);
static retval_t spirit1PhySetBaudRate(uint32_t baudRate);
static retval_t spirit1PhySetOutPower(int16_t outPower);
static retval_t spirit1PhyEncryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size);
static retval_t spirit1PhyDecryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size);
static retval_t spirit1PhySetProcessPriority(osPriority priority);
static osPriority spirit1PhyGetProcessPriority(void);

/* ******Cerberus process Func*************************/
static void spirit1ProcessFunc(const void* args);
/* ****************************************************/

/* ******Cerberus private Funcs*************************/
static void spirit1SendTimeoutCb(const void* args);

static retval_t spirit1Init(spirit1Data_t* spirit1Data, uint32_t spiDevId);
static void spirit1IrqCb(const void* args);


phyLayerOps_t spirit1PhyLayer = {
	.phyLayerInit = spirit1PhyLayerInit,
	.phyLayerDeInit = spirit1PhyLayerDeInit,
	.phySendPacket = spirit1PhySendPacket,
	.phyPacketSent = spirit1PhyPacketSent,
	.phyPacketReceived = spirit1PhyPacketReceived,
	.phySetModeReceiving = spirit1PhySetModeReceiving,
	.phySetModeIdle = spirit1PhySetModeIdle,
	.phySetModeSleep = spirit1PhySetModeSleep,
	.phyCheckChannelRssi = spirit1PhyCheckChannelRssi,
	.phyGetLastRssi = spirit1PhyGetLastRssi,
	.phySetBaseFreq = spirit1PhySetBaseFreq,
	.phyGetBaseFreq = spirit1PhyGetBaseFreq,
	.phyGetChannelNum = spirit1PhyGetChannelNum,
	.phySetFreqChannel = spirit1PhySetFreqChannel,
	.phyGetFreqChannel = spirit1PhyGetFreqChannel,
	.phySetBaudRate = spirit1PhySetBaudRate,
	.phySetOutPower = spirit1PhySetOutPower,
	.phyEncryptPacket = spirit1PhyEncryptPacket,
	.phyDecryptPacket = spirit1PhyDecryptPacket,
	.phySetProcessPriority = spirit1PhySetProcessPriority,
	.phyGetProcessPriority = spirit1PhyGetProcessPriority
};



/**
 *
 * @return
 */
static retval_t spirit1PhyLayerInit(){

#ifndef PLATFORM_SPIRIT1_SPI_DEV
	return RET_ERROR;
#else

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(spirit1PhyStarted || spirit1OpRunning){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	phyChannel = SPIRIT1_CHANNEL;
	taskEXIT_CRITICAL();

	/*Initialize Spirit1*/

	if((spirit1Data = newSpirit1Data(PLATFORM_SPIRIT1_CS_PIN, PLATFORM_SPIRIT1_SDN_PIN, PLATFORM_SPIRIT1_GPIO3_PIN, spirit1IrqCb, NULL)) == NULL){
		spirit1OpRunning--;
		return RET_ERROR;
	}

	if(spirit1Init(spirit1Data, PLATFORM_SPIRIT1_SPI_DEV) != RET_OK){
		deleteSpirit1Data(spirit1Data);
		spirit1OpRunning--;
		return RET_ERROR;
	}


	taskENTER_CRITICAL();

	/*Initialize variables*/
	spirit1MutexId = ytMutexCreate();
	sendingPacket = NULL;
	lastInterruptTime = 0;
	lastSendPcktLength = 0;

	interruptMessageQId = ytMessageqCreate(INTERRUPT_QUEUE_SIZE);
	spirit1SendTimer = ytTimerCreate(osTimerOnce, spirit1SendTimeoutCb, (void*) NULL);

	spirit1PhyStarted++;
	spirit1OpRunning--;

	ytStartThread("spirit1PhyProc", spirit1ProcessFunc, PHY_LAYER_PROCESS_PRIORITY, 400, &spirit1ProcId, NULL);
	taskEXIT_CRITICAL();

	return RET_OK;
#endif
}


/**
 *
 * @return
 */
static retval_t spirit1PhyLayerDeInit(){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1PhyStarted = 0;

	while(spirit1OpRunning){			/*Wait until all operations are finished before closing*/
		taskEXIT_CRITICAL();
		osDelay(1);
		taskENTER_CRITICAL();
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	osThreadTerminate (spirit1ProcId);
	osMutexDelete(spirit1MutexId);
	osMessageDelete(interruptMessageQId);
	osTimerStop(spirit1SendTimer);
	osTimerDelete(spirit1SendTimer);

	spirit1HwDeInit(spirit1Data);
	deleteSpirit1Data(spirit1Data);

	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t spirit1PhySendPacket(netPacket_t* packet, uint16_t size){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();
	osMutexWait(spirit1MutexId, osWaitForever);
	if(sendingPacket != NULL){
		osMutexRelease(spirit1MutexId);
		spirit1OpRunning--;
		return RET_ERROR;
	}

	sendingPacket = packet;
	lastSendPcktLength = size;
	if(spirit1SendData(spirit1Data, (uint8_t*) &(packet->packetData[0]), size) != RET_OK){
		sendingPacket = NULL;
		osMutexRelease(spirit1MutexId);
		spirit1OpRunning--;
		return RET_ERROR;
	}
	osTimerStart(spirit1SendTimer, SEND_PACKET_TIMEOUT);

	osMutexRelease(spirit1MutexId);
	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t spirit1PhyPacketSent(netPacket_t* packet, uint16_t size){	/*This function is not used in this implementation*/
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();
	netstack.macLayerOps->macPacketSent(packet, size);
	osMutexWait(spirit1MutexId, osWaitForever);
	osTimerStop(spirit1SendTimer);
	sendingPacket = NULL;
	osMutexRelease(spirit1MutexId);
	spirit1OpRunning--;
	return RET_OK;
}


/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t spirit1PhyPacketReceived(netPacket_t* packet, uint16_t size){	/*This function is not used in this implementation*/
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	netstack.macLayerOps->macPacketReceived(packet, size);

	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t spirit1PhySetModeReceiving(void){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	ret = spirit1SetModeRx(spirit1Data);

	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return ret;
}


/**
 *
 * @return
 */
static retval_t spirit1PhySetModeIdle(void){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	ret = spirit1SetModeIdle(spirit1Data);

	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return ret;
}

/**
 *
 * @return
 */
static retval_t spirit1PhySetModeSleep(void){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	ret = spirit1SetModeSleep(spirit1Data);

	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return ret;
}

/**
 *
 * @param rssi
 * @return
 */
static retval_t spirit1PhyCheckChannelRssi(float32_t* rssi){

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	*rssi = spirit1CheckChannelRssi(spirit1Data);

	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @param rssi
 * @return
 */
static retval_t spirit1PhyGetLastRssi(float32_t* rssi){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	*rssi = spirit1GetLastRssi(spirit1Data);

	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @param baseFreq
 * @return
 */
static retval_t spirit1PhySetBaseFreq(uint32_t baseFreq){
	return RET_ERROR;
}

/**
 *
 * @param baseFreq
 * @return
 */
static retval_t spirit1PhyGetBaseFreq(uint32_t* baseFreq){
	return RET_ERROR;
}

/**
 *
 * @param channelNum
 * @return
 */
static retval_t spirit1PhyGetChannelNum(uint32_t* channelNum){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);
	*channelNum = phyChannel;
	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @param channelNum
 * @return
 */
static retval_t spirit1PhySetFreqChannel(uint32_t channelNum){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);
	phyChannel = channelNum;
	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @param channelNum
 * @return
 */
static retval_t spirit1PhyGetFreqChannel(uint32_t* channelNum){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);
	*channelNum = phyChannel;
	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;
	return RET_OK;
}

/**
 *
 * @param baudRate
 * @return
 */
static retval_t spirit1PhySetBaudRate(uint32_t baudRate){
	return RET_ERROR;
}

/**
 *
 * @param outPower
 * @return
 */
static retval_t spirit1PhySetOutPower(int16_t outPower){
	return RET_ERROR;
}

/**
 *
 * @param packet
 * @param key
 * @param size
 * @return
 */
static retval_t spirit1PhyEncryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	ret = spirit1AesEncryptData(spirit1Data, (uint8_t*) &(packet->packetData[0]), key, size);

	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;

	return ret;
}

/**
 *
 * @param packet
 * @param key
 * @param size
 * @return
 */
static retval_t spirit1PhyDecryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);

	ret = spirit1AesDecryptData(spirit1Data, (uint8_t*) &(packet->packetData[0]), key, size);

	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;

	return ret;
}


/**
 *
 * @param priority
 * @return
 */
static retval_t spirit1PhySetProcessPriority(osPriority priority){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);
	osThreadSetPriority(spirit1ProcId, priority);
	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;

	return RET_OK;
}

/**
 *
 * @return
 */
static osPriority spirit1PhyGetProcessPriority(void){
	osPriority priority;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return osPriorityError;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(spirit1MutexId, osWaitForever);
	priority = osThreadGetPriority(spirit1ProcId);
	osMutexRelease(spirit1MutexId);

	spirit1OpRunning--;

	return priority;
}

/**
 *
 * @param args
 */
static void spirit1ProcessFunc(const void* args){
	netPacket_t* rcvPacket;
	netPacket_t* sentPacket;
	uint16_t  tempLastSendPcktLength;
	uint16_t ret;
	uint8_t numRrcvBytes = 0;
	osEvent event;
	while(1){
		event = osMessageGet (interruptMessageQId, PROCESS_CHECK_TIMEOUT);

		taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
		if(!spirit1PhyStarted){
			taskEXIT_CRITICAL();	/*It is entered here when the layer is de initializing. Wait until the thread is closed by the DeInit func*/
			while(1){
				osDelay(5000);
			}
		}
		spirit1OpRunning++;
		taskEXIT_CRITICAL();
		if(event.value.v == CERBERUS_SEND_ERROR_CMD){

			osMutexWait(spirit1MutexId, osWaitForever);
			sendingPacket->packetErrorFlag++;
			tempLastSendPcktLength = lastSendPcktLength;
			sentPacket = sendingPacket;
			osTimerStop(spirit1SendTimer);
			sendingPacket = NULL;
			osMutexRelease(spirit1MutexId);

			netstack.macLayerOps->macPacketSent(sentPacket, tempLastSendPcktLength);
		}
		else{
		/* Process Spirit1 interrupt */
			if(phyChannel == event.value.v){
				osMutexWait(spirit1MutexId, osWaitForever);
				ret = spirit1IrqRoutine(spirit1Data);
				if(ret & SPIRIT1_RET_PCKT_SENT){
					if(sendingPacket != NULL){
						osTimerStop(spirit1SendTimer);
						tempLastSendPcktLength = lastSendPcktLength;
						sentPacket = sendingPacket;
						sendingPacket = NULL;
						osMutexRelease(spirit1MutexId);

						netstack.macLayerOps->macPacketSent(sentPacket, tempLastSendPcktLength);	/*To the MAC layer*/
					}
					else{
						osMutexRelease(spirit1MutexId);
					}
				}
				else if(ret & SPIRIT1_RET_PCKT_RCV){
					if((rcvPacket = packetbufferGetFreePacket(rxPacketbuffer)) != NULL){
						spirit1ReadNumRcvBytes(spirit1Data, &numRrcvBytes);
						if(numRrcvBytes){
							spirit1ReadRcvData(spirit1Data, (uint8_t*) &(rcvPacket->packetData[0]), (uint16_t) numRrcvBytes);
							rcvPacket->rcvTickTime = lastInterruptTime;
							osMutexRelease(spirit1MutexId);
							netstack.macLayerOps->macPacketReceived(rcvPacket, numRrcvBytes);	/*To the MAC layer*/
						}
						else{
							osMutexRelease(spirit1MutexId);
						}
						packetbufferReleasePacket(rxPacketbuffer, rcvPacket);
					}
					else{
						spirit1ReadNumRcvBytes(spirit1Data, &numRrcvBytes);
						if(numRrcvBytes){
							spirit1FlushLastRcvData(spirit1Data, (uint16_t)numRrcvBytes);
						}
						osMutexRelease(spirit1MutexId);
					}
				}
				else{	/*IRQ ERROR*/
					osMutexRelease(spirit1MutexId);
				}
			}

		}
		spirit1OpRunning--;
	}
}

/**
 *
 * @param args
 */
static void spirit1SendTimeoutCb(const void* args){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!spirit1PhyStarted){
		taskEXIT_CRITICAL();
		return;
	}
	spirit1OpRunning++;
	taskEXIT_CRITICAL();

	osMessagePut (interruptMessageQId, CERBERUS_SEND_ERROR_CMD, 0);

	spirit1OpRunning--;
}


/**
 *
 * @param spirit1Data
 * @param spiDevId
 * @return
 */
static retval_t spirit1Init(spirit1Data_t* spirit1Data, uint32_t spiDevId){
	spirit1Config_t spirit1InitConfig;

	spirit1InitConfig.baud_rate = SPIRIT1_DEFAULT_BAUD_RATE;
	spirit1InitConfig.channel_num = SPIRIT1_DEFAULT_CHANNEL;
	spirit1InitConfig.channel_spacing = SPIRIT1_DEFAULT_SPACING;
	spirit1InitConfig.freq_deviation = SPIRIT1_DEFAULT_FREQ_DEV;
	spirit1InitConfig.modulation = SPIRIT1_DEFAULT_MODULATION;
	spirit1InitConfig.modulation_freq = SPIRIT1_DEFAULT_FREQ;
	spirit1InitConfig.output_power = SPIRIT1_DEFAULT_OUT_POWER;
	spirit1InitConfig.rx_bandwidth = SPIRIT1_DEFAULT_RX_BW;
	spirit1InitConfig.xtal_freq = SPIRIT1_DEFAULT_XTAL;

	return spirit1HwInit(spirit1Data, &spirit1InitConfig, spiDevId);
}


/**
 *
 * @param args
 */
static void spirit1IrqCb(const void* args){
	lastInterruptTime = osKernelSysTick();
	osMessagePut (interruptMessageQId, SPIRIT1_CHANNEL, 0);
}


#endif
