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
 * cerberusPhy.c
 *
 *  Created on: 23 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cerberusPhy
 */

#include "netstack.h"
#include "yetiOS.h"
#include "spirit1Core.h"
#include "cc2500Core.h"

#if PLATFORM_USE_CERBERUS_PHY_LAYER

#if (PLATFORM_ENABLE_SPIRIT1_433 || PLATFORM_ENABLE_SPIRIT1_868 || PLATFORM_ENABLE_CC2500)

#define SEND_PACKET_TIMEOUT		200		//Un paquete de 64 bytes no deberia tardar mas de 5ms en enviarse a 250 kbps

/* *********SPIRIT1 433 DEFAULT CONFIG ***********/
#define SPIRIT1_433_DEFAULT_XTAL		50e6

#define SPIRIT1_433_DEFAULT_BAUD_RATE	250e3	// 250 kbps
#define SPIRIT1_433_DEFAULT_SPACING		250e3	// 250 KHz
#define SPIRIT1_433_DEFAULT_FREQ_DEV	127e3	// 127 KHz
#define SPIRIT1_433_DEFAULT_MODULATION	GFSK_BT1
#define SPIRIT1_433_DEFAULT_FREQ		433.05e6	// 433.05 MHz
#define SPIRIT1_433_DEFAULT_OUT_POWER	11		// 11 dBm
#define SPIRIT1_433_DEFAULT_RX_BW		540e3	// 540 KHz
#define SPIRIT1_433_DEFAULT_CHANNEL		0

#define SPIRIT1_433_DEFAULT_PACKET_LENGTH	FIXED_PACKET_SIZE
/* *********SPIRIT1 433 DEFAULT CONFIG ***********/


/* *********SPIRIT1 868 DEFAULT CONFIG ***********/
#define SPIRIT1_868_DEFAULT_XTAL		50e6

#define SPIRIT1_868_DEFAULT_BAUD_RATE	250e3	// 250 kbps
#define SPIRIT1_868_DEFAULT_SPACING		250e3	// 250 KHz
#define SPIRIT1_868_DEFAULT_FREQ_DEV	127e3	// 127 KHz
#define SPIRIT1_868_DEFAULT_MODULATION	GFSK_BT1
#define SPIRIT1_868_DEFAULT_FREQ		868e6	// 868.00 MHz
#define SPIRIT1_868_DEFAULT_OUT_POWER	11		// 11 dBm
#define SPIRIT1_868_DEFAULT_RX_BW		540e3	// 540 KHz
#define SPIRIT1_868_DEFAULT_CHANNEL		0

#define SPIRIT1_868_DEFAULT_PACKET_LENGTH	FIXED_PACKET_SIZE
/* *********SPIRIT1 433 DEFAULT CONFIG ***********/


/* *********CC2500 DEFAULT CONFIG ***********/
#define CC2500_DEFAULT_BAUD_RATE	250e3	// 250 kbps
#define CC2500_DEFAULT_SPACING		250e3	// 250 KHz
#define CC2500_DEFAULT_FREQ_DEV		0		// Do not apply to MSK modulation
#define CC2500_DEFAULT_MODULATION	CC2500_MSK
#define CC2500_DEFAULT_FREQ			2410	// 2.41 Ghz
#define CC2500_DEFAULT_OUT_POWER	1		// 1 dBm
#define CC2500_DEFAULT_RX_BW		540e3	// 540 KHz
#define CC2500_DEFAULT_CHANNEL		0

#define CC2500_DEFAULT_PACKET_LENGTH	FIXED_PACKET_SIZE
/* *********SPIRIT1 433 DEFAULT CONFIG ***********/

#define PROCESS_CHECK_TIMEOUT			2000

#define INTERRUPT_QUEUE_SIZE			4
/* CHANNELS */
/* FROM CHANNEL 0 to CHANNEL 255, CC2500 CHANNELS: 2410 MHz to 2474 */
/* CHANNEL 256: SPIRIT 1 433MHz CHANNEL */
/* CHANNEL 512: SPIRIT 1 868MHz CHANNEL */
#define CC2500_BASE_CHANNEL		0
#define SPIRIT1_433_CHANNEL		256
#define SPIRIT1_868_CHANNEL		512

#define CERBERUS_SEND_ERROR_CMD	0xFFFFFF00


#if CERBERUS_DEFAULT_TRANSCEIVER == CERBERUS_CC2500_TRANSCEIVER
	#if PLATFORM_ENABLE_CC2500
	#define CERBERUS_DEFAULT_CHANNEL	CC2500_BASE_CHANNEL
	#else
	#define CERBERUS_DEFAULT_CHANNEL	0
	#endif
#endif
#if CERBERUS_DEFAULT_TRANSCEIVER == CERBERUS_SPIRIT1_433_TRANSCEIVER
	#if PLATFORM_ENABLE_SPIRIT1_433
	#define CERBERUS_DEFAULT_CHANNEL	SPIRIT1_433_CHANNEL
	#else
	#define CERBERUS_DEFAULT_CHANNEL	0
	#endif
#endif
#if CERBERUS_DEFAULT_TRANSCEIVER == CERBERUS_SPIRIT1_868_TRANSCEIVER
	#if PLATFORM_ENABLE_SPIRIT1_868
	#define CERBERUS_DEFAULT_CHANNEL	SPIRIT1_868_CHANNEL
	#else
	#define CERBERUS_DEFAULT_CHANNEL	0
	#endif
#endif




#if PLATFORM_ENABLE_SPIRIT1_433
static spirit1Data_t* spirit1Data433;
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
static spirit1Data_t* spirit1Data868;
#endif
#if PLATFORM_ENABLE_CC2500
static cc2500Data_t* cc2500Data;
#endif

static uint32_t lastInterruptTime;
static osMessageQId interruptMessageQId;
static osMutexId cerberusMutexId;
static netPacket_t* sendingPacket;
static uint16_t lastSendPcktLength;
static osThreadId cerbProcId;
static uint16_t phyChannel;
static osTimerId cerbSendTimer;

static uint16_t cerberusPhyStarted = 0;
static uint16_t cerberusOpRunning = 0;


/*Cerberus Board Phy Functions*/
static retval_t cerberusPhyLayerInit(void);
static retval_t cerberusPhyLayerDeInit(void);
static retval_t cerberusPhySendPacket(netPacket_t* packet, uint16_t size);
static retval_t cerberusPhyPacketSent(netPacket_t* packet, uint16_t size);
static retval_t cerberusPhyPacketReceived(netPacket_t* packet, uint16_t size);
static retval_t cerberusPhySetModeReceiving(void);
static retval_t cerberusPhySetModeIdle(void);
static retval_t cerberusPhySetModeSleep(void);
static retval_t cerberusPhyCheckChannelRssi(float32_t* rssi);
static retval_t cerberusPhyGetLastRssi(float32_t* rssi);
static retval_t cerberusPhySetBaseFreq(uint32_t baseFreq);
static retval_t cerberusPhyGetBaseFreq(uint32_t* baseFreq);
static retval_t cerberusPhyGetChannelNum(uint32_t* channelNum);
static retval_t cerberusPhySetFreqChannel(uint32_t channelNum);
static retval_t cerberusPhyGetFreqChannel(uint32_t* channelNum);
static retval_t cerberusPhySetBaudRate(uint32_t baudRate);
static retval_t cerberusPhySetOutPower(int16_t outPower);
static retval_t cerberusPhyEncryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size);
static retval_t cerberusPhyDecryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size);
static retval_t cerberusPhySetProcessPriority(osPriority priority);
static osPriority cerberusPhyGetProcessPriority(void);

/* ******Cerberus process Func*************************/
static void cerberusProcessFunc(const void* args);
/* ****************************************************/

/* ******Cerberus private Funcs*************************/
static void cerberusSendTimeoutCb(const void* args);
/* SPIRIT1 433 PRIVATE FUNCS*/
#if PLATFORM_ENABLE_SPIRIT1_433
static retval_t spirit1Init433(spirit1Data_t* spirit1Data, uint32_t spiDevId);
static void spirit1IrqCb433(const void* args);
#endif
/* SPIRIT1 433 PRIVATE FUNCS*/

/* SPIRIT1 433 PRIVATE FUNCS*/
#if PLATFORM_ENABLE_SPIRIT1_868
static retval_t spirit1Init868(spirit1Data_t* spirit1Data, uint32_t spiDevId);
static void spirit1IrqCb868(const void* args);
#endif
/* SPIRIT1 433 PRIVATE FUNCS*/

/* CC2500 PRIVATE FUNCS*/
#if PLATFORM_ENABLE_CC2500
static retval_t cc2500Init(cc2500Data_t* cc2500Data, uint32_t spiDevId);
static void cc2500IrqCb(const void* args);
#endif
/* CC2500 PRIVATE FUNCS*/


phyLayerOps_t cerberusPhyLayer = {
	.phyLayerInit = cerberusPhyLayerInit,
	.phyLayerDeInit = cerberusPhyLayerDeInit,
	.phySendPacket = cerberusPhySendPacket,
	.phyPacketSent = cerberusPhyPacketSent,
	.phyPacketReceived = cerberusPhyPacketReceived,
	.phySetModeReceiving = cerberusPhySetModeReceiving,
	.phySetModeIdle = cerberusPhySetModeIdle,
	.phySetModeSleep = cerberusPhySetModeSleep,
	.phyCheckChannelRssi = cerberusPhyCheckChannelRssi,
	.phyGetLastRssi = cerberusPhyGetLastRssi,
	.phySetBaseFreq = cerberusPhySetBaseFreq,
	.phyGetBaseFreq = cerberusPhyGetBaseFreq,
	.phyGetChannelNum = cerberusPhyGetChannelNum,
	.phySetFreqChannel = cerberusPhySetFreqChannel,
	.phyGetFreqChannel = cerberusPhyGetFreqChannel,
	.phySetBaudRate = cerberusPhySetBaudRate,
	.phySetOutPower = cerberusPhySetOutPower,
	.phyEncryptPacket = cerberusPhyEncryptPacket,
	.phyDecryptPacket = cerberusPhyDecryptPacket,
	.phySetProcessPriority = cerberusPhySetProcessPriority,
	.phyGetProcessPriority = cerberusPhyGetProcessPriority
};



/**
 *
 * @return
 */
static retval_t cerberusPhyLayerInit(){

#if	((CERBERUS_DEFAULT_CHANNEL == SPIRIT1_868_CHANNEL) && (!PLATFORM_ENABLE_SPIRIT1_868))
	return RET_ERROR;
#endif

#if	((CERBERUS_DEFAULT_CHANNEL == SPIRIT1_433_CHANNEL) && (!PLATFORM_ENABLE_SPIRIT1_433))
	return RET_ERROR;
#endif

#if	((CERBERUS_DEFAULT_CHANNEL == CC2500_BASE_CHANNEL) && (!PLATFORM_ENABLE_CC2500))
	return RET_ERROR;
#endif

#ifndef PLATFORM_CERBERUS_SPI_DEV
	return RET_ERROR;
#else

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(cerberusPhyStarted || cerberusOpRunning){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	phyChannel = CERBERUS_DEFAULT_CHANNEL;
	taskEXIT_CRITICAL();

	/*Initialize Spirit1 433*/
#if PLATFORM_ENABLE_SPIRIT1_433
	if((spirit1Data433 = newSpirit1Data(PLATFORM_SPIRIT1_433_CS_PIN, PLATFORM_SPIRIT1_433_SDN_PIN, PLATFORM_SPIRIT1_433_GPIO3_PIN, spirit1IrqCb433, NULL)) == NULL){
		cerberusOpRunning--;
		return RET_ERROR;
	}

	if(spirit1Init433(spirit1Data433, PLATFORM_CERBERUS_SPI_DEV) != RET_OK){
		deleteSpirit1Data(spirit1Data433);
		cerberusOpRunning--;
		return RET_ERROR;
	}
#endif

	/*Initialize CC2500*/
#if PLATFORM_ENABLE_CC2500
	if((cc2500Data = newCc2500Data(PLATFORM_CC2500_CS_PIN, PLATFORM_CC2500_GDO2_PIN, cc2500IrqCb, NULL)) == NULL){
#if PLATFORM_ENABLE_SPIRIT1_433
		spirit1HwDeInit(spirit1Data433);
		deleteSpirit1Data(spirit1Data433);
#endif
		cerberusOpRunning--;
		return RET_ERROR;
	}

	if(cc2500Init(cc2500Data, PLATFORM_CERBERUS_SPI_DEV) != RET_OK){
#if PLATFORM_ENABLE_SPIRIT1_433
		spirit1HwDeInit(spirit1Data433);
		deleteSpirit1Data(spirit1Data433);
#endif
		deleteCc2500Data(cc2500Data);
		cerberusOpRunning--;
		return RET_ERROR;
	}
#endif

	/*Initialize Spirit1 868*/
#if PLATFORM_ENABLE_SPIRIT1_868
	if((spirit1Data868 = newSpirit1Data(PLATFORM_SPIRIT1_868_CS_PIN, PLATFORM_SPIRIT1_868_SDN_PIN, PLATFORM_SPIRIT1_868_GPIO3_PIN, spirit1IrqCb868, NULL)) == NULL){
#if PLATFORM_ENABLE_SPIRIT1_433
		spirit1HwDeInit(spirit1Data433);
		deleteSpirit1Data(spirit1Data433);
#endif
#if PLATFORM_ENABLE_CC2500
		cc2500HwDeInit(cc2500Data);
		deleteCc2500Data(cc2500Data);
#endif
		cerberusOpRunning--;
		return RET_ERROR;
	}

	if(spirit1Init868(spirit1Data868, PLATFORM_CERBERUS_SPI_DEV) != RET_OK){
#if PLATFORM_ENABLE_SPIRIT1_433
		spirit1HwDeInit(spirit1Data433);
		deleteSpirit1Data(spirit1Data433);
#endif
#if PLATFORM_ENABLE_CC2500
		cc2500HwDeInit(cc2500Data);
		deleteCc2500Data(cc2500Data);
#endif
		deleteSpirit1Data(spirit1Data868);
		cerberusOpRunning--;
		return RET_ERROR;
	}
#endif

	if(phyChannel < SPIRIT1_433_CHANNEL){	//CC2500 selected. Sleep the spirit1_433 and the spirit_868
#if PLATFORM_ENABLE_SPIRIT1_433
		spirit1SetModeSleep(spirit1Data433);
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
		spirit1SetModeSleep(spirit1Data868);
#endif
	}

	if(phyChannel == SPIRIT1_433_CHANNEL){	//Spirit1_433 selected. Sleep the CC2500 and the spirit_868
#if PLATFORM_ENABLE_CC2500
		cc2500SetModeSleep(cc2500Data);
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
		spirit1SetModeSleep(spirit1Data868);
#endif
	}
	if(phyChannel == SPIRIT1_868_CHANNEL){	//Spirit1_868 selected. Sleep the spirit1_433 and the cc2500
#if PLATFORM_ENABLE_CC2500
		cc2500SetModeSleep(cc2500Data);
#endif
#if PLATFORM_ENABLE_SPIRIT1_433
		spirit1SetModeSleep(spirit1Data433);
#endif
	}

	taskENTER_CRITICAL();

	/*Initialize variables*/
	cerberusMutexId = ytMutexCreate();
	sendingPacket = NULL;
	lastInterruptTime = 0;
	lastSendPcktLength = 0;

	interruptMessageQId = ytMessageqCreate(INTERRUPT_QUEUE_SIZE);
	cerbSendTimer = ytTimerCreate(osTimerOnce, cerberusSendTimeoutCb, (void*) NULL);

	cerberusPhyStarted++;
	cerberusOpRunning--;

	ytStartThread("cerbPhyProc", cerberusProcessFunc, PHY_LAYER_PROCESS_PRIORITY, 400, &cerbProcId, NULL);
	taskEXIT_CRITICAL();

	return RET_OK;
#endif
}


/**
 *
 * @return
 */
static retval_t cerberusPhyLayerDeInit(){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusPhyStarted = 0;

	while(cerberusOpRunning){			/*Wait until all operations are finished before closing*/
		taskEXIT_CRITICAL();
		osDelay(1);
		taskENTER_CRITICAL();
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);

	osThreadTerminate (cerbProcId);
	osMutexDelete(cerberusMutexId);
	osMessageDelete(interruptMessageQId);
	osTimerStop(cerbSendTimer);
	osTimerDelete(cerbSendTimer);

#if PLATFORM_ENABLE_SPIRIT1_433
	spirit1HwDeInit(spirit1Data433);
	deleteSpirit1Data(spirit1Data433);
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	spirit1HwDeInit(spirit1Data868);
	deleteSpirit1Data(spirit1Data868);
#endif
#if PLATFORM_ENABLE_CC2500
	cc2500HwDeInit(cc2500Data);
	deleteCc2500Data(cc2500Data);
#endif
	cerberusOpRunning--;
	return RET_OK;
}

/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t cerberusPhySendPacket(netPacket_t* packet, uint16_t size){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();
	osMutexWait(cerberusMutexId, osWaitForever);
	if(sendingPacket != NULL){
		osMutexRelease(cerberusMutexId);
		cerberusOpRunning--;
		return RET_ERROR;
	}
#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		sendingPacket = packet;
		lastSendPcktLength = size;
		if(spirit1SendData(spirit1Data433, (uint8_t*) &(packet->packetData[0]), size) != RET_OK){
			sendingPacket = NULL;
			osMutexRelease(cerberusMutexId);
			cerberusOpRunning--;
			return RET_ERROR;
		}
		osTimerStart(cerbSendTimer, SEND_PACKET_TIMEOUT);
		osMutexRelease(cerberusMutexId);
		cerberusOpRunning--;
		return RET_OK;
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		sendingPacket = packet;
		lastSendPcktLength = size;
		if(spirit1SendData(spirit1Data868, (uint8_t*) &(packet->packetData[0]), size) != RET_OK){
			sendingPacket = NULL;
			osMutexRelease(cerberusMutexId);
			cerberusOpRunning--;
			return RET_ERROR;
		}
		osTimerStart(cerbSendTimer, SEND_PACKET_TIMEOUT);
		osMutexRelease(cerberusMutexId);
		cerberusOpRunning--;
		return RET_OK;
	}
#endif
#if PLATFORM_ENABLE_CC2500
	if(phyChannel < SPIRIT1_433_CHANNEL){
		sendingPacket = packet;
		lastSendPcktLength = size;
		if(cc2500SendData(cc2500Data, (uint8_t*) &(packet->packetData[0]), size) != RET_OK){
			sendingPacket = NULL;
			osMutexRelease(cerberusMutexId);
			cerberusOpRunning--;
			return RET_ERROR;
		}
		osTimerStart(cerbSendTimer, SEND_PACKET_TIMEOUT);

		osMutexRelease(cerberusMutexId);
		cerberusOpRunning--;
		return RET_OK;

	}
#endif

	osMutexRelease(cerberusMutexId);
	cerberusOpRunning--;
	return RET_ERROR;
}

/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t cerberusPhyPacketSent(netPacket_t* packet, uint16_t size){	/*This function is not used in this implementation*/
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();
	netstack.macLayerOps->macPacketSent(packet, size);
	osMutexWait(cerberusMutexId, osWaitForever);
	osTimerStop(cerbSendTimer);
	sendingPacket = NULL;
	osMutexRelease(cerberusMutexId);
	cerberusOpRunning--;
	return RET_OK;
}


/**
 *
 * @param packet
 * @param size
 * @return
 */
static retval_t cerberusPhyPacketReceived(netPacket_t* packet, uint16_t size){	/*This function is not used in this implementation*/
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	netstack.macLayerOps->macPacketReceived(packet, size);

	cerberusOpRunning--;
	return RET_OK;
}

/**
 *
 * @return
 */
static retval_t cerberusPhySetModeReceiving(void){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);

#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		ret = spirit1SetModeRx(spirit1Data433);
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		ret = spirit1SetModeRx(spirit1Data868);
	}
#endif
#if PLATFORM_ENABLE_CC2500
	if(phyChannel < SPIRIT1_433_CHANNEL){
		ret = cc2500SetModeRx(cc2500Data);
	}
#endif

	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return ret;
}


/**
 *
 * @return
 */
static retval_t cerberusPhySetModeIdle(void){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);

#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		ret = spirit1SetModeIdle(spirit1Data433);
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		ret = spirit1SetModeIdle(spirit1Data868);
	}
#endif
#if PLATFORM_ENABLE_CC2500
	else if(phyChannel < SPIRIT1_433_CHANNEL){
		ret = cc2500SetModeIdle(cc2500Data);
	}
#endif

	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return ret;
}

/**
 *
 * @return
 */
static retval_t cerberusPhySetModeSleep(void){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);

#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		ret = spirit1SetModeSleep(spirit1Data433);
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		ret = spirit1SetModeSleep(spirit1Data868);
	}
#endif
#if PLATFORM_ENABLE_CC2500
	else if(phyChannel < SPIRIT1_433_CHANNEL){
		ret = cc2500SetModeSleep(cc2500Data);
	}
#endif

	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return ret;
}

/**
 *
 * @param rssi
 * @return
 */
static retval_t cerberusPhyCheckChannelRssi(float32_t* rssi){

	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);

#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		*rssi = spirit1CheckChannelRssi(spirit1Data433);
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		*rssi = spirit1CheckChannelRssi(spirit1Data868);
	}
#endif
#if PLATFORM_ENABLE_CC2500
	else if(phyChannel < SPIRIT1_433_CHANNEL){
		*rssi = cc2500GetLastRssi(cc2500Data);
	}
#endif

	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return RET_OK;
}

/**
 *
 * @param rssi
 * @return
 */
static retval_t cerberusPhyGetLastRssi(float32_t* rssi){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);

#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		*rssi = spirit1GetLastRssi(spirit1Data433);
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		*rssi = spirit1GetLastRssi(spirit1Data868);
	}
#endif
#if PLATFORM_ENABLE_CC2500
	else if(phyChannel < SPIRIT1_433_CHANNEL){
		*rssi = cc2500GetLastRssi(cc2500Data);
	}
#endif

	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return RET_OK;
}

/**
 *
 * @param baseFreq
 * @return
 */
static retval_t cerberusPhySetBaseFreq(uint32_t baseFreq){
	return RET_ERROR;
}

/**
 *
 * @param baseFreq
 * @return
 */
static retval_t cerberusPhyGetBaseFreq(uint32_t* baseFreq){
	return RET_ERROR;
}

/**
 *
 * @param channelNum
 * @return
 */
static retval_t cerberusPhyGetChannelNum(uint32_t* channelNum){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);
	*channelNum = phyChannel;
	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return RET_OK;
}

/**
 *
 * @param channelNum
 * @return
 */
static retval_t cerberusPhySetFreqChannel(uint32_t channelNum){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);
	if(channelNum < SPIRIT1_433_CHANNEL){
#if PLATFORM_ENABLE_CC2500
		cc2500SetChannel(cc2500Data, channelNum);
#endif
	}
	else if((channelNum != SPIRIT1_433_CHANNEL) && (phyChannel != SPIRIT1_868_CHANNEL)){
		osMutexRelease(cerberusMutexId);
		cerberusOpRunning--;
		return RET_ERROR;
	}
	phyChannel = channelNum;
	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return RET_OK;
}

/**
 *
 * @param channelNum
 * @return
 */
static retval_t cerberusPhyGetFreqChannel(uint32_t* channelNum){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);
	*channelNum = phyChannel;
	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;
	return RET_OK;
}

/**
 *
 * @param baudRate
 * @return
 */
static retval_t cerberusPhySetBaudRate(uint32_t baudRate){
	return RET_ERROR;
}

/**
 *
 * @param outPower
 * @return
 */
static retval_t cerberusPhySetOutPower(int16_t outPower){
	return RET_ERROR;
}

/**
 *
 * @param packet
 * @param key
 * @param size
 * @return
 */
static retval_t cerberusPhyEncryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);
#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		ret = spirit1AesEncryptData(spirit1Data433, (uint8_t*) &(packet->packetData[0]), key, size);
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		ret = spirit1AesEncryptData(spirit1Data868, (uint8_t*) &(packet->packetData[0]), key, size);
	}
#endif
	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;

	return ret;
}

/**
 *
 * @param packet
 * @param key
 * @param size
 * @return
 */
static retval_t cerberusPhyDecryptPacket(netPacket_t* packet, uint8_t* key, uint16_t size){
	retval_t ret = RET_ERROR;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return ret;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);
#if PLATFORM_ENABLE_SPIRIT1_433
	if(phyChannel == SPIRIT1_433_CHANNEL){
		ret = spirit1AesDecryptData(spirit1Data433, (uint8_t*) &(packet->packetData[0]), key, size);
	}
#endif
#if PLATFORM_ENABLE_SPIRIT1_868
	if(phyChannel == SPIRIT1_868_CHANNEL){
		ret = spirit1AesDecryptData(spirit1Data868, (uint8_t*) &(packet->packetData[0]), key, size);
	}
#endif
	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;

	return ret;
}


/**
 *
 * @param priority
 * @return
 */
static retval_t cerberusPhySetProcessPriority(osPriority priority){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return RET_ERROR;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);
	osThreadSetPriority(cerbProcId, priority);
	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;

	return RET_OK;
}

/**
 *
 * @return
 */
static osPriority cerberusPhyGetProcessPriority(void){
	osPriority priority;
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return osPriorityError;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMutexWait(cerberusMutexId, osWaitForever);
	priority = osThreadGetPriority(cerbProcId);
	osMutexRelease(cerberusMutexId);

	cerberusOpRunning--;

	return priority;
}

/**
 *
 * @param args
 */
static void cerberusProcessFunc(const void* args){
	netPacket_t* rcvPacket;
	netPacket_t* sentPacket;
	uint16_t  tempLastSendPcktLength;
	uint16_t ret;
	uint8_t numRrcvBytes = 0;
	osEvent event;
	while(1){
		event = osMessageGet (interruptMessageQId, PROCESS_CHECK_TIMEOUT);

		taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
		if(!cerberusPhyStarted){
			taskEXIT_CRITICAL();	/*It is entered here when the layer is de initializing. Wait until the thread is closed by the DeInit func*/
			while(1){
				osDelay(5000);
			}
		}
		cerberusOpRunning++;
		taskEXIT_CRITICAL();
		if(event.value.v == CERBERUS_SEND_ERROR_CMD){

			osMutexWait(cerberusMutexId, osWaitForever);
			sendingPacket->packetErrorFlag++;
			tempLastSendPcktLength = lastSendPcktLength;
			sentPacket = sendingPacket;
			osTimerStop(cerbSendTimer);
			sendingPacket = NULL;
			osMutexRelease(cerberusMutexId);

			netstack.macLayerOps->macPacketSent(sentPacket, tempLastSendPcktLength);
		}
		else{
		/* Process Spirit1 433 interrupt */
#if PLATFORM_ENABLE_SPIRIT1_433
			if((phyChannel == SPIRIT1_433_CHANNEL) && (phyChannel == event.value.v)){
				osMutexWait(cerberusMutexId, osWaitForever);
				ret = spirit1IrqRoutine(spirit1Data433);
				if(ret & SPIRIT1_RET_PCKT_SENT){
					if(sendingPacket != NULL){
						osTimerStop(cerbSendTimer);
						tempLastSendPcktLength = lastSendPcktLength;
						sentPacket = sendingPacket;
						sendingPacket = NULL;
						osMutexRelease(cerberusMutexId);

						netstack.macLayerOps->macPacketSent(sentPacket, tempLastSendPcktLength);	/*To the MAC layer*/
					}
					else{
						osMutexRelease(cerberusMutexId);
					}
				}
				else if(ret & SPIRIT1_RET_PCKT_RCV){
					if((rcvPacket = packetbufferGetFreePacket(rxPacketbuffer)) != NULL){
						spirit1ReadNumRcvBytes(spirit1Data433, &numRrcvBytes);
						if(numRrcvBytes){
							spirit1ReadRcvData(spirit1Data433, (uint8_t*) &(rcvPacket->packetData[0]), (uint16_t) numRrcvBytes);
							rcvPacket->rcvTickTime = lastInterruptTime;
							osMutexRelease(cerberusMutexId);
							netstack.macLayerOps->macPacketReceived(rcvPacket, numRrcvBytes);	/*To the MAC layer*/
						}
						else{
							osMutexRelease(cerberusMutexId);
						}
						packetbufferReleasePacket(rxPacketbuffer, rcvPacket);
					}
					else{
						spirit1ReadNumRcvBytes(spirit1Data433, &numRrcvBytes);
						if(numRrcvBytes){
							spirit1FlushLastRcvData(spirit1Data433, (uint16_t)numRrcvBytes);
						}
						osMutexRelease(cerberusMutexId);
					}
				}
				else{	/*IRQ ERROR*/
					osMutexRelease(cerberusMutexId);
				}
			}
#endif

		/* Process Spirit1 868 interrupt */
#if PLATFORM_ENABLE_SPIRIT1_868
			if((phyChannel == SPIRIT1_868_CHANNEL) && (phyChannel == event.value.v)){
				osMutexWait(cerberusMutexId, osWaitForever);
				ret = spirit1IrqRoutine(spirit1Data868);

				if(ret & SPIRIT1_RET_PCKT_SENT){
					if(sendingPacket != NULL){
						osTimerStop(cerbSendTimer);
						tempLastSendPcktLength = lastSendPcktLength;
						sentPacket = sendingPacket;
						sendingPacket = NULL;
						osMutexRelease(cerberusMutexId);

						netstack.macLayerOps->macPacketSent(sentPacket, tempLastSendPcktLength);	/*To the MAC layer*/
					}
					else{
						osMutexRelease(cerberusMutexId);
					}

				}
				else if(ret & SPIRIT1_RET_PCKT_RCV){
					if((rcvPacket = packetbufferGetFreePacket(rxPacketbuffer)) != NULL){
						spirit1ReadNumRcvBytes(spirit1Data868, &numRrcvBytes);
						if(numRrcvBytes){
							spirit1ReadRcvData(spirit1Data868, (uint8_t*) &(rcvPacket->packetData[0]), (uint16_t) numRrcvBytes);
							rcvPacket->rcvTickTime = lastInterruptTime;
							osMutexRelease(cerberusMutexId);

							netstack.macLayerOps->macPacketReceived(rcvPacket, numRrcvBytes);	/*To the MAC layer*/
						}
						else{
							osMutexRelease(cerberusMutexId);
						}
						packetbufferReleasePacket(rxPacketbuffer, rcvPacket);
					}
					else{
						spirit1ReadNumRcvBytes(spirit1Data868, &numRrcvBytes);
						if(numRrcvBytes){
							spirit1FlushLastRcvData(spirit1Data868, (uint16_t)numRrcvBytes);
						}
						osMutexRelease(cerberusMutexId);
					}
				}
				else{	/*IRQ ERROR*/
					osMutexRelease(cerberusMutexId);
				}
			}
#endif

#if PLATFORM_ENABLE_CC2500
			/* Process CC2500 interrupt */
			if((phyChannel < SPIRIT1_433_CHANNEL) && (event.value.v == CC2500_BASE_CHANNEL)){
				osMutexWait(cerberusMutexId, osWaitForever);
				ret = cc2500IrqRoutine(cc2500Data);

				if(ret & CC2500_RET_PCKT_SENT){
					if(sendingPacket != NULL){
						osTimerStop(cerbSendTimer);
						tempLastSendPcktLength = lastSendPcktLength;
						sentPacket = sendingPacket;
						sendingPacket = NULL;

						osMutexRelease(cerberusMutexId);

						netstack.macLayerOps->macPacketSent(sentPacket, tempLastSendPcktLength);	/*To the MAC layer*/
					}
					else{
						osMutexRelease(cerberusMutexId);
					}
				}
				else if(ret & CC2500_RET_PCKT_RCV){
					if((rcvPacket = packetbufferGetFreePacket(rxPacketbuffer)) != NULL){
						cc2500ReadNumRcvBytes(cc2500Data, &numRrcvBytes);
						if(numRrcvBytes){
							cc2500ReadRcvData(cc2500Data, (uint8_t*) &(rcvPacket->packetData[0]), (uint16_t) numRrcvBytes);
							rcvPacket->rcvTickTime = lastInterruptTime;
							osMutexRelease(cerberusMutexId);

							netstack.macLayerOps->macPacketReceived(rcvPacket, numRrcvBytes);	/*To the MAC layer*/
						}
						else{
							osMutexRelease(cerberusMutexId);
						}
						packetbufferReleasePacket(rxPacketbuffer, rcvPacket);
					}
					else{
						cc2500ReadNumRcvBytes(cc2500Data, &numRrcvBytes);
						if(numRrcvBytes){
							cc2500FlushLastRcvData(cc2500Data, (uint16_t)numRrcvBytes);
						}
						osMutexRelease(cerberusMutexId);
					}
				}
				else{	/*IRQ ERROR*/
					osMutexRelease(cerberusMutexId);
				}

			}
#endif
		}
		cerberusOpRunning--;
	}
}

/**
 *
 * @param args
 */
static void cerberusSendTimeoutCb(const void* args){
	taskENTER_CRITICAL();		/*Evaluate started condition in a critical section*/
	if(!cerberusPhyStarted){
		taskEXIT_CRITICAL();
		return;
	}
	cerberusOpRunning++;
	taskEXIT_CRITICAL();

	osMessagePut (interruptMessageQId, CERBERUS_SEND_ERROR_CMD, 0);

	cerberusOpRunning--;
}

#if PLATFORM_ENABLE_SPIRIT1_433
/**
 *
 * @param spirit1Data
 * @param spiDevId
 * @return
 */
static retval_t spirit1Init433(spirit1Data_t* spirit1Data, uint32_t spiDevId){
	spirit1Config_t spirit1InitConfig;

	spirit1InitConfig.baud_rate = SPIRIT1_433_DEFAULT_BAUD_RATE;
	spirit1InitConfig.channel_num = SPIRIT1_433_DEFAULT_CHANNEL;
	spirit1InitConfig.channel_spacing = SPIRIT1_433_DEFAULT_SPACING;
	spirit1InitConfig.freq_deviation = SPIRIT1_433_DEFAULT_FREQ_DEV;
	spirit1InitConfig.modulation = SPIRIT1_433_DEFAULT_MODULATION;
	spirit1InitConfig.modulation_freq = SPIRIT1_433_DEFAULT_FREQ;
	spirit1InitConfig.output_power = SPIRIT1_433_DEFAULT_OUT_POWER;
	spirit1InitConfig.rx_bandwidth = SPIRIT1_433_DEFAULT_RX_BW;
	spirit1InitConfig.xtal_freq = SPIRIT1_433_DEFAULT_XTAL;

	return spirit1HwInit(spirit1Data, &spirit1InitConfig, spiDevId);
}
#endif

#if PLATFORM_ENABLE_SPIRIT1_868
/**
 *
 * @param spirit1Data
 * @param spiDevId
 * @return
 */
static retval_t spirit1Init868(spirit1Data_t* spirit1Data, uint32_t spiDevId){
	spirit1Config_t spirit1InitConfig;

	spirit1InitConfig.baud_rate = SPIRIT1_868_DEFAULT_BAUD_RATE;
	spirit1InitConfig.channel_num = SPIRIT1_868_DEFAULT_CHANNEL;
	spirit1InitConfig.channel_spacing = SPIRIT1_868_DEFAULT_SPACING;
	spirit1InitConfig.freq_deviation = SPIRIT1_868_DEFAULT_FREQ_DEV;
	spirit1InitConfig.modulation = SPIRIT1_868_DEFAULT_MODULATION;
	spirit1InitConfig.modulation_freq = SPIRIT1_868_DEFAULT_FREQ;
	spirit1InitConfig.output_power = SPIRIT1_868_DEFAULT_OUT_POWER;
	spirit1InitConfig.rx_bandwidth = SPIRIT1_868_DEFAULT_RX_BW;
	spirit1InitConfig.xtal_freq = SPIRIT1_868_DEFAULT_XTAL;

	return spirit1HwInit(spirit1Data, &spirit1InitConfig, spiDevId);
}
#endif

#if PLATFORM_ENABLE_CC2500
/**
 *
 * @param cc2500Data
 * @param spiDevId
 * @return
 */
static retval_t cc2500Init(cc2500Data_t* cc2500Data, uint32_t spiDevId){
	cc2500Config_t cc2500InitConfig;

	cc2500InitConfig.baud_rate = CC2500_DEFAULT_BAUD_RATE;
	cc2500InitConfig.channel_num = CC2500_DEFAULT_CHANNEL;
	cc2500InitConfig.channel_spacing = CC2500_DEFAULT_SPACING;
	cc2500InitConfig.freq_deviation = CC2500_DEFAULT_FREQ_DEV;
	cc2500InitConfig.modulation = CC2500_DEFAULT_MODULATION;
	cc2500InitConfig.modulation_freq = CC2500_DEFAULT_FREQ;
	cc2500InitConfig.output_power = CC2500_DEFAULT_OUT_POWER;
	cc2500InitConfig.rx_bandwidth = CC2500_DEFAULT_RX_BW;

	return cc2500HwInit(cc2500Data, &cc2500InitConfig, spiDevId);
}
#endif

#if PLATFORM_ENABLE_SPIRIT1_433
/**
 *
 * @param args
 */
static void spirit1IrqCb433(const void* args){
	lastInterruptTime = osKernelSysTick();
	osMessagePut (interruptMessageQId, SPIRIT1_433_CHANNEL, 0);
}
#endif


#if PLATFORM_ENABLE_SPIRIT1_868
/**
 *
 * @param args
 */
static void spirit1IrqCb868(const void* args){
	lastInterruptTime = osKernelSysTick();
	osMessagePut (interruptMessageQId, SPIRIT1_868_CHANNEL, 0);

}
#endif

#if PLATFORM_ENABLE_CC2500
/**
 *
 * @param args
 */
static void cc2500IrqCb(const void* args){
	lastInterruptTime = osKernelSysTick();
	osMessagePut (interruptMessageQId, CC2500_BASE_CHANNEL, 0);
}
#endif

#endif
#endif
