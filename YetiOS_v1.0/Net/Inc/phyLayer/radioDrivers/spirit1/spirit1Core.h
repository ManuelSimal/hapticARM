/*
 * Copyright (c) 2017, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * spirit1Core.h
 *
 *  Created on: 4 de dic. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file spirit1Core.h
 */
#ifndef APPLICATION_BOARD_PLATFORM_RADIO_SPIRIT1_INCLUDE_SPIRIT1_CORE_H_
#define APPLICATION_BOARD_PLATFORM_RADIO_SPIRIT1_INCLUDE_SPIRIT1_CORE_H_

#include "yetiOS.h"
#include "arm_math.h"

#define DOUBLE_XTAL_THR                         30000000
#define SPIRIT1_FUNCS_TIMEOUT					100

#define SPIRIT1_RET_PCKT_RCV	0x01
#define SPIRIT1_RET_PCKT_SENT	0x02

/**
 * @brief  SPIRIT Modulation enumeration
 */
typedef enum
{
  FSK         = 0x00, /*!< 2-FSK modulation selected */
  GFSK_BT05   = 0x50, /*!< GFSK modulation selected with BT=0.5 */
  GFSK_BT1    = 0x10, /*!< GFSK modulation selected with BT=1 */
  ASK_OOK     = 0x20, /*!< ASK or OOK modulation selected. ASK will use power ramping */
  MSK         = 0x30  /*!< MSK modulation selected */

}ModulationSelect;

typedef ModulationSelect modulation_t;

typedef struct spirit1_config_{
	uint32_t	baud_rate;			//In bps
	uint32_t 	channel_spacing;	//In Khz
	uint32_t	freq_deviation;		//In KHz
	uint32_t 	rx_bandwidth;		//In Khz
	uint32_t	modulation_freq;	//Freq in Hz
	uint32_t 	xtal_freq;			//Xtal Freq in Hz
	int16_t 	output_power;		//In dBm. Positive or negative
	modulation_t modulation;
	uint16_t 	channel_num;
}spirit1Config_t;

typedef struct __packed spirit1Data_{
	uint32_t cs_pin;
	uint32_t sdn_pin;
	uint32_t gpio3_int_pin;
	deviceFileHandler_t* spi_id;
	uint32_t spirit1_rx_time;
	uint32_t spirit1_tx_time;
	osMutexId spirit1_mutex_id;
	osSemaphoreId spirit1_aes_semph_id;
	ytThreadFunc_t interrupt_cb_func;
	void* interrupt_cb_arg;
	uint32_t xtal_freq;
	uint8_t transmitting_packet;
	uint8_t receiving_packet;
	uint8_t checking_rssi;
}spirit1Data_t;

spirit1Data_t* newSpirit1Data(uint32_t cs_pin, uint32_t sdn_pin, uint32_t gpio3_int_pin, ytThreadFunc_t interrupt_cb_func, void* args);
retval_t deleteSpirit1Data(spirit1Data_t* spirit1Data);

retval_t spirit1HwInit(spirit1Data_t* spirit1Data, spirit1Config_t* spirit1InitConfig, uint32_t spiDevId);
retval_t spirit1HwDeInit(spirit1Data_t* spirit1Data);

retval_t spirit1SetBaudRate(spirit1Data_t* spirit1Data, uint32_t baud_rate);
retval_t spirit1SetOutputPower(spirit1Data_t* spirit1Data, int16_t output_power);
retval_t spirit1SetChannelNum(spirit1Data_t* spirit1Data, uint16_t channel_num);
retval_t spirit1SetBaseFreq(spirit1Data_t* spirit1Data, uint32_t base_freq);
retval_t spirit1SetPacketSize(spirit1Data_t* spirit1Data, uint16_t packet_size);

retval_t spirit1SendData(spirit1Data_t* spirit1Data, uint8_t* data, uint16_t size);

retval_t spirit1SetModeRx(spirit1Data_t* spirit1Data);
retval_t spirit1SetModeSleep(spirit1Data_t* spirit1Data);
retval_t spirit1SetModeIdle(spirit1Data_t* spirit1Data);

retval_t spirit1PowerOff(spirit1Data_t* spirit1Data);
retval_t spirit1PowerOn(spirit1Data_t* spirit1Data);

retval_t spirit1CheckDeviceInfoReg(spirit1Data_t* spirit1Data);

float32_t spirit1CheckChannelRssi(spirit1Data_t* spirit1Data);
float32_t spirit1GetLastRssi(spirit1Data_t* spirit1Data);

retval_t spirit1AesEncryptData(spirit1Data_t* spirit1Data, uint8_t* data, uint8_t* key, uint16_t size);
retval_t spirit1AesDecryptData(spirit1Data_t* spirit1Data, uint8_t* data, uint8_t* key, uint16_t size);

/*Routine called by the thread which is released after the interrupt is launched*/
uint16_t spirit1IrqRoutine(spirit1Data_t* spirit1Data);

retval_t spirit1ReadNumRcvBytes(spirit1Data_t* spirit1Data, uint8_t* num_rcv_bytes);
retval_t spirit1ReadRcvData(spirit1Data_t* spirit1Data, uint8_t* packet_data, uint16_t size);
retval_t spirit1FlushLastRcvData(spirit1Data_t* spirit1Data, uint16_t size);

#include "spirit1_arch.h"

#endif /* APPLICATION_BOARD_PLATFORM_RADIO_SPIRIT1_INCLUDE_SPIRIT1_CORE_H_ */
