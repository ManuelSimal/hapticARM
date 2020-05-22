/*
 * Copyright (c) 2018, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * cc2500Core.h
 *
 *  Created on: 3 de may. de 2018
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cc2500Core.h
 */
#ifndef YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CORE_H_
#define YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CORE_H_

#include "yetiOS.h"
#include "cc2500_const.h"

#define CC2500_RET_PCKT_RCV	0x01
#define CC2500_RET_PCKT_SENT	0x02

typedef struct cc2500Config_{
	uint32_t	baud_rate;			//In bps
	uint32_t 	channel_spacing;	//In Khz
	uint32_t	freq_deviation;		//In KHz
	uint32_t 	rx_bandwidth;		//In Khz
	uint32_t	modulation_freq;	//Freq in Hz
	int32_t 	output_power;		//In dBm. Positive or negative
	cc2500_modulation_t modulation;
	uint16_t 	channel_num;
}cc2500Config_t;

typedef struct cc2500Data_{
	uint32_t cs_pin;
	uint32_t gdo2_int_pin;
	deviceFileHandler_t* spi_id;
	uint32_t cc2500_rx_time;
	uint32_t cc2500_tx_time;
	osMutexId cc2500_mutex_id;
	uint32_t cc2500_aes_semph_id;
	ytThreadFunc_t interrupt_cb_func;
	void* interrupt_cb_arg;
	uint8_t transmitting_packet;
	uint8_t receiving_packet;
}cc2500Data_t;


cc2500Data_t* newCc2500Data(uint32_t cs_pin, uint32_t gdo2_int_pin, ytThreadFunc_t interrupt_cb_func, void* args);
retval_t deleteCc2500Data(cc2500Data_t* cc2500Data);
retval_t cc2500HwInit(cc2500Data_t* cc2500Data, cc2500Config_t* cc2500InitConfig, uint32_t spiDevId);
retval_t cc2500HwDeInit(cc2500Data_t* cc2500Data);
retval_t cc2500SendData(cc2500Data_t* cc2500Data, uint8_t* data, uint16_t size);
retval_t cc2500SetModeRx(cc2500Data_t* cc2500Data);
retval_t cc2500SetModeSleep(cc2500Data_t* cc2500Data);
retval_t cc2500SetModeIdle(cc2500Data_t* cc2500Data);
retval_t cc2500PowerOff(cc2500Data_t* cc2500Data);
retval_t cc2500PowerOn(cc2500Data_t* cc2500Data);
float32_t cc2500GetLastRssi(cc2500Data_t* cc2500Data);
uint16_t cc2500IrqRoutine(cc2500Data_t* cc2500Data);
retval_t cc2500ReadNumRcvBytes(cc2500Data_t* cc2500Data, uint8_t* num_recv_bytes);
retval_t cc2500ReadRcvData(cc2500Data_t* cc2500Data, uint8_t* packet_data, uint16_t size);
retval_t cc2500FlushLastRcvData(cc2500Data_t* cc2500Data, uint16_t size);
retval_t cc2500SetChannel(cc2500Data_t* cc2500Data, uint8_t channel);



#endif /* YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CORE_H_ */
