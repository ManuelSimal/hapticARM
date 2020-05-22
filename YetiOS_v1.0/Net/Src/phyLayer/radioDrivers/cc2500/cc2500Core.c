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
 * cc2500Core.c
 *
 *  Created on: 3 de may. de 2018
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cc2500Core.c
 */

#include <cc2500Core.h>
#include "yetiOS.h"
#include "cc2500_const.h"
#include "cc2500_arch.h"
#include "cc2500_config.h"

#define CC2500_MAX_FIFO_LENGTH	64

#define CC2500_SEND_TIMEOUT 	50
#define CC2500_RCV_TIMEOUT 		50

/**
 *
 * @param cs_pin
 * @param gdo2_int_pin
 * @param interrupt_cb_func
 * @param args
 * @return
 */
cc2500Data_t* newCc2500Data(uint32_t cs_pin, uint32_t gdo2_int_pin, ytThreadFunc_t interrupt_cb_func, void* args){
	cc2500Data_t* new_cc2500Data;
	if((new_cc2500Data = (cc2500Data_t*) pvPortMalloc(sizeof(cc2500Data_t))) == NULL){
		return NULL;
	}
	new_cc2500Data->cs_pin = cs_pin;
	new_cc2500Data->gdo2_int_pin = gdo2_int_pin;
	new_cc2500Data->spi_id = 0;
	new_cc2500Data->interrupt_cb_func = interrupt_cb_func;
	new_cc2500Data->interrupt_cb_arg = args;
	new_cc2500Data->transmitting_packet = 0;
	new_cc2500Data->receiving_packet = 0;

	new_cc2500Data->cc2500_rx_time = 0;
	new_cc2500Data->cc2500_tx_time = 0;

	new_cc2500Data->cc2500_mutex_id = ytMutexCreate();

	return new_cc2500Data;
}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t deleteCc2500Data(cc2500Data_t* cc2500Data){
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	osMutexDelete(cc2500Data->cc2500_mutex_id);
	vPortFree(cc2500Data);
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @param cc2500_init_config
 * @param spi_dev
 * @return
 */
retval_t cc2500HwInit(cc2500Data_t* cc2500Data, cc2500Config_t* cc2500InitConfig, uint32_t spiDevId){
	cc2500_status_t status;
	uint8_t read_val;

	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	if(cc2500_arch_init(cc2500Data, spiDevId) != RET_OK){
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	cc2500_arch_power_on(cc2500Data);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	osDelay(2);
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SRES);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	osDelay(5);
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	status = cc2500_arch_get_status(cc2500Data);
	if(status.CHIP_RDYn){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}

	/* Check partnum */
	if(cc2500_arch_read_status_reg(cc2500Data, CC2500_PARTNUM, &read_val) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(read_val != 0x80){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SIDLE);

	if(cc2500_config_gdo2_pin(cc2500Data, CC2500_SENT_RECEIVED) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(cc2500_disable_fixed_packet_length(cc2500Data) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}

	if(cc2500_set_channel_num(cc2500Data, cc2500InitConfig->channel_num) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(cc2500_set_base_freq(cc2500Data, cc2500InitConfig->modulation_freq) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(cc2500_set_rx_bw(cc2500Data, cc2500InitConfig->rx_bandwidth) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(cc2500_set_datarate(cc2500Data, cc2500InitConfig->baud_rate) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(cc2500_set_modulation_format(cc2500Data, cc2500InitConfig->modulation) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(cc2500_set_channel_spacing(cc2500Data, cc2500InitConfig->channel_spacing) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}
	if(cc2500InitConfig->modulation != CC2500_MSK){
		if(cc2500_set_freq_dev(cc2500Data, cc2500InitConfig->freq_deviation) != RET_OK){
			cc2500_arch_power_off(cc2500Data);
			cc2500_arch_deInit(cc2500Data);
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}
	if(cc2500_set_output_pwr(cc2500Data, cc2500InitConfig->output_power) != RET_OK){
		cc2500_arch_power_off(cc2500Data);
		cc2500_arch_deInit(cc2500Data);
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}

	cc2500_arch_write_reg(cc2500Data, CC2500_PKTCTRL1, 0x00);	//Disable status bytes

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFRX);
	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFTX);

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SPWD);
	cc2500_arch_disable_interrupt(cc2500Data);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500HwDeInit(cc2500Data_t* cc2500Data){

	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	cc2500_arch_power_off(cc2500Data);
	cc2500_arch_deInit(cc2500Data);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @param data
 * @param size
 * @return
 */
retval_t cc2500SendData(cc2500Data_t* cc2500Data, uint8_t* data, uint16_t size){
	cc2500_status_t status;

	if(size > CC2500_MAX_FIFO_LENGTH-1) {		//Alloctae one byte for size
		return RET_ERROR;
	}
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	if(cc2500Data->transmitting_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_tx_time) > CC2500_SEND_TIMEOUT){
			cc2500Data->transmitting_packet = 0;
		}
		else{
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}

	cc2500Data->cc2500_tx_time = osKernelSysTick();
	cc2500Data->transmitting_packet = 1;

	if (cc2500Data->receiving_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_rx_time) > CC2500_RCV_TIMEOUT){
			cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFRX);
			cc2500Data->receiving_packet = 0;
		}
		cc2500Data->transmitting_packet = 0;
		osMutexRelease(cc2500Data->cc2500_mutex_id);
		return RET_ERROR;
	}

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SIDLE);	//Voy a idle
	do{
		status =  cc2500_arch_get_status(cc2500Data);
		cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SIDLE);
	}while(status.CC2500_STATE != CC2500_STATE_IDLE);

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFTX);	//Primero limpio la tx fifo
	cc2500_arch_write_txfifo(cc2500Data, (uint8_t*) &size, 1);	//Fill the size byte
	cc2500_arch_write_txfifo(cc2500Data, data, size);			//Fill the tx fifo

	cc2500_arch_enable_interrupt(cc2500Data);

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_STX);	//Voy al estado tx

	osMutexRelease(cc2500Data->cc2500_mutex_id);

	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500SetModeRx(cc2500Data_t* cc2500Data){
	cc2500_status_t status;

	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	if(cc2500Data->transmitting_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_tx_time) > CC2500_SEND_TIMEOUT){
			cc2500Data->transmitting_packet = 0;
		}
		else{
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}

	if(cc2500Data->receiving_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_rx_time) > CC2500_RCV_TIMEOUT){
			cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFRX);
			cc2500Data->receiving_packet = 0;
		}
		else{
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}


	status =  cc2500_arch_get_status(cc2500Data);

	if(status.CC2500_STATE != CC2500_STATE_RX){
		do{
			status =  cc2500_arch_get_status(cc2500Data);
			cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SIDLE);
		}while(status.CC2500_STATE != CC2500_STATE_IDLE);


		cc2500_arch_enable_interrupt(cc2500Data);

		cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SRX);	//Me pongo en modo RX este donde este
	}
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500SetModeSleep(cc2500Data_t* cc2500Data){

	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	if(cc2500Data->transmitting_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_tx_time) > CC2500_SEND_TIMEOUT){
			cc2500Data->transmitting_packet = 0;
		}
		else{
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}

	if(cc2500Data->receiving_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_rx_time) > CC2500_RCV_TIMEOUT){
			cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFRX);
			cc2500Data->receiving_packet = 0;
		}
		else{
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}

	cc2500Data->transmitting_packet = 0;
	cc2500Data->receiving_packet = 0;

	cc2500_arch_disable_interrupt(cc2500Data);

	cc2500_arch_power_off(cc2500Data);

	osMutexRelease(cc2500Data->cc2500_mutex_id);

	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500SetModeIdle(cc2500Data_t* cc2500Data){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	if(cc2500Data->transmitting_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_tx_time) > CC2500_SEND_TIMEOUT){
			cc2500Data->transmitting_packet = 0;
		}
		else{
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}

	if(cc2500Data->receiving_packet){
		if((osKernelSysTick() - cc2500Data->cc2500_rx_time) > CC2500_RCV_TIMEOUT){
			cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFRX);
			cc2500Data->receiving_packet = 0;
		}
		else{
			osMutexRelease(cc2500Data->cc2500_mutex_id);
			return RET_ERROR;
		}
	}

	cc2500Data->transmitting_packet = 0;
	cc2500Data->receiving_packet = 0;

	cc2500_arch_power_on(cc2500Data);

	cc2500_arch_disable_interrupt(cc2500Data);


	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFRX);
	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFTX);

	cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SIDLE);

	osMutexRelease(cc2500Data->cc2500_mutex_id);

	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500PowerOff(cc2500Data_t* cc2500Data){
	retval_t ret;
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	cc2500_arch_disable_interrupt(cc2500Data);
	ret = cc2500_arch_power_off(cc2500Data);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return ret;
}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500PowerOn(cc2500Data_t* cc2500Data){
	retval_t ret;
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	ret = cc2500_arch_power_on(cc2500Data);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return ret;
}

/**
 *
 * @param cc2500Data
 * @return
 */
float32_t cc2500GetLastRssi(cc2500Data_t* cc2500Data){
	float32_t rssi;
	int8_t rssi_reg_val;
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	cc2500_arch_read_status_reg(cc2500Data, CC2500_RSSI, (uint8_t*) &rssi_reg_val);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	rssi = ((float32_t)rssi_reg_val)/2 - 72;
	return rssi;
}


/**
 *
 * @param cc2500Data
 * @return
 */
uint16_t cc2500IrqRoutine(cc2500Data_t* cc2500Data){

	uint16_t ret = 0;
	uint8_t rx_fifo_bytes;

	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	if(ytGpioPinGet(cc2500Data->gdo2_int_pin)){	//High level
		if(!cc2500Data->transmitting_packet){
			cc2500Data->receiving_packet = 1;
			cc2500Data->cc2500_rx_time = osKernelSysTick();
		}

	}
	else{	//Low level
		if(cc2500Data->transmitting_packet){
			cc2500Data->transmitting_packet = 0;
			cc2500_arch_disable_interrupt(cc2500Data);
			ret |= CC2500_RET_PCKT_SENT;
		}
		else if (cc2500Data->receiving_packet){
			cc2500_arch_read_status_reg(cc2500Data, CC2500_RXBYTES, &rx_fifo_bytes);
			if(rx_fifo_bytes & 0x80){	//FIFO OVERFLOW
				cc2500Data->receiving_packet = 0;
				cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SFRX);
				cc2500_arch_send_cmd_strobe(cc2500Data, CC2500_STROBE_SRX);
				osMutexRelease(cc2500Data->cc2500_mutex_id);
				return ret;
			}

			else{
				ret |= CC2500_RET_PCKT_RCV;
			}

		}
	}

	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return ret;
}

/**
 *
 * @param cc2500Data
 * @param num_recv_bytes
 * @return
 */
retval_t cc2500ReadNumRcvBytes(cc2500Data_t* cc2500Data, uint8_t* num_recv_bytes){
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	cc2500_arch_read_rxfifo(cc2500Data, num_recv_bytes, 1);

	osMutexRelease(cc2500Data->cc2500_mutex_id);

	return RET_OK;
}
/**
 *
 * @param cc2500Data
 * @param packet_data
 * @param size
 * @return
 */
retval_t cc2500ReadRcvData(cc2500Data_t* cc2500Data, uint8_t* packet_data, uint16_t size){
	if(size > CC2500_MAX_FIFO_LENGTH-1){
		return RET_ERROR;
	}
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);

	cc2500_arch_read_rxfifo(cc2500Data, packet_data, size);
	cc2500Data->receiving_packet = 0;

	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @param size
 * @return
 */
retval_t cc2500FlushLastRcvData(cc2500Data_t* cc2500Data, uint16_t size){
	uint8_t* null_read;
	if(size > CC2500_MAX_FIFO_LENGTH){
		return RET_ERROR;
	}
	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	null_read = (uint8_t*) pvPortMalloc(size);
	cc2500_arch_read_rxfifo(cc2500Data, null_read, size);
	cc2500Data->receiving_packet = 0;
	vPortFree(null_read);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @param channel
 * @return
 */
retval_t cc2500SetChannel(cc2500Data_t* cc2500Data, uint8_t channel){
	retval_t ret;

	osMutexWait(cc2500Data->cc2500_mutex_id, osWaitForever);
	ret = cc2500_set_channel_num(cc2500Data, channel);
	osMutexRelease(cc2500Data->cc2500_mutex_id);
	return ret;
}
