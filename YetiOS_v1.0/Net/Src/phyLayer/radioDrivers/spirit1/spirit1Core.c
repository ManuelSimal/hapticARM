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
 * spirit1Core.c
 *
 *  Created on: 4 de dic. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file spirit1Core.c
 */

#include <spirit1Core.h>
#include "SPIRIT_config.h"
#include "SPIRIT_Aes.h"


/* PACKET DEFAULT CONFIG */
#define PREAMBLE_LENGTH             PKT_PREAMBLE_LENGTH_04BYTES
#define SYNC_LENGTH                 PKT_SYNC_LENGTH_4BYTES
#define SYNC_WORD                   0xC5C5C5C5
#define LENGTH_TYPE                 PKT_LENGTH_VAR
#define LENGTH_WIDTH                8								//8 bits length width
#define CRC_MODE                    PKT_CRC_MODE_8BITS
#define CONTROL_LENGTH              PKT_CONTROL_LENGTH_0BYTES
#define EN_FEC                      S_ENABLE
#define EN_WHITENING                S_ENABLE
#define EN_ADDRESS                  S_DISABLE

/* DEFAULT XTAL PPM */
#define SPIRIT1_XTAL_PPM		0


#define SPIRIT1_MAX_FIFO_LEN	96

#define AES_TIMEOUT		100

#define SPIRIT1_SEND_TIMEOUT 	50
#define SPIRIT1_RCV_TIMEOUT 	50

static retval_t spirit1SetMode(spirit1Data_t* spirit1Data, spirit1_state_t state);

/**
 *
 * @param cs_pin
 * @param sdn_pin
 * @param gpio3_int_pin
 * @param interrupt_cb_func
 * @return
 */
spirit1Data_t* newSpirit1Data(uint32_t cs_pin, uint32_t sdn_pin, uint32_t gpio3_int_pin, ytThreadFunc_t interrupt_cb_func, void* args){
	spirit1Data_t* new_spirit1Data;
	if((new_spirit1Data = (spirit1Data_t*) pvPortMalloc(sizeof(spirit1Data_t))) == NULL){
		return NULL;
	}
	new_spirit1Data->cs_pin = cs_pin;
	new_spirit1Data->gpio3_int_pin = gpio3_int_pin;
	new_spirit1Data->sdn_pin = sdn_pin;
	new_spirit1Data->spi_id = 0;
	new_spirit1Data->interrupt_cb_func = interrupt_cb_func;
	new_spirit1Data->interrupt_cb_arg = args;
	new_spirit1Data->transmitting_packet = 0;
	new_spirit1Data->receiving_packet = 0;
	new_spirit1Data->checking_rssi = 0;
	new_spirit1Data->spirit1_rx_time = 0;
	new_spirit1Data->spirit1_tx_time = 0;
	new_spirit1Data->spirit1_mutex_id = ytMutexCreate();

	new_spirit1Data->spirit1_aes_semph_id = ytSemaphoreCreate(1);
	osSemaphoreWait(new_spirit1Data->spirit1_aes_semph_id, osWaitForever);

	return new_spirit1Data;
}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t deleteSpirit1Data(spirit1Data_t* spirit1Data){
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);

	osSemaphoreDelete(spirit1Data->spirit1_aes_semph_id);
	osMutexDelete(spirit1Data->spirit1_mutex_id);
	vPortFree(spirit1Data);
	return RET_OK;

}


/**
 *
 * @param spirit1Data
 * @param spirit1_init_config
 * @return
 */
retval_t spirit1HwInit(spirit1Data_t* spirit1Data, spirit1Config_t* spirit1InitConfig, uint32_t spiDevId){
	spirit1_status_t status_b;
	uint32_t t1;

	SGpioInit xGpioIRQ={
	  SPIRIT_GPIO_3,
	  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
	  SPIRIT_GPIO_DIG_OUT_IRQ
	};

	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);

	spirit1_arch_init(spirit1Data, spiDevId);	/*Initialize HW*/

	spirit1_arch_power_on(spirit1Data);	/*Power on and wait*/
	osDelay(2);

	spirit1_arch_send_cmd_strobe(spirit1Data, COMMAND_READY);	/*Send Initial Strobe*/

	spirit1_arch_send_cmd_strobe(spirit1Data, COMMAND_SRES);	/*Reset*/

	status_b = spirit1_arch_get_status(spirit1Data);
	t1 = osKernelSysTick();
		while(status_b.SPIRIT1_STATE != SPIRIT1_STATE_READY){

			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);

			if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
				spirit1_arch_power_off(spirit1Data);
				osMutexRelease(spirit1Data->spirit1_mutex_id);
				return RET_ERROR;
			}

			status_b = spirit1_arch_get_status(spirit1Data);
		}

	if(spirit1CheckDeviceInfoReg(spirit1Data) != RET_OK){	/*Check info register*/
		spirit1_arch_power_off(spirit1Data);
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}

	if(spirit1SetMode(spirit1Data, SPIRIT1_STATE_STANDBY) != RET_OK){		/*Standby mode*/
		spirit1_arch_power_off(spirit1Data);
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}

	if (spirit1InitConfig->xtal_freq < DOUBLE_XTAL_THR) {
		SpiritRadioSetDigDiv(spirit1Data, S_DISABLE);
		SpiritRadioSetRefDiv(spirit1Data, S_DISABLE);
	}
	else {
		SpiritRadioSetDigDiv(spirit1Data, S_ENABLE);
		SpiritRadioSetRefDiv(spirit1Data, S_ENABLE);
	}

	if(spirit1SetMode(spirit1Data, SPIRIT1_STATE_READY) != RET_OK){
		spirit1_arch_power_off(spirit1Data);
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}

	SpiritRadioSetDatarate(spirit1Data, spirit1InitConfig->baud_rate, spirit1InitConfig->xtal_freq);
	SpiritRadioSetFrequencyDev(spirit1Data, spirit1InitConfig->freq_deviation, spirit1InitConfig->xtal_freq);
	SpiritRadioSetChannelBW(spirit1Data, spirit1InitConfig->rx_bandwidth, spirit1InitConfig->xtal_freq);
	SpiritRadioSetModulation(spirit1Data, spirit1InitConfig->modulation);

	SpiritSetAnalogIfOffset(spirit1Data, spirit1InitConfig->xtal_freq);
	SpiritSetDigitalIfOffset(spirit1Data, spirit1InitConfig->xtal_freq);

	/* Sets Xtal configuration */
	if (spirit1InitConfig->xtal_freq > DOUBLE_XTAL_THR) {
		SpiritRadioSetXtalFlag(spirit1Data, XTAL_FLAG((spirit1InitConfig->xtal_freq / 2)));
	} else {
		SpiritRadioSetXtalFlag(spirit1Data, XTAL_FLAG(spirit1InitConfig->xtal_freq));
	}

	SpiritRadioSetChannelSpace(spirit1Data, spirit1InitConfig->channel_spacing, spirit1InitConfig->xtal_freq);
	SpiritRadioSetChannel(spirit1Data, (uint8_t)spirit1InitConfig->channel_num);

	/* Enable the freeze option of the AFC on the SYNC word */
	SpiritRadioAFCFreezeOnSync(spirit1Data, S_ENABLE);

	SpiritRadioSetFrequencyBase(spirit1Data, spirit1InitConfig->modulation_freq, spirit1InitConfig->xtal_freq);

	SpiritRadioSetFrequencyOffsetPpm(spirit1Data, SPIRIT1_XTAL_PPM, spirit1InitConfig->xtal_freq);

	SpiritRadioSetPALeveldBm(spirit1Data, 0, spirit1InitConfig->output_power, spirit1InitConfig->xtal_freq);
	SpiritRadioSetPALevelMaxIndex(spirit1Data, 0);

	PktBasicInit x_basic_init = {
		  PREAMBLE_LENGTH,
		  SYNC_LENGTH,
		  SYNC_WORD,
		  LENGTH_TYPE,
		  LENGTH_WIDTH,
		  CRC_MODE,
		  CONTROL_LENGTH,
		  EN_ADDRESS,
		  EN_FEC,
		  EN_WHITENING
	};
	SpiritPktBasicInit(spirit1Data, &x_basic_init);

	SpiritRadioPersistenRx(spirit1Data, S_ENABLE);
	SpiritQiSetSqiThreshold(spirit1Data, SQI_TH_0);
	SpiritQiSqiCheck(spirit1Data, S_ENABLE);
	SpiritTimerSetRxTimeoutStopCondition(spirit1Data, SQI_ABOVE_THRESHOLD);	/*Important to enable RX Data interrupt*/


	SpiritGpioInit(spirit1Data, &xGpioIRQ);	/*Init GPIO3 as IRQ output pin*/

	SpiritIrqDeInit(spirit1Data, NULL);			/*Disable and clear al Spirit IRQs*/
	SpiritIrqClearStatus(spirit1Data);

	SpiritIrq(spirit1Data, VALID_SYNC, S_ENABLE);		/*Enable used interrupts*/
	SpiritIrq(spirit1Data, TX_DATA_SENT, S_ENABLE);
	SpiritIrq(spirit1Data, RX_DATA_READY, S_ENABLE);
	SpiritIrq(spirit1Data, TX_FIFO_ERROR, S_ENABLE);
	SpiritIrq(spirit1Data, RX_FIFO_ERROR, S_ENABLE);
	SpiritIrq(spirit1Data, AES_END, S_ENABLE);

	spirit1Data->xtal_freq = spirit1InitConfig->xtal_freq;

	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);
	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FTX);

	if(spirit1SetMode(spirit1Data, SPIRIT1_STATE_STANDBY) != RET_OK){		/*Standby mode at startup*/
		spirit1_arch_power_off(spirit1Data);
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}

	spirit1_arch_disable_interrupt(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1HwDeInit(spirit1Data_t* spirit1Data){

	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	spirit1_arch_power_off(spirit1Data);
	spirit1_arch_deInit(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);

	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param baud_rate
 * @return
 */
retval_t spirit1SetBaudRate(spirit1Data_t* spirit1Data, uint32_t baud_rate){

	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	if(spirit1Data->transmitting_packet || spirit1Data->receiving_packet || spirit1Data->checking_rssi){
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}

	if (baud_rate > 128000){
		SpiritRadioSetDatarate(spirit1Data, baud_rate, spirit1Data->xtal_freq);
		SpiritRadioSetFrequencyDev(spirit1Data, 128e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelBW(spirit1Data, 540e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelSpace(spirit1Data, 540e3, spirit1Data->xtal_freq);
	}
	else if(baud_rate > 64000){
		SpiritRadioSetDatarate(spirit1Data, baud_rate, spirit1Data->xtal_freq);
		SpiritRadioSetFrequencyDev(spirit1Data, 64e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelBW(spirit1Data, 280e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelSpace(spirit1Data, 280e3, spirit1Data->xtal_freq);
	}
	else if(baud_rate > 40000){
		SpiritRadioSetDatarate(spirit1Data, baud_rate, spirit1Data->xtal_freq);
		SpiritRadioSetFrequencyDev(spirit1Data, 40e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelBW(spirit1Data, 150e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelSpace(spirit1Data, 150e3, spirit1Data->xtal_freq);
	}
	else if(baud_rate > 20000){
		SpiritRadioSetDatarate(spirit1Data, baud_rate, spirit1Data->xtal_freq);
		SpiritRadioSetFrequencyDev(spirit1Data, 20e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelBW(spirit1Data, 100e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelSpace(spirit1Data, 100e3, spirit1Data->xtal_freq);
	}
	else if(baud_rate > 10000){
		SpiritRadioSetDatarate(spirit1Data, baud_rate, spirit1Data->xtal_freq);
		SpiritRadioSetFrequencyDev(spirit1Data, 10e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelBW(spirit1Data, 58e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelSpace(spirit1Data, 58e3, spirit1Data->xtal_freq);
	}
	else if(baud_rate > 2000){
		SpiritRadioSetDatarate(spirit1Data, baud_rate, spirit1Data->xtal_freq);
		SpiritRadioSetFrequencyDev(spirit1Data, 8e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelBW(spirit1Data, 58e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelSpace(spirit1Data, 58e3, spirit1Data->xtal_freq);
	}
	else{
		SpiritRadioSetDatarate(spirit1Data, baud_rate, spirit1Data->xtal_freq);
		SpiritRadioSetFrequencyDev(spirit1Data, 4.8e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelBW(spirit1Data, 58e3, spirit1Data->xtal_freq);
		SpiritRadioSetChannelSpace(spirit1Data, 58e3, spirit1Data->xtal_freq);
	}
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param output_power
 * @return
 */
retval_t spirit1SetOutputPower(spirit1Data_t* spirit1Data, int16_t output_power){
	if(spirit1Data == NULL){
		return RET_ERROR;
	}
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	if(spirit1Data->transmitting_packet || spirit1Data->receiving_packet || spirit1Data->checking_rssi){
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}
	SpiritRadioSetPALeveldBm(spirit1Data, 0, output_power, spirit1Data->xtal_freq);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param channel_num
 * @return
 */
retval_t spirit1SetChannelNum(spirit1Data_t* spirit1Data, uint16_t channel_num){
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	if(spirit1Data->transmitting_packet || spirit1Data->receiving_packet || spirit1Data->checking_rssi){
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}
	SpiritRadioSetChannel(spirit1Data, (uint8_t)channel_num);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param base_freq
 * @return
 */
retval_t spirit1SetBaseFreq(spirit1Data_t* spirit1Data, uint32_t base_freq){
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	if(spirit1Data->transmitting_packet || spirit1Data->receiving_packet || spirit1Data->checking_rssi){
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}
	SpiritRadioSetFrequencyBase(spirit1Data, base_freq, spirit1Data->xtal_freq);
	SpiritRadioSetFrequencyOffsetPpm(spirit1Data, SPIRIT1_XTAL_PPM, spirit1Data->xtal_freq);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param packet_size
 * @return
 */
retval_t spirit1SetPacketSize(spirit1Data_t* spirit1Data, uint16_t packet_size){
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	if(spirit1Data->transmitting_packet || spirit1Data->receiving_packet || spirit1Data->checking_rssi){
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}
	SpiritPktBasicSetPayloadLength(spirit1Data, packet_size);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param data
 * @param size
 * @return
 */
retval_t spirit1SendData(spirit1Data_t* spirit1Data, uint8_t* data, uint16_t size){

	if(size > SPIRIT1_MAX_FIFO_LEN-1){
		return RET_ERROR;
	}

	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);

	if(spirit1Data->transmitting_packet){
		if((osKernelSysTick() - spirit1Data->spirit1_tx_time) > SPIRIT1_SEND_TIMEOUT){
			spirit1Data->transmitting_packet = 0;
		}
		else{
			osMutexRelease(spirit1Data->spirit1_mutex_id);
			return RET_ERROR;
		}
	}

	spirit1Data->spirit1_tx_time = osKernelSysTick();
	spirit1Data->transmitting_packet = 1;

	if (spirit1Data->receiving_packet){
		if((osKernelSysTick() - spirit1Data->spirit1_rx_time) > SPIRIT1_RCV_TIMEOUT){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);
			spirit1Data->receiving_packet = 0;
		}
		spirit1Data->transmitting_packet = 0;
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return RET_ERROR;
	}


	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FTX);

	SpiritPktBasicSetPayloadLength(spirit1Data, size);
	spirit1_arch_write_txfifo(spirit1Data, data, size);

	spirit1_arch_enable_interrupt(spirit1Data);

	spirit1SetMode(spirit1Data, SPIRIT1_STATE_TX);

	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1SetModeRx(spirit1Data_t* spirit1Data){
	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);

	if(spirit1Data->transmitting_packet){
		if((osKernelSysTick() - spirit1Data->spirit1_tx_time) > SPIRIT1_SEND_TIMEOUT){
			spirit1Data->transmitting_packet = 0;
		}
		else{
			osMutexRelease(spirit1Data->spirit1_mutex_id);
			return RET_ERROR;
		}
	}

	if (spirit1Data->receiving_packet){
		if((osKernelSysTick() - spirit1Data->spirit1_rx_time) > SPIRIT1_RCV_TIMEOUT){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);
			spirit1Data->receiving_packet = 0;
		}
		else{
			osMutexRelease(spirit1Data->spirit1_mutex_id);
			return RET_ERROR;
		}
	}

	spirit1_arch_enable_interrupt(spirit1Data);

	spirit1SetMode(spirit1Data, SPIRIT1_STATE_RX);	//Me pongo en modo RX este donde este

	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1SetModeSleep(spirit1Data_t* spirit1Data){
	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);

	if(spirit1Data->transmitting_packet){
		if((osKernelSysTick() - spirit1Data->spirit1_tx_time) > SPIRIT1_SEND_TIMEOUT){
			spirit1Data->transmitting_packet = 0;
		}
		else{
			osMutexRelease(spirit1Data->spirit1_mutex_id);
			return RET_ERROR;
		}
	}

	if (spirit1Data->receiving_packet){
		if((osKernelSysTick() - spirit1Data->spirit1_rx_time) > SPIRIT1_RCV_TIMEOUT){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);
			spirit1Data->receiving_packet = 0;
		}
		else{
			osMutexRelease(spirit1Data->spirit1_mutex_id);
			return RET_ERROR;
		}
	}

	spirit1Data->transmitting_packet = 0;
	spirit1Data->receiving_packet = 0;

	spirit1_arch_disable_interrupt(spirit1Data);

	spirit1SetMode(spirit1Data, SPIRIT1_STATE_STANDBY);	//Me pongo en modo STANDBY este donde este

	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FTX);	//Limpio las fifos
	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);	//Limpio las fifos


	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1SetModeIdle(spirit1Data_t* spirit1Data){
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	spirit1Data->transmitting_packet = 0;
	spirit1Data->receiving_packet = 0;

	spirit1_arch_disable_interrupt(spirit1Data);

	spirit1SetMode(spirit1Data, SPIRIT1_STATE_READY);	//Me pongo en modo READY este donde este

	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FTX);	//Limpio las fifos
	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);	//Limpio las fifos
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1PowerOff(spirit1Data_t* spirit1Data){
	retval_t ret;
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	spirit1_arch_disable_interrupt(spirit1Data);
	ret = spirit1_arch_power_off(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return ret;
}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1PowerOn(spirit1Data_t* spirit1Data){
	retval_t ret;
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	ret = spirit1_arch_power_on(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return ret;
}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1CheckDeviceInfoReg(spirit1Data_t* spirit1Data){
	uint16_t tmp_reg_val;


	spirit1_arch_read_multi_reg(spirit1Data, DEVICE_INFO1_PARTNUM, (uint8_t*) &tmp_reg_val, 2);

	if(tmp_reg_val != 0x3001){
		return RET_ERROR;
	}

	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @return
 */
float32_t spirit1CheckChannelRssi(spirit1Data_t* spirit1Data){
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	if(spirit1Data->transmitting_packet || spirit1Data->receiving_packet || spirit1Data->checking_rssi){
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return 999;
	}
	float32_t val;
	spirit1Data->checking_rssi = 1;

	if(spirit1SetMode(spirit1Data, SPIRIT1_STATE_RX) != RET_OK){
		spirit1Data->checking_rssi = 0;
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return 999;
	}

	spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_SABORT);

	spirit1Data->checking_rssi = 0;


	val = SpiritQiGetRssidBm(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return val;
}

/**
 *
 * @param spirit1Data
 * @return
 */
float32_t spirit1GetLastRssi(spirit1Data_t* spirit1Data){
	float32_t retVal;
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	retVal = SpiritQiGetRssidBm(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return retVal;
}

/**
 *
 * @param spirit1Data
 * @param data
 * @param key
 * @param size
 * @return
 */
retval_t spirit1AesEncryptData(spirit1Data_t* spirit1Data, uint8_t* data, uint8_t* key, uint16_t size){
	uint16_t i;
	uint16_t num_conv;
	uint16_t processed_bytes = 0;
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	num_conv = size/16;

	spirit1_arch_enable_interrupt(spirit1Data);
	SpiritAesMode(spirit1Data, S_ENABLE);

	if(size < 16){
		SpiritAesWriteKey(spirit1Data, key);
		SpiritAesWriteDataIn(spirit1Data, data, size);
		spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_AES_ENC);
		if(osSemaphoreWait(spirit1Data->spirit1_aes_semph_id, AES_TIMEOUT) != RET_OK){
			SpiritAesMode(spirit1Data, S_DISABLE);
			osMutexRelease(spirit1Data->spirit1_mutex_id);
			return RET_ERROR;
		}
		SpiritAesReadDataOut(spirit1Data, data, size);
		processed_bytes += size;
	}

	else{
		for(i=0; i<num_conv; i++){
			SpiritAesWriteKey(spirit1Data, key);
			SpiritAesWriteDataIn(spirit1Data, &data[i*16], 16);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_AES_ENC);
			if(osSemaphoreWait(spirit1Data->spirit1_aes_semph_id, AES_TIMEOUT) != RET_OK){
				SpiritAesMode(spirit1Data, S_DISABLE);
				osMutexRelease(spirit1Data->spirit1_mutex_id);
				return RET_ERROR;
			}
			SpiritAesReadDataOut(spirit1Data, &data[i*16], 16);
			processed_bytes += 16;
		}

		if(size-processed_bytes > 0){
			SpiritAesWriteKey(spirit1Data, key);
			SpiritAesWriteDataIn(spirit1Data, &data[i*16], (size-processed_bytes));
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_AES_ENC);
			if(osSemaphoreWait(spirit1Data->spirit1_aes_semph_id, AES_TIMEOUT) != RET_OK){
				SpiritAesMode(spirit1Data, S_DISABLE);
				osMutexRelease(spirit1Data->spirit1_mutex_id);
				return RET_ERROR;
			}
			SpiritAesReadDataOut(spirit1Data, &data[i*16], (size-processed_bytes));
			processed_bytes += (size-processed_bytes);
		}
	}

	SpiritAesMode(spirit1Data, S_DISABLE);
	spirit1_arch_disable_interrupt(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param data
 * @param key
 * @param size
 * @return
 */
retval_t spirit1AesDecryptData(spirit1Data_t* spirit1Data, uint8_t* data, uint8_t* key, uint16_t size){
	uint16_t i;
	uint16_t num_conv;
	uint16_t processed_bytes = 0;
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	num_conv = size/16;

	spirit1_arch_enable_interrupt(spirit1Data);
	SpiritAesMode(spirit1Data, S_ENABLE);

	if(size < 16){
		SpiritAesWriteKey(spirit1Data, key);
		SpiritAesWriteDataIn(spirit1Data, data, size);
		spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_AES_KEYDESC);
		if(osSemaphoreWait(spirit1Data->spirit1_aes_semph_id, AES_TIMEOUT) != RET_OK){
			SpiritAesMode(spirit1Data, S_DISABLE);
			osMutexRelease(spirit1Data->spirit1_mutex_id);
			return RET_ERROR;
		}
		SpiritAesReadDataOut(spirit1Data, data, size);
		processed_bytes += size;
	}

	else{
		for(i=0; i<num_conv; i++){
			SpiritAesWriteKey(spirit1Data, key);
			SpiritAesWriteDataIn(spirit1Data, &data[i*16], 16);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_AES_KEYDESC);
			if(osSemaphoreWait(spirit1Data->spirit1_aes_semph_id, AES_TIMEOUT) != RET_OK){
				SpiritAesMode(spirit1Data, S_DISABLE);
				osMutexRelease(spirit1Data->spirit1_mutex_id);
				return RET_ERROR;
			}
			SpiritAesReadDataOut(spirit1Data, &data[i*16], 16);
			processed_bytes += 16;
		}

		if(size-processed_bytes > 0){
			SpiritAesWriteKey(spirit1Data, key);
			SpiritAesWriteDataIn(spirit1Data, &data[i*16], (size-processed_bytes));
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_AES_KEYDESC);
			if(osSemaphoreWait(spirit1Data->spirit1_aes_semph_id, AES_TIMEOUT) != RET_OK){
				SpiritAesMode(spirit1Data, S_DISABLE);
				osMutexRelease(spirit1Data->spirit1_mutex_id);
				return RET_ERROR;
			}
			SpiritAesReadDataOut(spirit1Data, &data[i*16], (size-processed_bytes));
			processed_bytes += (size-processed_bytes);
		}
	}

	SpiritAesMode(spirit1Data, S_DISABLE);
	spirit1_arch_disable_interrupt(spirit1Data);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param packet_data
 * @param size
 * @return
 */
uint16_t spirit1IrqRoutine(spirit1Data_t* spirit1Data){

	SpiritIrqs x_irq_status;
	uint16_t ret = 0;
	SpiritIrqGetStatus(spirit1Data, &x_irq_status);
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	/* AES operation DONE */
	if(x_irq_status.IRQ_AES_END) {
		osSemaphoreRelease(spirit1Data->spirit1_aes_semph_id);
	}

	/* RX FIFO ERROR: Flush and exit */
	if(x_irq_status.IRQ_RX_FIFO_ERROR) {
		spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);
		spirit1Data->receiving_packet = 0;
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return ret;
	}

	/* TX FIFO ERROR: Flush and exit */
	if(x_irq_status.IRQ_TX_FIFO_ERROR) {
		spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FTX);
		spirit1Data->transmitting_packet = 0;
		osMutexRelease(spirit1Data->spirit1_mutex_id);
		return ret;
	}


	/* The IRQ_VALID_SYNC is used to notify a new packet is coming */
	if(x_irq_status.IRQ_VALID_SYNC) {
		if((!spirit1Data->transmitting_packet) && (!spirit1Data->checking_rssi)){
			spirit1Data->receiving_packet = 1;
			spirit1Data->spirit1_rx_time = osKernelSysTick();
		}
	}

	/* The IRQ_TX_DATA_SENT notifies the packet received. Puts the SPIRIT1 in RX */
	if(x_irq_status.IRQ_TX_DATA_SENT) {
		if(spirit1Data->transmitting_packet){
			spirit1Data->transmitting_packet = 0;
			ret |= SPIRIT1_RET_PCKT_SENT;
		}
	}

	/* The IRQ_RX_DATA_READY notifies a new packet arrived */
	if(x_irq_status.IRQ_RX_DATA_READY) {
		if((!spirit1Data->receiving_packet) || (x_irq_status.IRQ_CRC_ERROR)){
			spirit1Data->receiving_packet = 0;
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_FRX);
		}
		else{
			ret |= SPIRIT1_RET_PCKT_RCV;
		}
	}

	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return ret;
}

/**
 *
 * @param spirit1Data
 * @param num_rcv_bytes
 * @return
 */
retval_t spirit1ReadNumRcvBytes(spirit1Data_t* spirit1Data, uint8_t* num_rcv_bytes){

	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);

	(*num_rcv_bytes) = (uint8_t) SpiritPktBasicGetReceivedPktLength(spirit1Data);

	osMutexRelease(spirit1Data->spirit1_mutex_id);

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param packet_data
 * @param size
 * @return
 */
retval_t spirit1ReadRcvData(spirit1Data_t* spirit1Data, uint8_t* packet_data, uint16_t size){
	uint8_t fifo_elements;
	if(size > SPIRIT1_MAX_FIFO_LEN-1){
		return RET_ERROR;
	}
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	spirit1_arch_read_rxfifo(spirit1Data, packet_data, size);

	fifo_elements = SpiritLinearFifoReadNumElementsRxFifo(spirit1Data);
	if(fifo_elements > 0){
		spirit1Data->receiving_packet = 1;
		spirit1Data->spirit1_rx_time = osKernelSysTick();
	}
	else{
		spirit1Data->receiving_packet = 0;
	}
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spirit1Data
 * @param size
 * @return
 */
retval_t spirit1FlushLastRcvData(spirit1Data_t* spirit1Data, uint16_t size){
	uint8_t* null_read;
	uint8_t fifo_elements;
	if(size > SPIRIT1_MAX_FIFO_LEN){
		return RET_ERROR;
	}
	osMutexWait(spirit1Data->spirit1_mutex_id, osWaitForever);
	null_read = (uint8_t*) pvPortMalloc(size);
	spirit1_arch_read_rxfifo(spirit1Data, null_read, size);

	fifo_elements = SpiritLinearFifoReadNumElementsRxFifo(spirit1Data);
	if(fifo_elements > 0){
		spirit1Data->receiving_packet = 1;
		spirit1Data->spirit1_rx_time = osKernelSysTick();
	}
	else{
		spirit1Data->receiving_packet = 0;
	}
	vPortFree(null_read);
	osMutexRelease(spirit1Data->spirit1_mutex_id);
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param state
 * @return
 */
static retval_t spirit1SetMode(spirit1Data_t* spirit1Data, spirit1_state_t state){
	uint64_t t1;
	spirit1_status_t status_b;
	if(spirit1Data == NULL){
		return RET_ERROR;
	}
	switch(state){
	case SPIRIT1_STATE_STANDBY:
		status_b = spirit1_arch_get_status(spirit1Data);

		if(status_b.SPIRIT1_STATE != SPIRIT1_STATE_STANDBY){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_SABORT);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_STANDBY);

		}
		break;
	case SPIRIT1_STATE_SLEEP:
		status_b = spirit1_arch_get_status(spirit1Data);

		if(status_b.SPIRIT1_STATE != SPIRIT1_STATE_SLEEP){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_SABORT);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_SLEEP);


		}
		break;
	case SPIRIT1_STATE_READY:
		status_b = spirit1_arch_get_status(spirit1Data);

		if(status_b.SPIRIT1_STATE != SPIRIT1_STATE_READY){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_SABORT);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);

		}
		break;
	case SPIRIT1_STATE_RX:
		status_b = spirit1_arch_get_status(spirit1Data);

		if(status_b.SPIRIT1_STATE != SPIRIT1_STATE_RX){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_SABORT);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);

			status_b = spirit1_arch_get_status(spirit1Data);
			t1 = osKernelSysTick();
			while(status_b.SPIRIT1_STATE != SPIRIT1_STATE_READY){

				spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);

				if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
					return RET_ERROR;
				}

				status_b = spirit1_arch_get_status(spirit1Data);
			}

			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_RX);

		}
		break;
	case SPIRIT1_STATE_TX:
		status_b = spirit1_arch_get_status(spirit1Data);

		if(status_b.SPIRIT1_STATE != SPIRIT1_STATE_TX){
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_SABORT);
			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);

			status_b = spirit1_arch_get_status(spirit1Data);
			t1 = osKernelSysTick();
			while(status_b.SPIRIT1_STATE != SPIRIT1_STATE_READY){

				spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);

				if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
					return RET_ERROR;
				}

				status_b = spirit1_arch_get_status(spirit1Data);
			}

			spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_TX);

		}
		break;
	default:
		return RET_ERROR;
		break;
	}

	return RET_OK;

}
