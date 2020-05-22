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
 * cc2500_config.c
 *
 *  Created on: 4 de may. de 2018
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cc2500_config.c
 */

#include "yetiOS.h"
#include "cc2500_arch.h"
#include "cc2500_config.h"
#include "cc2500_const.h"

#define CONST_2_POW_28	268435356
#define CONST_2_POW_27	134217728
#define CONST_2_POW_26	67108864
#define CONST_2_POW_25	33554432
#define CONST_2_POW_24	16777216
#define CONST_2_POW_23 	8388608
#define CONST_2_POW_22 	4194304
#define CONST_2_POW_21 	2097152
#define CONST_2_POW_20 	1048576
#define CONST_2_POW_19 	524288
#define CONST_2_POW_18 	262144
#define CONST_2_POW_17 	131072
#define CONST_2_POW_16 	65535
#define CONST_2_POW_15 	32768
#define CONST_2_POW_14 	16384
#define CONST_2_POW_13 	8192
#define CONST_2_POW_12 	4096
#define CONST_2_POW_11  2048
#define CONST_2_POW_10 	1024


static int32_t pow_value(uint8_t e_val);

/**
 *
 * @param cc2500Data
 * @param gdo_config
 * @return
 */
retval_t cc2500_config_gdo2_pin(cc2500Data_t* cc2500Data, gdo_config_t gdo_config){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	return cc2500_arch_write_reg(cc2500Data, CC2500_IOCFG2, (uint8_t) gdo_config);
}

/**
 *
 * @param cc2500Data
 * @param pckt_len
 * @return
 */
retval_t cc2500_set_packet_length(cc2500Data_t* cc2500Data, uint16_t pckt_len){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	return cc2500_arch_write_reg(cc2500Data, CC2500_PKTLEN, pckt_len);
}


/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_enable_fixed_packet_length(cc2500Data_t* cc2500Data){
	uint8_t reg_val;
	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	if(cc2500_arch_read_reg(cc2500Data, CC2500_PKTCTRL0, &reg_val) != RET_OK){
		return RET_ERROR;
	}

	reg_val &= 0xFC;

	cc2500_arch_write_reg(cc2500Data, CC2500_PKTCTRL0, reg_val);

	cc2500_arch_read_reg(cc2500Data, CC2500_PKTCTRL0, &reg_val);
	return RET_OK;
}


/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_disable_fixed_packet_length(cc2500Data_t* cc2500Data){
	uint8_t reg_val;
	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	if(cc2500_arch_read_reg(cc2500Data, CC2500_PKTCTRL0, &reg_val) != RET_OK){
		return RET_ERROR;
	}

	reg_val &= 0xFC;
	reg_val |= 0x01;

	return cc2500_arch_write_reg(cc2500Data, CC2500_PKTCTRL0, reg_val);
}

/**
 *
 * @param cc2500Data
 * @param channel
 * @return
 */
retval_t cc2500_set_channel_num(cc2500Data_t* cc2500Data, uint8_t channel){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	return cc2500_arch_write_reg(cc2500Data, CC2500_CHANNR, channel);
}


/**
 *
 * @param cc2500Data
 * @param freq				Desired Frequency Base in MHz
 * @return
 */
retval_t cc2500_set_base_freq(cc2500Data_t* cc2500Data, uint32_t freq){	//Freq in MHz
	uint8_t* write_reg_val;
	uint32_t freq_reg_val;
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	if((freq < CC2500_MIN_BASE_FREQ) || (freq>CC2500_MAX_BASE_FREQ)){
		return RET_ERROR;
	}
	freq_reg_val = (65535*freq)/CC2500_XTAL_FREQ;

	write_reg_val = (uint8_t*) &freq_reg_val;

	if(cc2500_arch_write_reg(cc2500Data, CC2500_FREQ0, (*write_reg_val)) != RET_OK){
		return RET_ERROR;
	}
	write_reg_val++;
	if(cc2500_arch_write_reg(cc2500Data, CC2500_FREQ1, (*write_reg_val)) != RET_OK){
		return RET_ERROR;
	}
	write_reg_val++;
	(*write_reg_val) |= 0x40;
	return cc2500_arch_write_reg(cc2500Data, CC2500_FREQ2, (*write_reg_val));
}



retval_t cc2500_set_rx_bw(cc2500Data_t* cc2500Data, uint32_t rx_bw){	//Rx Bandwidth in Hz

	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	uint8_t m_val, e_val, reg_val;

	if(rx_bw > 812500){
		return RET_ERROR;
	}

	if(cc2500_arch_read_reg(cc2500Data, CC2500_MDMCFG4, &reg_val) != RET_OK){
		return RET_ERROR;
	}

	if(rx_bw > 400000){
		e_val = 0;
		m_val = (uint8_t)(((CC2500_XTAL_FREQ*1000000)/(8*rx_bw)) - 4);
	}
	else if(rx_bw > 200000){
		e_val = 1;
		m_val = (uint8_t)(((CC2500_XTAL_FREQ*1000000)/(16*rx_bw)) - 4);
	}
	else if(rx_bw > 100000){
		e_val = 2;
		m_val = (uint8_t)(((CC2500_XTAL_FREQ*1000000)/(32*rx_bw)) - 4);
	}
	else{
		e_val = 3;
		m_val = (uint8_t)(((CC2500_XTAL_FREQ*1000000)/(64*rx_bw)) - 4);
	}

	if(m_val > 3){
		m_val = 3;
	}

	reg_val &= 0x0F;

	reg_val |= (e_val<<6) | (m_val<<4);

	return cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG4, reg_val);
}


/**
 *
 * @param cc2500Data
 * @param datarate
 * @return
 */
retval_t cc2500_set_datarate(cc2500Data_t* cc2500Data, uint32_t datarate){

	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	uint8_t e_val, m_val, reg_val;
	int32_t test_m_val;

	if(datarate > 500000){
		return RET_ERROR;
	}

	/*Set datarate first */
	e_val = 16;
	test_m_val = -1;
	while((test_m_val < 0) || (test_m_val > 255)){
		e_val--;
		test_m_val = (((pow_value(e_val)/CC2500_XTAL_FREQ) *  ((int32_t)datarate/1000))/1000) -256;
		if(!e_val){
			if((test_m_val < 0) || (test_m_val > 255)){
				return RET_ERROR;
			}
			break;
		}
	}

	m_val = (uint8_t) test_m_val;

	if(cc2500_arch_read_reg(cc2500Data, CC2500_MDMCFG4, &reg_val) != RET_OK){
		return RET_ERROR;
	}

	reg_val &= 0xF0;

	reg_val |= e_val;

	if(cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG4, reg_val) != RET_OK){
		return RET_ERROR;
	}

	if(cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG3, m_val) != RET_OK){
		return RET_ERROR;
	}

	/*Set FSCAL_3 (SmartRF values)*/
	if(datarate > 100000){
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCAL3, 0xEA) != RET_OK){
			return RET_ERROR;
		}
	}
	else{
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCAL3, 0xA9) != RET_OK){
			return RET_ERROR;
		}
	}

	/*Set MDMCFG1 NUM_PREAMBLE 8 for 500000 datarate  and set FSCAL0*/
	if(datarate == 500000){

		if(cc2500_arch_read_reg(cc2500Data, CC2500_MDMCFG1, &reg_val) != RET_OK){
			return RET_ERROR;
		}

		reg_val &= 0x8F;

		reg_val |= 0x40;

		if(cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG1, 0xEA) != RET_OK){
			return RET_ERROR;
		}

		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCAL0, 0x19) != RET_OK){
			return RET_ERROR;
		}

	}
	else{
		if(cc2500_arch_read_reg(cc2500Data, CC2500_MDMCFG1, &reg_val) != RET_OK){
			return RET_ERROR;
		}

		reg_val &= 0x8F;

		reg_val |= 0x20;

		if(cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG1, 0xEA) != RET_OK){
			return RET_ERROR;
		}

		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCAL0, 0x11) != RET_OK){
			return RET_ERROR;
		}
	}

	/*Set FOCCFG BSCFG FREND1 using SmartRF values*/
	if(datarate >= 250000){
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FOCCFG, 0x1D) != RET_OK){
			return RET_ERROR;
		}
		if(cc2500_arch_write_reg(cc2500Data, CC2500_BSCFG, 0x1C) != RET_OK){
			return RET_ERROR;
		}
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FREND1, 0xB6) != RET_OK){
			return RET_ERROR;
		}

	}
	else{
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FOCCFG, 0x16) != RET_OK){
			return RET_ERROR;
		}
		if(cc2500_arch_write_reg(cc2500Data, CC2500_BSCFG, 0x6C) != RET_OK){
			return RET_ERROR;
		}
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FREND1, 0x56) != RET_OK){
			return RET_ERROR;
		}
	}

	/*Set FSCTRL1 using SmartRF values*/
	if(datarate > 250000){
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCTRL1, 0x0C) != RET_OK){
			return RET_ERROR;
		}
	}
	else if(datarate > 10000){
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCTRL1, 0x0A) != RET_OK){
			return RET_ERROR;
		}
	}
	else if(datarate > 2400){
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCTRL1, 0x06) != RET_OK){
			return RET_ERROR;
		}
	}
	else{
		if(cc2500_arch_write_reg(cc2500Data, CC2500_FSCTRL1, 0x08) != RET_OK){
			return RET_ERROR;
		}
	}

	cc2500_arch_write_reg(cc2500Data, CC2500_FSCAL1, 0x00);

	return cc2500_arch_write_reg(cc2500Data, CC2500_MCSM0, 0x18);
}

/**
 *
 * @param cc2500Data
 * @param modulation
 * @return
 */
retval_t cc2500_set_modulation_format(cc2500Data_t* cc2500Data, cc2500_modulation_t modulation){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	uint8_t reg_val;

	reg_val = ((uint8_t) modulation) | 0x03;

	if(cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG2, reg_val) != RET_OK){
		return RET_ERROR;
	}

	if(modulation == CC2500_MSK){
		if(cc2500_arch_write_reg(cc2500Data, CC2500_DEVIATN, 0x00) != RET_OK){
			return RET_ERROR;
		}
	}
	return RET_OK;
}

/**
 *
 * @param cc2500Data
 * @param channel_spacing
 * @return
 */
retval_t cc2500_set_channel_spacing(cc2500Data_t* cc2500Data, uint32_t channel_spacing){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	uint8_t e_val, m_val, reg_val;
	int32_t test_m_val;

	if(channel_spacing > 405000){
		return RET_ERROR;
	}

	/*Set datarate first */
	e_val = 4;
	test_m_val = -1;
	while((test_m_val < 0) || (test_m_val > 255)){
		e_val--;
		test_m_val = (((pow_value((e_val+10))/CC2500_XTAL_FREQ) *  ((int32_t)channel_spacing/1000))/1000) - 256;
		if(!e_val){
			if((test_m_val < 0) || (test_m_val > 255)){
				return RET_ERROR;
			}
			break;
		}
	}

	m_val = (uint8_t) test_m_val;

	if(cc2500_arch_read_reg(cc2500Data, CC2500_MDMCFG1, &reg_val) != RET_OK){
		return RET_ERROR;
	}

	reg_val &= 0xFC;

	reg_val |= e_val;

	if(cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG1, reg_val) != RET_OK){
		return RET_ERROR;
	}

	return cc2500_arch_write_reg(cc2500Data, CC2500_MDMCFG0, m_val);

}

/**
 *
 * @param cc2500Data
 * @param freq_dev
 * @return
 */
retval_t cc2500_set_freq_dev(cc2500Data_t* cc2500Data, uint32_t freq_dev){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	uint8_t e_val, m_val, reg_val;
	int32_t test_m_val;

	if(freq_dev > 380000){
		return RET_ERROR;
	}

	/*Set datarate first */
	e_val = 8;
	test_m_val = -1;
	while((test_m_val < 0) || (test_m_val > 7)){
		e_val--;
		test_m_val = (((pow_value((e_val+11))/CC2500_XTAL_FREQ) *  ((int32_t)freq_dev/1000))/1000) - 8;
		if(!e_val){
			if((test_m_val < 0) || (test_m_val > 7)){
				return RET_ERROR;
			}
			break;
		}
	}

	m_val = (uint8_t) test_m_val;

	reg_val = m_val | (e_val<<4);

	return cc2500_arch_write_reg(cc2500Data, CC2500_DEVIATN, reg_val);
}

/**
 *
 * @param cc2500Data
 * @param out_pwr
 * @return
 */
retval_t cc2500_set_output_pwr(cc2500Data_t* cc2500Data, int32_t out_pwr){
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	uint8_t reg_val;

	switch(out_pwr){
	case 1:
		reg_val = 0xFF;
		break;
	case 0:
		reg_val = 0xFE;
		break;
	case -2:
		reg_val = 0xBB;
		break;
	case -4:
		reg_val = 0xA9;
		break;
	case -6:
		reg_val = 0x7F;
		break;
	case -8:
		reg_val = 0x6E;
		break;
	case -10:
		reg_val = 0x97;
		break;
	case -12:
		reg_val = 0xC6;
		break;
	case -14:
		reg_val = 0x8D;
		break;
	case -16:
		reg_val = 0x55;
		break;
	case -18:
		reg_val = 0x93;
		break;
	case -20:
		reg_val = 0x46;
		break;
	case -22:
		reg_val = 0x81;
		break;
	case -24:
		reg_val = 0x84;
		break;
	case -26:
		reg_val = 0xC0;
		break;
	case -28:
		reg_val = 0x44;
		break;
	case -30:
		reg_val = 0x50;
		break;
	default:
		return RET_ERROR;
		break;
	}

	return cc2500_arch_write_reg(cc2500Data, CC2500_PATABLE, reg_val);
}

/**
 *
 * @param e_val
 * @return
 */
static int32_t pow_value(uint8_t e_val){
	switch (e_val){

	case 18:
		return CONST_2_POW_10;
		break;

	case 17:
		return CONST_2_POW_11;
		break;

	case 16:
		return CONST_2_POW_12;
		break;

	case 15:
		return CONST_2_POW_13;
		break;

	case 14:
		return CONST_2_POW_14;
		break;

	case 13:
		return CONST_2_POW_15;
		break;

	case 12:
		return CONST_2_POW_16;
		break;

	case 11:
		return CONST_2_POW_17;
		break;

	case 10:
		return CONST_2_POW_18;
		break;

	case 9:
		return CONST_2_POW_19;
		break;

	case 8:
		return CONST_2_POW_20;
		break;

	case 7:
		return CONST_2_POW_21;
		break;

	case 6:
		return CONST_2_POW_22;
		break;

	case 5:
		return CONST_2_POW_23;
		break;

	case 4:
		return CONST_2_POW_24;
		break;

	case 3:
		return CONST_2_POW_25;
		break;

	case 2:
		return CONST_2_POW_26;
		break;

	case 1:
		return CONST_2_POW_27;
		break;

	case 0:
		return CONST_2_POW_28;
		break;

	default:
		return -1;
		break;
	}
}
