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
 * spirit1_arch.c
 *
 *  Created on: 8 de nov. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file spirit1_arch.c
 */

#include <spirit1Core.h>
#include "yetiOS.h"
#include "SPIRIT_Regs.h"


#define SPIRIT1_FIFO_ADDR		0xFF
#define MAX_FIFO_SIZE			96

#define WRITE_REG_HDR			0x00
#define READ_REG_HDR			0x01
#define COMMAND_HDR				0x80



static retval_t config_spi_device(spirit1Data_t* spirit1Data, uint32_t spiDevId);
/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1_arch_init(spirit1Data_t* spirit1Data, uint32_t spiDevId){
	//First open and configure the spi to be used
	if (config_spi_device(spirit1Data, spiDevId) != RET_OK){
		return RET_ERROR;
	}

	//Configure SDN PIN and GPIO3 INTERRUPT pin
	if(ytGpioInitPin(spirit1Data->sdn_pin, GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_NO_PULL) != RET_OK){
		return RET_ERROR;
	}
	ytGpioPinSet(spirit1Data->sdn_pin);		//Shutdown the spirit1

	if(ytGpioInitPin(spirit1Data->gpio3_int_pin, GPIO_PIN_INTERRUPT_FALLING, GPIO_PIN_PULLUP) != RET_OK){
		return RET_ERROR;
	}

	if(ytGpioPinSetCallback(spirit1Data->gpio3_int_pin, NULL, NULL) != RET_OK){
		return RET_ERROR;
	}

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1_arch_deInit(spirit1Data_t* spirit1Data){
	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	//Deinit Interrupt and SDN PIN
	if(ytGpioDeInitPin(spirit1Data->gpio3_int_pin)!= RET_OK){
		return RET_ERROR;
	}
	ytGpioPinSet(spirit1Data->sdn_pin);		//Shutdown the spirit1
	if(ytGpioDeInitPin(spirit1Data->sdn_pin)!= RET_OK){
		return RET_ERROR;
	}

	//Close Spi Device
	ytClose(spirit1Data->spi_id);
	spirit1Data->spi_id = 0;

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param reg_addr
 * @param read_val
 * @return
 */
retval_t spirit1_arch_read_reg(spirit1Data_t* spirit1Data, uint8_t reg_addr, uint8_t* read_val){

	uint8_t tx_val[3];
	uint8_t rx_val[3];
	ytSpiReadWriteBuff_t spi_read_write;

	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	tx_val[0] = READ_REG_HDR;
	tx_val[1] = reg_addr;

	spi_read_write.prx = &rx_val[0];
	spi_read_write.ptx = &tx_val[0];
	spi_read_write.size = 3;


	if(ytIoctl(spirit1Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		return RET_ERROR;
	}

	*read_val = rx_val[2];

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param reg_addr
 * @param write_val
 * @return
 */
retval_t spirit1_arch_write_reg(spirit1Data_t* spirit1Data, uint8_t reg_addr, uint8_t write_val){

	uint8_t tx_val[3];

	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	tx_val[0] = WRITE_REG_HDR;
	tx_val[1] = reg_addr;
	tx_val[2] = write_val;

	if(ytWrite(spirit1Data->spi_id, (void*) &tx_val[0], 3) != 3){
		return RET_ERROR;
	}

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param reg_addr
 * @param read_val
 * @param size
 * @return
 */
retval_t spirit1_arch_read_multi_reg(spirit1Data_t* spirit1Data, uint8_t reg_addr, uint8_t* read_val, uint16_t size){

	ytSpiReadWriteBuff_t spi_read_write;
	uint16_t i;
	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	spi_read_write.ptx = (uint8_t*) pvPortMalloc(2+size);
	spi_read_write.prx = (uint8_t*) pvPortMalloc(2+size);

	spi_read_write.ptx[0] = READ_REG_HDR;
	spi_read_write.ptx[1] = reg_addr;

	spi_read_write.size = 2+size;


	if(ytIoctl(spirit1Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		vPortFree(spi_read_write.ptx);
		vPortFree(spi_read_write.prx);
		return RET_ERROR;
	}

	for(i=0; i<size; i++){
		read_val[i] = spi_read_write.prx[i+2];
	}


	vPortFree(spi_read_write.ptx);
	vPortFree(spi_read_write.prx);

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param reg_addr
 * @param write_val
 * @param size
 * @return
 */
retval_t spirit1_arch_write_multi_reg(spirit1Data_t* spirit1Data, uint8_t reg_addr, uint8_t* write_val, uint16_t size){

	uint8_t* tx_val;
	uint16_t i;

	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	tx_val = (uint8_t*)pvPortMalloc(2+size);

	tx_val[0] = WRITE_REG_HDR;
	tx_val[1] = reg_addr;
	for(i=0; i<size; i++){
		tx_val[2+i] = write_val[i];
	}

	if(ytWrite(spirit1Data->spi_id, (void*) tx_val, 2+size) != 2+size){
		vPortFree(tx_val);
		return RET_ERROR;
	}
	vPortFree(tx_val);
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param tx_data
 * @param size
 * @return
 */
retval_t spirit1_arch_write_txfifo(spirit1Data_t* spirit1Data, uint8_t* tx_data, uint16_t size){

	uint8_t* tx_val;
	uint16_t i;

	if(spirit1Data == NULL){
		return RET_ERROR;
	}
	if(size > MAX_FIFO_SIZE){
		return RET_ERROR;
	}

	tx_val = (uint8_t*)pvPortMalloc(2+size);

	tx_val[0] = WRITE_REG_HDR;
	tx_val[1] = SPIRIT1_FIFO_ADDR;
	for(i=0; i<size; i++){
		tx_val[2+i] = tx_data[i];
	}

	if(ytWrite(spirit1Data->spi_id, (void*) tx_val, 2+size) != 2+size){
		vPortFree(tx_val);
		return RET_ERROR;
	}
	vPortFree(tx_val);
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @param rx_data
 * @param size
 * @return
 */
retval_t spirit1_arch_read_rxfifo(spirit1Data_t* spirit1Data, uint8_t* rx_data, uint16_t size){

	ytSpiReadWriteBuff_t spi_read_write;
	uint16_t i;
	if(spirit1Data == NULL){
		return RET_ERROR;
	}
	if(size > MAX_FIFO_SIZE){
		return RET_ERROR;
	}

	spi_read_write.ptx = (uint8_t*) pvPortMalloc(2+size);
	spi_read_write.prx = (uint8_t*) pvPortMalloc(2+size);

	spi_read_write.ptx[0] = READ_REG_HDR;
	spi_read_write.ptx[1] = SPIRIT1_FIFO_ADDR;

	spi_read_write.size = 2+size;

	if(ytIoctl(spirit1Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		vPortFree(spi_read_write.ptx);
		vPortFree(spi_read_write.prx);
		return RET_ERROR;
	}

	for(i=0; i<size; i++){
		rx_data[i] = spi_read_write.prx[i+2];
	}


	vPortFree(spi_read_write.ptx);
	vPortFree(spi_read_write.prx);

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
spirit1_status_t spirit1_arch_get_status(spirit1Data_t* spirit1Data){
	spirit1_status_t* ret_status;

	uint16_t ret_uint16;
	uint8_t tx_val[4];
	uint8_t rx_val[4];
	ytSpiReadWriteBuff_t spi_read_write;
	ret_status =  (spirit1_status_t*) &ret_uint16;
	ret_uint16 = 0xFFFF;
	if(spirit1Data == NULL){
		return *ret_status;
	}

	tx_val[0] = READ_REG_HDR;
	tx_val[1] = MC_STATE1_BASE;

	spi_read_write.prx = &rx_val[0];
	spi_read_write.ptx = &tx_val[0];
	spi_read_write.size = 4;


	if(ytIoctl(spirit1Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		return *ret_status;
	}

	ret_uint16 =  (((((uint16_t)(rx_val[2]))<<8) & 0xFF00) | (((uint16_t)(rx_val[3])) & 0x00FF));

	return *ret_status;

}

/**
 *
 * @param spirit1Data
 * @param cmd_strobe
 * @return
 */
retval_t spirit1_arch_send_cmd_strobe(spirit1Data_t* spirit1Data, cmd_strobe_t cmd_strobe){

	uint8_t tx_val[2];

	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	tx_val[0] = COMMAND_HDR;
	tx_val[1] = (uint8_t) cmd_strobe;

	if(ytWrite(spirit1Data->spi_id, (void*) &tx_val[0], 2) != 2){
		return RET_ERROR;
	}

	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1_arch_disable_interrupt(spirit1Data_t* spirit1Data){

	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	if(ytGpioPinSetCallback(spirit1Data->gpio3_int_pin, NULL, NULL) != RET_OK){
		return RET_ERROR;
	}
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1_arch_enable_interrupt(spirit1Data_t* spirit1Data){

	if(spirit1Data == NULL){
		return RET_ERROR;
	}

	if(ytGpioPinSetCallback(spirit1Data->gpio3_int_pin, spirit1Data->interrupt_cb_func, (void*) spirit1Data->interrupt_cb_arg) != RET_OK){
		return RET_ERROR;
	}
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1_arch_power_off(spirit1Data_t* spirit1Data){

	if(spirit1Data == NULL){
		return RET_ERROR;
	}
	ytGpioPinSet(spirit1Data->sdn_pin);		//Shutdown the spirit1
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
retval_t spirit1_arch_power_on(spirit1Data_t* spirit1Data){

	if(spirit1Data == NULL){
		return RET_ERROR;
	}
	ytGpioPinReset(spirit1Data->sdn_pin);		//Enable Spirit1
	return RET_OK;

}

/**
 *
 * @param spirit1Data
 * @return
 */
static retval_t config_spi_device(spirit1Data_t* spirit1Data, uint32_t spiDevId){

	ytIoctlCmd_t spi_ioctl_cmd;
	ytSpiPolarity_t spiPol = SPI_POL_LOW;
	ytSpiEdge_t spiEdge = SPI_EDGE_FIRST;
	ytSpiMode_t spiMode = SPI_MASTER;
	uint32_t spiSpeed = SPIRIT1_SPI_SPEED;
	if ((spirit1Data->spi_id = ytOpen(spiDevId, 0)) == NULL){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_SW_CS_PIN;
	if(ytIoctl(spirit1Data->spi_id, spi_ioctl_cmd, &(spirit1Data->cs_pin)) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_POLARITY;
	if(ytIoctl(spirit1Data->spi_id, spi_ioctl_cmd, &spiPol) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_EDGE;
	if(ytIoctl(spirit1Data->spi_id, spi_ioctl_cmd, &spiEdge) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_MODE;
	if(ytIoctl(spirit1Data->spi_id, spi_ioctl_cmd, &spiMode) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_SPEED;
	if(ytIoctl(spirit1Data->spi_id, spi_ioctl_cmd, &spiSpeed) != RET_OK){
		return RET_ERROR;
	}
	return RET_OK;

}

