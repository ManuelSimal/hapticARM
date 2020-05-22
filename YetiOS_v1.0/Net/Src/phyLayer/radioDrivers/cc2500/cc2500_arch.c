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
 * cc2500_arch.c
 *
 *  Created on: 3 de may. de 2018
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cc2500_arch.c
 */

#include <cc2500Core.h>
#include "yetiOS.h"
#include "cc2500_const.h"
#include "cc2500_arch.h"


#define MAX_FIFO_SIZE			64

#define WRITE_REG_MASK			0x7F
#define READ_REG_MASK			0x80
#define BURST_BIT_MASK			0x40


static retval_t config_spi_device(cc2500Data_t* cc2500Data, uint32_t spiDevId);
/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_arch_init(cc2500Data_t* cc2500Data, uint32_t spiDevId){

	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	//First open and configure the spi to be used
	if (config_spi_device(cc2500Data, spiDevId) != RET_OK){
		return RET_ERROR;
	}

	//Configure GDO2 INTERRUPT pin
	if(ytGpioInitPin(cc2500Data->gdo2_int_pin, GPIO_PIN_INTERRUPT_FALLING_RISING, GPIO_PIN_NO_PULL) != RET_OK){
		return RET_ERROR;
	}

	if(ytGpioPinSetCallback(cc2500Data->gdo2_int_pin, NULL, NULL) != RET_OK){
		return RET_ERROR;
	}

	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_arch_deInit(cc2500Data_t* cc2500Data){

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	//Deinit Interrupt and SDN PIN
	if(ytGpioDeInitPin(cc2500Data->gdo2_int_pin)!= RET_OK){
		return RET_ERROR;
	}

	//Close Spi Device
	ytClose(cc2500Data->spi_id);
	cc2500Data->spi_id = 0;

	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @param reg_addr
 * @param read_val
 * @return
 */
retval_t cc2500_arch_read_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* read_val){

	uint8_t tx_val[2];
	uint8_t rx_val[2];
	ytSpiReadWriteBuff_t spi_read_write;

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	tx_val[0] = (READ_REG_MASK | reg_addr) & ~BURST_BIT_MASK;
	tx_val[1] = 0;

	spi_read_write.prx = &rx_val[0];
	spi_read_write.ptx = &tx_val[0];
	spi_read_write.size = 2;


	if(ytIoctl(cc2500Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		return RET_ERROR;
	}

	*read_val = rx_val[1];

	return RET_OK;

}


/**
 *
 * @param cc2500Data
 * @param reg_addr
 * @param write_val
 * @return
 */
retval_t cc2500_arch_write_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t write_val){

	uint8_t tx_val[2];

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	tx_val[0] = (WRITE_REG_MASK & reg_addr) & ~BURST_BIT_MASK;
	tx_val[1] = write_val;

	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val[0], 2) != 2){
		return RET_ERROR;
	}

	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @param reg_addr
 * @param read_val
 * @param size
 * @return
 */
retval_t cc2500_arch_read_multi_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* read_val, uint16_t size){

	ytSpiReadWriteBuff_t spi_read_write;
	uint16_t i;
	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	spi_read_write.ptx = (uint8_t*) pvPortMalloc(1+size);
	spi_read_write.prx = (uint8_t*) pvPortMalloc(1+size);

	spi_read_write.ptx[0] = (READ_REG_MASK | reg_addr) | BURST_BIT_MASK;

	spi_read_write.size = 1+size;


	if(ytIoctl(cc2500Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		vPortFree(spi_read_write.ptx);
		vPortFree(spi_read_write.prx);
		return RET_ERROR;
	}

	for(i=0; i<size; i++){
		read_val[i] = spi_read_write.prx[i+1];
	}


	vPortFree(spi_read_write.ptx);
	vPortFree(spi_read_write.prx);

	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @param reg_addr
 * @param write_val
 * @param size
 * @return
 */
retval_t cc2500_arch_write_multi_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* write_val, uint16_t size){

	uint8_t* tx_val;
	uint16_t i;

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	tx_val = (uint8_t*)pvPortMalloc(1+size);

	tx_val[0] = (WRITE_REG_MASK & reg_addr) | BURST_BIT_MASK;
	for(i=0; i<size; i++){
		tx_val[1+i] = write_val[i];
	}

	if(ytWrite(cc2500Data->spi_id, (void*) tx_val, 1+size) != 1+size){
		vPortFree(tx_val);
		return RET_ERROR;
	}
	vPortFree(tx_val);
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @param tx_data
 * @param size
 * @return
 */
retval_t cc2500_arch_write_txfifo(cc2500Data_t* cc2500Data, uint8_t* tx_data, uint16_t size){

	uint8_t* tx_val;
	uint16_t i;

	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	if(size > MAX_FIFO_SIZE){
		return RET_ERROR;
	}

	tx_val = (uint8_t*)pvPortMalloc(1+size);

	tx_val[0] = (CC2500_FIFO_ADDR & WRITE_REG_MASK) | BURST_BIT_MASK;
	for(i=0; i<size; i++){
		tx_val[1+i] = tx_data[i];
	}

	if(ytWrite(cc2500Data->spi_id, (void*) tx_val, 1+size) != 1+size){
		vPortFree(tx_val);
		return RET_ERROR;
	}
	vPortFree(tx_val);
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @param rx_data
 * @param size
 * @return
 */
retval_t cc2500_arch_read_rxfifo(cc2500Data_t* cc2500Data, uint8_t* rx_data, uint16_t size){

	ytSpiReadWriteBuff_t spi_read_write;
	uint16_t i;
	if(cc2500Data == NULL){
		return RET_ERROR;
	}
	if(size > MAX_FIFO_SIZE){
		return RET_ERROR;
	}

	spi_read_write.ptx = (uint8_t*) pvPortMalloc(1+size);
	spi_read_write.prx = (uint8_t*) pvPortMalloc(1+size);

	spi_read_write.ptx[0] = (READ_REG_MASK | CC2500_FIFO_ADDR) | BURST_BIT_MASK;

	spi_read_write.size = 1+size;

	if(ytIoctl(cc2500Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		vPortFree(spi_read_write.ptx);
		vPortFree(spi_read_write.prx);
		return RET_ERROR;
	}

	for(i=0; i<size; i++){
		rx_data[i] = spi_read_write.prx[i+1];
	}


	vPortFree(spi_read_write.ptx);
	vPortFree(spi_read_write.prx);

	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @return
 */
cc2500_status_t cc2500_arch_get_status(cc2500Data_t* cc2500Data){
	cc2500_status_t* ret_status;

	uint8_t tx_val;
	uint8_t rx_val;
	ytSpiReadWriteBuff_t spi_read_write;
	ret_status =  (cc2500_status_t*) &rx_val;

	tx_val = CC2500_STROBE_SNOP;
	rx_val = 0;

	if(cc2500Data == NULL){
		return *ret_status;
	}


	spi_read_write.prx = &rx_val;
	spi_read_write.ptx = &tx_val;
	spi_read_write.size = 1;


	if(ytIoctl(cc2500Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		return *ret_status;
	}

	return *ret_status;

}

/**
 *
 * @param cc2500Data
 * @param cmd_strobe
 * @return
 */
retval_t cc2500_arch_send_cmd_strobe(cc2500Data_t* cc2500Data, cc2500_cmd_strobe_t cmd_strobe){

	uint8_t tx_val;

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	tx_val = cmd_strobe;

	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val, 1) != 1){
		return RET_ERROR;
	}

	return RET_OK;

}


retval_t cc2500_arch_read_status_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* read_val){

	uint8_t tx_val[2];
	uint8_t rx_val[2];
	ytSpiReadWriteBuff_t spi_read_write;

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	tx_val[0] = (READ_REG_MASK | reg_addr) | BURST_BIT_MASK;
	tx_val[1] = 0;

	spi_read_write.prx = &rx_val[0];
	spi_read_write.ptx = &tx_val[0];
	spi_read_write.size = 2;


	if(ytIoctl(cc2500Data->spi_id, SPI_READ_WRITE, (void*) &spi_read_write) != RET_OK){
		return RET_ERROR;
	}

	*read_val = rx_val[1];

	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_arch_disable_interrupt(cc2500Data_t* cc2500Data){

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	if(ytGpioPinSetCallback(cc2500Data->gdo2_int_pin, NULL, NULL) != RET_OK){
		return RET_ERROR;
	}
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_arch_enable_interrupt(cc2500Data_t* cc2500Data){

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	if(ytGpioPinSetCallback(cc2500Data->gdo2_int_pin, cc2500Data->interrupt_cb_func, (void*) cc2500Data->interrupt_cb_arg) != RET_OK){
		return RET_ERROR;
	}
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_arch_power_off(cc2500Data_t* cc2500Data){

	uint8_t tx_val;

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	tx_val = CC2500_STROBE_SFRX;
	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val, 1) != 1){
		return RET_ERROR;
	}

	tx_val = CC2500_STROBE_SFTX;
	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val, 1) != 1){
		return RET_ERROR;
	}

	tx_val = CC2500_STROBE_SIDLE;
	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val, 1) != 1){
		return RET_ERROR;
	}

	tx_val = CC2500_STROBE_SPWD;
	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val, 1) != 1){
		return RET_ERROR;
	}
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @return
 */
retval_t cc2500_arch_power_on(cc2500Data_t* cc2500Data){

	uint8_t tx_val;

	if(cc2500Data == NULL){
		return RET_ERROR;
	}

	tx_val = CC2500_STROBE_SNOP;									//Only one NOP strobe should be necessary to wake up the CC2500
	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val, 1) != 1){
		return RET_ERROR;
	}

	tx_val = CC2500_STROBE_SNOP;
	if(ytWrite(cc2500Data->spi_id, (void*) &tx_val, 1) != 1){
		return RET_ERROR;
	}
	return RET_OK;

}

/**
 *
 * @param cc2500Data
 * @return
 */
static retval_t config_spi_device(cc2500Data_t* cc2500Data, uint32_t spiDevId){

	ytIoctlCmd_t spi_ioctl_cmd;
	ytSpiPolarity_t spiPol = SPI_POL_LOW;
	ytSpiEdge_t spiEdge = SPI_EDGE_FIRST;
	ytSpiMode_t spiMode = SPI_MASTER;
	uint32_t spiSpeed = CC2500_SPI_SPEED;
	if((cc2500Data->spi_id = ytOpen(spiDevId, 0)) == NULL){
		return RET_ERROR;
	}

	spi_ioctl_cmd = SPI_SET_SW_CS_PIN;
	if(ytIoctl(cc2500Data->spi_id, spi_ioctl_cmd, &(cc2500Data->cs_pin)) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_POLARITY;
	if(ytIoctl(cc2500Data->spi_id, spi_ioctl_cmd, &spiPol) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_EDGE;
	if(ytIoctl(cc2500Data->spi_id, spi_ioctl_cmd, &spiEdge) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_MODE;
	if(ytIoctl(cc2500Data->spi_id, spi_ioctl_cmd, &spiMode) != RET_OK){
		return RET_ERROR;
	}
	spi_ioctl_cmd = SPI_SET_SPEED;
	if(ytIoctl(cc2500Data->spi_id, spi_ioctl_cmd, &spiSpeed) != RET_OK){
		return RET_ERROR;
	}

	return RET_OK;
}

