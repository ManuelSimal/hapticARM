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
 * cc2500_arch.h
 *
 *  Created on: 3 de may. de 2018
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cc2500_arch.h
 */
#ifndef YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_ARCH_H_
#define YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_ARCH_H_


#include <cc2500Core.h>
#include "cc2500_const.h"

#define CC2500_SPI_SPEED		3000000

retval_t cc2500_arch_init(cc2500Data_t* cc2500Data, uint32_t spiDevId);
retval_t cc2500_arch_deInit(cc2500Data_t* cc2500Data);

retval_t cc2500_arch_read_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* read_val);
retval_t cc2500_arch_write_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t write_val);

retval_t cc2500_arch_read_multi_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* read_val, uint16_t size);
retval_t cc2500_arch_write_multi_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* write_val, uint16_t size);

retval_t cc2500_arch_write_txfifo(cc2500Data_t* cc2500Data, uint8_t* tx_data, uint16_t size);
retval_t cc2500_arch_read_rxfifo(cc2500Data_t* cc2500Data, uint8_t* rx_data, uint16_t size);

cc2500_status_t cc2500_arch_get_status(cc2500Data_t* cc2500Data);
retval_t cc2500_arch_send_cmd_strobe(cc2500Data_t* cc2500Data, cc2500_cmd_strobe_t cmd_strobe);
retval_t cc2500_arch_read_status_reg(cc2500Data_t* cc2500Data, uint8_t reg_addr, uint8_t* read_val);

retval_t cc2500_arch_disable_interrupt(cc2500Data_t* cc2500Data);
retval_t cc2500_arch_enable_interrupt(cc2500Data_t* cc2500Data);

retval_t cc2500_arch_power_on(cc2500Data_t* cc2500Data);
retval_t cc2500_arch_power_off(cc2500Data_t* cc2500Data);

#endif /* YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_ARCH_H_ */
