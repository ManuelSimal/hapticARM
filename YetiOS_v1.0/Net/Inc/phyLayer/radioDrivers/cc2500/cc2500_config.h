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
 * cc2500_config.h
 *
 *  Created on: 4 de may. de 2018
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cc2500_config.h
 */
#ifndef YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CONFIG_H_
#define YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CONFIG_H_

#include <cc2500Core.h>

retval_t cc2500_config_gdo2_pin(cc2500Data_t* cc2500Data, gdo_config_t gdo_config);
retval_t cc2500_set_packet_length(cc2500Data_t* cc2500Data, uint16_t pckt_len);
retval_t cc2500_enable_fixed_packet_length(cc2500Data_t* cc2500Data);
retval_t cc2500_disable_fixed_packet_length(cc2500Data_t* cc2500Data);
retval_t cc2500_set_channel_num(cc2500Data_t* cc2500Data, uint8_t channel);
retval_t cc2500_set_base_freq(cc2500Data_t* cc2500Data, uint32_t freq);	//Freq in MHz
retval_t cc2500_set_rx_bw(cc2500Data_t* cc2500Data, uint32_t rx_bw);	//Rx Bandwidth in KHz
retval_t cc2500_set_datarate(cc2500Data_t* cc2500Data, uint32_t datarate);
retval_t cc2500_set_modulation_format(cc2500Data_t* cc2500Data, cc2500_modulation_t modulation);
retval_t cc2500_set_channel_spacing(cc2500Data_t* cc2500Data, uint32_t channel_spacing);
retval_t cc2500_set_freq_dev(cc2500Data_t* cc2500Data, uint32_t freq_dev);
retval_t cc2500_set_output_pwr(cc2500Data_t* cc2500Data, int32_t out_pwr);

#endif /* YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CONFIG_H_ */
