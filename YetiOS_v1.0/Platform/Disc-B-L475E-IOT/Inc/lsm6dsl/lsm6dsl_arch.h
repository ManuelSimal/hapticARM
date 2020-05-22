/*
 * Copyright (c) 2020, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * lsm6dsl_arch.h
 *
 *  Created on: Feb 24, 2020
 *      Author: Santiago Isidro Real <sreal@b105.upm.es>
 */

#ifndef BSP_COMPONENTS_LSM6DSL_INCLUDE_LSM6DSL_ARCH_H_
#define BSP_COMPONENTS_LSM6DSL_INCLUDE_LSM6DSL_ARCH_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"


int32_t LSM6DSL_arch_Init(void);
int32_t LSM6DSL_arch_DeInit(void);
int32_t LSM6DSL_arch_GetTick(void);
int32_t LSM6DSL_arch_WriteReg(uint16_t address, uint16_t regAddress, uint8_t *buf, uint16_t length);
int32_t LSM6DSL_arch_ReadReg(uint16_t address, uint16_t regAddress, uint8_t *buf, uint16_t length);

void LSM6DSL_arch_setSPIHandle(I2C_HandleTypeDef* e_hi2c);
//LSM6DSL_IO_t getLSM6DSL_default_IO(void);



#endif /* BSP_COMPONENTS_LSM6DSL_INCLUDE_LSM6DSL_ARCH_H_ */
