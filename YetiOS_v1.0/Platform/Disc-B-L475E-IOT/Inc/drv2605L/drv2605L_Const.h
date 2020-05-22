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
 * drv2605L_Const.h
 *
 *  Created on: Mar 22, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */

#ifndef YETIOS_PLATFORM_DISC_B_L475E_IOT_INC_LSM6DSL_LSM6DSL_CONST_H_
#define YETIOS_PLATFORM_DISC_B_L475E_IOT_INC_LSM6DSL_LSM6DSL_CONST_H_

#define DRV2605L_I2C_ADDRESS  (0x5A << 1)


#define DRV2605L_ACTUATOR1			drv2605L_actuator_LRA0934W
#define DRV2605L_MODE_SELECTION1	drv2605L_mode_internalTrigger
#define DRV2605L_LIBRARY1			drv2605L_lib_ROM_LRA

#define DRV2605L_ACTUATOR2			drv2605L_actuator_LRA0934W
#define DRV2605L_MODE_SELECTION2	drv2605L_mode_internalTrigger
#define DRV2605L_LIBRARY2			drv2605L_lib_ROM_LRA

#define DRV2605L_ACTUATOR3			drv2605L_actuator_LRA0934W
#define DRV2605L_MODE_SELECTION3	drv2605L_mode_internalTrigger
#define DRV2605L_LIBRARY3			drv2605L_lib_ROM_LRA


#endif /* YETIOS_PLATFORM_DISC_B_L475E_IOT_INC_DRV205L_DRV2605L_CONST_H_ */
