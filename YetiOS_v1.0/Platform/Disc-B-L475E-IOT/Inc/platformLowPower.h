/*
 * Copyright (c) 2019, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * platformLowPower.h
 *
 *  Created on: 17 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file platformLowPower.h
 */

#ifndef YETIOS_PLATFORM_HEIMDALL_INC_PLATFORMLOWPOWER_H_
#define YETIOS_PLATFORM_HEIMDALL_INC_PLATFORMLOWPOWER_H_



#include "platformConf.h"
#ifdef USE_HEIMDALL_L4
#include "stm32l4xx_hal.h"
#endif

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "lowPowerTimer.h"

#define PLATFORM_NUM_LOW_POWER_MODES	3		//We define three low power modes: SLEEP, STOP_1, and STOP_2

#define DEFINE_STOP_1_MODE_CONSUMPTION	8	/*8 uA consumption (from specs)*/
#define DEFINE_STOP_2_MODE_CONSUMPTION	2	/*2 uA consumption (from specs)*/

#define DEFINE_SLEEP_MODE_BASE_CONSUMPTION		33	/*33 uA minimum consumption running at lowest frequency (100KHz)*/
#define DEFINE_SLEEP_MODE_CONSUMPTION_100_KHZ	3	/*3 uA consumption increase each 100 KHz*/

#define PLATFORM_NUM_RUN_MODES	8 	/*8 Run modes, corresponding to the MSI RANGES*/

//#define RUN_MODE_0_CONSUMPTION         42  /* MSI = 100 KHz  */
//#define RUN_MODE_1_CONSUMPTION         54  /*!< MSI = 200 KHz  */
//#define RUN_MODE_2_CONSUMPTION         78  /*!< MSI = 400 KHz  */
//#define RUN_MODE_3_CONSUMPTION         126  /*!< MSI = 800 KHz  */
//#define RUN_MODE_4_CONSUMPTION         154  /*!< MSI = 1 MHz    */
//#define RUN_MODE_5_CONSUMPTION         272  /*!< MSI = 2 MHz    */
//#define RUN_MODE_6_CONSUMPTION         550  /*!< MSI = 4 MHz    */
//#define RUN_MODE_7_CONSUMPTION         980  /*!< MSI = 8 MHz    */
//#define RUN_MODE_8_CONSUMPTION         1830  /*!< MSI = 16 MHz   */
//#define RUN_MODE_9_CONSUMPTION         2880  /*!< MSI = 24 MHz   */
//#define RUN_MODE_10_CONSUMPTION        4240 /*!< MSI = 32 MHz   */
//#define RUN_MODE_11_CONSUMPTION        6280 /*!< MSI = 48 MHz   */

//#define RUN_MODE_0_CONSUMPTION         42  /* MSI = 100 KHz  */
//#define RUN_MODE_1_CONSUMPTION         54  /*!< MSI = 200 KHz  */
//#define RUN_MODE_2_CONSUMPTION         78  /*!< MSI = 400 KHz  */
//#define RUN_MODE_3_CONSUMPTION         100  /*!< MSI = 800 KHz  */
#define RUN_MODE_4_CONSUMPTION         120  /*!< MSI = 1 MHz    */
#define RUN_MODE_5_CONSUMPTION         230  /*!< MSI = 2 MHz    */
#define RUN_MODE_6_CONSUMPTION         660  /*!< MSI = 4 MHz    */
#define RUN_MODE_7_CONSUMPTION         1100  /*!< MSI = 8 MHz    */
#define RUN_MODE_8_CONSUMPTION         2100  /*!< MSI = 16 MHz   */
#define RUN_MODE_9_CONSUMPTION         3220  /*!< MSI = 24 MHz   */
#define RUN_MODE_10_CONSUMPTION        3960 /*!< MSI = 32 MHz   */
#define RUN_MODE_11_CONSUMPTION        5950 /*!< MSI = 48 MHz   */


typedef enum platformLowPowerMode_{
	SLEEP_MODE = 0,
	STOP_1_MODE = 1,
	STOP_2_MODE = 2,
}platformLowPowerMode_t;


/*It is important to take into account that at very low frequency (100 KHz) the overload of processing the systick each
 * millisecond may be too high, and the system may get blocked*/
typedef enum platformRunMode_{
	RUN_MODE_48MHZ = 0,
	RUN_MODE_32MHZ = 1,
	RUN_MODE_24MHZ = 2,
	RUN_MODE_16MHZ = 3,
	RUN_MODE_8MHZ = 4,
	RUN_MODE_4MHZ = 5,
	RUN_MODE_2MHZ = 6,
	RUN_MODE_1MHZ = 7,
}platformRunMode_t;

#define PLATFORM_DEFAULT_RUN_MODE		RUN_MODE_48MHZ

/*Used by Port.c to get the current low power mode. Use a variable instead a get function to reduce execution cycles*/
extern platformLowPowerMode_t platformCurrentLowPowerMode;

void platformInitLowPowerManager();

uint32_t platformUpdateLowPowerModeConsumption();

retval_t platformSetCurrentLowPowerMode(uint32_t newLowPowerMode);
uint32_t platformGetCurrentLowPowerMode(void);

void platformEnterCurrentLowPowerMode();


uint32_t platformUpdateRunModeConsumption();
retval_t platformSetCurrentRunMode(uint32_t newRunMode);
uint32_t platformGetCurrentRunMode(void);

void platformSetClockFrequency(uint32_t runMode);
#endif

#endif /* YETIOS_PLATFORM_HEIMDALL_INC_PLATFORMLOWPOWER_H_ */
