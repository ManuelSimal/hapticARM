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
 * platformLowPower.c
 *
 *  Created on: 17 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file platformLowPower.c
 */


#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "stm32l4xx_hal.h"
#include "lowPower.h"

platformLowPowerMode_t platformCurrentLowPowerMode = PLATFORM_DEFAULT_INIT_LOW_POWER_MODE;

static platformRunMode_t platformCurrentRunMode = PLATFORM_DEFAULT_INIT_RUN_MODE;



void platformInitLowPowerManager(){
	/*Always set the MSI as the running clock when returning from sleep states*/
	HAL_RCCEx_WakeUpStopCLKConfig(RCC_STOP_WAKEUPCLOCK_MSI);
#if !PLATFORM_ENABLE_STOP_MODE_DEBUG
  HAL_DBGMCU_DisableDBGStopMode();
#endif
#if CONFIG_USE_TRACEALYZER_SW
	platformCurrentLowPowerMode = SLEEP_MODE;
	platformCurrentRunMode= RUN_MODE_48MHZ;
	peripheralFreqChangeLock();						/*CPU Frequency Changes not allowed while using the USB Driver. It must run at 48 MHz*/
	setDeviceLowPowerMode((uint32_t) SLEEP_MODE);	/*SLEEP MODE is the only low power mode allowed*/
#else
	platformCurrentLowPowerMode = PLATFORM_DEFAULT_INIT_LOW_POWER_MODE;
	platformCurrentRunMode= PLATFORM_DEFAULT_RUN_MODE;
#endif

}


/**
 * @brief	Updates the current consumption of the selected low pwoer mode and returns it
 * @return
 */
uint32_t platformUpdateLowPowerModeConsumption(){
	switch (platformCurrentLowPowerMode){
	case SLEEP_MODE:
		return DEFINE_SLEEP_MODE_BASE_CONSUMPTION +
				(DEFINE_SLEEP_MODE_CONSUMPTION_100_KHZ*(configCPU_CLOCK_HZ/100000));
		break;
	case STOP_1_MODE:
		return DEFINE_STOP_1_MODE_CONSUMPTION;
		break;
	case STOP_2_MODE:
		return DEFINE_STOP_2_MODE_CONSUMPTION;
		break;
	default:
		return 0;
		break;
	}
}

/**
 *
 * @param newLowPowerMode
 */
retval_t platformSetCurrentLowPowerMode(uint32_t newLowPowerMode){
	if(newLowPowerMode < PLATFORM_NUM_LOW_POWER_MODES){
		platformCurrentLowPowerMode = (platformLowPowerMode_t)newLowPowerMode;
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 *
 * @return
 */
uint32_t platformGetCurrentLowPowerMode(void){
	return (uint32_t) platformCurrentLowPowerMode;
}

/**
 *
 */
void platformEnterCurrentLowPowerMode(){
	switch(platformCurrentLowPowerMode){
	case SLEEP_MODE:
		__asm volatile( "dsb" ::: "memory" );
		__asm volatile( "wfi" );
		__asm volatile( "isb" );
		break;
	case STOP_1_MODE:
		HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
		break;
	case STOP_2_MODE:
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		break;
	default:
		break;
	}
}

/**
 *
 * @return
 */
uint32_t platformUpdateRunModeConsumption(){
	switch (platformCurrentRunMode){
	case RUN_MODE_1MHZ:
		return RUN_MODE_4_CONSUMPTION;
		break;
	case RUN_MODE_2MHZ:
		return RUN_MODE_5_CONSUMPTION;
		break;
	case RUN_MODE_4MHZ:
		return RUN_MODE_6_CONSUMPTION;
		break;
	case RUN_MODE_8MHZ:
		return RUN_MODE_7_CONSUMPTION;
		break;
	case RUN_MODE_16MHZ:
		return RUN_MODE_8_CONSUMPTION;
		break;
	case RUN_MODE_24MHZ:
		return RUN_MODE_9_CONSUMPTION;
		break;
	case RUN_MODE_32MHZ:
		return RUN_MODE_10_CONSUMPTION;
		break;
	case RUN_MODE_48MHZ:
		return RUN_MODE_11_CONSUMPTION;
		break;
	default:
		return 0;
		break;
	}
}

/**
 *
 * @param newRunMode
 * @return
 */
retval_t platformSetCurrentRunMode(uint32_t newRunMode){
	if(newRunMode < PLATFORM_NUM_RUN_MODES){

		/*Run the port function to change the clock frequency and reconfigure the systick timer*/
		vPortSetClockFrequency(newRunMode);
		platformCurrentRunMode = (platformLowPowerMode_t)newRunMode;
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 *
 * @return
 */
uint32_t platformGetCurrentRunMode(void){
	return (uint32_t) platformCurrentRunMode;
}

/**
 *
 * @param runMode
 * @return
 */
void platformSetClockFrequency(uint32_t runMode){
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* If the current frequency is lower to 2 MHz, and the next frequency higher to it, it is necessary to
	 * disable de low power regulator*/
	if(((uint32_t)platformCurrentRunMode >= (uint32_t)RUN_MODE_2MHZ) && (runMode < (uint32_t)RUN_MODE_2MHZ) ){
//		HAL_PWREx_DisableLowPowerRunMode();
	}

	/*Set new MSI clock*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;

	switch ((platformRunMode_t)runMode){
	case RUN_MODE_1MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
		break;
	case RUN_MODE_2MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
		break;
	case RUN_MODE_4MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
		break;
	case RUN_MODE_8MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
		break;
	case RUN_MODE_16MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
		break;
	case RUN_MODE_24MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
		break;
	case RUN_MODE_32MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
		break;
	case RUN_MODE_48MHZ:
		RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
		break;
	default:
		return;
		break;
	}

	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* If the next frequency is lower to 2 MHz, the low power regulator can be enabled*/
	if((uint32_t)runMode >= (uint32_t)RUN_MODE_2MHZ){
//		HAL_PWREx_EnableLowPowerRunMode();
	}

}
#endif
