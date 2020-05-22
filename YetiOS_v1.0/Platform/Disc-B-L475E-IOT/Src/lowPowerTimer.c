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
 * lowPowerTimer.c
 *
 *  Created on: 18 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file lowPowerTimer.c
 */
#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "stm32l4xx_hal.h"
#include "lowPowerTimer.h"

static LPTIM_HandleTypeDef hlptim1;
static uint16_t lowPowerTimerInterruptedFlag;

/**
 *
 * @return
 */
retval_t platformLowPowerTimerInit(){
	  hlptim1.Instance = LPTIM1;
	  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
	  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
	  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
	  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
	  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
	  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
	  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
	  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
	  {
		  return RET_ERROR;
	  }
	  return RET_OK;
}
/**
 *
 * @return
 */
retval_t platformLowPowerTimerDeInit(){
	if(HAL_LPTIM_DeInit(&hlptim1) != HAL_OK){
		return RET_ERROR;
	}
	return RET_OK;
}

/**
 *
 * @param lowPowerTimerCount
 */
void platformStartLowPowerTimer(uint32_t lowPowerTimerCount){
	lowPowerTimerInterruptedFlag = 0;
	__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1, platformLOW_POWER_TIMER_MAX_COUNT, lowPowerTimerCount);
}

/**
 *
 * @return
 */
void platformStopLowPowerTimer(){
	HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
	__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);
}

/**
 *
 * @return
 */
uint32_t platformGetLowPowerTimerCount(){
	uint32_t count = HAL_LPTIM_ReadCounter(&hlptim1);
	lowPowerTimerInterruptedFlag = (uint16_t) __HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_CMPM);
	__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);
	return count;
}

/**
 *
 * @return
 */
uint16_t platformLowPowerTimerInterrupted(){
	return lowPowerTimerInterruptedFlag;
}
/*Callback functions called by the HAL on init and deinit*/
/**
 *
 * @param lptimHandle
 */
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* lptimHandle)
{

  if(lptimHandle->Instance==LPTIM1)
  {
    /* LPTIM1 clock enable */
    __HAL_RCC_LPTIM1_CLK_ENABLE();

    /* LPTIM1 interrupt Init */
    HAL_NVIC_SetPriority(LPTIM1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
  }
}


/**
 *
 * @param lptimHandle
 */
void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef* lptimHandle)
{

  if(lptimHandle->Instance==LPTIM1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM1_CLK_DISABLE();

    /* LPTIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
  }
}


/**
* @brief This function handles LPTIM1 global interrupt.
*/
void LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM1_IRQn 0 */

  /* USER CODE END LPTIM1_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim1);
  /* USER CODE BEGIN LPTIM1_IRQn 1 */

  /* USER CODE END LPTIM1_IRQn 1 */
}

#endif
