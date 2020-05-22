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
 * timer.c
 *
 *  Created on: 14 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file timer.c
 */

#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "stm32l4xx_hal.h"
#include "timer.h"


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8) {
    HAL_IncTick();
  }
}

#if PLATFORM_USE_TIME_MEAS

static uint32_t lastReferenceCpuClock;

retval_t initTimeMeasureTimer(void){

	  /* Enable TIM5 clock */
	  __HAL_RCC_TIM5_CLK_ENABLE();

	/* Initialize TIM5 */
	  htim5.Instance = TIM5;

	  /*
	   * Configure the timer
	  */
	  htim5.Init.Period = 0xFFFFFFFF;	/*MAX count available in 32 bits timer*/
	  if(configCPU_CLOCK_HZ < PLATFORM_TIME_MEASURE_RESOLUTION){
		  htim5.Init.Prescaler = 0;
	  }
	  else{
		  htim5.Init.Prescaler = (configCPU_CLOCK_HZ/PLATFORM_TIME_MEASURE_RESOLUTION) - 1;
	  }
	  htim5.Init.ClockDivision = 0;
	  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	  if(HAL_TIM_Base_Init(&htim5) == HAL_OK)
	  {
	    /* Start the TIM time Base generation*/
	    HAL_TIM_Base_Start(&htim5);
	  }
	  lastReferenceCpuClock = configCPU_CLOCK_HZ;
	  return RET_OK;
}

void timeMeasureResetCount(void){
	if(lastReferenceCpuClock != configCPU_CLOCK_HZ){	/*If the system clock has changed update the timer prescaler*/
		  if(configCPU_CLOCK_HZ < PLATFORM_TIME_MEASURE_RESOLUTION){
			  htim5.Init.Prescaler = 0;
		  }
		  else{
			  htim5.Init.Prescaler = (configCPU_CLOCK_HZ/PLATFORM_TIME_MEASURE_RESOLUTION) - 1;
		  }
		  HAL_TIM_Base_Stop(&htim5);
		  if(HAL_TIM_Base_Init(&htim5) == HAL_OK)
		  {
		    /* Start the TIM time Base generation*/
		    HAL_TIM_Base_Start(&htim5);
		  }
		  lastReferenceCpuClock = configCPU_CLOCK_HZ;
	}
	else{
		__HAL_TIM_SET_COUNTER(&htim5,0);
	}
}
#endif

#endif
