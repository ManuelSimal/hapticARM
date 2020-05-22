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
 * platformLeds.c
 *
 *  Created on: 24 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file platformLeds.c
 */

#include "yetiOS.h"
#ifdef USE_HEIMDALL_L4
#include "stm32l4xx_hal.h"
#endif

#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM
/* LED PINS */
#define LED_PORT_B		GPIOB
#define LED_BLUE_PIN	GPIO_PIN_5
#define LED_GREEN_PIN	GPIO_PIN_4
#define LED_PORT_A		GPIOA
#define LED_RED1_PIN	GPIO_PIN_8
#define LED_RED2_PIN	GPIO_PIN_15


/**
 *
 * @return
 */
retval_t platformLedsInit(){

	GPIO_InitTypeDef GPIO_InitStruct;

	__GPIOB_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	/*Configure GPIO pins*/
	GPIO_InitStruct.Pin = LED_BLUE_PIN|LED_GREEN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(LED_PORT_B, &GPIO_InitStruct);

	/*Configure GPIO pins*/
	GPIO_InitStruct.Pin = LED_RED1_PIN|LED_RED2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(LED_PORT_A, &GPIO_InitStruct);

	return RET_OK;
}

/**
 *
 * @param led
 */
retval_t platformLedOn(ytLed_t led){
	switch(led){
	case LED_RED_1:
		HAL_GPIO_WritePin(LED_PORT_A, LED_RED1_PIN, GPIO_PIN_SET);
		break;
	case LED_RED_2:
		HAL_GPIO_WritePin(LED_PORT_A, LED_RED2_PIN, GPIO_PIN_SET);
		break;
	case LED_BLUE_1:
		HAL_GPIO_WritePin(LED_PORT_B, LED_BLUE_PIN, GPIO_PIN_SET);
		break;
	case LED_GREEN_1:
		HAL_GPIO_WritePin(LED_PORT_B, LED_GREEN_PIN, GPIO_PIN_SET);
		break;
	default:
		return RET_ERROR;
		break;
	}
	return RET_OK;
}

/**
 *
 * @param led
 */
retval_t platformLedOff(ytLed_t led){
	switch(led){
	case LED_RED_1:
		HAL_GPIO_WritePin(LED_PORT_A, LED_RED1_PIN, GPIO_PIN_RESET);
		break;
	case LED_RED_2:
		HAL_GPIO_WritePin(LED_PORT_A, LED_RED2_PIN, GPIO_PIN_RESET);
		break;
	case LED_BLUE_1:
		HAL_GPIO_WritePin(LED_PORT_B, LED_BLUE_PIN, GPIO_PIN_RESET);
		break;
	case LED_GREEN_1:
		HAL_GPIO_WritePin(LED_PORT_B, LED_GREEN_PIN, GPIO_PIN_RESET);
		break;
	default:
		return RET_ERROR;
		break;
	}
	return RET_OK;
}


/**
 *
 * @param led
 */
retval_t platformLedToggle(ytLed_t led){
	switch(led){
	case LED_RED_1:
		HAL_GPIO_TogglePin(LED_PORT_A, LED_RED1_PIN);
		break;
	case LED_RED_2:
		HAL_GPIO_TogglePin(LED_PORT_A, LED_RED2_PIN);
		break;
	case LED_BLUE_1:
		HAL_GPIO_TogglePin(LED_PORT_B, LED_BLUE_PIN);
		break;
	case LED_GREEN_1:
		HAL_GPIO_TogglePin(LED_PORT_B, LED_GREEN_PIN);
		break;
	default:
		return RET_ERROR;
		break;
	}
	return RET_OK;
}
#endif
