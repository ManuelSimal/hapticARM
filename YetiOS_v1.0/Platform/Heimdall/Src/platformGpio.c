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
 * platformGpio.c
 *
 *  Created on: 27 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file platformGpio.c
 */

#include "yetiOS.h"
#ifdef USE_HEIMDALL_L4
#include "stm32l4xx_hal.h"
#endif

#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM
#include "platformGpio.h"

#define 	NUM_EXTI_LINES			16


typedef struct platformGpioPin_{
	GPIO_TypeDef* port;
	uint16_t pin;
	uint16_t interruptLineNum;
	uint16_t extiIrq;
}platformGpioPin_t;

static const platformGpioPin_t platformGpioPinTable[AVAILABLE_GPIO_PINS] = {
		{GPIOA, GPIO_PIN_0, 0, EXTI0_IRQn},			/*Gpio Pin 0*/
		{GPIOA, GPIO_PIN_1, 1, EXTI1_IRQn},			/*Gpio Pin 1*/
		{GPIOA, GPIO_PIN_4, 4, EXTI4_IRQn},			/*Gpio Pin 2*/
		{GPIOB, GPIO_PIN_0, 0, EXTI0_IRQn},			/*Gpio Pin 3*/
		{GPIOB, GPIO_PIN_1, 1, EXTI1_IRQn},			/*Gpio Pin 4*/
		{GPIOB, GPIO_PIN_2, 2, EXTI2_IRQn},			/*Gpio Pin 5*/
		{GPIOB, GPIO_PIN_8, 8, EXTI9_5_IRQn},		/*Gpio Pin 6*/
		{GPIOB, GPIO_PIN_9, 9, EXTI9_5_IRQn},		/*Gpio Pin 7*/
		{GPIOB, GPIO_PIN_10, 10, EXTI15_10_IRQn},	/*Gpio Pin 8*/
		{GPIOB, GPIO_PIN_11, 11, EXTI15_10_IRQn},	/*Gpio Pin 9*/
		{GPIOB, GPIO_PIN_12, 12, EXTI15_10_IRQn},	/*Gpio Pin 10*/
		{GPIOC, GPIO_PIN_0, 0, EXTI0_IRQn},			/*Gpio Pin 11*/
		{GPIOC, GPIO_PIN_1, 1, EXTI1_IRQn},			/*Gpio Pin 12*/
		{GPIOC, GPIO_PIN_2, 2, EXTI2_IRQn},			/*Gpio Pin 13*/
		{GPIOC, GPIO_PIN_3, 3, EXTI3_IRQn},			/*Gpio Pin 14*/
		{GPIOC, GPIO_PIN_4, 4, EXTI4_IRQn},			/*Gpio Pin 15*/
		{GPIOC, GPIO_PIN_5, 5, EXTI9_5_IRQn},		/*Gpio Pin 16*/
		{GPIOC, GPIO_PIN_6, 6, EXTI9_5_IRQn},		/*Gpio Pin 17*/
		{GPIOC, GPIO_PIN_7, 7, EXTI9_5_IRQn},		/*Gpio Pin 18*/
		{GPIOC, GPIO_PIN_13, 13, EXTI15_10_IRQn},	/*Gpio Pin 19*/
};

static uint32_t interruptPinTable[NUM_EXTI_LINES];

/**
 *
 * @return
 */
retval_t platformGpioInit(){
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	memset(interruptPinTable, 0xFF, sizeof(uint32_t)*NUM_EXTI_LINES);
	return RET_OK;
}

/**�
 *
 * @return
 */
retval_t platformGpioDeInit(){
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	memset(interruptPinTable, 0xFF, sizeof(uint32_t)*NUM_EXTI_LINES);
	return RET_OK;
}

/**
 *
 * @param gpio
 * @return
 */
uint16_t checkAvailablePlatformPin(uint32_t gpio){
	if(gpio < AVAILABLE_GPIO_PINS){
		return 1;
	}
	return 0;
}

/**
 *
 * @param gpio
 * @param gpioMode
 * @param gpioPull
 * @return
 */
retval_t platformInitPin(uint32_t gpio, ytGpioMode_t gpioMode, ytGpioPull_t gpioPull){

	GPIO_InitTypeDef GPIO_InitStruct;

	if(gpio >= AVAILABLE_GPIO_PINS){
		return RET_ERROR;
	}

	GPIO_InitStruct.Pin = platformGpioPinTable[gpio].pin;

	/*Config pin mode and pull*/
	switch(gpioMode){
	case GPIO_PIN_INPUT:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] == gpio){
			interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = 0xFFFFFFFF;
		}
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT ;
		break;
	case GPIO_PIN_OUTPUT_PUSH_PULL:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] == gpio){
			interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = 0xFFFFFFFF;
		}
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		break;
	case GPIO_PIN_OUTPUT_OPEN_DRAIN:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] == gpio){
			interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = 0xFFFFFFFF;
		}
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		break;
	case GPIO_PIN_INTERRUPT_FALLING:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] != 0xFFFFFFFF){
			return RET_ERROR;
		}
		interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = gpio;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		break;
	case GPIO_PIN_INTERRUPT_RISING:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] != 0xFFFFFFFF){
			return RET_ERROR;
		}
		interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = gpio;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		break;
	case GPIO_PIN_INTERRUPT_FALLING_RISING:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] != 0xFFFFFFFF){
			return RET_ERROR;
		}
		interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = gpio;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		break;
	case GPIO_PIN_ANALOG:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] == gpio){
			interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = 0xFFFFFFFF;
		}
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		break;
	case GPIO_PIN_ANALOG_ADC:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] == gpio){
			interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = 0xFFFFFFFF;
		}
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
		break;
	default:
		return RET_ERROR;
		break;
	}

	switch(gpioPull){
	case GPIO_PIN_PULLUP:
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		break;
	case GPIO_PIN_PULLDOWN:
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		break;
	case GPIO_PIN_NO_PULL:
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		break;
	default:
		if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] == gpio){
			interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = 0xFFFFFFFF;
		}
		return RET_ERROR;
		break;
	}

	HAL_GPIO_Init(platformGpioPinTable[gpio].port, &GPIO_InitStruct);

	/*Enable the interrupt if needed*/
	if((gpioMode == GPIO_PIN_INTERRUPT_FALLING) || (gpioMode == GPIO_PIN_INTERRUPT_RISING)
			|| (gpioMode == GPIO_PIN_INTERRUPT_FALLING_RISING)){

		HAL_NVIC_SetPriority(platformGpioPinTable[gpio].extiIrq, 13, 0);
		HAL_NVIC_EnableIRQ(platformGpioPinTable[gpio].extiIrq);
	}


	return RET_OK;
}

/**
 *
 * @param gpio
 * @return
 */
retval_t platformDeInitPin(uint32_t gpio){
	if(gpio >= AVAILABLE_GPIO_PINS){
		return RET_ERROR;
	}
	/*If another pin is using the interrupt line, dont disable interrupts*/
	if(interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] == gpio){
		HAL_NVIC_DisableIRQ(platformGpioPinTable[gpio].extiIrq);
		interruptPinTable[platformGpioPinTable[gpio].interruptLineNum] = 0xFFFFFFFF;
	}

	HAL_GPIO_DeInit(platformGpioPinTable[gpio].port, platformGpioPinTable[gpio].pin);

	return RET_OK;
}

/**
 *
 * @param gpio
 * @return
 */
retval_t platformPinSet(uint32_t gpio){
	if(gpio >= AVAILABLE_GPIO_PINS){
		return RET_ERROR;
	}
	HAL_GPIO_WritePin(platformGpioPinTable[gpio].port, platformGpioPinTable[gpio].pin, GPIO_PIN_SET);
	return RET_ERROR;
}

/**
 *
 * @param gpio
 * @return
 */
retval_t platformPinReset(uint32_t gpio){
	if(gpio >= AVAILABLE_GPIO_PINS){
		return RET_ERROR;
	}
	HAL_GPIO_WritePin(platformGpioPinTable[gpio].port, platformGpioPinTable[gpio].pin, GPIO_PIN_RESET);
	return RET_ERROR;
}

/**
 *
 * @param gpio
 * @return
 */
retval_t platformPinToggle(uint32_t gpio){
	if(gpio >= AVAILABLE_GPIO_PINS){
		return RET_ERROR;
	}
	HAL_GPIO_TogglePin(platformGpioPinTable[gpio].port, platformGpioPinTable[gpio].pin);
	return RET_ERROR;
}

/**
 *
 * @param gpio
 * @return
 */
uint16_t platformPinGet(uint32_t gpio){
	if(gpio >= AVAILABLE_GPIO_PINS){
		return RET_ERROR;
	}
	return (uint16_t)HAL_GPIO_ReadPin(platformGpioPinTable[gpio].port, platformGpioPinTable[gpio].pin);
}




/*GPIO INTERRUPT HANDLERS*/
/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void)
{

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    if(interruptPinTable[0] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
    	gpioInterruptCallback(interruptPinTable[0]);
    }
  }

}
/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void)
{

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
    if(interruptPinTable[1] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
    	gpioInterruptCallback(interruptPinTable[1]);
    }
  }

}
/**
 * @brief This function handles EXTI line2 interrupt.
 */
void EXTI2_IRQHandler(void)
{

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    if(interruptPinTable[2] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
    	gpioInterruptCallback(interruptPinTable[2]);
    }
  }

}
/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler(void)
{

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    if(interruptPinTable[3] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
    	gpioInterruptCallback(interruptPinTable[3]);
    }
  }

}

/**
 *	@brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void)
{

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
    if(interruptPinTable[4] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
    	gpioInterruptCallback(interruptPinTable[4]);
    }
  }

}

/**
* @brief This function handles EXTI line5-9 interrupt.
*/
void EXTI9_5_IRQHandler(void)
{
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
	    if(interruptPinTable[5] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[5]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
	    if(interruptPinTable[6] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[6]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	    if(interruptPinTable[7] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[7]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
	    if(interruptPinTable[8] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[8]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
	    if(interruptPinTable[9] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[9]);
	    }
	  }
}
/**
* @brief This function handles EXTI line10-15 interrupt.
*/
void EXTI15_10_IRQHandler(void){
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
	    if(interruptPinTable[10] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[10]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
	    if(interruptPinTable[11] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[11]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
	    if(interruptPinTable[12] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[12]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
	    if(interruptPinTable[13] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[13]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
	    if(interruptPinTable[14] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[14]);
	    }
	  }
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
	    if(interruptPinTable[15] != 0xFFFFFFFF){			/*Check the interrupt of this pin is enabled in sw*/
	    	gpioInterruptCallback(interruptPinTable[15]);
	    }
	  }
}

#endif
