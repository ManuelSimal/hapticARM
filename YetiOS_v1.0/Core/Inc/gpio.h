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
 * gpio.h
 *
 *  Created on: 27 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file gpio.h
 */

#ifndef YETIOS_CORE_INC_GPIO_H_
#define YETIOS_CORE_INC_GPIO_H_


typedef enum ytGpioMode_{
	GPIO_PIN_INPUT,
	GPIO_PIN_OUTPUT_PUSH_PULL,
	GPIO_PIN_OUTPUT_OPEN_DRAIN,
	GPIO_PIN_INTERRUPT_FALLING,
	GPIO_PIN_INTERRUPT_RISING,
	GPIO_PIN_INTERRUPT_FALLING_RISING,
	GPIO_PIN_ANALOG,
	GPIO_PIN_ANALOG_ADC,
}ytGpioMode_t;

typedef enum ytGpioPull_{
	GPIO_PIN_NO_PULL,
	GPIO_PIN_PULLUP,
	GPIO_PIN_PULLDOWN,
}ytGpioPull_t;

retval_t ytGpioInit();
retval_t ytGpioDeInit();

void gpioInterruptCallback(uint32_t gpio);

retval_t platformGpioInit();
retval_t platformGpioDeInit();
uint16_t checkAvailablePlatformPin(uint32_t gpio);
retval_t platformInitPin(uint32_t gpio, ytGpioMode_t gpioMode, ytGpioPull_t gpioPull);
retval_t platformDeInitPin(uint32_t gpio);
retval_t platformPinSet(uint32_t gpio);
retval_t platformPinReset(uint32_t gpio);
retval_t platformPinToggle(uint32_t gpio);
uint16_t platformPinGet(uint32_t gpio);


#endif /* YETIOS_CORE_INC_GPIO_H_ */
