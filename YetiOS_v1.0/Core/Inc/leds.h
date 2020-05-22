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
 * leds.h
 *
 *  Created on: 24 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file leds.h
 */

#ifndef YETIOS_CORE_INC_LEDS_H_
#define YETIOS_CORE_INC_LEDS_H_

#include "types.h"


/*Generic Led definitions. The available leds are platform-dependent. Led functions will return RET_ERROR
 * if using a LED not defined in the platform
 */
typedef enum ytLed_{
	LED_RED_1,
	LED_RED_2,
	LED_RED_3,
	LED_RED_4,

	LED_BLUE_1,
	LED_BLUE_2,
	LED_BLUE_3,
	LED_BLUE_4,

	LED_GREEN_1,
	LED_GREEN_2,
	LED_GREEN_3,
	LED_GREEN_4,

	LED_YELLOW_1,
	LED_YELLOW_2,
	LED_YELLOW_3,
	LED_YELLOW_4,

	LED_WHITE_1,
	LED_WHITE_2,
	LED_WHITE_3,
	LED_WHITE_4,
}ytLed_t;

retval_t platformLedsInit();
retval_t platformLedOn(ytLed_t led);
retval_t platformLedOff(ytLed_t led);
retval_t platformLedToggle(ytLed_t led);

/**
 *
 * @param led
 * @return
 */
INLINE_FUNC retval_t ytLedOn(ytLed_t led){
	return platformLedOn(led);
}

/**
 *
 * @param led
 * @return
 */
INLINE_FUNC retval_t ytLedOff(ytLed_t led){
	return platformLedOff(led);
}

/**
 *
 * @param led
 * @return
 */
INLINE_FUNC retval_t ytLedToggle(ytLed_t led){
	return platformLedToggle(led);
}

#endif /* YETIOS_CORE_INC_LEDS_H_ */
