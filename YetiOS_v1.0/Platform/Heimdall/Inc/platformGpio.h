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
 * platformGpio.h
 *
 *  Created on: 27 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file platformGpio.h
 */

#ifndef YETIOS_PLATFORM_HEIMDALL_INC_PLATFORMGPIO_H_
#define YETIOS_PLATFORM_HEIMDALL_INC_PLATFORMGPIO_H_


#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM

#define		GPIO_PIN_A0		0	/*Gpio Pin 0*/
#define		GPIO_PIN_A1	 	1	/*Gpio Pin 1*/
#define		GPIO_PIN_A4		2	/*Gpio Pin 2*/
#define		GPIO_PIN_B0		3	/*Gpio Pin 3*/
#define		GPIO_PIN_B1		4	/*Gpio Pin 4*/
#define		GPIO_PIN_B2 	5	/*Gpio Pin 5*/
#define		GPIO_PIN_B8		6	/*Gpio Pin 6*/
#define		GPIO_PIN_B9		7	/*Gpio Pin 7*/
#define		GPIO_PIN_B10	8	/*Gpio Pin 8*/
#define		GPIO_PIN_B11	9	/*Gpio Pin 9*/
#define		GPIO_PIN_B12	10	/*Gpio Pin 10*/
#define		GPIO_PIN_C0 	11	/*Gpio Pin 11*/
#define		GPIO_PIN_C1 	12	/*Gpio Pin 12*/
#define		GPIO_PIN_C2		13	/*Gpio Pin 13*/
#define		GPIO_PIN_C3		14	/*Gpio Pin 14*/
#define		GPIO_PIN_C4		15	/*Gpio Pin 15*/
#define		GPIO_PIN_C5		16	/*Gpio Pin 16*/
#define		GPIO_PIN_C6		17	/*Gpio Pin 17*/
#define		GPIO_PIN_C7		18	/*Gpio Pin 18*/
#define		GPIO_PIN_C13	19	/*Gpio Pin 19*/

#define 	AVAILABLE_GPIO_PINS		20

#endif

#endif /* YETIOS_PLATFORM_HEIMDALL_INC_PLATFORMGPIO_H_ */
