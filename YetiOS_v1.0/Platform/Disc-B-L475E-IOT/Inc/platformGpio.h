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


#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM

#define		GPIO_PIN_A0		0	/*Gpio Pin 0*/
#define		GPIO_PIN_A1	 	1	/*Gpio Pin 1*/
#define		GPIO_PIN_A2	 	2	/*Gpio Pin 2*/
#define		GPIO_PIN_A3	 	3	/*Gpio Pin 3*/
#define		GPIO_PIN_A4		4	/*Gpio Pin 4*/
#define		GPIO_PIN_A8		5	/*Gpio Pin 5*/
#define		GPIO_PIN_A15	6	/*Gpio Pin 6*/

#define		GPIO_PIN_B0		7	/*Gpio Pin 7*/
#define		GPIO_PIN_B1		8	/*Gpio Pin 8*/
#define		GPIO_PIN_B2 	9	/*Gpio Pin 9*/
#define		GPIO_PIN_B4		10	/*Gpio Pin 10*/
#define		GPIO_PIN_B5 	11	/*Gpio Pin 11*/
#define		GPIO_PIN_B12	12	/*Gpio Pin 12*/
#define		GPIO_PIN_B13	13	/*Gpio Pin 13*/
#define		GPIO_PIN_B15	14	/*Gpio Pin 14*/

#define		GPIO_PIN_C0 	15	/*Gpio Pin 15*/
#define		GPIO_PIN_C1 	16	/*Gpio Pin 16*/
#define		GPIO_PIN_C2		17	/*Gpio Pin 17*/
#define		GPIO_PIN_C3		18	/*Gpio Pin 18*/
#define		GPIO_PIN_C4		19	/*Gpio Pin 19*/
#define		GPIO_PIN_C5		20	/*Gpio Pin 20*/
#define		GPIO_PIN_C6		21	/*Gpio Pin 21*/
#define		GPIO_PIN_C7		22	/*Gpio Pin 22*/
#define		GPIO_PIN_C8		23	/*Gpio Pin 23*/
#define		GPIO_PIN_C13	24	/*Gpio Pin 24*/

#define		GPIO_PIN_D7		25	/*Gpio Pin 25*/
#define		GPIO_PIN_D10	26	/*Gpio Pin 26*/
#define		GPIO_PIN_D11	27	/*Gpio Pin 27*/
#define		GPIO_PIN_D13	28	/*Gpio Pin 28*/
#define		GPIO_PIN_D14	29	/*Gpio Pin 29*/
#define		GPIO_PIN_D15	30	/*Gpio Pin 30*/

#define		GPIO_PIN_E0		31	/*Gpio Pin 31*/
#define		GPIO_PIN_E1		32	/*Gpio Pin 32*/
#define		GPIO_PIN_E2		33	/*Gpio Pin 33*/
#define		GPIO_PIN_E4		34	/*Gpio Pin 34*/
#define		GPIO_PIN_E5		35	/*Gpio Pin 35*/
#define		GPIO_PIN_E6		36	/*Gpio Pin 36*/
#define		GPIO_PIN_E8		37	/*Gpio Pin 37*/

#define 	AVAILABLE_GPIO_PINS		38

#endif

#endif /* YETIOS_PLATFORM_HEIMDALL_INC_PLATFORMGPIO_H_ */
