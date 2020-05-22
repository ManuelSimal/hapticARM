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
 * heimdallConf.h
 *
 *  Created on: 14 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file heimdallConf.h
 */

#ifndef YETIOS_BOARD_HEIMDALL_HEIMDALLCONF_H_
#define YETIOS_BOARD_HEIMDALL_HEIMDALLCONF_H_

#if defined(STM32L476RETx)
#define USE_HEIMDALL_L4
#else
#error "Current selected CPU not allowed for this Board. Please change the current CPU by changing the Active Configuration in the Project Properties"
#endif

#ifdef USE_HEIMDALL_L4		/* Platform configuration for HEIMDALL_L4 BOARD */
#include "platformGpio.h"
#include "cerberusPhy.h"

/* Config the heap size reserved for dynamic memory in FreeRTOS */
#ifndef CONFIG_PLATFORM_FREERTOS_HEAP_SIZE
#define CONFIG_PLATFORM_FREERTOS_HEAP_SIZE		78000
#endif

/* Config the low power mode entered during sleep: SLEEP_MODE, STOP_1_MODE, STOP_2_MODE*/
#ifndef PLATFORM_DEFAULT_INIT_LOW_POWER_MODE
#define PLATFORM_DEFAULT_INIT_LOW_POWER_MODE	SLEEP_MODE
#endif

/* Config the running mode used on startup: from 48 MHz to 100 KHz*/
#ifndef PLATFORM_DEFAULT_INIT_RUN_MODE
#define PLATFORM_DEFAULT_INIT_RUN_MODE		RUN_MODE_48MHZ
#endif

/*Allow using the debugger while the MCU is in STOP mode. This will increase the power consumption.
 * In addition, it is important to note that if STOP modes are used, the Debugger will not work as
 * well as if these modes are not used, and may get occasionally disconnected.
 */
#ifndef PLATFORM_ENABLE_STOP_MODE_DEBUG
#define PLATFORM_ENABLE_STOP_MODE_DEBUG		1
#endif

/* Config Use Tracealyzer Software. This will use the USB interface to send trace data*/
#ifndef CONFIG_USE_TRACEALYZER_SW
#define CONFIG_USE_TRACEALYZER_SW		0	/* Default Value Disabled*/
#endif

/*Define the status LED*/
#ifndef PLATFORM_STATUS_LED
#define PLATFORM_STATUS_LED		LED_GREEN_1
#endif

/*Define the error LED*/
#ifndef PLATFORM_ERROR_LED
#define PLATFORM_ERROR_LED		LED_RED_2
#endif


/*Defines for SPI Driver. Comment (undefine) if not needed on startup*/
#ifndef PLATFORM_SPI1_DEVICE_ID
#define PLATFORM_SPI1_DEVICE_ID		1	/*Each different device driver must have a different ID defined on this file*/
#endif
#ifndef PLATFORM_SPI2_DEVICE_ID
#define PLATFORM_SPI2_DEVICE_ID		2	/*Each different device driver must have a different ID defined on this file*/
#endif
#if PLATFORM_SPI1_DEVICE_ID
	#ifndef PLATFORM_SPI1_PIN_PORT
		#define PLATFORM_SPI1_PIN_PORT		GPIOA
	#endif
	#ifndef PLATFORM_SPI1_CLK_PIN
		#define PLATFORM_SPI1_CLK_PIN		GPIO_PIN_5
	#endif
	#ifndef PLATFORM_SPI1_MISO_PIN
		#define PLATFORM_SPI1_MISO_PIN		GPIO_PIN_6
	#endif
	#ifndef PLATFORM_SPI1_MOSI_PIN
		#define PLATFORM_SPI1_MOSI_PIN		GPIO_PIN_7
	#endif
	#ifndef PLATFORM_SPI1_NSSP_PIN
		#define PLATFORM_SPI1_NSSP_PIN		GPIO_PIN_4
	#endif
#endif
#if PLATFORM_SPI2_DEVICE_ID
	#ifndef PLATFORM_SPI2_PIN_PORT
		#define PLATFORM_SPI2_PIN_PORT		GPIOB
	#endif
	#ifndef PLATFORM_SPI2_CLK_PIN
		#define PLATFORM_SPI2_CLK_PIN		GPIO_PIN_13
	#endif
	#ifndef PLATFORM_SPI2_MISO_PIN
		#define PLATFORM_SPI2_MISO_PIN		GPIO_PIN_14
	#endif
	#ifndef PLATFORM_SPI2_MOSI_PIN
		#define PLATFORM_SPI2_MOSI_PIN		GPIO_PIN_15
	#endif
	#ifndef PLATFORM_SPI2_NSSP_PIN
		#define PLATFORM_SPI2_NSSP_PIN		GPIO_PIN_12
	#endif
#endif

/*Defines for USB driver*/
#ifndef PLATFORM_USB_DEVICE_ID
#if (!CONFIG_USE_TRACEALYZER_SW)
#define PLATFORM_USB_DEVICE_ID		3	/*Each different device driver must have a different ID defined on this file*/
#endif
#endif

#if PLATFORM_USB_DEVICE_ID				/*The USB device must be enabled to use the stdio*/
#ifndef PLATFORM_DEFAULT_STDIO_INTERFACE
#define PLATFORM_DEFAULT_STDIO_INTERFACE	PLATFORM_USB_DEVICE_ID
#endif
#endif

#ifndef PLATFORM_ADC_DEVICE_ID
#define PLATFORM_ADC_DEVICE_ID		4
#endif


/*Defines for I2C Driver. Comment (undefine) if not needed on startup*/
#ifndef PLATFORM_I2C1_DEVICE_ID
#define PLATFORM_I2C1_DEVICE_ID		5	/*Each different device driver must have a different ID defined on this file*/
#endif
#ifndef PLATFORM_I2C2_DEVICE_ID
#define PLATFORM_I2C2_DEVICE_ID		6	/*Each different device driver must have a different ID defined on this file*/
#endif
#if PLATFORM_I2C1_DEVICE_ID
	#ifndef PLATFORM_I2C1_PIN_PORT
		#define PLATFORM_I2C1_PIN_PORT		GPIOB
	#endif
	#ifndef PLATFORM_I2C1_SCL_PIN
		#define PLATFORM_I2C1_SCL_PIN		GPIO_PIN_6
	#endif
	#ifndef PLATFORM_I2C1_SDA_PIN
		#define PLATFORM_I2C1_SDA_PIN		GPIO_PIN_7
	#endif
#endif
#if PLATFORM_I2C2_DEVICE_ID
	#ifndef PLATFORM_I2C2_PIN_PORT
		#define PLATFORM_I2C2_PIN_PORT		GPIOB
	#endif
	#ifndef PLATFORM_I2C2_SCL_PIN
		#define PLATFORM_I2C2_SCL_PIN		GPIO_PIN_10
	#endif
	#ifndef PLATFORM_I2C2_SDA_PIN
		#define PLATFORM_I2C2_SDA_PIN		GPIO_PIN_11
	#endif
#endif

/*Defines for AT30Ts75 Driver. Comment (undefine) if not needed on startup*/
#ifndef PLATFORM_AT30TS75_DEVICE_ID
#define PLATFORM_AT30TS75_DEVICE_ID		7	/*Each different device driver must have a different ID defined on this file*/
#endif

/*Defines for MMA8652 Driver. Comment (undefine) if not needed on startup*/
/*TODO: THIS DRIVER IS NOT DONE! Only implemented the Init function to set the accelerometer to sleep mode*/
#ifndef PLATFORM_MMA8652_DEVICE_ID
#define PLATFORM_MMA8652_DEVICE_ID	8	/*Each different device driver must have a different ID defined on this file*/
#endif

#ifndef PLATFORM_USE_TIME_MEAS
#define PLATFORM_USE_TIME_MEAS	1
#endif
#if	PLATFORM_USE_TIME_MEAS
#define	PLATFORM_TIME_MEASURE_RESOLUTION	1000000
#endif

/*Defines for unique ID*/
#ifndef PLATFORM_USE_16_BITS_UNIQUE_ID
#define	PLATFORM_USE_16_BITS_UNIQUE_ID	1
#endif
#ifndef PLATFORM_USE_32_BITS_UNIQUE_ID
#define	PLATFORM_USE_32_BITS_UNIQUE_ID	1
#endif
#ifndef PLATFORM_USE_64_BITS_UNIQUE_ID
#define	PLATFORM_USE_64_BITS_UNIQUE_ID	1
#endif
#ifndef PLATFORM_USE_96_BITS_UNIQUE_ID
#define	PLATFORM_USE_96_BITS_UNIQUE_ID	1
#endif

/*Netstack Phy default layer*/
#ifndef PLATFORM_PHY_DEFAULT_LAYER
#define	PLATFORM_PHY_DEFAULT_LAYER		cerberusPhyLayer
#endif

/*Cerberus configuration*/
#ifdef	PLATFORM_SPI2_DEVICE_ID
	#ifndef PLATFORM_USE_CERBERUS_PHY_LAYER
	#define PLATFORM_USE_CERBERUS_PHY_LAYER	1
	#endif

	#ifndef PLATFORM_CERBERUS_SPI_DEV
	#define PLATFORM_CERBERUS_SPI_DEV			PLATFORM_SPI2_DEVICE_ID
	#endif

	/* *********CC2500 DEFAULT CONFIG ***********/
	#ifndef PLATFORM_ENABLE_CC2500
	#define PLATFORM_ENABLE_CC2500				1
	#endif
	#ifndef PLATFORM_CC2500_CS_PIN
	#define PLATFORM_CC2500_CS_PIN				GPIO_PIN_B2
	#endif
	#ifndef PLATFORM_CC2500_GDO2_PIN
	#define PLATFORM_CC2500_GDO2_PIN			GPIO_PIN_C0
	#endif
	/* *********SPIRIT1 433 DEFAULT CONFIG ***********/
	#ifndef PLATFORM_ENABLE_SPIRIT1_433
	#define PLATFORM_ENABLE_SPIRIT1_433			1
	#endif
	#ifndef PLATFORM_SPIRIT1_433_CS_PIN
	#define PLATFORM_SPIRIT1_433_CS_PIN			GPIO_PIN_B12
	#endif
	#ifndef PLATFORM_SPIRIT1_433_SDN_PIN
	#define PLATFORM_SPIRIT1_433_SDN_PIN		GPIO_PIN_C7
	#endif
	#ifndef PLATFORM_SPIRIT1_433_GPIO3_PIN
	#define PLATFORM_SPIRIT1_433_GPIO3_PIN		GPIO_PIN_C4
	#endif
	/* *********SPIRIT1 868 DEFAULT CONFIG ***********/
	#ifndef PLATFORM_ENABLE_SPIRIT1_868
	#define PLATFORM_ENABLE_SPIRIT1_868			1
	#endif
	#ifndef PLATFORM_SPIRIT1_868_CS_PIN
	#define PLATFORM_SPIRIT1_868_CS_PIN			GPIO_PIN_B0
	#endif
	#ifndef PLATFORM_SPIRIT1_868_SDN_PIN
	#define PLATFORM_SPIRIT1_868_SDN_PIN		GPIO_PIN_C6
	#endif
	#ifndef PLATFORM_SPIRIT1_868_GPIO3_PIN
	#define PLATFORM_SPIRIT1_868_GPIO3_PIN		GPIO_PIN_C2
	#endif

	#ifndef CERBERUS_DEFAULT_TRANSCEIVER
	#define CERBERUS_DEFAULT_TRANSCEIVER		CERBERUS_SPIRIT1_433_TRANSCEIVER
	#endif
#endif
/* End Cerberus Config***/

#endif



#endif /* YETIOS_BOARD_HEIMDALL_HEIMDALLCONF_H_ */
