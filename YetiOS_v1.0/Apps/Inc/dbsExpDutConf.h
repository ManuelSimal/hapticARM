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
 * dbsExpDutConf.h
 *
 *  Created on: 8 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file dbsExpDutConf.h
 */

#ifndef YETIOS_APPS_INC_DBSEXPDUTCONF_H_
#define YETIOS_APPS_INC_DBSEXPDUTCONF_H_

#ifdef APP_CUSTOM_CONFIG_DEFINED
#error "A custom config have already been selected. Please select only one custom config in appConfig.h file"
#endif
#define APP_CUSTOM_CONFIG_DEFINED


#if CONFIG_SELECTED_PLATFORM != HEIMDALL_PLATFORM
#error "This application is not supported for this board"
#endif

/*YetiOS Config defines*/
#define YETIOS_ENABLE_STDIO 			0		/*Do not use STDIO or Yetishell*/
#define YETIOS_ENABLE_YETISHELL			0
#define YETIOS_ENABLE_ADAPTIVE_ENGINE	1		/*The adaptive engine must be enabled to use this application*/
#define	YETIOS_STATUS_LED_TIME_ON		0


/*Heimdall platform Config defines*/
#define PLATFORM_DEFAULT_INIT_LOW_POWER_MODE	STOP_2_MODE
#define PLATFORM_ENABLE_STOP_MODE_DEBUG			0

#define PLATFORM_DEFAULT_INIT_RUN_MODE			RUN_MODE_48MHZ	/*This is the only Run mode allowed when using the USB*/
#define PLATFORM_SPI1_DEVICE_ID		2			/*Only use the SPI2 device driver*/
#define PLATFORM_SPI2_DEVICE_ID		1
#define PLATFORM_USB_DEVICE_ID		0			/*Disable USB driver*/

#endif /* YETIOS_APPS_INC_DBSEXPDUTCONF_H_ */
