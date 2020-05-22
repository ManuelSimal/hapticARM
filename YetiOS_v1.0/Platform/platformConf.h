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
 * platformConf.h
 *
 *  Created on: 13 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file platformConf.h
 * @brief	Configuration of platform dependent code
 */

#ifndef YETIOS_CORE_PLATFORMCONF_H_
#define YETIOS_CORE_PLATFORMCONF_H_

#include "availablePlatforms.h"	/* Contains the currently available Boards for YetiOS */
#include "appConfig.h"			/* This file selects the config for the current app */

#define DEFAULT_PLATFORM		HEIMDALL_PLATFORM	/*This is the default board when it is not defined in a custom app config*/

#ifndef	CONFIG_SELECTED_PLATFORM
#define CONFIG_SELECTED_PLATFORM DEFAULT_PLATFORM
#warning "No board selected. Using Default board"
#endif

#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM
#include "heimdallConf.h"
#elif CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "disc-b-l475e-iotConf.h"
#else
#error "Please select an available Board in your application Conf file"
#endif



#endif /* YETIOS_CORE_PLATFORMCONF_H_ */
