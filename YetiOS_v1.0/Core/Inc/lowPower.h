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
 * lowPower.h
 *
 *  Created on: 17 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file lowPower.h
 */

#ifndef YETIOS_CORE_INC_LOWPOWER_H_
#define YETIOS_CORE_INC_LOWPOWER_H_

#include "types.h"
#include "platformLowPower.h"

/*Initialization function*/
retval_t ytInitLowPowerManager();

/*The low power mode numbers are ordered from the Highest power consumption and larger peripheral
 * availables (mode 0, typical Slep Mode), to the lowest power consumption and lowest peripheral available
 * (modes > 0, typical Deep Sleep Modes)*/
uint32_t ytGetCurrentLowPowerMode();
retval_t ytSetCurrentLowPowerMode(uint32_t newLowPowerMode);
uint32_t ytGetCurrentLowPowerConsumption();

/*The run mode numbers are ordered from the lowest frequency and lowest power consumption (mode 0, i.e. 1MHz),
 * to the highest frequency highest power consumption (mode > 0, i.e. 80 MHz)*/
uint32_t ytGetCurrentRunMode();
retval_t ytSetCurrentRunMode(uint32_t newRunMode);
uint32_t ytGetCurrentRunConsumption();


/*Used by peripherals to block entering the current low power mode*/
void ytPeripheralBlockLowPower();
void ytPeripheralReleaseLowPower();
uint16_t ytIsPeripheralBlockingSleep();

/*Used internally by port.c tickless routine to allow estimating power consumption*/
void ytAcumulateLowPowerTicks(uint32_t ticks);

/*Used by core or even user applications to estimate power consumption*/
void ytStartEstimatingConsumption();
uint32_t ytGetLastEstimatedConsumption();

/*Functions used by device drivers to set a low power mode that allows them keep running*/
void ytSetDeviceLowPowerMode(uint32_t selectedLowPowerMode);
void ytSetDeviceDefaultLowPowerMode();

/*Function used by YetiOS to change the curernt default low power mode*/
void ytUpdateDefaultLowPowerMode(uint32_t newLowPowerMode);

/*Functions used by device drivers to lock and unlock clock frequency changes when using them*/
void ytPeripheralFreqChangeLock();
void ytPeripheralFreqChangeUnlock();

#endif /* YETIOS_CORE_INC_LOWPOWER_H_ */
