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
 * timeMeas.c
 *
 *  Created on: 14 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file timeMeas.c
 */

#include "yetiOS.h"
#include "timeMeas.h"

#if PLATFORM_USE_TIME_MEAS
#include "timer.h"
#endif

/**
 *
 * @return
 */
retval_t ytInitTimeMeas(void){
#if PLATFORM_USE_TIME_MEAS
	return  initTimeMeasureTimer();
#endif
	return RET_OK;
}

/**
 * @brief			Start the time measurement
 * @note			The HW HS timer count is reseted when starting measure.
 * @param time_meas	Time measurement struct
 * @return			Return Status
 */
retval_t ytStartTimeMeasure(timeMeas_t* timeMeas){
#if PLATFORM_USE_TIME_MEAS
	timeMeasureResetCount();
	timeMeas->startTime = getTimeMeasureCount();
	return RET_OK;
#else
	return RET_ERROR;
#endif
}

/**
 * @brief			Acquire the elapsed time since the previous start_time_measure() call
 * @param time_meas	Time mesasurement struct
 * @return			Return Status
 */
retval_t ytStopTimeMeasure(timeMeas_t* timeMeas){
#if PLATFORM_USE_TIME_MEAS
	timeMeas->stopTime = getTimeMeasureCount();
	timeMeas->elapsedTime = timeMeas->stopTime - timeMeas->startTime;
	return RET_OK;
#else
		return RET_ERROR;
#endif
}
