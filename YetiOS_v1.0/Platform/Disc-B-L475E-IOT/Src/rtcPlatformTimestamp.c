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
 * rtcPlatformTimestamp.c
 *
 *  Created on: 24 ene. 2020
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file rtcPlatformTimestamp.c
 */

#include "yetiOS.h"

#if CONFIG_SELECTED_PLATFORM == DISC_B_L475E_IOT_PLATFORM
#include "timestamp.h"

RTC_HandleTypeDef hrtc;

static retval_t prepareTimestamp(uint64_t newTimestamp, RTC_TimeTypeDef* rtcTime, RTC_DateTypeDef* rtcDate);

retval_t ytInitTimestamp(void){

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	/* Initialize RTC*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 31;
	hrtc.Init.SynchPrediv = 1023;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		return RET_ERROR;
	}

	/*Initialize RTC and set the Time and Date*/
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
	sTime.Hours = 12;
	sTime.Minutes = 30;
	sTime.Seconds = 0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		return RET_ERROR;
	}

	sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 24;
	sDate.Year = 20;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		return RET_ERROR;
	}

	HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
	}

	return RET_OK;
}

/**
 *
 * @return
 */
retval_t ytDeInitTimestamp(void){

	HAL_RTC_DeInit(&hrtc);

	return RET_OK;
}

/**
 *
 * @return
 */
uint64_t ytGetTimestamp(void){
	uint64_t timestamp = 0;
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef rtcDate;

	uint16_t year, isLeapYear, elapsedYear, numLeapYears, numLeapYears1970;

	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

	year =  rtcDate.Year + 2000;

	isLeapYear = 0;

	if((year%4) == 0){			/*Gregorian calendar for leap years*/
		isLeapYear = 1;
		if((year%100) == 0){
			isLeapYear = 0;
		}
		if((year%400) == 0){
			isLeapYear = 1;
		}
	}

	elapsedYear = year - 1970;
	numLeapYears = (year/4) - (year/100) + (year/400) - isLeapYear -1;
	numLeapYears1970 = 492 - 19 + 4; /*Num of leap years elapsed till 1970. Counts the 4 and 400 multiples, and discount 100 multiples.*/


	numLeapYears -= numLeapYears1970;
	elapsedYear -= numLeapYears;

	timestamp += (uint64_t) ((uint64_t)numLeapYears*366ULL*24ULL*3600ULL*1000ULL);
	timestamp += (uint64_t) ((uint64_t)elapsedYear*365ULL*24ULL*3600ULL*1000ULL);

	switch(rtcDate.Month){
	case 1:
		break;
	case 2:
		timestamp += (uint64_t)(31ULL * 24ULL * 3600ULL * 1000ULL);
		break;
	case 3:
		timestamp += (uint64_t)(59ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 4:
		timestamp += (uint64_t)(90ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 5:
		timestamp += (uint64_t)(120ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 6:
		timestamp += (uint64_t)(151ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 7:
		timestamp += (uint64_t)(181ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 8:
		timestamp += (uint64_t)(212ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 9:
		timestamp += (uint64_t)(243ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 10:
		timestamp += (uint64_t)(273ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 11:
		timestamp += (uint64_t)(304ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	case 12:
		timestamp += (uint64_t)(334ULL+(uint64_t)isLeapYear) * 24ULL * 3600ULL * 1000ULL;
		break;
	default:
		return 0;
		break;
	}
	timestamp += (uint64_t)(rtcDate.Date) * 24ULL * 3600ULL * 1000ULL;
	timestamp += (uint64_t)(rtcTime.Hours) * 3600ULL * 1000ULL;
	timestamp += (uint64_t)(rtcTime.Minutes) * 60ULL * 1000ULL;
	timestamp += (uint64_t)(rtcTime.Seconds) * 1000ULL;
	timestamp += (uint64_t)((1023-(uint64_t)rtcTime.SubSeconds)*1000ULL) / 1024ULL;

	return timestamp;
}

/**
 *
 * @param new_timestamp
 * @return
 */
retval_t ytSetTimestamp(uint64_t newTimestamp){
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef rtcDate;

	prepareTimestamp(newTimestamp, &rtcTime, &rtcDate);

	HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
	return RET_OK;
}


/**
 *
 * @param newTimestamp
 * @param rtcTime
 * @param rtcDate
 * @return
 */
static retval_t prepareTimestamp(uint64_t newTimestamp, RTC_TimeTypeDef* rtcTime, RTC_DateTypeDef* rtcDate){

	uint32_t day;
	uint16_t elapsed_leap_years = 0;
	uint16_t i;
	uint16_t real_year;
	uint16_t local_weekday;

	uint16_t is_leap_year = 0;
	uint64_t year = newTimestamp/1000;
	year = year/365;
	year = year/24;
	year = year/3600;
	year = year + 1970;
	real_year = year;
	year = real_year-2000;
	rtcDate->Year = (uint8_t) year;

	for(i=1972; i<real_year; i+=4){		//1972 First leap year since 1970
		elapsed_leap_years++;
	}

	if((real_year%4) == 0){			//Gregorian calendar for leap years
		is_leap_year = 1;
		if((real_year%100) == 0){
			is_leap_year = 0;
		}
		if((real_year%400) == 0){
			is_leap_year = 1;
		}
	}
	day = (uint32_t)(newTimestamp/(1000*3600*24));			//The elapsed days since 1st January 1970

	local_weekday = (uint16_t) (day%7);						//Init day Thursday 1st January 1970
	if(local_weekday >= 4){
		rtcDate->WeekDay = local_weekday - 3;
	}
	else{
		rtcDate->WeekDay = local_weekday + 4;
	}

	day = day - ((real_year-1970)*365) - elapsed_leap_years + 1;			//The day on this year

	if(day<=31){
		rtcDate->Month = 1;
		rtcDate->Date = day;
	}
	else if(day<= (59+is_leap_year)){
		rtcDate->Month = 2;
		rtcDate->Date = day-31;
	}
	else if(day<= (90+is_leap_year)){
		rtcDate->Month = 3;
		rtcDate->Date = day-(59+is_leap_year);
	}
	else if(day<= (120+is_leap_year)){
		rtcDate->Month = 4;
		rtcDate->Date = day-(90+is_leap_year);
	}
	else if(day<= (151+is_leap_year)){
		rtcDate->Month = 5;
		rtcDate->Date = day-(120+is_leap_year);
	}
	else if(day<= (181+is_leap_year)){
		rtcDate->Month = 6;
		rtcDate->Date = day-(151+is_leap_year);
	}
	else if(day<= (212+is_leap_year)){
		rtcDate->Month = 7;
		rtcDate->Date = day-(181+is_leap_year);
	}
	else if(day<= (243+is_leap_year)){
		rtcDate->Month = 8;
		rtcDate->Date = day-(212+is_leap_year);
	}
	else if(day<= (273+is_leap_year)){
		rtcDate->Month = 9;
		rtcDate->Date = day-(243+is_leap_year);
	}
	else if(day<= (304+is_leap_year)){
		rtcDate->Month = 10;
		rtcDate->Date = day-(273+is_leap_year);
	}
	else if(day<= (334+is_leap_year)){
		rtcDate->Month = 11;
		rtcDate->Date = day-(304+is_leap_year);
	}
	else if(day<= (365+is_leap_year)){
		rtcDate->Month = 12;
		rtcDate->Date = day-(334+is_leap_year);
	}
	rtcTime->Hours = (newTimestamp/(1000*3600))%24;
	rtcTime->Minutes = (newTimestamp/(1000*60))%60;
	rtcTime->Seconds = (newTimestamp/(1000))%60;
	rtcTime->SubSeconds = 1023 -(((newTimestamp%1000)*1024)/1000);
	rtcTime->StoreOperation = RTC_STOREOPERATION_RESET;
	rtcTime->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	rtcTime->TimeFormat = RTC_FORMAT_BIN;

	return RET_OK;
}

/**
 *
 * @param rtcHandle
 */
void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{
  if(rtcHandle->Instance==RTC)
  {
    __HAL_RCC_RTC_ENABLE();
  }
}

/**
 *
 * @param rtcHandle
 */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{
  if(rtcHandle->Instance==RTC)
  {
    __HAL_RCC_RTC_DISABLE();
  }
}
#endif
