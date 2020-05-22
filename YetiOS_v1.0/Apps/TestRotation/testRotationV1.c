/*
 * Copyright (c) 2020, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * testRotationV1.c
 *
 *  Created on: Apr 2, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */


#include "yetiOS.h"
#include "lsm6dsl.h"
#include "lsm6dsl_arch.h"
#include "drv2605L.h"
#include "quatOps.h"

#if TEST_ROTATION_V1

typedef struct{
	LSM6DSL_Axes_t accValue;
	LSM6DSL_Axes_t gyroValue;
} imuPkg;

/*typedef struct{
	double w;
	double x;
	double y;
	double z;
} Quaternion;*/

static osThreadId testAppThreadHandle;

static void testAppFunc(void const * argument);

/*Quaternion ToQuaternion(double yaw, double pitch, double roll);
Quaternion conjugateQ(Quaternion q);
Quaternion normaliseQ(Quaternion q1);
Quaternion multiplicationQ(Quaternion q1, Quaternion q2);
Quaternion rotationQ(Quaternion q, Quaternion p0);*/
void IMUCallback(void const * argument);
//void hapticsCallback(void const * argument);

YtAutoInitThread(TestApp, testAppFunc, osPriorityLow, 256, &testAppThreadHandle, NULL);

osSemaphoreId IMUSemaphore;

deviceFileHandler_t* uartDevice;
deviceFileHandler_t* gyroDevice;
deviceFileHandler_t* hapticsDevice1;
imuPkg imuData;

double roll;
double pitch;
double yaw;

Quaternion p0;
Quaternion p;
Quaternion q1;
Quaternion q2;
Quaternion q3;

uint8_t notFirst = 0;
uint8_t notEffect = 0;

osTimerId tmr;

char outputLine[40];

static void testAppFunc(void const * argument){

	//drv2605L_Effect_t effect = drv2605L_effect_sharpClick_30P;

	roll = 0;
	pitch = 0;
	yaw = 0;


	p0.w = 0;
	p0.x = 7;
	p0.y = 0;
	p0.z = 0;

	p.w = 0;
	p.x = 7;
	p.y = 0;
	p.z = 0;


	if((uartDevice = ytOpen(PLATFORM_UART1_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}


	if((gyroDevice = ytOpen(PLATFORM_GYRO_DEVICE_ID, 0)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	if((hapticsDevice1 = ytOpen(PLATFORM_HAPTICS_DEVICE_ID, 1)) == NULL){
			ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
			while(1);
	}


	tmr = ytTimerCreate(osTimerPeriodic, IMUCallback, NULL);

	osTimerStart(tmr, 20);


	//IMUSemaphore = ytSemaphoreCreate(1);
	//osSemaphoreWait(IMUSemaphore, osWaitForever);
	while(1){
		//ytLedToggle(LED_GREEN_1);

		//osSemaphoreWait(IMUSemaphore, osWaitForever);


		//sprintf(outputLine, "%ld, %ld, %ld\n\r", imuDataPkg.gyroValue.x, imuDataPkg.gyroValue.y, imuDataPkg.gyroValue.z);
		/*sprintf(outputLine, "%f, %f, %f\n\r", p.x, p.y, p.z);
		uartDevice->device->ops->write(uartDevice, (uint8_t*)outputLine, strlen(outputLine));*/

		/*if ((p.x < -5) && (p.y > -2) && (p.y < 2) && (p.z > -1) && (p.z < 1) && (notEffect == 0)){
			hapticsDevice1->device->ops->ioctl(hapticsDevice1, FIRE_HAPTIC_EFFECT, &effect);
			ytLedToggle(LED_YELLOW_1);
		}
		sprintf(outputLine, "%f, %f, %f\n\r", p.x, p.y, p.z);
		uartDevice->device->ops->write(uartDevice, (uint8_t*)outputLine, strlen(outputLine));
		notEffect = notEffect + 1;
		if (notEffect = 4){
			notEffect = 0;
		}*/
		osDelay(2);
	}
}


/*Quaternion ToQuaternion(double yaw, double pitch, double roll){ // yaw (Z), pitch (Y), roll (X)

    // Abbreviations for the various angular functions
    double cy = cos(yaw / 2);
    double sy = sin(yaw / 2);
    double cp = cos(pitch / 2);
    double sp = sin(pitch / 2);
    double cr = cos(roll / 2);
    double sr = sin(roll / 2);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

Quaternion conjugateQ(Quaternion q1){

	Quaternion q;

	q.w = q1.w;
	q.x = -q1.x;
	q.y = -q1.y;
	q.z = -q1.z;

	return q;
}

Quaternion normaliseQ(Quaternion q1){
	double n = sqrt(q1.x*q1.x + q1.y*q1.y + q1.z*q1.z + q1.w*q1.w);
	Quaternion q;

	q.w = q1.w/n;
	q.x = q1.x/n;
	q.y = q1.y/n;
	q.z = q1.z/n;

	return q;
}

Quaternion multiplicationQ(Quaternion q1, Quaternion q2){

	Quaternion q;

	q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	q.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	q.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;

	return q;
}

Quaternion rotationQ(Quaternion q, Quaternion p0){

	Quaternion qC;

	qC = conjugateQ(q);


	Quaternion q1 = multiplicationQ(qC, p0);
	Quaternion q2 = multiplicationQ(q1, q);

	return q2;
}*/


void IMUCallback(void const * argument){

	drv2605L_Effect_t effect = drv2605L_effect_strongClick_30P;

	gyroDevice->device->ops->read(gyroDevice, (uint8_t*) &imuData.gyroValue, sizeof(LSM6DSL_Axes_t));

	yaw = ((double) (imuData.gyroValue.z - 440)) * 0.02 * 0.001 * M_PI / 180;
	pitch = ((double) (imuData.gyroValue.y + 160)) * 0.02 * 0.001 * M_PI / 180;
	roll = ((double) (imuData.gyroValue.x + 70)) * 0.02 * 0.001 * M_PI / 180;

	q1 = q3;
	q2 = ToQuaternion(yaw, pitch, roll);

	if (notFirst < 1){
		q3 = q2;
		notFirst = 1;
	}else{
		q3 = multiplicationQ(conjugateQ(q2), q1);
		q3 = normaliseQ(q3);
	}

	p = rotationQ(q3, p0);

	p.z = - p.z;

	if ((p.x < -5) && (p.y > -2) && (p.y < 2) && (p.z > -1) && (p.z < 1)){
		hapticsDevice1->device->ops->ioctl(hapticsDevice1, FIRE_HAPTIC_EFFECT, &effect);
		ytLedToggle(LED_YELLOW_1);
	}
	//osSemaphoreRelease(IMUSemaphore);
}

#endif

