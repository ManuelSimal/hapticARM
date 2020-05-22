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
 * quatOps.c
 *
 *  Created on: Apr 2, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */

#include "quatOps.h"
#include "yetiOS.h"
#include "stdio.h"


Quaternion ToQuaternion(double yaw, double pitch, double roll){ // yaw (Z), pitch (Y), roll (X)

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
}




