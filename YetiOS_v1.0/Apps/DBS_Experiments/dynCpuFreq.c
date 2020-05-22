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
 * dynCpuFreq.c
 *
 *  Created on: 31 ago. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file dynCpuFreq.c
 */

/*Commented. TODO*/
//static float32_t* rewardsQTable;
//static uint32_t numStates;
//
//static uint32_t currentState;
//
//static float32_t learningRate;
//static float32_t discountFactor;
//
//static uint32_t currentRunMode;
//
//
///* Q Learning Functions*/
//static void dynCpuFreqFunc(void const * argument){
//
//
//	/*Initially set the maximum freq*/
//	setCurrentRunMode(0);
//	currentRunMode = getCurrentRunMode();
//
//	while(1){
//
//		if(trainingMode){
//			startEstimatingConsumption();
//			osDelay(TRAINING_PERIOD);
//			currentEst = getLastEstimatedConsumption();
//
//			currentState = getExecutionState(currentEst);
//			float32_t currentReward = -((float32_t)currentEst);
//			float32_t bestRewardQValue = getCurrentBestQValue(currentState);
//
//			/*Calculate the new Q value*/
//			float32_t newRewardQ = (1-learningRate)*rewardsQTable[(numStates*currentState) + currentRunMode];
//			newRewardQ += learningRate*(currentReward + (discountFactor*bestRewardQValue));
//
//			/*Place the value in the table*/
//			rewardsQTable[(numStates*currentState) + currentRunMode] = newRewardQ;
//
//			/*Update the Run Mode to test all of them and fill the table*/
//			currentRunMode++;
//			if(currentRunMode >= PLATFORM_NUM_RUN_MODES){
//				currentRunMode = 0;
//			}
//			setCurrentRunMode(currentRunMode);
//		}
//		else{
//			osDelay(CONTINUOUS_RUN_PERIOD);
//		}
//	}
//
//}
//
//static float32_t getCurrentBestQValue(uint32_t currentState){
//	uint16_t i;
//	float32_t maxQValue = -100000000;
//
//	for(i=0; i<PLATFORM_NUM_RUN_MODES; i++){
//		if(rewardsQTable[(numStates*currentState) + i] > maxQValue){
//			maxQValue = rewardsQTable[(numStates*currentState) + i];
//		}
//	}
//	return maxQValue;
//}
//
//
///* ***CLUSTERING FUNCTIONS***/
//#define MAX_CLUSTERS_NUM	32
//
//uint32_t getClusterNumber(float32_t threshold, float32_t* sampleBuff, uint32_t sampleNum, float32_t** clusterCentroids){
//	uint32_t i, j, k, centroidIndex;
//	float32_t currentCentroids[MAX_CLUSTERS_NUM];
//	float32_t minDistance, dist;
//	uint16_t existingCentroid;
//
//	k = 0;
//	for(i=0; i<sampleNum; i++){
//
//		centroidIndex = 0;
//		existingCentroid = 0;
//		minDistance = 100000000;
//		for(j=0; j<k; j++){
//
//			dist = currentCentroids[j] - sampleBuff[i];
//			dist *= dist;
//			if(dist < threshold){	/*This points belongs to an existing centroid*/
//				if(dist < minDistance){	/*Get the nearest centroid*/
//					minDistance = dist;
//					existingCentroid++;
//					centroidIndex = j;
//				}
//			}
//
//		}
//
//		if(!existingCentroid){	/*Add a new centroid*/
//			if(k < MAX_CLUSTERS_NUM){
//				currentCentroids[k] = sampleBuff[i];
//				k++;
//
//			}else{	/*Break. Error*/
//				return 0xFFFFFFFF;
//			}
//
//		}
//		else{	/*Recalculate the centroid value*/
//			currentCentroids[centroidIndex] += sampleBuff[i];
//			currentCentroids[centroidIndex] /= 2;
//		}
//	}
//
//	(*clusterCentroids) = (float32_t*) pvPortMalloc(sizeof(float32_t)*k);	/*Allocate memory for the calculated centroids to be returned*/
//	return k;
//}
//
//void kMeansAlg(uint32_t k, float32_t* initCent, float32_t* sampleBuff, uint32_t sampleNum){
//
//	float32_t* newCent = (float32_t*)pvPortMalloc(sizeof(float32_t)*k);
//	float32_t* currentCent = (float32_t*)pvPortMalloc(sizeof(float32_t)*k);
//	uint32_t* pointsCount = (float32_t*)pvPortMalloc(sizeof(uint32_t)*k);
//	uint32_t i, j, centroidIndex;
//	float32_t minDistance, currentDistance;
//	memcpy(currentCent, initCent, k*sizeof(float32_t));
//
//	while(1){
//		/*Initialize new centroids and the point counts of each centroid to zero*/
//		memset(pointsCount, 0, k*sizeof(uint32_t));
//		memset(newCent, 0, k*sizeof(float32_t));
//
//		/*For each point calculate the minimun distance to a centroid*/
//		for(i=0; i<sampleNum; i++){
//
//			centroidIndex = 0;
//			minDistance = 100000000;
//
//			for(j=0; j<k; j++){	/*Calculate the distance of this point to each centroid*/
//
//				currentDistance = sampleBuff(i) - currentCent(j);
//				currentDistance *= currentDistance;			/*Quadratic distance to remove negative numbers*/
//
//				if(currentDistance < minDistance){	/*Check the minimum distance*/
//					minDistance = currentDistance;
//					centroidIndex = j;
//				}
//
//				newCent[centroidIndex] += sampleBuff(i);	/*Add the point information to its nearest centroid*/
//				pointsCount[centroidIndex]++;
//			}
//		}
//
//		/*Calculate the new centroids*/
//		for(j=0; j<k; j++){
//			if(pointsCount[j] > 0){
//				newCent[j] /= pointsCount[j];
//			}
//			else{	/*Should not happen, but include it to prevent divison by zero*/
//				newCent[j] = currentCent[j];
//			}
//		}
//
//		if(compareEqualCentroids(k, newCent, currentCent)){	/*End algorithm when the centroids do not change*/
//			break;
//		}
//		memcpy(currentCent, newCent, k*sizeof(float32_t));	/*Update current to the new centroids*/
//	}
//	vPortFree(currentCent);
//	vPortFree(newCent);
//	vPortFree(pointsCount);
//}
//
//uint16_t compareEqualCentroids(uint32_t k, float32_t* newCent, float32_t* currentCent){
//	uint32_t i;
//	for(i=0; i<k; i++){
//		if(newCent[i] != currentCent[i]){
//			return 0;
//		}
//	}
//	return 1;
//}
//
