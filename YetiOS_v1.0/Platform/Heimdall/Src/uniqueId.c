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
 * uniqueId.c
 *
 *  Created on: 9 ene. 2020
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file uniqueId.c
 */

#include "yetiOS.h"
#if CONFIG_SELECTED_PLATFORM == HEIMDALL_PLATFORM
#define NODEID_LOCATION_BASE ((uint32_t*)0x1FFF7590)

static uint32_t uniqueId96Bits[3];

#if PLATFORM_USE_16_BITS_UNIQUE_ID
/**
 *
 * @return
 */
uint16_t ytGetUniqueId16Bits(void){
	uint16_t uniqueId = 0;

	uniqueId = ((uint16_t)(NODEID_LOCATION_BASE)[0]) | ((uint16_t)((NODEID_LOCATION_BASE)[0]>>16));

	return uniqueId;
}
#endif


#if PLATFORM_USE_32_BITS_UNIQUE_ID
/**
 *
 * @return
 */
uint32_t ytGetUniqueId32Bits(void){
	uint32_t uniqueId = 0;

	uniqueId = (((NODEID_LOCATION_BASE)[0]) ^ ((NODEID_LOCATION_BASE)[1])) & ((NODEID_LOCATION_BASE)[2]);
	return uniqueId;
}
#endif

#if PLATFORM_USE_64_BITS_UNIQUE_ID
/**
 *
 * @return
 */
uint64_t ytGetUniqueId64Bits(void){
	uint64_t uniqueId = 0;


	uniqueId =   ((uint64_t)(NODEID_LOCATION_BASE)[0])  | (((uint64_t)(NODEID_LOCATION_BASE)[1])<<32);

	return uniqueId;
}
#endif

#if PLATFORM_USE_96_BITS_UNIQUE_ID
/**
 *
 * @return
 */
uint32_t* ytGetUniqueId96Bits(void){

	uniqueId96Bits[0] = 0;
	uniqueId96Bits[1] = 0;
	uniqueId96Bits[2] = 0;

	uniqueId96Bits[0] = (NODEID_LOCATION_BASE)[0];
	uniqueId96Bits[1] = (NODEID_LOCATION_BASE)[1];
	uniqueId96Bits[2] = (NODEID_LOCATION_BASE)[2];

	return uniqueId96Bits;
}
#endif

#endif
