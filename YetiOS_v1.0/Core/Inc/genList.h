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
 * genList.h
 *
 *  Created on: 26 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file genList.h
 */

#ifndef YETIOS_CORE_INC_GENLIST_H_
#define YETIOS_CORE_INC_GENLIST_H_


#include "types.h"

typedef struct genListElement_{
	void* item;
	struct genListElement_* next;
	struct genListElement_* prev;
}genListElement_t;

typedef struct genList{
	genListElement_t* headElement;	/*Last introduced element*/
	genListElement_t* tailElement;	/*First introduced element*/
	uint32_t numElements;			/*Number of elements introduced*/
}genList_t;

genList_t* genListInit();
void genListRemoveAll(genList_t* list);
void genListRemoveAndDeleteAll(genList_t* list);

retval_t genListAdd(genList_t* list, void* item);
retval_t genListRemoveAndDelete(genList_t* list, void* item);
retval_t genListRemove(genList_t* list, void* item);
retval_t genListRemoveLast(genList_t* list);
retval_t genListRemoveAndDeleteLast(genList_t* list);
void* genListGetLast(genList_t* list);
void* genListGetFirst(genList_t* list);
void* genListPopLast(genList_t* list);
void* genListPopFirst(genList_t* list);
uint32_t genListContains(genList_t* list, void* item);


#endif /* YETIOS_CORE_INC_GENLIST_H_ */
