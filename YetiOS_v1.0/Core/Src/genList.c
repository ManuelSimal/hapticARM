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
 * genList.c
 *
 *  Created on: 26 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file genList.c
 */

#include "genList.h"
#include "yetiOS.h"

genList_t* genListInit(){

	genList_t* list;

	if((list = pvPortMalloc(sizeof(genList_t))) != NULL){
		list->headElement = NULL;
		list->tailElement = NULL;
		list->numElements = 0;
	}
	return list;
}

/**
 *
 * @param list
 */
void genListRemoveAndDeleteAll(genList_t* list){

	while(list->headElement != NULL){
		genListRemoveAndDeleteLast(list);
	}
	vPortFree(list);
}

/**
 *
 * @param list
 */
void genListRemoveAll(genList_t* list){

	while(list->headElement != NULL){
		genListRemoveLast(list);
	}
	vPortFree(list);
}

/**
 *
 * @param list
 * @param item
 * @return
 */
retval_t genListAdd(genList_t* list, void* item){

	if(list->headElement == NULL){	/*The list is empty*/
		genListElement_t* newElement = (genListElement_t*)pvPortMalloc(sizeof(genListElement_t));
		newElement->item = item;
		newElement->next = NULL;
		newElement->prev = NULL;	/*That means it is the first element*/
		list->tailElement = newElement;
		list->headElement = newElement;
		list->numElements++;
	}
	else{
		genListElement_t* newElement = (genListElement_t*)pvPortMalloc(sizeof(genListElement_t));
		newElement->item = item;
		newElement->next = NULL;
		newElement->prev = list->headElement;
		list->headElement->next = newElement;
		list->headElement = newElement;
		list->numElements++;
	}
	return RET_OK;
}

/**
 *
 * @param list
 * @return
 */
retval_t genListRemoveLast(genList_t* list){

	if(list->headElement == NULL){	/*EMPTY list*/
		return RET_ERROR;
	}
	genListElement_t* lastElement = list->headElement;

	list->numElements--;
	list->headElement = lastElement->prev; /*If prev is null, the list is empty*/
	if(lastElement->prev == NULL){
		list->tailElement = NULL;
	}

	vPortFree(lastElement);

	return RET_OK;
}

/**
 *
 * @param list
 */
retval_t genListRemoveAndDeleteLast(genList_t* list){

	if(list->headElement == NULL){	/*EMPTY list*/
		return RET_ERROR;
	}
	genListElement_t* lastElement = list->headElement;

	list->numElements--;
	list->headElement = lastElement->prev; /*If prev is null, the list is empty*/
	if(lastElement->prev == NULL){
		list->tailElement = NULL;
	}

	vPortFree(lastElement->item);
	vPortFree(lastElement);

	return RET_OK;

}

/**
 *
 * @param list
 * @param item
 */
retval_t genListRemoveAndDelete(genList_t* list, void* item){

	genListElement_t* current = list->tailElement;

	while(current != NULL){
		if(current->item == item){
			if(current->prev == NULL){	/*If it is the first element set the new tail*/
				list->tailElement = current->next;
			}
			else{
				current->prev->next = current->next;
			}

			if(current->next == NULL){	/*If it is the last element change the head*/
				list->headElement = current->prev;
			}
			else{
				current->next->prev = current->prev;
			}
			list->numElements--;
			vPortFree(current->item);
			vPortFree(current);
			return RET_OK;
		}
		else{
			current = current->next;
		}
	}
	return RET_ERROR;
}

/**
 *
 * @param list
 * @param item
 */
retval_t genListRemove(genList_t* list, void* item){

	genListElement_t* current = list->tailElement;

	while(current != NULL){
		if(current->item == item){
			if(current->prev == NULL){	/*If it is the first element set the new tail*/
				list->tailElement = current->next;
			}
			else{
				current->prev->next = current->next;
			}

			if(current->next == NULL){	/*If it is the last element change the head*/
				list->headElement = current->prev;
			}
			else{
				current->next->prev = current->prev;
			}
			list->numElements--;
			vPortFree(current);
			return RET_OK;
		}
		else{
			current = current->next;
		}
	}
	return RET_ERROR;
}

/**
 *
 * @param list
 */
void* genListGetLast(genList_t* list){

	if(list->headElement == NULL){
		return NULL;
	}
	return list->headElement->item;
}

/**
 *
 * @param list
 */
void* genListGetFirst(genList_t* list){

	if(list->tailElement == NULL){
		return NULL;
	}
	return list->tailElement->item;
}

/**
 *
 * @param list
 */
void* genListPopLast(genList_t* list){


	if(list->headElement == NULL){
		return NULL;
	}
	void* pItem = list->headElement->item;
	genListElement_t* lastElement = list->headElement;

	list->numElements--;
	list->headElement = lastElement->prev; /*If prev is null, the list is empty*/
	if(lastElement->prev == NULL){
		list->tailElement = NULL;
	}

	vPortFree(lastElement);
	return pItem;
}

/**
 *
 * @param list
 */
void* genListPopFirst(genList_t* list){


	if(list->tailElement == NULL){
		return NULL;
	}
	void* pItem = list->tailElement->item;
	genListElement_t* firstElement = list->tailElement;

	list->numElements--;
	list->tailElement = firstElement->next; /*If next is null, the list is empty*/
	if(firstElement->next == NULL){
		list->headElement = NULL;
	}

	vPortFree(firstElement);
	return pItem;
}


/**
 *
 * @param list
 * @param item
 * @return
 */
uint32_t genListContains(genList_t* list, void* item){
	genListElement_t* current = list->tailElement;

	while(current != NULL){
		if(current->item == item){
			return 1;
		}
		current = current->next;
	}
	return 0;
}
