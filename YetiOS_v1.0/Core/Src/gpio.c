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
 * gpio.c
 *
 *  Created on: 27 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file gpio.c
 */


#include "yetiOS.h"
#include "gpio.h"

/*YetiOS GPIO pins ordered from 0 to PLATFORM_NUMBER_OF_PINS. Check platformGpio.h for specific platform translation*/

typedef struct gpioInterruptCallback_{
	ytThreadFunc_t gpioInterruptCallbackFunc;
	void* args;
}gpioInterruptCallback_t;


typedef struct ytGpioInitConf_{
	uint32_t	gpioPin;
	ytGpioMode_t gpioMode;
	ytGpioPull_t gpioPull;
	gpioInterruptCallback_t gpioInterruptCallback;
//	uint16_t gpio_pin_state;
}ytGpioInitConf_t;


static genList_t* gpioList;


static ytGpioInitConf_t* getPinFromList(uint32_t gpio);

/**
 *
 * @return
 */
retval_t ytGpioInit(){
	if(platformGpioInit() != RET_OK){
		return RET_ERROR;
	}
	if((gpioList = genListInit()) != NULL){
		return RET_OK;
	}
	return RET_ERROR;

}

/**
 *
 * @return
 */
retval_t ytGpioDeInit(){
	genListRemoveAndDeleteAll(gpioList);
	return platformGpioDeInit();
}

/**
 *
 * @param gpio
 * @param gpioMode
 * @param gpioPull
 * @return
 */
retval_t ytGpioInitPin(uint32_t gpio, ytGpioMode_t gpioMode, ytGpioPull_t gpioPull){
	/*First check if this pin is available by the platform*/
	if(!checkAvailablePlatformPin(gpio)){
		return RET_ERROR;
	}
	/*Now check if this pin has already been initialized*/
	if((getPinFromList(gpio)) != NULL){
		return RET_ERROR;
	}

	/*HW initialize Pin*/
	if(platformInitPin(gpio, gpioMode, gpioPull)!= RET_OK){
		return RET_ERROR;
	}

	/*Create the Pin Config struct and store it in the list to register the initialized pin*/
	ytGpioInitConf_t* gpioConf = (ytGpioInitConf_t*)pvPortMalloc(sizeof(ytGpioInitConf_t));
	gpioConf->gpioPin = gpio;
	gpioConf->gpioMode = gpioMode;
	gpioConf->gpioPull = gpioPull;
	gpioConf->gpioInterruptCallback.gpioInterruptCallbackFunc = NULL; /*The interrupt callback func is initialized disabled*/
	genListAdd(gpioList, (void*) gpioConf);

	return RET_OK;

}

/**
 *
 * @param gpio
 * @return
 */
retval_t ytGpioDeInitPin(uint32_t gpio){
	ytGpioInitConf_t* gpioConf;
	/*Check if this pin has been initialized*/
	if((gpioConf = getPinFromList(gpio)) == NULL){
		return RET_ERROR;
	}

	/*HW De-Init Pin*/
	if(platformDeInitPin(gpio) != RET_OK){
		return RET_ERROR;
	}
	gpioConf->gpioInterruptCallback.gpioInterruptCallbackFunc = NULL;

	/*Remove the pin from the list*/
	genListRemove(gpioList, gpioConf);

	return RET_OK;
}

/**
 *
 * @param gpio
 * @return
 */
retval_t ytGpioPinSet(uint32_t gpio){
	/*Check if this pin has been initialized*/
	if((getPinFromList(gpio)) == NULL){
		return RET_ERROR;
	}

	/*Platform Gpio Set*/
	return platformPinSet(gpio);
}

/**
 *
 * @param gpio
 * @return
 */
retval_t ytGpioPinReset(uint32_t gpio){
	/*Check if this pin has been initialized*/
	if((getPinFromList(gpio)) == NULL){
		return RET_ERROR;
	}

	/*Platform Gpio Reset*/
	return platformPinReset(gpio);
}

/**
 *
 * @param gpio
 * @return
 */
retval_t ytGpioPinToggle(uint32_t gpio){
	/*Check if this pin has been initialized*/
	if((getPinFromList(gpio)) == NULL){
		return RET_ERROR;
	}

	/*Platform Gpio Toggle*/
	return platformPinToggle(gpio);
}

/**
 *
 * @param gpio
 * @return
 */
uint16_t ytGpioPinGet(uint32_t gpio){
	/*Check if this pin has been initialized*/
	if((getPinFromList(gpio)) == NULL){
		return RET_ERROR;
	}

	/*Platform Gpio Get*/
	return platformPinGet(gpio);
}


/**
 *
 * @param gpio
 * @param gpioInterruptCallbackFunc
 * @param args
 * @return
 */
retval_t ytGpioPinSetCallback(uint32_t gpio, ytThreadFunc_t gpioInterruptCallbackFunc, void* args){
	ytGpioInitConf_t* gpioConf;
	/*Check if this pin has been initialized*/
	if((gpioConf = getPinFromList(gpio)) == NULL){
		return RET_ERROR;
	}

	/*Set the new callback function*/
	gpioConf->gpioInterruptCallback.args = args;
	gpioConf->gpioInterruptCallback.gpioInterruptCallbackFunc = gpioInterruptCallbackFunc;

	return RET_OK;

}

/**
 *
 * @param gpio
 */
void gpioInterruptCallback(uint32_t gpio){
	ytGpioInitConf_t* gpioConf;
	/*Check if this pin has been initialized*/
	if((gpioConf = getPinFromList(gpio)) == NULL){
		return;
	}

	/*Launch the callback function of this pin*/
	if(gpioConf->gpioInterruptCallback.gpioInterruptCallbackFunc != NULL){
		gpioConf->gpioInterruptCallback.gpioInterruptCallbackFunc(gpioConf->gpioInterruptCallback.args);
	}
}

/**
 *
 * @param gpio
 * @return
 */
static ytGpioInitConf_t* getPinFromList(uint32_t gpio){
	genListElement_t* current = gpioList->tailElement;
	ytGpioInitConf_t* gpioConf;

	while(current != NULL){
		gpioConf = (ytGpioInitConf_t*) current->item;
		if(gpioConf->gpioPin == gpio){
			return gpioConf;
		}
		current = current->next;
	}

	return NULL;
}


/*Weak platform functions definition*/
__weak retval_t platformGpioInit(){
	return RET_OK;
}
__weak retval_t platformGpioDeInit(){
	return RET_OK;
}
__weak uint16_t checkAvailablePlatformPin(uint32_t gpio){
	return 0;
}
__weak retval_t platformInitPin(uint32_t gpio, ytGpioMode_t gpioMode, ytGpioPull_t gpioPull){
	return RET_ERROR;
}
__weak retval_t platformDeInitPin(uint32_t gpio){
	return RET_ERROR;
}
__weak retval_t platformPinSet(uint32_t gpio){
	return RET_ERROR;
}
__weak retval_t platformPinReset(uint32_t gpio){
	return RET_ERROR;
}
__weak retval_t platformPinToggle(uint32_t gpio){
	return RET_ERROR;
}
__weak uint16_t platformPinGet(uint32_t gpio){
	return 0xFFFF;
}
