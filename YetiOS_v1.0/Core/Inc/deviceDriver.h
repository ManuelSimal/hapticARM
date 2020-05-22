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
 * deviceDriver.h
 *
 *  Created on: 26 jul. 2019
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file deviceDriver.h
 */

#ifndef YETIOS_CORE_INC_DEVICEDRIVER_H_
#define YETIOS_CORE_INC_DEVICEDRIVER_H_

#include "genList.h"

/*Sets the maximum allowed number of simultaneous opened file handlers for each device*/
#define MAX_DEVICE_FILE_HANDLERS	6

/* Ioctl commands available for device drivers using YetiOS
 * Only some of them are allowed for each specific platform and for each specific
 * device driver. If the command is not allowed the ytIoctl function returns RET_ERROR.
 * More commands may be included in future releases if required by new platforms or peripherals*/
typedef enum ytIoctlCmd_{
	/*Common commands*/
	/*ASYNC READ COMMANDS*/
	SET_ASYNC_READ_MODE,			/*Use the ytRead operation reading in async mode (non blocking)*/
	SET_ASYNC_CIRCULAR_READ_MODE,	/*Use the ytRead operation reading in async mode (non blocking) using an internal circular buffer*/
	STOP_ASYNC_READ,				/*Stops a currently running async read*/
	GET_CURRENT_READ_PTR, 			/*Returns (in the content of args*) the current reading pointer. Used in async read mode*/
	GET_CURRENT_REMAINING_COUNT,	/*Returns (in the content of args*) the current remaining bytes to be sent/written. Used in async read mode*/
	WAIT_FOR_NEW_DATA,				/*Function that waits until some new data is stored in the read buffer. Timeout specified in args. Used in async read mode*/
	SET_READ_COMPLETE_CB,			/*This function set a callback function to be executed when the read buffer is filled. Used in async read mode*/
	SET_HALF_COMPLETE_CB,			/*This function set a callback function to be executed when the read buffer is half filled. Used in async read mode*/
	/*STANDARD READ COMMANDS*/
	SET_STD_READ_MODE,				/*Use the ytRead operation reading in standard mode (blocking)*/
	/*SPI Device File specific commands*/
	SPI_READ_WRITE,			/*Standard Duplex read write operation. The args must be a ytSpiReadWriteBuff* pointer containing the buffers*/
	SPI_SET_SW_CS_PIN,		/*Enable the selected GPIO pin (from args casted to uint32_t*) as CS*/
	SPI_SET_SW_CS_EACH_BYTE,/*Enable the selected GPIO pin (from args casted to uint32_t*) as CS. The CS activates/deactivates for each byte*/
	SPI_SET_SW_CS_ALWAYS,	/*Enable the selected GPIO pin (from args casted to uint32_t*) as CS. The CS is always active (low level)*/
	/*SPI Device commands. The effects of these commands affects to every device file opened of the device*/
	SPI_SET_HW_CS,			/*Enable the automatic HW CS if provided by the platform*/
	SPI_SET_HW_PULSE_CS,	/*Enable the automatic HW CS for each byte if provided by the platform*/
	SPI_SET_SPEED,			/*Set the SPI speed to the selected in args (casted as uint32_t*), or to the closest available value*/
	SPI_SET_POLARITY,		/*Set the SPI polarity to the selected in args (casted as ytSpiPolarity_t*), HIGH or LOW*/
	SPI_SET_EDGE,			/*Set the SPI edge to the selected in args (casted as ytSpiEdge_t*), FIRST or SECOND */
	SPI_SET_MODE,			/*Set the SPI modee to the selected in args (casted as ytSpiMode_t*), MASTER or SLAVE */
	/*ADC Device commands*/
	SET_ADC_SAMPLE_RATE,		/*Set the sample rate of the ADC device (from args casted to uint32_t*)*/
	SET_ADC_DUAL_MODE,			/*Enable ADC Simultaneous dual mode*/
	SET_ADC_SINGLE_MODE,		/*Enable single or independent channel mode for ADC*/
	CONFIG_ADC_FIRST_CHANNEL,	/*Config a single ADC channel if SINGLE_MODE selected, or the main ADC channel if DUAL_MODE selected. Args-> adcChannelConfig_t* */
	CONFIG_ADC_SECOND_CHANNEL,	/*Config the second channel if DUAL_MODE selected. Args-> adcChannelConfig_t* */
	/*I2C Device commands. The effects of these commands affects to every device file opened of the device*/
	I2C_SET_FREQ_MODE,			/*Config the Clock Frequency of the I2C: (from args casted to uint16_t*): 0=>100 KHz; 1=>400KHz; 2=>1MHz  */
	I2C_MEM_RW_SET_REG_ADD,
	I2C_MEM_RW_SET_REG_ADD_SIZE,
	I2C_MEM_RW_SET_BUF_SIZE,
	I2C_MEM_READ,				/*Reads the specified memory address of the slave device*/
	I2C_MEM_WRITE,				/*Writes in the specified memory address of the slave device*/
	/*UART Device commands*/
	SET_UART_SPEED,
	UART_SET_NO_PARITY,
	UART_SET_PARITY_EVEN,
	UART_SET_PARITY_ODD,
	/*PWM Device commands*/
	PWM_SET_CARRIER_FREQ,
	PWM_SET_CHANNEL_1_DC,
	PWM_SET_CHANNEL_2_DC,
	PWM_SET_CHANNEL_3_DC,
	/*HAPTICS Device commands*/
	FIRE_HAPTIC_EFFECT,
}ytIoctlCmd_t;

struct devHandler_;
struct deviceFileHandler_;

typedef struct deviceDriverOps_{
	retval_t (*open)(struct deviceFileHandler_* devFile, uint32_t flags);
	retval_t (*close)(struct deviceFileHandler_* devFile);
	uint32_t (*read)(struct deviceFileHandler_* devFile, uint8_t* buff, uint32_t size);
	uint32_t (*write)(struct deviceFileHandler_* devFile, uint8_t* buff, uint32_t size);
	retval_t (*ioctl)(struct deviceFileHandler_* devFile, ytIoctlCmd_t command, void* args);
	retval_t (*changedCpuFreq)(struct devHandler_* devHandler);
}deviceDriverOps_t;

typedef struct devHandler_{
	uint32_t devId;			/*Unique ID of a device driver.*/
	deviceDriverOps_t* ops;	/*Device driver operations*/
	void* privateData;		/*Pointer to private data that may be used (or not) by any specific driver*/
	genList_t* deviceFileHandlerList; /*List containing the opened file handlers*/
	uint16_t deviceLock;	/*Counter increased each time anyone uses a device operation (read, write, ioctl)*/
	uint16_t devOpened;		/*Counts the number of times the device has been opened*/
}devHandler_t;

typedef struct deviceFileHandler_{
	devHandler_t* device; 	/*Reference to the parent device that contains this opened file handler*/
	void* privateData; 		/*Pointer to private data that may be used by the specific file operations*/
	uint16_t devFileReady;
}deviceFileHandler_t;


/*Highest priority device initialization function*/
#define InitDevice(fn) \
    static retval_t (*__initcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_initcall_0"))) = fn;\
    void fooInit##fn(void){ __initcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/

#define ExitDevice(fn) \
    static retval_t (*__exitcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_exitcall_0"))) = fn;\
    void fooExit##fn(void){ __exitcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/

/*Second priority device initialization function*/
#define InitDevice_1(fn) \
    static retval_t (*__initcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_initcall_1"))) = fn;\
    void fooInit1##fn(void){ __initcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/


#define ExitDevice_1(fn) \
    static retval_t (*__exitcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_exitcall_1"))) = fn;\
    void fooExit1##fn(void){ __exitcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/

/*Third priority device initialization function*/
#define InitDevice_2(fn) \
    static retval_t (*__initcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_initcall_2"))) = fn;\
    void fooInit2##fn(void){ __initcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/


#define ExitDevice_2(fn) \
    static retval_t (*__exitcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_exitcall_2"))) = fn;\
    void fooExit2##fn(void){ __exitcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/

/*Last priority device initialization function*/
#define InitDevice_3(fn) \
    static retval_t (*__initcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_initcall_3"))) = fn;\
    void fooInit3##fn(void){ __initcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/


#define ExitDevice_3(fn) \
    static retval_t (*__exitcall_##fn)(void) __attribute__((__used__)) \
    __attribute__((__section__(".dev_exitcall_3"))) = fn;\
    void fooExit3##fn(void){ __exitcall_##fn();}	/*This function definition is used to prevent unused warning from Eclipse indexer*/

/*Functions called to start and exit all device drivers*/
retval_t ytInitDevices(void);
retval_t ytExitDevices(void);

/*Functions used by specific device drivers to be registered or unregistered*/
devHandler_t* ytNewDevice(uint32_t deviceId, deviceDriverOps_t* ops);
retval_t ytDeleteDevice(devHandler_t* dev);
retval_t ytRegisterDevice(devHandler_t* dev);
devHandler_t* ytUnregisterDevice(uint32_t deviceId);

/*When the CPU frequency changes it is possible that some actions need to be performed by the peripherals
 * to get configured with the new clock*/
retval_t ytUpdateDevicesCpuFreq();

#endif /* YETIOS_CORE_INC_DEVICEDRIVER_H_ */
