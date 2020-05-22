/*
 * lsm6dsl_arch.c
 *
 *  Created on: Jan 31, 2020
 *      Author: sreal
 */
#include <lsm6dsl_arch.h>
#include <lsm6dsl.h>
#include "lsm6dsl_const.h"
#include "yetiOS.h"



//static configAdcDevice(deviceFileHandler_t* adcDevice);
static deviceFileHandler_t* i2cDevice;



int32_t LSM6DSL_arch_Init(void){
	uint16_t addressSize = 1;

	/*Open I2C device*/
	if((i2cDevice = ytOpen(PLATFORM_I2C2_DEVICE_ID, SLAVE_ADDRESS)) == NULL){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	i2cDevice->device->ops->ioctl(i2cDevice, I2C_MEM_RW_SET_REG_ADD_SIZE, &addressSize);

	//i2cDevice->device->ops->ioctl();
	return 0;
}

int32_t LSM6DSL_arch_DeInit(void){

	if(ytClose(i2cDevice) != RET_OK){
		ytLedOn(PLATFORM_ERROR_LED);	/*Stop the app if ERROR happened*/
		while(1);
	}

	return 0;
}

int32_t LSM6DSL_arch_GetTick(void){
	//What? This is included in the library but seems like it is not used

	return 0;
}

int32_t LSM6DSL_arch_WriteReg(uint16_t address, uint16_t regAddress, uint8_t *buf, uint16_t length){
	//HAL_I2C_Mem_Write(hi2c, address, regAddress, 1, (uint8_t*) buf, length, 0xFFFFFFFF);
	i2cDevice->device->ops->ioctl(i2cDevice, I2C_MEM_RW_SET_REG_ADD, &regAddress);
	i2cDevice->device->ops->ioctl(i2cDevice, I2C_MEM_RW_SET_BUF_SIZE, &length);
	i2cDevice->device->ops->ioctl(i2cDevice, I2C_MEM_WRITE, buf);

	return 0;
}

int32_t LSM6DSL_arch_ReadReg(uint16_t address, uint16_t regAddress, uint8_t *buf, uint16_t length){
	//HAL_I2C_Mem_Read(hi2c, address, regAddress, 1, (uint8_t*) buf, length, 0xFFFFFFFF);
	i2cDevice->device->ops->ioctl(i2cDevice, I2C_MEM_RW_SET_REG_ADD, &regAddress);
	i2cDevice->device->ops->ioctl(i2cDevice, I2C_MEM_RW_SET_BUF_SIZE, &length);
	i2cDevice->device->ops->ioctl(i2cDevice, I2C_MEM_READ, buf);
	return 0;
}

/**
 *
 * @param adcDevice
 * @return
 */
//static retval_t configAdcDevice(deviceFileHandler_t* adcDevice){
//	adcChannelConfig_t adcChannel;
//
//	/*Set Sample Rate to 3125 Samples/s*/
//	uint32_t sampleRate = 3125;
//	if(ytIoctl(adcDevice, SET_ADC_SAMPLE_RATE, &sampleRate) != RET_OK){
//		return RET_ERROR;
//	}
//	/*Set Dual Mode*/
//	if(ytIoctl(adcDevice, SET_ADC_DUAL_MODE, NULL) != RET_OK){
//		return RET_ERROR;
//	}
//
//	/*Config first channel*/
//	adcChannel.channelMode = CHANNEL_SINGLE_ENDED;
//	adcChannel.gpioPin1 = VSYS_ADC_PIN;
//	if(ytIoctl(adcDevice, CONFIG_ADC_FIRST_CHANNEL, &adcChannel) != RET_OK){
//		return RET_ERROR;
//	}
//
//	/*Config second channel*/
//	adcChannel.channelMode = CHANNEL_SINGLE_ENDED;
//	adcChannel.gpioPin1 = VDIG_ADC_PIN;
//	if(ytIoctl(adcDevice, CONFIG_ADC_SECOND_CHANNEL, &adcChannel) != RET_OK){
//		return RET_ERROR;
//	}
//
//	/*Set async circular mode*/
//	if(ytIoctl(adcDevice, SET_ASYNC_CIRCULAR_READ_MODE, NULL) != RET_OK){
//		return RET_ERROR;
//	}
//
//	/*Set half complete callback*/
//	if(ytIoctl(adcDevice, SET_HALF_COMPLETE_CB, halfCommpleteCb) != RET_OK){
//		return RET_ERROR;
//	}
//
//	/*Set read complete callback*/
//	if(ytIoctl(adcDevice, SET_READ_COMPLETE_CB, halfCommpleteCb) != RET_OK){
//		return RET_ERROR;
//	}
//
//	/*Set half callback*/
//
//	return RET_OK;
//}
