/***************************************************************************//**
*   @file    AD7124.c
*   @brief   AD7124 implementation file.
*   @devices AD7124-4, AD7124-8
*
********************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include "ad7124.h"
#include "yetiOS.h"

/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */
#define SPI_RDY_POLL_CNT	1000

/***************************************************************************//**
 * @brief Reads the value of the specified register without checking if the
 *        device is ready to accept user requests.
 *
 * @param dev   - The handler of the instance of the driver.
 * @param pReg - Pointer to the register structure holding info about the
 *               register to be read. The read value is stored inside the
 *               register structure.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124NoCheckReadRegister(ad7124Dev_t *dev,
				      struct ad7124StReg* pReg)
{
	int32_t ret = 0;
	uint8_t rxBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t txBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i = 0;
	uint8_t check8 = 0;
	uint8_t msgBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	ytSpiReadWriteBuff_t spiReadWriteBuffs;
	spiReadWriteBuffs.prx = rxBuffer;
	spiReadWriteBuffs.ptx = txBuffer;

	if(!dev || !pReg)
		return INVALID_VAL;

	/* Build the Command word */
	txBuffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
		    AD7124_COMM_REG_RA(pReg->addr);

	/* Read data from the device */
	spiReadWriteBuffs.size = ((dev->useCrc != AD7124_DISABLE_CRC) ? pReg->size + 1 : pReg->size) + 1;
	if(ytIoctl(dev->spiFd, SPI_READ_WRITE, (void*) &spiReadWriteBuffs) != RET_OK){
		return -1;
	}

	/* Check the CRC */
	if(dev->useCrc == AD7124_USE_CRC) {
		msgBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
			     AD7124_COMM_REG_RA(pReg->addr);
		for(i = 1; i < pReg->size + 2; ++i) {
			msgBuf[i] = rxBuffer[i];
		}
		check8 = ad7124ComputeCrc8(msgBuf, pReg->size + 2);
	}

	if(check8 != 0) {
		/* ReadRegister checksum failed. */
		return COMM_ERR;
	}

	/* Build the result */
	pReg->value = 0;
	for(i = 1; i < pReg->size + 1; i++) {
		pReg->value <<= 8;
		pReg->value += rxBuffer[i];
	}

	return ret;
}

/***************************************************************************//**
 * @brief Writes the value of the specified register without checking if the
 *        device is ready to accept user requests.
 *
 * @param dev - The handler of the instance of the driver.
 * @param reg - Register structure holding info about the register to be written
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124NoCheckWriteRegister(ad7124Dev_t *dev,
				       struct ad7124StReg reg)
{
	int32_t ret = 0;
	int32_t regValue = 0;
	uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i = 0;
	uint8_t crc8 = 0;

	if(!dev)
		return INVALID_VAL;

	/* Build the Command word */
	wrBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR |
		    AD7124_COMM_REG_RA(reg.addr);

	/* Fill the write buffer */
	regValue = reg.value;
	for(i = 0; i < reg.size; i++) {
		wrBuf[reg.size - i] = regValue & 0xFF;
		regValue >>= 8;
	}

	/* Compute the CRC */
	if(dev->useCrc != AD7124_DISABLE_CRC) {
		crc8 = ad7124ComputeCrc8(wrBuf, reg.size + 1);
		wrBuf[reg.size + 1] = crc8;
	}

	/* Write data to the device */
	if(!ytWrite(dev->spiFd, wrBuf, (dev->useCrc != AD7124_DISABLE_CRC) ? reg.size + 2 : reg.size + 1)){
		return -1;
	}


	return ret;
}

/***************************************************************************//**
 * @brief Reads the value of the specified register only when the device is ready
 *        to accept user requests. If the device ready flag is deactivated the
 *        read operation will be executed without checking the device state.
 *
 * @param dev   - The handler of the instance of the driver.
 * @param pReg - Pointer to the register structure holding info about the
 *               register to be read. The read value is stored inside the
 *               register structure.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124ReadRegister(ad7124Dev_t *dev,
			     struct ad7124StReg* pReg)
{
	int32_t ret;

	if (pReg->addr != AD7124_ERR_REG && dev->checkReady) {
		ret = ad7124WaitForSpiReady(dev,
							SPI_RDY_POLL_CNT);
		if (ret < 0)
			return ret;
	}
	ret = ad7124NoCheckReadRegister(dev,
					    pReg);

	return ret;
}

/***************************************************************************//**
 * @brief Writes the value of the specified register only when the device is
 *        ready to accept user requests. If the device ready flag is deactivated
 *        the write operation will be executed without checking the device state.
 *
 * @param dev - The handler of the instance of the driver.
 * @param reg - Register structure holding info about the register to be written
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124WriteRegister(ad7124Dev_t *dev,
			      struct ad7124StReg pReg)
{
	int32_t ret;

	if (dev->checkReady) {
		ret = ad7124WaitForSpiReady(dev,
							SPI_RDY_POLL_CNT);
		if (ret < 0)
			return ret;
	}
	ret = ad7124NoCheckWriteRegister(dev,
					     pReg);

	return ret;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124Reset(ad7124Dev_t *dev)
{
	int32_t ret = 0;
	uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	if(!dev)
		return INVALID_VAL;

	ytWrite(dev->spiFd, wrBuf, 8);

	/* Wait for the reset to complete */
	ret = ad7124WaitToPowerOn(dev,
						SPI_RDY_POLL_CNT);

	return ret;
}

/***************************************************************************//**
 * @brief Waits until the device can accept read and write user actions.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124WaitForSpiReady(ad7124Dev_t *dev,
				  uint32_t timeout)
{
	struct ad7124StReg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!ready && --timeout) {
		/* Read the value of the Error Register */
		ret = ad7124ReadRegister(dev, &regs[AD7124_Error]);
		if(ret < 0)
			return ret;

		/* Check the SPI IGNORE Error bit in the Error Register */
		ready = (regs[AD7124_Error].value &
			 AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Waits until the device finishes the power-on reset operation.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124WaitToPowerOn(ad7124Dev_t *dev,
				uint32_t timeout)
{
	struct ad7124StReg *regs;
	int32_t ret;
	int8_t poweredOn = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!poweredOn && timeout--) {
		ret = ad7124ReadRegister(dev,
					   &regs[AD7124_Status]);
		if(ret < 0)
			return ret;

		/* Check the POR_FLAG bit in the Status Register */
		poweredOn = (regs[AD7124_Status].value &
			      AD7124_STATUS_REG_POR_FLAG) == 0;
	}

	return (timeout || poweredOn) ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Waits until a new conversion result is available.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns if no new data is available.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124WaitForConvReady(ad7124Dev_t *dev,
				   uint32_t timeout)
{
	struct ad7124StReg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!ready && --timeout) {
		/* Read the value of the Status Register */
		ret = ad7124ReadRegister(dev, &regs[AD7124_Status]);
		if(ret < 0)
			return ret;

		/* Check the RDY bit in the Status Register */
		ready = (regs[AD7124_Status].value &
			 AD7124_STATUS_REG_RDY) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Reads the conversion result from the device.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param p_data  - Pointer to store the read data.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_read_data(ad7124Dev_t *dev,
			 int32_t* pData)
{
	struct ad7124StReg *regs;
	int32_t ret;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	/* Read the value of the Status Register */
	ret = ad7124NoCheckReadRegister(dev, &regs[AD7124_Data]);

	/* Get the read result */
	*pData = regs[AD7124_Data].value;

	return ret;
}



int32_t ad7124ReadDataStatusAppend(ad7124Dev_t *dev,
			 int32_t* pData, uint8_t* pStatus)
{
	struct ad7124StReg *regs;
	int32_t ret = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	/* Read the value of the Status Register */

	uint8_t rxBuffer[5] = {0, 0, 0, 0, 0};
	uint8_t txBuffer[5] = {0, 0, 0, 0, 0};
	uint8_t i = 0;
	ytSpiReadWriteBuff_t ytSpiReadWriteBuff;
	ytSpiReadWriteBuff.prx = rxBuffer;
	ytSpiReadWriteBuff.ptx = txBuffer;

	if(!dev)
		return INVALID_VAL;

	/* Build the Command word */
	txBuffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
		    AD7124_COMM_REG_RA(regs[AD7124_Data].addr);

	/* Read data from the device */
	ytSpiReadWriteBuff.size = regs[AD7124_Data].size + 1 + 1;	//Include status byte and command byte
	if(ytIoctl(dev->spiFd, SPI_READ_WRITE, (void*) &ytSpiReadWriteBuff) != RET_OK){
		return -1;
	}

	/* Build the result */
	regs[AD7124_Data].value = 0;
	for(i = 1; i < regs[AD7124_Data].size + 1; i++) {
		regs[AD7124_Data].value <<= 8;
		regs[AD7124_Data].value += rxBuffer[i];
	}
	regs[AD7124_Status].value = rxBuffer[i];
	/* Get the read result */
	*pData = regs[AD7124_Data].value;
	*pStatus = regs[AD7124_Status].value;

	return ret;
}

/***************************************************************************//**
 * @brief Computes the CRC checksum for a data buffer.
 *
 * @param p_buf    - Data buffer
 * @param buf_size - Data buffer size in bytes
 *
 * @return Returns the computed CRC checksum.
*******************************************************************************/
uint8_t ad7124ComputeCrc8(uint8_t * pBuf, uint8_t bufSize)
{
	uint8_t i = 0;
	uint8_t crc = 0;

	while(bufSize) {
		for(i = 0x80; i != 0; i >>= 1) {
			if(((crc & 0x80) != 0) != ((*pBuf & i) != 0)) { /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			} else {
				crc <<= 1;
			}
		}
		pBuf++;
		bufSize--;
	}
	return crc;
}

/***************************************************************************//**
 * @brief Updates the CRC settings.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return None.
*******************************************************************************/
void ad7124UpdateCrcsetting(ad7124Dev_t *dev)
{
	struct ad7124StReg *regs;

	if(!dev)
		return;

	regs = dev->regs;

	/* Get CRC State. */
	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_CRC_ERR_EN) {
		dev->useCrc = AD7124_USE_CRC;
	} else {
		dev->useCrc = AD7124_DISABLE_CRC;
	}
}

/***************************************************************************//**
 * @brief Updates the device SPI interface settings.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return None.
*******************************************************************************/
void ad7124UpdateDevSpiSettings(ad7124Dev_t *dev)
{
	struct ad7124StReg *regs;

	if(!dev)
		return;

	regs = dev->regs;

	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN) {
		dev->checkReady = 1;
	} else {
		dev->checkReady = 0;
	}
}

/***************************************************************************//**
 * @brief Initializes the AD7124.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124Setup(ad7124Dev_t **device,
					struct ad7124StReg *regs)
{
	int32_t ret;
	enum ad7124Registers regNr;
	ad7124Dev_t *dev;

	dev = (ad7124Dev_t*)pvPortMalloc(sizeof(ad7124Dev_t));
	if (!dev)
		return INVALID_VAL;

	dev->regs = regs;


	/* Initialize the SPI communication. */
	ret = ad7124ConfigSpiDevice(dev);
//	ret = spi_init(&dev->spi_desc, &init_param.spi_init);
	if (ret < 0)
		return ret;

	/*  Reset the device interface.*/
	ret = ad7124Reset(dev);
	if (ret < 0)
		return ret;

	/* Update the device structure with power-on/reset settings */
	dev->checkReady = 1;

	/* Initialize registers AD7124_ADC_Control through AD7124_Filter_7. */
	for(regNr = AD7124_Status; (regNr < AD7124_Offset_0) && !(ret < 0);
			regNr++) {
		if (dev->regs[regNr].rw == AD7124_RW) {
			ret = ad7124WriteRegister(dev, dev->regs[regNr]);
			if (ret < 0)
				break;
		}

		/* Get CRC State and device SPI interface settings */
		if (regNr == AD7124_Error_En) {
			ad7124UpdateCrcsetting(dev);
			ad7124UpdateDevSpiSettings(dev);
		}
	}

	*device = dev;

	return ret;
}

/***************************************************************************//**
 * @brief Free the resources allocated by AD7124_Setup().
 *
 * @param dev - The device structure.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad7124Remove(ad7124Dev_t *dev)
{
	if(dev){
		ytClose(dev->spiFd);
		vPortFree(dev);
		return 0;
	}
	return -1;
}


__weak int32_t ad7124ConfigSpiDevice(ad7124Dev_t* ad7124Dev){
	return -1;
}
