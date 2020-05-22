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
 * drv2605L.c
 *
 *  Created on: Mar 22, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */

#include "yetiOS.h"

#include "drv2605L.h"


osMutexDef (drv260x_mutex);

retval_t  drv2605L_register_read(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf);

retval_t drv2605L_register_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t buf);

retval_t  drv2605L_block_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf, uint16_t length);


retval_t  drv2605L_register_read(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf)
{
	uint16_t length = 1;

	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_RW_SET_REG_ADD, &regAddress);
	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_RW_SET_BUF_SIZE, &length);
	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_READ, buf);

    return RET_OK;
}

retval_t drv2605L_register_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t buf)
{
	uint16_t length = 1;

	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_RW_SET_REG_ADD, &regAddress);
	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_RW_SET_BUF_SIZE, &length);
	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_WRITE, &buf);

	return RET_OK;
}

retval_t  drv2605L_block_write(drv2605L_t* drvDevice, drv2605L_Register_t regAddress, uint8_t *buf, uint16_t length)
{

	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_RW_SET_REG_ADD, &regAddress);
	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_RW_SET_BUF_SIZE, &length);
	drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_WRITE, buf);

	return RET_OK;
}

retval_t 
drv2605L_create (drv2605L_t** drvDevice)
{
    /* Allocate memory */
    drv2605L_t* new_drvDevice = pvPortMalloc(sizeof(drv2605L_t));
    if (new_drvDevice == NULL)
    {
        return RET_ERROR;
    }

    new_drvDevice->lock = osMutexCreate(osMutex(drv260x_mutex));
    new_drvDevice->state = DRIVER_NOT_INIT;

    *drvDevice = new_drvDevice;
    return RET_OK;
}

retval_t 
drv2605L_delete (drv2605L_t* drvDevice)
{
    if (drvDevice == NULL)
    {
        return RET_ERROR;
    }

    if (drvDevice->state != DRIVER_NOT_INIT)
    {
        return RET_ERROR;
    }

    /* Delete mutex */
    osMutexDelete(drvDevice->lock);

    /* Free memory */
    vPortFree(drvDevice);

    return RET_OK;
}

retval_t
drv2605L_init(drv2605L_t* drvDevice,
			 i2cChannel_t i2cChannel,
			 uint32_t ENgpio)
{
    if (drvDevice == NULL)
    {
        return RET_ERROR;
    }

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    /* Check for already initialized driver */
    if(drvDevice->state != DRIVER_NOT_INIT)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }


    /* Enable drvDevice */
    drvDevice->ENgpio = ENgpio;

    ytGpioInitPin(ENgpio, GPIO_PIN_OUTPUT_PUSH_PULL, GPIO_PIN_NO_PULL);
    ytGpioPinSet(ENgpio);

    osDelay(5);


    /*Open I2C drvDevice */

    if((drvDevice->i2cDevice = ytOpen(i2cChannel, DRV2605L_I2C_ADDRESS)) == NULL){
    	ytLedOn(PLATFORM_ERROR_LED);	//Stop the app if ERROR happened
    	while(1);
    }

    uint16_t addressSize = 1;

    drvDevice->i2cDevice->device->ops->ioctl(drvDevice->i2cDevice, I2C_MEM_RW_SET_REG_ADD_SIZE, &addressSize);

    /* Initialize drvDevice */
    drvDevice->state = DRIVER_READY;

    osMutexRelease(drvDevice->lock);

    if (drv2605L_reset(drvDevice) != RET_OK)
    {
        osMutexWait(drvDevice->lock, osWaitForever);
        ytClose(drvDevice->i2cDevice);
        drvDevice->state = DRIVER_NOT_INIT;
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    return RET_OK;
}

retval_t
drv2605L_deinit(drv2605L_t* drvDevice)
{
    if (drvDevice == NULL)
    {
        return RET_ERROR;
    }

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    /* Disable drvDevice */
    ytGpioPinReset(drvDevice->ENgpio);

    /* Close I2C port */
    ytClose(drvDevice->i2cDevice);

    drvDevice->state = DRIVER_NOT_INIT;

    osMutexRelease(drvDevice->lock);
    
    return RET_OK;
}

retval_t
drv2605L_reset(drv2605L_t* drvDevice)
{
    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, 0) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    
    osMutexRelease(drvDevice->lock);
    return RET_OK;
}

retval_t
drv2605L_enterStandby(drv2605L_t* drvDevice)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if (drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    modeReg.b.bStandby = true;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, modeReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);
    return RET_OK;
}

retval_t
drv2605L_exitStandby(drv2605L_t* drvDevice)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    modeReg.b.bStandby = false;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, modeReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);
    return RET_OK;
}

retval_t
drv2605L_getDeviceID(drv2605L_t* drvDevice,
                    drv2605L_DeviceID_t* id)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    *id = (drv2605L_DeviceID_t)statusReg.b.bDeviceID;
    return RET_OK;
}

retval_t
drv2605L_setMode(drv2605L_t* drvDevice, drv2605L_Mode_t mode)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    modeReg.b.bMode = (uint8_t)mode;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, modeReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_getMode(drv2605L_t* drvDevice,
                drv2605L_Mode_t* mode)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if (drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    osMutexRelease(drvDevice->lock);

    *mode = (drv2605L_Mode_t)(modeReg.b.bMode);
    return RET_OK;
}

retval_t
drv2605L_getDiagResult(drv2605L_t* drvDevice,
                      bool* diagResult)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    *diagResult = statusReg.b.bDiagResult;
    return RET_OK;
}
retval_t
drv2605L_getTempStatus(drv2605L_t* drvDevice,
                      bool* tempStatus)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    *tempStatus = statusReg.b.bOverTemp;
    return RET_OK;
}

retval_t
drv2605L_getOverCurrentStatus(drv2605L_t* drvDevice,
                             bool* overCurrentStatus)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    *overCurrentStatus = statusReg.b.bOverCurrent;
    return RET_OK;
}

retval_t
drv2605L_loadActuatorConfig(drv2605L_t* drvDevice, const drv2605L_Actuator_t *actuator)
{
    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if (drv2605L_block_write(drvDevice,
                            drv2605L_reg_ratedVoltage,
                            (uint8_t*)&(actuator->actuatorData[0]),
                            10) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_go(drv2605L_t* drvDevice)
{
    drv2605L_GoReg_t goReg;
    goReg.b.bGo = true;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_write(drvDevice, drv2605L_reg_go, goReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_stop(drv2605L_t* drvDevice)
{
    drv2605L_GoReg_t goReg;
    goReg.b.bGo = false;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_write(drvDevice, drv2605L_reg_go, goReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_blockUntilGoIsCleared(drv2605L_t* drvDevice)
{
    drv2605L_GoReg_t goReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    do
    {
        if(drv2605L_register_read(drvDevice, drv2605L_reg_go, &(goReg.r)) != RET_OK)
        {
            osMutexRelease(drvDevice->lock);
            return RET_ERROR;
        }
    }
    while (goReg.b.bGo != false);
    
    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_runAutoCalibration(drv2605L_t* drvDevice,
                           bool* result)
{
    drv2605L_Mode_t contextSavedMode;
    bool diagnosticResult;

    if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_getMode(drvDevice, &contextSavedMode) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_setMode(drvDevice, drv2605L_mode_autoCalibration) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_go(drvDevice) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_getDiagResult(drvDevice, &diagnosticResult) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_setMode(drvDevice, contextSavedMode) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    
    *result = diagnosticResult;
    return RET_OK;
}

retval_t
drv2605L_storeAutoCalibrationResults(drv2605L_t* drvDevice, drv2605L_Actuator_t *actuator)
{
    uint8_t compensation, backEMF;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_autoCalibrationCompResult, &compensation) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    if(drv2605L_register_read(drvDevice, drv2605L_reg_autoCalibrationBackEMFResult, &backEMF) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    actuator->autoCal_BackEMF.b.bAutocalBackEMFResult = backEMF;
    actuator->autoCal_Compensation.b.bAutocalCompResult = compensation;
    return RET_OK;
}

retval_t
drv2605L_selectEffectLibrary(drv2605L_t* drvDevice, drv2605L_Library_t library)
{
    drv2605L_LibSelectReg_t libSelectReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_librarySelection, &(libSelectReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    libSelectReg.b.bLibrarySelection = (uint8_t)library;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_librarySelection, libSelectReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_fireROMLibraryEffect(drv2605L_t* drvDevice, drv2605L_Effect_t effect, bool wait)
{
    drv2605L_WaveformSeqReg_t playlist[2];

    playlist[0].b.bParam = effect;
    playlist[0].b.bDelay = false;
    playlist[1].b.bParam = drv2605L_effect_stop;
    playlist[1].b.bDelay = false;

    if (wait == true)
    {
        if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
        {
            osMutexRelease(drvDevice->lock);
            return RET_ERROR;
        }
    }

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_block_write(drvDevice, drv2605L_reg_waveformSequence1, &(playlist[0].r), 2) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    
    osMutexRelease(drvDevice->lock);

    if(drv2605L_go(drvDevice) != RET_OK)
    {
        return RET_ERROR;
    }

    return RET_OK;
}

retval_t
drv2605L_setHighZOutputState(drv2605L_t* drvDevice)
{
    drv2605L_LibSelectReg_t libSelectReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_librarySelection, &(libSelectReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    libSelectReg.b.bHighZ = true;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_librarySelection, libSelectReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_clearHighZOutputState(drv2605L_t* drvDevice)
{
    drv2605L_LibSelectReg_t libSelectReg;

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_librarySelection, &(libSelectReg.r)) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    libSelectReg.b.bHighZ = false;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_librarySelection, libSelectReg.r) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_loadWaveformPlaylist(drv2605L_t* drvDevice,
        const drv2605L_WaveformPlaylist_t *playlist)
{
    if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
    {
        return RET_ERROR;
    }

    /* Wait for mutex */
    osMutexWait(drvDevice->lock, osWaitForever);

    if(drv2605L_block_write(drvDevice,
    				       drv2605L_reg_waveformSequence1,
			               (uint8_t*)(&playlist->waveformPlaylistData[0]),
					       8) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_block_write(drvDevice,
    	                   drv2605L_reg_overdriveTimeOffset,
					       (uint8_t*)(&playlist->waveformPlaylistData[8]),
					       4) != RET_OK)
    {
        osMutexRelease(drvDevice->lock);
        return RET_ERROR;
    }

    osMutexRelease(drvDevice->lock);

    return RET_OK;
}

retval_t
drv2605L_fireWaveformPlaylist(drv2605L_t* drvDevice,
        const drv2605L_WaveformPlaylist_t *playlist)
{
    if(drv2605L_loadWaveformPlaylist(drvDevice, playlist) != RET_OK)
    {
        return RET_ERROR;
    }
    if(drv2605L_go(drvDevice) != RET_OK)
    {
        return RET_ERROR;
    }

    return RET_OK;
}

