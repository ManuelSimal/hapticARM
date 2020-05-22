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
 * drv2605L.h
 *
 *  Created on: Mar 22, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */


#ifndef drv2605L_H_
#define drv2605L_H_


#include "yetiOS.h"

#include <stdint.h>
#include "stm32l4xx_hal.h"

#include "drv2605L_Actuators.h"
#include "drv2605L_RegisterFile.h"
#include "drv2605L_ROMEffects.h"
#include "drv2605L_Const.h"
#include "drv2605LDefs.h"

#include "periph_drivers.h"

typedef enum i2cChannel {
	I2C_ACTUATOR_1 = PLATFORM_I2C1_DEVICE_ID,
	I2C_ACTUATOR_2 = PLATFORM_I2C2_DEVICE_ID,
	I2C_ACTUATOR_3 = PLATFORM_I2C1_DEVICE_ID,
}i2cChannel_t;


typedef struct drv2605L_{
    driver_state_t state;
    osMutexId lock;
    deviceFileHandler_t* i2cDevice;
    uint32_t ENgpio;
}drv2605L_t;

typedef union
{
    uint8_t waveformPlaylistData[12];
    struct
    {
        drv2605L_WaveformSeqReg_t waveformList[8];
        drv2605L_OvdrTimeOffsetReg_t ovdrTimeOffset;
        drv2605L_SustTimePosOffsetReg_t sustTimePosOffset;
        drv2605L_SustTimeNegOffsetReg_t sustTimeNegOffset;
        drv2605L_BrakeTimeOffsetReg_t brakeTimeOffset;
    };
} drv2605L_WaveformPlaylist_t;

/**
* @brief                Create a instance of drv260x driver.
* @param device         Pointer address to store allocated instance.
* @retval               RET_OK if successful.
*/
retval_t 
drv2605L_create (drv2605L_t** device);

/**
* @brief                Frees a instance of drv260x driver. It should have been deinited before.
* @param device         Instance to be freed.
* @retval               RET_OK if successful.
*/
retval_t 
drv2605L_delete (drv2605L_t* device);

/**
* @brief                    Initialize a instance. It will enable the device using the EN input.
* @param device             Instance to be initialized.
* @param i2cChannel        	I2C number to communicate with the driver.
* @param ENgpio             GPIO for EN input.
* @retval                   RET_OK if successful.
*/
retval_t
drv2605L_init(drv2605L_t* drvDevice,
		 	  i2cChannel_t i2cChannel,
			  uint32_t ENgpio);

/**
* @brief                Deinitialize a instance. It will disable the device using the EN input.
* @param device         Instance to be deinitialized.
* @retval               RET_OK if successful.
*/
retval_t
drv2605L_deinit(drv2605L_t* drvDevice);

/**
*   @brief Perform a complete reset of the DRV26x device.
*   @param device Instance to be used
*/
retval_t
drv2605L_reset(drv2605L_t* drvDevice);

/**
*   @brief Put the DRV26x device into software standby.
*   @param device Instance to be used
*/
retval_t
drv2605L_enterStandby(drv2605L_t* drvDevice);

/**
*   @brief Take the DRV26x device out of software standby.
*   @param device Instance to be used
*/
retval_t
drv2605L_exitStandby(drv2605L_t* drvDevice);

/**
*   @brief Query the device ID.
*   @param device Instance to be used
*   @return the device ID. A list of valid IDs is given below:
*           eDRV2604, edrv2605L, eDRV2604L, edrv2605L
*/
retval_t
drv2605L_getDeviceID(drv2605L_t* drvDevice,
                    drv2605L_DeviceID_t* id);

/**
*   @brief Put the DRV26x device into a specific operational mode.
*   @param device Instance to be used
*   @param mode defines the mode to set.  Valid parameters include the
*          following:
*          drv2605L_mode_internalTrigger, drv2605L_mode_externalEdgeTrigger,
*          drv2605L_mode_externalLevelTrigger, drv2605L_mode_PWMandAnalog,
*          drv2605L_mode_audioToVibe, drv2605L_mode_realtimePlayback,
*          drv2605L_mode_diagnostics, drv2605L_mode_autoCalibration
*/
retval_t
drv2605L_setMode(drv2605L_t* drvDevice,
               drv2605L_Mode_t mode);

/**
*   @brief Query the current operational mode of the DRV26x.
*   @param device Instance to be used
*   @return the current mode.  Options include the following:
*          drv2605L_mode_internalTrigger, drv2605L_mode_externalEdgeTrigger,
*          drv2605L_mode_externalLevelTrigger, drv2605L_mode_PWMandAnalog,
*          drv2605L_mode_audioToVibe, drv2605L_mode_realtimePlayback,
*          drv2605L_mode_diagnostics, drv2605L_mode_autoCalibration
*/
retval_t
drv2605L_getMode(drv2605L_t* drvDevice,
                drv2605L_Mode_t* mode);

/**
*   @brief Query whether a diagnostic or auto-calibration was successful.
*   @param device Instance to be used
*   @return true if a failure was detected, else false
*/
retval_t
drv2605L_getDiagResult(drv2605L_t* drvDevice,
                      bool* diagResult);

/**
*   @Brief Query whether the device is over-temp.
*   @param device Instance to be used
*   @return true if the device is exceeding the temperature threshold, else
*           false
*/
retval_t
drv2605L_getTempStatus(drv2605L_t* drvDevice,
                      bool* tempStatus);

/**
*   @brief Query whether the device load is below the impedance threshold,
*          causing an over-current condition.
*   @param device Instance to be used
*   @return true if an overcurrent event was detected, else false
*/
retval_t
drv2605L_getOverCurrentStatus(drv2605L_t* drvDevice,
                             bool* overCurrentStatus);

/**
*   @brief Fire playback on the DRV26x device.
*   @param device Instance to be used
*/
retval_t
drv2605L_go(drv2605L_t* drvDevice);

/**
*   @brief Cancel playback on the DRV26x device.
*   @param device Instance to be used
*/
retval_t
drv2605L_stop(drv2605L_t* drvDevice);

/**
*   @brief Wait until the GO status has cleared, indicating the activity
*          previously fired has completed and it is safe to again assert 
*          the GO bit via drv2605L_go().
*   @param device Instance to be used
*/
retval_t
drv2605L_blockUntilGoIsCleared(drv2605L_t* drvDevice);

/**
*   @brief Load in an actuator configuration before firing any effects.
*   @param device Instance to be used
*   @param actuator is the actuator structure to load to the DRV26x device.
*/
retval_t
drv2605L_loadActuatorConfig(drv2605L_t* drvDevice,
                          const drv2605L_Actuator_t *actuator);

/**
*   @brief Run an auto-calibration of the actuator attached to the DRV26x.  
*          This will modify the drv2605L_reg_autoCalibrationCompResult and
*          drv2605L_reg_autoCalibrationBackEMFResult registers on the device.
*          The  calibrated values may be loaded back into the application 
*          by calling drv2605L_stordrv2605L_mode_autoCalibrationResults, if
*          the actuator structure is stored in read/write capable memory.
*   @param device Instance to be used
*/
retval_t
drv2605L_runAutoCalibration(drv2605L_t* drvDevice,
                           bool* result);

/**
*   @brief Read back the drv2605L_reg_autoCalibrationCompResult and
*          drv2605L_reg_autoCalibrationBackEMFResult registers on the device
*          into an  actuator data structure, if the actuator structure 
*          is stored in read/write capable memory.
*   @param device Instance to be used
*   @param actuator is the actuator structure to store the results into.
*/
retval_t
drv2605L_storeAutoCalibrationResults(drv2605L_t* drvDevice,
                                   drv2605L_Actuator_t *actuator);

/**
*   @brief Select the haptic effect library to use.
*   @param device Instance to be used
*   @param library specifies the library to load.  Valid parameters include
*          the following:
*          drv2605L_lib_RAM = 0, drv2605L_lib_ROM_A, drv2605L_lib_ROM_B,
*          drv2605L_lib_ROM_C, drv2605L_lib_ROM_D, drv2605L_lib_ROM_E,
*          drv2605L_lib_ROM_LRA, drv2605L_lib_ROM_F
*/
retval_t
drv2605L_selectEffectLibrary(drv2605L_t* drvDevice,
                           drv2605L_Library_t library);

/**
*   @brief Trigger the playback of a single effect from the ROM library.
*   @param device Instance to be used
*   @param effect specifies the effect to play.  Effects may be found in the
*          drv2605L_ROMEffects.h file.
*   @param pass true to wait for all other effects to complete, else false
*/
retval_t
drv2605L_fireROMLibraryEffect(drv2605L_t* drvDevice,
                            drv2605L_Effect_t effect, bool wait);

/**
*   @brief Set the output driver to a high-z state.
*   @param device Instance to be used
*/
retval_t
drv2605L_setHighZOutputState(drv2605L_t* drvDevice);

/*   
*   @Brief Clear the output driver from a high-z state.
*   @param device Instance to be used
*/
retval_t
drv2605L_clearHighZOutputState(drv2605L_t* drvDevice);

/**
*   @brief Setup a playlist in the DRV26x without firing playback.
*   @param device Instance to be used
*   @param playlist is a pointer to the playlist to load.
*/
retval_t
drv2605L_loadWaveformPlaylist(drv2605L_t* drvDevice,
                            const drv2605L_WaveformPlaylist_t *playlist);

/**
*   @brief Load a playlist and fires playback immediately.
*   @param device Instance to be used
*   @param playlist is a pointer to the playlist to load.
*/
retval_t
drv2605L_fireWaveformPlaylist(drv2605L_t* drvDevice,
                            const drv2605L_WaveformPlaylist_t *playlist);

#endif /* drv2605L_H_ */
