/**
  ******************************************************************************
  * @file    SPIRIT_Management.c
  * @author  VMA division - AMS
  * @version 3.2.3
  * @date    21-October-2015
  * @brief   The management layer for SPIRIT1 library.
  * @details
  *
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Management.h"
#include "SPIRIT_Radio.h"
#include "SPIRIT_Calibration.h"

/**
* @addtogroup SPIRIT_Libraries
* @{
*/


/**
* @defgroup SPIRIT_MANAGEMENT              SPIRIT Management
* @{
*/

/**
* @brief  BS value to write in the SYNT0 register according to the selected band
*/
static const uint8_t s_vectcBandRegValue[4]={SYNT0_BS_6, SYNT0_BS_12, SYNT0_BS_16, SYNT0_BS_32};

#define COMMUNICATION_STATE_TX          0
#define COMMUNICATION_STATE_RX          1
#define COMMUNICATION_STATE_NONE        2

//static uint32_t s_nDesiredFrequency;

volatile static uint8_t s_cCommunicationState = COMMUNICATION_STATE_NONE;


/**
* @brief  Factor is: B/2 used in the formula for SYNTH word calculation
*/
static const uint8_t s_vectcBHalfFactor[4]={(HIGH_BAND_FACTOR/2), (MIDDLE_BAND_FACTOR/2), (LOW_BAND_FACTOR/2), (VERY_LOW_BAND_FACTOR/2)};


/**
* @defgroup SPIRIT_MANAGEMENT_FUNCTIONS    SPIRIT Management Functions
* @{
*/


/**
* @defgroup WORKAROUND_FUNCTIONS              SPIRIT Management Workaround Functions
* @{
*/

/**
* @brief  Private SpiritRadioSetFrequencyBase function only used in SpiritManagementWaVcoCalibration.
* @param  lFBase the base carrier frequency expressed in Hz as unsigned word.
* @retval None.
*/
void SpiritManagementSetFrequencyBase(spirit1Data_t* spirit1Data, uint32_t lFBase, uint32_t xtal_freq)	//RRZ DONE
{
  uint32_t synthWord, Fc;
  uint8_t band, anaRadioRegArray[4], wcp;
  
  /* Check the parameter */
  s_assert_param(IS_FREQUENCY_BAND(lFBase));
  
  /* Search the operating band */
  if(IS_FREQUENCY_BAND_HIGH(lFBase))
  {
    band = HIGH_BAND;
  }
  else if(IS_FREQUENCY_BAND_MIDDLE(lFBase))
  {
    band = MIDDLE_BAND;
  }
  else if(IS_FREQUENCY_BAND_LOW(lFBase))
  {
    band = LOW_BAND;
  }
  else if(IS_FREQUENCY_BAND_VERY_LOW(lFBase))
  {
    band = VERY_LOW_BAND;
  }
  else{
	  band = HIGH_BAND;
  }
  
  int32_t FOffset  = SpiritRadioGetFrequencyOffset(spirit1Data, xtal_freq);
  uint32_t lChannelSpace  = SpiritRadioGetChannelSpace(spirit1Data, xtal_freq);
  uint8_t cChannelNum = SpiritRadioGetChannel(spirit1Data);
  
  /* Calculates the channel center frequency */
  Fc = lFBase + FOffset + lChannelSpace*cChannelNum;
  
  /* Reads the reference divider */
  uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv(spirit1Data)+1;
  
  switch(band)
  {
  case VERY_LOW_BAND:
    if(Fc<161281250)
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_L);
    }
    else
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_H);
    }
    break;
    
  case LOW_BAND:
    if(Fc<322562500)
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_L);
    }
    else
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_H);
    }
    break;
    
  case MIDDLE_BAND:
    if(Fc<430083334)
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_L);
    }
    else
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_H);
    }
    break;
    
  case HIGH_BAND:
    if(Fc<860166667)
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_L);
    }
    else
    {
      SpiritCalibrationSelectVco(spirit1Data, VCO_H);
    }
  }
  
  /* Search the VCO charge pump word and set the corresponding register */
  wcp = SpiritRadioSearchWCP(Fc);
  
  synthWord = (uint32_t)(lFBase*(((double)(FBASE_DIVIDER*cRefDiv*s_vectcBHalfFactor[band]))/xtal_freq));
  
  /* Build the array of registers values for the analog part */
  anaRadioRegArray[0] = (uint8_t)(((synthWord>>21)&(0x0000001F))|(wcp<<5));
  anaRadioRegArray[1] = (uint8_t)((synthWord>>13)&(0x000000FF));
  anaRadioRegArray[2] = (uint8_t)((synthWord>>5)&(0x000000FF));
  anaRadioRegArray[3] = (uint8_t)(((synthWord&0x0000001F)<<3)| s_vectcBandRegValue[band]);
  
  /* Configures the needed Analog Radio registers */
  spirit1_arch_write_multi_reg(spirit1Data, SYNT3_BASE, anaRadioRegArray, 4);
}

uint8_t SpiritManagementWaVcoCalibration(spirit1Data_t* spirit1Data, uint32_t xtal_freq)	//RRZ DONE
{
  uint8_t s_cVcoWordRx;
  uint8_t s_cVcoWordTx;
  uint32_t nFreq;
  uint8_t cRestore = 0;
  uint8_t cStandby = 0;
  uint8_t nLockwon=0;
  spirit1_status_t status;
  uint32_t t1;
  
  /* Enable the reference divider if the XTAL is between 48 and 52 MHz */
  if(xtal_freq>DOUBLE_XTAL_THR)
  {
    if(!SpiritRadioGetRefDiv(spirit1Data))
    {
      cRestore = 1;
      nFreq = SpiritRadioGetFrequencyBase(spirit1Data, xtal_freq);
      SpiritRadioSetRefDiv(spirit1Data, S_ENABLE);
      SpiritManagementSetFrequencyBase(spirit1Data, nFreq, xtal_freq);
    }
  }
  nFreq = SpiritRadioGetFrequencyBase(spirit1Data, xtal_freq);
  
  /* Increase the VCO current */
  uint8_t tmp = 0x19;
  spirit1_arch_write_reg(spirit1Data, 0xA1, tmp);
  
  SpiritCalibrationVco(spirit1Data, S_ENABLE);
  
  status = spirit1_arch_get_status(spirit1Data);

  if(status.SPIRIT1_STATE == SPIRIT1_STATE_STANDBY)
  {
    cStandby = 1;
    spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);
    t1 = osKernelSysTick();
    do{
    	status = spirit1_arch_get_status(spirit1Data);
    	if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
    		break;;
    	}
      
    }while(status.SPIRIT1_STATE != SPIRIT1_STATE_READY);
  }
  
  spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_LOCKTX);
  
  nLockwon=0;
  t1 = osKernelSysTick();
  do{
	status = spirit1_arch_get_status(spirit1Data);
    if(status.SPIRIT1_STATE == SPIRIT1_STATE_LOCKWON)
    {
      if(nLockwon++==5) return 1;
    }

	if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
		break;;
	}
  }while(status.SPIRIT1_STATE != SPIRIT1_STATE_LOCK);
    
  s_cVcoWordTx = SpiritCalibrationGetVcoCalData(spirit1Data);
  
  spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);
  
  t1 = osKernelSysTick();
  do{
	  	status = spirit1_arch_get_status(spirit1Data);
		if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
			break;;
		}
  }while(status.SPIRIT1_STATE != SPIRIT1_STATE_READY);
  
    
  spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_LOCKRX);
  
  nLockwon=0;
  t1 = osKernelSysTick();
  do{
	  	status = spirit1_arch_get_status(spirit1Data);
		if(status.SPIRIT1_STATE == SPIRIT1_STATE_LOCKWON)
		{
			if(nLockwon++==5) return 1;
		}
		if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
			break;;
		}
  }while(status.SPIRIT1_STATE != SPIRIT1_STATE_LOCK);
  
  s_cVcoWordRx = SpiritCalibrationGetVcoCalData(spirit1Data);
  
  spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);
  
  t1 = osKernelSysTick();
  do{
	  	status = spirit1_arch_get_status(spirit1Data);
		if((osKernelSysTick()-t1) > SPIRIT1_FUNCS_TIMEOUT){
			break;;
		}
  }while(status.SPIRIT1_STATE != SPIRIT1_STATE_READY);
  
  if(cStandby == 1)
  {
	  spirit1_arch_send_cmd_strobe(spirit1Data, SPIRIT1_STROBE_READY);
  }
  SpiritCalibrationVco(spirit1Data, S_DISABLE);
  
  /* Disable the reference divider if the XTAL is between 48 and 52 MHz */
  if(cRestore)
  {
    SpiritRadioSetRefDiv(spirit1Data, S_DISABLE);
    SpiritManagementSetFrequencyBase(spirit1Data, nFreq, xtal_freq);
  }
  
  /* Restore the VCO current */
  tmp = 0x11;
  spirit1_arch_write_reg(spirit1Data, 0xA1, tmp);
  
  SpiritCalibrationSetVcoCalDataTx(spirit1Data, s_cVcoWordTx);
  SpiritCalibrationSetVcoCalDataRx(spirit1Data, s_cVcoWordRx);
  
  return 0;
}


//void SpiritManagementWaCmdStrobeTx(void)
//{
//  if(s_cCommunicationState != COMMUNICATION_STATE_TX)
//  {
//    /* uint32_t xtal_frequency = SpiritRadioGetXtalFrequency(); */
//
//    /* To achive the max output power */
//    if(s_nDesiredFrequency>=150000000 && s_nDesiredFrequency<=470000000)
//    {
//      /* Optimal setting for Tx mode only */
//      SpiritRadioSetPACwc(LOAD_3_6_PF);
//    }
//    else
//    {
//      /* Optimal setting for Tx mode only */
//      SpiritRadioSetPACwc(LOAD_0_PF);
//    }
//
//    uint8_t tmp = 0x11; SpiritSpiWriteRegisters(0xa9, 1, &tmp); /* Enable VCO_L buffer */
//    tmp = 0x20; SpiritSpiWriteRegisters(PM_CONFIG1_BASE, 1, &tmp); /* Set SMPS switching frequency */
//
//    s_cCommunicationState = COMMUNICATION_STATE_TX;
//  }
//}
//
//
//void SpiritManagementWaCmdStrobeRx(void)
//{
//  if(s_cCommunicationState != COMMUNICATION_STATE_RX)
//  {
//    uint8_t tmp = 0x98; SpiritSpiWriteRegisters(PM_CONFIG1_BASE, 1, &tmp); /* Set SMPS switching frequency */
//    SpiritRadioSetPACwc(LOAD_0_PF); /* Set the correct CWC parameter */
//
//    s_cCommunicationState = COMMUNICATION_STATE_RX;
//  }
//}
//
//void SpiritManagementWaTRxFcMem(uint32_t nDesiredFreq)
//{
//  s_cCommunicationState = COMMUNICATION_STATE_NONE;
//  s_nDesiredFrequency = nDesiredFreq;
//}
//
//
//void SpiritManagementWaExtraCurrent(void)
//{
//  uint8_t tmp= 0xCA;SpiritSpiWriteRegisters(0xB2, 1, &tmp);
//  tmp= 0x04;SpiritSpiWriteRegisters(0xA8, 1, &tmp);
//  /* just a read to loose some microsecs more */
//  SpiritSpiReadRegisters(0xA8, 1, &tmp);
//  tmp= 0x00;SpiritSpiWriteRegisters(0xA8, 1, &tmp);
//}

/**
* @}
*/



/**
* @}
*/


/**
* @}
*/

/**
* @}
*/


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
