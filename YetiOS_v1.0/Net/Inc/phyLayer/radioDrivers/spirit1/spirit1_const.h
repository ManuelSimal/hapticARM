/*
 * Copyright (c) 2017, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * spirit1_const.h
 *
 *  Created on: 15 de nov. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file spirit1_const.h
 */
#ifndef APPLICATION_BOARD_PLATFORM_RADIO_SPIRIT1_INCLUDE_SPIRIT1_CONST_H_
#define APPLICATION_BOARD_PLATFORM_RADIO_SPIRIT1_INCLUDE_SPIRIT1_CONST_H_


typedef enum cmd_strobe_{

	SPIRIT1_STROBE_TX        	=     	0x60,
	SPIRIT1_STROBE_RX        	=     	0x61,
	SPIRIT1_STROBE_READY    	=     	0x62,
	SPIRIT1_STROBE_STANDBY   	=     	0x63,
	SPIRIT1_STROBE_SLEEP     	=     	0x64,
	SPIRIT1_STROBE_LOCKRX    	=     	0x65,
	SPIRIT1_STROBE_LOCKTX	 	=     	0x66,
	SPIRIT1_STROBE_SABORT    	=     	0x67,
	SPIRIT1_STROBE_LDC_RELOAD 	=		0x68,
	SPIRIT1_STROBE_SEQ_UPDATE 	=		0x69,
	SPIRIT1_STROBE_AES_ENC	 	=		0x6A,
	SPIRIT1_STROBE_AES_KEY	 	=	 	0x6B,
	SPIRIT1_STROBE_AES_DESC  	=		0x6C,
	SPIRIT1_STROBE_AES_KEYDESC 	= 		0x6D,
	SPIRIT1_STROBE_SRES      	=     	0x70,
	SPIRIT1_STROBE_FRX       	=     	0x71,
	SPIRIT1_STROBE_FTX       	=     	0x72,
}cmd_strobe_t;


typedef enum spirit1_state_{
	SPIRIT1_STATE_STANDBY           =0x40,	/*!< STANDBY */
	SPIRIT1_STATE_SLEEP             =0x36,	/*!< SLEEP */
	SPIRIT1_STATE_READY             =0x03,	/*!< READY */
	SPIRIT1_STATE_PM_SETUP          =0x3D,	/*!< PM_SETUP */
	SPIRIT1_STATE_XO_SETTLING       =0x23,	/*!< XO_SETTLING */
	SPIRIT1_STATE_SYNTH_SETUP       =0x53,	/*!< SYNT_SETUP */
	SPIRIT1_STATE_PROTOCOL          =0x1F,	/*!< PROTOCOL */
	SPIRIT1_STATE_SYNTH_CALIBRATION =0x4F,	/*!< SYNTH */
	SPIRIT1_STATE_LOCK              =0x0F,	/*!< LOCK */
	SPIRIT1_STATE_RX                =0x33,	/*!< RX */
	SPIRIT1_STATE_TX                =0x5F,	/*!< TX */
	SPIRIT1_STATE_LOCKWON           =0x13	/*!< LOCKWON */
}spirit1_state_t;


typedef struct spirit1_status_{
  uint8_t XO_ON:1;					/*!< This one bit field notifies if XO is operating
  	  	  	  	     	 	 	 		(XO_ON is 1) or not (XO_On is 0) */
  spirit1_state_t SPIRIT1_STATE: 7;	/*!< This 7 bits field indicates the state of the
   	   	   	   	     	 	 	 	 	 Main Controller of SPIRIT. The possible states
   	   	   	   	     	 	 	 	 	 and their corresponding values are defined in
   	   	   	   	     	 	 	 	 	 @ref spirit1_state_t */
  uint8_t ERROR_LOCK: 1;      		 /*!< This one bit field notifies if there is an
   	   	   	   	     	 	 	 	 	 error on RCO calibration (ERROR_LOCK is 1) or
   	   	   	   	     	 	 	 	 	 not (ERROR_LOCK is 0) */
  uint8_t RX_FIFO_EMPTY: 1;   		 /*!< This one bit field notifies if RX FIFO is empty
   	   	   	   	     	 	 	 	 	 (RX_FIFO_EMPTY is 1) or not (RX_FIFO_EMPTY is 0) */
  uint8_t TX_FIFO_FULL: 1;			 /*!< This one bit field notifies if TX FIFO is full
  	  	  	  	     	 	 	 	 	 (TX_FIFO_FULL is 1) or not (TX_FIFO_FULL is 0) */
  uint8_t ANT_SELECT: 1;       		 /*!< This one bit field notifies the currently selected
   	   	   	   	     	 	 	 	 	 antenna */
  uint8_t : 4;						 /*!< This 4 bits field are reserved and equal to 5 */

}spirit1_status_t;


#endif /* APPLICATION_BOARD_PLATFORM_RADIO_SPIRIT1_INCLUDE_SPIRIT1_CONST_H_ */
