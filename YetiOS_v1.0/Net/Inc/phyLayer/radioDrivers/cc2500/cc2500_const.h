/*
 * Copyright (c) 2018, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * cc2500_const.h
 *
 *  Created on: 3 de may. de 2018
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file cc2500_const.h
 */
#ifndef YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CONST_H_
#define YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CONST_H_



#define CC2500_XTAL_FREQ		26		//Xtal freq in MHz
#define CC2500_MIN_BASE_FREQ	2400	//Min base freq in Mhz
#define CC2500_MAX_BASE_FREQ	2483	//Max base freq in Mhz

/*
 * Definitions for the TI CC2500 transceiver. See the datasheet.
 */
/* configuration registers, can be read and written in burst. */
#define CC2500_IOCFG2         	0x00
#define CC2500_IOCFG1       	0x01
#define CC2500_IOCFG0       	0x02
#define CC2500_FIFOTHR      	0x03
#define CC2500_SYNC1        	0x04
#define CC2500_SYNC0        	0x05
#define CC2500_PKTLEN       	0x06
#define CC2500_PKTCTRL1     	0x07
#define CC2500_PKTCTRL0     	0x08
#define CC2500_ADDR         	0x09
#define CC2500_CHANNR       	0x0A
#define CC2500_FSCTRL1      	0x0B
#define CC2500_FSCTRL0      	0x0C
#define CC2500_FREQ2        	0x0D
#define CC2500_FREQ1        	0x0E
#define CC2500_FREQ0        	0x0F
#define CC2500_MDMCFG4      	0x10
#define CC2500_MDMCFG3      	0x11
#define CC2500_MDMCFG2      	0x12
#define CC2500_MDMCFG1      	0x13
#define CC2500_MDMCFG0      	0x14
#define CC2500_DEVIATN      	0x15
#define CC2500_MCSM2        	0x16
#define CC2500_MCSM1        	0x17
#define CC2500_MCSM0        	0x18
#define CC2500_FOCCFG       	0x19
#define CC2500_BSCFG        	0x1A
#define CC2500_AGCCTRL2     	0x1B
#define CC2500_AGCCTRL1     	0x1C
#define CC2500_AGCCTRL0     	0x1D
#define CC2500_WOREVT1      	0x1E
#define CC2500_WOREVT0      	0x1F
#define CC2500_WORCTRL      	0x20
#define CC2500_FREND1       	0x21
#define CC2500_FREND0       	0x22
#define CC2500_FSCAL3       	0x23
#define CC2500_FSCAL2       	0x24
#define CC2500_FSCAL1       	0x25
#define CC2500_FSCAL0       	0x26
#define CC2500_RCCTRL1      	0x27
#define CC2500_RCCTRL0      	0x28
#define CC2500_FSTEST       	0x29
#define CC2500_PTEST        	0x2A
#define CC2500_AGCTEST      	0x2B
#define CC2500_TEST2        	0x2C
#define CC2500_TEST1        	0x2D
#define CC2500_TEST0        	0x2E

/* Status Registers */
#define CC2500_PARTNUM      	0x30
#define CC2500_VERSION      	0x31
#define CC2500_FREQEST      	0x32
#define CC2500_LQI          	0x33
#define CC2500_RSSI         	0x34
#define CC2500_MARCSTATE    	0x35
#define CC2500_WORTIME1     	0x36
#define CC2500_WORTIME0     	0x37
#define CC2500_PKTSTATUS    	0x38
#define CC2500_VCO_VC_DAC   	0x39
#define CC2500_TXBYTES      	0x3A
#define CC2500_RXBYTES      	0x3B
#define CC2500_RCCTRL1_STATUS 	0x3C
#define CC2500_RCCTRL0_STATUS 	0x3D
#define CC2500_PATABLE  		0x3E
/* 0x2F is reserved */

#define CC2500_FIFO_ADDR		0x3F

typedef enum cc2500_cmd_strobe_{

	CC2500_STROBE_SRES        	=     	0x30,
	CC2500_STROBE_SFSTXON      	=     	0x31,
	CC2500_STROBE_SXOFF    		=     	0x32,
	CC2500_STROBE_SCAL   		=     	0x33,
	CC2500_STROBE_SRX     		=     	0x34,
	CC2500_STROBE_STX    		=     	0x35,
	CC2500_STROBE_SIDLE	 		=     	0x36,
	CC2500_STROBE_SWOR    		=     	0x38,
	CC2500_STROBE_SPWD 			=		0x39,
	CC2500_STROBE_SFRX 			=		0x3A,
	CC2500_STROBE_SFTX	 		=		0x3B,
	CC2500_STROBE_SWORRST	 	=	 	0x3C,
	CC2500_STROBE_SNOP  		=		0x3D,

}cc2500_cmd_strobe_t;


typedef enum cc2500_state_{
	CC2500_STATE_IDLE           	=0x00,	/*!< IDLE */
	CC2500_STATE_RX		            =0x01,	/*!< RX */
	CC2500_STATE_TX             	=0x02,	/*!< TX */
	CC2500_STATE_FSTXON		        =0x03,	/*!< FSTXON */
	CC2500_STATE_CALIBRATE	        =0x04,	/*!< CALIBRATE */
	CC2500_STATE_SETTLING 	        =0x05,	/*!< SETTLING */
	CC2500_STATE_RXFIFO_OVERFLOW	=0x06,	/*!< RXFIFO_OVERFLOW */
	CC2500_STATE_TXFIFO_UNDERFLOW   =0x07,	/*!< TXFIFO_UNDERFLOW */
}cc2500_state_t;

typedef struct cc2500_status_{
  uint8_t FIFO_BYTES_AVAILABLE: 4;
  cc2500_state_t CC2500_STATE: 3;
  uint8_t CHIP_RDYn: 1;
}cc2500_status_t;

typedef enum gdo_config_{
	CC2500_RX_FIFO_THR_1 		= 0x00,
	CC2500_RX_FIFO_THR_2 		= 0x01,
	CC2500_TX_FIFO_THR_1 		= 0x02,
	CC2500_TX_FIFO_THR_2 		= 0x03,
	CC2500_RX_FIFO_OVERFLOW		= 0x04,
	CC2500_TX_FIFO_UNDERFLOW	= 0x05,
	CC2500_SENT_RECEIVED		= 0x06,
	CC2500_CRC_OK_RCV			= 0x07,
	CC2500_PQI_REACHED			= 0x08,
	CC2500_CCA					= 0x09,
	CC2500_PLL_DET_OUT			= 0x0A,
	CC2500_SERIAL_CLOCK			= 0x0B,
	CC2500_SERIAL_SYNC_DATA_OUT	= 0x0C,
	CC2500_SERIAL_DATA_OUT		= 0x0D,
	CC2500_CARRIER_SENSE		= 0x0E,
	CC2500_CRC_OK				= 0x0F,
	CC2500_RX_HARD_DATA_1		= 0x16,
	CC2500_RX_HARD_DATA_0		= 0x17,
	CC2500_PA_PD				= 0x1B,
	CC2500_LNA_PD				= 0x1C,
	CC2500_RX_SYMBOL_TICK		= 0x1D,
	CC2500_WOR_EVNT0			= 0x24,
	CC2500_WOR_EVNT1			= 0x25,
	CC2500_CLK_32K				= 0x27,
	CC2500_CHIP_RDYn			= 0x29,
	CC2500_XOSC_STABLE			= 0x2B,
	CC2500_GDO0_Z_EN_N			= 0x2D,
	CC2500_HW_TO_0				= 0x2F,
	CC2500_CLK_XOSC_1			= 0x30,
	CC2500_CLK_XOSC_1_5			= 0x31,
	CC2500_CLK_XOSC_2			= 0x32,
	CC2500_CLK_XOSC_3			= 0x33,
	CC2500_CLK_XOSC_4			= 0x34,
	CC2500_CLK_XOSC_6			= 0x35,
	CC2500_CLK_XOSC_8			= 0x36,
	CC2500_CLK_XOSC_12			= 0x37,
	CC2500_CLK_XOSC_16			= 0x38,
	CC2500_CLK_XOSC_24			= 0x39,
	CC2500_CLK_XOSC_32			= 0x3A,
	CC2500_CLK_XOSC_48			= 0x3B,
	CC2500_CLK_XOSC_64			= 0x3C,
	CC2500_CLK_XOSC_96			= 0x3D,
	CC2500_CLK_XOSC_128			= 0x3E,
	CC2500_CLK_XOSC_192			= 0x3F
}gdo_config_t;

typedef enum cc2500_modulation_{
	CC2500_FSK_2	= 0x00,
	CC2500_GFSK		= 0x10,
	CC2500_OOK		= 0x30,
	CC2500_MSK		= 0x70
}cc2500_modulation_t;


#endif /* YETIOS_HW_RESOURCES_RADIO_CC2500_INCLUDE_CC2500_CONST_H_ */
