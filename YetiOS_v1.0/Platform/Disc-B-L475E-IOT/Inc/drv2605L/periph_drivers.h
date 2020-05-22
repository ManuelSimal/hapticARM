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
 * periph_drivers.h
 *
 *  Created on: 6 de feb. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file periph_drivers.h
 */
#ifndef BSP_PERIPHERAL_PERIPH_DRIVERS_H_
#define BSP_PERIPHERAL_PERIPH_DRIVERS_H_


typedef enum{
	DRIVER_NOT_INIT = 0,		//!< DRIVER hw driver not initialized
	DRIVER_READY,				//!> DRIVER hw driver initialized
	DRIVER_BUSY,				//!> DRIVER driver is busy transmitting and/or receiving
	DRIVER_ERROR,				//!> DRIVER driver had an error
}driver_state_t;

typedef enum{
	DRIVER_PORT_CLOSED = 0,		//!> DRIVER port closed
	DRIVER_PORT_OPENED_READY,	//!> DRIVER port opened
	DRIVER_PORT_READING,		//!> DRIVER port is reading
	DRIVER_PORT_WRITING,		//!> DRIVER port is writing
	DRIVER_PORT_READ_WRITE,		//!> DRIVER port is performing duplex read/write
	DRIVER_PORT_ERROR,			//!> DRIVER port had an error
}port_state_t;


#endif /* BSP_PERIPHERAL_PERIPH_DRIVERS_H_ */
