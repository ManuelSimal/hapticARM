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
 * netYetiOS.h
 *
 *  Created on: 10 dic. 2019
 *  Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file netYetiOS.h
 */
#ifndef YETIOS_NET_NETYETIOS_H_
#define YETIOS_NET_NETYETIOS_H_

#include "netstack.h"

/* API FUNCS */
/* UDP Like UNRELIABLE COMMUNICATION FUNCS */
retval_t ytUdpOpen(uint16_t portNumber);
retval_t ytUdpClose(uint16_t portNumber);
uint32_t ytUdpSend(netAddr_t destAddr, uint16_t portNumber, uint8_t* data, uint32_t size);
uint32_t ytUdpRcv(netAddr_t fromAddr, uint16_t portNumber, uint8_t* data, uint32_t size);
retval_t ytUdpRcvAsync(uint16_t portNumber, netReadCbFunc_t readCbFunc);
retval_t ytUdpStopRcvAsync(uint16_t portNumber);

/*TODO*/
//float32_t net_uc_get_last_signal_level(uint16_t port);

/*TCP like RELIABLE CONNECTION FUNCS */
uint32_t ytTcpCreateServer(uint16_t portNumber);
retval_t ytTcpDeleteServer(uint32_t serverFd);

uint32_t ytTcpListenConnection(uint32_t serverFd, netAddr_t fromAddr);
uint32_t ytTcpConnectServer(netAddr_t destAddr, uint16_t portNumber);
retval_t ytTcpCloseConnection(uint32_t connectionFd);

uint32_t ytTcpSend(uint32_t connectionFd, uint8_t* data, uint32_t size);
uint32_t ytTcpRcv(uint32_t connectionFd, uint8_t* data, uint32_t size);
retval_t ytTcpRcvAsync(uint32_t connectionFd, netReadCbFunc_t readCbFunc);
retval_t ytTcpStopRcvAsync(uint32_t connectionFd);

uint32_t ytTcpCheckConnection(uint32_t connectionFd);
retval_t ytTcpSetConnectionTimeout(uint32_t connectionFd, uint32_t timeout);

/*TODO*/
//float32_t net_rc_get_last_signal_level(uint32_t connection_fd);


/*Network Addresses Functions */
netAddr_t ytNetNewAddr(char* netAddrStr, char format);
netAddr_t ytNetNewEmptyAddr(void);
retval_t ytNetDeleteNetAddr(netAddr_t netAddr);
retval_t ytNetAddrToString(netAddr_t netAddr, char* netAddrStr, char format);
retval_t ytNetAddrCpy(netAddr_t destAddr, netAddr_t fromAddr);
uint16_t ytNetAddrCmp(netAddr_t aAddr, netAddr_t bAddr);

netAddr_t ytNetGetNodeAddr(uint16_t index);
retval_t ytNetAddNodeAddr(netAddr_t nodeAddr);
retval_t ytNetRemoveNodeAddr(netAddr_t nodeAddr);

retval_t ytNetAddRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops);
retval_t ytNetRemoveRoute(netAddr_t destAddr, netAddr_t nextAddr, uint16_t hops);
retval_t ytNetSetNodeAsGw(void);

/*TODO*/
//uint16_t net_mac_is_node_linked(void);
//retval_t net_mac_set_duty_cycle(uint32_t duty_cycle);



#endif /* YETIOS_NET_NETYETIOS_H_ */
