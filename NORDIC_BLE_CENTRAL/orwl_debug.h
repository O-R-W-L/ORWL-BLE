/**===========================================================================
 * @file orwl_debug.h
 *
 * @brief This file contains the debug configurations used in ORWL BLE
 * 
 *
 * @author aakash.sarkar@design-shift.com
 *
============================================================================
 *
 * Copyright © Design SHIFT, 2017-2018
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright.
 *     * Neither the name of the [ORWL] nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY DESIGN SHIFT ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DESIGN SHIFT BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
============================================================================
 *
 */

#ifndef __ORWL_DEBUG_H__
#define __ORWL_DEBUG_H__
#include "SEGGER_RTT.h"

/* Configure the debug level for trouble shooting the application */
#define LEVEL_DEBUG 			(1)
#define LEVEL_INFO			(2)
#define LEVEL_ERR			(4)
/* Define the debugging print level */
#define DEBUG_LEVEL			(LEVEL_DEBUG|LEVEL_INFO|LEVEL_ERR)
/* Provide debug interface over UART or SWDIO Debug Interface */
#ifndef ORWL_CENTRAL
#define DEBUG_VIA_UART		/*  This will intialize debugging over UART Interface
						    Useful on Eval Kit - Port */
#endif
#define DEBUG_VIA_SWDIO_RTT		(1)
#ifdef DEBUG_VIA_SWDIO_RTT
extern char debug_rtt_buff[ 256 ];
#endif /* DEBUG_VIA_SWDIO_RTT */

#if defined(DEBUG_VIA_SWDIO_RTT) && defined(DEBUG_VIA_UART) && (DEBUG_LEVEL!=0)
#define debug_print(level,...)  		do{						\
							if(level&DEBUG_LEVEL)		\
							{				\
							printf(__VA_ARGS__) ;		\
							sprintf(debug_rtt_buff,__VA_ARGS__); \
							SEGGER_RTT_WriteString(0, debug_rtt_buff);\
							}					\
						  }while(0)
#elif defined(DEBUG_VIA_SWDIO_RTT)&& (DEBUG_LEVEL!=0)
#define debug_print(level,...)  		do{						\
							if(level&DEBUG_LEVEL)			\
							{					\
							sprintf(debug_rtt_buff,__VA_ARGS__); 	\
							SEGGER_RTT_WriteString(0, debug_rtt_buff);\
											}	\
									  }while(0)
#elif defined(DEBUG_VIA_UART)&& (DEBUG_LEVEL!=0)
#define debug_print(level, a,...)  		do{						\
							if(level&DEBUG_LEVEL)			\
									{			\
									printf(__VA_ARGS__) ;	\
											}	\
									  }while(0)
#else
#define debug_print(level, a,...)  		do{						\
						  }while(0)
#endif /* defined(DEBUG_VIA_SWDIO_RTT) && defined(DEBUG_VIA_UART)	*/
#endif /*__ORWL_DEBUG_H__	*/
