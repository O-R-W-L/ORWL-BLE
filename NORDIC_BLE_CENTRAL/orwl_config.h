/**===========================================================================
 * @file orwl_config.h
 *
 * @brief This file contains the current configurations used by ORWL BLE
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

#ifndef __ORWL_CONFIG_H__
#define __ORWL_CONFIG_H__

#include <stdio.h>
#include <stdbool.h>
#include <sdk_errors.h>
#include "orwl_debug.h"
#include "suc_ble_ipc.h"

/* Number can go upto 255 */
#define orwl_configSW_MAJOR_NUMBER	(3)
#define orwl_configSW_MINOR_NUMBER 	(0)
#define TRUE 				(1)
#define FALSE 				(0)
#define orwl_configIDLE			(1)     /**< BLE state. Scanning is disabled */
#define orwl_configOBSERVATION 		(0)	/**< BLE state. Scanning is activated */
#define orwl_configPROXIMITY_LOCK	(1)	/**< BLE substate. Device locked on proximity */
#define orwl_configPROXIMITY_UNLOCK	(0)	/**< BLE substate. Device unlocked on proximity */
#define orwl_configGPIO_OUTPUT_PIN  	(12)
#endif /*__ORWL_CONFIG_H__*/
