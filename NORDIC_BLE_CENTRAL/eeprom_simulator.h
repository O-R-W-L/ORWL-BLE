/**===========================================================================
 * @file eeprom_simulator.h
 *
 * @brief This file contains the functions for simulating EEPROM memory in BLE
 * for comm. with the SUC
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

#include "sdk_errors.h"
#include "ble_flash.h"
#include "suc_ble_ipc.h"
/*#include "nrf_nvmc.h"*/

/*#define NVM_START_ADDR 0x00015000*/
#define eeprom_simulatorREAD_BUF_SIZE 16

 /** @brief Function to return the device information
  *
  *   This function reads the device information (device mac id and s/w version
  *   info) from the ble stack and stores it to the BLE IPC memory
  *
  *   @param xDevInfo    Structure to store mac id and S/W version info
  */
void vEeprom_simulatorReadDeviceInfo( bleDeviceInfo_t* xDevInfo );

 /**
  * @ingroup twi_master_with_twis_slave_example
  * @defgroup eeprom_simulator EEPROM simulator
  *
  * This module simulates the behavior of TWI EEPROM.
  * There are no functions to access internal memory array.
  * Use TWI interface to read or write any data.
  *
  * @attention
  * During initialization EEPROM memory is filled by pattern that is
  * values from 127 downto 0.
  * @{
  */
ret_code_t xEeprom_simulatorInit( void );

 /**
  * @brief Check if there was any error detected
  *
  * This function returns internal error flag.
  * Internal error flag is set if any error was detected during transmission.
  * To clear this flag use @ref eeprom_simulator_error_get_and_clear
  *
  * @retval true There is error detected.
  * @retval false There is no error detected.
  */
uint8_t ucEeprom_simulatorErrorCheck( void );

 /**
  * @brief Get and clear transmission error
  *
  * Function returns transmission error data and clears internal error flag
  *
  * @return Error that comes directly from @ref nrf_drv_twis_error_get
  */
uint32_t ulEeprom_simulatorErrorGetAndClear( void );
 /** @} */
