	/**===========================================================================
	* @file suc_ble_ipc.h
	*
	* @brief This file contains the Shared memory structure to be used 
	* for comm. between SUC and BLE
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

/*IPC Between SuC and Nordic BLE Observer for KeyFOB Proximity

128 Bytes of RAM
		 ___________________ ___________________
	0x7F|									|										|
		|										|										|
		|										|										|
		|										|										|
		|			resv					|										|
		|										|										|
	0x60|_________________|  	SuC Write Only	|
	0x5F|									|										|
		|										|										|
		|  Scan Data to be  |										|
		|  used by Observer	|										|
		|										|										|
		|										|										|
	0x40|_________________|___________________|
	  |		resv						|										|
	0x2F|_________________|										|
		|  ble device info	|										|
	0x22|_________________|				  					|
		|  Dynamic scan 		|										|
		|   Status					|										|
	0x1F|_________________|		SuC Read Only		|
		|										|										|
		|  Scan Data 				|										|
		|  used by Observer	|										|
		|  rssi range, adv	|										|
		|  attr							|										|
		|										|										|
	0x00|_________________|___________________|


		Write Memory
		0x60 - 0x7F =>	reserved	
						 				
		0x40 - 0x5F	=> 	31 bytes of scan data that need to be used by Observer.
						Dynamic update. Update every boot and as needed when rotation 
                      determined by SuC
 
		Read Memory
		0x2F - 0x3F =>  reserved

		0x22 - 0x2E =>  ble device info

		0x1F - 0x21 => 	scan status list
						
		0x00 - 0x1E =>  Current 31 bytes of scan data used by BLE Observer  
*/

 #ifndef __SUC_BLE_IPC_CONFIG_H__
 #define __SUC_BLE_IPC_CONFIG_H__
 #include <stdint.h>
 #include <ble.h>
 
#define suc_ble_ipcEEPROM_SIMUL_SIZE			0x80			/**< Simulation of EEPROM RAM SIZE Which is used for comm. between SUC and BLE */
#define	suc_ble_ipcSCAN_RSP_STATUS_HISTORY		0x03			/**< 3-Scan status for the device */
#define suc_ble_ipcBLE_ADV_MAX_SIZE             0x1F			/**< 31 bytes max adv data */
#define suc_ble_ipcBLE_ADV_CUR_SIZE             0x06			/**< 30 current size of advertisement used */
#define suc_ble_ipcADDR_BLE_SW_VERSION			0x29
#define suc_ble_ipcADDR_BLE_MAC_ID              0x22
#define suc_ble_ipcBLE_MAC_ADDR_SIZE			0x6
#define suc_ble_ipcBLE_IN_RANGE                 0x01			/**< Key Fob advertisement received */
#define suc_ble_ipcBLE_OUT_RANGE                0x00			/**< Key Fob advertisement not received */
#define suc_ble_ipcBUFF_SIZE_I2C                (suc_ble_ipcEEPROM_SIMUL_SIZE/2)
#define suc_ble_ipcBROADCAST_REFRESH_IN_SEC		1			/**< 1 Seconds is the beacon broadcast interval */
#define suc_ble_ipcIPC_READ_MEM_START 			0x00			/**< Read memory start xaddr */
#define suc_ble_ipcIPC_WRITE_MEM_START			0x40			/**< Write memory start xaddr */
#define suc_ble_ipcDEVICE_INFO_START_ADDR       0x22
#define suc_ble_ipcHEARTBEAT_INFO_START_ADDR    0x2F
#define suc_ble_ipcHEART_BEAT_INFO_SIZE         0x08
#define suc_ble_ipcPADDING_BYTES                0x00

#pragma pack(push,1)
/**@brief IPC write only memory for writing broadcast data
*
*/
typedef struct
{
	uint8_t ucdata[ suc_ble_ipcBLE_ADV_MAX_SIZE ];				/**< broadcast data given by SUC */
	uint8_t ucresv[ suc_ble_ipcEEPROM_SIMUL_SIZE -
                        suc_ble_ipcBLE_ADV_MAX_SIZE -
                        suc_ble_ipcIPC_WRITE_MEM_START ];
}suc_ble_ipc_write_t ;

/**
*@brief BLE device information
*/
typedef struct
{
	ble_gap_addr_t  xble_mac_id;						/**< bt mac address */
	ble_version_t   xble_version;						/**< version info */
} ble_device_info_t ;

/**@brief IPC Read only memory for reading device info
*
*/
typedef struct
{
  uint8_t ucdata[ suc_ble_ipcBLE_ADV_MAX_SIZE ];				/**<broadcast data currently used by BLE*/
  uint8_t ucscan_status[ suc_ble_ipcSCAN_RSP_STATUS_HISTORY ] ;			/**<scan status list */
  ble_device_info_t xdevice_info;						/**<BLE MAC aadr + BLE software version info */
  uint64_t ulheart_beat_info;							/**<to check if the device is hanged or not */
  uint8_t ucresv[ suc_ble_ipcEEPROM_SIMUL_SIZE -
		 (suc_ble_ipcIPC_WRITE_MEM_START +
		  suc_ble_ipcBLE_ADV_MAX_SIZE +
		  suc_ble_ipcSCAN_RSP_STATUS_HISTORY +
		  sizeof(ble_device_info_t) +
		  suc_ble_ipcHEART_BEAT_INFO_SIZE +
		  suc_ble_ipcPADDING_BYTES)];
}suc_ble_ipc_read_t ;

/**@brief Complete IPC memory for SUC - BLE communication
*
*/
typedef struct
{
	suc_ble_ipc_read_t xmemory_read ;					/**< read only memory for I2C comm. */
	suc_ble_ipc_write_t xmemory_write ;					/**< write only memory for I2C comm.*/
}suc_ble_ipc_t ;
#pragma pack(pop)
#endif /*__SUC_BLE_IPC_CONFIG_H__*/
