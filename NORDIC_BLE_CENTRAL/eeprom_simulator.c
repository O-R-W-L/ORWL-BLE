 /**===========================================================================
 * @file eeprom_simulator.c
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
#include <string.h>
#include "config.h"
#include "eeprom_simulator.h"
#include "nrf_drv_twis.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "suc_ble_ipc.h"
#include "orwl_config.h"

/**
  * @ingroup eeprom_simulator
  * @defgroup eeprom_simulator_ivars EEPROM simulator internal variables
  *
  * Internal variables required for the module to work.
  * @{
  */

/**
 * @brief Current memory address
 *
 * Memory pointer for any operation that would be processed on memory array.
 */
static size_t xmaddr = 0;

/**
 * @brief Receive buffer
 *
 * Receiving buffer has to contain address and 8 bytes of data
 */
static uint8_t ucm_rxbuff[suc_ble_ipcBUFF_SIZE_I2C+1];

/**
 * @brief Internal error flag
 *
 * This flag is set if any communication error is detected.
 * It would be cleared by calling the @ref eeprom_simulator_error_get function.
 */
static bool merror_flag = FALSE;

/**
 * @brief TWIS instance
 *
 * TWIS driver instance used by this EEPROM simulator
 */
static const nrf_drv_twis_t xm_twis = NRF_DRV_TWIS_INSTANCE(EEPROM_SIM_TWIS_INST);

/** @} */
/**
 * @ingroup eeprom_simulator
 * @defgroup eeprom_simulator_ifunc EEPROM simulator internal functions
 *
 * Internal auxiliary functions
 * @{
 */
/**
 * @brief Set current address pointer
 *
 * Sets address for the next operation on the memory array
 *
 * @param xaddr Address to set
 */
static void prvEeprom_simulator_setAddr(size_t xaddr)
{
    xmaddr = xaddr;
}
/*----------------------------------------------------------------------------*/

/**
 *@brief Perform write operation on memory array
 *
 * Write single byte into memory array using @ref xmaddr as a write pointer.
 *
 * @param data Data to be written
 */
static void prvEeprom_simulatorWrite(uint8_t ucdata)
{
 /* Starting address is calculated from the offset of the
    eeprom from write only section */
    if(xmaddr >= sizeof(ucm_eeprom_sim_mem))
     {
          xmaddr = suc_ble_ipcIPC_WRITE_MEM_START;
     }
    ucm_eeprom_sim_mem[xmaddr++] = ucdata;
}
/*----------------------------------------------------------------------------*/

/**
 * @brief Start after WRITE command
 *
 * Function sets pointers for TWIS to receive data
 * WRITE command does not write directly to memory array.
 * Temporary receive buffer is used.
 * @sa ucm_rxbuff
 */
static void prvEeprom_simulatorWriteBegin(void)
{
    nrf_drv_twis_rx_prepare(&xm_twis, ucm_rxbuff, sizeof(ucm_rxbuff));
}
/*----------------------------------------------------------------------------*/

/**
 * @brief Finalize WRITE command
 *
 * Function should be called when write command is finished.
 * It sets memory pointer and write all received data to memory array
 */
static void prvEeprom_simulatorWriteEnd(size_t xcnt)
{
    if(xcnt > 0)
    {
        debug_print(LEVEL_DEBUG,"Received count for write:%d\n\r",xcnt) ;
        size_t xn;
        prvEeprom_simulator_setAddr(ucm_rxbuff[0]);
        for(xn=1; xn<xcnt; ++xn)
        {
            prvEeprom_simulatorWrite(ucm_rxbuff[xn]);
        }
        if(xcnt >= 2)
        {
            /* indicates that update of broadcast data is there.
               we need to update the broadcast data. */
	    memcpy(px_suc_ble_ipc_strut->xmemory_read.ucdata,
		   px_suc_ble_ipc_strut->xmemory_write.ucdata,
		   suc_ble_ipcBLE_ADV_MAX_SIZE) ;
            xm_data_rx_from_mcu = TRUE ;
        }
    }
}
/*----------------------------------------------------------------------------*/

/**
 * @brief Start after READ command
 *
 * Function sets pointers for TWIS to transmit data from current address to the
 * end of memory.
 */
static void prvEeprom_simulatorReadBegin(void)
{
    if(xmaddr >= sizeof(suc_ble_ipc_read_t))
    {
        xmaddr = suc_ble_ipcIPC_READ_MEM_START;
    }
    /* Size of the memory is restricted to only readonly section of the Simulated
       Memory  */
    nrf_drv_twis_tx_prepare(&xm_twis,
                            ucm_eeprom_sim_mem+xmaddr,
			                /*eeprom_simulatorREAD_BUF_SIZE*/
                            sizeof(suc_ble_ipc_read_t)-xmaddr);
}
/*----------------------------------------------------------------------------*/

/**
 * @brief Finalize READ command
 *
 * Function should be called when read command is finished.
 * It adjusts current xmaddr pointer.
 *
 * @param cnt Number of bytes readed
 */
static void prvEeprom_simulatorReadEnd(size_t xcnt)
{
    xmaddr += xcnt;
}
/*----------------------------------------------------------------------------*/

/**
 * @brief Event processing
 *
 *
 */
static void prvEeprom_simulatorEventHandler(nrf_drv_twis_evt_t const * const pxevent)
{
    switch(pxevent->type)
    {
        case TWIS_EVT_READ_REQ:
            if(pxevent->data.buf_req)
            {
                prvEeprom_simulatorReadBegin();
            }
            break;
        case TWIS_EVT_READ_DONE:
            prvEeprom_simulatorReadEnd(pxevent->data.tx_amount);
            break;
        case TWIS_EVT_WRITE_REQ:
            if(pxevent->data.buf_req)
            {
                prvEeprom_simulatorWriteBegin();
            }
            break;
        case TWIS_EVT_WRITE_DONE:
            prvEeprom_simulatorWriteEnd(pxevent->data.rx_amount);
            break;
        case TWIS_EVT_READ_ERROR:
        case TWIS_EVT_WRITE_ERROR:
        case TWIS_EVT_GENERAL_ERROR:
            merror_flag = TRUE;
            break;
        default:
            break;
    }
}
/*----------------------------------------------------------------------------*/

/** @brief Function to return the device information
 *
 *  This function reads device information (device mac id, s/w version) from
 *  the ble stack and stores it to the BLE IPC memory
 *
 *  @param xdev_info    Structure to store mac id and S/W version info
 */
void vEeprom_simulatorReadDeviceInfo( ble_device_info_t* xdev_info )
{
    uint32_t ulerr_code;
    ulerr_code = sd_ble_version_get( &xdev_info->xble_version );
    if( NRF_SUCCESS != ulerr_code )
    {
        debug_print( LEVEL_DEBUG,"err code version info : %d\n\r",ulerr_code ) ;
        memset( xdev_info, 0xff, sizeof( ble_device_info_t ) );
        return;
    }
    ulerr_code = sd_ble_gap_address_get( &xdev_info->xble_mac_id );
    if( NRF_SUCCESS != ulerr_code )
    {
        memset( xdev_info, 0xff, sizeof(ble_device_info_t) );
        return;
    }
}
/*----------------------------------------------------------------------------*/
/**  @{ */

/** @brief initialize eeprom_simulator module */
ret_code_t xEeprom_simulatorInit(void)
{
    ret_code_t xret;
    const nrf_drv_twis_config_t config =
    {
        .addr               = {EEPROM_SIM_ADDR, 0},
        .scl                = EEPROM_SIM_SCL_S,
        .sda                = EEPROM_SIM_SDA_S,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    /* Initialize data with pattern */
    xmaddr = 0;
    /* Init TWIS */
    do
    {
        xret = nrf_drv_twis_init( &xm_twis, &config, prvEeprom_simulatorEventHandler );
        if( NRF_SUCCESS != xret )
        {
            break;
        }
        nrf_drv_twis_enable( &xm_twis );
    }while(0);
    vEeprom_simulatorReadDeviceInfo( &px_suc_ble_ipc_strut->xmemory_read.xdevice_info );
    return xret;
}
/*----------------------------------------------------------------------------*/

/** @brief Function to return eeprom error status */
uint8_t ucEeprom_simulatorErrorCheck( void )
{
    return merror_flag;
}
/*----------------------------------------------------------------------------*/

/** @brief Function to handle eeprom error Status */
uint32_t ulEeprom_simulatorErrorGetAndClear( void )
{
    uint32_t ulret = nrf_drv_twis_error_get_and_clear( &xm_twis );
    sd_ble_gap_scan_stop();
    ucm_state = orwl_configIDLE;
    ucm_sub_state = orwl_configPROXIMITY_LOCK;
    xm_data_rx_from_mcu = FALSE;
    merror_flag = FALSE;
    return ulret;
}
/** @} */
/*----------------------------------------------------------------------------*/
