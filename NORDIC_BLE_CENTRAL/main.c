/**===========================================================================
 * @file main.c
 *
 * @brief This file contains the functions for implementing scanning and monitoring
 * functions in BLE
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

/* #define USE_TERMINAL_LOG */
#include "nrf_drv_twis.h"
#include <ble.h>
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#ifdef USE_TERMINAL_LOG
#include "nrf_delay.h"
#endif
/* Include local files for ORWL */
#include "orwl_config.h"
#include "eeprom_simulator.h"

#define mainCENTRAL_LINK_COUNT      1      /**< Number of central links used by the application.
					          When changing this number remember to adjust the RAM settings*/
#define mainPERIPHERAL_LINK_COUNT   0      /**< Number of peripheral links used by the application. When changing
					          this number remember to adjust the RAM settings*/
#ifdef DEBUG_VIA_UART
#define mainUART_TX_BUF_SIZE        (256)    /**< UART TX buffer size. */
#define mainUART_RX_BUF_SIZE        (256)    /**< UART RX buffer size. */
#endif /* DEBUG_VIA_UART */
#define mainAPP_TIMER_PRESCALER     (327)    /**< Value of the RTC1 PRESCALER register - 10ms TICK */
#define mainAPP_TIMER_OP_QUEUE_SIZE (3)                 /**< Size of timer operation queues. */
#define mainBROADCAST_INTERVAL_TIMER_DEFAULT	APP_TIMER_TICKS((suc_ble_ipcBROADCAST_REFRESH_IN_SEC*1000),\
								 mainAPP_TIMER_PRESCALER)
#define mainSCAN_INTERVAL           (0x06E0) /**< Determines scan interval in units of 0.625 millisecond. 0x6E0=1.1 sec */
#define mainSCAN_WINDOW             (0x0640) /**< Determines scan window in units of 0.625 millisecond. 0x0640 =1sec*/
#define mainSCAN_ACTIVE             (0)      /**< If 1, performe active scanning (scan requests). 0 - NON Active */
#define mainSCAN_SELECTIVE          (0)      /**< If 1, ignore unknown devices (non whitelisted). */
#define mainSCAN_TIMEOUT            (0x0000) /**< Timout when scanning. 0x0000 disables timeout. */

/***
 * Global variables
 */
uint8_t            ucMEepromSimMem[ suc_ble_ipcEEPROM_SIMUL_SIZE ] ;	/**< for simulating EEPROM Memory for I2C Bus Com. with SuC */
sucBleIpc_t      *pxSucBleIpcStrut ;				/**< IPC pointer to Simulated EEPROM */
bool               xMDataRxFromMcu ;				/**< Flag for indicating SuC has given the broadcast data to be observed*/
uint8_t            ucState = orwl_configIDLE;				/**< BLE state i.e, orwl_configIDLE or OBSERVING */
uint8_t            ucSubState = orwl_configPROXIMITY_LOCK;		/**< BLE sub state i.e, LOCK or UNLOCK */
#ifdef DEBUG_VIA_SWDIO_RTT
char               debug_rtt_buff[ 256 ];				/* Buffer is required for the system to support the RTT.*/
#endif /* DEBUG_VIA_SWDIO_RTT */

/**
 * Static variables
 */
uint8_t		   ucMBleRangeInfo = suc_ble_ipcBLE_OUT_RANGE;
static uint8_t     ucIndexScan ;

APP_TIMER_DEF( xMBroadcastTimerId );

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t xMScanParams =
  {
    .active      = mainSCAN_ACTIVE,
    .selective   = mainSCAN_SELECTIVE,
    .p_whitelist = NULL,
    .interval    = mainSCAN_INTERVAL,
    .window      = mainSCAN_WINDOW,
    .timeout     = mainSCAN_TIMEOUT
  };

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param line_num     Line number of the failing ASSERT call.
 * @param pFileName  File name of the failing ASSERT call.
 */
void vMainAssertNrfCallback(uint16_t usLineNum, const uint8_t * pFileName)
{
    app_error_handler(0xDEADBEEF, usLineNum, pFileName);
}
/*----------------------------------------------------------------------------*/
/**
 * @ingroup main_application
 * @defgroup main_application_funcs main application
 *
 * Internal functions required for the module to work.
 * @{
 */

/**@brief Function to start scanning.
 */
static void prvMainScanStart(void)
{
    uint32_t ulErrCode;
    ulErrCode = sd_ble_gap_scan_start(&xMScanParams);
    APP_ERROR_CHECK(ulErrCode);
    ulErrCode = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ulErrCode);
}
/*----------------------------------------------------------------------------*/

/**@brief Function to configure the GPIO output pin
 *
 * This function configures the GPIO pin as output to indicate the key Fob proximity state
 */
void vMainGpioConfig()
{
    nrf_gpio_cfg_output(orwl_configGPIO_OUTPUT_PIN);
    nrf_gpio_pin_clear(orwl_configGPIO_OUTPUT_PIN);
}
/*----------------------------------------------------------------------------*/

/**@brief Function to set or reset the GPIO output pin
 *
 *  This Function set or reset the GPIO output pin based on the Ble sub state
 *
 *  @param		ucSubState  BLE sub state
 */
void vMainToggleGpio(uint8_t ucSubState)
{
    switch( ucSubState )
    {
        case orwl_configPROXIMITY_LOCK :
            nrf_gpio_pin_set(orwl_configGPIO_OUTPUT_PIN);
            break;
        case orwl_configPROXIMITY_UNLOCK :
            nrf_gpio_pin_clear(orwl_configGPIO_OUTPUT_PIN);
            break;
    }
}
/*----------------------------------------------------------------------------*/

/**@brief Function to get the key Fob proximity status
 *
 *  This function returns the key Fob proximity status based on the scan status list
 *
 *  @return TRUE      At least one value in scan list is suc_ble_ipcBLE_IN_RANGE
 *  @return FALSE     All the three values in scan list is suc_ble_ipcBLE_OUT_RANGE
 *
 *  @see	      v_main_updateStatusToSuc()
 */
bool xMainKeyfobProximityStatusGet()
{
    uint8_t ucTestBlock[ suc_ble_ipcSCAN_RSP_STATUS_HISTORY ];
    memset( ucTestBlock, 0x00, suc_ble_ipcSCAN_RSP_STATUS_HISTORY );
    if(memcmp( pxSucBleIpcStrut->xMemoryRead.ucScanStatus,
	       ucTestBlock, suc_ble_ipcSCAN_RSP_STATUS_HISTORY ) == 0)
    {
        return false;
    }
    else
        return true;
}
/*----------------------------------------------------------------------------*/

/**@brief  Function for handling application timer expire handler for broadcast status update.
 *
 * @details This function is callback function to handle events from the application timer in for
 *          interval broadcast packet is define. This updates the dynamic status to SuC to take
 *          decision for KeyFOB BT to be in range or not in range from history of 4 period intervals
 *
 * @param   temp  Pointer to the data structure passed during creation of timer.
 */
void vMainBroadcastCheckHandler( void *temp )
{
    if(ucEeprom_simulatorErrorCheck())
    {
        ulEeprom_simulatorErrorGetAndClear();
    }
    else
    {
        pxSucBleIpcStrut->xMemoryRead.ucScanStatus[ucIndexScan++] = ucMBleRangeInfo ;
        /* Always assume that the KeyFOB is not in range for next interval
	   If KeyFOB data is received, then packet received event will change the status */
	ucMBleRangeInfo = suc_ble_ipcBLE_OUT_RANGE ;
	/*Schedule timer for updating the status */
	app_timer_start( xMBroadcastTimerId, mainBROADCAST_INTERVAL_TIMER_DEFAULT, NULL);
	debug_print( LEVEL_DEBUG,"In Timer Handler- Status Index: %d RangeStatus: %d\r\n",
		     ucIndexScan, pxSucBleIpcStrut->xMemoryRead.ucScanStatus[ ucIndexScan-1 ] );
	if( ucIndexScan >= suc_ble_ipcSCAN_RSP_STATUS_HISTORY )
	{
	    ucIndexScan = 0 ;
	}
	bool xGpioStatus = xMainKeyfobProximityStatusGet();
	if( xGpioStatus )
	{
	    ucSubState = orwl_configPROXIMITY_UNLOCK;
	}
	else
	{
	    ucSubState = orwl_configPROXIMITY_LOCK;
	}
    }
    /* keep track if the device is active */
    pxSucBleIpcStrut->xMemoryRead.ulHeartBeatInfo++;
}
/*----------------------------------------------------------------------------*/

/**@brief Function for checking whether the keyFOB is in range or not.
 *
 * @details This function tries to compare to all attributes and data SuC indicates to check on
 *          received broadcast packet. It will examine the BroadCast Data, RSSI Range and Broadcast
 *          data length that is valid. Data will be updated regularly by SuC on the simulated EEPROM
 *          memory
 *
 * @param ble_adv_report  Pointer to the data structure for advertisement/broadcast packet
 */
void vMainUpdateStatusToSuc( const ble_gap_evt_adv_report_t * pxBleAdvReport )
{
    /* Check if the broadcast is same as what need to be monitored by SuC */
    if( memcmp( pxBleAdvReport->data,
		pxSucBleIpcStrut->xMemoryRead.ucData,
		suc_ble_ipcBLE_ADV_CUR_SIZE )==0 )
    {
        /* At present, only broadcast data is verified. This will be extended to check RSSI Range
           when feature integration takes place */
        debug_print( LEVEL_DEBUG,"Received Packet for Set Broadcast Data\r\n" ) ;
        ucMBleRangeInfo = suc_ble_ipcBLE_IN_RANGE ;
    }
}
/*----------------------------------------------------------------------------*/

/**@brief Function for handling a BLE stack event
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received. In ORWL, we use perform only BLE observer activity to detect whether
 *          the requested keyFOB is in range or not. Except for Broadcast/Advertisment report, we
 *          are not interested for ORWL implementation
 *
 * @param   pBleEvt  Bluetooth stack event.
 */
static void ble_evt_handler( ble_evt_t * pBleEvt )
{
    const ble_gap_evt_t * pGapEvt = &pBleEvt->evt.gap_evt;
    switch ( pBleEvt->header.evt_id )
    {
        case BLE_GAP_EVT_ADV_REPORT:
            vMainUpdateStatusToSuc( &pGapEvt->params.adv_report );
            break;
	default:
            break;
    }
}
/*----------------------------------------------------------------------------*/

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void vMainBleStackInit( void )
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t xClockLfCfg = NRF_CLOCK_LFCLKSRC;
    /*Initialize the SoftDevice handler module. */
    SOFTDEVICE_HANDLER_INIT( &xClockLfCfg, NULL );
    ble_enable_params_t xBleEnableParams;
    err_code = softdevice_enable_get_default_config( mainCENTRAL_LINK_COUNT,
                                                     mainPERIPHERAL_LINK_COUNT,
                                                     &xBleEnableParams );
    APP_ERROR_CHECK( err_code );
    /*Check the ram settings against the used number of links*/
    CHECK_RAM_START_ADDR( mainCENTRAL_LINK_COUNT,mainPERIPHERAL_LINK_COUNT );
    /* Enable BLE stack.*/
    err_code = softdevice_enable( &xBleEnableParams );
    APP_ERROR_CHECK( err_code );
    /* Register with the SoftDevice handler module for BLE events.*/
    err_code = softdevice_ble_evt_handler_set( ble_evt_handler );
    APP_ERROR_CHECK( err_code );
}
/*----------------------------------------------------------------------------*/

#ifdef DEBUG_VIA_UART
void vMainUartErrorHandle( app_uart_evt_t * pEvent )
{
    if ( pEvent->evt_type == APP_UART_COMMUNICATION_ERROR )
    {
        APP_ERROR_HANDLER( pEvent->data.error_communication );
    }
    else if ( pEvent->evt_type == APP_UART_FIFO_ERROR )
    {
        APP_ERROR_HANDLER( pEvent->data.error_code );
    }
}
/*----------------------------------------------------------------------------*/

/**@brief Function for initializing the UART.
 */
static void uart_init( void )
{
    uint32_t ulErrCode;
    const app_uart_comm_params_t xCommParams =
    {
       .rx_pin_no    = RX_PIN_NUMBER,
       .tx_pin_no    = TX_PIN_NUMBER,
       .rts_pin_no   = RTS_PIN_NUMBER,
       .cts_pin_no   = CTS_PIN_NUMBER,
       .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
       .use_parity   = false,
       .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };
    APP_UART_FIFO_INIT( &xCommParams,
                        mainUART_RX_BUF_SIZE,
                        mainUART_TX_BUF_SIZE,
                        vMainUartErrorHandle,
                        APP_IRQ_PRIORITY_LOW,
                        ulErrCode );
    APP_ERROR_CHECK( ulErrCode );
}
/*----------------------------------------------------------------------------*/

#endif /* DEBUG_VIA_UART */

/** @brief Function for the Power manager.
 */
static void prvMainPowerManage( void )
{
    uint32_t ulErrCode = sd_app_evt_wait();
    APP_ERROR_CHECK( ulErrCode );
}
/*----------------------------------------------------------------------------*/
/** @} */

int main(void)
{
    ret_code_t xErrCode;
    APP_TIMER_INIT(mainAPP_TIMER_PRESCALER, mainAPP_TIMER_OP_QUEUE_SIZE, NULL);
    /* Create timer for checking the status index update for every
       3 seconds. Which is the broadcast interval. */
    xErrCode = app_timer_create( &xMBroadcastTimerId,
                                  APP_TIMER_MODE_SINGLE_SHOT,
                                  vMainBroadcastCheckHandler );
    APP_ERROR_CHECK(xErrCode);
#ifdef DEBUG_VIA_UART
    uart_init();
#endif /* DEBUG_VIA_UART */
    /* Initialize the SuC-BLE IPC data structure */
    pxSucBleIpcStrut = (sucBleIpc_t *)ucMEepromSimMem ;
    vMainBleStackInit();
    vMainGpioConfig();
    xErrCode = xEeprom_simulatorInit();
    APP_ERROR_CHECK(xErrCode);
    /* Start application timers. */
    xErrCode = app_timer_start( xMBroadcastTimerId,
				 mainBROADCAST_INTERVAL_TIMER_DEFAULT,
				 NULL );
    APP_ERROR_CHECK( xErrCode );
    while(1)
    {
        switch ( ucState )
        {
	    case orwl_configIDLE :
	        ucSubState = orwl_configPROXIMITY_LOCK;
		vMainToggleGpio( ucSubState );
		if( xMDataRxFromMcu )
		{
		    /* Start scanning for peripherals and initiate connection
		       with devices */
		    ucState = orwl_configOBSERVATION;
		    prvMainScanStart();
		}
		else
		{
		    memset( pxSucBleIpcStrut->xMemoryRead.ucScanStatus,
		   	    suc_ble_ipcBLE_OUT_RANGE,
			    suc_ble_ipcSCAN_RSP_STATUS_HISTORY );
		    memset( pxSucBleIpcStrut->xMemoryRead.ucData,
		    	    0xff, suc_ble_ipcBLE_ADV_MAX_SIZE );
	#ifdef USE_TERMINAL_LOG
	    debug_print( LEVEL_INFO,"state orwl_configIDLE\r\n\r\n" );
	#endif
	        }
		break;
	    case orwl_configOBSERVATION :
	  #ifdef USE_TERMINAL_LOG
	        debug_print( LEVEL_INFO,"state orwl_configOBSERVATION\r\n\r\n" );
	  #endif
	        switch ( ucSubState )
		{
		    case orwl_configPROXIMITY_LOCK :
			#ifdef USE_TERMINAL_LOG
			debug_print( LEVEL_INFO,"state orwl_configPROXIMITY_LOCK\r\n\r\n" );
			#endif
            vMainToggleGpio( ucSubState );
		        break;
	            case orwl_configPROXIMITY_UNLOCK :
		        #ifdef USE_TERMINAL_LOG
		        debug_print( LEVEL_INFO,"state orwl_configPROXIMITY_UNLOCK\r\n\r\n" );
		        #endif
		        vMainToggleGpio( ucSubState );
		        break;
	            default :
		        break;
	        }
		  break;
	    default :
	        break;
        }
	prvMainPowerManage();
#ifdef USE_TERMINAL_LOG
	nrf_delay_ms( 999 );
#endif
    }
}
/*----------------------------------------------------------------------------*/
