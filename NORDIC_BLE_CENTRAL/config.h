/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
/* History ORWL
 * ============================================================================
 * 26-Jun-16	Vikrant			Customization for ORWL Product
 * 25-Jul-16    Gupta	   		Review and update
 */

/**
 * @ingroup twi_master_with_twis_slave_example
 * @defgroup twi_master_with_twis_slave_example_config Example code configuration
 *
 * Configuration for the code presenting TWIS and TWI functionality
 * @{
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define EEPROM_SIM_SIZE          128  //!< Simulated EEPROM size
#define EEPROM_SIM_SEQ_WRITE_MAX 8    //!< Maximum number of bytes writable in one sequential access
#define EEPROM_SIM_ADDR          0x50 //!< Simulated EEPROM TWI address

#define TWI_SCL_M                3   //!< Master SCL pin
#define TWI_SDA_M                4   //!< Master SDA pin

#ifndef ORWL_CENTRAL
#define EEPROM_SIM_SCL_S         31   //!< Slave SCL pin
#define EEPROM_SIM_SDA_S         30   //!< Slave SDA pin
#else
#define EEPROM_SIM_SCL_S         5    //!< Slave SCL pin
#define EEPROM_SIM_SDA_S         6    //!< Slave SDA pin
#endif //ORWL_CENTRAL

#define EEPROM_SIM_TWIS_INST     1    //!< TWIS interface used by EEPROM simulator
#define MASTER_TWI_INST          0    //!< TWI interface used as a master accessing EEPROM memory

#define IN_LINE_PRINT_CNT        16  //<! Number of data bytes printed in single line

#endif //__CONFIG_H__
/** @} */
