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

/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */


#include "TMP116.h"


uint8_t const tmp116_conf_reg_addr  = TMP116_REG_CONF;
uint8_t const tmp116_temp_reg_addr  = TMP116_REG_TEMP;



// Set default configuration of TMP116 - write 0000011000100100 to Conf register.
static uint16_t const default_config[] = { TMP116_REG_CONF, 0x0624 };
app_twi_transfer_t const tmp116_init_transfers[TMP116_INIT_TRANSFER_COUNT] =
{
    APP_TWI_WRITE(TMP116_ADDR, default_config, sizeof(default_config), 0)
};


// Set onshot configuration of TMP116 - write 000011100100100 to Conf register.
static uint16_t const oneshot_config[] = { TMP116_REG_CONF, 0x0E24 };
app_twi_transfer_t const tmp116_oneshot_transfers[TMP116_ONESHOT_TRANSFER_COUNT] =
{
    APP_TWI_WRITE(TMP116_ADDR, oneshot_config, sizeof(oneshot_config), 0)
};


