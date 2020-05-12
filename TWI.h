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

#ifndef TWI_H__
#define TWI_H__


#include "app_twi.h"


// 0x90 is the LM75B's address in the mbed Application Shield, it contains
// R/W bit and "nrf_drv_twi" (and consequently "app_twi") requires slave
// address without this bit, hence shifting.
#define TMP116_ADDR          (0x90U >> 1)

#define TMP116_REG_TEMP      	0x00
#define TMP116_REG_CONF      	0x01


// [use "/ 32" instead of ">> 5", as the result of right-shifting of a signed
//  type value is implementation-defined]
#define TMP116_GET_TEMPERATURE_VALUE(temp_hi, temp_lo) \
    ((((int16_t)temp_hi << 8) | temp_lo))


extern uint8_t const tmp116_conf_reg_addr;
extern uint8_t const tmp116_temp_reg_addr;
extern uint8_t const tmp116_tos_reg_addr;
extern uint8_t const tmp116_thyst_reg_addr;


#define TMP116_READ(p_reg_addr, p_buffer, byte_cnt) \
    APP_TWI_WRITE(TMP116_ADDR, p_reg_addr, 1,        APP_TWI_NO_STOP), \
    APP_TWI_READ (TMP116_ADDR, p_buffer,   byte_cnt, 0)

#define TMP116_READ_TEMP(p_buffer) \
    TMP116_READ(&tmp116_temp_reg_addr, p_buffer, 2)

#define TMP116_INIT_TRANSFER_COUNT 2
extern app_twi_transfer_t const tmp116_init_transfers[TMP116_INIT_TRANSFER_COUNT];


#endif // TWI_H__
