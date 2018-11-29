/*
 * STMicroelectronics lsm6ds3 i2c driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 */

#include	"lsm6ds3.h"

#ifdef LSM6DS3_I2C_INTERFACE

ret_code_t lsm6ds3_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply );
ret_code_t lsm6ds3_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data);

const lsm6ds3_comm_t lsm6ds3_read = (lsm6ds3_comm_t) &lsm6ds3_i2c_read;
const lsm6ds3_comm_t lsm6ds3_write = (lsm6ds3_comm_t) &lsm6ds3_i2c_write;

ret_code_t lsm6ds3_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply )
{
	T_TWIM_PACKET packet;
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t res;
		
	packet.i2c_id = LSM6DS3_I2C_ADDR;
	packet.reg_addr = reg_addr;
	packet.buffer = reply;	      //pointer to return data
	packet.length = len;
	res = hal_twim_read( &packet );
	if( res != NRF_SUCCESS ){
		if( prv_res != res )
		{
			app_trace_log(DEBUG_HIGH, "IMU Read Failed\r\n");
		}
	}
	prv_res = res;
	
	return res;
}

ret_code_t lsm6ds3_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	T_TWIM_PACKET packet;
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t res;
	
	packet.i2c_id = LSM6DS3_I2C_ADDR;
	packet.reg_addr = reg_addr;
	packet.buffer = data;	      //pointer to data to write
	packet.length = len;
	res = hal_twim_write( &packet );
	if( res != NRF_SUCCESS ){
		if( prv_res != res )
		{
			app_trace_log(DEBUG_HIGH, "IMU Write Failed\r\n");
		}
	}
	prv_res = res;
	
	return res;
}

#endif
