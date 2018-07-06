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

status_code_t lsm6ds3_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply );
status_code_t lsm6ds3_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data);

const lsm6ds3_comm_t lsm6ds3_read = (lsm6ds3_comm_t) &lsm6ds3_i2c_read;
const lsm6ds3_comm_t lsm6ds3_write = (lsm6ds3_comm_t) &lsm6ds3_i2c_write;

status_code_t lsm6ds3_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply )
{
	T_TWIM_PACKET packet;
	status_code_t stat;
		
	packet.i2c_id = LSM6DS3_I2C_ADDR;
	packet.reg_addr = reg_addr;
	packet.buffer = reply;	      //pointer to return data
	packet.length = len;
	stat = hal_twim_read( &packet );
	if( stat != ERR_NONE ){
		if (imu_debug) app_trace_log("IMU Read Failed\r");
	}
	return stat;
}

status_code_t lsm6ds3_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	T_TWIM_PACKET packet;
	status_code_t stat;
	
	packet.i2c_id = LSM6DS3_I2C_ADDR;
	packet.reg_addr = reg_addr;
	packet.buffer = data;	      //pointer to data to write
	packet.length = len;
	stat = hal_twim_write( &packet );
	if( stat != ERR_NONE ){
		if (imu_debug) app_trace_log("IMU Write Failed\r");
	}
	return stat;
}

#endif
