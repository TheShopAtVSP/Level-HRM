/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 *
 * Ported to C: 6/29/2015 2:25:07 PM
 */

#ifndef LSM6DS3_H_
#define LSM6DS3_H_

#include "../hal_imu.h"
#include "lis3mdl.h"

//Determine which communication interface to use, can't use both...
#define LSM6DS3_I2C_INTERFACE 
//#define LSM6DS3_SPI_INTERFACE

#define LSM6DS3_I2C_ADDR	0x6A

#define LSM6DS3_EN_BIT						0x01
#define LSM6DS3_DIS_BIT						0x00

//Configurables for the accelerometer
#define DEF_ACCEL_OSR		52							//52 HZ
#define DEF_GYRO_OSR		52							//52 HZ
#define DEF_GYRO_FSR		1000						//+-1000 DPS
#define DEF_ACCEL_FSR		4							//+-4g
#define DEF_ACCEL_SENS		(32768/DEF_ACCEL_FSR)
#define DEF_GYRO_SENS		(32768.0f/DEF_GYRO_FSR)

//#define HZ_TO_PERIOD_NSEC(hz)		(1000 * 1000 * 1000 / ((uint32_t)(hz)))

typedef enum {
	LSM6DS3_ACCEL = 0,
	LSM6DS3_GYRO,
	LSM6DS3_SIG_MOTION,
	LSM6DS3_STEP_COUNTER,
	LSM6DS3_STEP_DETECTOR,
	LSM6DS3_TILT,
	LSM6DS3_EXTERN_SENS,
	LSM6DS3_MOTION_WAKE,
	
	LSM6DS3_SENSORS_NUMB,		//Total Number of Sensor types
} T_LSM6DS3_SENSOR_TYPE;

#define ACCEL_EN_MASK			(1<<LSM6DS3_ACCEL)
#define GYRO_EN_MASK			(1<<LSM6DS3_GYRO)
#define SIG_MOTION_EN_MASK		(1<<LSM6DS3_SIG_MOTION)
#define STEP_COUNTER_EN_MASK	(1<<LSM6DS3_STEP_COUNTER)
#define STEP_DETECTOR_EN_MASK	(1<<LSM6DS3_STEP_DETECTOR)
#define TILT_EN_MASK			(1<<LSM6DS3_TILT)
#define EXTERN_SENS_EN_MASK		(1<<LSM6DS3_EXTERN_SENS)
#define MOTION_WAKE_EN_MASK		(1<<LSM6DS3_MOTION_WAKE)

#define EMBEDDED_FUNC_EN_MASK	(SIG_MOTION_EN_MASK|STEP_COUNTER_EN_MASK|STEP_DETECTOR_EN_MASK|TILT_EN_MASK|EXTERN_SENS_EN_MASK)

enum fifo_mode {
	BYPASS = 0,
	CONTINUOS,
};

#define DEF_ZERO					(0x00)

/* Sensitivity Acc */
#define SENSITIVITY_ACC_2G			(61)	/** ug/LSB */
#define SENSITIVITY_ACC_4G			(122)	/** ug/LSB */
#define SENSITIVITY_ACC_8G			(244)	/** ug/LSB */
#define SENSITIVITY_ACC_16G			(488)	/** ug/LSB */
/* Sensitivity Gyr */
#define SENSITIVITY_GYR_125			(437)	/** 10udps/LSB */
#define SENSITIVITY_GYR_245			(875)	/** 10udps/LSB */
#define SENSITIVITY_GYR_500			(1750)	/** 10udps/LSB */
#define SENSITIVITY_GYR_1000		(3500)	/** 10udps/LSB */
#define SENSITIVITY_GYR_2000		(7000)	/** 10udps/LSB */

#define LSM6DS3_RX_TX_MAX_LENGTH	(0xFE)

//Functions are defined for whichever comms module is included in project: I2C or SPI
typedef int (*lsm6ds3_comm_t) (uint8_t reg_addr, uint8_t length, uint8_t *data);	
typedef void (*save_fifo_func_ptr) ( uint8_t * );

int lsm6ds3_init_sensors( void );
uint16_t lsm6ds3_active_sensors (void );
int lsm6ds3_read_accel_reg( uint8_t *data );
int lsm6ds3_read_gyro_reg( uint8_t *data );
int lsm6ds3_read_sens_hub_reg( uint8_t *data );
uint16_t lsm6ds3_get_accel_fsr( void );
uint16_t lsm6ds3_get_gyro_fsr( void );
uint16_t lsm6ds3_get_accel_samp( void );
uint16_t lsm6ds3_get_gyro_samp( void );
uint16_t lsm6ds3_get_accel_sens( void );
float lsm6ds3_get_gyro_sens( void );
int lsm6ds3_set_odr( T_LSM6DS3_SENSOR_TYPE s_type, uint16_t * odr_hz);
int lsm6ds3_enable_sensors( T_LSM6DS3_SENSOR_TYPE s_type );
int lsm6ds3_disable_sensors( T_LSM6DS3_SENSOR_TYPE s_type );
int lsm6ds3_disable_all( void );
int lsm6ds3_wake_on_motion( bool turn_on );
int lsm6ds3_check_wom( void );
bool lsm6ds3_check_wakeup_evt( void );
int lsm6ds3_fifo_management( void );
int lsm6ds3_irq_management( void );
void gyro_save_init( save_fifo_func_ptr save_gyro_func );
void accel_save_init( save_fifo_func_ptr save_accel_func );
void extern_save_init( save_fifo_func_ptr save_extern_func );
void steps_save_init( save_fifo_func_ptr save_steps_func );

#endif /* LSM6DS3_H_ */
