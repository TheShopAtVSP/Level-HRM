/*
 * hal_imu.h
 *
 * Created: 6/4/2015 9:53:43 AM
 *  Author: matt
 */ 


#ifndef HAL_IMU_H_
#define HAL_IMU_H_

#include "../global.h"
#include "hal_twim.h"
//#include <stdlib.h>

//Select Motion Sensors:
//#define MPU9250
#define LSM6DS3
//#define LIS3MDL

#define FIFO_MODE
//#define POLLING_MODE

typedef __packed union {
	__packed struct {
		bool x: 1;
		bool y: 1;
		bool z: 1;
	};
	uint8_t flags;
} T_ACTIVE_3AXIS;

typedef __packed struct {
	T_ACTIVE_3AXIS accel;
	T_ACTIVE_3AXIS gyro;
	T_ACTIVE_3AXIS magnet;
} T_ACTIVE_SENSORS;

typedef __packed struct {
	int16_t	x;
	int16_t	y;
	int16_t	z;
	uint16_t m;	//magnitude of resultant vector
} T_3AXIS;

typedef __packed struct {
	T_3AXIS accel;
	T_3AXIS gyro;
	T_3AXIS mag;
} T_9AXIS;

#define IMU_FIFO_MASK	0x7F
typedef struct {
	T_ACTIVE_3AXIS active_axis;
	uint16_t rate;
	uint16_t range;
	uint8_t head;
	T_3AXIS fifo[IMU_FIFO_MASK+1];		//recent acceleration data holding buffer
}T_3AXIS_FIFO;

extern bool imu_debug;
//extern uint32_t getSystemTimeMs( void );

ret_code_t imu_init( bool );
uint16_t get_accel_fsr( void );
uint16_t get_gyro_fsr( void );
uint16_t get_compass_fsr( void );
uint16_t get_accel_sens( void );
float get_gyro_sens( void );
float get_compass_sens( void );
uint16_t get_accel_sample_rate( void );
uint16_t get_gyro_sample_rate( void );
uint16_t get_compass_sample_rate( void );
bool start_accel( void );
bool start_compass( void );
bool start_gyro( void );
bool stop_compass( void );
bool stop_gyro( void );
bool stop_imu( void );
bool get_accel_en( void );
bool get_gyro_en( void );
void imu_power_state( void );
void keep_accel_on( void );
void keep_gyro_on( void );
void keep_magnet_on( void );
ret_code_t enable_motion_wakeup( void );
ret_code_t collect_imu_data( void );
bool imu_slp_check( void );
void record_accel_on( void ); 
void record_accel_off( void ); 
void record_gyro_on( void ); 
void record_gyro_off( void ); 
void record_compass_on( void ); 
void record_compass_off( void ); 
ret_code_t test_imu( void );

#endif /* HAL_IMU_H_ */
