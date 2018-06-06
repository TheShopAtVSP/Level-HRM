/*
 * mpu9250.h
 *
 * Created: 7/30/2014 3:38:02 PM
 *  Author: RichKl
 */ 


#ifndef MPU9250_H_
#define MPU9250_H_

#define MPU9250

#include "inv_mpu.h"

//Configurables for the accelerometer
#define IMU_ADDR			0x68
#define DEF_GYRO_SAMP_RATE	50							//Hz
#define DEF_ACCEL_SAMP_RATE	50							//Hz
#define DEF_MAG_SAMP_RATE	50							//Hz
#define DEF_GYRO_FSR		1000						//+-1000 DPS
#define DEF_ACCEL_FSR		4							//+-4g
#define DEF_MAG_FSR			4800						//+-4800 uTesla
#define DEF_ACCEL_SENS		(32768/DEF_ACCEL_FSR)
#define DEF_GYRO_SENS		(32768.0f/DEF_GYRO_FSR)
#define DEF_MAG_SENS		(32768.0f/DEF_MAG_FSR)
#define DEF_LPF_CUTOFF		20							//Hz

T_IMU_ERROR_CODE mpu9250_board_init( void );
bool mpu9250_check( void );

#endif /* MUP9250_H_ */
