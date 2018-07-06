/*
 * hal_imu.c
 *
 * Created: 6/4/2015 9:39:33 AM
 *  Author: matt
 */ 
#include <string.h>
#include "hal_imu.h"
#include "../data_crunch.h"
#if defined(MPU9250)
#include "invensense/mpu9250.h"
#include "invensense/inv_mpu.h"
#elif defined(LSM6DS3)
#include "stmicro/lsm6ds3.h"
#if defined(LIS3MDL)
#include "stmicro/lis3mdl.h"
#endif
#endif

#if defined(FIFO_MODE) && defined(POLLING_MODE)
#error "Have to Pick one or the other!!!"
#endif

#define IMU_RES_TIMEOUT		1000

typedef enum {
	WAIT_FOR_MOTION = 0,
	IMU_ACTIVE = 1,
} T_IMU_POWER_STATE;

bool imu_debug = true;

#define ACCEL_PD_TO		4000		//motion timeout in milliseconds to wait before going into low power mode
#define GYRO_PD_TO		1500
#define MAG_PD_TO		4000

T_3AXIS_FIFO accel_buf = {
	.active_axis = {0},
	.rate = 1,
	.range = 1,
	.head = 0,
	.fifo = {0},
};
T_3AXIS_FIFO gyro_buf = {
	.active_axis = {0},
	.rate = 1,
	.range = 1,
	.head = 0,
	.fifo = {0},
};
T_3AXIS_FIFO magnet_buf = {
	.active_axis = {0},
	.rate = 1,
	.range = 1,
	.head = 0,
	.fifo = {0},
};
	
status_code_t read_avl_data( void );
void add_gyro_buffer( T_3AXIS * data );
void add_accel_buffer( T_3AXIS * data );
void add_magnet_buffer( T_3AXIS * data );
static bool check_wake_up_evt( void );

status_code_t imu_init( bool debug )
{
	status_code_t error = ERR_NONE;
	
	imu_debug = debug;
	
#if defined MPU9250
	// init mpu_9250
	error = (status_code_t) mpu9250_board_init();
#elif defined LSM6DS3
	error = (status_code_t) lsm6ds3_init_sensors();
	gyro_save_init( (save_fifo_func_ptr) &add_gyro_buffer );
	accel_save_init( (save_fifo_func_ptr) &add_accel_buffer );
#endif
	
	if ( error != ERR_NONE )
	{
		if (imu_debug) app_trace_log("imu_init: failed %02u\r", error);
		return error;
	}
	else {
		if (imu_debug) app_trace_log("imu_init: done\r");
	}
	
	return ERR_NONE;
}

///
//! \fn
/// \brief
/// \param
/// return
///
#define IMU_DATA_DEBUG	false
void add_gyro_buffer( T_3AXIS * data ) {
	gyro_buf.fifo[gyro_buf.head].x = data->x;
	gyro_buf.fifo[gyro_buf.head].y = data->y;
	gyro_buf.fifo[gyro_buf.head].z = data->z;
	calc_magnitude( &gyro_buf.fifo[gyro_buf.head] );
	
	if( IMU_DATA_DEBUG ) app_trace_log("(%04u)", gyro_buf.fifo[gyro_buf.head].m);
	
	gyro_buf.head = (gyro_buf.head+1)&IMU_FIFO_MASK;
}

///
//! \fn
/// \brief
/// \param
/// return
///
void add_accel_buffer( T_3AXIS * data ) {
	accel_buf.fifo[accel_buf.head].x = data->x;
	accel_buf.fifo[accel_buf.head].y = data->y;
	accel_buf.fifo[accel_buf.head].z = data->z;
	calc_magnitude( &accel_buf.fifo[accel_buf.head] );
	
	if( IMU_DATA_DEBUG ) app_trace_log("(%04u)", accel_buf.fifo[accel_buf.head].m);
	
	accel_buf.head = (accel_buf.head+1)&IMU_FIFO_MASK;
}

///
//! \fn
/// \brief
/// \param
/// return
///
void add_magnet_buffer( T_3AXIS * data ) {
	magnet_buf.fifo[magnet_buf.head].x = data->x;
	magnet_buf.fifo[magnet_buf.head].y = data->y;
	magnet_buf.fifo[magnet_buf.head].z = data->z;
	calc_magnitude( &magnet_buf.fifo[magnet_buf.head] );
	
	if( IMU_DATA_DEBUG ) app_trace_log("(%04u)", magnet_buf.fifo[magnet_buf.head].m);
	
	magnet_buf.head = (magnet_buf.head+1)&IMU_FIFO_MASK;
}

///
//! \fn
/// \brief
/// \param
/// return: 2, 4, 8, 16
///
uint16_t get_accel_fsr( void )
{
	uint8_t fsr = 0;
	
#if defined MPU9250
	if( mpu_get_accel_fsr( &fsr ) != 0 ) {
		fsr = DEF_ACCEL_FSR;
	}
#elif defined LSM6DS3
	fsr = lsm6ds3_get_accel_fsr();
#endif
	
	return fsr;
}

///
//! \fn
/// \brief
/// \param
/// return: 250, 500, 1000, 2000
///
uint16_t get_gyro_fsr( void )
{
	uint16_t fsr = 0;
	
#if defined MPU9250
	if( mpu_get_gyro_fsr( &fsr ) != 0 ) {
		fsr = DEF_GYRO_FSR;
	}
#elif defined LSM6DS3
	fsr = lsm6ds3_get_gyro_fsr();
#endif
	
	return fsr;
}

///
//! \fn
/// \brief
/// \param
/// return: 16384, 8192, 4096, 2048
///
uint16_t get_compass_fsr( void )
{
	uint16_t fsr = 0;
	
#if defined MPU9250
	if( mpu_get_compass_fsr( &fsr ) != 0 ) {
		fsr = DEF_MAG_FSR;
	}
#elif defined LIS3MDL
	fsr = lis3mdl_get_fsr();
#endif
	
	return fsr;
}

///
//! \fn
/// \brief
/// \param
/// return: 16384, 8192, 4096, 2048
///
uint16_t get_accel_sens( void )
{
	uint16_t scale = 0;
	
#if defined MPU9250
	if( mpu_get_accel_sens( &scale ) != 0 ) {
		scale = DEF_ACCEL_SENS;
	}
#elif defined LSM6DS3
	scale = lsm6ds3_get_accel_sens();
#endif
	
	return scale;
}

///
//! \fn
/// \brief
/// \param
/// return: 32768/(250, 500, 1000, or 2000)
///
float get_gyro_sens( void )
{
	float scale = 0.0;

#if defined MPU9250
	if( mpu_get_gyro_sens( &scale ) != 0 ) {
		scale = DEF_GYRO_SENS;
	}
#elif defined LSM6DS3
	scale = lsm6ds3_get_gyro_sens();
#endif
	
	return scale;
}

///
//! \fn
/// \brief
/// \param
/// return: 32768/(250, 500, 1000, or 2000)
///
float get_compass_sens( void )
{
	float scale = 0;
	
#if defined MPU9250
	scale = DEF_MAG_SENS;	//sensitivity of 1 uTesla
#elif defined LIS3MDL
	scale = lis3mdl_get_sens();	// 
#endif
	
	return scale;
}

///
//! \fn
/// \brief
/// \param
/// return: sample Rate in Hz
///
uint16_t get_accel_sample_rate( void )
{
	uint16_t rate = 0;
	
#if defined MPU9250
	if(mpu_get_sample_rate( &rate ) != 0 ) {
		//Not Sampling?
		rate = 0;
	}
#elif defined LSM6DS3
	rate = lsm6ds3_get_accel_samp();
#endif
	
	return rate;
}

///
//! \fn
/// \brief
/// \param
/// return: sample Rate in Hz
///
uint16_t get_gyro_sample_rate( void )
{
	uint16_t rate = 0;
	
#if defined MPU9250
	if(mpu_get_sample_rate( &rate ) != 0 ) {
		//Not Sampling?
		rate = 0;
	}
#elif defined LSM6DS3
	rate = lsm6ds3_get_gyro_samp();
#endif
	
	return rate;
}

///
//! \fn
/// \brief
/// \param
/// return: sample Rate in Hz
///
uint16_t get_compass_sample_rate( void )
{
	uint16_t rate = 0;

#if defined MPU9250
	if( mpu_get_compass_sample_rate( &rate ) != 0 ) {
		//Not Sampling?
		rate = 0;
	}
#elif defined LIS3MDL
	rate = lis3mdl_get_odr();
#endif

	return rate;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
status_code_t collect_imu_data(void)
{
	status_code_t read_res;
	
#if defined( POLLING_MODE )
	read_res = (status_code_t) read_avl_data();
#else
	read_res = (status_code_t) lsm6ds3_fifo_management();
#endif
	
	return read_res;
}

///
//! \fn
/// \brief
/// \param
/// return
///
status_code_t read_avl_data( void )
{
	status_code_t error = ERR_DEVICE_DISABLED;
	T_ACTIVE_SENSORS * act_axes = NULL;
	
	get_active_axes( act_axes );
	
	//Retrieve Accelerometer Data
	if( act_axes->accel.flags != 0 ) {	//At least 1 Axis is On
#if defined MPU9250				
		error = (status_code_t) mpu_get_accel_reg((short *)&accel_buf.fifo[accel_buf.head], 0);
#else			
		error = (status_code_t) lsm6ds3_read_accel_reg( (uint8_t *) &accel_buf.fifo[accel_buf.head] );
#endif			

		if( error == ERROR_NONE ) {
			calc_magnitude( &accel_buf.fifo[accel_buf.head] );
			accel_buf.head = (accel_buf.head+1)&IMU_FIFO_MASK;
		}
		else {
			error = ERR_IO_ERROR;
		}
	}
		
	//Retrieve Gyroscope Data
	if( act_axes->gyro.flags != 0 ) {
#if defined MPU9250				
		error = (status_code_t) mpu_get_gyro_reg((short *)&gyro_buf.fifo[gyro_buf.head], 0);
#else			
		error = (status_code_t) lsm6ds3_read_gyro_reg( (uint8_t *) &gyro_buf.fifo[gyro_buf.head] );
#endif			

		if( error == ERROR_NONE ) {
			calc_magnitude( &gyro_buf.fifo[gyro_buf.head] );	
			gyro_buf.head = (gyro_buf.head+1)&IMU_FIFO_MASK;
		}
		else {
			error = ERR_IO_ERROR;
		}
	}
		
	//Retrieve Magnetometer Data
	if( act_axes->magnet.flags != 0 ) {
#if defined MPU9250				
		error = (status_code_t) mpu_get_compass_reg((short *)&magnet_buf.fifo[magnet_buf.head], 0);		
#else		
		error = (status_code_t) lsm6ds3_read_sens_hub_reg( (uint8_t *) &magnet_buf.fifo[magnet_buf.head] );
#endif			

		if( error == ERROR_NONE ) {
			calc_magnitude( &magnet_buf.fifo[magnet_buf.head] );
			magnet_buf.head = (magnet_buf.head+1)&IMU_FIFO_MASK;
		}
		else {
			error = ERR_IO_ERROR;
		}
	}
	
	return error;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool start_accel( void )
{
	status_code_t error = ERR_NONE;

#if defined( MPU9250 )
	unsigned char act_sensors = 0;
	mpu_get_power_state( &act_sensors );
	if( act_sensors == 0 ) {
		mpu_set_int_enable( 1 );				//Turn on Data Ready Interrupt
		mpu_lp_motion_interrupt(1, 1, 0);		//0 sample rate restores active mode
	}
	error = (status_code_t) mpu_set_sensors(act_sensors|INV_XYZ_ACCEL);
#elif defined( LSM6DS3 )
	error = (status_code_t) lsm6ds3_wake_on_motion( LSM6DS3_DIS_BIT );
	error = (status_code_t) lsm6ds3_enable_sensors( LSM6DS3_ACCEL );
#else
	//no accel defined
	#error "No accelerometer defined!!!"
#endif

	if( error != ERR_NONE ) {
		accel_buf.active_axis.flags = 0;
		if( imu_debug ) app_trace_log("Start Accel Err: %01u\r", error);
		return 0;
	}
	accel_buf.active_axis.x = 1;
	accel_buf.active_axis.y = 1;
	accel_buf.active_axis.z = 1;
	accel_buf.rate = get_accel_sample_rate();
	accel_buf.range = get_accel_fsr();

	return 1;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool start_gyro( void )
{
	T_IMU_ERROR_CODE error = ERROR_NONE;
	
#if defined MPU9250
	uint8_t act_sensors;

	mpu_get_power_state( &act_sensors );
	if( act_sensors == 0 ) {
		mpu_set_int_enable( 1 );			//Turn on Data Ready Interrupt
		mpu_lp_motion_interrupt(1, 1, 0);	//0 sample rate restores active mode
	}
	mpu_set_sensors(act_sensors|INV_XYZ_GYRO);
#elif defined LSM6DS3
	error = (T_IMU_ERROR_CODE) lsm6ds3_enable_sensors( LSM6DS3_GYRO );
#endif

	if( error != ERROR_NONE ) {
		gyro_buf.active_axis.flags = 0;
		if( imu_debug ) app_trace_log("Start Gyro Err: %01u\r", error);
		return 0;
	}
	gyro_buf.active_axis.x = 1;
	gyro_buf.active_axis.y = 1;
	gyro_buf.active_axis.z = 1;
	gyro_buf.rate = get_gyro_sample_rate();
	gyro_buf.range = get_gyro_fsr();

	return 1;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool start_compass( void )
{
	T_IMU_ERROR_CODE error = ERROR_NONE;
	
#if defined MPU9250
	uint8_t act_sensors;

	mpu_get_power_state( &act_sensors );
	if( act_sensors == 0 ) {
		mpu_set_int_enable( 1 );			//Turn on Data Ready Interrupt
		mpu_lp_motion_interrupt(1, 1, 0);	//0 sample rate restores active mode
	}
	mpu_set_sensors(act_sensors|INV_XYZ_COMPASS);
#elif defined LSM6DS3
	error = (T_IMU_ERROR_CODE) lsm6ds3_enable_sensors( LSM6DS3_EXTERN_SENS );
#endif

	if( error != ERROR_NONE ) {
		magnet_buf.active_axis.flags = 0;
		if( imu_debug ) app_trace_log("Start Compass Err: %01u\r", error);
		return 0;
	}
	magnet_buf.active_axis.x = 1;
	magnet_buf.active_axis.y = 1;
	magnet_buf.active_axis.z = 1;
	magnet_buf.rate = get_compass_sample_rate();
	magnet_buf.range = get_compass_fsr();

	return 1;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool stop_gyro( void )
{
	T_IMU_ERROR_CODE error = ERROR_NONE;
	
#if defined MPU9250
	uint8_t act_sensors;
	
	mpu_get_power_state( &act_sensors );
	if( (act_sensors&INV_XYZ_GYRO) != 0 ) {
		mpu_set_sensors((~INV_XYZ_GYRO)&act_sensors);
	}
#elif defined LSM6DS3
	error = (T_IMU_ERROR_CODE) lsm6ds3_disable_sensors( LSM6DS3_GYRO );
#endif

	gyro_buf.active_axis.flags = 0;
	gyro_buf.rate = 0;
	
	if( error != ERROR_NONE ) {
		if( imu_debug ) app_trace_log("Stop Gyro Err: %01u\r", error);
		return 0;
	}

	return 1;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool stop_compass( void )
{
	T_IMU_ERROR_CODE error = ERROR_NONE;
	
#if defined MPU9250
	uint8_t act_sensors;
	
	mpu_get_power_state( &act_sensors );
	if( (act_sensors&INV_XYZ_COMPASS) != 0 ) {
		mpu_set_sensors((~INV_XYZ_COMPASS)&act_sensors);
	}
#elif defined LSM6DS3
	error = (T_IMU_ERROR_CODE) lsm6ds3_disable_sensors( LSM6DS3_EXTERN_SENS );
#endif

	magnet_buf.active_axis.flags = 0;
	magnet_buf.rate = 0;
	
	if( error != ERROR_NONE ) {
		if( imu_debug ) app_trace_log("Stop Compass Err: %01u\r", error);
		return 0;
	}

	return 1;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool stop_imu( void )
{
	T_IMU_ERROR_CODE error = ERROR_NONE;
	
#if defined MPU9250
	mpu_set_sensors(0);
#elif defined LSM6DS3
	error = (T_IMU_ERROR_CODE) lsm6ds3_disable_all();
#endif

	accel_buf.active_axis.flags = 0;
	accel_buf.rate = 0;
	
	if( error != ERROR_NONE ) {
		if( imu_debug ) app_trace_log("Stop IMU Err: %01u\r", error);
		return 0;
	}

	return 1;
}

///
//! \fn
/// \brief
/// \param
/// return
///
int enable_motion_wakeup( void )
{
	int error;
#if defined MPU9250
	error = mpu_lp_motion_interrupt(75, 1, 10);	//Wakeup thres in mg, time(not used by mpu_9250), sample rate
#else
	error = lsm6ds3_wake_on_motion( LSM6DS3_EN_BIT );			
#endif
	
	if( error != ERROR_NONE ) {
		if( imu_debug ) app_trace_log("WOM Err: %01u\r", error);
		return 0;
	}

	return error;
}

///
//! \fn
/// \brief
/// \param
/// return
///
void get_active_axes( T_ACTIVE_SENSORS * act_axes )
{		
	act_axes->accel.flags = 0;
	act_axes->gyro.flags = 0;
	act_axes->magnet.flags = 0;
		
#if defined MPU9250
	unsigned char act_sensors = 0;
	mpu_get_power_state( &act_sensors );
	
	if( (act_sensors&INV_XYZ_ACCEL) > 0 ) {
		act_axes->accel.x = 1;
		act_axes->accel.y = 1;
		act_axes->accel.z = 1;
	}

	if( (act_sensors&INV_XYZ_COMPASS) > 0) {
		act_axes->magnet.x = 1;
		act_axes->magnet.y = 1;
		act_axes->magnet.z = 1;
	}

	if( (act_sensors&INV_X_GYRO) > 0) {
		act_axes->gyro.x = 1;
	}
	if( (act_sensors&INV_Y_GYRO) > 0) {
		act_axes->gyro.y = 1;
	}
	if( (act_sensors&INV_Z_GYRO) > 0) {
		act_axes->gyro.z = 1;
	}
#else
	uint16_t act_sensors = lsm6ds3_active_sensors();
	
	if( (act_sensors&GYRO_EN_MASK) > 0 ) {
		act_axes->gyro.x = 1;
		act_axes->gyro.y = 1;
		act_axes->gyro.z = 1;
	}
	
	if( (act_sensors&EXTERN_SENS_EN_MASK) > 0 ) {
		act_axes->magnet.x = 1;
		act_axes->magnet.y = 1;
		act_axes->magnet.z = 1;
	}
	 
	act_sensors &= ~(GYRO_EN_MASK|EXTERN_SENS_EN_MASK);	//clear the gyro and magnet enabled bit, if any other bits are set, then the accelerometer is On
	
	if( (act_sensors > 0) && (act_sensors&MOTION_WAKE_EN_MASK) == 0) {
		//accel on and not in sleep mode
		act_axes->accel.x = 1;
		act_axes->accel.y = 1;
		act_axes->accel.z = 1;
	}
#endif

}

///
//! \fn
/// \brief
/// \param
/// return
///
bool imu_slp_check( void ) 
{
	bool res = true;
	
#if defined MPU9250
	res = mpu9250_check();
#elif defined LSM6DS3
	if( lsm6ds3_check_wom() != ERR_NONE ) {
		if( enable_motion_wakeup() != ERR_NONE ) {
			if( imu_debug ) app_trace_log("IMU WOM Error\r");
			return false;
		}
	}
#endif

	return res;
}

///
//! \fn
/// \brief
/// \param
/// return
///
static bool check_wake_up_evt( void ) 
{
	bool wakeup = true;
	
#if defined MPU9250
#else
	wakeup = lsm6ds3_check_wakeup_evt();		
#endif
	
	return wakeup;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
volatile TTASK_TIMER accelPowerDownTimer = { .timer = 0, .period = 0 };
volatile TTASK_TIMER gyroPowerDownTimer= { .timer = 0, .period = 0 };
volatile TTASK_TIMER magPowerDownTimer = { .timer = 0, .period = 0 };
T_ACTIVE_SENSORS lock = { .accel.flags = 0, .gyro.flags = 0, .magnet.flags = 0 };
void imu_power_state( void )
{
	static T_IMU_POWER_STATE imu_mode_sv = WAIT_FOR_MOTION; 
	static uint8_t retries[3] = {0};
	T_ACTIVE_SENSORS act_sensors;
	
	switch ( imu_mode_sv ) {
			
		case WAIT_FOR_MOTION:
			if( check_wake_up_evt() ) {
				if( start_accel() == true ) {
					keep_accel_on();	//Keep Accel On for at least X more seconds to determine motion...
					if( imu_debug ) app_trace_log("IMU Motion Wakeup!!!\r");
					
					imu_mode_sv = IMU_ACTIVE;
				}
				else {
					if( imu_debug ) app_trace_log("Motion Wakeup Failure!!!\r");
				}
			}
			else if( imu_slp_check() != true ) {
				//imu is not in it's wake on motion mode
				if( enable_motion_wakeup() == ERROR_NONE ) {
					if( imu_debug ) app_trace_log("IMU Sleep Mode\r");
				}
			}
			else {
				//imu monitoring for motion, but there has not been a wake event... so why are we here?
				if( imu_debug ) app_trace_log("IMU Asleep???\r");
			}
			break;
			
		case IMU_ACTIVE:
			//turn Gyro and Mag on as needed
			//If nothing needs to be on, go back to waiting for motion
		
			//Get the On/Off status of the 9 Axis IMU...
			get_active_axes( &act_sensors );
			
			if( task_time(accelPowerDownTimer) && lock.accel.flags == 0 ) {	
				//Time to enter back into low power wait for motion mode. Make sure everything is powered down		
				restart_timer( gyroPowerDownTimer, 0 );	//turn Timer Off
				retries[G] = 0;
				if( act_sensors.gyro.flags != 0 )	{
					if( imu_debug ) app_trace_log("Gyro Off\r");
					stop_gyro();
				}
				
				restart_timer( magPowerDownTimer, 0 );	//turn Timer Off
				retries[M] = 0;
				if( act_sensors.magnet.flags != 0 )	{
					if( imu_debug ) app_trace_log("Compass Off\r");
					stop_compass();
				}
				
				//turn on wake on motion interrupt
				if( enable_motion_wakeup() == ERROR_NONE ) {
					if( imu_debug ) app_trace_log("IMU Sleep\r");
			
					imu_mode_sv = WAIT_FOR_MOTION;
				}
			}
			else {
				//Handle the turning on/off of the Gyroscope	
				if( act_sensors.gyro.flags == 0 ) {
					//sensor is currently Off
					if( (gyroPowerDownTimer.period > 0 || lock.gyro.flags != 0) && retries[G] < 3 ) {
						//Something has requested it be on either temporarily or for good
						if( imu_debug ) app_trace_log("Gyro On\r");
						start_gyro();
						retries[G]++;
					}
				}
				else {
					//Sensor is On, wait for the On request to be removed
					if( task_time(gyroPowerDownTimer) && lock.gyro.flags == 0 ) {
						restart_timer( gyroPowerDownTimer, 0 );	//turn Timer Off
						retries[G] = 0;
						if( imu_debug ) app_trace_log("Gyro Off\r");
						stop_gyro();
					}
				}
				
				//Handle the turning on/off of the Magnetometer	
				if( act_sensors.magnet.flags == 0 ) {
					//sensor is currently Off
					if( (magPowerDownTimer.period > 0 || lock.magnet.flags != 0) && retries[M] < 3 ) {
						//Something has requested it be on either temporarily or for good
						if( imu_debug ) app_trace_log("Compass On\r");
						start_compass();
						retries[M]++;
					}
				}
				else {
					//Sensor is On, wait for the On request to be removed
					if( task_time(magPowerDownTimer) && lock.magnet.flags == 0 ) {
						restart_timer( magPowerDownTimer, 0 );	//turn Timer Off
						retries[M] = 0;
						if( imu_debug ) app_trace_log("Compass Off\r");
						stop_compass();
					}
				}
			}
			
			break;
						
		default:
			imu_mode_sv = WAIT_FOR_MOTION; 
			break;
			
	}
}

//Reset timer. Temporarily keeps accelerometer on.
void keep_accel_on( void )
{
	restart_timer( accelPowerDownTimer, ACCEL_PD_TO );
}

//Reset timer. Temporarily keeps gyrometer on.
void keep_gyro_on( void )
{
	restart_timer( gyroPowerDownTimer, GYRO_PD_TO );
}

//Reset timer. Temporarily keeps magnetometer on.
void keep_magnet_on( void )
{
	restart_timer( magPowerDownTimer, MAG_PD_TO );
}

//Force Accelerometer to stay On
void lock_accel( bool lock_on )
{
	if( lock_on == true ) {
		lock.accel.x = 1;
		lock.accel.y = 1;
		lock.accel.z = 1;
	}
	else {
		lock.accel.flags = 0;
	}
}
