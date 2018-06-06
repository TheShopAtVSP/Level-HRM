/*
 * hal_imu.c
 *
 * Created: 6/4/2015 9:39:33 AM
 *  Author: matt
 */ 
#include <string.h>
#include "hal_imu.h"
#include "reports.h"
#include "../data_crunch.h"
#include "nrf_drv_gpiote.h"
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

// some HRM test shit globals
extern uint16_t accel_x;
extern uint16_t accel_y;
extern uint16_t accel_z;
	
ret_code_t read_avl_data( void );
void add_gyro_buffer( T_3AXIS * data );
void add_accel_buffer( T_3AXIS * data );
void add_magnet_buffer( T_3AXIS * data );
static bool check_wake_up_evt( void );
static void lock_accel( bool );
static void lock_gyro( bool );
static void lock_magnet( bool );
static void get_active_axes( T_ACTIVE_SENSORS * );

ret_code_t imu_init( bool debug )
{
	ret_code_t err = NRF_SUCCESS;
	
	imu_debug = debug;
	
	accel_buf.head = 0;
	gyro_buf.head = 0;
	magnet_buf.head = 0;
	
#if defined MPU9250
	// init mpu_9250
	err = mpu9250_board_init();
#elif defined LSM6DS3
	err = lsm6ds3_init_sensors();
	gyro_save_init( (save_fifo_func_ptr) &add_gyro_buffer );
	accel_save_init( (save_fifo_func_ptr) &add_accel_buffer );
#endif
	
	//stop_imu();
	
	if ( err != NRF_SUCCESS )
	{
		if (imu_debug) app_trace_log(DEBUG_MED, "imu_init: failed %02u\r", err);
		return err;
	}
	else {
		if (imu_debug) app_trace_log(DEBUG_LOW, "imu_init: done\r");
	}
	
	return NRF_SUCCESS;
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
	
	if( IMU_DATA_DEBUG ) app_trace_log(DEBUG_LOW, "(%04u)", gyro_buf.fifo[gyro_buf.head].m);
	
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
	accel_x = data->x;
	accel_buf.fifo[accel_buf.head].y = data->y;
	accel_y = data->y;
	accel_buf.fifo[accel_buf.head].z = data->z;
	accel_z = data->z;
	calc_magnitude( &accel_buf.fifo[accel_buf.head] );
	
	if( IMU_DATA_DEBUG ) app_trace_log(DEBUG_LOW, "(%04u)", accel_buf.fifo[accel_buf.head].m);
	
	accel_buf.head = (accel_buf.head+1)&IMU_FIFO_MASK;
	lock_accel(1);
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
	
	if( IMU_DATA_DEBUG ) app_trace_log(DEBUG_LOW, "(%04u)", magnet_buf.fifo[magnet_buf.head].m);
	
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
ret_code_t collect_imu_data(void)
{
	ret_code_t res;
	
#if defined( POLLING_MODE )
	res = read_avl_data();
#else
	res = lsm6ds3_fifo_management();
#endif
	
	return res;
}

///
//! \fn
/// \brief
/// \param
/// return
///
ret_code_t read_avl_data( void )
{
	ret_code_t err = NRF_ERROR_NOT_SUPPORTED;
	T_ACTIVE_SENSORS * act_axes = NULL;
	
	get_active_axes( act_axes );
	
	//Retrieve Accelerometer Data
	if( act_axes->accel.flags != 0 ) {	//At least 1 Axis is On
#if defined MPU9250				
		err = mpu_get_accel_reg((short *)&accel_buf.fifo[accel_buf.head], 0);
#else			
		err = lsm6ds3_read_accel_reg( (uint8_t *) &accel_buf.fifo[accel_buf.head] );
#endif			

		if( err == NRF_SUCCESS) {
			calc_magnitude( &accel_buf.fifo[accel_buf.head] );
			accel_buf.head = (accel_buf.head+1)&IMU_FIFO_MASK;
		}
		else {
			err = NRF_ERROR_NOT_FOUND;
		}
	}
		
	//Retrieve Gyroscope Data
	if( act_axes->gyro.flags != 0 ) {
#if defined MPU9250				
		err = mpu_get_gyro_reg((short *)&gyro_buf.fifo[gyro_buf.head], 0);
#else			
		err = lsm6ds3_read_gyro_reg( (uint8_t *) &gyro_buf.fifo[gyro_buf.head] );
#endif			

		if( err == NRF_SUCCESS ) {
			calc_magnitude( &gyro_buf.fifo[gyro_buf.head] );	
			gyro_buf.head = (gyro_buf.head+1)&IMU_FIFO_MASK;
		}
		else {
			err = NRF_ERROR_NOT_FOUND;
		}
	}
		
	//Retrieve Magnetometer Data
	if( act_axes->magnet.flags != 0 ) {
#if defined MPU9250				
		err = (status_code_t) mpu_get_compass_reg((short *)&magnet_buf.fifo[magnet_buf.head], 0);		
#else		
		err = lsm6ds3_read_sens_hub_reg( (uint8_t *) &magnet_buf.fifo[magnet_buf.head] );
#endif			

		if( err == NRF_SUCCESS ) {
			calc_magnitude( &magnet_buf.fifo[magnet_buf.head] );
			magnet_buf.head = (magnet_buf.head+1)&IMU_FIFO_MASK;
		}
		else {
			err = NRF_ERROR_NOT_FOUND;
		}
	}
	
	return err;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool start_accel( void )
{
	static ret_code_t prv_err = NRF_SUCCESS;
	ret_code_t err = NRF_SUCCESS;

#if defined( MPU9250 )
	unsigned char act_sensors = 0;
	mpu_get_power_state( &act_sensors );
	if( act_sensors == 0 ) {
		mpu_set_int_enable( 1 );				//Turn on Data Ready Interrupt
		mpu_lp_motion_interrupt(1, 1, 0);		//0 sample rate restores active mode
	}
	error = (status_code_t) mpu_set_sensors(act_sensors|INV_XYZ_ACCEL);
#elif defined( LSM6DS3 )
	err = lsm6ds3_wake_on_motion( LSM6DS3_DIS_BIT );
	err = lsm6ds3_enable_sensors( LSM6DS3_ACCEL );
#else
	//no accel defined
	#error "No accelerometer defined!!!"
#endif

	if( err != NRF_SUCCESS ) {
		accel_buf.active_axis.flags = 0;
		if( prv_err != err ) app_trace_log(DEBUG_HIGH, "Start Accel Err: %01u\r", err);
		prv_err = err;
		return false;
	}
	prv_err = err;
	
	accel_buf.active_axis.x = 1;
	accel_buf.active_axis.y = 1;
	accel_buf.active_axis.z = 1;
	accel_buf.rate = get_accel_sample_rate();
	accel_buf.range = get_accel_fsr();

	return true;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool start_gyro( void )
{
	static ret_code_t prv_err = NRF_SUCCESS;
	ret_code_t err = NRF_SUCCESS;
	
#if defined MPU9250
	uint8_t act_sensors;

	mpu_get_power_state( &act_sensors );
	if( act_sensors == 0 ) {
		mpu_set_int_enable( 1 );			//Turn on Data Ready Interrupt
		mpu_lp_motion_interrupt(1, 1, 0);	//0 sample rate restores active mode
	}
	mpu_set_sensors(act_sensors|INV_XYZ_GYRO);
#elif defined LSM6DS3
	err = lsm6ds3_enable_sensors( LSM6DS3_GYRO );
#endif

	if( err != NRF_SUCCESS ) {
		gyro_buf.active_axis.flags = 0;
		if( prv_err != err ) app_trace_log(DEBUG_MED, "Start Gyro Err: %01u\r", err);
		prv_err = err;
		return false;
	}
	prv_err = err;
	
	gyro_buf.active_axis.x = 1;
	gyro_buf.active_axis.y = 1;
	gyro_buf.active_axis.z = 1;
	gyro_buf.rate = get_gyro_sample_rate();
	gyro_buf.range = get_gyro_fsr();

	return true;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool start_compass( void )
{
	static ret_code_t prv_err = NRF_SUCCESS;
	ret_code_t err = NRF_SUCCESS;
	
#if defined MPU9250
	uint8_t act_sensors;

	mpu_get_power_state( &act_sensors );
	if( act_sensors == 0 ) {
		mpu_set_int_enable( 1 );			//Turn on Data Ready Interrupt
		mpu_lp_motion_interrupt(1, 1, 0);	//0 sample rate restores active mode
	}
	mpu_set_sensors(act_sensors|INV_XYZ_COMPASS);
#elif defined LSM6DS3
	err = lsm6ds3_enable_sensors( LSM6DS3_EXTERN_SENS );
#endif

	if( err != NRF_SUCCESS ) {
		magnet_buf.active_axis.flags = 0;
		if( prv_err != err ) app_trace_log(DEBUG_MED, "Start Compass Err: %01u\r", err);
		prv_err = err;
		return false;
	}
	prv_err = err;
	
	magnet_buf.active_axis.x = 1;
	magnet_buf.active_axis.y = 1;
	magnet_buf.active_axis.z = 1;
	magnet_buf.rate = get_compass_sample_rate();
	magnet_buf.range = get_compass_fsr();

	return true;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool stop_gyro( void )
{
	static ret_code_t prv_err = NRF_SUCCESS;
	ret_code_t err = NRF_SUCCESS;
	
#if defined MPU9250
	uint8_t act_sensors;
	
	mpu_get_power_state( &act_sensors );
	if( (act_sensors&INV_XYZ_GYRO) != 0 ) {
		mpu_set_sensors((~INV_XYZ_GYRO)&act_sensors);
	}
#elif defined LSM6DS3
	err = lsm6ds3_disable_sensors( LSM6DS3_GYRO );
#endif

	gyro_buf.active_axis.flags = 0;
	gyro_buf.rate = 0;
	
	if( err != NRF_SUCCESS ) {
		if( prv_err != err ) app_trace_log(DEBUG_MED, "Stop Gyro Err: %01u\r", err);
		prv_err = err;
		return false;
	}
	prv_err = err;

	return true;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool stop_compass( void )
{
	static ret_code_t prv_err = NRF_SUCCESS;
	ret_code_t err = NRF_SUCCESS;
	
#if defined MPU9250
	uint8_t act_sensors;
	
	mpu_get_power_state( &act_sensors );
	if( (act_sensors&INV_XYZ_COMPASS) != 0 ) {
		mpu_set_sensors((~INV_XYZ_COMPASS)&act_sensors);
	}
#elif defined LSM6DS3
	err = lsm6ds3_disable_sensors( LSM6DS3_EXTERN_SENS );
#endif

	magnet_buf.active_axis.flags = 0;
	magnet_buf.rate = 0;
	
	if( err != NRF_SUCCESS) {
		if( prv_err != err ) app_trace_log(DEBUG_MED, "Stop Compass Err: %01u\r", err);
		prv_err = err;
		return false;
	}
	prv_err = err;

	return true;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool stop_imu( void )
{
	static ret_code_t prv_err = NRF_SUCCESS;
	ret_code_t err = NRF_SUCCESS;
	
#if defined MPU9250
	mpu_set_sensors(0);
#elif defined LSM6DS3
	err = lsm6ds3_disable_all();
#endif

	accel_buf.active_axis.flags = 0;
	accel_buf.rate = 0;
	
	if( err != NRF_SUCCESS ) {
		if( prv_err != err ) app_trace_log(DEBUG_MED, "Stop IMU Err: %01u\r", err);
		prv_err = err;
		return false;
	}
	prv_err = err;

	return true;
}

///
//! \fn
/// \brief
/// \param
/// return
///
ret_code_t enable_motion_wakeup( void )
{
	ret_code_t err;
#if defined MPU9250
	err = mpu_lp_motion_interrupt(75, 1, 10);	//Wakeup thres in mg, time(not used by mpu_9250), sample rate
#else
	err = lsm6ds3_wake_on_motion( LSM6DS3_EN_BIT );			
#endif
	
	if( err != NRF_SUCCESS ) {
		app_trace_log(DEBUG_LOW, "WOM Err: %01u\r", err);
	}

	return err;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool get_accel_en( void )
{
	#if defined MPU9250
		unsigned char temp_act_sensors = 0;
		mpu_get_power_state( &temp_act_sensors );
		
		if( (temp_act_sensors&INV_XYZ_ACCEL) > 0 )
		{
			return ON;
		}
	#else
		if( (lsm6ds3_active_sensors()&ACCEL_EN_MASK) > 0 ) 
		{
			return ON;
		}
	#endif
	
	return OFF;
}

///
//! \fn
/// \brief
/// \param
/// return
///
bool get_gyro_en( void )
{
	#if defined MPU9250
		unsigned char temp_act_sensors = 0;
		mpu_get_power_state( &temp_act_sensors );
		
		if( (temp_act_sensors&INV_XYZ_GYRO) > 0 )
		{
			return ON;
		}
	#else
		if( (lsm6ds3_active_sensors()&GYRO_EN_MASK) > 0 ) 
		{
			return ON;
		}
	#endif
	
	return OFF;
}

///
//! \fn
/// \brief
/// \param
/// return
///
static void get_active_axes( T_ACTIVE_SENSORS * act_axes )
{		
	act_axes->accel.flags = 0;
	act_axes->gyro.flags = 0;
	act_axes->magnet.flags = 0;
		
#if defined MPU9250
	unsigned char temp_act_sensors = 0;
	mpu_get_power_state( &temp_act_sensors );
	
	if( (temp_act_sensors&INV_XYZ_ACCEL) > 0 ) {
		act_axes->accel.x = 1;
		act_axes->accel.y = 1;
		act_axes->accel.z = 1;
	}

	if( (temp_act_sensors&INV_XYZ_COMPASS) > 0) {
		act_axes->magnet.x = 1;
		act_axes->magnet.y = 1;
		act_axes->magnet.z = 1;
	}

	if( (temp_act_sensors&INV_X_GYRO) > 0) {
		act_axes->gyro.x = 1;
	}
	if( (temp_act_sensors&INV_Y_GYRO) > 0) {
		act_axes->gyro.y = 1;
	}
	if( (act_sensors&INV_Z_GYRO) > 0) {
		act_axes->gyro.z = 1;
	}
#else
	uint16_t temp_act_sensors = lsm6ds3_active_sensors();
	
	if( (temp_act_sensors&GYRO_EN_MASK) > 0 ) {
		act_axes->gyro.x = 1;
		act_axes->gyro.y = 1;
		act_axes->gyro.z = 1;
	}
	
	if( (temp_act_sensors&EXTERN_SENS_EN_MASK) > 0 ) {
		act_axes->magnet.x = 1;
		act_axes->magnet.y = 1;
		act_axes->magnet.z = 1;
	}
	 
	temp_act_sensors &= ~(GYRO_EN_MASK|EXTERN_SENS_EN_MASK);	//clear the gyro and magnet enabled bit, if any other bits are set, then the accelerometer is On
	
	if( (temp_act_sensors > 0) && (temp_act_sensors&MOTION_WAKE_EN_MASK) == 0) {
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
	static ret_code_t prv_err = NRF_SUCCESS;
	bool ret = true;
	
#if defined MPU9250
	ret = mpu9250_check();
#elif defined LSM6DS3
	ret_code_t err = lsm6ds3_check_wom();
	if( err != NRF_SUCCESS ) {
		err = enable_motion_wakeup();
		if( err != NRF_SUCCESS ) {
			if( prv_err != err ) 
			{
				app_trace_log(DEBUG_HIGH, "IMU WOM Error\r");
			}
			ret = false;
		}
	}
	prv_err = err;
#endif

	return ret;
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
static volatile TTASK_TIMER accelPowerDownTimer = { 0, 0 };
static volatile TTASK_TIMER gyroPowerDownTimer= { 0, 0 };
static volatile TTASK_TIMER magPowerDownTimer = { 0, 0 };
static T_ACTIVE_SENSORS lock = { .accel.flags = 0, .gyro.flags = 0, .magnet.flags = 0 };
void imu_power_state( void )
{
	static T_IMU_POWER_STATE imu_mode_sv = WAIT_FOR_MOTION; 
	static uint8_t retries[3] = {0};
	T_ACTIVE_SENSORS act_sensors;
	
	switch ( imu_mode_sv ) {
			
		case WAIT_FOR_MOTION:
			if( (lock.accel.flags != 0x00) ) {
				//Accelerometer needs to turn On to Record data
				if( start_accel() == true ) {
					if( imu_debug ) app_trace_log(DEBUG_MED, "Accel On\r");	
					imu_mode_sv = IMU_ACTIVE;
				}
				else {
					if( imu_debug ) app_trace_log(DEBUG_MED, "Accel On Failure!!!\r");
				}
			}
			else if( check_wake_up_evt() ) {
				if( start_accel() == true ) {
					keep_accel_on();	//Keep Accel On for at least X more seconds to determine motion...
					if( imu_debug ) app_trace_log(DEBUG_MED, "IMU Motion Wakeup!!!\r");
					
					imu_mode_sv = IMU_ACTIVE;
				}
				else {
					if( imu_debug ) app_trace_log(DEBUG_MED, "Motion Wakeup Failure!!!\r");
				}
			}
			else if( imu_slp_check() != true ) {
				//imu is not in it's wake on motion mode
				if( enable_motion_wakeup() == NRF_SUCCESS ) {
					if( imu_debug ) app_trace_log(DEBUG_MED, "IMU Sleep Mode\r");
				}
			}
			else {
				//imu monitoring for motion, but there has not been a wake event... so why are we here?
				if( imu_debug ) app_trace_log(DEBUG_MED, "IMU Asleep???\r");
			}
			break;
			
		case IMU_ACTIVE:
			//turn Gyro and Mag on as needed
			//If nothing needs to be on, go back to waiting for motion
		
			//Get the On/Off status of the 9 Axis IMU...
			get_active_axes( &act_sensors );
			
			if( (lock.accel.flags == 0x00) && task_time(accelPowerDownTimer) ) {
				//Time to enter back into low power wait for motion mode. Make sure everything is powered down					
				stop_task_timer( gyroPowerDownTimer );	//turn Timer Off
				retries[G] = 0;
				if( act_sensors.gyro.flags != 0 )	{
					if( imu_debug ) app_trace_log(DEBUG_LOW, "Gyro Off\r");
					stop_gyro();
				}
				
				stop_task_timer( magPowerDownTimer );	//turn Timer Off
				retries[M] = 0;
				if( act_sensors.magnet.flags != 0 )	{
					if( imu_debug ) app_trace_log(DEBUG_LOW, "Compass Off\r");
					stop_compass();
				}
				
				//turn on wake on motion interrupt
				if( enable_motion_wakeup() == NRF_SUCCESS ) {
					if( imu_debug ) app_trace_log(DEBUG_MED, "IMU Sleep\r");
			
					imu_mode_sv = WAIT_FOR_MOTION;
				}
			}
			else {
				//Handle the turning on/off of the Gyroscope	
				if( act_sensors.gyro.flags == 0 ) {
					//sensor is currently Off
					if( (lock.gyro.flags != 0 || gyroPowerDownTimer.period > 0) && retries[G] < 3 ) {
						//Something has requested the Gyro to turn On either temporarily(timer) or indefinitely(lock)
						if( imu_debug ) app_trace_log(DEBUG_MED, "Gyro On\r");
						start_gyro();
						if( lock.gyro.flags != 0 ) {
							//Gyro data is going to be recorded. This will force recorded Accel data to sync with it
							flag_raw_data_sync();
						}
						retries[G]++;
					}
				}
				else {
					//Sensor is On, wait for the On request to be removed
					if( (lock.gyro.flags == 0) && task_time(gyroPowerDownTimer) ) {
						stop_task_timer( gyroPowerDownTimer );	//turn Timer Off
						retries[G] = 0;
						if( imu_debug ) app_trace_log(DEBUG_MED, "Gyro Off\r");
						stop_gyro();
					}
				}
				
				//Handle the turning on/off of the Magnetometer	
				if( act_sensors.magnet.flags == 0 ) {
					//sensor is currently Off
					if( (lock.magnet.flags != 0 || magPowerDownTimer.period > 0) && retries[M] < 3 ) {
						//Something has requested the Compass to turn On either temporarily or indefinitely
						if( imu_debug ) app_trace_log(DEBUG_MED, "Compass On\r");
						start_compass();
						if( lock.magnet.flags != 0 ) {
							//Compass data is going to be recorded. This will force recorded Accel data to sync with it
							flag_raw_data_sync();
						}
						retries[M]++;
					}
				}
				else {
					//Sensor is On, wait for the On request to be removed
					if( (lock.magnet.flags == 0) && task_time(magPowerDownTimer) ) {
						stop_task_timer( magPowerDownTimer );	//turn Timer Off
						retries[M] = 0;
						if( imu_debug ) app_trace_log(DEBUG_MED, "Compass Off\r");
						stop_compass();
					}
				}
			}
			break;
			
			//Force Accel and Gyro to stay On
//			start_gyro();
//			imu_mode_sv = 3;
//			break;
//		case 3:
//			//leave on forever	
//			break;		
		
		default:
			imu_mode_sv = WAIT_FOR_MOTION; 
			break;
			
	}
}

//Reset timer. Temporarily keeps accelerometer on.
void keep_accel_on( void )
{
	start_task_timer( accelPowerDownTimer, ACCEL_PD_TO );
}

//Reset timer. Temporarily keeps gyrometer on.
void keep_gyro_on( void )
{
	start_task_timer( gyroPowerDownTimer, GYRO_PD_TO );
}

//Reset timer. Temporarily keeps magnetometer on.
void keep_magnet_on( void )
{
	start_task_timer( magPowerDownTimer, MAG_PD_TO );
}

//Force Accelerometer to stay On
static void lock_accel( bool lock_on )
{
	T_ACTIVE_SENSORS act_sensors;
	get_active_axes( &act_sensors );
	
	if( lock_on == true ) {
///		app_trace_puts(DEBUG_MED, "Accel Lock: On\r");
		lock.accel.flags = 0;
		lock.accel.x = 1;
		lock.accel.y = 1;
		lock.accel.z = 1;
		if( act_sensors.accel.flags == 0 ) {
			//Accelerometer is in WOM mode, we need to force an update to kick it ON
			imu_power_state();
		}
	}
	else {
		app_trace_puts(DEBUG_MED, "Accel Lock: Off\r");
		lock.accel.flags = 0;
	}
}

//Force Gyrometer to stay On
static void lock_gyro( bool lock_on )
{
	if( lock_on == true ) {
		app_trace_puts(DEBUG_MED, "Gyro Lock: On\r");
		lock.gyro.flags = 0;
		lock.gyro.x = 1;
		lock.gyro.y = 1;
		lock.gyro.z = 1;
	}
	else {
		app_trace_puts(DEBUG_MED, "Gyro Lock: Off\r");
		lock.gyro.flags = 0;
	}
}

//Force Magnetometer to stay On
static void lock_magnet( bool lock_on )
{
	if( lock_on == true ) {
		app_trace_puts(DEBUG_MED, "Compass Lock: On\r");
		lock.magnet.flags = 0;
		lock.magnet.x = 1;
		lock.magnet.y = 1;
		lock.magnet.z = 1;
	}
	else {
		app_trace_puts(DEBUG_MED, "Compass Lock: Off\r");
		lock.magnet.flags = 0;
	}
}


//The following methods allow outside DAta logging routines to force sensors to come On and stay On
static bool record_accel = false;
static bool record_gyro = false;
static bool record_compass = false;
void record_accel_on( void ) 
{
	lock_accel( true );
	record_accel = true;
}

void record_accel_off( void ) 
{
	record_accel = false;
	if( record_gyro == false && record_compass == false ) {
		//Nothing recording, Accel can go Off and back to WOM mode
		lock_accel( false );
	}
}

void record_gyro_on( void ) 
{
	lock_gyro( true );
	record_gyro = true;
	if( record_accel == false ) {
		//Accelerometer needs to lock On so IMU can't go back to sleep
		lock_accel( true );
	}
}

void record_gyro_off( void ) 
{
	lock_gyro( false );
	record_gyro = false;
	if( record_accel == false && record_compass == false ) {
		//Nothing recording, Accel can go Off and back to WOM mode
		lock_accel( false );
	}
}

void record_compass_on( void ) 
{
	lock_magnet( true );
	record_compass = true;
	if( record_accel == false ) {
		//Accelerometer needs to lock On so IMU can't go back to sleep
		lock_accel( true );
	}
}

void record_compass_off( void ) 
{
	lock_magnet( false );
	record_compass = false;
	if( record_accel == false && record_gyro == false ) {
		//Nothing recording, Accel can go Off and back to WOM mode
		lock_accel( false );
	}
}

ret_code_t test_imu( void )
{
	ret_code_t res;
	
#if defined MPU9250
	res = ERR_NONE;
#elif defined LSM6DS3
	//turn Interrupt Off during self test
	nrf_drv_gpiote_in_event_disable( IMU_INT );
	
	res = lsm6ds3_run_self_test();
	
	nrf_drv_gpiote_in_event_enable( IMU_INT, true );
#endif
	
	return res;
}
