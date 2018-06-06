/*
 * mpu9250.c
 *
 * Created: 10/01/2014 1:52:00 PM
 *  Author: MattWo
 */ 

#include "mpu9250.h"

const T_3AXIS void_res = {
	.x = 0,
	.y = 0,
	.z = 0,
};
	
T_IMU_ERROR_CODE mpu9250_board_init(void)
{
	uint8_t reply[4];
	T_IMU_ERROR_CODE op_stat;
	
	// first check if the part is responsive...
	op_stat = mpu_who_am_i( reply );
	if( op_stat != ERROR_NONE) {
		return op_stat;
	}
	if( reply[0] != 0x71 ) {
		return ERROR_DIDNOTID;	//mpu9250 responds with 0x71 by default
	}
	
	//wake device up and init stuffs...
	if( mpu_init() != 0 ) {
		//something has failed...
		return ERROR_INITFAILED;
	}
		
	return ERROR_NONE;
}


///
//! \fn
/// Verify that the accelerometer is still in wakeup on motion state. 
/// If not, we are never going to track anything again...
/// \param
/// \return .
///
bool mpu9250_check( void )
{
	bool res = PASS;
	unsigned char accel_intel = 0;
	
	if( mpu_get_accel_intel_reg(&accel_intel) == 0 )
	{	//read register accel_intel, if Wake on Motion is not set, we have a problem
		if( (accel_intel&BIT_WOM_EN) == 0 )
		{	//Interrupt is off
			if (imu_debug) puts("Motion WakeUp Not Active!!!\r");
			res = FAIL;
		}
	}
	else
	{	//accel read failed! Not good...
		if (imu_debug) puts("MPU Read Error!!!\r");
		res = FAIL;
	}
	
	return res;
}


//use to adjust accel on dev board to make it usable...
//{
	//int16_t accel_bias[3];
	//mpu_read_6500_accel_bias( accel_bias );
	//printf("OTP Bias: %i, %i, %ir\n", accel_bias[0], accel_bias[1], accel_bias[2]);
	//accel[0] = 0;
	//accel[1] = 0;
	//accel[2] = -225;
	//mpu_set_accel_bias_6500_reg( accel );
	//mpu_read_6500_accel_bias( accel );
	//printf("Adjusted Bias: %ld, %ld, %ld\r\n", accel[0], accel[1], accel[2]);
//}


