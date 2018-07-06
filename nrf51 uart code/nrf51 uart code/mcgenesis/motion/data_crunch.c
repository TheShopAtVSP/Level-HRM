/*
 * data_crunch.c
 *
 * Created: 3/18/2015 8:55:15 AM
 *  Author: matt
 */ 

#include "data_crunch.h"
#include "step_detect.h"

//Return the approximate angle in degrees of the ratio of 256*Opposite/Hypotenuse:
const uint8_t arccos[256] = 
{	
	90,90,90,89,89,89,89,88,88,88,88,88,87,87,87,87,86,86,86,86,86,85,85,85,85,84,84,84,84,83,83,83,
	83,83,82,82,82,82,81,81,81,81,81,80,80,80,80,79,79,79,79,78,78,78,78,78,77,77,77,77,76,76,76,76,
	75,75,75,75,75,74,74,74,74,73,73,73,73,72,72,72,72,71,71,71,71,71,70,70,70,70,69,69,69,69,68,68,
	68,68,67,67,67,67,66,66,66,66,65,65,65,65,64,64,64,64,63,63,63,63,62,62,62,62,61,61,61,61,60,60,
	60,60,59,59,59,59,58,58,58,58,57,57,57,56,56,56,56,55,55,55,55,54,54,54,53,53,53,53,52,52,52,51,
	51,51,51,50,50,50,49,49,49,48,48,48,48,47,47,47,46,46,46,45,45,45,44,44,44,43,43,43,43,42,42,41,
	41,41,40,40,40,39,39,39,38,38,38,37,37,36,36,36,35,35,35,34,34,33,33,33,32,32,31,31,30,30,29,29,
	29,28,28,27,27,26,26,25,25,24,23,23,22,22,21,20,20,19,18,18,17,16,15,14,13,12,11,10, 9, 7, 5, 0
};

//Ruturn the approximate angle in degress of the ratio of 4096*Opposite/Hypotenuse. However the 
//input Range is limited to the 64 values between {4032:4095}. This gives an improved angle resolution
//for small angles.
const uint8_t smallangle_arccos[64] =
{
	10,10,10,10,10,10,10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7,
	 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 1, 0
};

extern bool mot_debug;
extern T_3AXIS_FIFO accel_buf;
extern T_3AXIS_FIFO gyro_buf;
extern T_3AXIS_FIFO magnet_buf;

//local circular buffer tail pointers, may be overrun if this function is not called frequently enough
uint16_t a_tail = 0;
uint16_t g_tail = 0;
uint16_t m_tail = 0;

TMOTION Motion;
TMOT_STATISTICS mStats;

#define USE_INTEGER_MATH
#ifdef USE_INTEGER_MATH
int32_t rot_factor[3][3];
struct {
	uint32_t deg_2_rads;
	uint8_t deg_2_rads_shift;	//the right shift value to keep 4 significant digits in the above scalar
} scalar;
#else
float rot_dis[3][3];
struct {
	float deg_2_rads;
	float grav_range;
} scalar;
#endif

T_ACTIVE_SENSORS active_axes;

void calc_rotation_coeffecients(void);
void calc_linear_accel(void);
void cyclic_motion_monitor( void );
int16_t get_angle( int16_t adjac, int16_t hypot );

///
//! \fn
/// \brief
/// \param
/// \return .
///
void init_motion_analysis( bool debug, uint32_t rel_step_cnt )
{
	Motion.head = 0;
	
	a_tail = accel_buf.head;
	g_tail = gyro_buf.head;
	m_tail = magnet_buf.head;
	
	check_sample_rate();
	calc_rotation_coeffecients();
	
	//Startup Step TRacking Algorithms
	initStepDetect( debug );
	setStepCnt( rel_step_cnt );

	x_initStepDetect( debug );
	x_setStepCnt( rel_step_cnt );
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
int16_t get_angle( int16_t adjac, int16_t hypot )
{
	int16_t angle_deg;
	int32_t cos_angle = (int32_t)4095*adjac/hypot;
	int16_t offset = 0;
	
	//keep cos_angle between 0-90 degrees for lookup table.
	if( cos_angle < 0 ) {
		 cos_angle *= -1;
		 offset = -180;
	}
	
	if( cos_angle >= 4095 ) {
		//undefined, return 0
		angle_deg = 0;
	}
	else if( cos_angle >= 4032 ) {
		//get result from small angle lookup table range: {0:63} => 10:0 degrees
		angle_deg = smallangle_arccos[ cos_angle-4032 ];
	}
	else {
		//get result from angle lookup table range: {0:251} => 90:10 degrees
		angle_deg = arccos[ cos_angle/16 ];	
	}
	
	angle_deg += offset;
	
	return angle_deg;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void check_sample_rate(void)
{
	uint16_t rate = 0;
	
	rate = get_accel_sample_rate();
		
	//Update Globals if there's been a change...
	if( rate != Motion.sampleRate ) {
		Motion.sampleRate = rate;
		if( Motion.sampleRate > 0 ) {
			Motion.samplePeriod = 1000/Motion.sampleRate;     //Period between samples in milliseconds
		}
		else {
			Motion.samplePeriod = 0;     //undef
		}
	}
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
T_ACCEL_COMPONENT * getCalcReadings( void )
{
	return &Motion.accel[Motion.head];
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void calc_rotation_coeffecients(void)
{
	uint16_t gyro_fsr = 0;
	uint16_t accel_1g = 0;
	
	//Get the gyro full scale range
	gyro_fsr = get_gyro_fsr();

	//Get the accelerometer sensitivity
	accel_1g = get_accel_sens();
	
#ifdef USE_INTEGER_MATH
	//Calculate scaler to convert gyro data X to a multiple of radians per sec:
	//X*(fsr/32768)*(pi/180) = X*fsr*3.14159/(180*32768)
	//Shift by S+5 to get 4 digits of precision: (fsr*3.14159/180)*X/(2^5*2^10) = (2^(S+5))*Scaler*X/2^10 => degs to 1024*Radians
	//To keep 4 significant digits, the scaler needs to be greater than 1000*divisor = 180,000
	scalar.deg_2_rads_shift = 0;
	do {
		scalar.deg_2_rads_shift++;
		scalar.deg_2_rads = ((uint32_t)gyro_fsr<<scalar.deg_2_rads_shift)*3.14159;
	} while( scalar.deg_2_rads < (1000*180) );
	scalar.deg_2_rads /= 180;			//value has 4 sig figs: 1000 < Gyro.deg2RadsScalar < 2000
	scalar.deg_2_rads_shift += 5;	//from above, we see that shift value = S+5 to yield conversion results that are 1/1024 rads/bit

	//Scalers for Rotational Accelerations about the pivot points of the head. The distances (in meters) have
	//been empirically determined by testing the algorithm with rotational motion, ie head nodding.
	rot_factor[X][X] = 0;
	rot_factor[X][Y] = -0.060*accel_1g/EARTH_G;	//X rotation distance (meters) around Y Axis with conversion to g scale
	rot_factor[X][Z] = -0.065*accel_1g/EARTH_G;	//X rotation distance around Z Axis
	rot_factor[Y][X] = -0.020*accel_1g/EARTH_G;	//Y rotation distance around X Axis
	rot_factor[Y][Y] = 0;
	rot_factor[Y][Z] = -0.040*accel_1g/EARTH_G;	//Y rotation distance around Z Axis
	rot_factor[Z][X] =  0.075*accel_1g/EARTH_G;	//Z rotation distance around X Axis
	rot_factor[Z][Y] =  0.075*accel_1g/EARTH_G;	//Z rotation distance around Y Axis
	rot_factor[Z][Z] = 0;
#else
	//There is some issue with using Floating Point???????
	//Calculate scaler to convert gyro data X to a multiple of radians per sec:
	scalar.deg_2_rads = ((double) gyro_fsr*3.14159/180)/32768;
	
	//Calculate scaler to convert gyro data X to a multiple of radians per sec:
	scalar.grav_range = Motion.accel_1g/EARTH_G;
	
	//Scalers for Rotational Accelerations about the pivot points of the head. The distances (in meters) have
	//been empirically determined by testing the algorithm with rotational motion, ie head nodding.
	rot_dis[X][X] = 0;
	rot_dis[X][Y] = -0.060;	//X rotation distance (meters) around Y Axis
	rot_dis[X][Z] = -0.065;	//X rotation distance around Z Axis
	rot_dis[Y][X] = -0.020;	//Y rotation distance around X Axis
	rot_dis[Y][Y] = 0;
	rot_dis[Y][Z] = -0.040;	//Y rotation distance around Z Axis
	rot_dis[Z][X] = 0.075;	//Z rotation distance around X Axis
	rot_dis[Z][Y] = 0.075;	//Z rotation distance around Y Axis
	rot_dis[Z][Z] = 0;
#endif
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void calc_linear_accel(void)
{
	int32_t itemp;
	static bool prv_valid = false;
#ifdef USE_INTEGER_MATH	
	int32_t angul_vel[3], angul_acc[3];
	static int32_t prv_av[3] = {0};
#else
	float angul_vel[3], angul_acc[3];
	static float prv_av[3] = {0};
#endif

	if( active_axes.gyro.flags > 0 ) {
		// Convert angular velocity to radians/sec
		g_tail &= IMU_FIFO_MASK;	//force to stay within array bounds	
	#ifdef USE_INTEGER_MATH		
		itemp = scalar.deg_2_rads*gyro_buf.fifo[g_tail].x;
		itemp >>= scalar.deg_2_rads_shift;           //result is shifted down to 1024*Radians
		angul_vel[X] = itemp;
		itemp = scalar.deg_2_rads*gyro_buf.fifo[g_tail].y;
		itemp >>= scalar.deg_2_rads_shift;
		angul_vel[Y] = itemp;
		itemp = scalar.deg_2_rads*gyro_buf.fifo[g_tail].z;
		itemp >>= scalar.deg_2_rads_shift;
		angul_vel[Z] = itemp;		
	#else
		angul_vel[X] = scalar.deg_2_rads*gyro_buf.fifo[g_tail].x;
		angul_vel[Y] = scalar.deg_2_rads*gyro_buf.fifo[g_tail].y;
		angul_vel[Z] = scalar.deg_2_rads*gyro_buf.fifo[g_tail].z;
	#endif
		if( g_tail != gyro_buf.head ) {
			g_tail = (g_tail+1)&IMU_FIFO_MASK;
		}

		if( prv_valid == true ) {
			// Compute the angular acceleration.
			angul_acc[X] = Motion.sampleRate*(angul_vel[X] - prv_av[X]);
			angul_acc[Y] = Motion.sampleRate*(angul_vel[Y] - prv_av[Y]);
			angul_acc[Z] = Motion.sampleRate*(angul_vel[Z] - prv_av[Z]);
			
			// Calculate the rotational components of accelerations: aR + w^2R (note: w^2 components will have little 
			// effect at the normal rotation speeds of a human head!!! )
			//result is in the same g-scale as the measured Accelerometer values
		#ifdef USE_INTEGER_MATH	
			itemp =  -angul_acc[Z]*rot_factor[Y][Z] +angul_acc[Y]*rot_factor[Z][Y];
			itemp >>= 10; //remove remaining 1024 factor
			Motion.accel[Motion.head].rot.x = itemp;		//result is in the same g-scale as the measured Accelerometer values
			itemp = +angul_acc[Z]*rot_factor[X][Z] -angul_acc[X]*rot_factor[Z][X];
			itemp >>= 10;
			Motion.accel[Motion.head].rot.y = itemp;
			itemp = -angul_acc[Y]*rot_factor[X][Y] +angul_acc[X]*rot_factor[Y][X];
			itemp >>= 10;
			Motion.accel[Motion.head].rot.z = itemp;
		#else
			Motion.accel[Motion.head].rot.x = scalar.grav_range*(-angul_acc[Z]*rot_dis[Y][Z] +angul_acc[Y]*rot_dis[Z][Y]);	
			Motion.accel[Motion.head].rot.y = scalar.grav_range*(+angul_acc[Z]*rot_dis[X][Z] -angul_acc[X]*rot_dis[Z][X]);
			Motion.accel[Motion.head].rot.z = scalar.grav_range*(-angul_acc[Y]*rot_dis[X][Y] +angul_acc[X]*rot_dis[Y][X]);
		#endif
		}
		else {
			Motion.accel[Motion.head].rot.x = 0;
			Motion.accel[Motion.head].rot.y = 0;
			Motion.accel[Motion.head].rot.z = 0;
			prv_valid = true;
		}
		
		prv_av[X] = angul_vel[X];
		prv_av[Y] = angul_vel[Y];
		prv_av[Z] = angul_vel[Z];
		
		//Linear Acceleration is difference of the reported Accel and Rotational components
		itemp = accel_buf.fifo[a_tail].x - Motion.accel[Motion.head].rot.x;
		if( itemp > 65535 ) itemp = 65535;			//These values will be Squared, make sure they will fit in a 32 bit result...
		else if( itemp < -65535 ) itemp = -65535;
		Motion.accel[Motion.head].lin.x = itemp;
		itemp = accel_buf.fifo[a_tail].y - Motion.accel[Motion.head].rot.y;
		if( itemp > 65535 ) itemp = 65535;
		else if( itemp < -65535 ) itemp = -65535;
		Motion.accel[Motion.head].lin.y = itemp;
		itemp = accel_buf.fifo[a_tail].z - Motion.accel[Motion.head].rot.z;
		if( itemp > 65535 ) itemp = 65535;
		else if( itemp < -65535 ) itemp = -65535;
		Motion.accel[Motion.head].lin.z = itemp;
		
		calc_magnitude( &Motion.accel[Motion.head].lin );
		
	}
	else {
		g_tail = gyro_buf.head;	//make sure the gyro buffer tail gets zeroed out
		
		Motion.accel[Motion.head].rot.x = 0;
		Motion.accel[Motion.head].rot.y = 0;
		Motion.accel[Motion.head].rot.z = 0;
		
		//No Rotation, thus no Rotational Acceleration, skip all the calcs
		Motion.accel[Motion.head].lin.x = accel_buf.fifo[a_tail].x;
		Motion.accel[Motion.head].lin.y = accel_buf.fifo[a_tail].y;
		Motion.accel[Motion.head].lin.z = accel_buf.fifo[a_tail].z;
		Motion.accel[Motion.head].lin.m = accel_buf.fifo[a_tail].m;
		
		prv_valid = false;
	}
}


///
//! \fn
/// \brief
/// \param
/// \return .
///
//#include "led.h"
void calc_magnitude( T_3AXIS * pythagoras )
{
	uint32_t mag;
	
	//Calculate Magnitude of the resulting Linear Acceleration Vector
	mag = pythagoras->x * pythagoras->x;
	mag += pythagoras->y * pythagoras->y;
	mag += pythagoras->z * pythagoras->z;
	
	pythagoras->m = fast_sqrt( mag );
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
//#include "led.h"
uint16_t fast_sqrt( uint32_t operand )
{
	uint16_t sqrt = 0;
	uint16_t test_bit = 0x8000;
	uint32_t test_root;
	
	//	nrf_gpio_pin_set( LED_ON );
	//Fast Integer Square Root
	do {
		test_root = sqrt|test_bit;
		if( (test_root*test_root) <= operand ) {
			sqrt |= test_bit;	//keep bit, it is part of the result
		}
		test_bit >>= 1;
	} while( test_bit != 0 );
//	nrf_gpio_pin_clear( LED_ON );
	
	return sqrt;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
int calc_statistics( void )
{
	static uint16_t tail[3] = {0};
	T_3AXIS_FIFO *fifo_ptr;
	int32_t sums[4];
	int32_t itemp = 0;
	uint16_t len;
	
	//Calc the Means and Variances of the Motion Sensor Signals 
	for( T_SENSOR_TYPE sens=A; sens<=M; sens++ ) {
		switch ( sens ) {
			case A:
				fifo_ptr = &accel_buf;
				break;
			case G:
				fifo_ptr = &gyro_buf;
				break;
			case M:
				fifo_ptr = &magnet_buf;
				break;
			default:
				return -1;

		}
		
		len = (fifo_ptr->head - tail[sens])&IMU_FIFO_MASK;
		if( len > 0 ) {
			//Calculate the Average signal value during the window of interest
			sums[X] = 0;
			sums[Y] = 0;
			sums[Z] = 0;
			sums[R] = 0;
			uint16_t temp_tail = tail[sens];
			for( int i=0; i<len; i++ ) {
				sums[X] += fifo_ptr->fifo[temp_tail].x;
				sums[Y] += fifo_ptr->fifo[temp_tail].y;
				sums[Z] += fifo_ptr->fifo[temp_tail].z;
				sums[R] += fifo_ptr->fifo[temp_tail].m;
				
				temp_tail++;
				temp_tail &= IMU_FIFO_MASK;
			}
			
			mStats.mean[sens].x = sums[X]/len;
			mStats.mean[sens].y = sums[Y]/len;
			mStats.mean[sens].z = sums[Z]/len;
			mStats.mean[sens].m = sums[R]/len;
			
			//Accumulate the Variances over this length window
			mStats.var[sens][X] = 0;
			mStats.var[sens][Y] = 0;
			mStats.var[sens][Z] = 0;
			for( int i=0; i<len; i++ ) {
				itemp = fifo_ptr->fifo[tail[sens]].x - mStats.mean[sens].x;
				mStats.var[sens][X] += (itemp*itemp)/len;
				itemp = fifo_ptr->fifo[tail[sens]].y - mStats.mean[sens].y;
				mStats.var[sens][Y] += (itemp*itemp)/len;
				itemp = fifo_ptr->fifo[tail[sens]].z - mStats.mean[sens].z;
				mStats.var[sens][Z] += (itemp*itemp)/len;
				itemp = fifo_ptr->fifo[tail[sens]].m - mStats.mean[sens].m;
				mStats.var[sens][R] += (itemp*itemp)/len;
				
				tail[sens]++;
				tail[sens] &= IMU_FIFO_MASK;
			}
		}
	}
	
	mStats.tilt = get_angle( mStats.mean[A].VERTICAL_AXIS, mStats.mean[A].m );
			
	return 0;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t motionAnalyze(void)
{	
	get_active_axes( &active_axes );
	
	//analyze any data points that have yet to be looked at...
	while( a_tail != accel_buf.head )
	{		
		//keep_accel_on();	//force accel to stay on
		
		//Calculate the Mean and Variance of the 9 axes over the recent population of Data
		//if( Motion.head >= MOT_BUF_MASK ) {
		//	calc_statistics();
		//			
		//	//if (mot_debug) app_trace_log("Tilt: %02u \r", mStats.tilt);
		//	//if (mot_debug) app_trace_log("A Mean: %6u, %6u, %6u\r", mStats.mean[A].x, mStats.mean[A].y, mStats.mean[A].z);
		//	//if (mot_debug) app_trace_log("A Var:  %6u, %6u, %6u\r", mStats.var[A][X], mStats.var[A][Y], mStats.var[A][Z]);
		//}
			
		//Uses approximate values for head pivot point to remove some of the rotational accelerations...
		calc_linear_accel();	
		
		//Print Results...
		//if (inv_debug) app_trace_log("Accel Rot: %04u, %04u, %04u\r", Motion.Calc[Motion.head].AxRot, Motion.Calc[Motion.head].AyRot, Motion.Calc[Motion.head].AzRot);
					
		//fall_motion_monitor( a_tail );
		
		cyclic_motion_monitor();	
		
		Motion.head = (Motion.head+1)&MOT_BUF_MASK;
		
		a_tail = (a_tail+1)&IMU_FIFO_MASK;
	}
	
	//Overall activity level High, Low, Zero
	
	//if( activeImuSensors == 0 && time > 0 ) {
		////reset variables
		//time = 0;
	//}
				
	return ( 1 );
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void cyclic_motion_monitor( void ) 
{
	TSTEP_FEEDBACK step_fb = NOTHING;
	static TRECOG_MOTION type_o_mo = UNRECOG;
	static uint32_t time = 0;
	volatile uint16_t accel_1g = get_accel_sens();
	
	time += Motion.samplePeriod;	//track time in milliseconds
	
	//Update Magnitude and its Derivative buffer's
	Motion.accel[Motion.head].filter_mag = (Motion.accel[Motion.head].lin.m + Motion.accel[(Motion.head-1)&MOT_BUF_MASK].filter_mag)/2;	//smooth it out a little...
	
	//if (imu_debug) app_trace_log("%04i\r", Motion.accel[Motion.head].filter_mag);

	switch ( type_o_mo ) {		
		case WALKING:
		case RUNNING:
			x_stepStateMachine( time, Motion.accel[Motion.head].lin.m );
			step_fb = stepStateMachine( time, Motion.accel[Motion.head].filter_mag );
			break;
				
		case CYCLING:
			//look for cycling
			type_o_mo = UNRECOG;	//if not determined, go back to Undetermined State
			break;
				
		case DRIVING:
			type_o_mo = UNRECOG;	//if not determined, go back to Undetermined State
			break;
				
		default:	//Undetermined Motion
			
			//look for stepping motion
			x_stepStateMachine( time, Motion.accel[Motion.head].lin.m );
			step_fb = stepStateMachine( time, Motion.accel[Motion.head].filter_mag );
			if( step_fb == STEP_CNT_ADD ) 
			{
				type_o_mo = WALKING;	//Stepping
			}
				
			//look for cycling motion
			break;
	}
}

///
//! ComplementaryFilter( )
/// function determines the tilt angle of the platform based off of gyro data and
/// readings of the g Vector.
/// \param
/// \return .
///
//void ComplementaryFilter( void );
//void ComplementaryFilter( void )
//{
//    static long pitchGyro = 0, rollGyro = 0;
//    long pitchAccel, rollAccel;
//    static int aveAx = 4731, aveAy = 4731, aveAz = 4731;    //(4731^2 + 4731^2 + 4731^2)^0.5 = 8196
//
//    // Gyro Sensitivity is +-Xdps/2^15bits. So READING*SENSITIVITY*dt = scaler*READING = angle
//    // scaler = SENSITIVITY*dt = (X/2^15)*(1/SAMPLE_RATE) = (X/SAMPLE_RATE)/2^15 = (X/SAMPLE_RATE)>>15
//    int X = Status.Imu.Gyro.uiSensitivity/Status.Imu.uiSampleRate;
//    pitchGyro += (long)X*Status.Imu.Raw.Gz.iVal >> 15;  // Angle around the Z-axis( horizontal axis perpendicular to temple)
//    rollGyro += (long)X*Status.Imu.Raw.Gx.iVal >> 15;   // Angle around the X-axis( horizontal axis parallel to temple)
//
//    //Tracking the average gives a pretty good indication of the g vector component
//    //contained in each accel axis reading
//    aveAx = (long)((long)31*aveAx + Status.Imu.Raw.Ax.iVal) >> 5;
//    aveAy = (long)((long)31*aveAy + Status.Imu.Raw.Ax.iVal) >> 5;
//    aveAz = (long)((long)31*aveAz + Status.Imu.Raw.Ax.iVal) >> 5;
//
//    // Z Axis points Perpendicularly from Temple ( pitch axis )
//    if( aveAy != 0 )
//    {
//        //Look up table based on aveAx/aveAy
//    }
//    else
//    {
//        //+-90 degrees
//    }
//    pitchGyro = (pitchGyro*98 + pitchAccel*2)/100;
//
//    if( aveAy > 0 )
//    {   //rightside up
//        if( aveAx > 0)
//        {   //pitched down into quadrant 4
//
//        }
//        else
//        {   //pitched up into quadrant 1
//
//        }
//
//    }
//    else if( aveAy < 0 )
//    {   //upside down
//        if( aveAx >= 0)
//        {   //pitched down into quadrant 3
//
//        }
//        else
//        {   //pitched up into quadrant 2
//
//        }
//    }
//
//    // X Axis points parallel to Temple ( roll axis )
//    if( aveAy != 0 )
//    {
//        //Look up table based on aveAz/aveAy
//    }
//    else
//    {
//        //+-90 degrees
//    }
//    rollGyro = (rollGyro*98 + rollAccel*2)/100;
//}

