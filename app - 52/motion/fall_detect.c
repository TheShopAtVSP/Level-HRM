/*
 * fall_detect.c
 *
 * Created: 4/6/2015 9:51:11 AM
 *  Author: matt
 */ 

//#include <math.h>
#include "data_crunch.h"

typedef struct {
	int16_t prefall_tilt;
	uint32_t pre_mag;
	uint32_t impact;
	TFALL_MOTION state;
}T_FALL;

T_FALL fall = {
	.state = NO_FALL,
};

extern TMOTION Motion;
extern TMOT_STATISTICS mStats;
extern T_3AXIS_FIFO accel_buf;

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t FALL_TIMER;
void fall_motion_monitor( uint16_t a_tail )
{
	int32_t acc_mm_s2, itemp, loop_lim, i;
	static int16_t fall_duration = 0;
	static uint32_t timer = 0;
	static uint32_t min_motion_variance;
	static int8_t tilt_filt = 0;
	static int8_t var_filt = 0;
	static int32_t vel_mm_s = 0;
	int16_t accel_1g = get_accel_sens();
	
	timer += Motion.samplePeriod;
	
	if( accel_buf.fifo[a_tail].m < ((uint32_t)5*accel_1g/6) ) {
		fall_duration += Motion.samplePeriod;
	}
	else {
		if( fall_duration >= 200 ) fall.state = POTENTIAL_FALL;
		
		fall_duration = 0;
	}
	
	switch ( fall.state )
	{
		case NO_FALL:
			//do nothing. Waiting for a period where g is reduced. Indicating a drop in elevation
			break;
		
		case POTENTIAL_FALL:		
			//potentially falling. Further analysis needed though.
			fall.prefall_tilt = mStats.tilt;
			fall.impact = 0;
			timer = 0;				//restart timer
			
			//Integrate the acceleration of the fall. If the velocity is high enough, we are certainly falling.
			vel_mm_s = 0;
			loop_lim = 0;
			i = a_tail;
			while ( accel_buf.fifo[i].m < accel_1g && loop_lim < 50 ) {
				acc_mm_s2 = (int32_t)(9807*Motion.accel[i].lin.m)/accel_1g - 9807;
				vel_mm_s += acc_mm_s2/Motion.sampleRate;
				if( --i < 0) i = IMU_FIFO_MASK;
				loop_lim++;
			}
			
			if( vel_mm_s <= -3000 ) {
				//Definitely a fall.
				fall.state = CRITICAL_FALL;
			}
			else {
				tilt_filt = 0;
				var_filt = 0;
				FALL_TIMER = 200;		//Impact should be coming shortly
				if (imu_debug) app_trace_log(DEBUG_LOW, "Possible Fall!!!\r");
				fall.state = WAIT_IMPACT;
			}
			break;
			
		case WAIT_IMPACT:
			acc_mm_s2 = (int32_t)(9807*accel_buf.fifo[a_tail].m)/accel_1g - 9807;
			itemp = acc_mm_s2/Motion.sampleRate;
			vel_mm_s += itemp;
		
			if( acc_mm_s2 > 1960 ) {	//0.20g
				//Accumulate the impact energy:
				fall.impact += itemp*itemp;
				FALL_TIMER = timer + 100;	//push timeout ahead by 100 ms
			}
			else if( timer > FALL_TIMER || timer > 1500 ) {
				if( fall.impact > 1000000 ) {
					//Calculate the minimum Variance that indicates motion:
					min_motion_variance = ((uint32_t)5*accel_1g/100)*((uint32_t)5*accel_1g/100);	//5% of 1g?
					FALL_TIMER = timer + 1000;
					fall.state = FALL;		//High enough impact to be a fall...
				}
				else if( fall.impact > 30000 ) {
					//likely a STAIR_DOWN or a SIT_DOWN
					fall.state = SIT_DOWN;
				}
				else {
					//likely a STAIR_UP or a STAND_UP
					fall.state = STAND_UP;
				}
			}
		
			if( imu_debug ) app_trace_log(DEBUG_LOW, "Mag: %04u, Impact: %04u\r",(unsigned int)accel_buf.fifo[a_tail].m, (int)fall.impact);
			break;
		
		case SIT_DOWN:
			if( false ) {
				//If there have recently been other SIT_DOWNs, then
				//it is most likely STAIRS
				fall.state = STAIR_DOWN;
			}
			else {
				//save Sitting activity
				if (imu_debug) app_trace_log(DEBUG_LOW, "Sit Down Detected\r");
				fall.state = NO_FALL;
			}
			break;
		
		case STAND_UP:
			if( false ) {
				//If there have recently been other STANDUPs, then
				//it is most likely STAIRS
				fall.state = STAIR_UP;
			}
			else {
				//save Sitting activity
				if (imu_debug) app_trace_log(DEBUG_LOW, "Stand Up Detected\r");
				fall.state = NO_FALL;
			}
			break;
		
		case STAIR_DOWN:
			//track stair count
			if (imu_debug) app_trace_log(DEBUG_LOW, "Step Down Detected\r");
			fall.state = NO_FALL;
			break;
		
		case STAIR_UP:
			//track stair count
			if (imu_debug) app_trace_log(DEBUG_LOW, "Step Up Detected\r");
			fall.state = NO_FALL;
			break;
		
		case FALL:
			if( FALL_TIMER <= timer )
			{
				FALL_TIMER = timer + 1000;	//re-look at everything once per sec
			
				if( (mStats.tilt - fall.prefall_tilt) > 45 || (mStats.tilt - fall.prefall_tilt) < -45) {
					//orientation has changed by more than 45 degrees since the pre-fall orient
					tilt_filt++;
				}
				else {
					tilt_filt--;
				}
			
				if( tilt_filt > 10 ) fall.state = CRITICAL_FALL;
				else if( tilt_filt < -10) fall.state = NO_FALL;
			
				if( mStats.var[A][R] > min_motion_variance ) {
					var_filt++;
				}
				else {
					var_filt--;
				}
			
				if( var_filt > 5 ) {
					//there is still quite a bit of activity
					fall.state = NO_FALL;
				}
			}
		
			if( fall.state == NO_FALL ) {
				if (imu_debug) app_trace_log(DEBUG_LOW, "Not a Fall!!!\r");
			}
			break;
		
		case CRITICAL_FALL:
			//Fall is for real...
			if (imu_debug) app_trace_log(DEBUG_LOW, "Fall Detected!!!\r");
			fall.state = NO_FALL;
			break;
		
		default:
			fall.state = NO_FALL;
			break;
	}
}
