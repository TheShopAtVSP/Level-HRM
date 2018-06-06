/*
 * data_crunch.h
 *
 * Created: 3/18/2015 9:52:15 AM
 *  Author: matt
 */ 


#ifndef DATA_CRUNCH_H_
#define DATA_CRUNCH_H_

#include "imu/hal_imu.h"

#define EARTH_G				9.807			//acceleration due to gravity at earths surface in meters/(sec^2)

#define VERTICAL_AXIS	x*(-1)		//in the normal orientation, the g vector will be close to the negative x axis

typedef enum {
	A = 0,
	G = 1,
	M = 2,
} T_SENSOR_TYPE;

typedef enum {
	UNKNOWN = -1,
	X = 0,
	Y = 1,
	Z = 2,
	R = 3,
} T_AXES;


#define MOT_BUF_LEN			64
#define MOT_BUF_MASK		MOT_BUF_LEN-1
typedef struct {
	T_3AXIS lin;			//Linear Acceleration
	T_3AXIS rot;			//Rotational Accelerations
	int32_t filter_mag;
} T_ACCEL_COMPONENT;

typedef struct {
	T_3AXIS mean[3];		//Average of each axis over the sample size of MOT_BUF_LEN
	uint32_t var[3][4];		//Variance in each axis seen during the period of MOT_BUF_LEN
	int16_t tilt;			//the angle between the Vertical axis and the mean_g vector
} TMOT_STATISTICS;

typedef struct {
	uint16_t sampleRate;		//in Hz
	uint16_t samplePeriod;		//in milliseconds
	uint16_t accel_ave;
	uint16_t head;
	T_ACCEL_COMPONENT accel[MOT_BUF_LEN];
} TMOTION;

typedef enum {
	NO_FALL = 0,
	POTENTIAL_FALL,
	WAIT_IMPACT,
	FALL,
	CRITICAL_FALL,
	SIT_DOWN,
	STAND_UP,
	STAIR_DOWN,
	STAIR_UP,
	//JUMPING, SKIPPING, RUNNING, //all things that would lead to a drop in accelerometer that looks like the start of a fall
} TFALL_MOTION;

typedef enum {
	UNRECOG = 0,
	WALKING,
	RUNNING,
	CYCLING,
	DRIVING,
} TRECOG_MOTION;

void init_motion_analysis(bool debug, uint32_t rel_step_cnt);
void check_sample_rate(void);
void calc_magnitude( T_3AXIS * pythagoras );
int calc_statistics( void );
T_ACCEL_COMPONENT * getCalcReadings( void );
uint32_t motionAnalyze(void);
void fall_motion_monitor( uint16_t a_tail );
uint16_t fast_sqrt( uint32_t operand );

#endif /* DATA_CRUNCH_H_ */



