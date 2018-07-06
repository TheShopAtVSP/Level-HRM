/* 
 * File:   step_detect.h
 * Author: MattWo
 *
 * Created on July 11, 2014, 11:11 AM
 */

#ifndef STEPDETECT_H
#define	STEPDETECT_H

#include "imu/hal_imu.h"
#include "data_crunch.h"

//State transition debug results
typedef enum {
	INVALID = -1,
    NOTHING = 0,
    MAXIMA_VALIDATED = 1,   //But not necessarily counted as a Step yet
    MINIMA_VALIDATED = 2,
    MAXIMA_TIMEOUT = 3,
    MINIMA_TIMEOUT = 4,
    MAXIMA_TOO_SOON = 5,
    POSSIBLE_MINIMA = 6,
    MINIMA_TOO_SOON = 7,
    POSSIBLE_MAXIMA = 8,
	MAXIMA_TOO_SMALL = 9,
	STEP_FILTER_CLR = 10,
	STEP_FILTER_1ST = 11,
	STEP_FILTER_ADD = 12,
	STEP_CNT_ADD = 13
} TSTEP_FEEDBACK;

// STEP DETECTION STATES
typedef enum {
	NA = -1,
	INIT = 0,
	FIND_SLOPE = 1,
	MINIMA_HUNT = 2,
	MAXIMA_HUNT = 3,
	VERIFY_MINIMA = 4,
	VERIFY_MAXIMA = 5,
	PEAK_STATS = 6,
	PEAK_TRACK = 7,
	UPDATE = 8,
} TSTEP_DET_STATE;

typedef enum {
	FALSE_STEP = 0,
	TRUE_STEP,
	POSS_STEP,
}T_STEP_CODE;

typedef struct {
	uint32_t time;
	int32_t mag;
} T_FEATURE;

#define SH_BUF_MASK	0x1F
#define SH_BUF_LEN	SH_BUF_MASK+1
typedef struct {
	uint8_t head;
	struct {
		T_STEP_CODE type : 8;
		uint32_t time;
	} step[SH_BUF_LEN];
} T_STEP_HISTORY;

//Step Algorithm calls
void initStepDetect( bool );
TSTEP_FEEDBACK stepStateMachine( uint32_t, uint32_t );
uint32_t getStartupCnt( void );
uint32_t getStepCnt( void );
void setStepCnt( uint32_t );
uint32_t getAbsoluteCnt( void );
T_STEP_HISTORY * step_history( void );

//Experimental Algorithm Calls
void x_initStepDetect( bool );
TSTEP_FEEDBACK x_stepStateMachine( uint32_t, int32_t );
uint32_t x_getStartupCnt( void );
uint32_t x_getStepCnt( void );
void x_setStepCnt( uint32_t );
uint32_t x_getAbsoluteCnt( void );
T_STEP_HISTORY * x_step_history( void );

#endif	/* STEPDETECT_H */

