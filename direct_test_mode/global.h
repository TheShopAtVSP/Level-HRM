

#ifndef GLOBAL_H__
#define GLOBAL_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_error.h"
#include "app_trace.h"
#include "nrf_gpio.h"

//#define APP_TRACE_ON

//Conditional Compile for the different Hardware versions
//#define 	LEVEL_1_0
#define 	LEVEL_1_1

#pragma 	anon_unions
typedef 	unsigned int 		uint;

#define 	PASS      			0
#define 	FAIL      			1
#define 	OFF      			0
#define 	ON      			1
#define	  	delay_ms(A) 		nrf_delay_ms(A)
#define	  	delay_us(A) 		nrf_delay_us(A)
#define 	true				1
#define		false 				0
#define 	min(a,b) 			((a<b)?a:b)
#define		BREAK_POINT			__asm volatile ("nop");			//Use to force a breakable location
#define		NOP()				__asm volatile ("nop");
#define 	BUILD_BUG_ON(condition) ( (void)sizeof(char[1 - 2*!!(condition)]) )
	
typedef union
{
  int32_t 	s32;
  uint32_t 	u32;
  int16_t 	s16[2];
  uint16_t 	u16[2];
  int8_t  	s8 [4];
  uint8_t  	u8 [4];
} Union32;

#define APP_TIMER_PRESCALER		0	/**< Value of the RTC1 PRESCALER register. */

// Macros and structure to use for Polled Periodic tasks. Great for ensuring a minimum time
// between tasks. Potentially quite sloppy, system time resolution can be as high as 125 ms.
// Setting period to 0 will disable the periodic task.
uint32_t getSystemTimeMs( void );
typedef struct {
	uint32_t start_time;
	uint32_t period;
} TTASK_TIMER;
#define stop_task_timer(x)		x.period = 0;
#define start_task_timer(x,y)	x.period = y; x.start_time = getSystemTimeMs();
#define re_arm_task_timer(x)	x.start_time = getSystemTimeMs();
#define task_time(x)			((x.period != 0) ? (((getSystemTimeMs()-x.start_time) > x.period) ? true : false) : false)

typedef enum {
	OPMODE_LOW_POWER,
	OPMODE_RUNNING,
	OPMODE_SHIPPING,
} T_OPMODE;
		
#endif
