

#ifndef GLOBAL_H__
#define GLOBAL_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf_delay.h"
#include "app_trace.h"

#pragma anon_unions

#define 	PASS      			0
#define 	FAIL      			1
#define 	OFF      			0
#define 	ON      			1
#define	  	delay_ms(A) 		nrf_delay_ms(A)
#define 	true				1
#define		false 				0
#define 	min(a,b) 			((a<b)?a:b)

typedef 	_Bool				Bool;
typedef 	unsigned int 		uint;

#define		BREAK_POINT			__asm volatile ("nop");			//Use to force a breakable location

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
// between tasks. Potential quite sloppy, system time resolution can be as high as 125 ms.
// Setting period to 0 will disable the periodic task.
uint32_t getSystemTimeMs( void );
typedef struct {
	uint32_t timer;
	uint32_t period;
} TTASK_TIMER;
#define re_arm_timer(x)		x.timer = getSystemTimeMs();
#define restart_timer(x,y)	x.period = y; x.timer = getSystemTimeMs();
#define task_time(x)		(x.period != 0) ? (((getSystemTimeMs()-x.timer) > x.period) ? true : false) : false


enum status_code {
	ERR_NONE               	=  0, //!< Success
	STATUS_ERR_BUSY         =  0x19,
	STATUS_ERR_DENIED       =  0x1C,
	STATUS_ERR_TIMEOUT      =  0x12,
	ERR_IO_ERROR            =  -1, //!< I/O error
	ERR_FLUSHED             =  -2, //!< Request flushed from queue
	ERR_TIMEOUT             =  -3, //!< Operation timed out
	ERR_BAD_DATA            =  -4, //!< Data integrity check failed
	ERR_PROTOCOL            =  -5, //!< Protocol error
	ERR_UNSUPPORTED_DEV     =  -6, //!< Unsupported device
	ERR_NO_MEMORY           =  -7, //!< Insufficient memory
	ERR_INVALID_ARG         =  -8, //!< Invalid argument
	ERR_BAD_ADDRESS         =  -9, //!< Bad address
	ERR_BUSY                =  -10, //!< Resource is busy
	ERR_BAD_FORMAT          =  -11, //!< Data format not recognized
	ERR_NO_TIMER            =  -12, //!< No timer available
	ERR_TIMER_ALREADY_RUNNING   =  -13, //!< Timer already running
	ERR_TIMER_NOT_RUNNING   =  -14, //!< Timer not running
	ERR_ABORTED             =  -15, //!< Operation aborted by user
	ERR_DEVICE_DISABLED     =  -16, //!< Device Not Running
	/**
	 * \brief Operation in progress
	 *
	 * This status code is for driver-internal use when an operation
	 * is currently being performed.
	 *
	 * \note Drivers should never return this status code to any
	 * callers. It is strictly for internal use.
	 */
	OPERATION_IN_PROGRESS	= -128,
};

typedef enum status_code status_code_t;

#ifdef NOMESSAGES
	DEBUG_PRINT(ERR_CODE,MSG,DISPLAY); 
#else
#define DEBUG_PRINT(ERR_CODE,MSG,DISPLAY)           		\
	do                                                      \
    {   													\
															\
		if (DISPLAY) {										\
			const uint32_t LOCAL_ERR_CODE = (ERR_CODE); 	\
			if (LOCAL_ERR_CODE != 0 )	{					\
				app_trace_s_msg(MSG);								\
				app_trace_s_msg(" failed\r");						\
			}												\
			else {											\
				app_trace_s_msg(MSG);								\
				app_trace_s_msg("\r");								\
			}												\
		}													\
    } while (0);											\
		
	
#endif
#endif
