

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
#include "timing.h"

//#define APP_TRACE_ON

//Conditional Compile for the different Hardware versions
//#define 	LEVEL_1_0
#define 	LEVEL_1_1
#ifdef LEVEL_1_0
	#define COMPILED_HW_REV		"C"
#else
	//bump version if hardware is modified
	//#define HW_REV				"D"
	#define COMPILED_HW_REV		"F"		//BOM modifictions: BQ25120A and a few resistor values
	//#define COMPILED_HW_REV		"G"		//BOM modifictions: BQ25120 with Slap-A-Cap
#endif	

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

// Keep as a (power of 2)-1 so timing is precise for unix_time (0 seems to cause occasional startup glitches)
// Tick generated every (APP_TIMER_PRESCALER + 1)/32768 = (15 + 1)/32768 = 488 us
#define APP_TIMER_PRESCALER		15	/**< Value of the RTC1 PRESCALER register. */

typedef enum {
	OPMODE_LOW_POWER,
	OPMODE_RUNNING,
	OPMODE_SHIPPING,
} T_OPMODE;
#endif
