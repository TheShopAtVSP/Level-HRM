/** @file
 *
 * @defgroup Defines globally used functions and variables
 * @{
 * @ingroup
 *
 */

#ifndef TIMING_H
#define TIMING_H

#include "sdk_errors.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nordic_common.h"

typedef struct
{
	uint32_t wait_a_ticks;
	uint32_t expiration;
} expire_timer_t;

void unix_timer_init( void );
uint32_t get_unix_time( void );
void set_unix_time( uint32_t new_time );
uint32_t getSystemTimeMs( void );
void get_expire_time( uint32_t wait_time_us, expire_timer_t * timer );
bool check_expiration( expire_timer_t * timer );
void cancel_expire_time( expire_timer_t * timer );
	
#endif /* TIMING_H */
/** @} */
