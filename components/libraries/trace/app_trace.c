/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>


#ifdef ENABLE_DEBUG_LOG_SUPPORT
#include "app_trace.h"
#include "nrf_log.h"

static T_DEBUG_PRIORITY minimum_priority = DEBUG_MED;

void app_trace_init(void)
{
    (void)NRF_LOG_INIT();
}

__INLINE T_DEBUG_PRIORITY app_get_debug_priority( void )
{	
	return minimum_priority;
}

bool app_set_min_debug_priority( T_DEBUG_PRIORITY pri )
{
	//force requested priority to be valid
	if( (uint16_t) pri > DEBUG_LOW ) return false;
	
	minimum_priority = pri;
	
	return true;
}

void app_trace_dump(uint8_t priority, uint8_t * p_buffer, uint32_t len)
{
	if( priority <= minimum_priority ) {
		NRF_LOG("\r");
		for (uint32_t index = 0; index <  len; index++)
		{
			NRF_LOG_PRINTF("0x%02X ", p_buffer[index]);
		}
		NRF_LOG("\r");
	}
}

#endif // ENABLE_DEBUG_LOG_SUPPORT

/**
 *@}
 **/

