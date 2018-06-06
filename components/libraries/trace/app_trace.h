#ifndef __DEBUG_H_
#define __DEBUG_H_

#include <stdint.h>
#include <stdio.h>

// Debug Priority Levels
enum debug_levels {
	DEBUG_NONE = 0,
	DEBUG_HIGH = 1,
	DEBUG_MED = 2,
	DEBUG_LOW = 3,
};
typedef enum debug_levels T_DEBUG_PRIORITY;

/**
 * @defgroup app_trace Debug Logger
 * @ingroup app_common
 * @{
 * @brief Enables debug logs/ trace over UART.
 * @details Enables debug logs/ trace over UART. Tracing is enabled only if 
 *          ENABLE_DEBUG_LOG_SUPPORT is defined in the project.
 */
#ifdef ENABLE_DEBUG_LOG_SUPPORT
#include "nrf_log.h"
#include <stdarg.h>

/**
 * @brief Module Initialization.
 *
 * @details Initializes the module to use UART as trace output.
 * 
 * @warning This function will configure UART using default board configuration. 
 *          Do not call this function if UART is configured from a higher level in the application. 
 */
void app_trace_init(void);

T_DEBUG_PRIORITY app_get_debug_priority( void );
bool app_set_min_debug_priority( T_DEBUG_PRIORITY pri );
	
/**
 * @brief Log debug messages.
 *
 * @details This API logs messages over UART. The module must be initialized before using this API.
 *
 * @note Though this is currently a macro, it should be used and treated as function.
 */
#define app_trace_log( pri, msg, ...) 	if( (unsigned) pri <= app_get_debug_priority() ) NRF_LOG_PRINTF(msg, ##__VA_ARGS__)

/**
 * @brief Log debug string messages. Faster than NRF_LOG_PRINTF because it only handles Strings, no formatting
 *
 * @details This API logs messages over UART. The module must be initialized before using this API.
 *
 * @note Though this is currently a macro, it should be used and treated as function.
 */
#define app_trace_puts( pri, msg, ...) 	if( (unsigned) pri <= app_get_debug_priority() ) NRF_LOG(msg, ##__VA_ARGS__)

/**
 * @brief Send data logged in RTT to a remote console instead of being retrieved by a debugger
 *
 * @details This API pulls data from the RTT log buffer
 *
 * @note Returns the length of data pulled from the buffer
 */
#define app_trace_remote NRF_LOG_REMOTE

/**
 * @brief Dump auxiliary byte buffer to the debug trace.
 *
 * @details This API logs messages over UART. The module must be initialized before using this API.
 * 
 * @param[in] p_buffer  Buffer to be dumped on the debug trace.
 * @param[in] len       Size of the buffer.
 */
void app_trace_dump(uint8_t pri, uint8_t * p_buffer, uint32_t len);

#else // ENABLE_DEBUG_LOG_SUPPORT

#define app_trace_init(...)
#define app_trace_log(...)
#define app_trace_dump(...)
#define app_trace_puts(...)

#endif // ENABLE_DEBUG_LOG_SUPPORT

/** @} */

#endif //__DEBUG_H_
