/*
 * hal_spim.h
 *
 * Created: 3/27/2017 9:43:00 AM
 *  Author: Matt Workman
 */ 


#ifndef HAL_SPIM_H_
#define HAL_SPIM_H_

#include <stdint.h>
#include "../global.h"
#include "sdk_errors.h"
#include "nrf_drv_spi.h"

typedef void (*spim_cb) ( bool success, uint16_t rx_len );

typedef struct 
{
	uint8_t * p_tx_buf;
	int16_t tx_len;
	uint8_t * p_rx_buf;
	int16_t rx_len;
	spim_cb cb;
	uint32_t cs_pin;
	bool hold_cs_pin;
} T_HAL_SPIM_XFER;

ret_code_t hal_spim_init( bool debug_spim );
ret_code_t hal_spim_xfer( T_HAL_SPIM_XFER * data );
void spim_timer_stop( void );

#endif /* HAL_SPIM_H_ */



