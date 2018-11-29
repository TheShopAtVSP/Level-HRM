/*
 * hal_spim.c
 *
 * Created: 3/27/17 9:47:00 AM
 *  Author: Matt Workman
 */ 

#include <stdlib.h>
#include <string.h>
#include "hal_spim.h"
#include "app_timer.h"
#include "app_util_platform.h"

#define SPIM_250MS_TO	( APP_TIMER_TICKS(250, APP_TIMER_PRESCALER) )	//Give calling function 250 ms to complete transaction
APP_TIMER_DEF(m_SPIM_timer_id);
	
#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spim = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static bool spim_busy = false;  /**< Flag used to indicate that SPI instance completed the transfer. */
static bool spim_debug = false;
static bool spim_error = false;

spim_cb xfer_complete_cb = NULL;
nrf_drv_spi_config_t spim_config = NRF_DRV_SPI_DEFAULT_CONFIG( SPI_INSTANCE );
static T_HAL_SPIM_XFER xfer_cpy;
static uint16_t xfer_total;
static uint8_t xfer_dummy_cnt = 0;

struct {
	uint32_t pin;
	bool leave_assert;
} cs = { NRF_DRV_SPI_PIN_NOT_USED, false };

static void spim_handler(nrf_drv_spi_evt_t const * p_event);
static void spim_timer_handler(void * p_context);
static ret_code_t spim_xfer( void );
	
//
//
//
ret_code_t hal_spim_init( bool debug )
{
	ret_code_t ret;
	
	spim_busy = false;
	spim_debug = debug;
	
	if (spim_debug) app_trace_log(DEBUG_LOW, "hal_spim_init: start\r\n");	
	
	ret = app_timer_create( &m_SPIM_timer_id, APP_TIMER_MODE_SINGLE_SHOT, spim_timer_handler );
	APP_ERROR_CHECK(ret);
		
	spim_config.mode = NRF_DRV_SPI_MODE_0;	///< SCK active high, sample on leading edge of clock.
	spim_config.frequency = NRF_DRV_SPI_FREQ_8M;
    ret = nrf_drv_spi_init(&spim, &spim_config, spim_handler);
	if( ret == NRF_SUCCESS) {		
		if (spim_debug) app_trace_log(DEBUG_LOW, "hal_spim_init: done\r\n");
	}
	else {
		if (spim_debug) app_trace_log(DEBUG_HIGH, "spi_master_init: failed\r\n");	
	}	
	return ret;
}

//
//
//
void hal_spim_uninit( void )
{
	nrf_drv_spi_uninit( &spim );
}

//
//
//
static void spim_timer_start( void )
{
	if (spim_debug) app_trace_log(DEBUG_LOW, "   [SPIM] Timer On @%01u\r\n", getSystemTimeMs());
	ret_code_t res = app_timer_start( m_SPIM_timer_id, SPIM_250MS_TO, NULL );	
	APP_ERROR_CHECK(res);
}

//
//
//
void spim_timer_stop( void )
{
	if (spim_debug) app_trace_log(DEBUG_LOW, "   [SPIM] Timer Off @%01u\r\n", getSystemTimeMs());
	ret_code_t res = app_timer_stop( m_SPIM_timer_id );
	APP_ERROR_CHECK(res);
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spim_timer_handler(void * p_context)
{
    if (spim_debug) app_trace_log(DEBUG_MED, "   [SPIM] Time Out @%01u\r\n", getSystemTimeMs());
	
	//Release lock on spim module
	spim_busy = false;
	
	//Deassert Chip Select line
	if( cs.pin != NRF_DRV_SPI_PIN_NOT_USED ) nrf_gpio_pin_set( cs.pin );

	//False that an error has occurred
	spim_error = true;
	
	if( xfer_complete_cb != NULL ) 
	{	//Inform of failure, some bytes may have transferred
		xfer_complete_cb( false, xfer_total );
	}
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spim_handler(nrf_drv_spi_evt_t const * p_event)
{
	//increment buffers based off of what was transmitted:
	int32_t rx_len = (p_event->data.done.rx_length - xfer_dummy_cnt);
	xfer_total += rx_len;
	xfer_cpy.p_tx_buf += p_event->data.done.tx_length;	
	xfer_cpy.p_rx_buf += rx_len;
	xfer_cpy.tx_len -= p_event->data.done.tx_length;
	if( xfer_cpy.tx_len < 0 ) xfer_cpy.tx_len = 0;
	xfer_cpy.rx_len -= rx_len;
	if( xfer_cpy.rx_len < 0 ) xfer_cpy.rx_len = 0;
	
	if( xfer_cpy.tx_len > 0 || xfer_cpy.rx_len > 0 )
	{	//transfer is not yet complete, continue transferring data:
		spim_xfer();
	}
	else
	{
		//Release lock on spim module
		spim_busy = false;
		
		if( !cs.leave_assert )
		{	//Deassert Chip Select line unless ordered not to by calling function
			if( cs.pin != NRF_DRV_SPI_PIN_NOT_USED ) nrf_gpio_pin_set( cs.pin );
			spim_timer_stop();
		}
		else
		{	//Give a finite amount of time for the calling function to finish up
			//whatever business it needs with the CS line.
		}

		if( xfer_complete_cb != NULL ) 
		{
			xfer_complete_cb( true, xfer_total );
		}
	}
}

static ret_code_t spim_xfer( void )
{
	ret_code_t res;
	uint8_t tx_len, rx_len;
	nrf_drv_spi_xfer_desc_t xfer_desc;
	
	//The DMA can only handle upto 255 Bytes. To transfer more data requires multiple transactions
	if( xfer_cpy.tx_len > 255 ) tx_len = 255;
	else if( xfer_cpy.tx_len < 0 ) tx_len = 0;
	else tx_len = xfer_cpy.tx_len;
	if( xfer_cpy.rx_len > 255 )	rx_len = 255;
	else if( xfer_cpy.rx_len < 0 ) rx_len = 0;
	else rx_len = xfer_cpy.rx_len;
	
	//Read fails when RX is 1 byte, so clock a dummy byte:
	if( rx_len == 1 ) 
	{
		xfer_dummy_cnt = 1;
		rx_len = 2;	
	}
	else
	{
		xfer_dummy_cnt = 0;
	}
		
    xfer_desc.p_tx_buffer = xfer_cpy.p_tx_buf;
    xfer_desc.p_rx_buffer = xfer_cpy.p_rx_buf;
    xfer_desc.tx_length   = tx_len;
    xfer_desc.rx_length   = rx_len;
	
	//app_trace_log(DEBUG_LOW, "   [SPIM] TL:0x%01X RL:0x%01X\r\n", xfer_cpy.tx_len, xfer_cpy.rx_len);
    res = nrf_drv_spi_xfer(&spim, &xfer_desc, 0);
	if( res != NRF_SUCCESS )
	{	//XFER failure!
		app_trace_log(DEBUG_HIGH, "   [SPIM] SPI Err: 0x%04X\r\n", res);
		
		spim_busy = false;
		
		//No response, make sure Chip Select gets deactivated
		if( cs.pin != NRF_DRV_SPI_PIN_NOT_USED ) nrf_gpio_pin_set( cs.pin );
		
		//flag an error
		spim_error = true;
		
		if( xfer_complete_cb != NULL ) 
		{	//Inform of failure, some bytes may have transferred...
			xfer_complete_cb( false, xfer_total );
		}
	}
	
	return res;
}

ret_code_t hal_spim_xfer( T_HAL_SPIM_XFER * data )
{
	static uint32_t last_used_time_ms = 0;
	ret_code_t res;
	
	//Check if an error has been detected, if so try to restart spim module:
	if( spim_error )
	{
		app_trace_log(DEBUG_MED, "   [SPIM] Repair");
		spim_error = false;
		
		res = nrf_drv_spi_repair(&spim, &spim_config, spim_handler);
		if( res == NRF_SUCCESS )
		{
			app_trace_log(DEBUG_MED, " CMPLT\r\r\r\n");
		}
		else
		{
			app_trace_log(DEBUG_HIGH, " Failure %01u\r\r\r\n", res);
		}
		
		// Stall for testing failures
		//while( 1 );
	}
		
	//If module is already transmitting, make sure that is indeed the case:
	if( spim_busy ) 
	{
	   if( (getSystemTimeMs() - last_used_time_ms) > 1000 )
		{	//spim has not been used in more than 1 second, it should be free
			app_trace_log(DEBUG_HIGH, "   [SPIM] Lock Time Out\r\n");
			spim_busy = false;
		}
		else
		{	//spim is potentially still busy
			app_trace_log(DEBUG_HIGH, "   [SPIM] Locked\r\n");
			return NRF_ERROR_BUSY;
		}
	}
	
	last_used_time_ms = getSystemTimeMs();
	spim_busy = true;
	memcpy( (void *) &xfer_cpy, (void *) data, sizeof(T_HAL_SPIM_XFER) );
	
	xfer_total = 0;					//reset total for this transfer
	xfer_complete_cb = xfer_cpy.cb;
	cs.pin = xfer_cpy.cs_pin;
	cs.leave_assert = xfer_cpy.hold_cs_pin;
	
	//Set time limit for transfer to complete
	spim_timer_start();	

	//Activate Chip Select
	if( cs.pin != NRF_DRV_SPI_PIN_NOT_USED ) nrf_gpio_pin_clear( cs.pin );
	
	res = spim_xfer();
	
	return res;
}


