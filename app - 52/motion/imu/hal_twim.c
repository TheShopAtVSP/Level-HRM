/*
 * hal_twin.c
 *
 * Created: 7/31/2014 9:43:28 AM
 *  Author: RichKl
 */ 

#include <stdlib.h>
#include <string.h>
#include "hal_twim.h"
#include "app_util_platform.h"
#include "timing.h"

#define RX 	0
#define TX 	1

#define CLR 0
#define SET 1
#define GET 2

//
static bool twim_debug = false;
static bool xfer_done = true;
static ret_code_t transfer_res;
static expire_timer_t to;

static const nrf_drv_twi_t m_twim_master = NRF_DRV_TWI_INSTANCE(0);
void twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
	
//
//
//
ret_code_t hal_twim_init( bool debug_twim )
{
	ret_code_t ret;
	
	twim_debug = debug_twim;
	
	if (twim_debug) app_trace_log(DEBUG_LOW, "hal_twim_init: start\r\n");	
	
	const nrf_drv_twi_config_t config =
	{
		.scl                = TWIM_SCL_M,
		.sda                = TWIM_SDA_M,
		.frequency          = NRF_TWI_FREQ_250K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH
	};

	ret = nrf_drv_twi_init(&m_twim_master, &config, twim_handler, NULL);
	if( ret == NRF_SUCCESS) {		
		nrf_drv_twi_enable(&m_twim_master);
		if (twim_debug) app_trace_log(DEBUG_LOW, "hal_twim_init: done\r\n");
	}
	else {
		if (twim_debug) app_trace_log(DEBUG_HIGH, "twi_master_init: failed\r\n");	
	}
	
	return ret;
}

//
//
//
void hal_twim_uninit( void )
{
	nrf_drv_twi_uninit( &m_twim_master );
}

/**
 * @brief TWI events handler.
 */
void twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
//    ret_code_t err_code;
    
	if( p_event->type == NRF_DRV_TWI_EVT_DONE ) {
		//Data was successfully transferred
		transfer_res = NRF_SUCCESS;
	}
	else if( p_event->type == NRF_DRV_TWI_EVT_ADDRESS_NACK ) {
		//A NACK was received on the Address
		transfer_res = NRF_ERROR_INVALID_ADDR;
		if(twim_debug) app_trace_log(DEBUG_HIGH, "hal_twim: ERR_BAD_ADDRESS\r\n");
	}
	else {
		//A NACK was received on the data
		transfer_res = NRF_ERROR_TIMEOUT;
		if(twim_debug) app_trace_log(DEBUG_HIGH, "hal_twim: ERR_ABORTED\r\n");
	} 
	
	xfer_done = true;
}

static bool twim_busy( uint8_t clr_set_get )
{
	static bool twim_busy_flag = false;
	static uint32_t last_used_time_ms = 0;
	
	if( (twim_busy_flag == true) && (getSystemTimeMs() - last_used_time_ms) > 1000 )
	{	//twim has not been used in more than 1 second, it should be freed
		app_trace_log(DEBUG_HIGH, "[TWIM] Block Time Out\r\n");
		twim_busy_flag = false;
	}
	
	switch( clr_set_get )
	{
		case 0:		//Clear
			twim_busy_flag = false;
			break;
		
		case 1:		//Set
			if( twim_busy_flag == false )
			{
				twim_busy_flag = true;
				last_used_time_ms = getSystemTimeMs();
			}
			break;
				
		default:	//Get
			break;
	}
	
	return twim_busy_flag;
}

static ret_code_t hal_twim_xfer( T_TWIM_PACKET * pkt, bool rxtx )
{		
	ret_code_t res;
	uint16_t length = pkt->length + 1;
	uint8_t outdata[length];
	
	transfer_res = NRF_SUCCESS;
	xfer_done = false;
	if( rxtx == TX )
	{
		//Combine the Starting Write Address and Write Data into 1 buffer that can be pointed to
		memcpy( (void *) &outdata[1], (void *) pkt->buffer, pkt->length );
		outdata[0] = pkt->reg_addr;
		res = nrf_drv_twi_tx(&m_twim_master, pkt->i2c_id, outdata, length, false);	
	}
	else
	{
		nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX( pkt->i2c_id, (uint8_t*)&pkt->reg_addr, 1, (uint8_t*)pkt->buffer, pkt->length );
		res = nrf_drv_twi_xfer( &m_twim_master, &xfer, false );
	}

	if( res == NRF_SUCCESS) 
	{
		//wait upto 250 ms for transfer to complete
		get_expire_time( (250*1000UL), &to );	//Max time that a routine may lock up twim	
		while( xfer_done == false ) 
		{
			if( check_expiration( &to ) )
			{
				transfer_res = NRF_ERROR_TIMEOUT;
				break;
			}
		}
	}
	else 
	{
		transfer_res = res;
	}
	
	return transfer_res;
}

ret_code_t hal_twim_write(T_TWIM_PACKET * pkt)
{
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t ret;
	
	if( twim_busy(GET) == false )
	{
		twim_busy(SET);
		ret = hal_twim_xfer( pkt, TX );
		twim_busy(CLR);
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "[TWIM] Busy!!!\r\n");
		ret = NRF_ERROR_BUSY;
	}

	if( ret != NRF_SUCCESS ) 
	{
		if( prv_res != ret)
		{
			app_trace_log(DEBUG_HIGH, "[TWIM_WR] Err %01d @%01u\r\n", ret, getSystemTimeMs());	
		}
	}
	prv_res = ret;
	
	return ret;
}

ret_code_t hal_twim_read(T_TWIM_PACKET * pkt)
{
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t ret;
	
	if( twim_busy(GET) == false )
	{
		twim_busy(SET);
		ret = hal_twim_xfer( pkt, RX );
		twim_busy(CLR);
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "[TWIM] Busy!!!\r\n");
		ret = NRF_ERROR_BUSY;
	}
	
	if( ret != NRF_SUCCESS ) 
	{
		if( prv_res != ret)
		{
			app_trace_log(DEBUG_HIGH, "[TWIM_RD] Err %01d @%01u\r\n", ret, getSystemTimeMs());	
		}
	}
	prv_res = ret;
	
	return ret;
}
