/*
 * hal_twin.c
 *
 * Created: 7/31/2014 9:43:28 AM
 *  Author: RichKl
 */ 

#include <stdlib.h>
#include <string.h>
#include "global.h"
#include "hal_twim.h"
#include "app_util_platform.h"

//
static bool twim_debug = false;
static bool xfer_done = true;
static ret_code_t transfer_res;
static TTASK_TIMER to;

static const nrf_drv_twi_t m_twim_master = NRF_DRV_TWI_INSTANCE(0);
void twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

//
//
//
ret_code_t hal_twim_init( bool debug_twim )
{
	ret_code_t ret;
	
	twim_debug = debug_twim;
	
	if (twim_debug) app_trace_log(DEBUG_LOW, "hal_twim_init: start\r");	
	
	const nrf_drv_twi_config_t config =
	{
		.scl                = TWIM_SCL_M,
		.sda                = TWIM_SDA_M,
		.frequency          = NRF_TWI_FREQ_400K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH
	};

	ret = nrf_drv_twi_init(&m_twim_master, &config, twim_handler, NULL);
	if( ret == NRF_SUCCESS) {		
		nrf_drv_twi_enable(&m_twim_master);
		if (twim_debug) app_trace_log(DEBUG_LOW, "hal_twim_init: done\r");
	}
	else {
		if (twim_debug) app_trace_log(DEBUG_HIGH, "twi_master_init: failed\r");	
	}
	
	return ret;
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
		if(twim_debug) app_trace_log(DEBUG_HIGH, "hal_twim: ERR_BAD_ADDRESS\r");
	}
	else {
		//A NACK was received on the data
		transfer_res = NRF_ERROR_TIMEOUT;
		if(twim_debug) app_trace_log(DEBUG_HIGH, "hal_twim: ERR_ABORTED\r");
	} 
	
	xfer_done = true;
}

//
//
//
void hal_twim_uninit( void )
{
	nrf_drv_twi_uninit( &m_twim_master );
}

ret_code_t hal_twim_write(T_TWIM_PACKET * pkt)
{
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t ret = NRF_ERROR_INTERNAL;
	uint16_t length = pkt->length + 1;
	uint8_t outdata[length];
	
	//Combine the Starting Write Address and Write Data into 1 buffer that can be pointed to
	outdata[0] = pkt->reg_addr;
	for (int i=0;i<pkt->length;i++) {
		outdata[i+1] = *pkt->buffer;
		pkt->buffer++;
	}
	
	transfer_res = NRF_SUCCESS;
	xfer_done = false;
	ret = nrf_drv_twi_tx(&m_twim_master, pkt->i2c_id, outdata, length, false);	
	
	if( ret == NRF_SUCCESS ) {
		start_task_timer( to, 250);		//wait upto 250 ms for transfer to complete
		while( xfer_done == false ) {
			if( task_time(to) ) {
				transfer_res = NRF_ERROR_TIMEOUT;
				break;
			}
		}
	}
	else {
		transfer_res = ret;
	}
	
	if( transfer_res != NRF_SUCCESS ) {
		if( prv_res != transfer_res)
		{
			app_trace_log(DEBUG_HIGH, "[TWIM_WR] Err %01d @%01u\r", transfer_res, getSystemTimeMs());	
		}	
	}
	prv_res = transfer_res;
	
	return transfer_res;
}

ret_code_t hal_twim_read(T_TWIM_PACKET * pkt)
{
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t ret = NRF_ERROR_INTERNAL;
	
	transfer_res = NRF_SUCCESS;
	xfer_done = false;
	nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX( pkt->i2c_id, (uint8_t*)&pkt->reg_addr, 1, (uint8_t*)pkt->buffer, pkt->length );
    ret = nrf_drv_twi_xfer( &m_twim_master, &xfer, false );
	
	if( ret == NRF_SUCCESS) {
		start_task_timer( to, 250);		//wait upto 250 ms for transfer to complete
		while( xfer_done == false ) {
			if( task_time(to) ) {
				transfer_res = NRF_ERROR_TIMEOUT;
				break;
			}
		}
	}
	else {
		transfer_res = ret;
	}
	
	if( transfer_res != NRF_SUCCESS ) {
		if( prv_res != transfer_res)
		{
			app_trace_log(DEBUG_HIGH, "[TWIM_RD] Err %01d @%01u\r", transfer_res, getSystemTimeMs());	
		}
	}
	prv_res = transfer_res;
	
	return transfer_res;
}


