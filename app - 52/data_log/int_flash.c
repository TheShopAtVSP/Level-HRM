/*
 * int_flash.c
 *
 * Created: 3/20/2017 3:13:07 PM
 *  Author: matt
 */ 

#include <string.h>
#include "../global.h"
#include "hal_nvm.h"
#include "int_flash.h"
#include "nrf_soc.h"

//Global
extern T_CONFIG g_config;
extern uint32_t fw_rev;

//#define INIT_FLASH

#define STORE_TIMEOUT			500						//All store operations will complete within this time

static bool flash_debug = false;

//
// pstorage
//
static pstorage_handle_t  	config_handle;
static volatile bool		wait_config_cb[5];
static pstorage_handle_t  	log_handle;

//local copy of opcodes shifted to start at 0
enum {
	STORE_OP_CODE = 0,
	LOAD_OP_CODE,
	CLEAR_OP_CODE,
	UPDATE_OP_CODE,
	
	OP_CODE_CNT
};
struct {
	uint wait:		1;
	uint success:	1;
} config_cb[OP_CODE_CNT];
struct {
	uint wait:		1;
	uint success:	1;
} log_cb[OP_CODE_CNT];

static uint32_t init_log_region( void );
static uint32_t init_config_region( void );

//Return the offset to the start of a Page
static uint32_t get_page_start_offset( uint32_t offset )
{
	return (offset & (~PAGE_MASK));
}

//Return the len into a page
static uint16_t get_page_remainder( uint32_t offset )
{
	return offset & PAGE_MASK;
}

//
static void config_cb_handler(pstorage_handle_t * handle,uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len)
{	
	switch(op_code)
	{
		case PSTORAGE_STORE_OP_CODE:
			config_cb[STORE_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (flash_debug) app_trace_log(DEBUG_LOW, "cfg_cb: stored\r");
				config_cb[STORE_OP_CODE].success = true;
				
			}
			else {
				app_trace_log(DEBUG_MED, "cfg_cb: store failed\r");
				config_cb[STORE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
			
		case PSTORAGE_LOAD_OP_CODE:
			config_cb[LOAD_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (flash_debug) app_trace_log(DEBUG_LOW, "cfg_cb: loaded\r");
				config_cb[LOAD_OP_CODE].success = true;
			}
			else {
				app_trace_log(DEBUG_MED, "cfg_cb: load failed\r");
				config_cb[LOAD_OP_CODE].success = false;
			}
			break;
		
		case PSTORAGE_CLEAR_OP_CODE:
			config_cb[CLEAR_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (flash_debug) app_trace_log(DEBUG_LOW, "cfg_cb: cleared\r");
				config_cb[CLEAR_OP_CODE].success = true;
			}
			else {
				app_trace_log(DEBUG_MED, "cfg_cb: clear failed\r");
				config_cb[CLEAR_OP_CODE].success = false;
			}
			break;
			
		case PSTORAGE_UPDATE_OP_CODE:
			config_cb[UPDATE_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (flash_debug) app_trace_log(DEBUG_LOW, "cfg_cb: updated\r");
				config_cb[UPDATE_OP_CODE].success = true;
				
			}
			else {
				app_trace_log(DEBUG_MED, "cfg_cb: update failed\r");
				config_cb[UPDATE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
		
		default:
			app_trace_log(DEBUG_MED, "cfg_cb: ?\r");
			config_cb[STORE_OP_CODE].wait = false;
			config_cb[LOAD_OP_CODE].wait = false;
			config_cb[CLEAR_OP_CODE].wait = false;
			config_cb[UPDATE_OP_CODE].wait = false;
			break;
    }
}

//Start Up onboard Storage 
ret_code_t init_flash_i( bool debug ) 
{	
	ret_code_t ret;
	
	flash_debug = debug;
	
	//Onboard Storage uses pStore
	ret = pstorage_init();	//should already be started by Device Manager
	if( ret == NRF_SUCCESS) 
	{
		//pStore mem module to init first (places it at the beginning of the persistent storage region)
		init_log_region();	
		
		//pStore module to init last (so it will be placed at the last available memory storage page)
		if( init_config_region()== NRF_SUCCESS )
		{
			//load config data into RAM
			memcpy( (uint8_t *)&g_config, (uint8_t *)config_handle.block_id, T_CONFIG_LEN );
		}
	}

	return ret;
}

static ret_code_t init_config_region(void)
{
	pstorage_module_param_t param;
	ret_code_t retval, block_size;
	
	// must reserve sizes in word aligned increments
	block_size = force_word_aligned(T_CONFIG_LEN);
	
    // register pstorage 	
	param.block_size  = block_size;
	param.block_count = 1;
	param.cb          = config_cb_handler;
	retval = pstorage_register(&param, &config_handle);
	if ( retval == NRF_SUCCESS ) 
	{
		if( flash_debug ) app_trace_log(DEBUG_LOW, "init_cfg_region: Storage Handle: 0x%05X\r", (uint) config_handle.block_id);
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "init_cfg_region: Storage Handle Failed %01u\r",retval);
		return retval;
	}
	
	if( retval != NRF_SUCCESS )
	{
		app_trace_log(DEBUG_MED, "init_cfg_region: Register Failed\r");
	}

	return retval;	
}

//check for changes to config memory and if any are found, save them
ret_code_t update_config_i( void )
{
	TTASK_TIMER timeout;
	ret_code_t retval = NRF_ERROR_INVALID_STATE;	
	uint8_t op_code = UPDATE_OP_CODE;
	
	start_task_timer( timeout, STORE_TIMEOUT );

	uint32_t count;
	pstorage_access_status_get( &count );
	if( count >= PSTORAGE_CMD_QUEUE_SIZE ) {
		//todo: pStore is busy, how to kick start it?
		app_trace_puts(DEBUG_HIGH, "update_cfg: PSTORE Busy\r");
		return NRF_ERROR_BUSY;
	}
		
	//check to make sure persistent data has been initialized
	if( g_config.rev == fw_rev ) 
	{	
		//Look for Data that has changed (Don't rewrite if nothing is different)
		if( memcmp( (uint8_t *)&g_config, (uint8_t *)config_handle.block_id, T_CONFIG_LEN ) != 0 ) 
		{
			uint16_t chksum = 0;
			uint8_t * data = (uint8_t *) &g_config;
			for( int i=0; i<(T_CONFIG_LEN-sizeof(g_config.chksum)); i++ )
			{
				chksum += data[i];
			}
			g_config.chksum = chksum;
				
			config_cb[op_code].wait = true;
			uint32_t update_size = force_word_aligned( T_CONFIG_LEN );
			retval = pstorage_update(&config_handle, (uint8_t *)&g_config, update_size, 0);
			//retval = pstorage_clear(&config_handle, update_size );			
			if( retval == NRF_SUCCESS ) {
				//wait until the call back verfies completion
				while( config_cb[op_code].wait == true ) {
					sd_app_evt_wait();
					if( task_time(timeout) ) {
						config_cb[op_code].success = false;
						retval = NRF_ERROR_TIMEOUT; 
						break;
					}
				}
				
				if( config_cb[op_code].success != true ) {
					app_trace_log(DEBUG_HIGH, "update_cfg: fail %01u\r", retval);
				}
				else {
					if(flash_debug) app_trace_log(DEBUG_MED, "update_cfg: complete\r");
				}
			}
			else {
				app_trace_log(DEBUG_MED, "update_cfg: failed %01u\r", retval);
			}
		}
		else {
			if(flash_debug) app_trace_log(DEBUG_LOW, "update_cfg: nothing to save\r");
			retval = NRF_SUCCESS;	
		}
	}
	else {
		app_trace_log(DEBUG_HIGH, "update_cfg: Config Not Initialized\r");
		retval = NRF_ERROR_INVALID_STATE;
	}
	
	return retval;
}


static void log_cb_handler(pstorage_handle_t * handle,uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len)
{
	switch(op_code)
	{				
		case PSTORAGE_STORE_OP_CODE:
			log_cb[STORE_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				//if (flash_debug) app_trace_log(DEBUG_LOW, "log_cb_handler: log stored\r");
				log_cb[STORE_OP_CODE].success = true;
			}
			else {
				app_trace_log(DEBUG_MED, "log_cb_handler: store failed\r");
				log_cb[STORE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
		
		case PSTORAGE_LOAD_OP_CODE:
			log_cb[LOAD_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				//if (flash_debug) app_trace_log(DEBUG_LOW, "log_cb_handler: log loaded\r");
				log_cb[LOAD_OP_CODE].success = true;
			}
			else {
				app_trace_log(DEBUG_MED, "log_cb_handler: load failed\r");
				log_cb[LOAD_OP_CODE].success = false;
			}
			break;
		
		case PSTORAGE_CLEAR_OP_CODE:
			log_cb[CLEAR_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (flash_debug) app_trace_log(DEBUG_LOW, "log_cb_handler: log cleared\r");
				log_cb[CLEAR_OP_CODE].success = true;
			}
			else {
				app_trace_log(DEBUG_MED, "log_cb_handler: update failed\r");
				log_cb[CLEAR_OP_CODE].success = false;
			}
			break;
			
		case PSTORAGE_UPDATE_OP_CODE:
			log_cb[UPDATE_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				//if (gs_bdebug) app_trace_log(DEBUG_LOW, "log_cb_handler: log updated\r");
				log_cb[UPDATE_OP_CODE].success = true;
				
			}
			else {
				app_trace_log(DEBUG_MED, "log_cb_handler: update failed\r");
				log_cb[UPDATE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
		
		default:
			app_trace_log(DEBUG_MED, "log_cb_handler: ?\r");
			log_cb[STORE_OP_CODE].wait = false;
			log_cb[LOAD_OP_CODE].wait = false;
			log_cb[CLEAR_OP_CODE].wait = false;
			log_cb[UPDATE_OP_CODE].wait = false;
			break;
    }
}

static ret_code_t init_log_region( void )
{
	pstorage_module_param_t param;
	ret_code_t retval;	
	
	// must reserve sizes in word aligned increments
	if( force_word_aligned(TOTAL_LOG_LEN) != TOTAL_LOG_LEN ) {
		app_trace_log(DEBUG_HIGH, "Invalid Log Region Size\r");
		return NRF_ERROR_INVALID_ADDR;
	}
	
    // register pstorage 
	param.block_size  = PAGE_LEN_BYTES;
	param.block_count = LOG_PAGE_CNT;
	param.cb          = log_cb_handler;
	retval = pstorage_register(&param, &log_handle);
	if ( retval == NRF_SUCCESS ) {
		if( flash_debug ) app_trace_log(DEBUG_LOW, "init_log_region: Storage Handle: 0x%05X\r", (uint)log_handle.block_id);
	}
	else{
		app_trace_log(DEBUG_HIGH, "init_log_region: Storage Handle Register Failed %01u\r",retval);
		return retval;
	}
	
	return retval;	
}

//load persistent log memory into ram
ret_code_t copy_log_i( uint8_t * data, uint16_t length, uint32_t offset )
{
	if( offset >= TOTAL_LOG_LEN ) {
		//block trying to read beyond buffer
		return NRF_ERROR_INVALID_ADDR;
	}
	
	//Read from Flash Area
	if( (offset+length) > TOTAL_LOG_LEN ) {
		//form a continuous variable that can be pointed to
		uint16_t split_len, remain_len;
		split_len = TOTAL_LOG_LEN - offset;
		remain_len = length - split_len;
		memcpy(data, (uint8_t *)(LOG_START_ADDR + offset), split_len);
		memcpy(data+split_len, (uint8_t *)LOG_START_ADDR, remain_len);
	}
	else {
		//header is not on rollover boundary, point directly to it
		memcpy(data, (uint8_t *)(LOG_START_ADDR + offset), length);
	}

	return NRF_SUCCESS;
}

ret_code_t clear_log_page_i( uint32_t block_id_offset ) 
{
	TTASK_TIMER timeout;
	uint8_t op_code = CLEAR_OP_CODE;
	ret_code_t retval;
	pstorage_handle_t temp_log_handle = log_handle;
	temp_log_handle.block_id += block_id_offset;	//handle to the memory page block that we want to modify
	
	start_task_timer( timeout, STORE_TIMEOUT );
	
	log_cb[op_code].wait = true;
	retval = pstorage_clear(&temp_log_handle, PAGE_LEN_BYTES);	
	if( retval == NRF_SUCCESS ) {
		//wait until the call back verfies completion
		while( log_cb[op_code].wait == true ) {
			sd_app_evt_wait();
			if( task_time(timeout) ) {
				app_trace_log(DEBUG_MED, "clear_log to: %01u, %01u\r", getSystemTimeMs(), timeout.start_time );
				log_cb[op_code].success = false;
				retval = NRF_ERROR_TIMEOUT; 
				break;
			}
		}
	}
	
	return retval;
}

//save changes to a partial page
ret_code_t store_log_i( uint8_t * data, uint16_t length, uint32_t offset )
{
	TTASK_TIMER timeout;
	ret_code_t retval;	
	pstorage_handle_t temp_log_handle = log_handle;
	uint8_t op_code = STORE_OP_CODE;
	uint16_t page_remainder = get_page_remainder( offset );
	
	start_task_timer( timeout, STORE_TIMEOUT );
	
	uint32_t count;
	pstorage_access_status_get( &count );
	if( count >= PSTORAGE_CMD_QUEUE_SIZE ) {
		//todo: pStore is busy, how to kick start it?
		//pstorage_init();
		//init_record();
		//init_config();	//pStore module to init last (so it will be placed at the last available memory page)
		app_trace_puts(DEBUG_MED, "store_log: PSTORE Busy\r");
		return NRF_ERROR_BUSY;
	}
	
	if( (page_remainder + length) > PAGE_LEN_BYTES ) {
		//Data is too long to be saved. It overflows into the next page
		return NRF_ERROR_INVALID_LENGTH;
	}
	
	temp_log_handle.block_id += get_page_start_offset( offset );

	log_cb[op_code].wait = true;
	retval = pstorage_store(&temp_log_handle, data, length, page_remainder);	
	if( retval == NRF_SUCCESS ) {
		//wait until the call back verfies completion
		while( log_cb[op_code].wait == true ) {
			sd_app_evt_wait();
			if( task_time(timeout) ) {
				app_trace_log(DEBUG_MED, "save_log to: %01u, %01u\r", getSystemTimeMs(), timeout.start_time );
				log_cb[op_code].success = false;
				retval = NRF_ERROR_TIMEOUT; 
				break;
			}
		}
		
		if( log_cb[op_code].success != true ) {
			app_trace_log(DEBUG_MED, "save_log_cb: failed %01u\r", retval);
		}
	}
	else {
		app_trace_log(DEBUG_MED, "save_log: failed %01u\r", retval);
	}
	
	return retval;
}
