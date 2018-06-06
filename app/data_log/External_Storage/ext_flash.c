/*
 * ext_flash.c
 *
 * Created: 3/29/2017 3:13:07 PM
 *  Author: matt
 */ 

#include <string.h>
#include "../global.h"
#include "hal_nvm.h"
#include "ext_flash.h"
#include "bsp.h"
#include "spiflash.h"

//Global
extern T_CONFIG g_config;
extern uint32_t fw_rev;

#define INIT_FLASH

//Times to be Used with polled TASK_TIMERS
#define STORE_TIMEOUT			500												//All store operations will complete within this time

static bool flash_debug = false;

T_SPIREAD_BUF * spiread = NULL;

//function prototypes:
static void update_config_cb( bool success );

////Return the address to the start of a 256 byte Page
//static uint32_t get_page_start( uint32_t addr )
//{
//	return (addr & (~PAGE_MASK));
//}

////Return the address to the start of a 4 KB block
//static uint32_t get_4k_block_start( uint32_t addr )
//{
//	return (addr & (~SMALL_BLOCK_MASK));
//}

////Return the len into a page
//static uint16_t get_page_remainder( uint32_t addr )
//{
//	return (addr & PAGE_MASK);
//}

//Start up external Nonvolatile Storage Device
ret_code_t init_ext_flash( bool debug ) 
{	
	ret_code_t ret;
	spiflash_id_t id;
	
	flash_debug = debug;
	
	//Deactivate HOLD pin 
	nrf_gpio_pin_clear( SPI_HOLD_PIN );
	nrf_gpio_cfg_output( SPI_HOLD_PIN );

	spiread = get_read_buf_ptr();
	if( SPI_BUF_MAX_DATA < T_CONFIG_LEN )
	{	//if config will not fit, then we will need to modify routines to operate on multiple pages
		app_trace_log(DEBUG_HIGH, "[INIT_E_FL] SPI Read Buffer too small\r");
	}
	
	id = spiflash_init( flash_debug );
	if( id == AT25XE041B ) 
	{
		//Protect Config Region Memory from Erase/Writes
		ret = spiflash_sector_protection( true, CONFIG_START_ADDR_E, NULL );
		if( ret != NRF_SUCCESS )
		{
			app_trace_log(DEBUG_HIGH, "[INIT_E_FL] Config Protect Fail %01u\r", ret);
		}
		
		//Load Config Memory
		ret = spiflash_read(CONFIG_START_ADDR_E, T_CONFIG_LEN, NULL );
		if( ret == NRF_SUCCESS )
		{
			// Verify checksum
			uint16_t chksum = 0;
			uint16_t start_sum = SPI_BUF_DATA_OFFSET;
			uint16_t end_sum = (T_CONFIG_LEN - sizeof(g_config.chksum)) + SPI_BUF_DATA_OFFSET;
			for( int i=start_sum; i<end_sum; i++ )
			{
				chksum += spiread->buf[i];
			}
			uint16_t config_chksum = (((uint16_t)spiread->buf[end_sum+1]<<8) + spiread->buf[end_sum]);
			if( chksum != config_chksum )
			{	//Checksum does not match. Config has either not been written (possible new variable) or
				//an incomplete update occurred. Load from the Copy buffer as it is written first and 
				//should be correct in the advent of an incomplete update to the Primary buffer.
				app_trace_log(DEBUG_HIGH, "[INIT_E_FL] Loading Config Copy Buffer: %01u != %01u\r", chksum, config_chksum);
				ret = spiflash_read( CONFIG_COPY_BUF_ADDR, T_CONFIG_LEN, NULL );
			}
			
			//load config data into RAM
			memcpy( (uint8_t *)&g_config, &spiread->buf[SPI_BUF_DATA_OFFSET], T_CONFIG_LEN );
		}
		else
		{
			app_trace_log(DEBUG_HIGH, "[INIT_E_FL] Config Not Read %01u\r", ret);
		}
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "[INIT_E_FL] Device Not Recognized %01u\r", id);
		ret = NRF_ERROR_NOT_FOUND;
	}
	
	//if( spiflash_ultra_deep_power_down( NULL ) != NRF_SUCCESS )
	if( spiflash_deep_power_down( NULL ) != NRF_SUCCESS )	//synchronous write
	{	//Failed to enter Sleep Mode
		app_trace_log(DEBUG_MED, "[INIT_E_FL] Sleep Err\r");
	}

	return ret;
}

ret_code_t ext_nvm_power_off( void )
{
	ret_code_t res = spiflash_ultra_deep_power_down( NULL );	//synchronous write
	
	return res;
}

//check for changes to config memory and if any are found, save them
static uint8_t update_cfg_state = 0;
ret_code_t ext_config_update( void )
{
	ret_code_t retval = NRF_ERROR_INVALID_STATE;
	
	//Check to make sure persistent data has been initialized
	if( g_config.rev == fw_rev ) 
	{		
		//register the update_config_callback to take control the the spiflash module
		if( !SPIFLASH_LOCK( update_config_cb ) ) return NRF_ERROR_BUSY;
			
		update_cfg_state = 0;		
		//Kickoff Update Config routine by reading config region.
		update_config_cb( true );
		
		//Force blocking for now
		bool block = true;
		if( block )
		{	//Call to synchronously issue operation. Wait for completion
			while( update_cfg_state < 0xFE )
			{
				__WFE();
			}
			if( update_cfg_state == 0xFF ) retval = NRF_ERROR_INTERNAL;	//operation failed
			else retval = NRF_SUCCESS;
		}
	}
	else 
	{
		app_trace_puts(DEBUG_HIGH, "[UPDATE_CFG] Config Not Initialized\r");
		retval = NRF_ERROR_INVALID_STATE;
	}
		
	return retval;
}

static void update_config_cb( bool success )
{
	ret_code_t res;
	T_DEBUG_PRIORITY db_pri = DEBUG_LOW;	//Set the none Error related messages priority
	
	if( !success )
	{
		update_cfg_state = 0xFF;
	}
	
	switch( update_cfg_state )
	{
		case 0:
			//first state triggers Wakeup
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Wake @%01u\r", getSystemTimeMs());
			
			update_cfg_state++;
			res = spiflash_wakeup( update_config_cb );
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;
			break;
		
		case 1:
			//callback from wakeup. Read Config Region
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Read @%01u\r", getSystemTimeMs());
			
			update_cfg_state++;				
			res = spiflash_read( CONFIG_START_ADDR_E, T_CONFIG_LEN, update_config_cb );
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;
			break;
		
		case 2:
			//callback after the Read operation completes
			if( memcmp( (uint8_t *)&g_config, &spiread->buf[SPI_BUF_DATA_OFFSET], T_CONFIG_LEN ) != 0 ) 
			{	//Data has changed (Don't need to Erase/Rewrite if nothing is different)
				if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Sect Unlock @%01u\r", getSystemTimeMs());
				
				uint16_t chksum = 0;
				uint8_t * data = (uint8_t *) &g_config;
				for( int i=0; i<(T_CONFIG_LEN-sizeof(g_config.chksum)); i++ )
				{
					chksum += data[i];
				}
				g_config.chksum = chksum;
			
				update_cfg_state++;
				res = spiflash_sector_protection( false, CONFIG_START_ADDR_E, update_config_cb );		//turn Off protection for config region
				if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;
			}
			else 
			{
				if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] nothing to save @%01u\r", getSystemTimeMs());
				
				SPIFLASH_RELEASE( update_config_cb );
				update_cfg_state = 0xFE;
			}
			break;

		case 3:
			//Call back after Sector Protection is turned Off. Issue Erase Command for Copy Config Buffer
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Erase Copy @%01u\r", getSystemTimeMs());
		
			update_cfg_state++;
			res = spiflash_page_erase( CONFIG_COPY_BUF_ADDR, update_config_cb );
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;			
			break;
		
		case 4:
			//callback after Erase completes, issue Write command
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Write Copy @%01u\r", getSystemTimeMs());
			
			update_cfg_state++;
			res = spiflash_byte_page_write( CONFIG_COPY_BUF_ADDR, (uint8_t *)&g_config, T_CONFIG_LEN, update_config_cb );
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;
			break;
			
		case 5:
			//callback after write copy buffer completes, issue Erase command for Primary Config Buffer
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Erase Primary @%01u\r", getSystemTimeMs());
		
			update_cfg_state++;
			res = spiflash_page_erase( CONFIG_START_ADDR_E, update_config_cb );
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;			
			break;
		
		case 6:
			//callback after Erase completes, issue Write command
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Write Primary @%01u\r", getSystemTimeMs());
			
			update_cfg_state++;
			res = spiflash_byte_page_write( CONFIG_START_ADDR_E, (uint8_t *)&g_config, T_CONFIG_LEN, update_config_cb );
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;
			break;
		
		case 7:
			//callback after write completes, issue Sector Protect command
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Sect Lock @%01u\r", getSystemTimeMs());
		
			update_cfg_state++;
			res = spiflash_sector_protection( true, CONFIG_START_ADDR_E, update_config_cb );		//turn protection back On
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;
			break;
		
		case 8:
			//Call back after Sector Protection is turned On. Issue Sleep Command
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Sleep @%01u\r", getSystemTimeMs());
		
			update_cfg_state++;
			res = spiflash_deep_power_down( update_config_cb );
			if( res != NRF_SUCCESS ) update_cfg_state = 0xFF;
			break;
		
		case 9:
			//callback after sleep command sent. Update complete
			if(flash_debug) app_trace_log(db_pri, "[UPDATE_CFG] Done @%01u\r", getSystemTimeMs());
		
			SPIFLASH_RELEASE( update_config_cb );
			update_cfg_state = 0xFE;
			break;
		
		default:
			update_cfg_state = 0xFF;
			break;
	}
	
	if( update_cfg_state == 0xFF )
	{
		app_trace_log(DEBUG_HIGH, "[UPDATE_CFG] Failed @%01u\r", getSystemTimeMs());
		SPIFLASH_RELEASE( update_config_cb );	//release spiflash module
	}
}

//load a section of flash into RAM
ret_code_t ext_copy_log( uint8_t * data, uint16_t length, uint32_t offset )
{
	static uint8_t call_num = 0;
	ret_code_t res;
	uint32_t addr = (LOG_START_ADDR + offset);
	
	if(flash_debug) app_trace_log(DEBUG_LOW, "[COPY_LOG %02X] AD:0x%04X, LN:%01u @%01u\r", call_num++, addr, length, getSystemTimeMs());
	
	if( offset > TOTAL_LOG_LEN ) 
	{	//block Reads that are beyond Log Region
		app_trace_log(DEBUG_HIGH, "[COPY_LOG] Offset Err @%01u\r", getSystemTimeMs());
		return NRF_ERROR_INVALID_ADDR;
	}
	else if( length > SPI_BUF_MAX_DATA ) 
	{
		app_trace_log(DEBUG_HIGH, "[COPY_LOG] Length Err @%01u\r", getSystemTimeMs());
		return NRF_ERROR_INVALID_LENGTH;
	}
	
	res = spiflash_wakeup( NULL );	//synchronous write
	if( res == NRF_SUCCESS )
	{	//Read data from Nonvolatile Memory and copy to calling function RAM buffer
		if( (offset+length) > TOTAL_LOG_LEN )
		{	//record falls on memory boundary, 2 Read calls must be made
			uint16_t read1_len, read2_len;
			read1_len = TOTAL_LOG_LEN - offset;
			read2_len = length - read1_len;
			res = spiflash_read( addr, read1_len, NULL );	//synchronous read
			memcpy( data, &spiread->buf[SPI_BUF_DATA_OFFSET], read1_len );
			res = spiflash_read( LOG_START_ADDR, read2_len, NULL );				//synchronous read
			memcpy( data+read1_len, &spiread->buf[SPI_BUF_DATA_OFFSET], read2_len );
		}
		else 
		{	//record is not on rollover boundary, direct copy
			res = spiflash_read( addr, length, NULL );		//synchronous read
			memcpy(data, &spiread->buf[SPI_BUF_DATA_OFFSET], length );
		}
		if( res != NRF_SUCCESS )
		{
			app_trace_log(DEBUG_HIGH, "[COPY_LOG] Failed %01u @%01u\r", res, getSystemTimeMs());
		}
	}
	
	if( spiflash_deep_power_down( NULL ) != NRF_SUCCESS )	//synchronous write
	{	//Failed to enter Sleep Mode
		app_trace_log(DEBUG_MED, "[COPY_LOG] Sleep Err\r");
	}
	
	return res;
}

//Erase a page of flash memory
ret_code_t ext_clear_log_page( uint32_t page_start_offset )  
{
	ret_code_t res;
	if(flash_debug) app_trace_log(DEBUG_MED, "[ERASE_PG] @%01u\r", getSystemTimeMs());
	
	if( page_start_offset > TOTAL_LOG_LEN ) 
	{	//block Erases that are beyond Log Region
		app_trace_log(DEBUG_HIGH, "[CLR_LOG] Offset Err @%01u\r", getSystemTimeMs());
		return NRF_ERROR_INVALID_ADDR;
	}
	
	res = spiflash_wakeup( NULL );	//synchronous write
	if( res == NRF_SUCCESS )
	{
		res = spiflash_page_erase( (LOG_START_ADDR + page_start_offset), NULL );	//synchronous call
		if( res != NRF_SUCCESS )
		{
			app_trace_log(DEBUG_HIGH, "[CLR_LOG] Failed %01u @%01u\r", res, getSystemTimeMs());
		}
		else
		{	//Supposedly succeeded!
			if( false )
			{	//Read back what was just written and compare to write buffer
				uint8_t read_data[SPI_BUF_SIZE];
				ext_copy_log( read_data, ERASE_PAGE_LEN, page_start_offset );
				for( int i=0; i<ERASE_PAGE_LEN; i++ )
				{
					if( read_data[i] != 0xFF )
					{
						app_trace_log(DEBUG_HIGH, "[CLR_LOG] Erase Error [0x%02X] = 0x%02X\r\r\r", i, read_data[i]);
						while( 1 ); // Stall for testing failures
					}
				}
			}
		}
	}

	if( spiflash_deep_power_down( NULL ) != NRF_SUCCESS )	//synchronous write
	{	//Failed to enter Sleep Mode
		app_trace_log(DEBUG_MED, "[CLR_LOG] Sleep Err\r");
	}
	
	return res;
}

//Erase a page of flash memory
ret_code_t ext_clear_log_block( uint32_t block_start_offset )  
{
	ret_code_t res;
	if(flash_debug) app_trace_log(DEBUG_MED, "[ERASE_BL] @%01u\r", getSystemTimeMs());
	
	if( block_start_offset > TOTAL_LOG_LEN ) 
	{	//block Erases that are beyond Log Region
		app_trace_log(DEBUG_HIGH, "[ERASE_BL] Offset Err @%01u\r", getSystemTimeMs());
		return NRF_ERROR_INVALID_ADDR;
	}
	
	res = spiflash_wakeup( NULL );	//synchronous write
	if( res == NRF_SUCCESS )
	{
		res = spiflash_erase_4k_block( (LOG_START_ADDR + block_start_offset), NULL );	//synchronous call
		if( res != NRF_SUCCESS )
		{
			app_trace_log(DEBUG_HIGH, "[ERASE_BL] Failed %01u @%01u\r", res, getSystemTimeMs());
		}
		else
		{	//Supposedly succeeded!
		}
	}

	if( spiflash_deep_power_down( NULL ) != NRF_SUCCESS )	//synchronous write
	{	//Failed to enter Sleep Mode
		app_trace_log(DEBUG_MED, "[ERASE_BL] Sleep Err\r");
	}

	return res;
}

//save changes to a full or partial page
ret_code_t ext_store_log( uint8_t * data, uint16_t length, uint32_t offset )
{
	ret_code_t res;	
	uint32_t addr = (LOG_START_ADDR + offset);
	
	if(flash_debug) app_trace_log(DEBUG_MED, "[SAVE_LOG] OF:0x%04X LN:0x%02X @%01u\r", addr, length, getSystemTimeMs());
	
	if( offset > TOTAL_LOG_LEN ) 
	{	//block Writes that are beyond Log Region
		app_trace_log(DEBUG_HIGH, "[SAVE_LOG] Offset Err @%01u\r", getSystemTimeMs());
		return NRF_ERROR_INVALID_ADDR;
	}
	else if( length > PAGE_LEN_BYTES )
	{	//Can't write more than a page worth of data
		app_trace_log(DEBUG_HIGH, "[SAVE_LOG] Length Err @%01u\r", getSystemTimeMs());
		return NRF_ERROR_INVALID_LENGTH;
	}
			
	res = spiflash_wakeup( NULL );	//synchronous write
	if( res == NRF_SUCCESS )
	{
		res = spiflash_byte_page_write( addr, data, length, NULL );	//synchronous call
		if( res != NRF_SUCCESS )
		{
			app_trace_log(DEBUG_HIGH, "[SAVE_LOG] Failed %01u @%01u\r", res, getSystemTimeMs());
		}
		else
		{	//Supposedly succeeded!
			if( false )
			{	//Read back what was just written and compare to write buffer
				uint8_t read_data[SPI_BUF_SIZE];
				ext_copy_log( read_data, length, offset );
				if( memcmp(data, read_data, length) != 0 )
				{	//Why the difference
					app_trace_log(DEBUG_HIGH, "[SAVE_LOG] Read Back Mismatch @%01u\r\r\r", res, getSystemTimeMs());
					while( 1 );	// Stall for testing failures
				}
			}
		}
	}

	if( spiflash_deep_power_down( NULL ) != NRF_SUCCESS )	//synchronous write
	{	//Failed to enter Sleep Mode
		app_trace_log(DEBUG_MED, "[SAVE_LOG] Sleep Err\r");
	}
	
	return res;
}

