/*
 * hal_nvm.c
 *
 * Created: 4/19/2017 5:00:07 PM
 *  Author: matt
 */ 

#include <string.h>
#include "nrf_soc.h"
#include "hal_nvm.h"

#define MEM_EXTERNAL		//Use external flash as datalog and config storage
//#define MEM_INTERNAL		//Use internal flash as datalog and config storage
#ifdef MEM_INTERNAL
	//Use internal flash for non-volatile storage (interfaced through pstore)	
	#include "int_flash.h"
#else
	//Use external flash for non-volatile storage
	#include "External_Storage\ext_flash.h"
#endif

//Global
extern T_CONFIG g_config;
extern uint32_t fw_rev;

#define CHECK_LEN			16*1024					//look for a valid Start of Record for at least the length of a few pages before giving up

static bool config_loaded = false;
static bool pointers_good = false;
static volatile struct {
	const uint32_t start;
	const uint32_t end;
	uint32_t head_offset;
	uint32_t tail_offset;
} log_region = { LOG_START_ADDR, LOG_END_ADDR, 0, 0 };
static bool nvm_debug = false;

static ret_code_t copy_nvm( uint8_t * data, uint16_t length, uint32_t offset );
static ret_code_t update_log( uint8_t * data, uint16_t total_wr_len, uint32_t log_flash_offset );
static uint32_t update_tail( uint32_t tail_inc );
static uint32_t update_head( uint32_t head_inc );
static uint32_t calc_log_len( uint32_t head_offset, uint32_t tail_offset );

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

//Return the offset to the start of a Block
static uint32_t get_block_start_offset( uint32_t offset )
{
	return (offset & (~ERASE_BLOCK_MASK));
}

//Return the len into a Block
static uint16_t get_block_remainder( uint32_t offset )
{
	return offset & ERASE_BLOCK_MASK;
}

//Return the len into a page
bool pointers_valid( void )
{
	return pointers_good;
}

//Start Up non-volatile memory storage 
ret_code_t hal_init_non_volatile_mem( bool debug ) 
{	
	ret_code_t ret;
	
	nvm_debug = debug;
	
	//Throw an error if T_CONFIG_LEN is greater than 1 page of memory. If it is,
	//we need to do some work to allow multiple pages to be written...
	BUILD_BUG_ON( T_CONFIG_LEN > 255 );
	
	#if defined(MEM_EXTERNAL)
		//primes g_config with stored Configuration values
		ret = init_ext_flash( nvm_debug );
	#else
		//primes g_config with stored Configuration values
		ret = init_flash_i( nvm_debug );
	#endif
			
	if( ret != NRF_SUCCESS) 
	{		
		app_trace_log(DEBUG_HIGH, "[INIT_NVM] Failed, Config Invalid!\r\n");	
	}
	else {
		//Config was loaded by init_flash(). Run a sanity check on the data
		config_loaded = true;	//claims to have loaded, allow changes to be made
	}

	return ret;
}


// directed call to implemented non-volatile memory type 
ret_code_t hal_update_config( void )
{
	ret_code_t ret;
	
	if( config_loaded != true )
	{	//config was not loaded , do NOT allow uninitialized config contents to be saved!
		return NRF_ERROR_INVALID_FLAGS;
	}
	
	if( pointers_valid() == true )
	{	//mem_check has been performed and pointers have been validated
		g_config.log_head_offset = log_region.head_offset;
		g_config.log_tail_offset = log_region.tail_offset;
	}
	
	#if defined(MEM_EXTERNAL)
		ret = ext_config_update();
	#else
		ret = update_config_i();
	#endif
		
	return ret;
}

// directed call to implemented non-volatile memory type 
static ret_code_t copy_nvm( uint8_t * data, uint16_t length, uint32_t offset )
{
	ret_code_t ret;
	
	#if defined( MEM_EXTERNAL )
		ret = ext_copy_log( data, length, offset );
	#else
		ret = copy_log_i( data, length, offset );
	#endif
	
	return ret;
}

// save data in record buffer to persistent memory
bool hal_store_data( uint8_t * data, uint16_t len, uint16_t head_inc, uint16_t tail_inc )
{
	uint8_t retry = 0;
	
	//Remaining space calc does not include any space that may be free in the Tail page so that page will be discarded
	//if the Head needs it.
	uint32_t remain_log_space = TOTAL_LOG_LEN - calc_log_len( log_region.head_offset, get_page_start_offset(log_region.tail_offset) );
	uint16_t write_len = force_word_aligned( len );
	uint32_t res;
	
	//check for possible errors!
	if( pointers_good == false ) 
	{
		app_trace_log(DEBUG_MED, "[PUSH] Pointers Not Initialized!\r\n");
		return false;	//record is bigger than memory, can't save this...
	}
	else if( write_len > remain_log_space )
	{	//Not enough space left to write this much data. Need to erase some pages so this will fit
		app_trace_log(DEBUG_HIGH, "[PUSH] Insufficient Mem: Ad:0x%04X 0x%02X<0x%02X\r\n", log_region.head_offset, remain_log_space, write_len);
		while( write_len > remain_log_space )
		{	//discard as many pages as needed to ensure 
			hal_discard_tail_page();	//tracks records that are being thrown out and then erases the page
			remain_log_space += ERASE_PAGE_LEN;
		}
	}

	do {
		res = update_log( data, write_len, log_region.head_offset );
		if( res != NRF_SUCCESS ) 
		{
			int i=0;
			bool error = false;
			uint8_t read_copy[ REC_MAX_MEM ];
			app_trace_log(DEBUG_MED, "[PUSH] Save Error: 0x%04X\r\n", log_region.head_offset+log_region.start);
			
			//Read back memory to find where Error Starts:
			copy_nvm( read_copy, write_len, log_region.head_offset );
			for( i=0; i<write_len; i++ )
			{
				if( read_copy[i] != data[i] )
				{	//there is a write error
					app_trace_log(DEBUG_HIGH, "[PUSH] Error AD:0x%04X %02X != %02X\r\n", (log_region.start+log_region.head_offset+i), read_copy[i], data[i] );
					
					//erase the problem section (need to make sure all preambles are eradicated):
					memset( read_copy, 0xFF, write_len);
					update_log( read_copy, write_len, log_region.head_offset );
					
					//increment the head passed the error byte and try again to save:
					update_head( force_word_aligned(i+WORD_SIZE) );
					error = true;
					break;	//stop for() loop
				}
			}
			
			if( error == false )
			{	//data check indicates NVM wrote correctly... proceed
				app_trace_log(DEBUG_MED, "[PUSH] No Error Found\r\n");
				res = NRF_SUCCESS;
			}
			else if( ++retry >= 3 )
			{	//this is not working, we need to give up!
				return false;
			}
		}
	} while( res != NRF_SUCCESS );
	
	//Update the head Offset to point to the next Record position in memory
	update_head( head_inc );
	//Some of the records passed in may have already been transmitted, update the tail as directed
	update_tail( tail_inc );
	
	//Successfully updated. Print new Head
	app_trace_log(DEBUG_MED, "[PUSH] OF:0x%04X @%01u\r\n", log_region.head_offset, getSystemTimeMs());
	
	return true;
}

// Our own Update routine that will run faster because it uses RAM instead of the swap flash page
static ret_code_t update_log( uint8_t * data, uint16_t save_len, uint32_t flash_offset )
{
	ret_code_t retval = NRF_SUCCESS;	
	uint8_t copy_page[PAGE_LEN_BYTES];	//Make the same size as 1 flash page in Words
	int32_t i_save_len = save_len;		//make an unsinged int copy to safely test for completion
	
	if( flash_offset > TOTAL_LOG_LEN )
	{	//Offset beyond Memory Region
		app_trace_log(DEBUG_HIGH, "[UPDATE] Illegal OF: 0x%04X\r\n", flash_offset);
		return NRF_ERROR_INVALID_ADDR;	
	}
	
	if(nvm_debug) 
	{
		app_trace_log(DEBUG_MED, "[UPDATE] OF:0x%04X LN:0x%02X @%01u\r\n", flash_offset, i_save_len, getSystemTimeMs());
	}

	// when trying to save across a memory page boundary, the memory object must be broken up so the calls can be 
	// successfully executed across multiple pages.
	while ( i_save_len > 0 )
	{			
		uint32_t page_start = get_page_start_offset( flash_offset );
		uint16_t offset_into_page = get_page_remainder( flash_offset );
		uint16_t page_update_len;
		
		if( i_save_len > (PAGE_LEN_BYTES - offset_into_page) ) 
		{	//More Data than can fit on the remainder of this page. Need to break up the save calls
			//if (gs_bdebug) app_trace_log(DEBUG_LOW, "mem_update: Crossing Page Boundary\r\n");		
			page_update_len = PAGE_LEN_BYTES - offset_into_page;
		}
		else 
		{	//This write will be the last
			page_update_len = i_save_len;
		}
		
		//Assume we are going to write the data as it has been passed to us, if an erase is required, these variables
		//will then be updated again 
		uint16_t wr_len = page_update_len;
		uint8_t * p_data = data;
		uint32_t mem_offset = flash_offset;
		
		//get a copy of the current memory page
		copy_nvm( copy_page, PAGE_LEN_BYTES, page_start );

		//Compare the contents of flash against what needs to be written. If a flash bit is cleared, but needs to be 
		//set, then a flash page erase must be performed. It the flash bits need to change from a 1 to a zero, that 
		//is possible without an erase.
		for( uint16_t i=0; i<page_update_len; i++ ) 
		{
			if( (data[i]&copy_page[offset_into_page+i]) != data[i] ) 
			{	//flash bits are cleared that must be set again via an Erase operation
				if(nvm_debug) 
				{
					app_trace_log(DEBUG_MED, "[UPDATE] Erase OF:0x%04X Old:0x%02X New:0x%02X\r\n", (page_start+offset_into_page+i), copy_page[i], data[i] );
				}
				
				#if defined( MEM_EXTERNAL )
					retval = ext_clear_log_page( page_start );
				#else
					retval = clear_log_page_i( page_start );
				#endif
				
				if(retval != NRF_SUCCESS) 
				{
					app_trace_log(DEBUG_MED, "[UPDATE] Failed Erase OF:0x%04X 0x%02X\r\n", page_start, retval);
					//Proceed for now
				}
			
				//need to re-write the start of this page, so change the start address and write length accordingly:
				wr_len = page_update_len + offset_into_page;	//write the new data + the proceeding page data that had be ne erased
				p_data = copy_page;
				mem_offset = page_start;						//write from the beginning of the page
				
				//Modify the copy page to include the new Data, then save the copy page back to NVM
				memcpy( &copy_page[offset_into_page], data, page_update_len );
				
				i = page_update_len;	//stop for() loop
			}
		}
		
		if( wr_len > 0 ) 
		{	//Data needs writing into NV memory:
			#if defined( MEM_EXTERNAL )
				retval = ext_store_log( p_data, wr_len, mem_offset );
			#else
				retval = store_log_i( p_data, wr_len, mem_offset );
			#endif
			if(retval != NRF_SUCCESS) 
			{		
				app_trace_log(DEBUG_MED, "[UPDATE] Possible Failure %u, AD:%04X LN:%02X\r\n", retval, log_region.start+mem_offset, wr_len);
				//read back the data that was just supposed to have been written
				copy_nvm( copy_page, wr_len, mem_offset );	
				if( memcmp( copy_page, p_data, wr_len ) != 0 )
				{	//NVM data did not write correctly
					app_trace_log(DEBUG_MED, "[UPDATE] Write Failed\r\n");
					break;	//save has failed, abort while() loop and return error
				}
				retval = NRF_SUCCESS;		//No error found in Write Data, continue
			}
		}

		//update variables in case we need to write another page:
		i_save_len -= page_update_len;		//decrement the amount of data left to save
		data += page_update_len;			//increment pointer
		flash_offset += page_update_len;	//increment memory offset
		flash_offset %= TOTAL_LOG_LEN;		//make sure offset wraps and stays within Log Region Bounds
	} 

	return retval;
}

// retrieve step record from persistent memory
bool hal_retrieve_rec(T_RECORD *rec)
{		
	ret_code_t ret = NRF_ERROR_NOT_FOUND;
	T_REC_START rec_hd;	//use to check Start of Record
	int32_t saved_log_len;
	uint16_t tail_inc = 0;
	
	saved_log_len = hal_unsent_log_len();
	if( saved_log_len < REC_MIN_LEN ) 
	{
		//nothing in memory log to pull out
		app_trace_log(DEBUG_MED, "[POP] Not Enough Mem: %01u\r\n", saved_log_len);
		return false;	
	}

	if( false )
	{	//tests the ability to hunt for Records if a misalignment were to occur:
		log_region.tail_offset -= 0x400;	
		log_region.tail_offset %= TOTAL_LOG_LEN;
	}
	
	//Start looking for a Start of Record Preamble
	do 
	{	
		//Move the tail up when hunting for preambles
		saved_log_len = update_tail( tail_inc );

		if( saved_log_len >= REC_MIN_LEN ) 
		{
			//Copy the just the first several bytes to check if it is a start of Record
			copy_nvm( (uint8_t *) &rec_hd, REC_FOOTER_LEN, log_region.tail_offset );
			
			if( rec_hd.preamble != NEW_REC_PREAMBLE ) 
			{
				if( rec_hd.preamble == OLD_REC_PREAMBLE ) 
				{	//When RAM buffer pushes Old Records to flash, we need to increment passed
					//them til we reach a New Record or the head
					if( rec_hd.hdr.rec_len > REC_MAX_LEN )
					{
						app_trace_log(DEBUG_MED, "[POP] Tail Record invalid\r\n");
						tail_inc = WORD_SIZE;
					}
					else
					{
						if(nvm_debug) app_trace_log( DEBUG_MED, "[POP] Old NV Rec @ 0x%04X\r\n", log_region.tail_offset );
						tail_inc = force_word_aligned(rec_hd.hdr.rec_len + REC_PREAMBLE_LEN);
					}
						
				}
				else 
				{	//copy a page of NV memory and hunt for a preamble
					uint8_t copy[256];

					copy_nvm( copy, sizeof(copy), log_region.tail_offset );
					
					//Look through page copy for next occurrence of a Preamble
					for( tail_inc = 0; tail_inc < sizeof(copy); tail_inc+=4 )
					{
						if( *((uint32_t*)&copy[tail_inc]) == NEW_REC_PREAMBLE )
						{	//Found a New Preamble
							break;	//stop for()
						}
						else if( *((uint32_t*)&copy[tail_inc]) == OLD_REC_PREAMBLE )
						{	//Found an Old Preamble
							break;	//stop for()
						}
					}
					
					app_trace_log(DEBUG_MED, "[POP] Hunt Rec @ 0x%04X, 0x%02X, %01d\r\n", log_region.tail_offset, tail_inc, saved_log_len);
				}
			}
		}
		else 
		{	//file isn't long enough to hold a valid record...
			app_trace_log(DEBUG_MED, "[POP] Rec too Short\r\n");
			return false;
		}
	} while ( rec_hd.preamble != NEW_REC_PREAMBLE );	
	
	
	if(	(rec_hd.hdr.rec_len < REC_MIN_LEN) || (rec_hd.hdr.rec_len > REC_MAX_LEN) ||
		((rec_hd.hdr.rec_len+REC_PREAMBLE_LEN) > saved_log_len) ) 
	{
		app_trace_log(DEBUG_MED, "[POP] Rec Length Error: %01u, %01u\r\n", rec_hd.hdr.rec_len, saved_log_len);
		
		//Update Tail Offset by 1 Word to move passed this erroneous "LOG:"
		update_tail( WORD_SIZE );	
		return false;
	}
//	if( rec->hdr.id != tail_id ) {
//		//Record ID should increment by one over the previous record...
//		app_trace_log(DEBUG_MED, "[POP] Rec ID Error: 0x%04X != 0x%04X, 0x%04Xr", rec_hd.hdr.id, tail_id, log_region.tail_offset );
//		tail_id = rec_start.hdr.id;
//		//return false; //continue for now...
//	}
	
	ret = copy_nvm( (uint8_t *) rec, rec_hd.hdr.rec_len+REC_PREAMBLE_LEN, log_region.tail_offset );
	if( ret != NRF_SUCCESS )
	{
		app_trace_log(DEBUG_MED, "[POP] Tail Failed: @0x%04X Er:%01u\r\n", log_region.tail_offset, ret);
		
		//Update Tail Offset by 1 Word to move passed this erroneous "LOG:"
		update_tail( WORD_SIZE );	
		return false;
	}
	else 
	{	//Copy was good}
		if( nvm_debug ) 
		{
			app_trace_log(DEBUG_MED, "[POP] ID:%01u RP:%01u TS:0x%02X LN:0x%03X OF:0x%04X @%01u\r\n", rec->hdr.id, rec->hdr.report_inst, rec->hdr.time, rec->hdr.rec_len, log_region.tail_offset, getSystemTimeMs());
		}
	}
	
	return true;
}

// Need to wait to update Log Tail Pointer until after the sent Record has been Acknowledged
ret_code_t hal_mark_rec_sent( uint16_t rec_id )
{
	uint32_t res = NRF_ERROR_NOT_FOUND;
	T_REC_START rec_hd;
	uint32_t log_size = 0;
	
	//Load a Start of New Record from memory
	do 
	{
		//Copy just the first several bytes to check if it is a valid Start of Record
		copy_nvm( (uint8_t *) &rec_hd, REC_FOOTER_LEN, log_region.tail_offset );
		
		if( rec_hd.preamble == OLD_REC_PREAMBLE )
		{
			if( rec_hd.hdr.rec_len > REC_MAX_LEN )
			{
				if(nvm_debug) app_trace_log(DEBUG_MED, "[MARK] Tail Record invalid\r\n");
				log_size = update_tail( WORD_SIZE );
			}
			else
			{
				if(nvm_debug) app_trace_log(DEBUG_MED, "[MARK] Tail already Marked\r\n");
				log_size = update_tail( (rec_hd.hdr.rec_len+REC_PREAMBLE_LEN) );
			}
			
			//Marked already, indicate as successful?
			//res = NRF_SUCCESS;
		}
		else if( rec_hd.preamble != NEW_REC_PREAMBLE )
		{
			app_trace_log(DEBUG_MED, "[MARK] Tail does Not point to Record\r\n");
			log_size = update_tail( WORD_SIZE );
		}
	}
	while( (rec_hd.preamble != NEW_REC_PREAMBLE) && (log_size > REC_START_LEN) );
	
	if( rec_hd.preamble == NEW_REC_PREAMBLE ) 
	{
		uint8_t retry = 0;
		
		if( rec_id != rec_hd.hdr.id ) 
		{	//Inform User of a Mismatch
			app_trace_log(DEBUG_MED, "[MARK] Record ID Mismatch: 0x%04X != 0x%04X\r\n", rec_id, rec_hd.hdr.id);
		}
		
		//Change preamble to indicate record as sent		
		do {
			uint32_t preamble = OLD_REC_PREAMBLE;	//Change preamble to indicate record as sent
			res = update_log( (uint8_t *) &preamble , WORD_SIZE, log_region.tail_offset );
			if( res == NRF_SUCCESS ) 
			{		
				if( rec_hd.hdr.rec_len > REC_MAX_LEN )
				{
					app_trace_log(DEBUG_MED, "[MARK] Rec Len Err: 0x%02X\r\n", rec_hd.hdr.rec_len);
					update_tail( WORD_SIZE );
				}
				else
				{
					update_tail( (rec_hd.hdr.rec_len+REC_PREAMBLE_LEN) );
				}
			}
			else 
			{
				app_trace_log(DEBUG_MED, "[MARK] Save Failed\r\n");
			}
		} while( res != NRF_SUCCESS && retry++ < 3 );
	}
	
	return res;
}

void hal_erase_tail_block( void )
{
	uint32_t tail_block_offset = get_block_start_offset( log_region.tail_offset );
	uint32_t block_remainder = get_block_remainder( log_region.tail_offset );
	uint32_t update_len = (ERASE_BLOCK_LEN - block_remainder);
	
	if(nvm_debug) app_trace_log(DEBUG_MED, "[DELETE] Log Block: 0x%04X\r\n", tail_block_offset);
	
	#if defined( MEM_EXTERNAL )
		ext_clear_log_block( tail_block_offset );
	#else
		clear_log_page_i( tail_block_offset );
	#endif
	
	if( update_len > hal_unsent_log_len() )
	{	//Force update length to stay within reason
		update_len = hal_unsent_log_len();
	}
	update_tail( update_len );
}

uint32_t hal_discard_tail_page( void )
{
	T_RECORD lost_rec;
	uint32_t delta_mem;
	uint16_t discard_count = 0;
	uint32_t tail_page_offset = get_page_start_offset( log_region.tail_offset );
	uint32_t dis_page_offset = tail_page_offset;
	
	if(nvm_debug) app_trace_log(DEBUG_MED, "[DISCARD] Clear Record Page:\r\n");
	
	do 
	{	//Continue pulling records until we're onto the next memory page.		
		//Keep looping as long as memory keeps getting shorter. If that is not the case, 
		//then we need to abort the loop.
		delta_mem = hal_unsent_log_len();			

		if( hal_retrieve_rec( &lost_rec ) ) 
		{
			if( lost_rec.hdr.rec_len > REC_MAX_LEN )
			{
				app_trace_log(DEBUG_MED, "[DISCARD] Rec Len Err: 0x%02X\r\n", lost_rec.hdr.rec_len);
				update_tail( WORD_SIZE );
			}
			else
			{
				update_tail( (lost_rec.hdr.rec_len+REC_PREAMBLE_LEN) );
			
				record_removed(lost_rec.hdr.report_inst);	//decrease report length
				discard_count++;
			}
		}
		
		delta_mem -= hal_unsent_log_len();
		tail_page_offset = get_page_start_offset( log_region.tail_offset );		
	} 
	while( (dis_page_offset == tail_page_offset) && (delta_mem > 0) );
	
	#if defined( MEM_EXTERNAL )
		ext_clear_log_page( dis_page_offset );
	#else
		clear_log_page_i( dis_page_offset );
	#endif

	if(nvm_debug) app_trace_log(DEBUG_MED, "[DISCARD] %01u Records Discarded\r\n", discard_count);
	
	return discard_count;
}

//returns the length of the log memory region in bytes
uint32_t hal_log_region_size( void )
{
	return TOTAL_LOG_LEN;
}

//Returns amount of data between head and tail
static uint32_t update_tail( uint32_t tail_inc )
{
	uint32_t saved_len = hal_unsent_log_len();
	
	tail_inc = force_word_aligned( tail_inc );
	if( tail_inc > saved_len ) 
	{
		app_trace_log(DEBUG_HIGH, "[TAIL_UPDATE] INC LEN ERR: %01u > %01u\r\n", tail_inc, saved_len);
		tail_inc = saved_len;
	}
	else
	{
		if(nvm_debug) app_trace_log(DEBUG_LOW, "[TAIL_UPDATE] 0x%04X+0x%02X\r\n", log_region.tail_offset, tail_inc);
	}
	
	//Update log tail:
	log_region.tail_offset = force_word_aligned( log_region.tail_offset + tail_inc );
	log_region.tail_offset %= TOTAL_LOG_LEN;
	
	return (saved_len-tail_inc);
}

//Returns amount of free space remaining in log region
static uint32_t update_head( uint32_t head_inc )
{
	uint32_t unsaved_len = TOTAL_LOG_LEN - hal_unsent_log_len();
	
	head_inc = force_word_aligned( head_inc );
	if( head_inc > unsaved_len ) 
	{
		app_trace_log(DEBUG_HIGH, "[HEAD_UPDATE] INC LEN ERR: %01u > %01u\r\n", head_inc, unsaved_len);
		head_inc = 0;
	}
	
	//Update log head:
	log_region.head_offset = force_word_aligned( log_region.head_offset + head_inc );
	log_region.head_offset %= TOTAL_LOG_LEN;	
	
	return ( unsaved_len+head_inc );
}

//returns the number of bytes of unsent Records sitting in Flash
uint32_t hal_unsent_log_len( void )
{
	//Get the number of Unsent bytes stored in Flash buffer
	uint32_t len = calc_log_len( log_region.head_offset, log_region.tail_offset );
	
	return (len);
}

ret_code_t hal_power_off_nvm( void )
{
	ret_code_t res = NRF_SUCCESS;
	
	#if defined( MEM_EXTERNAL )
		res = ext_nvm_power_off();
	#else
		//no equivalent
	#endif
	
	return res;
}

//returns the Length between 2 points in the Flash buffer
static uint32_t calc_log_len( uint32_t head_offset, uint32_t tail_offset )
{
	if( head_offset >= TOTAL_LOG_LEN || tail_offset >= TOTAL_LOG_LEN )
	{
		app_trace_log( DEBUG_HIGH, "[LOG_LEN] Offset Out of Bounds: H:0x%04X, T:0x%04X\r\n", head_offset, tail_offset );
		return 0;
	}
	
	return ( (head_offset - tail_offset)%TOTAL_LOG_LEN );
}

//check memory: copy sections of memory from NVM and accumulate all Unsent
//Records that are found.
#define NVM_COPY_SECTION_OFFSET_MASK	(0x0000001FF)
#define NVM_COPY_SECTION_ALIGN_MASK		(~NVM_COPY_SECTION_OFFSET_MASK)
#define NVM_COPY_SECTION_SIZE			(NVM_COPY_SECTION_OFFSET_MASK+1)
uint16_t hal_log_memory_check( void )
{
	//Copy a little extra just in case the very end is a LOG_PREAMBLE
	uint8_t nvm_copy[NVM_COPY_SECTION_SIZE + REC_HEADER_LEN];	
	T_MEM_CHECK_ST search_state;
	uint32_t nvm_copy_section;
	volatile T_REC_START * temp_rec;
	struct {
		T_REC_START rec;
		uint32_t offset;
	} recent;
	int32_t search_len = 0;
	uint16_t inc_temp_ptr;	
	volatile uint16_t test_id;
	volatile int32_t offset_copy;
	uint16_t head_id, tail_id;
		
//	T_REC_START rec_seed = {
//		.preamble = NEW_REC_PREAMBLE,
//		.hdr = {
//			.id = 0,
//			.rec_len = 0xFFF,	
//			.report_inst = 0xF,
//		}
//	};
//	offset = 0x80;

//	update_log( (uint8_t *) &rec_seed, REC_FOOTER_LEN, offset );
//						
//	while(1);				
			
	//Make sure the head and tail are valid. They must be word aligned
	log_region.head_offset = force_word_aligned( g_config.log_head_offset );
	if( log_region.head_offset >= TOTAL_LOG_LEN ) {
		app_trace_log(DEBUG_MED, "[MEM_CHECK] Head Offset Err 0x%04X\r\n", log_region.head_offset);
		log_region.head_offset = 0;
	}
	log_region.tail_offset = force_word_aligned( g_config.log_tail_offset );
	if( log_region.tail_offset >= TOTAL_LOG_LEN ) {
		//offset is invalid, set it equal to head_offset
		app_trace_log(DEBUG_MED, "[MEM_CHECK] Tail Offset Err 0x%04X\r\n", log_region.tail_offset);
		log_region.tail_offset = log_region.head_offset;
	}
	
	//Copy large section of NVM into RAM to verify Record pointers
	offset_copy = log_region.head_offset;
	nvm_copy_section = (offset_copy&NVM_COPY_SECTION_ALIGN_MASK);
	copy_nvm( nvm_copy, sizeof(nvm_copy), nvm_copy_section );
	recent.offset = offset_copy;
	memcpy( (uint8_t *) &recent.rec, &nvm_copy[recent.offset&NVM_COPY_SECTION_OFFSET_MASK], REC_START_LEN );
	
	//Point to what should be the Head of Memory:
	temp_rec = (T_REC_START *) &nvm_copy[offset_copy&NVM_COPY_SECTION_OFFSET_MASK];
	
	if( (temp_rec->preamble == NEW_REC_PREAMBLE) && (temp_rec->hdr.rec_len == 0x0FFF) ) 
	{
		//The Head does point to LOG: with a perfectly invalid length
		head_id = temp_rec->hdr.id;
	}
	else 
	{
		//Check what the Head is pointing to
		head_id = 0;
		search_len = 0;
		inc_temp_ptr = 0;			//don't move pointer forward yet
		do 
		{
			//Start Hunting for a char that can be a Start of Record Preamble
			search_state = HUNT_START;
			do 
			{
				int32_t last_head;
				
				//move pointer forward by predicted amount
				inc_temp_ptr = force_word_aligned( inc_temp_ptr );	
				offset_copy += inc_temp_ptr;
				if( offset_copy >= TOTAL_LOG_LEN ) 
				{
					offset_copy %= TOTAL_LOG_LEN;	
				}
				if( nvm_copy_section != (offset_copy&NVM_COPY_SECTION_ALIGN_MASK) )
				{	//Need to copy a new section of NVM into RAM
					nvm_copy_section = (offset_copy&NVM_COPY_SECTION_ALIGN_MASK);
					copy_nvm( nvm_copy, sizeof(nvm_copy), nvm_copy_section );
				}
				//could check that we have already scanned full memory range
				
				//Load what could be the start of a Record from memory
				temp_rec = (T_REC_START *) &nvm_copy[offset_copy&NVM_COPY_SECTION_OFFSET_MASK];
			
				//Check if a Start of Record has been found recently. Use to Bail if nothing
				//new is seen for a while.
				search_len += inc_temp_ptr;
				last_head = calc_log_len( offset_copy, recent.offset );
			
				if( temp_rec->preamble == NEW_REC_PREAMBLE || temp_rec->preamble == OLD_REC_PREAMBLE ) 
				{
					search_state = NEXT_LOG;
				}
				else if( last_head > CHECK_LEN || search_len > TOTAL_LOG_LEN ) 
				{
					//As long as reserved memory is > CHECK_LEN bytes, we will eventaully break due to
					//this condition. A new head will stop being seen once the ID sequence is broken.
					//make sure the Head is pointing to a Start of Log Preamble. If not, it needs to...
					offset_copy = recent.offset;	//make sure offset is pointing back at the last detected Record
						
					if( (recent.rec.preamble != NEW_REC_PREAMBLE) || (recent.rec.hdr.rec_len != 0x0FFF) )
					{
						//Seed the first Preamble. They are normally written as a footer appended to the end of the previous record
						//Start of New Log (invalid length indicates its the Head)
						T_REC_START rec_seed = 
						{
							.preamble = NEW_REC_PREAMBLE,
							.hdr = {
								.id = 0,
								.rec_len = 0xFFF,	
								.report_inst = 0xF,
								.time = 0xFFFFFFFF,
							}
						};
					
						if( recent.offset == log_region.head_offset ) 
						{
							//There have been 0 valid records found, seed directly to the saved head
							inc_temp_ptr = 0;
						}
						else 
						{
							//at least 1 valid record (old or new) was found. Seed at the end of the last one found
							inc_temp_ptr = force_word_aligned( recent.rec.hdr.rec_len + REC_PREAMBLE_LEN );
							rec_seed.hdr.id = recent.rec.hdr.id + 1;	//continue ID Sequence (prevent a discontinuity)
						}
						
						recent.offset += inc_temp_ptr;
						if( recent.offset >= TOTAL_LOG_LEN ) 
						{
							recent.offset %= TOTAL_LOG_LEN;
						}
						
						update_log( (uint8_t *) &rec_seed, REC_FOOTER_LEN, recent.offset );
						
						app_trace_log(DEBUG_MED, "[MEM_CHECK] Seeded 0x%04X\r\n", (recent.offset+log_region.start));
					}
					search_state = COMPLETE;	//stop looking for Head of Records, no more exist...
				}
				else 
				{
					if( inc_temp_ptr != WORD_SIZE ) 
					{
						//Preamble was not located at this address
						offset_copy = recent.offset;		//move offset back to last known record to start searching Word by Word from that point
					}
					inc_temp_ptr = WORD_SIZE;				//look to next word(4 bytes) in log
				}

			} 
			while( search_state == HUNT_START );
		
			if( search_state != COMPLETE ) 
			{
				//Check that the Record ID Sequence is tracking as expected
				if( temp_rec->preamble == NEW_REC_PREAMBLE || temp_rec->preamble == OLD_REC_PREAMBLE ) 
				{
					if( (temp_rec->hdr.rec_len <= REC_MAX_LEN && temp_rec->hdr.rec_len >= REC_MIN_LEN) ) 
					{
						if( temp_rec->hdr.id != ++head_id ) 
						{
							//Sequence doesn't match expected
							uint32_t next_offset;
							T_REC_START next_rec;
							
							if(nvm_debug) app_trace_log(DEBUG_MED, "[MEM_CHECK] ID Sequence Start @ 0x%04X\r\n", log_region.start+offset_copy);
							
							//move pointer forward by Record Length + Preamble Length
							inc_temp_ptr = force_word_aligned( temp_rec->hdr.rec_len + REC_PREAMBLE_LEN );	
							next_offset= offset_copy + inc_temp_ptr;
							if( next_offset >= TOTAL_LOG_LEN) 
							{
								next_offset %= TOTAL_LOG_LEN;
							}
							
							//Load next_rec with data:
							copy_nvm( (uint8_t *) &next_rec, REC_START_LEN, next_offset );

							if( next_rec.hdr.id == temp_rec->hdr.id+1 ) 
							{
								if(nvm_debug) app_trace_log(DEBUG_MED, "Verified\r\n");
								head_id = temp_rec->hdr.id;		//head of current Record is valid
							}	
							else 
							{
								//Doesn't fit Sequence, will likely be skipped
								app_trace_log(DEBUG_MED, "Anomaly\r\n", log_region.start+offset_copy);
							}
						}
						else
						{	//ID matches predicted value
						}
					}
				}
			
				//If we are pointing at a Start of Record, check if it is valid 
				inc_temp_ptr = WORD_SIZE;	//increment temp_ptr by 1 word, unless changed below
				if( temp_rec->preamble == OLD_REC_PREAMBLE ) 
				{	//"HOG:"
					if(nvm_debug) app_trace_log(DEBUG_LOW, "[MEM_CHECK] hog @ 0x%04X\r\n", log_region.start+offset_copy);
					if( temp_rec->hdr.id == head_id ) 
					{
						//if (gs_bdebug) app_trace_log(DEBUG_LOW, "HOG\r\n");
						recent.offset = offset_copy;	//new Record header
						memcpy( (uint8_t *) &recent.rec, &nvm_copy[offset_copy&NVM_COPY_SECTION_OFFSET_MASK], REC_START_LEN );
						if( (temp_rec->hdr.rec_len <= REC_MAX_LEN && temp_rec->hdr.rec_len >= REC_MIN_LEN) ) 
						{
							inc_temp_ptr = temp_rec->hdr.rec_len + REC_PREAMBLE_LEN;	//move pointer forward by Record Length + Preamble Length
						}
					}
					else 
					{
						//Random "HOG:" written into Memory?
						app_trace_log(DEBUG_MED, "[MEM_CHECK] hog Skipped\r\n");
					}
				}
				else if( temp_rec->preamble == NEW_REC_PREAMBLE ) 
				{	//"LOG:"
					if(nvm_debug) app_trace_log(DEBUG_LOW, "[MEM_CHECK] log @ 0x%04X\r\n", log_region.start+offset_copy);
					if( temp_rec->hdr.rec_len == 0x0FFF ) 
					{	//The length is Perfectly unwritten. This is the Head of the Log!!!
						//report instance and time should also be completely unwritten. Could check to be extra certain???
						head_id = temp_rec->hdr.id;
						recent.offset = offset_copy;
						memcpy( (uint8_t *) &recent.rec, &nvm_copy[offset_copy&NVM_COPY_SECTION_OFFSET_MASK], REC_START_LEN );
						search_state = COMPLETE;
					}
					else if( temp_rec->hdr.id == head_id ) 
					{
						recent.offset = offset_copy; //new unsent record header
						memcpy( (uint8_t *) &recent.rec, &nvm_copy[offset_copy&NVM_COPY_SECTION_OFFSET_MASK], REC_START_LEN );
						if( (temp_rec->hdr.rec_len <= REC_MAX_LEN) && (temp_rec->hdr.rec_len >= REC_MIN_LEN) ) 
						{
							//not the most recent head (most recent will have length 0xFFF), need to look further
							inc_temp_ptr = temp_rec->hdr.rec_len + REC_PREAMBLE_LEN; //move pointer forward by Record Length + Preamble Length
						}
						else 
						{
							app_trace_log(DEBUG_MED, "[MEM_CHECK] Invalid Rec Length\r\n");
						}
					}
					else 
					{
						//Random "LOG:" written into Memory?
						app_trace_log(DEBUG_MED, "[MEM_CHECK] log Skipped\r\n");
					}
				}	
				else 
				{
					app_trace_log(DEBUG_MED, "[MEM_CHECK] ?\r\n");
				}
			}				
		} 
		while( search_state != COMPLETE );
	}
	
	//The Head now points to the most recent "LOG:" that was discovered, or seeded.
	recent.offset = force_word_aligned( recent.offset );	
	app_trace_log(DEBUG_MED, "[MEM_CHECK] Head: 0x%04X saved ptr, 0x%04X new ptr\r\n", log_region.head_offset+log_region.start, recent.offset+log_region.start);
	log_region.head_offset = recent.offset;
	
	//Make sure the COpy buffer holds the pag the Head is on
	nvm_copy_section = (log_region.head_offset&NVM_COPY_SECTION_ALIGN_MASK);
	copy_nvm( nvm_copy, sizeof(nvm_copy), nvm_copy_section );
	temp_rec = (T_REC_START *) &nvm_copy[log_region.head_offset&NVM_COPY_SECTION_OFFSET_MASK];
	
	//Check what the Tail is pointing to
	offset_copy = recent.offset;
	tail_id = head_id;
	test_id = tail_id;
	search_state = HUNT_START;
	do {
		//Record ID sequence has not been established
		if( temp_rec->preamble == NEW_REC_PREAMBLE ) {
			//could also make sure length is plausible
			//app_trace_log(DEBUG_LOW, "Found Rec: ");
			
			test_id--;	//next id should be 1 less than the last found
			tail_id = temp_rec->hdr.id;
			recent.offset = offset_copy;	//This is a valid unsent message

			if( temp_rec->hdr.rec_len <= REC_MAX_LEN && temp_rec->hdr.report_inst < REP_CNT ) {
				//add 1 to reporter length
				//app_trace_log(DEBUG_LOW, "INC\r\n");
				record_added( temp_rec->hdr.report_inst );	//increase report length
			}
			else {
				//app_trace_log(DEBUG_LOW, "EXC\r\n");
			}
		}
		else if( temp_rec->preamble == OLD_REC_PREAMBLE ) {
			//we have found the Sent Record to Unsent Boundary (could also make sure length is plausible)
			search_state = COMPLETE;
		}
		
		//Look for a char that can be the next Start of Record header
		if( search_state != COMPLETE ) {
			volatile uint last_tail;
			uint16_t copy_test_id = test_id;	//instantiate to the 1 value that can't prematurely initiate a ReSequence
			uint16_t copy_offset_copy;
			
			search_state = HUNT_START;
			while (search_state == HUNT_START ) {				
				offset_copy -= WORD_SIZE;
				if( offset_copy < 0 ) {
					offset_copy += TOTAL_LOG_LEN;
				}
				if( nvm_copy_section != (offset_copy&NVM_COPY_SECTION_ALIGN_MASK) )
				{	//Need to copy a new section of NVM into RAM
					nvm_copy_section = (offset_copy&NVM_COPY_SECTION_ALIGN_MASK);
					copy_nvm( nvm_copy, sizeof(nvm_copy), nvm_copy_section );
				}
				
				//point to Potential Start of Record
				temp_rec = (T_REC_START *) &nvm_copy[offset_copy&NVM_COPY_SECTION_OFFSET_MASK];
				
				last_tail = calc_log_len( recent.offset, offset_copy );	//use to bail early if no Tail Record has Not been seen for some time
				
				if( calc_log_len( log_region.head_offset, offset_copy ) > (TOTAL_LOG_LEN-8) ) {
					//running out of memory to search
					if(nvm_debug) app_trace_log(DEBUG_LOW, "[MEM_CHECK] Searched\r\n");
					search_state = COMPLETE;
				}
				else if( temp_rec->preamble == NEW_REC_PREAMBLE || temp_rec->preamble == OLD_REC_PREAMBLE ) {	//"LOG:" || "HOG:"
					if( temp_rec->hdr.id == test_id ) {
						//We have an ID sequence match!
						search_state = NEXT_LOG;
					}
					else {						
						//Sequence ID does not match what we expected. Could be a random HOG or LOG saved into memory. Could be an ID Sequence discontinuity...
						app_trace_log(DEBUG_MED, "[MEM_CHECK] ID Seq Err: 0x%0X != 0x%0X\r\n", temp_rec->hdr.id, test_id );
						
						if( temp_rec->hdr.id == copy_test_id ) {
							T_REC_START old_rec;
							
							//2 ID matches in a row... an ID Sequence discontinuity has occurred
							app_trace_log(DEBUG_MED, "[MEM_CHECK] ReSequencing\r\n");

							//reload the record header from the start of the discontinuity
							copy_nvm( (uint8_t *) &old_rec, REC_FOOTER_LEN, copy_offset_copy );
							
							test_id = old_rec.hdr.id;
							search_state = NEXT_LOG;
						}
						
						copy_test_id = temp_rec->hdr.id - 1;	//if we get a match on this next time through, there is a new ID sequence
						copy_offset_copy = offset_copy;
					}
				}
				else if( last_tail > CHECK_LEN ) {
					//haven't seen a Start of Record in too long
					//if (gs_bdebug) app_trace_log(DEBUG_LOW, "Mem Check: NTF\r\n");	//No Tail Found
					search_state = COMPLETE;
				}
				else {
					//continue looking
				}
			}
		}
		
		//BREAK_POINT;
		
	} while( search_state != COMPLETE );	
	
	//The Tail now points to the LAST "LOG:" or the largest "ROG:" that was discovered...
	recent.offset = force_word_aligned( recent.offset );	
	app_trace_log(DEBUG_MED, "[MEM_CHECK] Tail: 0x%04X saved ptr, 0x%04X new ptr\r\n", log_region.tail_offset+log_region.start, recent.offset+log_region.start);
	log_region.tail_offset = recent.offset;
		
	//allow other functions to use pointers
	pointers_good = true;
	
	return head_id;
}


