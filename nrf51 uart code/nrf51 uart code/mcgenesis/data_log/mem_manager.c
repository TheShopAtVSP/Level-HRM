/*
 * mem_manager.c
 *
 * Created: 11/7/2014 3:13:07 PM
 *  Author: matt
 */ 

#include <string.h>
#include "mem_manager.h"
#include "nrf_soc.h"
#include "pstorage.h"
#include "nrf_error.h"

//Global
T_CONFIG g_config;

extern Bool gs_bdebug;
extern uint32_t fw_rev;

#define PSTORE_TIMEOUT			500		//All p_store operations will complete within this time

#define HW_PAGE_LEN_BYTES		FLASH_PAGE_SIZE						//PSTORAGE_FLASH_PAGE_SIZE
#define HW_PAGE_MASK			(HW_PAGE_LEN_BYTES-1)
#define LOG_BLOCK_CNT			NUM_LOG_PAGES						//64 blocks of 4KB memory pages
#define TOTAL_LOG_LEN			HW_PAGE_LEN_BYTES*LOG_BLOCK_CNT		//Size of Memory to reserve for data logging 256 KB
#define TRUNC_PAGE_BOUNDARY(x)	x&(~HW_PAGE_MASK)

#define CHECK_LEN				4*HW_PAGE_LEN_BYTES		//look for a valid Start of Record for at least the length of a few pages before giving up

#define INCREMENT(x,a) { x += a; if(x > (uint8_t *)log_region.end) {x -= log_region.size;} }
#define DECREMENT(x,a) { x -= a; if(x < (uint8_t *)log_region.start) {x += log_region.size;}}
//#define INIT_FLASH

extern void cmd_queue_dequeue(void);
//static uint8_t *pHead;
//static uint8_t *pTail;
static volatile uint16_t head_id = 0;
static volatile uint16_t tail_id = 0;
static Bool pointers_valid = false;
static uint32_t intermed_tail = 0;
static Bool intermed_tail_valid = false;	
static uint16_t tail_rep_inst;
struct {
	uint32_t start;
	uint32_t end;
	uint32_t size;
	uint32_t head_offset;
	uint32_t tail_offset;
} log_region = { 0xFFFFFFF, 0xFFFFFFF, 0, 0, 0 };

//
// pstorage
//
static pstorage_handle_t  	config_handle;
//static pstorage_handle_t  	config_block_handle;
static volatile Bool		wait_config_cb[5];
static pstorage_handle_t  	log_handle;
//static pstorage_handle_t  	log_block_handle;

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

static uint32_t init_config_region(void);
static uint32_t init_log_region( void );
static void log_memory_check( void );
static uint32_t load_config( void );
static uint32_t load_log( uint8_t * data, uint16_t length, uint32_t offset );
static uint32_t store_log( uint8_t * data, uint16_t length, uint32_t offset );
static uint32_t clear_log_page( uint32_t block_id_offset );
static Bool log_erase_check( uint8_t * write_data, uint8_t * flash_data, uint16_t byte_cnt, uint32_t offset );
static uint32_t update_log( uint8_t * data, uint16_t word_len, uint32_t offset );
static uint32_t memory_len( uint32_t head_offset, uint32_t tail_offset );
static void discard_page( uint32_t page_addr );

static __INLINE uint32_t force_word_aligned( uint32_t size )
{
    return (size+WORD_ALIGN_MASK)&(~WORD_ALIGN_MASK);
}

//load persistent config memory into ram
uint32_t load_config( void )
{
	memcpy( (uint8_t *)&g_config, (uint8_t *)config_handle.block_id, sizeof(g_config) );

	return NRF_SUCCESS;
}

void init_mem_manager( void ) 
{
//	uint32_t curr_state = 0, prev_state = 0;
//	do {
//		prev_state = curr_state;
//		curr_state = pstorage_state();
//		if( prev_state != curr_state ) {
//			if (gs_bdebug) app_trace_log("pstore state: %01u", curr_state);
//		}
//		
//	} while( curr_state != 0 );
	
	//Have to Start pStore before making any calls to it
	pstorage_init();	//should already be started by Device Manager
	
	//pStore mem module to init first (places it at the beginning of the persistent storage region)
	init_log_region();	
	
	//pStore module to init last (so it will be placed at the last available memory storage page)
	init_config_region();	
	
	//After Config variables are loaded, the recorders can be initialized with the last saved settings
	init_reporters();
	
	//Check log memory region. Validate pointers or find true head and tail.
	log_memory_check( );
}

static void config_cb_handler(pstorage_handle_t * handle,uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len)
{	
	if (gs_bdebug) app_trace_log("cfg_cb_handler: ");
	switch(op_code)
	{
		case PSTORAGE_STORE_OP_CODE:
			config_cb[STORE_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (gs_bdebug) app_trace_log("stored\r");
				config_cb[STORE_OP_CODE].success = true;
				
			}
			else {
				if (gs_bdebug) app_trace_log("store failed\r");
				config_cb[STORE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
			
		case PSTORAGE_LOAD_OP_CODE:
			config_cb[LOAD_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (gs_bdebug) app_trace_log("loaded\r");
				config_cb[LOAD_OP_CODE].success = true;
			}
			else {
				if (gs_bdebug) app_trace_log("load failed\r");
				config_cb[LOAD_OP_CODE].success = false;
			}
			break;
		
		case PSTORAGE_CLEAR_OP_CODE:
			config_cb[CLEAR_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (gs_bdebug) app_trace_log("cleared\r");
				config_cb[CLEAR_OP_CODE].success = true;
			}
			else {
				if (gs_bdebug) app_trace_log("clear failed\r");
				config_cb[CLEAR_OP_CODE].success = false;
			}
			break;
			
		case PSTORAGE_UPDATE_OP_CODE:
			config_cb[UPDATE_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (gs_bdebug) app_trace_log("updated\r");
				config_cb[UPDATE_OP_CODE].success = true;
				
			}
			else {
				if (gs_bdebug) app_trace_log("update failed\r");
				config_cb[UPDATE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
		
		default:
			if (gs_bdebug) app_trace_log("?\r");
			config_cb[STORE_OP_CODE].wait = false;
			config_cb[LOAD_OP_CODE].wait = false;
			config_cb[CLEAR_OP_CODE].wait = false;
			config_cb[UPDATE_OP_CODE].wait = false;
			break;
    }
}

uint32_t init_config_region(void)
{
	pstorage_module_param_t param;
	uint32_t retval, block_size;	
	
	// must reserve sizes in word aligned increments
	block_size = force_word_aligned(sizeof(g_config));
	
    // register pstorage 	
	param.block_size  = block_size;
	param.block_count = 1;
	param.cb          = config_cb_handler;
	retval = pstorage_register(&param, &config_handle);
	if ( retval == NRF_SUCCESS ) {
		if( gs_bdebug ) app_trace_log("init_config: Storage Handle: 0x%05X\r", (uint) config_handle.block_id);
	}
	else{
		if( gs_bdebug ) app_trace_log("init_config: Storage Handle Register Failed %01u\r",retval);
		return retval;
	}
	
//	retval = pstorage_block_identifier_get(&config_handle, 0, &config_block_handle);
//	if ( retval != NRF_SUCCESS ) {
//		if( gs_bdebug ) app_trace_log("init_config: pstorage_block_identifier_get failed %01u\r",retval);	
//		return retval;	
//	}
//	else  {
//		if( gs_bdebug ) app_trace_log("init_config: pstorage_block_id 0x%05X\r", (unsigned int) config_block_handle.block_id);
//	}

#ifdef INIT_FLASH	// force init...
	if (gs_bdebug) app_trace_log("Forced Initializing Persistent Storage\r");
	g_config.rev = fw_rev;
	g_config.magic_num = MAGIC_NUM;
	for (uint32_t iii = 0; iii < sizeof(g_config.user_UUID);iii++) g_config.user_UUID[iii] = iii;
	g_config.totalsteps = 0;
	g_config.log_head_offset = 0;
	g_config.log_tail_offset = 0;
	g_config.advertise_dfu = 0;
	memset( &g_config.report_atts, 0, sizeof(g_config.report_atts) );
	retval = update_config();	//Sets Type to NO_TYPE
#else
	retval = load_config();
	if( retval == NRF_SUCCESS ) {
		// init if magic number is incorrect
		if ( g_config.magic_num != MAGIC_NUM ) {
			//If something needs to be handled for older revs, do it here
//			if ( g_config.rev <= 0x0022 ) {
//			}
			
			if (gs_bdebug) app_trace_log("Initializing Config Memory\r");
			g_config.rev = fw_rev;
			//g_config.device_frame_ID = 0;
			for (uint32_t iii = 0; iii < sizeof(g_config.user_UUID);iii++) g_config.user_UUID[iii] = iii;
			g_config.totalsteps = 0;				// Start Steps at 0
			g_config.log_head_offset = 0;	// Start log at memory beginning
			g_config.log_tail_offset = 0;	
			g_config.advertise_dfu = 0;			
			g_config.magic_num = MAGIC_NUM;
			memset( &g_config.report_atts, 0, sizeof(g_config.report_atts) );	//Sets Type to NO_TYPE
			retval = update_config();
		}
		else {
			if (gs_bdebug) app_trace_log("Config Memory Loaded\r");
		}
	}
#endif

	return retval;	
}

Bool update_needed = false;

void flag_config_update( void )
{
	update_needed = true;
}

Bool config_update_requested( void )
{
	if( update_needed == true ) {
		update_needed = false;
		return true;
	}
	
	return false;
}

//check for changes to config memory and if any are found, save them
uint32_t update_config( void )
{
	TTASK_TIMER timeout = { .timer = getSystemTimeMs(), .period = PSTORE_TIMEOUT };
	volatile uint32_t retval = NRF_ERROR_BUSY;	
	uint8_t op_code = UPDATE_OP_CODE;
		
	//check to make sure persistent data has been initialized
	if( g_config.magic_num == MAGIC_NUM ) {
		if( pointers_valid == true )
		{	//mem_check has been performed and pointers have been validated or found
			g_config.log_head_offset = log_region.head_offset;
			g_config.log_tail_offset = log_region.tail_offset;
		}
	
		//Look for Data that has changed (Don't rewrite if nothing is different)
		if( memcmp( (uint8_t *)&g_config, (uint8_t *)config_handle.block_id, sizeof(g_config) ) != 0 ) {
			config_cb[op_code].wait = true;
			uint32_t update_size = force_word_aligned( sizeof(g_config) );
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
					if(gs_bdebug) app_trace_log("update_config: fail %01u\r", retval);
				}
			}
			else {
				if(gs_bdebug) app_trace_log("update_config: failed %01u\r", retval);
			}
		}
	}
	else {
		if (gs_bdebug) app_trace_log("Config Not Initialized\r");
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
				//if (gs_bdebug) app_trace_log("log_cb_handler: log stored\r");
				log_cb[STORE_OP_CODE].success = true;
			}
			else {
				if (gs_bdebug) app_trace_log("log_cb_handler: store failed\r");
				log_cb[STORE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
		
		case PSTORAGE_LOAD_OP_CODE:
			log_cb[LOAD_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				//if (gs_bdebug) app_trace_log("log_cb_handler: log loaded\r");
				log_cb[LOAD_OP_CODE].success = true;
			}
			else {
				if (gs_bdebug) app_trace_log("log_cb_handler: load failed\r");
				log_cb[LOAD_OP_CODE].success = false;
			}
			break;
		
		case PSTORAGE_CLEAR_OP_CODE:
			log_cb[CLEAR_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				if (gs_bdebug) app_trace_log("log_cb_handler: log cleared\r");
				log_cb[CLEAR_OP_CODE].success = true;
			}
			else {
				if (gs_bdebug) app_trace_log("log_cb_handler: update failed\r");
				log_cb[CLEAR_OP_CODE].success = false;
			}
			break;
			
		case PSTORAGE_UPDATE_OP_CODE:
			log_cb[UPDATE_OP_CODE].wait = false;
			if (result == NRF_SUCCESS) {
				//if (gs_bdebug) app_trace_log("log_cb_handler: log updated\r");
				log_cb[UPDATE_OP_CODE].success = true;
				
			}
			else {
				if (gs_bdebug) app_trace_log("log_cb_handler: update failed\r");
				log_cb[UPDATE_OP_CODE].success = false;
			}
			// Source memory can now be reused or freed.
			break;
		
		default:
			if (gs_bdebug) app_trace_log("log_cb_handler: ?\r");
			log_cb[STORE_OP_CODE].wait = false;
			log_cb[LOAD_OP_CODE].wait = false;
			log_cb[CLEAR_OP_CODE].wait = false;
			log_cb[UPDATE_OP_CODE].wait = false;
			break;
    }
}

uint32_t init_log_region( void )
{
	pstorage_module_param_t param;
	uint32_t retval;	
	
	// must reserve sizes in word aligned increments
	if( force_word_aligned(TOTAL_LOG_LEN) != TOTAL_LOG_LEN ) return NRF_ERROR_INVALID_ADDR;
	
    // register pstorage 
	param.block_size  = HW_PAGE_LEN_BYTES;
	param.block_count = LOG_BLOCK_CNT;
	param.cb          = log_cb_handler;
	retval = pstorage_register(&param, &log_handle);
	if ( retval == NRF_SUCCESS ) {
		if( gs_bdebug ) app_trace_log("init_record: Storage Handle: 0x%05X\r", (uint)log_handle.block_id);
	}
	else{
		if( gs_bdebug ) app_trace_log("init_record: Storage Handle Register Failed %01u\r",retval);
		return retval;
	}
	
//	retval = pstorage_block_identifier_get(&log_handle, 0, &log_block_handle);
//	if ( retval != NRF_SUCCESS ) {
//		if( gs_bdebug ) app_trace_log("init_record: Block Handle Failed %01u\r",retval);	
//		return retval;	
//	}
//	else  {
//		if( gs_bdebug ) app_trace_log("init_record: Block Handle 0x%05X\r", (uint)log_handle.block_id);
//	}
	
	log_region.size = TOTAL_LOG_LEN;
	log_region.start = log_handle.block_id;
	log_region.end = log_region.start + log_region.size - 1;		//Address of last byte in Log Memory
	
	return retval;	
}

//load persistent log memory into ram
static uint32_t load_log( uint8_t * data, uint16_t length, uint32_t offset )
{
	if( (offset+length) > log_region.size ) {
		//form a continuous variable that can be pointed to
		uint16_t split_len, remain_len;
		split_len = log_region.size - offset;
		remain_len = length - split_len;
		memcpy(data, (uint8_t *)(log_region.start + offset), split_len);
		memcpy(data+split_len, (uint8_t *)log_region.start, remain_len);
	}
	else {
		//header is not on rollover boundary, point directly to it
		memcpy(data, (uint8_t *)(log_region.start + offset), length);
	}

	return NRF_SUCCESS;
}

static uint32_t clear_log_page( uint32_t block_id_offset ) {
	TTASK_TIMER timeout = { .timer = getSystemTimeMs(), .period = PSTORE_TIMEOUT };
	uint8_t op_code = CLEAR_OP_CODE;
	volatile uint32_t retval;
	pstorage_handle_t temp_log_handle = log_handle;
	temp_log_handle.block_id += block_id_offset;	//handle to the memory page block that we want to modify
	
	log_cb[op_code].wait = true;
	retval = pstorage_clear(&temp_log_handle, HW_PAGE_LEN_BYTES);	
	if( retval == NRF_SUCCESS ) {
		//wait until the call back verfies completion
		while( log_cb[op_code].wait == true ) {
			sd_app_evt_wait();
			if( task_time(timeout) ) {
				if(gs_bdebug) app_trace_log("timeout: %01u, %01u\r", getSystemTimeMs(), timeout.timer );
				log_cb[op_code].success = false;
				retval = NRF_ERROR_TIMEOUT; 
				break;
			}
		}
	}
	
	return retval;
}

//save changes to a partial page
static uint32_t store_log( uint8_t * data, uint16_t length, uint32_t offset )
{
	TTASK_TIMER timeout = { .timer = getSystemTimeMs(), .period = PSTORE_TIMEOUT };
	volatile uint32_t retval;	
	pstorage_handle_t temp_log_handle = log_handle;
	uint8_t op_code = STORE_OP_CODE;
	uint16_t page_offset = offset%HW_PAGE_LEN_BYTES;
	
	if( (page_offset + length) > HW_PAGE_LEN_BYTES ) {
		//Data is too long to be saved. It overflows into the next page
		return NRF_ERROR_INVALID_LENGTH;
	}
	
	temp_log_handle.block_id += TRUNC_PAGE_BOUNDARY(offset);

	log_cb[op_code].wait = true;
	retval = pstorage_store(&temp_log_handle, data, length, page_offset);	
	if( retval == NRF_SUCCESS ) {
		//wait until the call back verfies completion
		while( log_cb[op_code].wait == true ) {
			sd_app_evt_wait();
			if( task_time(timeout) ) {
				if(gs_bdebug) app_trace_log("timeout: %01u, %01u\r", getSystemTimeMs(), timeout.timer );
				log_cb[op_code].success = false;
				retval = NRF_ERROR_TIMEOUT; 
				break;
			}
		}
		
		if( log_cb[op_code].success != true ) {
			if(gs_bdebug) app_trace_log("save_log: failed %01u\r", retval);
		}
	}
	else {
		if(gs_bdebug) app_trace_log("save_log: failed %01u\r", retval);
	}
	
	return retval;
}

static Bool log_erase_check( uint8_t * write_data, uint8_t * flash_data, uint16_t byte_cnt, uint32_t offset )
{
	volatile uint32_t retval;
	uint32_t block_id_offset = TRUNC_PAGE_BOUNDARY(offset);
	uint16_t page_offset = offset%HW_PAGE_LEN_BYTES;
	
	if( page_offset+byte_cnt > HW_PAGE_LEN_BYTES ) return false;
	
	//Compare the contents of flash against what needs to be written. If a flash bit is cleared, but needs to be 
	//set, then a flash page erase must be performed. It the flash bits need to change from a 1 to a zero, that 
	//be possible without an erase.
	for( uint16_t i=page_offset; i<(page_offset+byte_cnt); i++ ) {
		if( (*write_data&flash_data[i]) != *write_data ) {	//flash bits are written that must be erased
			if (gs_bdebug) {
				app_trace_log("Erase Page: Addr 0x%04X, RdVal 0x%02X, WrVal 0x%02X\r", (block_id_offset+i), flash_data[i], *write_data );
			}
			retval = clear_log_page( block_id_offset );
			return true;
		}
		write_data++;
	}

	return false;
}

// Our own Update routine that will run faster because it uses RAM instead of the swap flash page
static uint32_t update_log( uint8_t * data, uint16_t total_wr_len, uint32_t log_offset )
{
	volatile uint32_t retval;	
	Bool erased = false;
	uint16_t bytes_written = 0;
	uint16_t page_wr_len;
	uint8_t copy_page[HW_PAGE_LEN_BYTES];	//Make the same size as 1 flash page in Words
	uint32_t block_id_offset = TRUNC_PAGE_BOUNDARY(log_offset);
	uint16_t page_offset = log_offset%HW_PAGE_LEN_BYTES;
	pstorage_handle_t temp_log_handle = log_handle;
	temp_log_handle.block_id += block_id_offset;	//handle to the memory page block that we want to modify
	
	uint32_t count;
	pstorage_access_status_get( &count );
	if( count > 0 ) {
		//todo: pStore is busy, how to kick start it?
		//pstorage_init();
		//init_record();
		//init_config();	//pStore module to init last (so it will be placed at the last available memory page)
		if (gs_bdebug) app_trace_s_msg("PSTORE Busy\r");
		return NRF_ERROR_BUSY;
	}

	if( (page_offset+total_wr_len) > HW_PAGE_LEN_BYTES ) {
		//if (gs_bdebug) app_trace_log("mem_update: Crossing Page Boundary\r");		
		page_wr_len = HW_PAGE_LEN_BYTES - page_offset;
	}
	else {
		page_wr_len = total_wr_len;
	}

	// when trying to save across a memory page boundary, the memory object must be broken up so the calls can be 
	// successfully executed on the 2 separate pages.
	do {	
		//get a copy of the current memory page
		load_log( copy_page, HW_PAGE_LEN_BYTES, block_id_offset );
		
		//Check if page needs erasing. Is so, this function will handle it (we already have a copy of the old info to re-save).
		erased = log_erase_check( data+bytes_written, copy_page, page_wr_len, log_offset );
		
		if( erased == true ) {
			//the page had to be erased, rewrite the page data that preceeds the start of this record
			for( uint16_t i=0; i<page_wr_len; i++ ) 
			{
				copy_page[ page_offset+i ] = data[ bytes_written+i ];
			}
			
			if( page_wr_len > 0 ) {
				retval = store_log( copy_page, page_wr_len+page_offset, block_id_offset );
				if(retval != NRF_SUCCESS) {
					if (gs_bdebug) app_trace_log("mem_check: Update Failed %u\r",retval);
					break;	//write has failed, abort
				}
			}
		}
		else {
			//the section we need to write was already completely erased
			if( page_wr_len > 0 ) {
				retval = store_log( data+bytes_written, page_wr_len, log_offset );
				if(retval != NRF_SUCCESS) {
					if (gs_bdebug) app_trace_log("mem_check: Update Failed %u\r",retval);
					break;	//write has failed, abort
				}
			}
		}

		bytes_written += page_wr_len;
		if( bytes_written < total_wr_len ) {	
			//next page needs to be handled now
			block_id_offset += HW_PAGE_LEN_BYTES;
			if( block_id_offset > (log_region.size-HW_PAGE_LEN_BYTES) ) block_id_offset = 0;	
			log_offset = block_id_offset;	//updated to the next available Memory Page
			
			page_offset = 0;				//new page, thus no offset
			page_wr_len = total_wr_len - bytes_written;		//the amount of data to be written to the beginning of this new page
		}
		
	} while ( bytes_written < total_wr_len );

	return retval;
}

//returns the length of the log file in bytes
static uint32_t memory_len( uint32_t head_offset, uint32_t tail_offset )
{
	int32_t length = head_offset - tail_offset;
	
	if( length < 0 )
	{	//head is smaller than tail, its wrapped around the end of memory
		length += log_region.size;
	}
	
	return (length);
}

//check memory, are pointers valid?
void log_memory_check( void )
{
	T_MEM_CHECK_ST search_state;
	volatile T_REC_START temp_rec;
	volatile Bool id_seq_validated = false;
	int32_t search_len = 0;
	uint16_t inc_temp_ptr;	
	volatile uint16_t test_id;
	volatile int32_t offset_copy, recent_rec_offset;
	
	if( (log_region.size < REC_HEADER_LEN) || (log_region.size > NUM_LOG_PAGES*FLASH_PAGE_SIZE) )	{
		if (gs_bdebug) app_trace_log("memory_check: Log Memory Len Error\r");
		return;
	}
	else if( log_region.start < PSTORAGE_LOG_START_ADDR ) {
		if (gs_bdebug) app_trace_log("memory_check: Log Location Error\r");
		//Can't allow this region to overlap with code
		log_region.start = PSTORAGE_LOG_START_ADDR;	
		log_region.size = 0;
		//erase page?
		return;
	}
	
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
	if( log_region.head_offset >= log_region.size ) {
		log_region.head_offset = 0;
	}
	log_region.tail_offset = force_word_aligned( g_config.log_tail_offset );
	if( log_region.tail_offset >= log_region.size ) {
		log_region.tail_offset = log_region.head_offset;
	}
	
	//check if head points to LOG: len=0x0FFF
	offset_copy = log_region.head_offset;
	recent_rec_offset = log_region.head_offset;
	load_log( (uint8_t *) &temp_rec, REC_FOOTER_LEN, offset_copy );
	
	if( (temp_rec.preamble == NEW_REC_PREAMBLE) && (temp_rec.hdr.rec_len == 0x0FFF) ) {
		//The Head does point to LOG: with a perfectly invalid length
		head_id = temp_rec.hdr.id;
	}
	else {
		//Check what the Head is pointing to
		head_id = 0;
		search_len = 0;
		inc_temp_ptr = 0;			//don't move pointer forward yet
		do {
			//Start Hunting for a char that can be a Start of Record Preamble
			search_state = HUNT_START;
			do {
				int32_t last_head;
				
				//move pointer forward by predicted amount
				inc_temp_ptr = force_word_aligned( inc_temp_ptr );	
				offset_copy += inc_temp_ptr;
				if( offset_copy >= log_region.size ) offset_copy -= log_region.size;	
				//could check that we have already scanned full memory range
				
				//Load what could be the start of a Record from memory
				load_log( (uint8_t *) &temp_rec, REC_FOOTER_LEN, offset_copy );
			
				//Check if a Start of Record has been found recently. Use to Bail is nothing
				//new is seen for a while.
				search_len += inc_temp_ptr;
				last_head = memory_len( offset_copy, recent_rec_offset );
			
				if( temp_rec.preamble == NEW_REC_PREAMBLE || temp_rec.preamble == OLD_REC_PREAMBLE ) {
					search_state = NEXT_LOG;
				}
				else if( last_head > CHECK_LEN || search_len > log_region.size ) {
					//As long as reserved memory is > CHECK_LEN bytes, we will eventaully break due to
					//this condition. A new head will stop being seen once the ID sequence is broken.
					//make sure the Head is pointing to a Start of Log Preamble. If not, it needs to...	
					offset_copy = recent_rec_offset;	//make sure offset is pointing back at the last detected Record
					load_log( (uint8_t *) &temp_rec, REC_FOOTER_LEN, offset_copy );
					if( (temp_rec.preamble != NEW_REC_PREAMBLE) || (temp_rec.hdr.rec_len != 0x0FFF)) {
						//Seed the first Preamble. They are normally written as a footer appended to the end of the previous record
						//Start of New Log (invalid length indicates its the Head)
						T_REC_START rec_seed = {
							.preamble = NEW_REC_PREAMBLE,
							.hdr = {
								.id = 0,
								.rec_len = 0xFFF,	
								.report_inst = 0xF,
							}
						};
					
						update_log( (uint8_t *) &rec_seed, REC_FOOTER_LEN, recent_rec_offset );
						if (gs_bdebug) app_trace_log("Mem Check: Seeded 0x%04X\r", (recent_rec_offset+log_region.start));
					}
					search_state = COMPLETE;	//stop looking for Head Start of Record, no more exist...
				}
				else {
					if( inc_temp_ptr != WORD_SIZE ) inc_temp_ptr = WORD_SIZE;	//look to next word(4 bytes) in log
				}

			} while( search_state == HUNT_START );
		
			if( search_state != COMPLETE ) {
				//If the Record ID sequence has not yet been confirmed, try.
				if( id_seq_validated == false ) {
					//Record ID sequence has not been established
					//if (gs_bdebug) app_trace_log("MEM Seq\r");
					if( temp_rec.preamble == NEW_REC_PREAMBLE || temp_rec.preamble == OLD_REC_PREAMBLE ) {	//"LOG:" || "HOG:"
						uint32_t offset_copy_copy;
						T_REC_START temp_rec2;
						
						//move pointer forward by Record Length + Preamble Length
						inc_temp_ptr = force_word_aligned( temp_rec.hdr.rec_len + REC_PREAMBLE_LEN );	
						offset_copy_copy = offset_copy + inc_temp_ptr;
						if( offset_copy_copy >= log_region.size) offset_copy_copy -= log_region.size;
						
						//Load what could be the start of a Record from memory
						load_log( (uint8_t *) &temp_rec2, REC_FOOTER_LEN, offset_copy_copy );
					
						if( temp_rec2.hdr.id == temp_rec.hdr.id+1 ) {
							recent_rec_offset = offset_copy;	//The record before copy_copy is validated
							id_seq_validated = true;
						}
					
						head_id = temp_rec.hdr.id;	//head of current Record is valid
					}
				}
				else {
					if( temp_rec.preamble == NEW_REC_PREAMBLE || temp_rec.preamble == OLD_REC_PREAMBLE ) {
						test_id = head_id+1;
						if( temp_rec.hdr.id == test_id ) {
							head_id++;
						}
						else {
						//	id_seq_validated = false;
						}
					}
				}
			
				//If we are pointing at a Start of Record, check if it is valid 
				inc_temp_ptr = WORD_SIZE;	//increment temp_ptr by 1 word, unless changed below
				if( temp_rec.preamble == OLD_REC_PREAMBLE ) {		//"JOG:"
					if( temp_rec.hdr.id == head_id ) {
						//if (gs_bdebug) app_trace_log("JOG\r");
						recent_rec_offset = offset_copy;	//new Record header
						if( (temp_rec.hdr.rec_len <= REC_MAX_LEN && temp_rec.hdr.rec_len >= REC_MIN_LEN) ) {
							inc_temp_ptr = temp_rec.hdr.rec_len + REC_PREAMBLE_LEN;	//move pointer forward by Record Length + Preamble Length
						}
					}
				}
				else if( temp_rec.preamble == NEW_REC_PREAMBLE ) {	//"LOG:"
					//if (gs_bdebug) app_trace_log("LOG\r");
					if( id_seq_validated == false ) {
						if( temp_rec.hdr.rec_len == 0x0FFF ) {	//The length is unwritten at the Head of the Log
							//assume this is the head
							head_id = temp_rec.hdr.id;
							recent_rec_offset = offset_copy;
							search_state = COMPLETE;
						}
					}
					else if( temp_rec.hdr.id == head_id ) {
						recent_rec_offset = offset_copy; //new unsent record header
						if( temp_rec.hdr.rec_len == 0x0FFF ) {	//The length is unwritten at the Head of the Log
							//head pointer is good, look no further
							//head_id = id;	//already equal
							search_state = COMPLETE;
						}
						else if( (temp_rec.hdr.rec_len <= REC_MAX_LEN) && (temp_rec.hdr.rec_len >= REC_MIN_LEN) ) {
							//not the most recent head, need to look further
							inc_temp_ptr = temp_rec.hdr.rec_len + REC_PREAMBLE_LEN; //move pointer forward by Record Length + Preamble Length
						}
						else {
							if (gs_bdebug) app_trace_log("Mem Check: Invalid Rec Length\r");
						}
					}
				}	
			}				
		} while( search_state != COMPLETE );
		//The Head now points to the largest "LOG:" or "ROG:" that was discovered...
		recent_rec_offset = force_word_aligned( recent_rec_offset );	
		if (gs_bdebug) app_trace_log("Mem Check Head: 0x%04X saved ptr, 0x%04X new ptr\r", log_region.head_offset+log_region.start, recent_rec_offset+log_region.start);
		log_region.head_offset = recent_rec_offset;
	}
		
	//Check what the Tail is pointing to
	offset_copy = recent_rec_offset;
	tail_id = head_id;
	test_id = tail_id;
	search_state = HUNT_START;
	do {
		//Record ID sequence has not been established
		if( temp_rec.preamble == NEW_REC_PREAMBLE ) {
			if( temp_rec.hdr.id == test_id )	//could also make sure length is plausible
			{	//A Start of Record is where we expected it
				//app_trace_log("Found Rec: ");
				test_id -= 1;
				tail_id = temp_rec.hdr.id;
				recent_rec_offset = offset_copy;	//This is a valid unsent message
				if( temp_rec.hdr.rec_len <= REC_MAX_LEN && temp_rec.hdr.report_inst < REP_CNT ) {
					//add 1 to reporter length
					//app_trace_log("INC\r");
					inc_report_length( temp_rec.hdr.report_inst );	//increase report length
				}
				else {
					//app_trace_log("EXC\r");
				}
			}
		}
		else if( temp_rec.preamble == OLD_REC_PREAMBLE ) {
			if( temp_rec.hdr.id == test_id )	//could also make sure length is plausible
			{
				//we have found the Sent Record to Unsent Boundary
				search_state = COMPLETE;
			}
		}
		
		//Look for a char that can be the next Start of Record header
		if( search_state != COMPLETE ) {
			volatile uint last_tail;
			
			search_state = HUNT_START;
			while (search_state == HUNT_START ) {				
				offset_copy -= WORD_SIZE;
				if( offset_copy < 0 ) {
					offset_copy += log_region.size;
				}
				
				//point to Potential Start of Record
				load_log( (uint8_t *) &temp_rec, REC_FOOTER_LEN, offset_copy );
				
				last_tail = memory_len( recent_rec_offset, offset_copy );	//use to bail early if no Tail Record has been seen for some time
				
				if( memory_len( log_region.head_offset, offset_copy ) > (log_region.size-8) ) {
					//running out of memory to search
					search_state = COMPLETE;
				}
				else if( temp_rec.preamble == NEW_REC_PREAMBLE || temp_rec.preamble == OLD_REC_PREAMBLE ) {	//"LOG:" || "HOG:"
					search_state = NEXT_LOG;
				}
				else if( last_tail > CHECK_LEN ) {
					//haven't seen a Start of Record in too long
					search_state = COMPLETE;
				}
			}
		}
		
		BREAK_POINT;
		
	} while( search_state != COMPLETE );	
	
	//The Tail now points to the LAST "LOG:" or the largest "ROG:" that was discovered...
	recent_rec_offset = force_word_aligned( recent_rec_offset );	
	if(gs_bdebug) app_trace_log("Mem Check Tail: 0x%04X saved ptr, 0x%04X new ptr\r", log_region.tail_offset+log_region.start, recent_rec_offset+log_region.start);
	log_region.tail_offset = recent_rec_offset;
		
	//allow other functions to use pointers
	pointers_valid = true;
	
	if(gs_bdebug) app_trace_log("Unsent Record Count: %03u\r", get_total_record_cnt());
}

// save data to persistent memory
Bool push_record( T_RECORD *rec, Bool ovr_wr_old )
{	
	uint32_t res;
	uint16_t write_len = force_word_aligned( rec->hdr.rec_len + REC_OVERHEAD );
	int16_t data_len = rec->hdr.rec_len - REC_HEADER_LEN;
	uint16_t footer_offset = force_word_aligned( data_len );
	uint32_t log_tail_page_addr = TRUNC_PAGE_BOUNDARY(log_region.tail_offset+log_region.start);
	uint32_t log_head_page_addr = TRUNC_PAGE_BOUNDARY(log_region.head_offset+log_region.start);
	
	//check for possible errors!
	if( pointers_valid == false ) {
		if(gs_bdebug) app_trace_log("Memory Pointers have not been Initialized...\r");
		return false;	//record is bigger than memory, can't save this...
	}
	if( write_len > REC_MAX_MEM ) {
		if(gs_bdebug) app_trace_log("Record Too Big! Rejected...\r");
		return false;	//record is bigger than memory, can't save this...
	}
	if( data_len < 0 ) {
		if(gs_bdebug) app_trace_log("Record Length Too Short! Cannot be Saved...\r");
		return false;	//record is bigger than memory, can't save this...
	}	

	//Keep file pointers from overflowing...
	uint32_t temp_head_offset = log_region.head_offset + write_len ;
	if( temp_head_offset >= log_region.size ) temp_head_offset -= log_region.size;
	uint32_t log_new_head_page_addr = TRUNC_PAGE_BOUNDARY(temp_head_offset + log_region.start);
	if( log_tail_page_addr == log_head_page_addr ) { //head and tail already on the same page. Tail must be less than head
		if( log_region.head_offset < log_region.tail_offset ) {
			//There's a serious issue here, this shouldn't happen
			if(gs_bdebug) app_trace_log("Insufficient Memory!\r");
			if( ovr_wr_old == true ) {
				discard_page( log_tail_page_addr );
			}
			else {
				//can't save rec, out of memory
				if(gs_bdebug) app_trace_log("Record not Saved...\r");
				return false;
			}
		}
	}
	else if( log_new_head_page_addr == log_tail_page_addr )
	{	//Head is about to pass Tail...
		if(gs_bdebug) app_trace_log("Insufficient Memory!\r");
		if( ovr_wr_old == true ) {
			discard_page( log_tail_page_addr );
		}
		else {
			//can't save rec, out of memory
			if(gs_bdebug) app_trace_log("Record not Saved...\r");
			return false;
		}
	}
	
	//Add heeader
	*((uint32_t *)&rec->preamble) = NEW_REC_PREAMBLE;		
	*((uint16_t *)&rec->hdr.id)= head_id;					//Next Record ID
	
	//Add footer that is the next Record ID with an invalid length of 0xFFFF(unwritten memory)
	*((uint32_t *)&rec->data[footer_offset]) = NEW_REC_PREAMBLE;
	*((uint16_t *)&rec->data[footer_offset+4])= head_id+1;	//Next Record ID
	*((uint16_t *)&rec->data[footer_offset+6])= 0xFFFF;		//Next Record Length and Report Instance

	res = update_log( (uint8_t *) rec, write_len, log_region.head_offset );
	if( res == NRF_SUCCESS ) {
		head_id++;
		inc_report_length( rec->hdr.report_inst );
		if (gs_bdebug) app_trace_log("Saved Record: 0x%04X\r", log_region.head_offset+log_region.start);
		log_region.head_offset += force_word_aligned( rec->hdr.rec_len+REC_PREAMBLE_LEN ) ;
		if( log_region.head_offset >= log_region.size ) log_region.head_offset -= log_region.size;
		return true;
	}
	else {
		if (gs_bdebug) app_trace_log("Failed to Save Record: 0x%04X\r", log_region.head_offset+log_region.start);
		//pstorage_init();
		//init_record();	
	}
	
	return false;
}

// retrieve step record from persistent memory
Bool pop_record(T_RECORD *rec)
{		
	if( log_region.tail_offset == log_region.head_offset ) return false;	//nothing in memory log
	T_REC_START rec_start;	//use to check Start of Record
	volatile int32_t remaining_len;

	//Start looking for a Start of Record Preamble
	load_log( (uint8_t *) &rec_start, REC_FOOTER_LEN, log_region.tail_offset );
	while ( rec_start.preamble != NEW_REC_PREAMBLE ) {
		if(gs_bdebug) app_trace_log("Memory Pointer Incorrect\r");
		
		log_region.tail_offset += WORD_SIZE;
		if( log_region.tail_offset >= log_region.size )	log_region.tail_offset -= log_region.size;
		
		load_log( (uint8_t *) &rec_start, REC_FOOTER_LEN, log_region.tail_offset );
	
		if( memory_len( log_region.head_offset, log_region.tail_offset ) < REC_MIN_LEN ) {
			//file isn't long enough to hold a valid record...
			log_region.tail_offset = log_region.head_offset;
			tail_id = head_id;
			//if(gs_bdebug) app_trace_log("Memory Rec too Short\r");
			return (false);
		}
	}		
		
	// Check the Record ID
	if( rec_start.hdr.id != tail_id ) {
		//Record ID should increment by one over the previous record...
		//if(gs_bdebug) app_trace_log("Memory Rec ID Error\r");
		tail_id = rec->hdr.id;
		//return 0; //continue for now...
	}
	
	// Check the Record Length
	remaining_len = memory_len( log_region.head_offset, log_region.tail_offset );
	if( (rec_start.hdr.rec_len < REC_MIN_LEN) || 
		(rec_start.hdr.rec_len > REC_MAX_LEN) || 
		(rec_start.hdr.rec_len+REC_PREAMBLE_LEN) > remaining_len ) 
	{
		//Length is not possible for a valid Record.
		log_region.tail_offset += WORD_SIZE;	//move Tail forward 1 to get passed this erroneous "LOG:"
		if( log_region.tail_offset >= log_region.size )	log_region.tail_offset -= log_region.size;	
		//if(gs_bdebug) app_trace_log("Memory Rec Length Error\r");
		return (false);
	}

	//Everything seems plausible, copy to record pointer...
	load_log( (uint8_t *) rec, rec_start.hdr.rec_len+REC_PREAMBLE_LEN, log_region.tail_offset );
	
	if (gs_bdebug) {
		//app_trace_log("Pop Record: 0x%04XT\r", (unsigned int)pTail);
	}
	
	// Indicate that new Pointer can be written if message is correctly sent
	intermed_tail = log_region.tail_offset;
	intermed_tail += force_word_aligned( rec_start.hdr.rec_len+REC_PREAMBLE_LEN );
	if( intermed_tail >= log_region.size ) intermed_tail -= log_region.size;
	force_word_aligned( intermed_tail );	//have to push to the next available Word boundary, length will not necessarily get it there.
	intermed_tail_valid = true;
	tail_rep_inst = (uint16_t)rec->hdr.report_inst;
	
	return 1;
}

// Need to wait to update Log Tail Pointer until after the sent Record has been Acknowledged
Bool update_log_tail( void )
{
	uint32_t res;
	uint32_t old_preamble = OLD_REC_PREAMBLE;
	
	if( intermed_tail_valid == false )
	{
		if(gs_bdebug) app_trace_log("Tail Update \r");
		return (false);
	}
	
	intermed_tail_valid = false;
			
	//Mark Record as Sent
	res = store_log( (uint8_t *) &old_preamble, WORD_SIZE, log_region.tail_offset );
	if( res == NRF_SUCCESS ) {
		tail_id++;
		log_region.tail_offset = intermed_tail;
		
		// Inform reporter that a record has sent
		dec_report_length(tail_rep_inst);	//decrease report length
		return true;
	}
	
	return (false);
}

static void discard_page( uint32_t page_addr )
{
	
	T_RECORD lost_rec;
	volatile uint32_t clear_page_addr = page_addr;
	volatile uint32_t current_page_addr;
	uint16_t discard_count = 0;
	
	do {
		//pop all records until were onto the next page
		if( pop_record(&lost_rec) == true ) {
			tail_id++;
			log_region.tail_offset = intermed_tail;
			intermed_tail_valid = false;
			
			// Inform reporter that a record has sent
			dec_report_length(tail_rep_inst);	//decrease report length
		}
		current_page_addr = TRUNC_PAGE_BOUNDARY( log_region.tail_offset+log_region.start );
		++discard_count;
	} while( (current_page_addr == clear_page_addr) && (discard_count < (HW_PAGE_LEN_BYTES/REC_MIN_LEN)) );
	
	if(gs_bdebug) app_trace_log("Oldest %01u Records Discarded...\r", discard_count);

}

// Need to wait to update Log Tail Pointer until after the sent Record has been Acknowledged
Bool memory_empty( void ) 
{
	if( log_region.tail_offset == log_region.head_offset ) return true;
	else return false;
}

//returns the length of the log memory region in bytes
__INLINE uint32_t log_region_size( void )
{
	return log_region.size;
}

//returns the length of the stored logs in bytes
uint32_t log_byte_size( void )
{
	return memory_len( log_region.head_offset, log_region.tail_offset );
}
