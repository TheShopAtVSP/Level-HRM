/******************************************************************************
 * @file mem_manager.c
 * @brief Manages the storage of Data logs into and out of Non-volatile Memory.	
 * @author Matt Workman
 ******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include "mem_manager.h"
#include "reports.h"
#include "hal_nvm.h"

//externals
extern uint32_t fw_rev;

//globals
T_CONFIG g_config;

//locals
static bool debug = false;
//static TTASK_TIMER to;	
static bool update_requested = false;
static bool manufact_mode = false;	//once enabled, only a reset can disable

//Buffers for passing data in and out of memory
#define IN_BUF_SIZE		0x400
#if ( IN_BUF_SIZE < 2*REC_MAX_MEM )
#error "HAL_MEM: IN_BUF_SIZE Too Small"
#endif
#define OUT_BUF_SIZE	REC_MAX_MEM
#if ( OUT_BUF_SIZE < REC_MAX_MEM )
#error "HAL_MEM: OUT_BUF_SIZE Too Small"
#endif
typedef struct {
	uint16_t len;		//index where to store the next (newest) record
	uint16_t rd_tail;	//index for the next record in the buffer to be tranceived
	uint8_t * data;
} T_INPUT_BUFFER;
typedef struct {
	uint16_t len;		//index where to store the next (newest) record
	uint8_t * data;
} T_OUTPUT_BUFFER;
static __ALIGNED(4) uint8_t data_in[IN_BUF_SIZE];
static __ALIGNED(4) uint8_t data_out[OUT_BUF_SIZE];
static T_INPUT_BUFFER in_buf = {0, 0, data_in};
static T_OUTPUT_BUFFER out_buf = {0, data_out};
uint8_t copy_security_key[ KEY_LEN ];

static uint16_t head_id = 0;

//prototypes
static bool nv_storage_empty( void );
static bool load_output_buffer( void );

/***************************************************************************//**
 * @brief
 *		Checks if length is an increment of a 4 byte. If not, if forces to the next 
 *		size up that will be an increment of 4 bytes
 * @param[in] size
 * 		the current length of data
 * @return
 * 		a length that is an increment of 4 bytes >= len.
 *
 ******************************************************************************/
uint32_t force_word_aligned( uint32_t len )
{
    return (len+WORD_ALIGN_MASK)&(~WORD_ALIGN_MASK);
}

/***************************************************************************//**
 * @brief
 *		Get flag that indicates hardware needs to power Off after being disconncted 
 *		from the battery.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool ship_mode( void )
{		
	if( g_config.ship_mode != 0 || manufact_mode )
	{	//Force device to enteer low power mode once off the charger
		return true;
	}
		
	return false;
}

/***************************************************************************//**
 * @brief
 *		Get flag that indicates device is in manufacture mode and BLE access is unprotected
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool manufacture_mode( void )
{			
	return manufact_mode;
}

/***************************************************************************//**
 * @brief
 *		In manufacture mode, the connection parameters must be known and simplified.
 *		This sets the key parameters to those known states and will ensure saved data
 *		gets cleared once the session ends.
 * @param[in]
 * 		none
 * @return
 * 		none
 *
 ******************************************************************************/
static void manufacture_connection_settings( void )
{
	for( int i = 0; i < KEY_LEN; i++)
	{	//default to 6 9s
		copy_security_key[i] = 0x09;
	}
		
	//allow connecting without encryption to make manufacture testing simpler	
	manufact_mode = true;		
	
	app_trace_log(DEBUG_MED, "[MANU_SETTING] Manufacture Mode EN\r");
}

/***************************************************************************//**
 * @brief
 *		Set flag that marks an unpdate is needed.
 * @note
 *		Updates cantake 10's of milliseconds. To gaurantee this executes in a 
 *		non-critical function, set this flag in a critical event and poll it
 *		from your main loop. If set, call update_config().
 * @param[in]
 * 		none
 * @return
 * 		none
 *
 ******************************************************************************/
void set_config_update_flag( void )
{
	update_requested = true;
}

/***************************************************************************//**
 * @brief
 *		Get flag that marks an unpdate is needed.
 * @note
 *		You must call update_config() to reset
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool get_config_update_flag( void )
{	
	return update_requested;
}

/***************************************************************************//**
 * @brief
 *		Save RAM config variables to flash.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool update_config( void )
{
	bool res = false;
	ret_code_t ret;
	
	if (debug) app_trace_log(DEBUG_MED, "[CFG_UPDATE] @ %01u\r", getSystemTimeMs());	
	
	update_requested = false;	//clear request
	
	ret = hal_update_config();		
	if( ret == NRF_SUCCESS )
	{		
		res = true;
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "[CFG_UPDATE] Err: 0x%02X\r", ret);	
	}
		
	return res;
}
/***************************************************************************//**
 * @brief
 *		Check config fw_rev variable, if out of date, update other variables as needed.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
//#define INIT_FLASH
static void config_sanity_check(void)
{
	uint8_t i;
	
	if( false )
	{	//Print Out Config Memory Contents
		app_trace_log(DEBUG_MED, "[CONFIG_CHK]:\r");
		for( int i=0; i<T_CONFIG_LEN; i++ )
		{	//Why the difference
			app_trace_log(DEBUG_HIGH, "%02X ", *((uint8_t *)&g_config+i));
		}
		app_trace_log(DEBUG_MED, "\r");
	}
		
	#ifdef INIT_FLASH	// force config memory to reset to default values
		if(debug) app_trace_log(DEBUG_HIGH, "[CONFIG_CHK] Forced Init\r");
		g_config.rev = 0;
	#endif
	
	// Need to update Config region if last saved Rev != latest loaded in flash
	if ( fw_rev != g_config.rev ) 
	{
		app_trace_log(DEBUG_MED, "[CONFIG_CHK] New Rev Update\r");
		
		//There has been a firware update.
		//If a variable needs to be added for older revs, do it here:
		{
			//Config has never been programmed! Set defaults for Config Variables:
			if ( g_config.rev > 0x0999 || g_config.rev == 0x0000 ) 
			{
				//Config memory is not valid ( version number should not be > 9.xx or = 0.00 )
				app_trace_log(DEBUG_HIGH, "[CONFIG_CHK] Reset Variables\r");
				app_trace_log(DEBUG_MED, "[CONFIG_CHK] %03X != %03X\r", g_config.rev, fw_rev);

				g_config.totalsteps = 0;	// Start Steps at 0
				for (i = 0; i < sizeof(g_config.user_UUID); i++) 
				{
					g_config.user_UUID[i] = i;
				}
				memset( &g_config.device_frame_ID, 0, sizeof(g_config.device_frame_ID) );	
				g_config.device_index_ring = 0;
				memset( &g_config.key_ring, 0xFF, sizeof(g_config.key_ring) );	
				memset( &g_config.report_atts, 0, sizeof(g_config.report_atts) );	//Sets Type to NO_TYPE
				g_config.ship_mode = 0xFF;
				g_config.hw_rev = 0;
				memcpy( (void *)&g_config.hw_rev, COMPILED_HW_REV, sizeof(g_config.hw_rev) );
				
				g_config.rev = 0;			// Start at 0 so all following additions will be made
			}
			
			//Add any Config Variables that are missing from earlier versions:
			if( g_config.rev < 0x0103 ) 
			{
				
			}
		}
		
		//Variables that can (and should) be reset everytime new code is loaded:
		g_config.advertise_dfu = 0;		//make sure we don't go back into DFU
		g_config.rev = fw_rev;			//make sure we save the new version number
		
		//Store Updates into Nonvolatile Memory:
		if( hal_update_config() != NRF_SUCCESS )		
		{
			app_trace_log(DEBUG_HIGH, "[CONFIG_CHK] Update Failed\r");
		}
	}

	//For Testing, may need to put device in or out of Manufacture Mode:
	#define BYPASS_MANUFACTURE_MODE
	//#define FORCE_MANUFACTURE_MODE
	#if defined BYPASS_MANUFACTURE_MODE
		memset( &g_config.security_key, 0x09, sizeof(g_config.security_key) );
		g_config.ship_mode = false;
		update_requested = true;
	#elif defined FORCE_MANUFACTURE_MODE
	//	memset( &g_config.security_key, 0xFF, sizeof(g_config.security_key) );
	//	g_config.ship_mode = true;
	//	update_requested = true;
	#else
//		app_trace_log(DEBUG_MED, "[CONFIG_CHK] Access Key: ");
//		for(int i=0; i<KEY_LEN; i++ )
//		{
//			app_trace_log(DEBUG_MED, " 0x%02X,", g_config.security_key[i]);
//		}
//		app_trace_puts(DEBUG_MED, "\r");	
	#endif
	
	//Make Copy of Security Key
	if( g_config.security_key[0] == 0xFF )	
	{	//No Key has been programmed, need to be in manufacture mode: Open radio access and ship mode enabled
		manufacture_connection_settings();
	}
	else
	{	//make copy of programmed KEY
		for (i = 0; i < KEY_LEN; i++)
		{
			copy_security_key[i] = g_config.security_key[i];
		}
		
		if( g_config.ship_mode > 0 )
		{	//Radio acess is protected, but ship mode is still enabled
			app_trace_log(DEBUG_MED, "[CONFIG_CHK] Ship Mode EN\r");
		}
		else
		{
			//normal operational mode
		}
	}
}

/***************************************************************************//**
 * @brief
 *		Start up flash device that will be used as Config and Data Storage
 * @param[in] debug_mem
 * 		true to turn on noncritical debug messages
 * @return ret_code_t
 * 		NRF_SUCCESS or NRF_xxx_ERROR
 *
 ******************************************************************************/
ret_code_t init_mem_manager( bool debug_mem )
{
	ret_code_t ret;	
		
	debug = debug_mem;
	
	if (debug) app_trace_log(DEBUG_LOW, "[INIT_MEM_MGR] start\r");	
	
	ret = hal_init_non_volatile_mem( debug );
	
	if( ret == NRF_SUCCESS )
	{
		//Make sure config values are reasonable
		config_sanity_check();
		
		//Config variables are loaded, the recorders can be initialized with the last saved settings
		init_reporters();
		
		//Check log memory region. Validate pointers and find head and tail record IDs.
		head_id = hal_log_memory_check();
		//update config???
		
		if(debug) app_trace_log(DEBUG_MED, "[INIT_MEM_MGR] Record Count: %03u\r", get_total_record_cnt());
	}
	else
	{
		if(debug) app_trace_log(DEBUG_HIGH, "[INIT_MEM_MGR] Failed %01u, Reporters Invalid!\r", ret);
		manufacture_connection_settings();	//Access Key is unknown. Allow open radio access to debug
	}
	
	in_buf.rd_tail = in_buf.len = 0;
	out_buf.len = 0;
	
	load_output_buffer();	//if any records are stored in non-volatile, this will load the oldest 1
	
	return ret;
}

/***************************************************************************//**
 * @brief
 *		Shutdown flash device that holds Config and Data Storage
 * @return ret_code_t
 * 		NRF_SUCCESS or NRF_xxx_ERROR
 *
 ******************************************************************************/
void uninit_mem_manager( void )
{
	hal_power_off_nvm();
	//etc...
}
/***************************************************************************//**
 * @brief
 *		Store Record in RAM Input Buffer. Move data from Input buffer to 
 *		non-volatile memory as needed.
 * @note
 *		The buffer being more full than the minimum write size will trigger a 
 *		transfer to non-volatile memory.
 * @param[in] rec
 * 		pointer to the record structure to save
 * @return ret_code_t
 * 		NRF_SUCCESS or NRF_xxx_ERROR
 *
 ******************************************************************************/
#define MIN_BUFFER_LEN	64
bool add_record( T_RECORD * rec ) 
{
	uint32_t res = true;
	uint32_t full_len = force_word_aligned( rec->hdr.rec_len + REC_PREAMBLE_LEN );
	
	if( full_len > REC_MAX_MEM )
	{
		app_trace_log(DEBUG_MED, "[ADD_REC] Rec Too Big!\r");
		return false;	//record is bigger than holding structure, can't save this...
	}
	else if( rec->hdr.rec_len < REC_HEADER_LEN )
	{
		app_trace_log(DEBUG_MED, "[ADD_REC] Rec Length Too Short!\r");
		return false;	//record len is impossibly short, don't save this...
	}	
	else if( (in_buf.len + full_len + REC_FOOTER_LEN) >= IN_BUF_SIZE )
	{
		app_trace_log(DEBUG_MED, "[ADD_REC] Not enough room in RAM buffer!\r");
		if( !save_buffered_recs() )
		{
			return false;	//RAM failed to back up in NVM, if we continue, RAM will corrupt
		}
	}

	// Add header Info and put into Input Buffer
	//app_trace_log(DEBUG_MED, "[REC_BUF] Add Record 0x%04X\r", head_id);
	rec->preamble = NEW_REC_PREAMBLE;		
	rec->hdr.id = head_id;					//Next Record ID
	memcpy( &in_buf.data[in_buf.len], (uint8_t *)rec, full_len );
	
	//Update lengths
	record_added( rec->hdr.report_inst );	
	head_id++;
	in_buf.len += full_len;

	//If nothing is in the output buffer, we have at least 1 record to now put in
	if( out_buf.len == 0 )
	{
		load_output_buffer();
	}
	
	return res;
}	

/***************************************************************************//**
 * @brief
 *		Call to inform memory manager when reporters are updated, at which point it
 *		can push buffered records to NVM all at once.
 * @param[in]
 * 		none
 * @return
 * 		none
 *
 ******************************************************************************/
void reporter_update_cmplt( void )
{
		//Save input buffer after it has accumulated X number of bytes
	if( in_buf.len >= MIN_BUFFER_LEN )
	{
		save_buffered_recs();
	}
}

/***************************************************************************//**
 * @brief
 *		Push records in RAM buffer to non-volatile memory.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool save_buffered_recs( void )
{
	uint32_t res = false;
	uint32_t footer_offset = force_word_aligned( in_buf.len );
	int32_t total_wr_len =  footer_offset + REC_FOOTER_LEN;
	
	if( in_buf.len == 0 )
	{	//Nothing to write. Short circuit back to calling function so we don't waste
		//time writing a footer into memory.
		return true;	
	}
	else if( total_wr_len > IN_BUF_SIZE )
	{	//Input Buffer can't hold this much data
		app_trace_log(DEBUG_MED, "[SAVE_BUF] In Buf Len Error!\r");
		return false;
	}
	
	//Add footer that is the next Record ID with an invalid length of 0xFFFF(unwritten memory).
	//If footer is always added, it will end up as the last entry in flash where it is used to verify 
	//the head pointer on startup.
	uint8_t footer[REC_FOOTER_LEN];
	*((uint32_t *)&footer[0]) = NEW_REC_PREAMBLE;
	*((uint16_t *)&footer[4])= head_id;			//Next Record ID
	*((uint16_t *)&footer[6])= 0xFFFF;			//Next Record Length and Report Instance
	memcpy( &in_buf.data[footer_offset], footer, REC_FOOTER_LEN );

	res = hal_store_data( in_buf.data, total_wr_len, footer_offset, in_buf.rd_tail );
	if( res != true )
	{
		app_trace_log(DEBUG_MED, "[SAVE_BUF] Failed!\r");
	}
	
	//there should be nothing left in the Input Buffer. Make Sure both tails are equal.
	in_buf.rd_tail = in_buf.len = 0;
	
	return res;
}

/***************************************************************************//**
 * @brief
 *		Retrieve next record to be sent.
 * @param[in] rec
 * 		pointer to the record structure to fill
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool get_next_record( T_RECORD *rec )
{
	bool res = false;
	
	//Verify Start of Record in Output buffer
	if( out_buf.len >= REC_START_LEN && out_buf.len <= OUT_BUF_SIZE )
	{
		memcpy( (uint8_t *)rec, &out_buf.data[0], out_buf.len );
		if( rec->preamble == NEW_REC_PREAMBLE )
		{	//Start of a New Record verified, tail checks out,
			res = true;
		}
		else
		{
			app_trace_log(DEBUG_MED, "[GET_REC] Invalid Preamble: ");
			res = false;
		}
	}
	else
	{
		app_trace_log(DEBUG_MED, "[GET_REC] Invalid Length: ");
		res = false;
	}
	
	if( res == false )
	{	//no record loaded, clear Out buffer and get next record
		rec->hdr.rec_len = 0;
		out_buf.len = 0;	//clear output buffer
		app_trace_log(DEBUG_MED, "Discarding Output\r");
		load_output_buffer();	//get next record
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 *		Director to inform mem_manager that a record has successfully been 
 *		transferred. It can now be updated to indicate it as a sent record.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
static bool load_output_buffer( void )
{
	bool res = false;
	
	//output buffer already has a record, only 1 at a time allowed
	if( out_buf.len > 0 ) 
	{
		app_trace_log(DEBUG_MED, "[LOAD_OUT] Already Loaded\r");
		return false;
	}
	
	if( !nv_storage_empty() )
	{
		//there are records stored in non-volatile memory. Get these first, they're oldest.
		T_RECORD temp_rec;
		
		res = hal_retrieve_rec( &temp_rec );
		
		if( res )
		{
			//Preamble has already been verified by pop_record()
			//Verify Length of Record before putting it in the Output buffer
			if( temp_rec.hdr.rec_len >= REC_MIN_LEN && temp_rec.hdr.rec_len <= REC_MAX_LEN )
			{
				memcpy( &out_buf.data[0], (uint8_t *)&temp_rec, (temp_rec.hdr.rec_len + REC_PREAMBLE_LEN) );
				out_buf.len = (temp_rec.hdr.rec_len + REC_PREAMBLE_LEN);
			}
			else
			{
				app_trace_log(DEBUG_MED, "[LOAD_OUT] Invalid Length: ");
				
				//Mark Record so we can move passed it
				memcpy( &out_buf.data[0], (uint8_t *)&temp_rec, sizeof(T_REC_START) );
				out_buf.len = sizeof(T_REC_START);	
				inform_rec_tranceived();
				res = false;
			}
		}
	}
	else
	{
		uint32_t unsent_in_buf_len = in_buf.len - in_buf.rd_tail;
		
		//Verify Start of Record in Input buffer
		while( unsent_in_buf_len >= REC_START_LEN )
		{
			T_RECORD * rec = (T_RECORD *) &in_buf.data[in_buf.rd_tail];
			if( rec->preamble == NEW_REC_PREAMBLE )
			{	//Start of a New Record verified, tail checks out,			
				if( rec->hdr.rec_len >= REC_MIN_LEN && rec->hdr.rec_len <= OUT_BUF_SIZE )
				{	//Make sure length is good before copying
					memcpy( &out_buf.data[0], (uint8_t *)&in_buf.data[in_buf.rd_tail], (rec->hdr.rec_len + REC_PREAMBLE_LEN) );
					out_buf.len = (rec->hdr.rec_len + REC_PREAMBLE_LEN);
					res = true;
					app_trace_log(DEBUG_MED, "[LOAD_OUT] ID:%01u RP:%01u TS:0x%02X LN:0x%02X @%01u\r", 
						rec->hdr.id, rec->hdr.report_inst, rec->hdr.time, rec->hdr.rec_len, getSystemTimeMs());
					
					break;		//stop executing while loop, next Record Verified
				}
				else
				{	//New Record Preamble is found, but this length is not good, continue to search for next record
					app_trace_log(DEBUG_MED, "[LOAD_OUT] Invalid Length: ");
					res = false;
				}
			}
			else
			{
				app_trace_log(DEBUG_MED, "[LOAD_OUT] In Tail Err: h-0x%04X, t-0x%04X\r", in_buf.len, in_buf.rd_tail);
			}
			
			//if code reaches this loop, the NEW_RECORD_PREAMBLE was not found, force tail to increment for search
			if( unsent_in_buf_len > REC_START_LEN )
			{
				in_buf.rd_tail = in_buf.rd_tail + WORD_SIZE;
				if( in_buf.rd_tail >= IN_BUF_SIZE ) 
				{
					in_buf.rd_tail %= IN_BUF_SIZE;
				}
			}
			else
			{	//nothing in Input buffer... make sure it gets zeroed out.
				app_trace_log(DEBUG_MED, "[LOAD_OUT] In Buf Cleared\r");
				in_buf.rd_tail = in_buf.len = 0;
			}
				
			unsent_in_buf_len = in_buf.len - in_buf.rd_tail;
		}
	}

	return res;
}

/***************************************************************************//**
 * @brief
 *		Director to inform mem_manager that a record has successfully been 
 *		transferred. It can now be updated to indicate it as a sent record.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool inform_rec_tranceived( void )
{
	uint32_t res = NRF_ERROR_NOT_FOUND;
	uint8_t rep_inst = 0xFF;
	T_RECORD * rec;
	
	if( out_buf.len < sizeof(T_REC_START) ) 
	{
		app_trace_log(DEBUG_MED, "[TRX] Nothing in Out Buffer!\r");
		return false;
	}

	if( !nv_storage_empty() )
	{	//there are records stored in non-volatile memory, update first (they're oldest).		
		rec = (T_RECORD *) out_buf.data;
		res = hal_mark_rec_sent( rec->hdr.id );
		if( res == NRF_SUCCESS )
		{
			rep_inst = rec->hdr.report_inst;
		}
	}
	else if( in_buf.len > in_buf.rd_tail )
	{
		uint32_t inc_len = 4;	//at a minimum move ptr forward 1 Word to prevent getting stuck forever
		
		rec = (T_RECORD *) &in_buf.data[in_buf.rd_tail];
		if( rec->preamble == NEW_REC_PREAMBLE )
		{
			//mark oldest Record in Input RAM Buffer as sent
			rec->preamble = OLD_REC_PREAMBLE;
					
			if( rec->hdr.rec_len <= (in_buf.len - in_buf.rd_tail) )
			{	//matches, update the tail
				rep_inst = rec->hdr.report_inst;
				inc_len = force_word_aligned( rec->hdr.rec_len + REC_PREAMBLE_LEN );
				res = NRF_SUCCESS;
			}
			else
			{	//There is not enough data in the Input buffer to account for this length???
				app_trace_log(DEBUG_MED, "[TRX] Tail Length Mismatch!\r");	
				res = NRF_ERROR_INVALID_LENGTH;
			}
		}
		else
		{
			res = NRF_ERROR_NOT_FOUND;
			//could look for next new preamble
		}
		
		in_buf.rd_tail += inc_len;
		if( in_buf.rd_tail >= IN_BUF_SIZE ) 
		{
			in_buf.rd_tail %= IN_BUF_SIZE;
		}
	}
	else
	{	//Don't see any records that need marking as sent??? 
		app_trace_log(DEBUG_MED, "[TRX] No Unsent Records to Mark!\r");
		res = NRF_ERROR_NO_MEM;
	}
		
	if( res == NRF_SUCCESS ) 
	{
		if(debug) app_trace_log(DEBUG_LOW, "[TRX] Tail Updated\r");
		
		//inform reporter manager that a record has been removed
		record_removed( rep_inst );
	}
	else {
		app_trace_log(DEBUG_MED, "[TRX] Mark Failed: 0x%04X\r", res);
		return false;
	}
	
	//Clear Output buffer so it can load the next record
	out_buf.len = 0;
	load_output_buffer();
	
	return true;
}

/***************************************************************************//**
 * @brief
 *		Retrieves the amount of transferrable data stored in memory.
 * @param[in]
 * 		none
 * @return
 * 		the number of bytes in memory
 *
 ******************************************************************************/
uint32_t records_byte_cnt( void )
{
	uint32_t res;
	uint32_t cnt = 0;
	
	cnt = hal_unsent_log_len();
	cnt += (in_buf.len - in_buf.rd_tail);
	
	//there is a known issue of the return Count being incorrect due to records that 
	//have to be padded to meet the 32 bit word aligned requirement... At this time
	//it's not worth the effort to determine the number of padding bytes.
	res = cnt - REC_PREAMBLE_LEN*get_total_record_cnt();
	
	return res;
}

/***************************************************************************//**
 * @brief
 *		Checks if there is any available data to transmit.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
bool data_log_empty( void )
{
	if( (in_buf.len - in_buf.rd_tail) == 0 && nv_storage_empty() )
	{
		return true;
	}
	
	return false;
}

/***************************************************************************//**
 * @brief
 *		check for data in nonvolatile memory.
 * @param[in]
 * 		none
 * @return
 * 		true/false
 *
 ******************************************************************************/
static bool nv_storage_empty( void )
{
	uint32_t saved_size;
	
	saved_size = hal_unsent_log_len();
	
	//app_trace_log(DEBUG_MED, "[NVM_STORE_SIZE] %01u\r", saved_size);
	
	if( saved_size == 0 )
	{
		return true;
	}
	
	return false;
}

/***************************************************************************//**
 * @brief
 *		Get the size of the nonvolatile memory storage region
 * @param[in]
 * 		none
 * @return
 * 		size in bytes
 *
 ******************************************************************************/
uint32_t get_log_region_size( void )
{
	return hal_log_region_size();
}

/***************************************************************************//**
 * @brief
 *		Delete the oldest block in nonvolatile memory that contains unsent records.
 * @param[in]
 * 		none
 * @return
 * 		true when there is data to erase, false when no data
 *
 ******************************************************************************/
bool delete_tail_block( void )
{
	if( hal_unsent_log_len() > 0 )
	{	//There is data to be erased!
		hal_erase_tail_block();
		return true;
	}
	
	return false;
}

/***************************************************************************//**
 * @brief
 *		director to allow mem_manager to inform the report manager a record has
 *		been removed from memory and it should update any local variables accordingly.
 * @param[in]
 * 		Reporter instance of record that has been marked as old or removed.
 * @return
 * 		none
 *
 ******************************************************************************/
void record_removed( uint8_t rep_inst )
{
	dec_report_length(rep_inst);
}

/***************************************************************************//**
 * @brief
 *		Director to allow mem_manager to inform the report manager a record has
 *		been added to memory and it should update any local variables accordingly.
 * @param[in]
 * 		Reporter instance of record that has been successfully added.
 * @return
 * 		none
 *
 ******************************************************************************/
void record_added( uint8_t rep_inst )
{
	inc_report_length(rep_inst);
}
