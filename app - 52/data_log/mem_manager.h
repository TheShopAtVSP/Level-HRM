/******************************************************************************
 * @file mem_manager.h
 * @brief Manages the storage of Data logs into and out of Non-volatile Memory.		
 * @author Matt Workman
 ******************************************************************************/

#ifndef MEM_MANAGER_H_
#define MEM_MANAGER_H_

#include <stdint.h>
#include "../global.h"
#include "sdk_errors.h"

#define REP_CNT 			8				//allow upto 8 simultaneous reporters

#define user_UUID_LEN		16
#define device_FRAMEID_LEN	4
#define KEY_LEN				6
typedef __packed struct {
	uint32_t rev;
	uint8_t	user_UUID[user_UUID_LEN];	
	uint8_t device_frame_ID[device_FRAMEID_LEN];
	uint32_t advertise_dfu;
	uint32_t totalsteps;
	uint32_t log_head_offset;
	uint32_t log_tail_offset;
	uint8_t report_atts[REP_CNT][16];		//copy of Report Attributes (not currently 16 bytes in length, but reserving just in case)
	uint8_t device_index_ring;
	uint8_t	key_ring[7];
	uint8_t security_key[KEY_LEN];
	uint8_t ship_mode;
	uint32_t hw_rev;
	
	//For the Love of God, ONLY add new variabels here (in front of the chksum)!
	//And then go to config_sanity_check() to support the new variable
	uint16_t chksum;
} T_CONFIG;
#define T_CONFIG_LEN	sizeof(T_CONFIG)

typedef  union {
	__packed struct{
		unsigned char month;
		unsigned char day;
		unsigned char year;
		unsigned char hour;
		unsigned char min;
		unsigned char sec;
	};
} T_CAL_TIME;

#define WORD_SIZE			4
#define WORD_ALIGN_MASK		0x0003

//Record Header
typedef __packed struct {
	uint16_t id;						//value 0-65535
	uint rec_len: 12;					//value 8-244 bytes (includes id and itself)
	uint report_inst: 4;				//Report Instance 0-7 (4bits)
	uint32_t time;						//4 bytes
} T_REC_HEADER;

// Defines for the Record Data Structure
#define NEW_REC_PREAMBLE		((uint32_t)':'<<24 | 'G'<<16 | 'O'<<8 | 'L')
#define OLD_REC_PREAMBLE		((uint32_t)':'<<24 | 'G'<<16 | 'O'<<8 | 'H')	//And 'L'(0x4C) can be written to a H(0x48) without an erase...
#define REC_PREAMBLE_LEN		sizeof(uint32_t)
#define REC_HEADER_LEN			sizeof(T_REC_HEADER)
#define REC_START_LEN 			(REC_PREAMBLE_LEN + REC_HEADER_LEN)
#define REC_TIME_LEN			sizeof(uint32_t)
#define REC_FOOTER_LEN			(REC_START_LEN - REC_TIME_LEN)					//Record footer is meant to be overwritten with the next record. In the meantime, length = 0xFFF
#define REC_MAX_MEM				512												//The maximum number of bytes to be saved per 1 complete record
#define REC_OVERHEAD			(REC_PREAMBLE_LEN + REC_FOOTER_LEN)				//All the non-transmitted portions of a record
#define REC_MIN_LEN				(REC_HEADER_LEN)
#define REC_MAX_LEN				(REC_MAX_MEM - REC_OVERHEAD)					//the number of bytes of transmitted information to the user: upto 244 bytes
#define REC_MAX_DATA_LEN		(REC_MAX_LEN - REC_HEADER_LEN)					//the length of the data contained within the record: upto 236 bytes
#define REC_DATA_AND_FOOTER_LEN	(REC_MAX_MEM - REC_START_LEN)					//All possible data bytes + extra appended footer:	244 bytes

//Memory Storage Structure for a Generic Data Record
typedef __packed struct {
	uint32_t preamble;					//The string: "LOG:" or "HOG:" is stored here
	T_REC_HEADER hdr;
} T_REC_START;

//Record needs to be aligned in memory because Pstore Mandates the RAM source location be word aligned
typedef __attribute__((aligned(4))) __packed struct {
	uint32_t preamble;					//The string: "LOG:" or "HOG:" is stored here
	T_REC_HEADER hdr;
	uint8_t data[REC_DATA_AND_FOOTER_LEN];	
} T_RECORD;

//States for when searching memory for beginning and ending of logged records
typedef enum {
	BEGIN = 0,
	NEXT_LOG = 1,
	NEXT_ROG = 2,
	HUNT_START = 3,
	COMPLETE = 4,
}T_MEM_CHECK_ST;

uint32_t force_word_aligned( uint32_t len );
bool ship_mode( void );
bool manufacture_mode( void );
void set_config_update_flag( void );
bool get_config_update_flag( void );
bool update_config( void );
ret_code_t init_mem_manager( bool debug_mem );
void uninit_mem_manager( void );
uint32_t records_byte_cnt( void );
bool inform_rec_tranceived( void );
bool delete_tail_block( void );
uint32_t get_log_region_size( void );
bool add_record( T_RECORD * rec );
void reporter_update_cmplt( void );
bool save_buffered_recs( void );
bool get_next_record( T_RECORD *rec );
bool data_log_empty( void );
void record_added( uint8_t rep_inst );
void record_removed( uint8_t rep_inst );
	
#endif /* MEM_MANAGER_H_ */



