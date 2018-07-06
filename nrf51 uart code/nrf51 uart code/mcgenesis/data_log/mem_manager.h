/**
 * \file
 *
 * \brief Persistent storage header
 *
 * 
 *
 */

#ifndef _MEM_MANAGER_H_
#define _MEM_MANAGER_H_

#include "reports.h"

#define FLASH_PAGE_SIZE				(0x1000)	
#define MAGIC_NUM 					0x694d5441
#define NUM_DM_PAGES				1		//First Page Registered to Device Manager
#define NUM_LOG_PAGES				64		//Next Pages Registered for Data Logging
#define NUM_CONFIG_PAGES			1		//Last User Page to store persistent configurations
#define PSTORAGE_DM_START_ADDR		((uint32_t)PSTORAGE_LOG_START_ADDR - NUM_DM_PAGES*FLASH_PAGE_SIZE)
#define PSTORAGE_START_ADDR			PSTORAGE_DM_START_ADDR
#define PSTORAGE_CONFIG_ADDR		((uint32_t)PSTORAGE_SWAP_ADDR - NUM_CONFIG_PAGES*FLASH_PAGE_SIZE)
#define PSTORAGE_LOG_START_ADDR     ((uint32_t)PSTORAGE_CONFIG_ADDR - NUM_LOG_PAGES*FLASH_PAGE_SIZE)

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
#define REC_START_LEN 			(REC_PREAMBLE_LEN+REC_HEADER_LEN)
#define REC_TIME_LEN			sizeof(uint32_t)
#define REC_FOOTER_LEN			(REC_START_LEN-REC_TIME_LEN)					//Record footer is meant to be overwritten with the next record. In the meantime, length = 0xFFF
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

typedef __packed struct {
	uint32_t preamble;					//The string: "LOG:" or "HOG:" is stored here
	T_REC_HEADER hdr;
	uint8_t data[REC_DATA_AND_FOOTER_LEN];	
} T_RECORD;

typedef enum {
	BEGIN = 0,
	NEXT_LOG = 1,
	NEXT_ROG = 2,
	HUNT_START = 3,
	COMPLETE = 4,
}T_MEM_CHECK_ST;

#define user_UUID_LEN		16
#define device_FRAMEID_LEN   4
typedef __packed struct{
	uint32_t rev;							//Use to incorporate new config variables without destroying old config info
	uint8_t	user_UUID[user_UUID_LEN];	
	uint8_t device_frame_ID[device_FRAMEID_LEN];
	uint32_t advertise_dfu;
	uint32_t totalsteps;
	uint32_t log_head_offset;
	uint32_t log_tail_offset;
	uint8_t report_atts[REP_CNT][16];		//copy of Report Attributes (not currently 16 bytes in length, but reserving just in case)
	
	uint32_t magic_num;						//leave at end of config!!!!
} T_CONFIG;

void init_mem_manager( void );
uint32_t update_config( void );
Bool push_record( T_RECORD *rec, Bool ovr_wr_old );
Bool pop_record( T_RECORD *rec );
Bool update_log_tail( void );
Bool memory_empty( void );
uint32_t log_region_size( void );
uint32_t log_byte_size( void );
void flag_config_update( void );
Bool config_update_requested( void );

#endif
