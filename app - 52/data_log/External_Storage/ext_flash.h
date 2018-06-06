/**
 * \file
 *
 * \brief Persistent storage header
 *
 * 
 *
 */

#ifndef _EXT_FLASH_H_
#define _EXT_FLASH_H_

#include "mem_manager.h"

#define EXTERNAL_MEM_LEN_BYTES	(1024*512UL)							//512 KBytes of External Memory
#define PAGE_LEN_BYTES			256UL									//a page is 256 Bytes
#define PAGE_MASK				(PAGE_LEN_BYTES-1)						//yeilds offset into Page
#define SMALL_BLOCK_LEN_BYTES	(4*1024UL)								//a small sector memory block is 4 Kbytes
#define SMALL_BLOCK_MASK		(SMALL_BLOCK_LEN_BYTES-1)
#define MED_BLOCK_LEN_BYTES		(32*1024UL)								//a medium sector memory block is 32 Kbytes
#define MED_BLOCK_MASK			(MED_BLOCK_LEN_BYTES-1)
#define LARGE_BLOCK_LEN_BYTES	(64*1024UL)								//a large sector memory block is 64 Kbytes?
#define LARGE_BLOCK_MASK		(LARGE_BLOCK_LEN_BYTES-1)
#define NUM_PAGES_SMALL_BLOCK	(SMALL_BLOCK_LEN_BYTES/PAGE_LEN_BYTES)	//Number of Pages per small sector block
#define TOTAL_PAGE_CNT			(EXTERNAL_MEM_LEN_BYTES/PAGE_LEN_BYTES)	//Total number of pages in External Memory
#define ERASE_PAGE_LEN			PAGE_LEN_BYTES							//Size of Page Erase
#define ERASE_BLOCK_LEN			SMALL_BLOCK_LEN_BYTES					//Size of Block Erase
#define ERASE_BLOCK_MASK		SMALL_BLOCK_MASK

//Setup Config to reside in the last Small Sector in memory (small sectors can be write protected)
#define CONFIG_PAGE_CNT			4*NUM_PAGES_SMALL_BLOCK					//Reserve athe last Protect Sector in memory (= 16Kbytes)
#define CONFIG_LEN				(CONFIG_PAGE_CNT*PAGE_LEN_BYTES)		//Size of memory to reserved for configuration variables
#define CONFIG_START_ADDR_E		((TOTAL_PAGE_CNT - CONFIG_PAGE_CNT)*PAGE_LEN_BYTES)
#define CONFIG_COPY_BUF_ADDR	(CONFIG_START_ADDR_E + CONFIG_LEN/2)
#define CONFIG_END_ADDR			(CONFIG_START_ADDR_E + (CONFIG_LEN-1))

#define LOG_PAGE_CNT			(TOTAL_PAGE_CNT-CONFIG_PAGE_CNT)		//
//#define LOG_PAGE_CNT			(32*4)									//limit to 32k for testing purposes
#define TOTAL_LOG_LEN			(LOG_PAGE_CNT*PAGE_LEN_BYTES)			//Size of memory to reserved for data logging
#define LOG_START_ADDR			0	
#define LOG_END_ADDR			((TOTAL_LOG_LEN - 1) - LOG_START_ADDR)	

ret_code_t init_ext_flash( bool debug );
ret_code_t ext_nvm_power_off( void );
ret_code_t ext_config_update( void );
ret_code_t ext_copy_log( uint8_t * data, uint16_t length, uint32_t offset );
ret_code_t ext_clear_log_page( uint32_t page_start_offset );
ret_code_t ext_clear_log_block( uint32_t block_start_offset );
ret_code_t ext_store_log( uint8_t * data, uint16_t length, uint32_t offset );

#endif

