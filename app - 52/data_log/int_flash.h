/**
 * \file
 *
 * \brief Persistent storage header
 *
 * 
 *
 */

#ifndef _INT_FLASH_H_
#define _INT_FLASH_H_

#include <stdint.h>
#include "sdk_errors.h"
#include "pstorage.h"

#define PAGE_LEN_BYTES			FLASH_PAGE_SIZE							//PSTORAGE_FLASH_PAGE_SIZE
#define PAGE_MASK				(PAGE_LEN_BYTES-1)
#define ERASE_PAGE_LEN			PAGE_LEN_BYTES							//Size of Standard Erase
#define ERASE_BLOCK_LEN			PAGE_LEN_BYTES							//Only 1 size of Erase, but to stay compatible need the define
#define ERASE_BLOCK_MASK		PAGE_MASK

#define CONFIG_LEN				(NUM_CONFIG_PAGES*PAGE_LEN_BYTES)		//Size of memory to Reserve for configuration variables
#define LOG_PAGE_CNT			NUM_LOG_PAGES							//64 blocks of 4KB memory pages
#define TOTAL_LOG_LEN			(LOG_PAGE_CNT*PAGE_LEN_BYTES)			//Size of Memory to reserve for data logging
#define LOG_START_ADDR			PSTORAGE_LOG_START_ADDR	
#define LOG_END_ADDR			((TOTAL_LOG_LEN - 1) - LOG_START_ADDR)
#define CONFIG_START_ADDR		PSTORAGE_CONFIG_ADDR	
#define CONFIG_END_ADDR			((CONFIG_LEN - 1) - CONFIG_START_ADDR)

ret_code_t init_flash_i( bool debug );
ret_code_t update_config_i( void );
ret_code_t copy_log_i( uint8_t * data, uint16_t length, uint32_t offset );
ret_code_t clear_log_page_i( uint32_t block_id_offset );
ret_code_t store_log_i( uint8_t * data, uint16_t length, uint32_t offset );
	
#endif
