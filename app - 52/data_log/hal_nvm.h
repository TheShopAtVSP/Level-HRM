/**
 * \file
 *
 * \brief Persistent storage header
 *
 * 
 *
 */

#ifndef _HAL_NVM_H_
#define _HAL_NVM_H_

#include <stdint.h>
#include "../global.h"
#include "sdk_errors.h"
#include "mem_manager.h"

extern uint8_t 		copy_security_key[ KEY_LEN ];

bool pointers_valid( void );
ret_code_t hal_init_non_volatile_mem( bool debug );
uint16_t hal_log_memory_check( void );
ret_code_t hal_update_config( void );
bool hal_store_data( uint8_t * data, uint16_t wr_len, uint16_t head_inc, uint16_t tail_inc );
bool hal_retrieve_rec(T_RECORD *rec);
ret_code_t hal_mark_rec_sent( uint16_t rec_id );
uint32_t hal_discard_tail_page( void );
void hal_erase_tail_block( void );
uint32_t hal_unsent_log_len( void );
uint32_t hal_log_region_size( void );
ret_code_t hal_power_off_nvm( void );
	
#endif
