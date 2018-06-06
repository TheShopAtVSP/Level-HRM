/****************************************************************************//**
 * @file spiflash.h
 * @brief Adesto Serial Flash Demo
 * @author Embedded Masters
 * @version 1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2016 Embedded Masters LLC, http://www.embeddedmasters.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Embedded Masters has no
 * obligation to support this Software. Embedded Masters is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Embedded Masters will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#ifndef SPIFLASH_H_
#define SPIFLASH_H_

#include "sdk_errors.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "bsp.h"

/*****************************************************************************/
/** @defgroup Adesto_FlashDrivers      Adesto_FlashDrivers
 */
/*****************************************************************************/

/***************************************************************************//**
 * @addtogroup Adesto_FlashDrivers
 * @{
 ******************************************************************************/

typedef enum
{
	AT25XE021A,
	AT25XE041B,
	
	PART_UNKNOWN
} spiflash_id_t;

#define MAX_FLASH_ID_LEN 4

typedef struct
{
	size_t size;
	uint8_t cmd;
	bool addr_needed;
} erase_info_t;

typedef void (*spiflash_cb) ( bool success, uint16_t rx_len );
typedef void (*rd_wr_er_cmplt_cb) ( bool success );

#define MAX_ERASE_SIZES 5

typedef struct
{
	const char *name;

	size_t id_size;
	const uint8_t id_bytes [MAX_FLASH_ID_LEN];
	size_t device_size;
	int address_bytes;  // only values supported are 2 and 3
	size_t program_page_size;

	// most parts have multiple erase commands of various sizes
	int erase_info_count;
	const erase_info_t erase_info[MAX_ERASE_SIZES];

	const size_t *protection_sector_sizes;
	unsigned int protection_sector_count;

	uint8_t read_status_cmd;
	uint8_t status_busy_mask;
	uint8_t status_busy_level;

	bool read_slow;  		// if true, use READ ARRAY SLOW command with no dummy byte
	bool has_so_irq;
	uint8_t so_done_level;  // level expected on SO when operation done, when using active status interrupt
} spiflash_info_t;

//external read buffer declared by whatever function interfaces:
#define SPI_BUF_DATA_OFFSET		4				//command + address Offset in transfer Receive buffer
#define SPI_BUF_MAX_DATA		0x210			//max data to send and receive
#define SPI_BUF_MARGIN			4				//Leave a little margin for the transfer drivers
#define SPI_MAX_XFER_SIZE		(SPI_BUF_DATA_OFFSET + SPI_BUF_MAX_DATA)
#define SPI_BUF_SIZE			(SPI_MAX_XFER_SIZE + SPI_BUF_MARGIN)
typedef struct {
	uint8_t buf[ SPI_BUF_SIZE ];	//data transferred in by reading
	uint16_t len;					//amount of data in buffer
} T_SPIREAD_BUF;
#if ( REC_MAX_MEM >= SPI_BUF_MAX_DATA )
#error "SPI_BUF must be bigger"
#elsif ( NVM_COPY_SIZE > SPI_BUF_MAX_DATA )
#error "NOOOOO"		//SPI buffer needs to be larger than the Mem Check Copy buffer
#endif

extern const spiflash_info_t spiflash_info_table[];

bool SPIFLASH_LOCK( rd_wr_er_cmplt_cb cb );
bool SPIFLASH_RELEASE( rd_wr_er_cmplt_cb cb );
T_SPIREAD_BUF * get_read_buf_ptr( void );
spiflash_id_t spiflash_init( bool debug);
void spi_check_cb( void );
ret_code_t spiflash_read( uint32_t addr, uint16_t read_len, rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_page_erase( uint32_t page_addr, rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_erase_4k_block( uint32_t block_addr, rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_byte_page_write( uint32_t addr, uint8_t * data, uint16_t len, rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_sector_protection( bool protection, uint32_t addr, rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_wakeup( rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_deep_power_down( rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_ultra_deep_power_down( rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_reset( rd_wr_er_cmplt_cb cb );
ret_code_t spiflash_read_otp( uint32_t addr, size_t len, rd_wr_er_cmplt_cb cb );

/** @} (end addtogroup Adesto_FlashDrivers) */

#endif /* SPIFLASH_H_ */
