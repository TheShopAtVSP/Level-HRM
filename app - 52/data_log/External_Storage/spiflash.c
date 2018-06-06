/******************************************************************************
 * @file spiflash.c
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

#include "hal_spim.h"
#include "spiflash.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "bsp.h"
#include "app_util_platform.h"
#include "app_timer.h"

//Times to be used with Timer Interrupt m_NVM_timer_id
#define NVM_CHECK_TIMER_1MS			( APP_TIMER_TICKS(1, APP_TIMER_PRESCALER) )	//Set Timer to go Off in 1MS
APP_TIMER_DEF(m_NVM_timer_id);

/***************************************************************************//**
 * @addtogroup Adesto_FlashDrivers
 * @{
 ******************************************************************************/

/* Adesto Command Hex Values */
#define CMD_READ_ARRAY                  0x0b
#define CMD_READ_ARRAY_SLOW             0x03  /* no dummy byte, but limited to lower clock (<25MHz) */
#define CMD_READ_ARRAY_DUAL             0x3b  /* not used, as we don't have dual-SPI hardware */
#define CMD_PAGE_ERASE                  0x81
#define CMD_RM25C_PAGE_ERASE            0x42  /* RM26C256DS only */
#define CMD_BLOCK_ERASE                 0x20
#define CMD_BLOCK_ERASE_LARGE           0x52
#define CMD_BLOCK_ERASE_LARGER          0xd8
#define CMD_CHIP_ERASE                  0x60
#define CMD_CHIP_ERASE2                 0xc7  /* not used, exactly the same as CMD_CHIP_ERASE */
#define CMD_BYTE_PAGE_PROGRAM           0x02
#define CMD_SEQUENTIAL_PROGRAM          0xad  /* not used, page program is more efficient */
#define CMD_SEQUENTIAL_PROGRAM2         0xaf  /* not used, page program is more efficient */
#define CMD_BYTE_PAGE_PROGRAM_DUAL      0xa2  /* not used, as we don't have dual-SPI hardware */
#define CMD_WRITE_ENABLE                0x06
#define CMD_WRITE_DISABLE               0x04
#define CMD_PROTECT_SECTOR              0x36
#define CMD_UNPROTECT_SECTOR            0x39
#define CMD_READ_SECTOR_PROTECTION      0x3c
#define CMD_PROGRAM_OTP                 0x9b
#define CMD_READ_OTP                    0x77
#define CMD_READ_STATUS                 0x05
	#define READ_STATUS_SPRL_MASK		0x80	//(R/W)1: Sector Protection Registers are locked
	#define READ_STATUS_SPM_MASK		0x40	//(R)1: sequential pogramming mode, 0: Byte/Page programming
	#define READ_STATUS_EPE_MASK		0x20	//(R)1: erase/program error
	#define READ_STATUS_WPP_MASK		0x10	//(R)1: Write Protect Pin is deasserted
	#define READ_STATUS_SWP_MASK		0x0C	//(R)00: All Sectors are Software Unprotected 11: All Sectors Protected
	#define READ_STATUS_WEL_MASK		0x02	//(R)1: Device Write Latch Enabled
	#define READ_STATUS_RDY_BSY_MASK	0x01	//(R)1: Device Busy with internal Operation
//#define CMD_DATAFLASH_READ_STATUS       0xd7
#define CMD_ACTIVE_STATUS_INTERRUPT     0x25
#define CMD_WRITE_STATUS_REG_BYTE_1     0x01
#define CMD_WRITE_STATUS_REG_BYTE_2     0x31
#define CMD_RESET                       0xf0
#define CMD_READ_ID                     0x9f
#define CMD_DEEP_POWER_DOWN             0xb9
#define CMD_RESUME_FROM_DEEP_POWER_DOWN 0xab
#define CMD_ULTRA_DEEP_POWER_DOWN       0x79

#define ARG_RESET                       0xd0  /* second byte required for reset command */

//static const uint8_t at25xe021a_id_bytes[] = { 0x1f, 0x43, 0x01, 0x00 };
static const size_t at25xe021a_protection_sector_sizes[] = { 65536, 65536, 65536, 65536 };

//static const uint8_t at25xe041b_id_bytes[] = { 0x1f, 0x44, 0x02, 0x00 };
static const size_t at25xe041b_protection_sector_sizes[] = { 65536, 65536, 65536, 65536, 65536, 65536, 65536, 32768, 8192, 8192, 16384 };

const spiflash_info_t spiflash_info_table[] =
{
	[AT25XE021A] = {
		.name                    = "AT25XE021A",
	    .id_size                 = 4,
	    .id_bytes                = { 0x1f, 0x43, 0x01, 0x00 },
	    .device_size             = (2 << 20) / 8,
	    .address_bytes           = 3,
	    .program_page_size       = 256,
	    .erase_info_count        = 5,
	    .erase_info              = {{ 256,    CMD_PAGE_ERASE,         true },
	                                { 4096,   CMD_BLOCK_ERASE,        true },
	    		                    { 32768,  CMD_BLOCK_ERASE_LARGE,  true },
	    		                    { 65536,  CMD_BLOCK_ERASE_LARGER, true },
	    		                    { (2 << 20) / 8, CMD_CHIP_ERASE,  false }},
	    .protection_sector_sizes = at25xe021a_protection_sector_sizes,
	    .protection_sector_count = sizeof(at25xe021a_protection_sector_sizes) / sizeof(size_t),
	    .read_status_cmd         = CMD_READ_STATUS,
	    .status_busy_mask        = 0x01,
	    .status_busy_level       = 0x01,
	    .has_so_irq              = true,
	    .so_done_level           = 0,
	},
	[AT25XE041B] = {
	    .name                    = "AT25XE041B",
	    .id_size                 = 4,
	    .id_bytes                = { 0x1f, 0x44, 0x02, 0x00 },
	    .device_size             = (4 << 20) / 8,
	    .address_bytes           = 3,
	    .program_page_size       = 256,
	    .erase_info_count        = 5,
	    .erase_info              = {{ 256,    CMD_PAGE_ERASE,         true },
	                                { 4096,   CMD_BLOCK_ERASE,        true },
	    		                    { 32768,  CMD_BLOCK_ERASE_LARGE,  true },
	    		                    { 65536,  CMD_BLOCK_ERASE_LARGER, true },
	    		                    { (4 << 20) / 8, CMD_CHIP_ERASE,  false }},
	    .protection_sector_sizes = at25xe041b_protection_sector_sizes,
	    .protection_sector_count = sizeof(at25xe041b_protection_sector_sizes) / sizeof(size_t),
	    .read_status_cmd         = CMD_READ_STATUS,
	    .status_busy_mask        = 0x01,
	    .status_busy_level       = 0x01,
	    .has_so_irq              = true,
	    .so_done_level           = 0,
	},
};

#define SPIFLASH_INFO_TABLE_SIZE (sizeof(spiflash_info_table) / sizeof(spiflash_info_t))

//local variables:
static spiflash_id_t id = PART_UNKNOWN;
static const spiflash_info_t * spiflash_info;
static bool g_spiflash_busy = false;
static bool g_spiflash_res = false;

static T_SPIREAD_BUF read;

static bool spif_debug = false;
static spiflash_cb spi_state_machine_cb = NULL;
static rd_wr_er_cmplt_cb user_cb = NULL;

//local prototypes:
static ret_code_t spiflash_xfer( uint8_t * tx_data, uint16_t tx_len, uint16_t rx_len, bool hold_cs_en );
static ret_code_t spiflash_address_command( uint8_t cmd, uint32_t addr, uint16_t rx_len, bool hold_cs_en );
static ret_code_t spiflash_write_enable( bool enable );
static ret_code_t spiflash_global_protect( bool protect );
static ret_code_t spiflash_read_id( void );		// Read the manufacturer and device ID of the flash chip.
static ret_code_t spiflash_global_protect( bool protect );
static ret_code_t spiflash_read_status( size_t len );
static ret_code_t spiflash_write_enable( bool enable );
static ret_code_t spiflash_enable_active_status( void );
//static void spiflash_enable_status_interrupt( void );
//static void spiflash_disable_status_interrupt( void );
ret_code_t spiflash_write_otp( uint32_t addr, size_t len, uint8_t * data );
void spiflash_write_status(uint8_t data);
void spiflash_write_status2(uint8_t data);

/***************************************************************************//**
 * @brief
 * 		Check is spiflash is currently engaged by another routine
 ******************************************************************************/
bool SPIFLASH_LOCK( rd_wr_er_cmplt_cb cb )
{
	static TTASK_TIMER to;
	
	if( user_cb != NULL ) 
	{	//already locked
		if( task_time(to) ) 
		{	//lock has been held for too long, release it
			app_trace_puts(DEBUG_MED, "[SPILOCK] Forced Lock Release\r");
			spi_state_machine_cb = NULL;
			user_cb = NULL;
		}
		else
		{	//calling function not granted the lock
			return false;
		}
	}
	
	start_task_timer( to, 750);	//Max time that a routine may lock up the spiflash
	
	spi_state_machine_cb = NULL;
	user_cb = cb;
	return true;
}

/***************************************************************************//**
 * @brief
 * 		Release hold on spiflash
 ******************************************************************************/
bool SPIFLASH_RELEASE( rd_wr_er_cmplt_cb cb )
{
	if( user_cb != cb ) 
	{	//this function doe not have the rights to release the spiflash
		app_trace_puts(DEBUG_MED, "[SPIRELEASE] Access Denied\r");
		return false;
	}
	
	spi_state_machine_cb = NULL;
	user_cb = NULL;
	return true;
}

/***************************************************************************//**
 * @brief
 * 		Check priveleges
 ******************************************************************************/
bool SPIFLASH_KEY_MATCH( rd_wr_er_cmplt_cb cb )
{
	if( user_cb == cb ) 
	{
		return true;
	}
	else
	{
		return false;
	}
}

/***************************************************************************//**
 * @brief
 * 		Once the SPI transfer is finished, it will issue this callback to proceed
 * @param[in] success
 * 		true if transfer completed, false if timeout issed callback
 * @param[in] rx_len
 * 		the length of the recieved data from the transfer
 ******************************************************************************/
static void transfer_complete_cb( bool success, uint16_t rx_len )
{	
	if( spif_debug ) app_trace_log(DEBUG_LOW, "   [SPI_CB] 0x%02X\r", rx_len);
	
	g_spiflash_busy = false;
	g_spiflash_res = success;
	read.len = rx_len;
	
	if( !success )
	{
		if( spif_debug ) app_trace_log(DEBUG_MED, "   [SPI_CB] Timeout\r");
	}

	if( spi_state_machine_cb != NULL )
	{	//SPI event callback takes priority over user callback
		spi_state_machine_cb( success, rx_len );
	}
	else if( user_cb != NULL )
	{
		user_cb( success );
	}
}

///***************************************************************************//**
// * @brief
// * 	When Active Status mode is enabled, the flash device SO pin mirrors the !Ready/Busy
// *	bit of the Status register. So while an operation is executing the pin is high.
// *	Once that operation completes, the pin pulls low and generates this callback.
// ******************************************************************************/
//static void spi_ready_cb(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
//{
//	if( spif_debug ) app_trace_log(DEBUG_LOW, "[ACT_STAT] Pin[%01u] = %01u\r", pin, nrf_gpio_pin_read( pin ));
//		
//	if( g_busy_pin_en == true )
//	{	//Only allow callback to execute once (with race conditions it could happen twice):	
//		g_busy_pin_en = false;
//		
//		spiflash_disable_status_interrupt();
//		
//		if( spi_state_machine_cb != NULL )
//		{
//			spi_state_machine_cb( true, 0 );
//		}
//	}
//}

/***************************************************************************//**
 * @brief
 * 		Start Check Timer to generate Interrupt in X milliseconds
 * 		
 ******************************************************************************/
static void start_check_timer( uint16_t milsec )
{
	uint32_t ticks = NVM_CHECK_TIMER_1MS*milsec;
	if( ticks < APP_TIMER_MIN_TIMEOUT_TICKS )
	{	//Set to minimum value
		if (spif_debug) app_trace_log(DEBUG_MED, "   [CHECK] Min Tick Violation @%01u\r", getSystemTimeMs());
		ticks = APP_TIMER_MIN_TIMEOUT_TICKS;
	}
	if (spif_debug) app_trace_log(DEBUG_LOW, "   [CHECK] Timer On @%01u\r", getSystemTimeMs());
	ret_code_t res = app_timer_start( m_NVM_timer_id, ticks, NULL );	
	APP_ERROR_CHECK(res);
}

/***************************************************************************//**
 * @brief
 * 		Stop Check Timer
 * 		
 ******************************************************************************/
//static void stop_check_timer( void )
//{
//	if (spif_debug) app_trace_log(DEBUG_LOW, "   [CHECK] Timer Off @%01u\r", getSystemTimeMs());
//	ret_code_t res = app_timer_stop( m_NVM_timer_id );
//	APP_ERROR_CHECK(res);
//}

/***************************************************************************//**
 * @brief
 * 		Callback for Check Timer Interrupt
 * 		
 ******************************************************************************/
static void check_timer_cb(void * p_context)
{
    if (spif_debug) app_trace_log(DEBUG_LOW, "   [CHECK] Time @%01u\r", getSystemTimeMs());
	
	if( spi_state_machine_cb != NULL )
	{
		spi_state_machine_cb( true, 0 );
	}
}

/***************************************************************************//**
 * @brief
 * 		Address of Read Structure
 * @return buffer pointer		
 ******************************************************************************/
//SPI Port Initialization:  Reads Device ID, Verify if it is Device we support
T_SPIREAD_BUF * get_read_buf_ptr( void )
{
	return &read;
}

/***************************************************************************//**
 * @brief
 * 		Initialize SPI Bus, Read Device ID, Determine Device properties
 * @param[in] debug
 * 		To print or not to print messages
 * @param[in] p_static_buffer
 * 		Pointer to static buffer that calling function will receive data on
 * @param[in] buf_len
 * 		The reserved length of the static buffer
 ******************************************************************************/
//SPI Port Initialization:  Reads Device ID, Verify if it is Device we support
spiflash_id_t spiflash_init( bool debug )
{
	int i;
	ret_code_t res;
	const spiflash_info_t *p;
	
	spif_debug = debug;
	spiflash_info = NULL;
	spi_state_machine_cb = NULL;
	user_cb = NULL;

	if( spif_debug ) app_trace_log( DEBUG_LOW, "[SPIINIT] Start\r" );
	
	// Pull Up Hold Pin to allow SPI flash to communicate
	nrf_gpio_pin_set( SPI_HOLD_PIN );
	nrf_gpio_cfg_output( SPI_HOLD_PIN );
	
	//Toggle Chip Select line. Wake up Device if in Ultra Deep Sleep
	nrf_gpio_pin_clear( SPI_CS_PIN );
	nrf_gpio_cfg_output( SPI_CS_PIN );
	nrf_gpio_pin_set( SPI_CS_PIN );
	delay_us(50);
	
	//Default to 0xFF to test how far first read really goes
	//memset( read.buf, 0xFF, SPI_BUF_SIZE );
	
	//create check timer for spiflash.c to use
	res = app_timer_create( &m_NVM_timer_id, APP_TIMER_MODE_SINGLE_SHOT, check_timer_cb );
	APP_ERROR_CHECK(res);
	
	res = hal_spim_init( debug );
	if( res != NRF_SUCCESS )
	{
		app_trace_log(DEBUG_HIGH, "[SPIINIT] Err\r");
		return PART_UNKNOWN;
	}
	
//	//Setup MISO line as interrupt when spiflash completes an operation (no longer busy)
//	if( nrf_drv_gpiote_is_init() == false )
//	{
//		res = nrf_drv_gpiote_init();
//		APP_ERROR_CHECK(res);
//	}
//	nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO( false );
//	res = nrf_drv_gpiote_in_init( SPI_MISO_PIN, &config, spi_ready_cb);
//	if( res != NRF_SUCCESS ) 
//	{
//		app_trace_log(DEBUG_HIGH, "[SPIINIT] MISO Interrupt Init Err\r");
//	}
	
	// Issue a resume from deep power down
	if( spiflash_wakeup(NULL) != NRF_SUCCESS )
	{	//data would not send, try again:
		if( spiflash_wakeup(NULL) != NRF_SUCCESS )
		{	//no go
			app_trace_log(DEBUG_HIGH, "[SPIINIT] Flash Wake Failed\r");
			return PART_UNKNOWN;
		}
	}
	
	// Issue a read ID command
	if( spiflash_read_id() != NRF_SUCCESS)
	{	//failed to send:
		if( spiflash_read_id() != NRF_SUCCESS )
		{	//no go
			app_trace_log(DEBUG_HIGH, "[SPIINIT] Flash Read Failed\r");
			return PART_UNKNOWN;
		}
	}
	if( spif_debug ) app_trace_log( DEBUG_MED, "[SPIINIT] ID: 0x%02X, 0x%02X, 0x%02X, 0x%02X\r", read.buf[1], read.buf[2], read.buf[3], read.buf[4] );
	for (i = 0; i < SPIFLASH_INFO_TABLE_SIZE; i++)
	{
		p = &spiflash_info_table[i];
		if( memcmp(&read.buf[1], p->id_bytes, p->id_size) == 0 )
		{
			spiflash_info = p;
			id = (spiflash_id_t) i;
		}
	}
		
	//Read the Status registers:
	spiflash_read_status( 2 );
	if( (read.buf[1] & READ_STATUS_SWP_MASK) != 0 )
	{	//At least some sectors are locked. Issue unlock so we start in known state.
		spiflash_write_enable( true );
		spiflash_global_protect( false );
	}	
	spiflash_read_status( 2 );	//verify that Protection Bits have cleared
	if( spif_debug ) app_trace_log( DEBUG_LOW, "[SPIINIT] Stat: 0x%02X, 0x%02X\r", read.buf[1], read.buf[2] );
	
//	//erase the first X 4K Sections of memory
//	for (int i=0; i<0x80; i++ )
//	{
//		app_trace_log(DEBUG_MED, "[SPIINIT] Erase Section 0x%02X\r", i*0x1000 );
//		spiflash_erase_4k_block( i*0x1000, NULL );
//	}
//	while(1);
	
	return id;
}

/***************************************************************************//**
 * @brief
 *   SPI Flash Command
 * @note
 * 		spiflash_command() can be used to issue any command. Since commands tend
 *		be short and coupled with other commands, this routine blocks until 
 *		the complete message has been transferred.
 * @param[in] tx_len
 * 		Command length, minimum 1 byte
 * @param[in] rx_len
 * 		Expected response length, set to 0 if no response is expected.
 * @return res
 * 		NRF_XXXXXX return code
 ******************************************************************************/
static ret_code_t spiflash_xfer( uint8_t * tx_data, uint16_t tx_len, uint16_t rx_len, bool hold_cs_en )
{
	ret_code_t res;
	T_HAL_SPIM_XFER spiflash;
	static uint8_t write_buf[SPI_BUF_SIZE];
	
	//Make sure parameters are acceptable:
	if( g_spiflash_busy ) 
	{
		app_trace_puts(DEBUG_MED, "[SPIXFR] Busy\r");
		return NRF_ERROR_BUSY;
	}
	else if( (rx_len + tx_len) > SPI_MAX_XFER_SIZE )
	{
		app_trace_puts(DEBUG_HIGH, "[SPIXFR] Buffer too Small\r");
		return NRF_ERROR_INVALID_LENGTH;
	}
	
	memcpy( write_buf, tx_data, tx_len);		//copy to static buffer and send!!!
	spiflash.p_tx_buf = write_buf;
	spiflash.tx_len = tx_len;
	spiflash.p_rx_buf = read.buf;
	if( rx_len != 0 ) spiflash.rx_len = (rx_len + tx_len);
	else spiflash.rx_len = 0;
	spiflash.cb = &transfer_complete_cb;
	spiflash.hold_cs_pin = hold_cs_en;
	spiflash.cs_pin = SPI_CS_PIN;
	
	g_spiflash_busy = true;			//a message is about to start, indicate SPI port is busy
	g_spiflash_res = false;
	res = hal_spim_xfer( &spiflash );
	
	return res;
}

/***************************************************************************//**
 * @brief
 *   SPI Flash Command with Address
 * @note
 * 		spiflash_address_command() can be used to issue commands with a starting
 * 		address of any length. The reading of the response will begin after the last
 *		command byte is written. If no response is to be read, pass rx_len == 0.
 * @param[in] cmd
 * 		Command to send
 * @param[in] addr
 * 		Address for command to use
 * @param[in] rx_len
 * 		Expected response length, set to 0 if no response is expected.
 ******************************************************************************/
static ret_code_t spiflash_address_command(uint8_t cmd, uint32_t addr, uint16_t rx_len, bool hold_cs_en )
{
	int i = 0;
	ret_code_t res;
	uint8_t tx_buf[4];

	tx_buf[i++] = cmd;
	if (spiflash_info->address_bytes >= 3)
		tx_buf[i++] = addr >> 16;
	if (spiflash_info->address_bytes >= 2)
		tx_buf[i++] = (addr >> 8) & 0xff;
	if (spiflash_info->address_bytes >= 1)
		tx_buf[i++] = addr & 0xff;

	res = spiflash_xfer( tx_buf, i, rx_len, hold_cs_en );
	
	return res;
}

/***************************************************************************//**
 * @brief
 *   SPI Flash Command with Address
 * @note
 * 		spiflash_address_command() can be used to issue commands with a starting
 * 		address of any length. The reading of the response will begin after the last
 *		command byte is written. If no response is to be read, pass rx_len == 0.
 * @param[in] cmd
 * 		Command to send
 * @param[in] addr
 * 		Address for command to use
 * @param[in] rx_len
 * 		Expected response length, set to 0 if no response is expected.
 ******************************************************************************/
static ret_code_t spiflash_write_data( uint32_t addr, uint8_t * data, uint16_t tx_len )
{
	int i = 0;
	ret_code_t res;
	uint8_t tx_buf[SPI_BUF_SIZE];

	tx_buf[i++] = CMD_BYTE_PAGE_PROGRAM;
	if (spiflash_info->address_bytes >= 3)
		tx_buf[i++] = addr >> 16;
	if (spiflash_info->address_bytes >= 2)
		tx_buf[i++] = (addr >> 8) & 0xff;
	if (spiflash_info->address_bytes >= 1)
		tx_buf[i++] = addr & 0xff;
	
	memcpy( &tx_buf[i], data, tx_len ); 
	
	res = spiflash_xfer( tx_buf, (i+tx_len), 0, false );
	
	if( res == NRF_SUCCESS && spi_state_machine_cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 *   State Machine for handling the Erase operation
 * @param[in] true/false
 * 		Indicates whether last event executed successfully
 * @param[in] len
 * 		Amount of data received during last event
 ******************************************************************************/
static struct {
	uint8_t state;
	uint32_t addr;
	uint32_t size;
	uint8_t cmd;
} erase = { 0, 0, 0, 0};
static void erase_sm( bool success, uint16_t len )
{
	static uint8_t retry = 0;
	ret_code_t res;
	T_DEBUG_PRIORITY db_priority = DEBUG_LOW;	//Set the none Error related messages priority
	
	if( !success ) erase.state = 0xFF;
	
	switch( erase.state )
	{
		case 0:
			//Send command to enable WEL bit (Write Enable)
			if(spif_debug) app_trace_log(db_priority, "  [SPIER] Wr En\r");
		
			erase.state++;
			res = spiflash_write_enable( true );
			if( res != NRF_SUCCESS ) erase.state = 0xFF;
			break;
		
		case 1:
			//Callback after WEL bit has been set. Check Status Registers	
			erase.state++;
			res = spiflash_read_status( 1 );
			if( res != NRF_SUCCESS ) 
			{
				erase.state = 0xFF;
				app_trace_log(DEBUG_MED, "  [SPIER] Stat Read Failed\r");
			}
			else
			{
				if(spif_debug) app_trace_log(db_priority, "  [SPIER] Stat Issued\r");
			}
			break;
		
		case 2:
			//Callback after Read of Status Register. If good, send Erase Command.
			if(false) 
			{
				app_trace_log(db_priority, "  [SPIER] Stat: ");
				for( int i=0; i<read.len; i++ )
				{
					app_trace_log(db_priority, "0x%02X, ", read.buf[i]);
				}
				app_trace_log(db_priority, "\r");
			}
			
			if( (read.buf[1]&READ_STATUS_RDY_BSY_MASK) != 0 )
			{	//Device Busy
				app_trace_log(DEBUG_MED, "  [SPIER] Device Busy\r");
				erase.state = 0xFF;
			}
			else if( (read.buf[1]&READ_STATUS_WEL_MASK) != 0 )
			{
				if(spif_debug) app_trace_log(db_priority, "  [SPIER] Erase Issued\r");
			
				erase.state++;
				res = spiflash_address_command( erase.cmd, erase.addr, 0, false );
				if( res != NRF_SUCCESS ) erase.state = 0xFF;
			}
			else
			{	//Write Enable Bit did not set
				erase.state = 0xFF;
			}
			break;
		
		case 3:
			//callback after erase has been issued
			if(spif_debug) app_trace_log(db_priority, "  [SPIER] Status Out En\r");
		
			erase.state++;
			res = spiflash_enable_active_status();	//Output Status Bit STATUS_RDY_BSY on MISO pin
			if( res != NRF_SUCCESS ) 
			{
				app_trace_log(DEBUG_HIGH, "  [SPIER] Active Status Err %01u\r", res);
				erase.state = 0xFF;
			}
			break;
		
		case 4:
			//callback after Enable Active Status has been written
			if( nrf_gpio_pin_read( SPI_MISO_PIN ) )
			{	//still busy
				if(spif_debug) app_trace_log(db_priority, "  [SPIER] Wait Erase %01u @ %01u\r", nrf_gpio_pin_read( SPI_MISO_PIN ), getSystemTimeMs());	
				
				//enable check timer...
				start_check_timer( 5 );	//generate interrupt in 5 milliseconds to check again for completion
			}
			else
			{	//no longer busy
				erase.state++;
				retry = 0;
				res = spiflash_read_status( 1 );
				if( res != NRF_SUCCESS ) erase.state = 0xFF;			
			}
			break;

		case 5:
			//Callback after Read of Status Register. If good, we're done:
			if(spif_debug) app_trace_log(db_priority, "  [SPIER] Stat: 0x%02X\r", read.buf[1]);
			if( (read.buf[1]&READ_STATUS_RDY_BSY_MASK) != 0 )
			{	//write has not completed
				app_trace_log(DEBUG_MED, "  [SPIER] Device Busy\r");
				if( retry++ < 5 ) 
				{
					res = spiflash_read_status( 1 );
					if( res != NRF_SUCCESS ) erase.state = 0xFF;
				}
				else
				{	//Time to quit
					erase.state = 0xFF;
				}
			}
			else if( (read.buf[1]&READ_STATUS_EPE_MASK) == 0 )
			{
				if(spif_debug) app_trace_log(db_priority, "  [SPIER] Cmplt @%01u\r", getSystemTimeMs());
				spi_state_machine_cb = NULL;
				erase.state = 0xFE;	//Finished
			
				if( user_cb != NULL ) user_cb( true );	//inform function that called write of completion
			}
			else
			{	//A byte did not write Correctly
				app_trace_log(DEBUG_HIGH, "  [SPIER] EPE Failure\r", getSystemTimeMs());
				erase.state = 0xFF;
			}
			break;
		
		default:
			erase.state = 0xFF;
			break;
	}
	
	if( erase.state == 0xFF )
	{	//operation has failed
		app_trace_log(DEBUG_HIGH, "  [SPIER] Fail @%01u\r\r\r", getSystemTimeMs());
		spi_state_machine_cb = NULL;
		g_spiflash_busy = false;
		
		if( user_cb != NULL ) user_cb( false );	//inform function that called erase of completion
	}
}

/***************************************************************************//**
 * @brief
 *   SPI Flash Erase Page. Start erase procedure for a 256 byte page
 * @param[in] page_addr
 * 		Address of Page to be Erased
 * @param[in] cb
 * 		cb acts as a key to access this function (successully registered using 
 *		SPIFLASH_LOCK( rd_wr_er_cmplt_cb cb )).
 * @return ret_code_t
 * 		Result of starting the page erase operation
 ******************************************************************************/
ret_code_t spiflash_page_erase( uint32_t page_addr, rd_wr_er_cmplt_cb cb )
{	
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIPGER] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	else if( id != PART_UNKNOWN && page_addr > spiflash_info_table[id].device_size )
	{
		app_trace_log(DEBUG_MED, "  [SPIPGER] Address Invalid\r");
		return NRF_ERROR_INVALID_ADDR;
	}
	else if( (page_addr&0x0000FF) != 0 )
	{	//addr not aligned to page boundary
		app_trace_log(DEBUG_MED, "  [SPIPGER] Addr Boundary Error: 0x%06X\r", page_addr);
		page_addr &= 0xFFFFFF00;	//align to page boundary
	}
	
	erase.state = 0;
	erase.size = 256;
	erase.addr = page_addr;
	erase.cmd = CMD_PAGE_ERASE;
	spi_state_machine_cb = &erase_sm;
	
	//kickoff event driven Erase State Machine
	erase_sm(true, 0);
	
	if( cb == NULL )
	{	//Caller wants to synchronously erase. Wait for state machine to complete
		while( erase.state < 0xFE )
		{
			__WFE();
		}
		if( erase.state == 0xFF ) 
		{
			if( false )
			{	//Print memory contents on failure!
				SPIFLASH_RELEASE(cb);
				spiflash_read( erase.addr, erase.size, NULL );
				
				app_trace_log(DEBUG_MED, "  [SPIPGER] Read on Err AD:0x%06X\r", erase.addr);
				for( int i=0; i<(read.len-SPI_BUF_DATA_OFFSET); i++ )
				{
					app_trace_log(DEBUG_MED, "%02X ", read.buf[SPI_BUF_DATA_OFFSET+i]);
				}
				app_trace_log(DEBUG_MED, "\r");
			}
			
			return NRF_ERROR_INTERNAL;	//operation failed
		}

	}
	
	return NRF_SUCCESS;
}

/***************************************************************************//**
 * @brief
 *   SPI Flash Erase 4K Block
 * @param[in] block_addr
 * 		Address of 4K Block to be Erased
 * @param[in] cb
 * 		cb acts as a key to access this function (successully registered using 
 *		SPIFLASH_LOCK( rd_wr_er_cmplt_cb cb )).
 * @return ret_code_t
 * 		Result of starting the block erase operation
 ******************************************************************************/
ret_code_t spiflash_erase_4k_block( uint32_t block_addr, rd_wr_er_cmplt_cb cb )
{
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPI4KER] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	else if( id != PART_UNKNOWN && block_addr > spiflash_info_table[id].device_size )
	{
		app_trace_log(DEBUG_MED, "  [SPI4KER] Address Invalid\r");
		return NRF_ERROR_INVALID_ADDR;
	}
	else if( (block_addr&0x000FFF) != 0 )
	{	//addr not aligned to 4K block boundary
		app_trace_log(DEBUG_MED, "  [SPI4KER] Addr Boundary Error: 0x%06X\r", block_addr);
		block_addr &= 0xFFFFF000;	//align to block boundary
	}
	
	erase.state = 0;
	erase.size = 4096;
	erase.addr = block_addr;
	erase.cmd = CMD_BLOCK_ERASE;
	spi_state_machine_cb = &erase_sm;
				
	//kickoff event driven Erase State Machine
	erase_sm(true, 0);
	
	if( cb == NULL )
	{	//Caller wants to synchronously erase. Wait for state machine to complete
		while( erase.state < 0xFE )
		{
			__WFE();
		}
		if( erase.state == 0xFF ) 
		{
			return NRF_ERROR_INTERNAL;	//operation failed
		}
	}
	
	return NRF_SUCCESS;
}


/***************************************************************************//**
 * @brief
 *   State Machine for handling the Erase operation
 * @param[in] true/false
 * 		Indicates whether last event executed successfully
 * @param[in] len
 * 		Amount of data received during last event
 ******************************************************************************/
static struct {
	uint8_t state;
	uint32_t addr;
	uint32_t len;
	uint8_t * data;
} write = { 0, 0, 0, 0};
static void write_sm( bool success, uint16_t len )
{
	static uint8_t retry = 0;
	ret_code_t res;
	T_DEBUG_PRIORITY db_priority = DEBUG_LOW;	//Set the none Error related messages priority
	
	if( !success ) write.state = 0xFF;
	
	switch( write.state )
	{
		case 0:
			//Send command to enable WEL bit (Write Enable)
			if(spif_debug) app_trace_log(db_priority, "  [SPIWR] Wr En\r");
		
			write.state++;
			res = spiflash_write_enable( true );
			if( res != NRF_SUCCESS ) write.state = 0xFF;
			break;
		
		case 1:
			//Callback after WEL bit has been set. Check Status Registers	
			write.state++;
			res = spiflash_read_status( 1 );
			if( res != NRF_SUCCESS )
			{
				app_trace_log(DEBUG_HIGH, "  [SPIWR] Stat Read Failed\r");
				write.state = 0xFF;
			}
			else
			{
				if(spif_debug) app_trace_log(db_priority, "  [SPIWR] Stat Issued\r");
			}
			break;
		
		case 2:
			//Callback after Read of Status Register. If good, send Write Command.
			if(false) 
			{
				app_trace_log(db_priority, "  [SPIWR] Stat: ");
				for( int i=0; i<read.len; i++ )
				{
					app_trace_log(db_priority, "0x%02X, ", read.buf[i]);
				}
				app_trace_log(db_priority, "\r");
			}
			
			if( (read.buf[1]&READ_STATUS_RDY_BSY_MASK) != 0 )
			{	//Device Busy
				app_trace_log(DEBUG_MED, "  [SPIWR] Device Busy\r");
				write.state = 0xFF;
			}
			else if( (read.buf[1]&READ_STATUS_WEL_MASK) != 0 )
			{			
				write.state++;
				res = spiflash_write_data( write.addr, write.data, write.len );
				if( res != NRF_SUCCESS ) write.state = 0xFF;
			}
			else
			{	//Write Enebale Bit did not set
				app_trace_log(DEBUG_HIGH, "  [SPIWR] Write En Err %01u\r", res);
				write.state = 0xFF;
			}
			break;
		
		case 3:
			//callback after Write command has been issued
			write.state++;		
			
			res = spiflash_enable_active_status();	//Output Status Bit STATUS_RDY_BSY on MISO pin
			if( res != NRF_SUCCESS ) 
			{
				app_trace_log(DEBUG_HIGH, "  [SPIWR] Act_Stat Err %01u\r", res);
				write.state = 0xFF;
			}			
			break;
		
		case 4:
			//callback after Enable Active Status has been written
			if( nrf_gpio_pin_read( SPI_MISO_PIN ) )
			{	//still busy
				if(spif_debug) app_trace_log(db_priority, "  [SPIWR] Wait Write @ %01u\r", getSystemTimeMs());	
				
				//enable check timer...
				start_check_timer( 5 );	//generate interrupt in 5 milliseconds to check again for completion
			}
			else
			{	//no longer busy
				write.state++;
				retry = 0;
				res = spiflash_read_status( 1 );
				if( res != NRF_SUCCESS ) write.state = 0xFF;			
			}
			break;

		case 5:
			//Callback after Read of Status Register. If good, we're done:
			if(spif_debug) app_trace_log(db_priority, "  [SPIWR] Stat: 0x%02X\r", read.buf[1]);
			if( (read.buf[1]&READ_STATUS_RDY_BSY_MASK) != 0 )
			{	//write has not completed
				app_trace_log(DEBUG_MED, "  [SPIWR] Device Busy\r");
				if( retry++ < 5 ) 
				{
					res = spiflash_read_status( 1 );
					if( res != NRF_SUCCESS ) write.state = 0xFF;
				}
				else
				{	//Time to quit
					write.state = 0xFF;
				}
			}
			else if( (read.buf[1]&READ_STATUS_EPE_MASK) == 0 )
			{
				if(spif_debug) app_trace_log(db_priority, "  [SPIWR] Cmplt @%01u\r", getSystemTimeMs());
				spi_state_machine_cb = NULL;
				write.state = 0xFE;	//Finished
			
				if( user_cb != NULL ) user_cb( true );	//inform function that called write of completion
			}
			else
			{	//A byte did not write Correctly
				app_trace_log(DEBUG_HIGH, "  [SPIWR] EPE Failure\r", getSystemTimeMs());
				write.state = 0xFF;
			}
			break;
		
		default:
			write.state = 0xFF;
			break;
	}
	
	if( write.state == 0xFF )
	{	//operation has failed
		app_trace_log(DEBUG_HIGH, "  [SPIWR] Fail @%01u\r\r\r", getSystemTimeMs());
		spi_state_machine_cb = NULL;
		g_spiflash_busy = false;
		
		if( user_cb != NULL ) user_cb( false );	//inform calling function of write failure
	}
}

/***************************************************************************//**
 * @brief
 *   SPI Flash Write Byte/Page
 * @note
 *		Can write upto 256 bytes in a page. It cannot write beyond the page boundary
 *		of the page that the addr is in. Thus, addr_page_offset + len must not
 *		cross into the next page.
 * @param[in] addr
 * 		Address of starting byte to write
 * @param[in] *data
 * 		data to be written from addr to addr+len
 * @param[in] len
 * 		Number of Bytes to write
 * @param[in] cb
 * 		cb acts as a key to access this function (successully registered using 
 *		SPIFLASH_LOCK( rd_wr_er_cmplt_cb cb )).
 * @return ret_code_t
 * 		NRF_SUCCESS, NRF_ERROR_BUSY, NRF_ERROR_INVALID_LENGTH
 ******************************************************************************/
ret_code_t spiflash_byte_page_write( uint32_t addr, uint8_t * data, uint16_t len, rd_wr_er_cmplt_cb cb )
{	
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIPGWR] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	else if( id != PART_UNKNOWN && addr > spiflash_info_table[id].device_size )
	{
		app_trace_log(DEBUG_MED, "  [SPIPGWR] Address Invalid\r");
		return NRF_ERROR_INVALID_ADDR;
	}
	else if( ((uint16_t)(addr&0x000000FF) + len) > 0x100 ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIPGWR] Error 0x%02X+0x%02X, 0x%04X\r", (addr&0x000000FF), len, addr);
		return NRF_ERROR_INVALID_LENGTH;
	}
		
	write.state = 0;
	write.len = len;
	write.addr = addr;
	write.data = data;
	spi_state_machine_cb = &write_sm;
	
	//kickoff event driven Write State Machine
	write_sm(true, 0);
	
	if( cb == NULL )
	{	//Caller wants to synchronously write. Wait for state machine to complete
		while( write.state < 0xFE )
		{
			__WFE();
		}
		if( write.state == 0xFF ) 
		{
			if( false )
			{	//Print memory contents on failure!
				app_trace_log(DEBUG_MED, "  [SPIPGWR] MISO PIN = %01u\r", nrf_gpio_pin_read( SPI_MISO_PIN ));
				
				SPIFLASH_RELEASE(cb);
				spiflash_read( write.addr, write.len, NULL );
				
				app_trace_log(DEBUG_MED, "  [SPIPGWR] Read on Err AD:0x%06X LN:0x%02X\r", write.addr, write.len);
				for( int i=0; i<(read.len-SPI_BUF_DATA_OFFSET); i++ )
				{
					app_trace_log(DEBUG_MED, "%02X ", read.buf[SPI_BUF_DATA_OFFSET+i]);
				}
				app_trace_log(DEBUG_MED, "\r");
			}

			return NRF_ERROR_INTERNAL;	//operation failed
		}
		
		if( false ) 
		{
			app_trace_log(DEBUG_MED, "  [SPIPGWR] Ad:0x%06X ", write.addr);
			for( int i=0; i<write.len; i++ )
			{
				app_trace_log(DEBUG_MED, "0x%02X, ", write.data[i]);
			}
			app_trace_log(DEBUG_MED, "\r");
		}
	}
	
	return NRF_SUCCESS;
}

/***************************************************************************//**
 * @brief
 *   State Machine for handling the Memory section protection operation
 * @param[in] true/false
 * 		Indicates whether last event executed successfully
 * @param[in] len
 * 		Amount of data received during last event
 ******************************************************************************/
static struct {
	uint8_t state;
	uint32_t addr;
	uint8_t cmd;
} protect = { 0, 0, 0 };
static void protect_sm( bool success, uint16_t len )
{
	ret_code_t res;
	T_DEBUG_PRIORITY db_priority = DEBUG_LOW;	//Set the none Error related messages priority
	
	if( !success ) protect.state = 0xFF;
	
	switch( protect.state )
	{
		case 0:
			//Send command to enable WEL bit (Write Enable)
			if(spif_debug) app_trace_log(db_priority, "  [SPIPRT] Wr En\r");
		
			protect.state++;
			res = spiflash_write_enable( true );
			if( res != NRF_SUCCESS ) protect.state = 0xFF;
			break;
		
		case 1:
			//Callback after WEL bit has been set. Check Status Registers			
			protect.state++;
			res = spiflash_read_status( 1 );
			if( res != NRF_SUCCESS )
			{
				protect.state = 0xFF;
				app_trace_log(DEBUG_MED, "  [SPIPRT] Stat Read Failed\r");
			}
			else
			{
				if(spif_debug) app_trace_log(db_priority, "  [SPIPRT] Stat Issued\r");
			}
			break;
		
		case 2:
			//Callback after Read of Status Register. If good, send Erase Command.
			if(false) 
			{
				app_trace_log(db_priority, "  [SPIPRT] Stat: ");
				for( int i=0; i<read.len; i++ )
				{
					app_trace_log(db_priority, "0x%02X, ", read.buf[i]);
				}
				app_trace_log(db_priority, "\r");
			}
			
			if( (read.buf[1]&READ_STATUS_RDY_BSY_MASK) != 0 )
			{	//Device Busy
				app_trace_log(DEBUG_MED, "  [SPIPRT] Device Busy\r");
				protect.state = 0xFF;
			}
			else if( (read.buf[1]&READ_STATUS_WEL_MASK) != 0 )
			{
				if(spif_debug) app_trace_log(db_priority, "  [SPIPRT] Protect Issued\r");
			
				protect.state++;
				res = spiflash_address_command( protect.cmd, protect.addr, 0, false );
				if( res != NRF_SUCCESS ) protect.state = 0xFF;
			}
			else
			{	//Write bit did not set
				protect.state = 0xFF;
			}
			break;
		
		case 3:
			//Callback after Protect/Unprotect Command issued
			if(spif_debug) app_trace_log(db_priority, "  [SPIPRT] Cmplt @%01u\r", getSystemTimeMs());
			spi_state_machine_cb = NULL;
			protect.state = 0xFE;		//Finished
		
			if( user_cb != NULL ) user_cb( true );	//inform function that called erase of completion
			break;
		
		default:
			protect.state = 0xFF;
			break;
	}
	
	if( protect.state == 0xFF )
	{	//operation has failed
		app_trace_log(DEBUG_HIGH, "  [SPIPRT] Fail @%01u\r", getSystemTimeMs());
		spi_state_machine_cb = NULL;
		g_spiflash_busy = false;
		
		if( user_cb != NULL ) user_cb( false );	//inform function that called erase of completion
	}
}

/***************************************************************************//**
 * @brief
 * 		Send Sector Protect/Unprotect Command
 * @param[in] protect
 * 		True/False
 * @param[in] addr
 * 		Address/Sector to protect
 ******************************************************************************/
ret_code_t spiflash_sector_protection( bool protection, uint32_t addr, rd_wr_er_cmplt_cb cb )
{
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIPRT] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}

	protect.state = 0;
	protect.addr = addr;
	protect.cmd = protection? CMD_PROTECT_SECTOR : CMD_UNPROTECT_SECTOR;
	spi_state_machine_cb = &protect_sm;
	
	protect_sm( true, 0 );
		
	if( cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( protect.state == 0xFF ) return NRF_ERROR_INTERNAL;	//operation failed
	}
	
	return NRF_SUCCESS;
}

/***************************************************************************//**
 * @brief
 * 		Send Write Enable/Disable Command
 * @param[in] *ref
 * 		Completion Function Pointer
 ******************************************************************************/
static ret_code_t spiflash_write_enable( bool enable )
{
	ret_code_t res;
	uint8_t cmd = enable ? CMD_WRITE_ENABLE : CMD_WRITE_DISABLE;
	
	res = spiflash_xfer( &cmd, 1, 0, false );
	
	if( res == NRF_SUCCESS && spi_state_machine_cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Send Global Protect Command
 * @param[in] protect
 * 		True/False
 ******************************************************************************/
static ret_code_t spiflash_global_protect( bool protect )
{
	ret_code_t res;
	uint8_t tx_buf[2] = 
	{ 	CMD_WRITE_STATUS_REG_BYTE_1,
		protect ? 0x3c : 0x00
	};

	if(spif_debug) app_trace_log(DEBUG_LOW, "  [SPIGP] GP = %01u\r", protect);
	
	res = spiflash_xfer( tx_buf, 2, 0, false );
	
	if( res == NRF_SUCCESS && spi_state_machine_cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Read Status Register
 * @note
 * 		For Adesto AT25XE021A and AT25XE041B, there are two bytes which can
 * 		be read with a single call with len == 2.
 * @param[in] len
 * 		Number bytes to read
 * @param[in] *buffer
 * 		Pointer to buffer store read bytes to.
 ******************************************************************************/
static ret_code_t spiflash_read_status( size_t read_len )
{
	ret_code_t res;
	uint8_t cmd = CMD_READ_STATUS;
	
	if(spif_debug) app_trace_log(DEBUG_LOW, "  [SPIRDST]\r");
	
	res = spiflash_xfer( &cmd, 1, read_len, false );
	
	if( res == NRF_SUCCESS && spi_state_machine_cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Read Device ID
 * @param[in] len
 * 		Byte to write to status register
 * @param[in] *buffer
 * 		Pointer to buffer to store Device ID
 ******************************************************************************/
static ret_code_t spiflash_read_id( void )
{
	ret_code_t res;
	uint8_t cmd = CMD_READ_ID;
	
	res = spiflash_xfer( &cmd, 1, MAX_FLASH_ID_LEN, false );
	
	if( res == NRF_SUCCESS && spi_state_machine_cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 *   SPI Flash enable Active Status Output
 * @note
 * 		CS line must be held asserted after enabling the Active Status Interrupt.
 *		Once activated, the SO line indicates the status of the !RDY/BSY Status bit.
 *		Deasserting the CS line terminates the Active Status Mode
 ******************************************************************************/
static ret_code_t spiflash_enable_active_status( void )
{
	ret_code_t res;
	uint8_t tx_buf[2];
		
	tx_buf[0] = CMD_ACTIVE_STATUS_INTERRUPT;
	tx_buf[1] = 0x00;

	res = spiflash_xfer( tx_buf, 1, 0, true );
	
	if( res == NRF_SUCCESS && spi_state_machine_cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}

	return res;
}

///***************************************************************************//**
// * @brief
// *   SPI Flash enable SO pin Interrupt
// * @note
// * 		CS line must be held asserted after enabling the Active Status Interrupt.
// *		Once activated, the SO line indicates the status of the !RDY/BSY Status bit.
// *		Deasserting the CS line terminates the Active Status Mode
// ******************************************************************************/

//static void spiflash_enable_status_interrupt( void )
//{
//	//turn on the SO Pin IRQ
//	nrf_drv_gpiote_in_event_enable( SPI_MISO_PIN, true );
//}


///***************************************************************************//**
// * @brief
// *   SPI Flash disable SO pin Interrupt
// * @note
// * 		Return everything to normal once interrupt has triggered
// ******************************************************************************/
//static void spiflash_disable_status_interrupt( void )
//{	
//	//Disable SO Interrupt
//	nrf_drv_gpiote_in_event_enable( SPI_MISO_PIN, false );
//	
//	//Stop time out timer
//	spim_timer_stop();
//	
//	//Deassert CS pin
//	nrf_gpio_pin_set( SPI_CS_PIN );	
//}

/***************************************************************************//**
 * @brief
 * 		Write First Status Register
 * @param[in] data
 * 		Byte to write to status register
 ******************************************************************************/
void spiflash_write_status(uint8_t data)
{
	//maw: this looks useful...
}

/***************************************************************************//**
 * @brief
 * 		Write Second Status Register
 * @param[in] data
 * 		Byte to write to status register
 ******************************************************************************/
void spiflash_write_status2(uint8_t data)
{
	//maw: this looks even more useful...
}

/***************************************************************************//**
 * @brief
 *   SPI Flash Read
 * @note
 * 		Calls spiflash_command_with_address to perform Read operation.
 * @param[in] addr
 * 		Address to read from
 * @param[in] len
 * 		Amount of data to be read
 * @param[in] cb
 * 		cb acts as a key to access this function (successully registered using 
 *		SPIFLASH_LOCK( rd_wr_er_cmplt_cb cb )).
 * @return ret_code_t
 * 		Result of starting the Read operation
 ******************************************************************************/
ret_code_t spiflash_read( uint32_t addr, uint16_t read_len, rd_wr_er_cmplt_cb cb )
{	
	T_DEBUG_PRIORITY db_priority = DEBUG_LOW;	//Set the none Error related messages priority
	
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIRD] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	else if( id != PART_UNKNOWN && addr > spiflash_info_table[id].device_size )
	{
		app_trace_log(DEBUG_MED, "  [SPIRD] Address Invalid\r");
		return NRF_ERROR_INVALID_ADDR;
	}
	else if( read_len > SPI_BUF_MAX_DATA )
	{
		app_trace_log(DEBUG_MED, "  [SPIRD] Buffer Too Small\r");
		return NRF_ERROR_DATA_SIZE;
	}
	
	ret_code_t res;
	uint8_t cmd;
	
	spi_state_machine_cb = NULL;			//No State machine callbacks needed, read can be done in 1 operation
	cmd = CMD_READ_ARRAY_SLOW;
	
	if(spif_debug) app_trace_log(db_priority, "  [SPIRD] AD:0x%04X LN:0x%02X\r", addr, read_len);

	read.len = 0;
	res = spiflash_address_command( cmd, addr, read_len, false );
			
	if( res == NRF_SUCCESS )
	{	
		if( cb == NULL )
		{	//Call to synchronously issue SPI message. Wait for transfer to complete
			while( g_spiflash_busy )
			{
				__WFE();
			}
			if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
			
			if( false ) 
			{
				for( int i=0; i<(read.len-SPI_BUF_DATA_OFFSET); i++ )
				{
					app_trace_log(db_priority, "%02X ", read.buf[SPI_BUF_DATA_OFFSET+i]);
				}
				app_trace_log(db_priority, "\r");
			}
		}
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "  [SPIRD] Failed AD:0x%06X, %01u\r", addr, res);
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Wake from PowerDown
 * @param[in] blocking
 * 		True = Block until complete, False = Return immediately
 ******************************************************************************/
ret_code_t spiflash_wakeup( rd_wr_er_cmplt_cb cb )
{
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIWK] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	
	ret_code_t res;
	// any command can be used to resume; device will otherwise ignore the cmd
	uint8_t cmd = CMD_RESUME_FROM_DEEP_POWER_DOWN;
	res = spiflash_xfer( &cmd, 1, 0, false );
	
	if( res == NRF_SUCCESS && cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Enter/Resume from Deep PowerDown
 * @param[in] blocking
 * 		True = Block until complete, False = Return immediately
 ******************************************************************************/
ret_code_t spiflash_deep_power_down( rd_wr_er_cmplt_cb cb )
{
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIDPD] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
		
	ret_code_t res;	
	uint8_t cmd = CMD_DEEP_POWER_DOWN;
	res = spiflash_xfer( &cmd, 1, 0, false );	//block until complete
	
	if( res == NRF_SUCCESS )
	{
		if( cb == NULL )
		{	//Call to synchronously issue SPI message. Wait for transfer to complete
			while( g_spiflash_busy )
			{
				__WFE();
			}
			if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
		}
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "  [SPIDPD] Sleep Cmd Failed %01u\r", res);
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Enter/Resume from Ultra Deep PowerDown
 * @param[in] blocking
 * 		True = Block until complete, False = Return immediately
 ******************************************************************************/
//Puts device IN/OUT ULTRA DEEP POWER DOWN, INPUT:  Boolean TRUE/FALSE
ret_code_t spiflash_ultra_deep_power_down( rd_wr_er_cmplt_cb cb )
{
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIUDPD] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	
	ret_code_t res;
	uint8_t cmd = CMD_ULTRA_DEEP_POWER_DOWN;
	res = spiflash_xfer( &cmd, 1, 0, false );
	
	if( res == NRF_SUCCESS )
	{	
		if( cb == NULL )
		{	//Call to synchronously issue SPI message. Wait for transfer to complete
			while( g_spiflash_busy )
			{
				__WFE();
			}
			if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
		}
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "  [SPIUDPD] Sleep Cmd Failed %01u\r", res);
	}
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Send Reset Command
 * @note
 * 		Currently not being used...untested
 * 		The reset command will be ignored by the flash chip unless it has been
 *		previously enabled by writing a 1 to the RSTE bit of the
 *		status register
 *
 * @param[in] void
 ******************************************************************************/
ret_code_t spiflash_reset( rd_wr_er_cmplt_cb cb )
{
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIRST] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	
	ret_code_t res;
	uint8_t cmd[2] = 
	{
		CMD_RESET,
		ARG_RESET
	};
	res = spiflash_xfer( cmd, 2, 0, false );	//block until complete
	
	if( res == NRF_SUCCESS && cb == NULL )
	{	//Call to synchronously issue SPI message. Wait for transfer to complete
		while( g_spiflash_busy )
		{
			__WFE();
		}
		if( g_spiflash_res == false ) res = NRF_ERROR_INTERNAL;	//operation failed
	}

	return res;
}

/***************************************************************************//**
 * @brief
 * 		Send Read OTP command
 * @note
 * 		Currently not being used...untested
 * @param[in] addr
 * 		OTP Address to read
 * @param[in] len
 * 		How many OTP bytes to read
 * @param[in]
 * 		Pointer to buffer to store read bytes.
 ******************************************************************************/
ret_code_t spiflash_read_otp( uint32_t addr, size_t read_len, rd_wr_er_cmplt_cb cb )
{
	if( !SPIFLASH_KEY_MATCH(cb) ) 
	{
		app_trace_log(DEBUG_MED, "  [SPIOTP] KEY Denied\r");
		return NRF_ERROR_BUSY;
	}
	
	ret_code_t res;
	
	res = spiflash_address_command( CMD_READ_OTP, addr, read_len+1, false );

	return res;
}

/***************************************************************************//**
 * @brief
 * 		Send Write OTP command
 * @note
 * 		Currently not being used...untested
 * @param[in] addr
 * 		OTP Address to write
 * @param[in] len
 * 		How many OTP bytes to write
 * @param[in] *data
 * 		Pointer to buffer with data to write.
 ******************************************************************************/
ret_code_t spiflash_write_otp( uint32_t addr, size_t wr_len, uint8_t * data )
{	
	ret_code_t res = NRF_ERROR_NOT_SUPPORTED;
	
	//res = spiflash_address_command( CMD_PROGRAM_OTP, addr, wr_len, false );
	
	return res;
}

/** @} (end addtogroup Adesto_FlashDrivers) */
