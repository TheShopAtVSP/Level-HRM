/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_app_bootloader_main main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file.
 *
 * -# Receive start data packet. 
 * -# Based on start packet, prepare NVM area to store received data. 
 * -# Receive data packet. 
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */
#include "dfu_transport_vsp.h"
#include "bootloader.h"
#include "bootloader_util.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "nrf.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_error.h"
#include "softdevice_handler_appsh.h"
#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "led.h"
#include "global.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                                       /**< Include the service_changed characteristic. For DFU this should normally be the case. */

//#define APP_TIMER_PRESCALER             0                                                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                       /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, 0)                        /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                20                                                      /**< Maximum number of events in the scheduler queue. */

APP_TIMER_DEF(m_TIMEOUT_timer_id);
#define TIMEOUT_TIME_INTERVAL			( APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER) )
app_timer_id_t g_TIMEOUT_timer_id;

APP_TIMER_DEF(m_feed_wdt_timer_id);
uint32_t wdt_feed_chan = 0;

#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] file_name   File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function to check if watchdog needs handling
 */
bool watch_dog_enabled( void )
{
	uint8_t i;
	
	if( NRF_WDT->RUNSTATUS > 0 ) {
		//Timer is Active
		for( i=0; i<8; i++ ) {
			if( (NRF_WDT->REQSTATUS&(1<<i)) > 0 ) {
				//this reload register can be used to reset the WDT
				wdt_feed_chan = i;
				break;
			}
		}
		
		return true;
	}
	
	return false;
}

/**@brief Handle events from tick timer.
 *
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void  timeout_timer_handler(void * p_context)
{
	//if (!m_dfu_in_progress) {
		// indicate white
		led( 1, 1, 1, -1, 0);
		NVIC_SystemReset();	
	//}
}

/**@brief Handle events from feed wdt timer.
 *
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void  feed_wdt_timer_handler(void * p_context)
{
	//feed the dog
	NRF_WDT->RR[ wdt_feed_chan ] = 0x6E524635;
}


/**@brief Function for initializing the timer handler module (app_timer).
 */
static void timers_init(void)
{
	uint32_t err_code;
	
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
	
	err_code = app_timer_create( &m_TIMEOUT_timer_id, APP_TIMER_MODE_REPEATED, timeout_timer_handler );
	APP_ERROR_CHECK(err_code);
	g_TIMEOUT_timer_id = m_TIMEOUT_timer_id;
	
	if( watch_dog_enabled() ) {
		uint32_t feed_period = (NRF_WDT->CRV/2)/(APP_TIMER_PRESCALER+1);	//trigger timer to feed when Watch Dog is half drained
		
		//Watch Dog is On and will need feeding. Create Timer to do that
		NRF_WDT->RR[ wdt_feed_chan ] = 0x6E524635;
		
		err_code = app_timer_create( &m_feed_wdt_timer_id, APP_TIMER_MODE_REPEATED, feed_wdt_timer_handler );
		APP_ERROR_CHECK(err_code);

		err_code = app_timer_start( m_feed_wdt_timer_id, feed_period, NULL );	
		APP_ERROR_CHECK(err_code);
	}
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void sys_evt_dispatch(uint32_t event)
{
    pstorage_sys_event_handler(event);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 *
 * @param[in] init_softdevice  true if SoftDevice should be initialized. The SoftDevice must only 
 *                             be initialized if a chip reset has occured. Soft reset from 
 *                             application must not reinitialize the SoftDevice.
 */
static void ble_stack_init(bool init_softdevice)
{
    uint32_t         err_code;
    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };
	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
	
    if (init_softdevice)
    {
        err_code = sd_mbr_command(&com);
        APP_ERROR_CHECK(err_code);
    }
    
    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
    APP_ERROR_CHECK(err_code);
   
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    // Only one connection as a central is used when performing dfu.
    err_code = softdevice_enable_get_default_config(1, 1, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for event scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t p_info)
{
	error_info_t * err_info = (error_info_t *) p_info;

	if(err_info->err_code != NRF_SUCCESS)
	{
#ifdef APP_TRACE_ON
		app_trace_log("Err %02u: l-%01u\r", err_info->err_code, err_info->line_num);	//, err_info->p_file_name);
#endif
	}
	return;
}

/**@brief Function for bootloader main entry.
 */
int main(void)
{
    uint32_t 	err_code;
    bool 		app_reset = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_START);

    if (app_reset)
    {
        NRF_POWER->GPREGRET = 0;
    }

#ifdef APP_TRACE_ON	
	app_trace_init();
	app_trace_log("\n\rHello World!!!\r");
#endif
	
    led_init(false, NULL);

    // This check ensures that the defined fields in the bootloader corresponds with actual
    // setting in the chip.
    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

    // Initialize.
    timers_init();

    (void)bootloader_init();
	
    if (bootloader_dfu_sd_in_progress())
    {
		// indicate white
		led( 1, 1, 1, -1, 0);

        err_code = bootloader_dfu_sd_update_continue();
        APP_ERROR_CHECK(err_code);

        ble_stack_init(!app_reset);
        scheduler_init();

        err_code = bootloader_dfu_sd_update_finalize();
        APP_ERROR_CHECK(err_code);
#ifdef APP_TRACE_ON
		app_trace_log("bootloader_dfu_sd_update_finalize() Done\r");
#endif
		// indicate off
		led( 0, 0, 0, 0, -1);
    }
    else
    {
        // If stack is present then continue initialization of bootloader.
        ble_stack_init(true); 
        scheduler_init();
    }
	
//#define MARK_BOOT_VERSION
#ifdef MARK_BOOT_VERSION
	//By including the lines below, the App code will execute and better still the bootloader_settings will be updated.
	//Once those are updated, you can comment this line back out (and reload) and the code will stop hanging in the bootloader
	dfu_update_status_t update_status = { DFU_UPDATE_APP_COMPLETE, };
	bootloader_dfu_update_process( update_status );
#endif

    if ( app_reset || (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)) )
    {
		if ( app_reset) {
			// start timeout timer
			err_code = app_timer_start(m_TIMEOUT_timer_id, TIMEOUT_TIME_INTERVAL , NULL);	
			APP_ERROR_CHECK(err_code);
		}
		
		// indicate white
		led( 1, 1, 1, -1, 0);

        // Initiate an update of the firmware.
        err_code = bootloader_dfu_start();
        APP_ERROR_CHECK(err_code);

		// indicate off
		led( 0, 0, 0, 0, -1);
    }

    if (bootloader_app_is_valid(DFU_BANK_0_REGION_START) && !bootloader_dfu_sd_in_progress())
    {
        // Select a bank region to use as application region.
        // @note: Only applications running from DFU_BANK_0_REGION_START is supported.
        bootloader_app_start(DFU_BANK_0_REGION_START);
    }
    // indicate white
	led( 1, 1, 1, -1, 0);
    NVIC_SystemReset();
}
