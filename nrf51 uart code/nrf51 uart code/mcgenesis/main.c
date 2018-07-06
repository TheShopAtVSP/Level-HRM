/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include "global.h"
#include "nordic_common.h"
#include "bsp.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "nrf_drv_twi.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_trace.h"
#include "ble_vsp_bas.h"
#include "ble_dis.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "device_manager.h"
#include "motion\imu\hal_imu.h"
#include "motion\imu\hal_twim.h"
#include "data_crunch.h"
#include "step_detect.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "data_log\comms.h"
#include "pstorage.h"
#include "data_log\mem_manager.h"
#include "led.h"
#include "battery.h"
#include "nrf_drv_wdt.h"
#include "bootloader_types.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT

#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

//#define LF_OSCILLATOR_SRC				NRF_CLOCK_LF_SRC_XTAL		
//NRF_CLOCK_LF_SRC_RC 
//NRF_CLOCK_LF_SRC_Xtal

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1											/**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT      		0                               			/**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   		1                               			/**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Level"                               		/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "TheShop"                      				/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "LevelTemple"								/**< Model number. Will be passed to Device Information Service. */
#define FW_REV_MAJOR					0
#define FW_REV_MINOR					26									 									//** Now with more connect!*/
#define HW_REV							"B"
#define MANUFACTURER_ID                 0x1122334455                              	/**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x667788 

#define MAGIC_DFU_ENABLE_NUM			234

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL_FAST           64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS_FAST	180                                         /**< The advertising timeout (in units of seconds). */

#define APP_ADV_INTERVAL_SLOW           800                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 500 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS_SLOW 180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. Skip upto X number of connection interval responses. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND					1											/**< Perform bonding. */
#define SEC_PARAM_MITM					0											/**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES		BLE_GAP_IO_CAPS_NONE						/**< No I/O capabilities. */
#define SEC_PARAM_OOB					0											/**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE			7											/**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE			16											/**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR					FW_REV_MAJOR								/** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR					FW_REV_MINOR								/** DFU Minor revision number to be exposed. */
#define DFU_REVISION					((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)		/** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START		0x000C										/**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX					0xFFFF										/**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_bas_t                        m_bas;    
static ble_uuid_t                       m_adv_uuids[] =			{
															{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE},														 
															{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
															{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
																}; /**< Universally unique service identifiers. */

														 
static bool								m_memory_access_in_progress = false;		/**< Flag to keep track of ongoing operations on persistent memory. */
static bool								nus_busy = false;
extern T_CONFIG							g_config;
static dm_application_instance_t		m_app_handle;								/**< Application identifier allocated by device manager */

#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t						m_dfus;										/**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT
static bool duf_enable = 0;
#define LED_ID_KEY 136
static uint8_t led_id_key = 0;
static uint8_t led_id_key_to = 0;
uint8_t led_id_key_state = 0;
uint8_t led_id_key_numb = 0;																
																
//
// globals
//
volatile uint32_t						gs_timecounter = 0;
volatile bool							gb_wakeup;
#define	 ONESEC_MS						1000
uint8_t									gu_battery_state;
uint8_t									gu_prev_battery_state = 0xFF;
uint8_t									gu_battery_level = 50;
<<<<<<< Updated upstream
bool									g_bpowered_down = false;
static bool 							g_update_and_reset = false;
uint32_t 								fw_rev = ((FW_REV_MAJOR<<8)|FW_REV_MINOR);
uint8_t								    connected_not_bonded = 0; //in DevMagPer.c as extern for lightshow semiphore
=======
bool										g_bpowered_down = false;
static bool 						g_update_and_reset = false;
uint8_t									connected_and_bonded = 0; //in DevMagPer.c as extern for lightshow semiphore
>>>>>>> Stashed changes

//
// main locals
//
nrf_drv_wdt_channel_id m_wdt_channel_id;

//
// steps
//
uint32_t CurrentStepTotal = 0;

//
// debugging defines
//
#ifdef DEBUG
	#define IMU_DEBUG				true
	#define MAIN_DEBUG				true
	#define BAT_DEBUG				false
	#define MOTION_DEBUG			true
	#define LED_DEBUG				false		
#else
	#define IMU_DEBUG				true
	#define MAIN_DEBUG				true
	#define BAT_DEBUG				false
	#define MOTION_DEBUG			false
	#define LED_DEBUG				false		
#endif
#define UART_ENABLE		(IMU_DEBUG|MAIN_DEBUG|BAT_DEBUG|MOTION_DEBUG|LED_DEBUG)
	
bool gs_bdebug = MAIN_DEBUG;
	
//
// timers
//
#define SYS_TICK_PERIOD		125
#define SYS_TIME_PERIOD		(1*SYS_TICK_PERIOD)
static volatile uint32_t systick_ms = 0;
static uint32_t systick_period_ms = SYS_TIME_PERIOD;
static uint32_t unix_time = 0;
	
#define APP_TIMERS_NUMBER				(4 + 1)
#define APP_TIMER_MAX_TIMERS            (APP_TIMERS_NUMBER + 1)                 	/**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */						
//define MS_TO_TICK(MS) 					( APP_TIMER_TICKS(SYS_TICK_PERIOD, APP_TIMER_PRESCALER*(MS/100)) )
#define	SYSTEM_TIME_INTERVAL	 		( APP_TIMER_TICKS(SYS_TIME_PERIOD, APP_TIMER_PRESCALER) )

APP_TIMER_DEF(m_SYST_timer_id);
APP_TIMER_DEF(m_BATTERY_MEASURE_timer_id);
APP_TIMER_DEF(m_LED_FLASH_timer_id);

// Default Main Loop task times:
#define DEFAULT_IMU_CHECK_PERIOD		60000*1	//time between imu checks

//
// imu/mag interrupt
//
static bool imu_int_req = false;


/**@brief Timing Functions
 *
 * @details These functions track time for the use of periodic tasks and time stamping.
 */
uint32_t get_unix_time( void )
{
	return unix_time;
}

void set_unix_time( uint32_t new_time )
{
	unix_time = new_time;
}

__INLINE uint32_t getSystemTimeMs(void)
{
	return systick_ms;
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	error_info_t err_info =
    {
        .line_num    = line_num,
        .p_file_name = p_file_name,
        .err_code    = NRF_ERROR_SVC_HANDLER_MISSING	//as good as any guess???
    };
	
    app_error_fault_handler(DEAD_BEEF, line_num, (uint32_t) &err_info);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	//Set the Preffered Peripheral Connection Parameters
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
																					
	// cut tx power
	sd_ble_gap_tx_power_set(-8);
}

//#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

}

/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
		
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }
	
	// indicate yellow
	ledraw(1,1,0);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
//#endif // BLE_DFU_APP_SUPPORT

void disconnect_and_advertize(void)
{
	uint32_t      err_code;
	
	err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	APP_ERROR_CHECK(err_code);
	err_code = ble_advertising_start(BLE_ADV_MODE_SLOW); 
	APP_ERROR_CHECK(err_code);
	led_id_key_state = 0;
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
void dfu_control_data_handler(ble_nus_t * p_bas, uint8_t * p_data, uint16_t length)
{
	volatile uint8_t magic;
	volatile uint32_t retval = NRF_SUCCESS;
	magic = *p_data;
<<<<<<< Updated upstream
	uint32_t err_code;

=======
	
>>>>>>> Stashed changes
	if(led_id_key_state == 2)
	{
		app_trace_log("LEDID State 2\r");
		if ( (length == 1) && ( magic == led_id_key_numb ) )
		{
			led_id_key_state = 3;
			led_id_key = 1;
			duf_enable = 1;
		}
		else
		{
			disconnect_and_advertize();
			app_trace_log("LEDID State 2 -> 0 user code failed\r");
			led_light_show( false, m_LED_FLASH_timer_id );
			led_id_key = 0;
			led_id_key_to = 0;
			gu_prev_battery_state = 10; // A cheap way to reset the led state to it's former magenta glory
			led_id_key_state = 0;
		}
	}

	if(led_id_key_state == 0)
	{
		app_trace_log("LEDID State 0\r");
		if ( (length == 1) && ( magic == LED_ID_KEY ) )
		{
			led_id_key_state = 1;
			led_id_key_to = 0;
			led_id_key_numb = (uint8_t)systick_ms; 
		}
	}


	if(duf_enable)
	{
		if ( (length == 1) && ( magic == MAGIC_DFU_ENABLE_NUM ) )
		{
			if (gs_bdebug) app_trace_log("Enable DFU\r");
			//g_config.advertise_dfu = MAGIC_DFU_ENABLE_NUM;
			g_update_and_reset = true;
			duf_enable = 0;
		}
	}

}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	T_PACKET rx_msg;

	//if (gs_bdebug) app_trace_log("UART data received\r");
	
	if( length < 2 || length > sizeof(T_PACKET) ) {
		DEBUG_PRINT(0,"nus: length error",MAIN_DEBUG); 
		
		rx_msg.id = 0;
		rx_msg.type = (T_PKT_TYPES) 0;
		rx_msg.pkt_len = 0;
		rx_msg.payload[0] = 0;
	}					
	else {
		// form message packet
		rx_msg.id = p_data[0];
		rx_msg.type = (T_PKT_TYPES) p_data[1];
		rx_msg.pkt_len = length;
		for (int i=2;i<rx_msg.pkt_len;i++) {
			rx_msg.payload[i-2] = p_data[i];
			//if (gs_bdebug) app_trace_log("%02X ", p_data[i]);
		}
	}
	
	if (gs_bdebug) app_trace_log("ID=%u TYPE=%u\r",rx_msg.id ,rx_msg.type);
	
	parse_msg(&rx_msg);
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void battery_level_update_handler(void * p_context)
static void battery_level_update_handler(void)	
{
    uint32_t err_code;

	// get new data
	battery_status_level(&gu_battery_level,&gu_battery_state);
	if (gs_bdebug) {
		static uint8_t last_level = 0xFF;
		
		//Only print when Battery Remaining Level Changes
		if( gu_battery_level != last_level ) {
			last_level = gu_battery_level;
			app_trace_log("bat_level_update: @%01u level=%01u, ", systick_ms, gu_battery_level); 
			if (gu_battery_state == BATTERY_CHARGING) app_trace_s_msg("CHARGING\r"); 	
			else if (gu_battery_state == BATTERY_DISCHARGING) app_trace_s_msg("DISCHARGING\r"); 
			else app_trace_s_msg("CHARGED\r"); 	
		}
	}
	// update ble
    err_code = ble_bas_battery_level_update(&m_bas, gu_battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        //nrf5_11 update(err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
		if (gs_bdebug) app_trace_log("bat_level_update: failed %01u\r",err_code);
        APP_ERROR_HANDLER(err_code);
    }
    err_code = ble_bas_battery_state_update(&m_bas, gu_battery_state);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
         //nrf5_11 update(err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
		if (gs_bdebug) app_trace_log("battery_state_update: failed %01u\r",err_code);
        APP_ERROR_HANDLER(err_code);
    }		
}

void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *) __get_MSP(); // Get stack pointer
    uint32_t ia = sp[24/4]; // Get instruction address from stack

    if (gs_bdebug) app_trace_log("Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
	UNUSED_VARIABLE(ia);
    while(1);	
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       		err_code;
    ble_nus_init_t 		nus_init;
    ble_dis_init_t   	dis_init;
	ble_bas_init_t   	bas_init;
    ble_dis_sys_id_t 	sys_id;
	char fw_rev[6];
	
	sprintf( fw_rev, "%u.%u", FW_REV_MAJOR, FW_REV_MINOR );
	
	// Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, HW_REV);	
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, fw_rev);	
	
    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
	if ((gs_bdebug) && (err_code != NRF_SUCCESS) ) app_trace_log("services_init: dis failed %i\r",err_code);		
    APP_ERROR_CHECK(err_code);		

	// Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.write_perm);
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_state_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_state_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_state_char_attr_md.write_perm);		

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 255;
    bas_init.initial_batt_state   = 255;
	  bas_init.dfu_control_data_handler = (ble_bas_data_handler_t) dfu_control_data_handler;
		
    err_code = ble_bas_init(&m_bas, &bas_init);
	if ((gs_bdebug) && (err_code != NRF_SUCCESS) ) app_trace_log("services_init: bas failed %i\r",err_code);	
    APP_ERROR_CHECK(err_code);
	
	if (gs_bdebug) app_trace_log("Services include NUS\r");		
	// Init UART Service	
	memset(&nus_init, 0, sizeof(nus_init));

	nus_init.data_handler = nus_data_handler;
	err_code = ble_nus_init(&m_nus, &nus_init);
	if ((gs_bdebug) && (err_code != NRF_SUCCESS) ) app_trace_log("services_init: nus failed %i\r",err_code);	
	APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
		DEBUG_PRINT(0,"On Conn Param Evt: Negotiation Failed",MAIN_DEBUG); 
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
	
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;

 		case BLE_ADV_EVT_SLOW:
            break;

        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
		
			if (!g_bpowered_down) {
				DEBUG_PRINT(0,"On Adv Evt: Restarted",MAIN_DEBUG); 
				err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
				APP_ERROR_CHECK(err_code);				
			}
            break;
			
        default:
			// No implementation needed.
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                //scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
			DEBUG_PRINT(0,"On Ble Evt: Connected",MAIN_DEBUG); 
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			flag_new_comms();
			nus_busy = false;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
			DEBUG_PRINT(0,"On Ble Evt: Disconnected",MAIN_DEBUG); 
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			nus_busy = false;
			led_id_key_state = 0;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
			DEBUG_PRINT(0,"On Ble Evt: Params Req",MAIN_DEBUG); 
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
			DEBUG_PRINT(0,"On Ble Evt: Atts Missing",MAIN_DEBUG); 
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	if( gs_bdebug ) app_trace_log("Ble Evt Dispatch 0x%02X\r", p_ble_evt->header.evt_id);
	
	dm_ble_evt_handler(p_ble_evt);
	ble_nus_on_ble_evt(&m_nus, p_ble_evt);
	ble_bas_on_ble_evt(&m_bas, p_ble_evt);	
	ble_conn_params_on_ble_evt(p_ble_evt);
  
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
	
	if( p_ble_evt->header.evt_id == BLE_EVT_TX_COMPLETE ) {
		nus_busy = false;
	}
	on_ble_evt(p_ble_evt);	
	ble_advertising_on_ble_evt(p_ble_evt); 	
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);	
}


/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
	
	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	ble_enable_params_t ble_enable_params;
	err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
	APP_ERROR_CHECK(err_code);

#ifdef BLE_DFU_APP_SUPPORT
    ble_enable_params.gatts_enable_params.service_changed = 1;
#endif // BLE_DFU_APP_SUPPORT
	
	//Check the ram settings against the used number of links
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
	
	// Enable BLE stack.
	err_code = softdevice_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);
	
    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);	
}

/**@brief Handle events from tick timer.
 *
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void  ticks_timer_handler(void * p_context)
{	
	static uint16_t second_counter = 0;
	
	// updated system timer
	systick_ms += systick_period_ms;
	
	// track time in seconds based on the precision 32Khz oscillator
	second_counter += SYSTEM_TIME_INTERVAL*(APP_TIMER_PRESCALER+1);
	if ( second_counter >= APP_TIMER_CLOCK_FREQ ) {
		//second_counter = 0;					//throws away any remainder that could be there, making time inaccurate
		second_counter -= APP_TIMER_CLOCK_FREQ;	//keeps remainder that could be there, more accurate time tracking
		unix_time++;
//		nrf_gpio_pin_toggle(HEART_LED);	
	}
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
	uint32_t      err_code;
	ble_advdata_t advdata;

	// Build advertising data struct to pass into @ref ble_advertising_init.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance      = true;
	advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	advdata.uuids_complete.p_uuids  = m_adv_uuids;

	ble_adv_modes_config_t options = {0};
	options.ble_adv_whitelist_enabled 		= BLE_ADV_WHITELIST_DISABLED;
	options.ble_adv_directed_enabled 		= BLE_ADV_DIRECTED_DISABLED;
	options.ble_adv_directed_slow_enabled 	= BLE_ADV_DIRECTED_SLOW_DISABLED;
	options.ble_adv_fast_enabled  = BLE_ADV_FAST_DISABLED;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL_FAST;
	options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS_FAST;
	options.ble_adv_slow_enabled  = BLE_ADV_SLOW_ENABLED;
	options.ble_adv_slow_interval = APP_ADV_INTERVAL_SLOW;
	options.ble_adv_slow_timeout  = APP_ADV_TIMEOUT_IN_SECONDS_SLOW;

	err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
}

bool ble_uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
	uint32_t err_code;
	
//	if (gs_bdebug) {
//		for (int i=0;i< buffer_len;i++) {
//			app_trace_log("%c\r",buffer[i]);
//		}
//		app_trace_log("\r");
//	}
	
	err_code = ble_nus_string_send(&m_nus, buffer,buffer_len);
	if( err_code != NRF_SUCCESS ) {
		if( MAIN_DEBUG ) app_trace_log("NUS Send Err: 0x%02X\r", err_code); 	
		return false;
	}
	
	nus_busy = true;
	
	return true;
}

static uint32_t wakeup_src_set( uint32_t pin )
{

	uint32_t new_config = NRF_GPIO->PIN_CNF[ pin ];
	uint32_t sense = GPIO_PIN_CNF_SENSE_High;
	
	//Errata Work Around for Wake from Sleep
	{
		// Configure a GPIO as input, detecting low level.
		NRF_GPIO->PIN_CNF[pin] =	(GPIO_PIN_CNF_DIR_Output	<< GPIO_PIN_CNF_DIR_Pos)	| 
									(GPIO_PIN_CNF_INPUT_Connect	<< GPIO_PIN_CNF_INPUT_Pos)	| 
									(GPIO_PIN_CNF_PULL_Disabled	<< GPIO_PIN_CNF_PULL_Pos)	| 
									(GPIO_PIN_CNF_DRIVE_S0S1	<< GPIO_PIN_CNF_DRIVE_Pos)	| 
									(GPIO_PIN_CNF_SENSE_Low		<< GPIO_PIN_CNF_SENSE_Pos);
		
		// Ensure that the level is low.
		NRF_GPIO->OUTCLR = 1 << pin;
		
		// Unconfigure the used GPIO.
		NRF_GPIO->PIN_CNF[pin] = 	(GPIO_PIN_CNF_DIR_Input			<< GPIO_PIN_CNF_DIR_Pos)	| 
									(GPIO_PIN_CNF_INPUT_Disconnect	<< GPIO_PIN_CNF_INPUT_Pos)	| 
									(GPIO_PIN_CNF_PULL_Disabled		<< GPIO_PIN_CNF_PULL_Pos)	| 
									(GPIO_PIN_CNF_DRIVE_S0S1		<< GPIO_PIN_CNF_DRIVE_Pos)	| 
									(GPIO_PIN_CNF_SENSE_Disabled	<< GPIO_PIN_CNF_SENSE_Pos);
	}

	new_config |= (sense << GPIO_PIN_CNF_SENSE_Pos);
	NRF_GPIO->PIN_CNF[ pin ] = new_config;
    
    return NRF_SUCCESS;
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage( bool mode )
{
	uint32_t err_code;	
	
	if( mode == true ) {		
		err_code = sd_app_evt_wait();
		APP_ERROR_CHECK(err_code);
	}
	else {	
		// turn off advertising
		err_code = sd_ble_gap_adv_stop();
		APP_ERROR_CHECK(err_code);
		
		// turn off led
		led(0,0,0,0,-1);
		led_control();		//Force LED pins to go to states set in led()
		
		// change power mode
		sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
		
		enable_motion_wakeup();
		
		hal_twim_uninit();
		
		app_timer_stop_all();
		
		battery_stop();

		// Prepare wakeup interrupt.
		wakeup_src_set( IMU_INT );
		
		NRF_P0->OUTCLR = NRF_P0->OUTSET;
		
		NRF_TWIM0->PSEL.SCL |= 0x80000000;
		NRF_TWIM0->PSEL.SDA |= 0x80000000;
		
		NRF_PPI->CHENCLR = NRF_PPI->CHEN;
		
		nrf_gpio_cfg_default( TWIM_SCL_M );
		nrf_gpio_cfg_default( TWIM_SDA_M );
		//nrf_gpio_cfg_default( RX_PIN_NUMBER );
		//nrf_gpio_cfg_default( TX_PIN_NUMBER );
		nrf_gpio_cfg_default( RED_LED );
		nrf_gpio_cfg_default( GRN_LED );
		nrf_gpio_cfg_default( BLU_LED );
#ifdef ALT_BLU_LED
		nrf_gpio_cfg_default( ALT_BLU_LED );
#endif
		nrf_gpio_pin_clear(BAT_HALF_ON);
		nrf_gpio_pin_clear(LED_ON);
		
		NRF_MWU->REGIONENCLR = 0xFFFFFFFF;	//errata 75
		
		// Go to system-off mode (this function will not return; wakeup will cause a reset).
		*(uint32_t *)0x4007C074 = 2976579765; 	//as per errata 16
		err_code = sd_power_system_off();
		APP_ERROR_CHECK(err_code);

//		sd_app_evt_wait();
		while(1) {
			sd_power_system_off();
//			sd_app_evt_wait();
		}
	}
}

void motion_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//app_trace_log("int hit\r");
	imu_int_req = true;
}

// mpu9250 init
static bool config_imu(void)
{
	//struct eic_line_config imu_line_conf;	
	DEBUG_PRINT(0,"config_imu: start",MAIN_DEBUG); 	
	
	// init i2c interface
	if( hal_twim_init(IMU_DEBUG) != ERR_NONE )
	{
		DEBUG_PRINT(0,"imu_init: hal_init failed\r",MAIN_DEBUG);
		return FAIL;
	}

	if( imu_init(IMU_DEBUG) != ERROR_NONE ) {	
		//Keep trying. If a reset happened in the middle of a read, the accel may be 
		//stuck for up to 8 retries (gives 1 clock per retry).
		DEBUG_PRINT(0,"config_imu: init fail",MAIN_DEBUG); 	
		return FAIL;
	}
	
	DEBUG_PRINT(0,"config_imu: done",MAIN_DEBUG); 
	return PASS;
}

static void gpio_init(void)
{
    ret_code_t err_code;
	
	//set the output of all unused pins to 0
	uint32_t clear_outputs = (	1<<12 | 1<<13 | 1<<14 | 1<<15 | 1<<16 | 
								1<<18 | 
								1<<22 | 1<<23 | 1<<24 | 1<<25 | 1<<26 | 1<<27 | 1<<28 | 1<<29 | 1<<30 );
	nrf_gpio_pins_clear( clear_outputs );	
	nrf_gpio_range_cfg_output( 12, 16 );
	nrf_gpio_cfg_output( 18 );
	nrf_gpio_range_cfg_output( 22, 30 );
	nrf_gpio_cfg_default( 31 );		//make sure pin 31 input is disconnected

	//Setup Interrupt from IMU to indicate Motion Wake Up and Data Ready
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI( false );
    err_code = nrf_drv_gpiote_in_init(IMU_INT, &config, motion_int_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(IMU_INT, true);	
}

void indicate_battery_state(void)
{
	if ( gu_prev_battery_state != gu_battery_state ) {
		
		gu_prev_battery_state = gu_battery_state;
		app_timer_stop( m_LED_FLASH_timer_id );
		
		switch (gu_battery_state) {
			case BATTERY_CHARGING:
				battery_monitor_interval( BATTERY_CHARGING );
				led(1,0,1,10,10);
				led_start_flash_timer( m_LED_FLASH_timer_id );
				DEBUG_PRINT(0,"indicate_battery_state: BATTERY_CHARGING",MAIN_DEBUG);
				break;
			case BATTERY_CHARGED:
				battery_monitor_interval( BATTERY_CHARGED );			
				led(1,0,1,-1,0);
				led_start_flash_timer( m_LED_FLASH_timer_id );
				DEBUG_PRINT(0,"indicate_battery_state: BATTERY_CHARGED",MAIN_DEBUG);
				break;
			case BATTERY_DISCHARGING:
				battery_monitor_interval( BATTERY_DISCHARGING );				
				led_stop_flash_timer(m_LED_FLASH_timer_id);			
				led(0,0,0,0,-1);
				led_control();		//Force LED pins to go to states set in led()
				DEBUG_PRINT(0,"indicate_battery_state: BATTERY_DISCHARGING",MAIN_DEBUG);
				break;
			default:
				battery_monitor_interval( BATTERY_LOW );
				// Using absolute stop instead of led_stop_flash_timer here.		
				app_timer_stop( m_LED_FLASH_timer_id );			
				led(0,0,0,0,-1);
				//led_control();		//Force LED pins to go to states set in led()
				nrf_gpio_pin_clear(LED_ON);		//Sure fire way to turn LEDs Off
				DEBUG_PRINT(0,"indicate_battery_state: BATTERY_LOW",MAIN_DEBUG);
				break;
		}	
	}			
}
	
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t p_info)
{
	error_info_t * err_info = (error_info_t *) p_info;
	
	if(err_info->err_code != NRF_SUCCESS) {
		if( MAIN_DEBUG ) {
			app_trace_log("Err %02u: l-%01u\r", err_info->err_code, err_info->line_num);	//, err_info->p_file_name); 	
			BREAK_POINT;
		}
	}
	return;
}

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
	app_trace_log("XXX\r");	//try to print before death toll
	while(1);
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	uint32_t err_code;
	
	// Check 32Khz oscillator source
	if( nrf_clock_lf_src_get() != NRF_CLOCK_LF_SRC_XTAL  ) {		
		//Stop the LF Clk in order to change the clk source
		nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTOP);
		while (nrf_clock_lf_is_running());
		
		//Set source to desired input
		nrf_clock_lf_src_set( NRF_CLOCK_LFCLK_Xtal );
		
		//Restart the LF Clk to now run on the Crystal
		nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);
		while( !nrf_clock_lf_is_running() );
	}

	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);	
	
	// timers
	err_code = app_timer_create( &m_SYST_timer_id, APP_TIMER_MODE_REPEATED, ticks_timer_handler );
	APP_ERROR_CHECK(err_code);	

	err_code = app_timer_create( &m_BATTERY_MEASURE_timer_id, APP_TIMER_MODE_REPEATED, battery_measurement_timer_handler );
	APP_ERROR_CHECK(err_code);	
	
#ifndef USEPWM
	err_code = app_timer_create( &m_LED_FLASH_timer_id, APP_TIMER_MODE_REPEATED, led_flash_timer_handler );
	APP_ERROR_CHECK(err_code);
#endif

	//Timer needs to be running for WDT to be happy???
	err_code = app_timer_start(m_SYST_timer_id, SYSTEM_TIME_INTERVAL, NULL);	
	APP_ERROR_CHECK(err_code);
	nrf_delay_ms(5);
	
	//Configure WDT.
#if WDT_ENABLED
    err_code = nrf_drv_wdt_init( NULL, wdt_event_handler );
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_wdt_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();	
#endif
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
//        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}

static void null_cb_handler(pstorage_handle_t * handle, uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len)
{	
	if (gs_bdebug) app_trace_log("null_cb_handler: \r");
}

/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
#define USE_DM	true
static void device_manager_init(bool erase_bonds)
{
	if( NUM_DM_PAGES > 0 ) {
		uint32_t               err_code;
		dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
		dm_application_param_t register_param;

		// Initialize persistent storage module.
		err_code = pstorage_init();
		APP_ERROR_CHECK(err_code);

		if( USE_DM ) {
			err_code = dm_init(&init_param);
			APP_ERROR_CHECK(err_code);

			memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

			register_param.sec_param.bond         = SEC_PARAM_BOND;
			register_param.sec_param.mitm         = SEC_PARAM_MITM;
			register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
			register_param.sec_param.oob          = SEC_PARAM_OOB;
			register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
			register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
			register_param.evt_handler            = device_manager_evt_handler;
			register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

			err_code = dm_register(&m_app_handle, &register_param);
			APP_ERROR_CHECK(err_code);
		}
		else {
			// register pstorage Page to keep Log and Config in same locations
			static pstorage_handle_t  	null_handle;
			uint32_t res;
			pstorage_module_param_t param;
			param.block_size  = FLASH_PAGE_SIZE;
			param.block_count = NUM_DM_PAGES;
			param.cb          = null_cb_handler;
			res = pstorage_register(&param, &null_handle);
			if ( res == NRF_SUCCESS ) {
				if( gs_bdebug ) app_trace_log("p_store success\r");
			}
			else{
				if( gs_bdebug ) app_trace_log("p_store: Failed %01u\r", res);
			}
		}
	}
}

/**@brief Application main function.
 */
int main(void)
{		
	uint32_t err_code;
	uint32_t prv_unix_time = 0;
	TTASK_TIMER imu_check = { 0, DEFAULT_IMU_CHECK_PERIOD };
	uint32_t reset_reason = NRF_POWER->RESETREAS;
	bool erase_bonds = true; // need to figure out when to erase bonds
	
	NRF_POWER->RESETREAS = 0x0000F000F;

	app_trace_init();
	app_trace_log("\n\r%s\r", DEVICE_NAME);
	app_trace_log("FW ver %u.%u\r", FW_REV_MAJOR, FW_REV_MINOR );
	app_trace_log("Reset Cause: 0x%08X\r", reset_reason );
	nrf_delay_ms(10);
	
	// set power mode and pof
	sd_power_mode_set(NRF_POWER_MODE_LOWPWR);		//sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
	sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
	sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V21);
	sd_power_pof_enable(true);
	
	led_init( LED_DEBUG );
	
	// indicate cyan
	ledraw(0,1,1);
	
	// Init timers and startup Systick timer
	app_trace_s_msg("Initializing Timers & WDT\r");
	timers_init();
	
	// indicate red
	ledraw(1,0,0);
	
	// ble
	app_trace_s_msg("Initializing Soft Device\r");
	nrf_delay_ms(10);
	ble_stack_init();
	device_manager_init(erase_bonds);
	gap_params_init(); 
	advertising_init();	
	nrf_delay_ms(25);	//Stall for DM prints. There's alot.

	// indicate purple
	ledraw(1,0,1);
	
	// Print BLE address
	ble_gap_addr_t d_addr;
	err_code = sd_ble_gap_address_get(&d_addr);
	app_trace_log("MAC: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r", d_addr.addr[5], d_addr.addr[4], d_addr.addr[3], d_addr.addr[2], d_addr.addr[1], d_addr.addr[0]);
	
	// storage & config
	init_mem_manager();
	nrf_delay_ms(5);
	
	services_init();
	conn_params_init();
	nrf_delay_ms(5);

	// indicate blue
	ledraw(0,0,1);
	
	// imu
	config_imu();
	nrf_delay_ms(5);
	
	// indicate yellow
	ledraw(1,1,0);
	
	// init battery status
	app_trace_s_msg("Initializing battery status\r"); 			
	battery_init(m_BATTERY_MEASURE_timer_id, BAT_DEBUG);
	nrf_delay_ms(10);	
	// update ble
	battery_level_update_handler();
	
	// indicate green
	ledraw(0,1,0);
	
	// start advertisting 
	app_trace_s_msg("Start Advertising\r"); 
	nrf_delay_ms(5);
	err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);	
	APP_ERROR_CHECK(err_code);
	g_bpowered_down = false;
	
	// motion tracking	
	gpio_init();
	init_motion_analysis( MOTION_DEBUG, CurrentStepTotal );
	
	// Start Wake On Motion
	enable_motion_wakeup();
	nrf_delay_ms(10);
	
	// LED will be solid magenta when charged, magenta when Charging, Otherwise Off
	indicate_battery_state();
	
	app_trace_s_msg("Starting main loop\r"); 
	nrf_delay_ms(5);
	
	// Enter main loop.
	for (;;) {
		
		//app_trace_s_msg("."); 	
		nrf_drv_wdt_channel_feed( m_wdt_channel_id );
		power_manage( true );
		
		if ( gu_battery_level > 0 ) {
			
			if (g_bpowered_down == true) {
				DEBUG_PRINT(0,"Powering Up",MAIN_DEBUG); 
				
				// set power mode
				//sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
				sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
				
				// restart sytick
				err_code = app_timer_start(m_SYST_timer_id, SYSTEM_TIME_INTERVAL, NULL);	
				APP_ERROR_CHECK(err_code);
				
				// start advertisting 
				DEBUG_PRINT(0,"Start Advertising",MAIN_DEBUG); 
				err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
				APP_ERROR_CHECK(err_code);	

				//init imu
				enable_motion_wakeup();
				
				//update battery monitoring interval
				indicate_battery_state();
			}
			g_bpowered_down = false;
	
			uint32_t steps = getStepCnt();
			if (  steps != CurrentStepTotal ) {
				CurrentStepTotal = steps;
				//if( MAIN_DEBUG ) app_trace_log("%d Steps\r", CurrentStepTotal);
			}
			
			if( imu_int_req == true ) {			
				imu_int_req = false;
				
				//DEBUG_PRINT(0,"IMU Irq",MAIN_DEBUG); 
				imu_power_state();
				status_code_t res = collect_imu_data();
				if ( res == ERR_NONE ) {
					motionAnalyze();	//check for motion profiles
				}
				else {
					if( MAIN_DEBUG ) app_trace_log("IMU Read Fail: %01u\r", res);
				}
				
				//push out the check, only desired when the imu goes silent
				restart_timer( imu_check, 1000 );	//run check once IRQs stop for more than 1 second
			}
			else if( task_time( imu_check ) ) {
				//No int_requests for some time. Make sure the IMU is still operational
				restart_timer( imu_check, DEFAULT_IMU_CHECK_PERIOD );
				//DEBUG_PRINT(0,"Checking IMU",MAIN_DEBUG);
				if( imu_slp_check() == false ) {	
					//Module is non-responsive
					DEBUG_PRINT(0,"IMU failure: re-initializing",MAIN_DEBUG);
					hal_twim_uninit();
					hal_twim_init(IMU_DEBUG);
					if( imu_init(IMU_DEBUG) == ERR_NONE ) {
						enable_motion_wakeup();
					}
					else {
						//flag that imu is down!!!
					}
				}
				
				//power_manage( false );	//force deep sleep
			}
			
			// handle events that are scheduled to updated based on calendar time
			if( unix_time != prv_unix_time)
			{
				//events that are scheduled to coincidence with the RTC second update
				prv_unix_time = unix_time;
				
				// Start the LED light show if connected and LED_ID_KEY has been sent
<<<<<<< Updated upstream
				if(led_id_key_state == 1)
				{
=======
				switch ( led_id_key_state )
				{
					case 1:	// The key has been recieved. 
					
>>>>>>> Stashed changes
					app_trace_log("LEDID State 1\r");
					if(connected_and_bonded == 1) // this is a bonded connect so enable services and we are done
					{
						app_trace_log("LEDID State 1 bonded\r");
						led_id_key_state = 4;
						connected_and_bonded = 0;
					}
					else
					{
						led_light_show( true, m_LED_FLASH_timer_id ); // not bonded. start lightshow
						led_id_key_state = 2;
					}
<<<<<<< Updated upstream
				}

				if(led_id_key_state == 2)// time out routine run while waiting for LED_ID_NUMB
				{
=======
					break;
				
				case 2:	// time out routine run while waiting for LED_ID_NUMB
			
>>>>>>> Stashed changes
					if(led_id_key == 0)
					{
						if(led_id_key_to == 60)
						{
							led_id_key_to = 0;
							led_light_show( false, m_LED_FLASH_timer_id );
							// Disconnect from peer and start advertizing.
<<<<<<< Updated upstream
							err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
							APP_ERROR_CHECK(err_code);
							err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
							APP_ERROR_CHECK(err_code);
=======
							disconnect_and_advertize();
>>>>>>> Stashed changes
							led_id_key_state = 0;
						}
						led_id_key_to++;
					}
					break;
					
				case 3: // Finished.
			
					app_trace_log("LEDID State 3\r");
					led_light_show( false, m_LED_FLASH_timer_id );
					led_id_key = 0;
					led_id_key_to = 0;
					gu_prev_battery_state = 10; // A cheap way to reset the led state to it's former magenta glory
					led_id_key_state = 4;
<<<<<<< Updated upstream
				}

				if(led_id_key_state == 4) // Finished. now looking for disconnect.
				{
					app_trace_log("LEDID State 4\r");
					if(m_nus.conn_handle != BLE_CONN_HANDLE_INVALID)
						led_id_key_state = 0;
				}
=======
					break;
				
				case 4: // Finished. Enable services. Now looking for disconnect.
				
					app_trace_log("LEDID State 4\r");
					//if(m_nus.conn_handle != BLE_CONN_HANDLE_INVALID)
					//	led_id_key_state = 0;	
					break;
					
				default:
				break;
			}
>>>>>>> Stashed changes
				
				// check if the reporters need servicing
				check_reporters();
				
				// if a config update has been requested, execute
				if( config_update_requested() ) update_config();
								
				//events that are scheduled to coincidence with the RTC minute update
				if( prv_unix_time%60 == 0 ) {
					//Add a check for adverstising???
					
					//events that are scheduled to coincidence with the RTC hour update		
					if( prv_unix_time%3600 == 0) {
						
						//events that are scheduled to coincidence with the RTC day update
						if( prv_unix_time%(3600*24) == 0) {	
							//make sure we periodically back up the config... in case of sudden power loss
							update_config();
						}
					}					 
				}
			}	
			
			// if there is data queued to transmit, send it
			if( (m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (nus_busy == false) ) {
				send_ble_data();
			}
			
			if (battery_isdav()) {
				battery_level_update_handler();
				indicate_battery_state();	
			}	

			if ( g_update_and_reset ) {
				if( MAIN_DEBUG ) app_trace_log( "Reset to Bootloader\r" );
				sd_power_gpregret_set( BOOTLOADER_DFU_START );
				reset_prepare();
				NVIC_SystemReset();
			}
		}
		else {
			if ( !g_bpowered_down ) {
				g_bpowered_down = true;
				
				DEBUG_PRINT(0,"Powering Down",MAIN_DEBUG); 
			
				// not needed in production.
				//delay_ms(1000);
				
				// turn off imu
				stop_imu();
			
				// turn off advertising
				err_code = sd_ble_gap_adv_stop();
				APP_ERROR_CHECK(err_code);
				
				// save config record
				update_config();
				
				// slow battery monitoring
				gu_battery_state = BATTERY_LOW;
				indicate_battery_state();		
				//led(0,0,0,0,-1);	// turned off through indicate_battery_state()
				app_timer_stop( m_SYST_timer_id );
			
				// turn off system and wait for an analog compare.	
				//sd_power_system_off ();
				// change power mode
				sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
			}
				
			// update battery reading which s/b going on off of a timer
			if (battery_isdav()) {				
				battery_status_level(&gu_battery_level,&gu_battery_state);
				indicate_battery_state();
			}
								
		}

	}//end forever loop
}//end main


/** 
 * @}
 */
