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
 
// 							====> Debug vars notes <=====
//
//  						LED_ID State 4 is off
//							bool erase_bonds = TRUE 
//							D&A disabled for iOS testing
//							Key State Machine is enabled
// 							====> Debug vars notes <=====

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
#include "comms.h"
#include "pstorage.h"
#include "mem_manager.h"
#include "reports.h"
#include "led.h"
#include "battery.h"
#include "nrf_drv_wdt.h"
#include "bootloader_types_vsp.h"
#include "nrf_drv_spi.h"
#include "manufacture_test.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "si115x_functions.h"

#define ERASE_BONDS false

#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1											/**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT      		0                               			/**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   		1                               			/**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Level"										/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "TheShop"                      				/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "678"										/**< Model number. Will be passed to Device Information Service. */
#define FW_REV_MAJOR					3
#define FW_REV_MINOR					45											/**< errata 20 implemented as possible fix for occassional sys_timer startup problems*/						 
#define MANUFACTURER_ID                 0x1122334455                              	/**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x667788 

#define MAGIC_DFU_ENABLE_NUM			234

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL_FAST           64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS_FAST	180                                         /**< The advertising timeout (in units of seconds). */

#define APP_ADV_INTERVAL_SLOW           800                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 500 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS_SLOW 180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)           	/**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)          	/**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                 	4                                         	/**< Slave latency. Skip upto X number of connection interval responses. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)           	/**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define USE_DM							true
#define SEC_PARAM_BOND					1											/**< Perform bonding. */
#define SEC_PARAM_MITM					0											/**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC					1											/**< Enable LE Secure Connection pairing. */
#define SEC_PARAM_IO_CAPABILITIES		BLE_GAP_IO_CAPS_NONE						/**< No I/O capabilities. */
#define SEC_PARAM_OOB					0											/**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE			7											/**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE			16											/**< Maximum encryption key size. */
//#define SEC_PARAM_TIMEOUT             30											/**< Maximum reponse time for SEC params. */

#define DEAD_BEEF                       0xDEADBEEF									/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

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
static ble_uuid_t                       m_adv_uuids[] =
{
	{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE},
	{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
	{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
}; /**< Universally unique service identifiers. */

extern T_CONFIG							g_config;
static bool								m_memory_access_in_progress = false;		/**< Flag to keep track of ongoing operations on persistent memory. */
static dm_application_instance_t		m_app_handle;								/**< Application identifier allocated by device manager */
uint8_t 								device_index_ring;							/**< Ring 0 through 6 assign p device index to device_index_ring manually. */
uint8_t 								key_ring[7];								/**< Key Ring 0 through 6. persistant to remember which bonds are unlocked */
uint8_t 								key_ring_index;								/**< Index for Key Ring */
#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t						m_dfus;										/**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT

//
// globals
//
bool 									erase_records_flag = false;
volatile uint32_t						gs_timecounter = 0;
volatile bool							gb_wakeup;
#define	 ONESEC_MS						1000
uint8_t									gu_battery_state = BATTERY_UNDEF_STATE;
uint8_t									gu_prev_battery_state = BATTERY_UNDEF_STATE;
uint8_t									gu_battery_level = 50;
static bool 							g_update_and_reset = false;
static bool								g_advertise_state = OFF;
uint32_t 								fw_rev = ((FW_REV_MAJOR<<8)|FW_REV_MINOR);
struct
{
	char hwid[14];
	char fw[10];
	char sw[20];
	char hw_rev[4];
} dis_str;

// some HRM test shit globals
uint16_t accel_x;
uint16_t accel_y;
uint16_t accel_z;
uint8_t uart_rev_buffer[1] = {0};
uint8_t uart_rx_ubyte;
T_PKT_TYPES tx_type;			//T_PKT_TYPES type;
uint8_t tx_payload[ MAX_PKT_PAYLOAD ];
int8_t tx_pay_len = -1;	//default no response
uint8_t hrm_step = 0;
//
// main locals
//
nrf_drv_wdt_channel_id m_wdt_channel_id;

//
// debugging defines
//
#define IMU_DEBUG				true
#define MAIN_DEBUG				true
#define BAT_DEBUG				true
#define MOTION_DEBUG			true
#define LED_DEBUG				false
#define MEM_DEBUG				true
bool gs_bdebug = true;

//
// timers
//
#define APP_TIMER_OP_QUEUE_SIZE         10		/**< Size of timer operation queues. */
#define SYS_TICK_PERIOD					125
#define SYS_TIME_PERIOD					(1*SYS_TICK_PERIOD)
#define BAT_DRAIN_TIME_PERIOD			25
#define	SYSTEM_TIME_TICKS	 			( APP_TIMER_TICKS(SYS_TIME_PERIOD, APP_TIMER_PRESCALER) )
#define	BAT_DRAIN_TIME_TICKS	 		( APP_TIMER_TICKS(BAT_DRAIN_TIME_PERIOD, APP_TIMER_PRESCALER) )
static volatile uint32_t systick_ms 	= 0;
static uint32_t systick_period_ms 		= SYS_TIME_PERIOD;
static uint32_t systick_period_ticks 	= SYSTEM_TIME_TICKS;
static uint32_t unix_time 				= 0;

APP_TIMER_DEF(m_SYST_timer_id);
APP_TIMER_DEF(m_LED_FLASH_timer_id);
APP_TIMER_DEF(m_DETACH_timer_id);

// Default Main Loop task times:
#define SHIP_MODE_DELAY					(5*60000UL)	//Shutoff after 5 minutes off charger
#define DEFAULT_IMU_CHECK_PERIOD		(60000UL)	//time between imu checks

//
// imu/mag interrupt
//
static bool imu_int_req = false;


//function prototypes
uint32_t ble_uart_tx(uint8_t *buffer, uint8_t buffer_len);
void delete_current_bond(void);

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
	char buf[50];
	uint8_t len;
	
	// Print BLE address
	ble_gap_addr_t d_addr;
	sd_ble_gap_address_get(&d_addr);
	
	if( manufacture_mode() )
	{
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);
	}

	len = sprintf( buf, "%s %02X%02X", DEVICE_NAME, d_addr.addr[1], d_addr.addr[0] );
	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) buf, len);
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency     = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	// Set the Preferred Peripheral Connection Parameters
	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);

	// cut tx power
	sd_ble_gap_tx_power_set(0); //-8
}

//#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
	uint32_t err_code;

	app_trace_puts(DEBUG_MED, "Advertising Stopped!\r");
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
	//uint32_t err_code;

	if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		// Disconnect from peer.
		//err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		//APP_ERROR_CHECK(err_code);
// WMMDB Bootloader
	}
	else
	{
		// If not connected, the device will be advertising. Hence stop the advertising.
		advertising_stop();
	}

	nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
//#endif // BLE_DFU_APP_SUPPORT

void terminate_connection(void)
{
	uint32_t      err_code;

	app_trace_puts(DEBUG_MED, "\rConnection Terminated!\r\r");
	err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	APP_ERROR_CHECK(err_code);
}

void delete_all_bonds(void)
{
	uint8_t i;
	
	dm_device_delete_all(&m_app_handle);
	g_config.device_index_ring = device_index_ring = 0;
	key_ring_index = 0;
	for(i=0; i<7; i++)
	{
		g_config.key_ring[i] = key_ring[i] = 0;
	}
	set_config_update_flag();
	app_trace_puts(DEBUG_MED, "dm_device_delete_all\r");
}

void delete_current_bond(void)
{
	m_nus.conn_handle = BLE_CONN_HANDLE_INVALID;
	//terminate_connection();
	gu_prev_battery_state = BATTERY_UNDEF_STATE; // Reset the led state to it's former magenta glory
	if(device_index_ring != 0)
		device_index_ring-- ;
	else
		device_index_ring = 6;
	key_ring[key_ring_index] = 0;
	g_config.key_ring[key_ring_index] = key_ring[key_ring_index];
	g_config.device_index_ring = device_index_ring;
	app_trace_log(DEBUG_MED, "[KR] delete_current_bond: key_ring[key_ring_index] %d  key_ring_index %d\r", key_ring[key_ring_index], key_ring_index);
	set_config_update_flag();
}

void set_key_ring(uint8_t key)
{
	key_ring[key_ring_index] = key;
	g_config.key_ring[key_ring_index] = key_ring[key_ring_index];
	g_config.device_index_ring = device_index_ring;
	app_trace_log(DEBUG_LOW, "[KR] ==>set_key_ring(int) key_ring[key_ring_index] %d  key_ring_index %d key %d\r", key_ring[key_ring_index], key_ring_index, key);
	set_config_update_flag();
}


/**@brief Function for handling the data from the hijacked Battery Service characteristic
 *
 * @details This function will start reset to the Bootlaoder when the Magic number is sent
 *
 * @param[in] p_bas    Battery Service structure.
 * @param[in] p_data   Data
 * @param[in] length   Length of the data.
 */
void dfu_control_data_handler(ble_nus_t * p_bas, uint8_t * p_data, uint16_t length)
{
	volatile uint8_t magic;
	volatile uint32_t retval = NRF_SUCCESS;
	magic = *p_data;
	app_trace_puts(DEBUG_LOW, "[DFU_CNTRL]\r");
	
	if( secure_connection() )
	{
		if ( (length == 1) && ( magic == MAGIC_DFU_ENABLE_NUM ) )
		{
			if (gs_bdebug) app_trace_puts(DEBUG_LOW, "Enable DFU\r");
			//g_config.advertise_dfu = MAGIC_DFU_ENABLE_NUM;
			g_update_and_reset = true;
		}
	}
	else
	{	//Not allowed. Good bye!
		app_trace_puts(DEBUG_MED, "[DFU_CNTRL] Access Denied!!!\r");
		terminate_connection();
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

	//if (gs_bdebug) app_trace_puts(DEBUG_LOW, "UART data received\r");

	if( length < 2 || length > sizeof(T_PACKET) )
	{
		app_trace_puts(DEBUG_MED, "[NUS] length error\r");

		rx_msg.id = 0;
		rx_msg.type = (T_PKT_TYPES) 0;
		rx_msg.pkt_len = 0;
		rx_msg.payload[0] = 0;
	}
	else
	{
		// form message packet
		rx_msg.id = p_data[0];
		rx_msg.type = (T_PKT_TYPES) p_data[1];
		rx_msg.pkt_len = length;
		for (int i=2; i<rx_msg.pkt_len; i++)
		{
			rx_msg.payload[i-2] = p_data[i];
			//if (gs_bdebug) app_trace_log(DEBUG_LOW, "%02X ", p_data[i]);
		}
	}

	if (gs_bdebug) app_trace_log(DEBUG_MED, "[NUS] PK:0x%02X TY:0x%02X LN:%u @%01u\r",rx_msg.id ,rx_msg.type, rx_msg.pkt_len, systick_ms);

	if( m_nus.conn_handle != BLE_CONN_HANDLE_INVALID ) {
		parse_msg(&rx_msg);
	}
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for handling the Battery Service updates.
 *
 * @details This function will be called each time new battery data is collected.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void ble_bas_update(void * p_context)
static void ble_bas_update( void )
{
	uint32_t err_code;
	
	// update ble
	err_code = ble_bas_battery_level_update(&m_bas, gu_battery_level);
	if(	(err_code != NRF_SUCCESS) &&
		(err_code != NRF_ERROR_INVALID_STATE) &&
		(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) )
	{
		//error 0x3004 = BLE_ERROR_NO_TX_PACKETS:
		if (gs_bdebug) app_trace_log(DEBUG_MED, "[BAS_UPDATE]: failed %01u\r", err_code);
		APP_ERROR_HANDLER(err_code);
	}
	
	err_code = ble_bas_battery_state_update(&m_bas, gu_battery_state);
	if(	(err_code != NRF_SUCCESS) &&
		(err_code != NRF_ERROR_INVALID_STATE) &&
		(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) )
	{
		if (gs_bdebug) app_trace_log(DEBUG_MED, "[BAS_UPDATE]: failed %01u\r", err_code);
		APP_ERROR_HANDLER(err_code);
	}
}

//Function for control LED to to indicate Charging/Charged
void indicate_battery_state(void)
{
	if ( gu_prev_battery_state != gu_battery_state )
	{

		gu_prev_battery_state = gu_battery_state;
		//app_timer_stop( m_LED_FLASH_timer_id );

		switch (gu_battery_state)
		{
			case BATTERY_CHARGING:
				//led(1,0,1,10,10);
				app_trace_puts(DEBUG_MED, "[LED_IND]: CHARGING\r");
				break;
			
			case BATTERY_CHARGED:
				//led(1,0,1,-1,0);
				app_trace_puts(DEBUG_MED, "[LED_IND]: CHARGED\r");
				break;
			
			case BATTERY_MONITOR_FAIL:
				//If the volatage reading is too low, there must be a problem with the voltage 
				//measurement circuit. In this case, continue operation as normal so a device
				//will be capable of connecting and learning of this issue.
				//led(0,0,0,0,-1);
				app_trace_puts(DEBUG_HIGH, "[LED_IND]: BAT MON FAIL\r");
				break;
			
			case BATTERY_DISCHARGING:
				//led(0,0,0,0,-1);
				app_trace_puts(DEBUG_MED, "[LED_IND]: DISCHARGING\r");
				break;
			
			default:
				// Using absolute stop instead of led_stop_flash_timer here.
				//led(0,0,0,0,-1);
				nrf_gpio_pin_clear(LED_ON);		//Sure fire way to turn LEDs Off
				app_trace_puts(DEBUG_MED, "[LED_IND]: LOW\r");
				break;
		}
	}
}

void HardFault_Handler(void)
{
	uint32_t *sp = (uint32_t *) __get_MSP(); // Get stack pointer
	uint32_t ia = sp[24/4]; // Get instruction address from stack

	app_trace_log(DEBUG_HIGH, "Hard Fault at address: 0x%08x\r", (unsigned int)ia);
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

	// Initialize Device Information Service.
	memset(&dis_init, 0, sizeof(dis_init));

	ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);
	ble_srv_ascii_to_utf8(&dis_init.serial_num_str, dis_str.hwid);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, dis_str.hw_rev);
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, dis_str.fw);
	ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, dis_str.sw);
	
	sys_id.manufacturer_id            = MANUFACTURER_ID;
	sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
	dis_init.p_sys_id                 = &sys_id;

	// Here the sec level for the Device Info Service can be changed/increased.
	if( manufacture_mode() )
	{
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.write_perm);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init.dis_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
	}

	err_code = ble_dis_init(&dis_init);
	if ((gs_bdebug) && (err_code != NRF_SUCCESS) ) app_trace_log(DEBUG_MED, "services_init: dis failed %i\r",err_code);
	APP_ERROR_CHECK(err_code);

	// Initialize Battery Service.
	memset(&bas_init, 0, sizeof(bas_init));

	// Here the sec level for the Battery Service can be changed/increased.
	if( manufacture_mode() )
	{
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.write_perm);

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_state_char_attr_md.cccd_write_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_state_char_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_state_char_attr_md.write_perm);

		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);
	}
	else
	{
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init.battery_level_char_attr_md.cccd_write_perm);
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init.battery_level_char_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init.battery_level_char_attr_md.write_perm);

		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init.battery_state_char_attr_md.cccd_write_perm);
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init.battery_state_char_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_state_char_attr_md.write_perm);

		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init.battery_level_report_read_perm);
	}

	bas_init.evt_handler          = NULL;
	bas_init.p_report_ref         = NULL;
	bas_init.initial_batt_level   = 255;
	bas_init.initial_batt_state   = 255;
	bas_init.dfu_control_data_handler = (ble_bas_data_handler_t) dfu_control_data_handler;

	err_code = ble_bas_init(&m_bas, &bas_init);
	if ((gs_bdebug) && (err_code != NRF_SUCCESS) ) app_trace_log(DEBUG_MED, "services_init: bas failed %i\r",err_code);
	APP_ERROR_CHECK(err_code);

	if (gs_bdebug) app_trace_puts(DEBUG_LOW, "Services include NUS\r");
	
	// Init Nordic UART Service
	memset(&nus_init, 0, sizeof(nus_init));
	
	// Security level for the Nordic UART Service is set to OPEN in ble_nus.c

	nus_init.data_handler = nus_data_handler;
	err_code = ble_nus_init(&m_nus, &nus_init);
	if ((gs_bdebug) && (err_code != NRF_SUCCESS) ) app_trace_log(DEBUG_MED, "services_init: nus failed %i\r",err_code);
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
		app_trace_puts(DEBUG_MED, "On Conn Param Evt: Negotiation Failed\r");
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
	T_DEBUG_PRIORITY db_pri = DEBUG_LOW;

	switch (ble_adv_evt)
	{
		case BLE_ADV_EVT_FAST:
			app_trace_puts(db_pri, "[ON_ADV_EVT] Fast Adv\r");
			g_advertise_state = ON;
			break;

		case BLE_ADV_EVT_SLOW:
			app_trace_puts(db_pri, "[ON_ADV_EVT] Slow Adv\r");
			g_advertise_state = ON;
			break;

		case BLE_ADV_EVT_IDLE:
			//sleep_mode_enter();
			app_trace_puts(db_pri, "[ON_ADV_EVT] Adv Idle\r");
			g_advertise_state = OFF;
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
			app_trace_puts(DEBUG_MED, "[ON_BLE_EVT] Connected\r");
			m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			g_advertise_state = OFF;
			flag_new_comms();
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			app_trace_puts(DEBUG_MED, "[ON_BLE_EVT] Disconnected\r");
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
			g_advertise_state = OFF;
			app_timer_stop(m_DETACH_timer_id);	//detach is unnecessary, make sure it is OFF
			gu_prev_battery_state = BATTERY_UNDEF_STATE; // Reset the led state to it's former magenta glory
			break;

		case BLE_GAP_EVT_TIMEOUT:
			app_trace_puts(DEBUG_LOW, "[ON_BLE_EVT] Adv Idle\r");
			g_advertise_state = OFF;
			break;
		
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			// This is handled by the DM in DMP.c
			if( !USE_DM )
			{
				app_trace_puts(DEBUG_MED, "[ON_BLE_EVT] Params Req in on_ble_evt\r");
				err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
				APP_ERROR_CHECK(err_code);
			}
			break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
			// No system attributes have been stored.
			app_trace_puts(DEBUG_MED, "[ON_BLE_EVT] Atts Missing\r");
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
	if(gs_bdebug) app_trace_log(DEBUG_LOW, "[BLE_EVT_DISPATCH] 0x%02X\r", p_ble_evt->header.evt_id);

	if( USE_DM ) dm_ble_evt_handler(p_ble_evt);
	ble_nus_on_ble_evt(&m_nus, p_ble_evt);
	ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	ble_conn_params_on_ble_evt(p_ble_evt);

#ifdef BLE_DFU_APP_SUPPORT
	/** @snippet [Propagating BLE Stack events to DFU Service] */
	ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
	/** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT

//	if( p_ble_evt->header.evt_id == BLE_EVT_TX_COMPLETE ) 
//	{
//		app_trace_log(DEBUG_MED, "NUS WR %01u: @%01u\r", p_ble_evt->evt.common_evt.params.tx_complete.count, getSystemTimeMs());
//	}
//	else if ( p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED ) {
//		app_trace_log(DEBUG_MED, "REQ CONN INT: %02u\r", p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval ); 
//	}
	
	on_ble_evt(p_ble_evt);
	//ble_advertising_on_ble_evt(p_ble_evt);	//advertising controlled from Main loop
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
	//These are all related to writing memory
	app_trace_log(DEBUG_LOW, "[SYS_EVT_DISPATCH] %x: @%01u\r", sys_evt, systick_ms);
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
	ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
#endif // BLE_DFU_APP_SUPPORT

	//Check the ram settings against the used number of links
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

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
	second_counter += systick_period_ticks*(APP_TIMER_PRESCALER+1);
	if ( second_counter >= APP_TIMER_CLOCK_FREQ )
	{
		//second_counter = 0;					//throws away any remainder that could be there, making time inaccurate
		second_counter -= APP_TIMER_CLOCK_FREQ;	//keeps remainder that could be there, more accurate time tracking
		unix_time++;
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
	//advdata.include_appearance      = true;
	advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.uuids_complete.uuid_cnt = 0;
	//advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
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

uint32_t ble_uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
	ret_code_t err_code = NRF_ERROR_BUSY;

//	if (gs_bdebug) {
//		for (int i=0;i< buffer_len;i++) {
//			app_trace_log(DEBUG_LOW, "%c\r",buffer[i]);
//		}
//		app_trace_puts(DEBUG_LOW, "\r");
//	}

	err_code = ble_nus_string_send(&m_nus, buffer,buffer_len);
	if( err_code == NRF_SUCCESS)
	{
		//app_trace_log(DEBUG_LOW, "NUS Sent\r");
	}
	else if( err_code == BLE_ERROR_NO_TX_PACKETS ) 
	{
		//No available packets to send on
	}
	else 
	{
		app_trace_log(DEBUG_MED, "NUS Send Err: 0x%02X\r", err_code);
	}

	return err_code;
}

static uint32_t wakeup_src_set( uint32_t pin )
{

	uint32_t new_config = NRF_GPIO->PIN_CNF[ pin ];
	uint32_t sense = GPIO_PIN_CNF_SENSE_Low;	//GPIO_PIN_CNF_SENSE_High;

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
static void system_powerdown( void )
{
	uint32_t err_code;

	// turn off advertising
	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);

	// turn off led
	led_uninit();

	//Cut power that memory device may be using
	uninit_mem_manager();
	
	stop_imu();
	//enable_motion_wakeup();

	hal_twim_uninit();

	app_timer_stop_all();

	battery_monitor_shutdown();

	// Prepare wakeup interrupt.
	wakeup_src_set( CHARGE_PG_INPUT );
	//wakeup_src_set( IMU_INT );

	nrf_gpio_pin_clear( BAT_HALF_EN );
	nrf_gpio_pin_clear( LED_ON );
	nrf_gpio_pin_clear( CD_OUT_PIN );

	NRF_MWU->REGIONENCLR = 0xFFFFFFFF;	//errata 75

	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	*(uint32_t *)0x4007C074 = 2976579765; 	//as per errata 16
	err_code = sd_power_system_off();
	APP_ERROR_CHECK(err_code);

	while(1)
	{	//If we didn't shutoff, there is a problem...
		sd_power_system_off();
	}
}

void motion_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//app_trace_puts(DEBUG_LOW, "int hit\r");
	imu_int_req = true;
}

// mpu9250 init
static ret_code_t config_imu(void)
{
	ret_code_t err;
	
	//struct eic_line_config imu_line_conf;
	app_trace_puts(DEBUG_LOW, "[INIT_IMU] start\r");

	// init i2c interface
	err = hal_twim_init( false );
	if( err == NRF_SUCCESS)
	{
		//Initialize IMU device
		err = imu_init(IMU_DEBUG);
		if( err == NRF_SUCCESS )
		{
			//Setup Interrupt from IMU to indicate Motion Wake Up and Data Ready
			if( nrf_drv_gpiote_is_init() == false )
			{	//gpiote needs to be initialized
				err = nrf_drv_gpiote_init();
				if( err != NRF_SUCCESS )
				{
					app_trace_log(DEBUG_HIGH, "[INIT_IMU] gpiote err: %01d\r", err);
				}
			}
			nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI( false );
			err = nrf_drv_gpiote_in_init(IMU_INT, &config, motion_int_handler);
			if( err == NRF_SUCCESS )
			{
				nrf_drv_gpiote_in_event_enable(IMU_INT, true);
				app_trace_puts(DEBUG_LOW, "[INIT_IMU] done\r");
			}
			else
			{
				app_trace_log(DEBUG_HIGH, "[INIT_IMU] imu_int pin err: %01d\r", err);
			}
		}
		else
		{
			//Keep trying. If a reset happened in the middle of a read, the accel may be
			//stuck for up to 8 retries (gives 1 clock per retry).
			app_trace_log(DEBUG_HIGH, "[INIT_IMU] imu init err: %01d\r", err);
		}
	}
	else
	{
		app_trace_log(DEBUG_HIGH, "[INIT_IMU] i2c init err: %01d\r", err);
	}
	
	init_motion_analysis( MOTION_DEBUG, 0 );	// set motion tracking variables and pointers
	
	return err;
}

void service_imu( void )
{
	static TTASK_TIMER imu_check = { 0, DEFAULT_IMU_CHECK_PERIOD };
	
	//If the Interrupt has triggered (or the pin is still active) read the IMU
	if( (imu_int_req == true) || (nrf_gpio_pin_read(IMU_INT) == 1) )
	{
		imu_int_req = false;

		//app_trace_puts(DEBUG_MED, ".");
		imu_power_state();
		ret_code_t res = collect_imu_data();
		if ( res == NRF_SUCCESS )
		{
			motionAnalyze();	//check for motion profiles
			
			//push out the check, only desired when the IMU goes silent
			start_task_timer( imu_check, 1000 );	//run check once IRQs stop for more than 1 second
		}
		else
		{
			if( MAIN_DEBUG ) app_trace_log(DEBUG_LOW, "IMU Read Fail: %01u\r", res);
		}
	}
	
	if( task_time( imu_check ) )
	{
		//No int_requests for some time. Make sure the IMU is still operational
		start_task_timer( imu_check, DEFAULT_IMU_CHECK_PERIOD );
		//app_trace_puts(DEBUG_LOW, "Checking IMU\r");
		
		if( imu_slp_check() == false )
		{
			//Module is non-responsive
			app_trace_puts(DEBUG_MED, "IMU failure: re-initializing\r");
			hal_twim_uninit();
			hal_twim_init(IMU_DEBUG);
			if( imu_init(IMU_DEBUG) == NRF_SUCCESS )
			{
				enable_motion_wakeup();
			}
			else
			{
				//imu is down!!!
			}
		}

		//power_manage( false );	//force deep sleep
	}
}

void service_calendar_events( void )
{
	static uint32_t prv_unix_time = 0;
	
	if( unix_time != prv_unix_time)
	{
		//events that are scheduled to coincidence with the RTC second update
		prv_unix_time = unix_time;
		
		// check if the reporters need servicing
		check_reporters();
		
		//events that are scheduled to coincidence with the RTC minute update
		if( prv_unix_time%60 == 0 )
		{
			//Add a check for adverstising???

			//events that are scheduled to coincidence with the RTC hour update
			if( prv_unix_time%3600 == 0)
			{

				//events that are scheduled to coincidence with the RTC day update
				if( prv_unix_time%(3600*24) == 0)
				{
					//make sure we periodically back up the config... in case of sudden power loss
					app_trace_puts(DEBUG_MED, "[UT] New Day, Save Cfg\r");
					set_config_update_flag();
				}
			}
		}
	}
}

static void gpio_defaults( void )
{
	uint32_t clear_outputs;
	
	nrf_gpio_cfg_input( AXIS_INT1, NRF_GPIO_PIN_NOPULL );
	nrf_gpio_cfg_input( AXIS_INT2, NRF_GPIO_PIN_NOPULL );
	
	#if defined( LEVEL_1_1 )
		// Make sure SPI flash pins go put flash device into idle mode
		nrf_gpio_pin_set( SPI_CS_PIN );
		nrf_gpio_pin_clear( SPI_HOLD_PIN );
		nrf_gpio_pin_clear( BAT_HALF_EN );
		nrf_gpio_cfg_output( SPI_CS_PIN );
		nrf_gpio_cfg_output( SPI_HOLD_PIN );
		nrf_gpio_cfg_output( BAT_HALF_EN );
	
		nrf_gpio_cfg_input( CHARGE_PG_INPUT, NRF_GPIO_PIN_NOPULL );
		nrf_gpio_cfg_input( BQ_INT_PIN, NRF_GPIO_PIN_PULLUP );
	
		//set the output of all unused pins to 0 (lowest current state)
		clear_outputs = (	1<<11 |
							1<<17 | 1<<18 | 1<<19 |
							1<<22 | 1<<23 | 1<<24 | 1<<25 | 1<<26 | 1<<27 | 1<<28 | 1<<29 | 1<<30 );
		nrf_gpio_pins_clear( clear_outputs );
		nrf_gpio_cfg_output( 11 );
		nrf_gpio_range_cfg_output( 17, 19 );
		nrf_gpio_range_cfg_output( 22, 30 );
		nrf_gpio_cfg_default( 31 );		//make sure pin 31 input is disconnected
	#else
	
		//set the output of all unused pins to 0 (lowest current state)
		clear_outputs = (	1<<12 | 1<<13 | 1<<14 | 1<<15 | 1<<16 |
							1<<18 |
							1<<22 | 1<<23 | 1<<24 | 1<<25 | 1<<26 | 1<<27 | 1<<28 | 1<<29 | 1<<30 );
		nrf_gpio_pins_clear( clear_outputs );
		nrf_gpio_range_cfg_output( 12, 16 );
		nrf_gpio_cfg_output( 18 );
		nrf_gpio_range_cfg_output( 22, 30 );
		nrf_gpio_cfg_default( 31 );		//make sure pin 31 input is disconnected
	#endif
	
	led_init( LED_DEBUG, m_LED_FLASH_timer_id );
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t p_info)
{
	error_info_t * err_info = (error_info_t *) p_info;

	if(err_info->err_code != NRF_SUCCESS)
	{
		if( MAIN_DEBUG )
		{
			app_trace_log(DEBUG_HIGH, "Err 0x%02X: l-%01u\r", err_info->err_code, err_info->line_num);	//, err_info->p_file_name);
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
	app_trace_puts(DEBUG_HIGH, "XXX\r");	//try to print before death toll
		
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
	T_DEBUG_PRIORITY pri = DEBUG_MED;

	app_trace_puts(pri, "[TIMERS_INIT] ");
	// Check 32Khz oscillator source
	if( (nrf_clock_lf_src_get() != NRF_CLOCK_LF_SRC_XTAL) || (nrf_clock_lf_actv_src_get() != NRF_CLOCK_LF_SRC_XTAL) )
	{
		app_trace_puts(pri, ": Switch LFCLK to XTAL ");

		//Stop the LF Clk in order to change the clk source
		nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTOP);
		while (nrf_clock_lf_is_running());

		//Set source to desired input
		nrf_clock_lf_src_set( NRF_CLOCK_LFCLK_Xtal );
	}
	app_trace_puts(pri, "\r");
	
	// Errata 20:
	{	//Execute before using RTC		
		NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;
		NRF_CLOCK->TASKS_LFCLKSTART     = 1;
		while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
		NRF_RTC0->TASKS_STOP = 0;
		NRF_RTC1->TASKS_STOP = 0;
		NRF_RTC2->TASKS_STOP = 0;
	}
	
	//Make sure RTC is OFF!!!
	NRF_RTC0->TASKS_STOP = 1;
	NRF_RTC1->TASKS_STOP = 1;
	NRF_RTC2->TASKS_STOP = 1;
	nrf_delay_us(100);
	
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

	// timers
	err_code = app_timer_create( &m_SYST_timer_id, APP_TIMER_MODE_REPEATED, ticks_timer_handler );
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create( &m_DETACH_timer_id, APP_TIMER_MODE_SINGLE_SHOT, detach_handler );
	APP_ERROR_CHECK(err_code);
	
#ifndef USEPWM
	err_code = app_timer_create( &m_LED_FLASH_timer_id, APP_TIMER_MODE_REPEATED, led_flash_timer_handler );
	APP_ERROR_CHECK(err_code);
#endif

	//Timer needs to be running for WDT to be happy???
	systick_ms = 0;
	systick_period_ms = SYS_TIME_PERIOD;
	systick_period_ticks = SYSTEM_TIME_TICKS;
	err_code = app_timer_start(m_SYST_timer_id, SYSTEM_TIME_TICKS, NULL);
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
	
	if( event_result != NRF_SUCCESS ) {
		if (gs_bdebug) app_trace_log(DEBUG_LOW, "dm_cb_handler: Evt_ID: 0x02X, Res: 0x02X\r", p_event->event_id, event_result);
	}

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
	if (gs_bdebug) app_trace_puts(DEBUG_LOW, "null_cb_handler: \r");
}


void form_version_strings( void )
{
	//Get and Check HW version String:
	memcpy( (void *)&dis_str.hw_rev, (void *)&g_config.hw_rev, sizeof(g_config.hw_rev) );
	if( dis_str.hw_rev[0] < 'A' || dis_str.hw_rev[0] > 'Z' )
	{	//Out of bounds!!!
		strcpy( dis_str.hw_rev, "XXX" );
	}
	
	ble_version_t sd_rev;
	bootloader_settings_t boot_settings;
	memcpy( (uint8_t *)&boot_settings, (uint8_t *)(BOOTLOADER_SETTINGS_ADDRESS), sizeof(bootloader_settings_t) );
	sd_ble_version_get( &sd_rev );

	//Form BLE address String and Print:
	ble_gap_addr_t d_addr;
	sd_ble_gap_address_get(&d_addr);
	sprintf( dis_str.hwid, "%02X%02X%02X%02X%02X%02X", (uint8_t)(d_addr.addr[5]), (uint8_t)(d_addr.addr[4]), (uint8_t)(d_addr.addr[3]), (uint8_t)(d_addr.addr[2]), (uint8_t)(d_addr.addr[1]), (uint8_t)(d_addr.addr[0]) );		//obfuscate partial MAC
	app_trace_log(DEBUG_MED, "[MAC]: %s%s\r", FG_RED, dis_str.hwid );
	
	//Form Version Strings and Print:
	sprintf( dis_str.fw, "%01u.%02u", FW_REV_MAJOR, FW_REV_MINOR );
	sprintf( dis_str.sw, "SD-%03u BL-%01u.%02u", sd_rev.subversion_number, (boot_settings.bl_rev>>8)&0xFF, boot_settings.bl_rev&0xFF );
	app_trace_log(DEBUG_HIGH, "HW-%s %s A-%s%s\r", dis_str.hw_rev, dis_str.sw, dis_str.fw, FG_RESET );
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
	if( NUM_DM_PAGES > 0 )
	{
		uint32_t               err_code;
		dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
		dm_application_param_t register_param;

		// Initialize persistent storage module.
		err_code = pstorage_init();
		APP_ERROR_CHECK(err_code);

		if( USE_DM )
		{
			err_code = dm_init(&init_param);
			APP_ERROR_CHECK(err_code);

			memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

			register_param.sec_param.bond         = SEC_PARAM_BOND;
			register_param.sec_param.mitm         = SEC_PARAM_MITM;
			//register_param.sec_param.lesc         = SEC_PARAM_LESC;
			register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
			register_param.sec_param.oob          = SEC_PARAM_OOB;
			register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
			register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
			register_param.evt_handler            = device_manager_evt_handler;
			register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

			err_code = dm_register(&m_app_handle, &register_param);
			APP_ERROR_CHECK(err_code);
		}
		else
		{
			// register pstorage Page to keep Log and Config in same locations
			static pstorage_handle_t  	null_handle;
			uint32_t res;
			pstorage_module_param_t param;
			param.block_size  = FLASH_PAGE_SIZE;
			param.block_count = NUM_DM_PAGES;
			param.cb          = null_cb_handler;
			res = pstorage_register(&param, &null_handle);
			if ( res == NRF_SUCCESS )
			{
				if( gs_bdebug ) app_trace_puts(DEBUG_LOW, "p_store success\r");
			}
			else
			{
				if( gs_bdebug ) app_trace_log(DEBUG_HIGH, "p_store: Failed %01u\r", res);
			}
		}
	}
}

/**@brief Function for initializing the default values of the Security Key Ring.
 *
 * @param[in] none
 */
void init_key_ring( void ) 
{
	uint8_t i;
	
	if(ERASE_BONDS)
	{
		device_index_ring = g_config.device_index_ring = 0;
		for(i=0; i<7; i++)
		{
			key_ring[i] = g_config.key_ring[i] = 0;
		}
		set_config_update_flag();
	}
	else
	{
		device_index_ring = g_config.device_index_ring;
		for(i=0; i<7; i++)
		{
			key_ring[i] = g_config.key_ring[i];
		}
	}
	/*
	for(i=0; i<7; i++)
	{
		app_trace_log(DEBUG_LOW, "[KR]: 0x%02X init_mem_manager: key_ring[i] = 0x%02X\r", i, key_ring[i]);
	}
	*/
}

/**@brief Application main function.
 */
int main(void)
{
	uint32_t err_code;
	uint32_t reset_reason = NRF_POWER->RESETREAS;
	bool power_down = false;
	T_OPMODE op_state = OPMODE_LOW_POWER;
////////	uint16_t distance;
////////	uint8_t Sidata[32];
////////	uint32_t hrm_accm;
	
	NRF_POWER->RESETREAS = 0x000F000F;

	
	gpio_defaults();
	app_trace_init();
	app_set_min_debug_priority(DEBUG_MED);	//Allow somethings to print!!!
	app_trace_log(DEBUG_HIGH, "%s\n\r%s\r", FG_CYAN, DEVICE_NAME);
	app_trace_log(DEBUG_HIGH, "Reset: 0x%08X%s\r", reset_reason, FG_RESET );
	//	SEGGER_RTT_printf(0, "\033[4;41mRESET\n");
	nrf_delay_ms(1);	//Give a little time for things to settle

	// Init timers and startup Systick timer:
	timers_init();
	
	// Startup Nonvolatile Storage & Load Config Info:
	nrf_drv_wdt_channel_feed( m_wdt_channel_id );	//feed before initializing mem
	if( init_mem_manager( MEM_DEBUG ) != NRF_SUCCESS )
	{
		flag_hw_init_error( FLASH_CHIP_INIT );
	}
	init_key_ring();
	nrf_delay_ms(5);

	// ble
	ble_stack_init();
	device_manager_init(ERASE_BONDS);
	gap_params_init();
	advertising_init();
	init_comms( m_DETACH_timer_id );
	nrf_delay_ms(25);	//Stall for DM prints. There's alot.	
	
	// init ble services
	form_version_strings();		//Form Version strings to be plugged into Device Info Service
	services_init();
	conn_params_init();
	nrf_delay_ms(5);
	
	// startup imu (starts I2C to be used by other sensors)
	if( config_imu() != NRF_SUCCESS )
	{
		flag_hw_init_error( IMU_CHIP_INIT );
	}	
	nrf_delay_ms(5);

	// init battery status (relies on I2C being started by config_imu())
	if( battery_init( BAT_DEBUG) != NRF_SUCCESS )
	{	//There is a problem with the Battery Charger IC
		//flag_hw_init_error( BAT_MANAGER_CHIP_INIT );
	}	
	else
	{	//Fully check out the Battery.
		//flag_hw_init_error( battery_self_test(false) );	//~100 ms to run
	}
	
	Si1153_twi_init();
	Si1153_Init();
	
	app_trace_log(DEBUG_MED, "Start of Main @%01u\r", getSystemTimeMs());
	
//	nrf_drv_wdt_channel_feed( m_wdt_channel_id );
//	set_hw_test_request();
//	test_hardware();
	
//	uint8_t rando_num = battery_get_voltage()%10;
	led(1,1,1,-1,0);
	
	// Enter main loop.
	for (;;)
	{		
		nrf_drv_wdt_channel_feed( m_wdt_channel_id );
		
		if ( battery_isdav() )
		{	// New Battery Reading available, update battery related states.
			// LED will be solid magenta when Charged, flashing magenta when Charging, otherwise Off.
			battery_report(&gu_battery_level, &gu_battery_state);
			indicate_battery_state();
			
			if( gu_battery_level <= LOW_BAT_THRESH )
			{
				if( power_down == false) app_trace_log(DEBUG_HIGH, "Low Battery Warning @%01u\r", getSystemTimeMs());
				power_down = true;
			}
			else
			{
				power_down = false;
			}
		}
		
		switch( op_state )
		{
			case OPMODE_LOW_POWER:
				if( !power_down || manufacture_mode() )
				{
					app_trace_log(DEBUG_MED, "Powering Up @%01u\r", getSystemTimeMs());
					
					// Light LED for testing:
					if( manufacture_mode() )
					{
						if( get_hw_test_flags() != NO_FAILURE )
						{	//Override the LED to indicate a failure
							led_over_ride( 100, 100, 150 );
						}
						else
						{	//Override the LED for manufacture test
							led_over_ride( 6000, 3000, 1 );
						}
					}
					else
					{	//allow the LED to do its thing
					}

					// set power mode
					//sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
					sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

					// restart sytick IRQ
					err_code = app_timer_start(m_SYST_timer_id, SYSTEM_TIME_TICKS, NULL);
					APP_ERROR_CHECK(err_code);

					// Start Wake On Motion (delta I = +5uA)
					enable_motion_wakeup();
					
					app_trace_puts(DEBUG_MED, "Advertising Start\r");
					err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
					APP_ERROR_CHECK(err_code);
					
					op_state = OPMODE_RUNNING;
				}
				break;
				
			case OPMODE_RUNNING:		
				
				if( m_conn_handle == BLE_CONN_HANDLE_INVALID && g_advertise_state == OFF )
				{	//No Connection and Not currently advertising: kick that Radio On!
					if(gs_bdebug) app_trace_puts(DEBUG_LOW, "Advertising Restart\r");
					err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
					APP_ERROR_CHECK(err_code);
				}
						
				//check if imu needs servicing
				service_imu();

				// handle events that are scheduled to updated based on calendar time
				service_calendar_events();
								
				// See if there is data queued to transmit, then send it
				if( m_nus.conn_handle != BLE_CONN_HANDLE_INVALID )
				{
					//Checks for changes in Battery Level and State, if changed notify Client:
					ble_bas_update();	
					
					//If Data is queued, pass it on to Client
					ble_send_nus_data();
				}
				
				// Set in Comms.c to request data memory be erased
				if( erase_records_flag )
				{	//Will continue to be called until there is nothing left to delete	
					if( delete_tail_block() == false ) 
					{	//all pages with unsent data have been erased
						erase_records_flag = false;
						set_config_update_flag();
					}
				}
				
				// If Config update has been requested, execute
				if( get_config_update_flag() ) 
				{
					update_config();
				}

				// Bootloader has been requested
				if ( g_update_and_reset )
				{
					if( MAIN_DEBUG ) app_trace_puts(DEBUG_MED, "Reset to Bootloader\r" );
					
					//Save active records and updated config values
					power_down_save_records();
					
					sd_power_gpregret_set( BOOTLOADER_DFU_START );
					reset_prepare();
					NVIC_SystemReset();
				}
					
				if( !ship_mode() ) 
				{
					if ( power_down )
					{	//enter low power hibernation mode
						app_trace_log(DEBUG_MED, "Low Power Mode @%01u\r", getSystemTimeMs());

						// If advertising, stop
						if( g_advertise_state != OFF )
						{
							advertising_stop();
							g_advertise_state = OFF;
						}
						
						// If connected, disconnect
						if( m_conn_handle != BLE_CONN_HANDLE_INVALID )
						{
							terminate_connection();
						}
						
						// turn off imu
						stop_imu();

						// save active Records
						power_down_save_records();

						// make sure LED is Off
						led_uninit();
						
						//Stop System Timer (Battery Timer will be only wake up source)
						app_timer_stop( m_SYST_timer_id );

						// change power mode
						sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
						
						op_state = OPMODE_LOW_POWER;
					}
				}
				else
				{	//Still in shipmode. Keep the battery from over charging and switch battery power Off when we are
					//no longer need to communicate.
					static TTASK_TIMER ship_mode_delay = { 0, SHIP_MODE_DELAY };
					
					if( get_hw_test_request() )
					{	//hardware tests have been requested
						test_hardware();
					}
					
					if( battery_chr_present() )
					{	//keep pushing out the shutoff timer as long as we are on the battery charger
						re_arm_task_timer( ship_mode_delay );
					}
					else 
					{	//Charger no longer Detected. Powerdown if in Manufacturing Mode and system has been On longer than 5 seconds 
						//(allows some time for LED feedback in a charger detect failure mode) or powerdown after 5 minutes off charger:
						if( task_time(ship_mode_delay) || ( manufacture_mode() && (getSystemTimeMs() > 5000)) )
						{	//Have been Off the Charger for X seconds, enter Ship Mode and turn power Off
							app_trace_log(DEBUG_MED, "Entering Ship Mode @%01u\r", getSystemTimeMs());
							
							// If advertising, stop
							if( g_advertise_state != OFF )
							{
								advertising_stop();
								g_advertise_state = OFF;
							}
							
							// If connected, disconnect
							if( m_conn_handle != BLE_CONN_HANDLE_INVALID )
							{
								terminate_connection();
							}
							
							// turn off imu
							stop_imu();
														
							// make sure LED is Off
							led_uninit();
							
							//Delete Any Records that were saved and Update Config variables
							delete_all_bonds();
							erase_reporter_config(); 
							while( delete_tail_block() );	//keep deleting log memory blocks until empty
							update_config();
							
							// change wake up period to be much more frequent (in case we need to drain battery)
							app_timer_stop( m_SYST_timer_id );
							systick_period_ms = BAT_DRAIN_TIME_PERIOD;
							systick_period_ticks = BAT_DRAIN_TIME_TICKS;
							err_code = app_timer_start(m_SYST_timer_id, BAT_DRAIN_TIME_TICKS, NULL);
							APP_ERROR_CHECK(err_code);
							
							op_state = OPMODE_SHIPPING;
						}
					}
				}
				break;
				
			case OPMODE_SHIPPING:				
				//Wait for battery to drop below 30% charger
				if( gu_battery_level <= SHIP_MODE_THRESH )
				{
					app_trace_log(DEBUG_MED, "Powering Off @%01u\r", getSystemTimeMs());
					delay_ms(250);	//Give time for print before powering Off
					
					if( true )
					{
						//Set the Shipping bit to turn ourselves OFF
						battery_shutoff();
						while(1);	//wait for Ship Mode to switch power Off
					}
					else
					{
						//older boards will need to enter this mode since they cannot shut off battery power:
						system_powerdown();	
					}
				}
				else
				{					
					delay_ms(25);	//waste some time in full power mode to up the current drain
				}

				//if device goes back on charger before shutting Off, go back to standard mode
				if( battery_chr_present() )
				{	//if we haven't shutoff yet and the battery charger is detected, go ahead and restart the program
					NVIC_SystemReset();
				}
				break;
			
			default:
				op_state = OPMODE_LOW_POWER;
				break;
		}
		
		//Put processor into sleep mode until next event:
//		err_code = sd_app_evt_wait();
//		APP_ERROR_CHECK(err_code);
		
		
/*                      Proto 1153 HRM                                  */
	if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
	{		
			tx_payload[0] = (uint8_t)(accel_x >>8);
			tx_payload[1] = (uint8_t)(accel_x & 0xFF);
			tx_payload[2] = 0;//(uint8_t)(accel_y >>8);
			tx_payload[3] = hrm_step;//(uint8_t)(accel_y & 0xFF);
			hrm_step = 0;
			tx_payload[4] = (uint8_t)(accel_z >>8);
			tx_payload[5] = (uint8_t)(accel_z & 0xFF);
			tx_payload[6] = (uint8_t)Si115xReadFromRegister(SI115x_REG_HOSTOUT1);
			tx_payload[7] = (uint8_t)Si115xReadFromRegister(SI115x_REG_HOSTOUT2);
			tx_payload[8] = (uint8_t)Si115xReadFromRegister(SI115x_REG_HOSTOUT4);
			tx_payload[9] = (uint8_t)Si115xReadFromRegister(SI115x_REG_HOSTOUT5);
			
			//Si115xBlockRead(SI115x_REG_IRQ_STATUS, 1, Sidata);
			Si115xWriteToRegister(SI115x_REG_COMMAND, 0x11); // Force cmd
			hrm_read_cmd();
			//Si115xDelay_10ms();				

			tx_type = HRM_PKT;
			tx_pay_len = 11; 
			queue_packet_wrapper( tx_type, tx_payload, tx_pay_len );
			ble_transfer_packets_wrapper();		//pass queued messages to the radio
			delay_ms(40);
		}
	nrf_drv_wdt_channel_feed( m_wdt_channel_id );
	}//end forever loop
}//end main


/**
 * @}
 */
