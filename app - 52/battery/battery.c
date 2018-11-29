/*
 * battery.c
 *
 * Created: 10/10/2014 2:07:19 PM
 *  Author: richkl
 */ 

#include "battery.h"
#include "app_error_vsp.h"
#include "nrf_soc.h"
#include "nrf_saadc.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "bq25120.h"

//Periods at which Battery Voltage will be measured:
#define CHR_INTERVAL				5000		//number of milliseconds
#define RUN_INTERVAL				5000
#define LOW_INTERVAL				5000
// Number of App Timer ticks in those periods:
#define BATTERY_MEASURE_INTERVAL_CHARGE	(APP_TIMER_TICKS(CHR_INTERVAL, APP_TIMER_PRESCALER))
#define BATTERY_MEASURE_INTERVAL_RUN	(APP_TIMER_TICKS(RUN_INTERVAL, APP_TIMER_PRESCALER))
#define BATTERY_MEASURE_INTERVAL_LOW	(APP_TIMER_TICKS(LOW_INTERVAL, APP_TIMER_PRESCALER))
#define BATTERY_MEASURE_SETTLE_TIME		APP_TIMER_MIN_TIMEOUT_TICKS		//minimum of 5 counts
#if ( WDT_CONFIG_RELOAD_VALUE <= LOW_INTERVAL )
 //This event is what will cause the dog to get fed in hibernate mode
 #error "WDT Period must be greater than Low Power Battery measure interval"
#endif

APP_TIMER_DEF(m_START_BAT_MEASURE_timer_id);
APP_TIMER_DEF(m_BAT_MEASURE_SM_timer_id);

static nrf_saadc_value_t			sample;
static nrf_ppi_channel_t       		bat_ppi_channel;

static bool volatile battery_debug;
static volatile bool battery_dav;
bool monitor_en = false;

////filter design from: http://www-users.cs.york.ac.uk/~fisher/mkfilter/racos.html
////Fcutoff = 0.1*Fsamp
//#define NZEROS			2
//#define AD_BUF_LEN		(NZEROS+1)
//#define AD_FILT_GAIN	2802
//static uint32_t xcoeffs[] = { 901, 1000, 901 };

const char s_bat_low[] = 			"LOW";
const char s_bat_discharging[] = 	"DISCHARGING";
const char s_bat_charging[] = 		"CHARGING";
const char s_bat_charged[] = 		"CHARGED";
const char s_bat_mon_fail[] = 		"FAILED";
const char s_bat_unknown[] =		"?";

#define AD_BUF_LEN 4
static struct {
	uint16_t buf[AD_BUF_LEN];
	uint8_t  buf_ptr;
} adc_sample = { {0}, 0 };

static struct {
	T_BATTERY_STATE state;
	uint8_t percent_rem;
	uint32_t volt_mv;
} battery;

//massaged version, may differ from battery.perc_update because that version holds info for recording
static uint8_t reporter_perc_rem;
static bool reporter_perc_update;
static bool chr_present = false;

const uint16_t battery_lut[] =
{
  3000,
  3000,3269,3368,3437,3493,3536,3568,3596,3621,3636,3655,3659,3662,3666,3669,3672,3675,3678,3681,3684,
  3688,3691,3695,3698,3702,3706,3710,3714,3718,3722,3725,3729,3733,3737,3740,3744,3747,3751,3754,3757,
  3760,3763,3765,3768,3771,3773,3776,3778,3780,3783,3785,3788,3790,3793,3796,3798,3801,3805,3808,3811,
  3815,3819,3823,3827,3832,3836,3841,3846,3852,3857,3863,3868,3874,3880,3886,3893,3899,3905,3911,3917,
  3923,3929,3935,3941,3947,3953,3958,3964,3969,3975,3981,3986,3993,3999,4006,4014,4022,4031,4042,4050,
  0xFFFF
};

//For Datalogging of battery levels, save 101 when device goes onto the Charger
#define CHARGING_IND_LEVEL			101

//Voltage based Thresholds:
#define CHARGER_PRESENT_VOLT_THRES 	4250		//minimum voltage that needs to be applied by charger
#define CHARGE_CYCLE_THRES			4050		//Cycle Charger back on after voltage falls from: 4150 to 4050
#define LOW_VOLT_THRES 				(battery_lut[LOW_BAT_THRESH])		//consider Battery Charged once it has reached this voltage
#define SHUTOFF_VOLT_THRES			2600		//battery Protection circuit cuts out at a minimum of 2700 mVolts. Nothing below that level can be correct

static void battery_monitor_start( void );
static void battery_monitor_stop( void );
static void battery_monitor_interval( uint );
static void battery_reading_sm( uint16_t adv_res );
static void battery_adc_callback(nrf_drv_saadc_evt_t const * p_event);
static void battery_update_status( void );
static void battery_measurement_timer_handler(void * p_context);
static void battery_sm_timer_handler(void * p_context);
static bool battery_enable_charging( void );
static bool battery_disable_charging( void );

bool battery_isdav(void)
{
	return battery_dav;
}

bool battery_chr_present( void )
{
	return chr_present;
}

T_BATTERY_STATE battery_get_current_state(void)
{
	return battery.state;
}

uint16_t battery_get_voltage(void)
{
	return battery.volt_mv;
}

uint8_t battery_get_level( void )
{	//return more raw version for recording
	return reporter_perc_rem;	
}

bool battery_level_change( uint8_t * remain )
{
	if( reporter_perc_update == true ) 
	{
		reporter_perc_update = false;
		*remain = reporter_perc_rem;
		return true;
	}
	
	return false;
}

//Note: Do not call function from an ISR as the I2C lines are used and we need 
//to play nice with them:
void battery_report( uint8_t * per_rem, uint8_t * state)
{	//get remaining level to report to user
	battery_update_status();
	
	*per_rem = battery.percent_rem;
	*state = battery.state;
}

void battery_monitor_shutdown(void)
{
	battery_monitor_stop();
	
	nrf_drv_ppi_channel_free(bat_ppi_channel);
	nrf_drv_saadc_uninit();
	nrf_saadc_disable();
}

/***************************************************************************//**
 * @brief
 * 		Turn Off the Power Supply from the battery
 * @param[out] true/false
 * 		true if operation succeeded, false otherwise
 ******************************************************************************/
bool battery_shutoff( void )
{
	bool ret_val = false;
	
	#ifdef BQ25120
		ret_code_t err = bq25120_en_shipmode();
		if( err != NRF_SUCCESS ) 
		{
			app_trace_log(DEBUG_MED, "[BAT_SHUTOFF] Err: %01u\r\n", err);
			ret_val = false;
		}
		else
		{
			ret_val = true;
		}
	#else
		//no controls to turn off battery power
	#endif
	
	return ret_val;
}

ret_code_t battery_init( bool debug )
{
	ret_code_t err;
	battery_debug = debug;
	
	if( battery_debug) app_trace_puts(DEBUG_MED, "[BAT_INIT]: start\r\n");

	//Create 2 timers: One that continuously initiates Battery Reads, another 
	//that progresses the Battery Read State Machine after initiation.
	err= app_timer_create( &m_START_BAT_MEASURE_timer_id, APP_TIMER_MODE_REPEATED, battery_measurement_timer_handler );
	APP_ERROR_CHECK(err);
	err= app_timer_create( &m_BAT_MEASURE_SM_timer_id, APP_TIMER_MODE_SINGLE_SHOT, battery_sm_timer_handler );
	APP_ERROR_CHECK(err);
	
	#ifdef BQ25120
		nrf_gpio_cfg_input( CHARGE_PG_INPUT, NRF_GPIO_PIN_PULLUP );
		// Init the BQ25120
		err = bq25120_init();
		if( err != NRF_SUCCESS )
		{
			app_trace_log(DEBUG_MED, "[BAT_INIT]: BQ25120 Init Failed %01d\r\n", err);
			return err;
		}
	#else
		nrf_gpio_cfg_input( CHARGE_PG_INPUT, NRF_GPIO_PIN_NOPULL );	
	#endif

	// set up ppi
	err = nrf_drv_ppi_init();
	if( err != NRF_SUCCESS ) return err;
	err = nrf_drv_ppi_channel_alloc(&bat_ppi_channel);
	if( err != NRF_SUCCESS ) return err;
	nrf_drv_ppi_channel_enable(bat_ppi_channel);
	
	// init saadc
	uint8_t channel_id = 0;
	nrf_saadc_channel_config_t channel_config = SAADC_BATTERY_CHANNEL_CONFIG( BAT_HALF_IN );
	
	nrf_drv_saadc_config_t  saadc_config = SAADC_BATTERY_CONFIG;
	err = nrf_drv_saadc_init(&saadc_config, battery_adc_callback);
	if( err != NRF_SUCCESS ) return err;

	err = nrf_drv_saadc_channel_init( channel_id, &channel_config );
	if( err != NRF_SUCCESS ) return err;
	
	NRF_SAADC->CH[channel_id].CONFIG |= (1<<24);	//enable burst mode

	err = nrf_drv_saadc_buffer_convert(&sample, 1);
	if( err != NRF_SUCCESS ) return err;	

	// battery tracking variables
	reporter_perc_rem = CHARGING_IND_LEVEL;
	reporter_perc_update = false;
	battery.state = BATTERY_DISCHARGING;
	battery.percent_rem = 100;
	battery.volt_mv = 0;
	adc_sample.buf_ptr = 0;
	
	//delay for BQ25120, otherwise it fails to respond during first measurement?
	delay_us(150);		
	
	// Setup as output so Voltage Divider can be turned On/Off
	nrf_gpio_pin_clear(BAT_HALF_EN);
	nrf_gpio_cfg_output(BAT_HALF_EN);
	
	//Kick start first Reading
	battery_reading_sm( NULL );	//kick off a single reading	
	while( !battery_dav );		//wait for reading to finish
	battery_update_status();
	
	//Start continuous Reading:
	battery_monitor_start();
	
	if( battery_debug) app_trace_puts(DEBUG_LOW, "[BAT_INIT]: done\r\n");	
	
	return NRF_SUCCESS;
}	

/***************************************************************************//**
 * @brief
 * 		Turn the battery charger On
 * @param[out] true/false
 * 		true if operation succeeded, false otherwise
 ******************************************************************************/
static bool battery_enable_charging( void )
{
	bool res = false;
	
	#ifdef BQ25120
		bq25120_charge_disable( false );
		res = true;
	#else
		//no controls to stop the charger
	#endif
	
	return res;
}

/***************************************************************************//**
 * @brief
 * 		Turn the battery charger Off
 * @param[out] true/false
 * 		true if operation succeeded, false otherwise
 ******************************************************************************/
static bool battery_disable_charging( void )
{
	bool res = false;
	
	#ifdef BQ25120
		bq25120_charge_disable( true );
		res = true;
	#else
		//no controls to stop the charger
	#endif
	
	return res;
}

static void battery_monitor_start( void )
{
	ret_code_t err;
	battery_dav = false;
	monitor_en = true;
	
	//Start Up continuous measurement timer
	err = app_timer_start(m_START_BAT_MEASURE_timer_id, BATTERY_MEASURE_INTERVAL_RUN, NULL);	
	APP_ERROR_CHECK(err);
}

static void battery_monitor_stop( void )
{
	ret_code_t err;
	monitor_en = false;
	
	err = app_timer_stop(m_START_BAT_MEASURE_timer_id);
	err = app_timer_stop(m_BAT_MEASURE_SM_timer_id);
	
	nrf_gpio_pin_clear(BAT_HALF_EN);
	
	battery_dav = false;
	
	APP_ERROR_CHECK(err);
}

static void battery_monitor_interval( uint bat_state )
{
	ret_code_t err;	
	uint32_t measure_interval;
	
	if( monitor_en == true ) {
		err = app_timer_stop(m_START_BAT_MEASURE_timer_id);
		APP_ERROR_CHECK(err);
		
		switch ( bat_state ) {
			case BATTERY_CHARGING:
				measure_interval = BATTERY_MEASURE_INTERVAL_CHARGE;
				break;
			
			case BATTERY_CHARGED:
				measure_interval = BATTERY_MEASURE_INTERVAL_CHARGE;
				break;
			
			case BATTERY_DISCHARGING:
				measure_interval = BATTERY_MEASURE_INTERVAL_RUN;	
				break;
			
			case BATTERY_MONITOR_FAIL:
				measure_interval = BATTERY_MEASURE_INTERVAL_RUN;	
				break;
			
			default:
				measure_interval = BATTERY_MEASURE_INTERVAL_LOW;
				break;
		}	

		err = app_timer_start(m_START_BAT_MEASURE_timer_id, measure_interval, NULL);	
		APP_ERROR_CHECK(err);
	}
}

// battery subsystem data
T_BQ25120_STATUS bq25120_response = BQ_READY;
static void battery_update_status( void )
{
	static const char * s_state = s_bat_discharging;	//Startup State
	T_BATTERY_STATE	next_state = BATTERY_DISCHARGING;
	uint8_t perc_rem = 0;
	int16_t ad_cnt;
	int32_t temp_bat_mv;
	bool charging_en = false;
	
	if( battery_dav == false )
	{	//No ADC reading to analyze
		app_trace_log(DEBUG_MED, "[BAT_UPDATE] %sNo Data Ava%s @%01u\r\n", FG_RED, FG_RESET, getSystemTimeMs());
		return;
	}
	battery_dav = false;		//clear data available flag
	
	ad_cnt = adc_sample.buf[adc_sample.buf_ptr];		
	temp_bat_mv = (ad_cnt * (int64_t)(ADC_VREF * ADC_SCALE * (RES_DIV_BOT + RES_DIV_TOP)))/RES_DIV_BOT;
	temp_bat_mv /= ADC_RES;
	if( temp_bat_mv < 0 ) temp_bat_mv = 0;	//positive values only
		
	// Percentage Remaining based off Lookup Table Curve Fit	
	while( battery_lut[perc_rem] < temp_bat_mv )
	{
		if( ++perc_rem >= sizeof( battery_lut) ) 
		{
			perc_rem = sizeof( battery_lut);
			break; 
		}
	}
	if ( perc_rem > 100 ) perc_rem = 100;
	
	#if defined LTC4070	
		//read state of charge indication pin:
		uint8_t charge_indicate_pin = nrf_gpio_pin_read(CHARGE_PG_INPUT);
	
		if( temp_bat_mv > CHARGER_PRESENT_VOLT_THRES ) 
		{	
			chr_present = true;	//Flag that a Valid Charger Voltage is present
			charging_en = true;
		}
		else 
		{
			chr_present = false;
			charging_en = false;
		}
		
	#else
		//read to make sure charger is Happy
		//BQ_READ_FAIL = 	0,
		//BQ_READY =		1,
		//BQ_CHARGING =		2,
		//BQ_CHARGED =		3,
		//BQ_FAULT =		4,
		bq25120_response = bq25120_read_status();			
		if( bq25120_response == BQ_CHARGED || bq25120_response == BQ_CHARGING )
		{	//Charging is good, no faults
			chr_present = true;	//Flag that a Valid Charger Voltage is present
			charging_en = true;
		}
		else
		{	//Valid Charger Voltage could still be present, however, we may not be all to charge due to some fault:
			chr_present = bq25120_charger_connected();	
			charging_en = false;
			if( bq25120_response == BQ_FAULT || bq25120_response == BQ_READ_FAIL )
			{	//in an error state
				temp_bat_mv = 0;	//force into BAT_MONITOR_FAIL state
			}
		}
	#endif
		
	//clear update flag (in case nothing else has)
	reporter_perc_update = false;
		
	//Battery State machine:
	switch ( battery.state )
	{				
		case BATTERY_DISCHARGING:
			if( temp_bat_mv <= SHUTOFF_VOLT_THRES ) 
			{	//The battery Auto Disables around ~2.8V. To read this low implies there is a fault with the measurement circuit. 	
				perc_rem = LOW_BAT_THRESH+1;		//keep us alive for now		
				next_state = BATTERY_MONITOR_FAIL;
			}
			else if( temp_bat_mv <= LOW_VOLT_THRES ) 
			{
				next_state = BATTERY_LOW;	
			}
			else if( charging_en == true ) 
			{
				next_state = BATTERY_CHARGING;	
			}
			else
			{
				if ( perc_rem < reporter_perc_rem) 
				{	//update calling functions battery level
					reporter_perc_rem = perc_rem;
					reporter_perc_update = true;
					//app_trace_puts(DEBUG_LOW, "LEVEL UPDATE!!!!\r\n");
				}
				else
				{	//set to lowest recorded reading to Stop remaining battery level from fluctuating
					perc_rem = reporter_perc_rem;	
				}
				
				next_state = BATTERY_DISCHARGING;	
			}
			break;
					
		case BATTERY_CHARGING:
			if ( perc_rem >= reporter_perc_rem)
			{	//Only true just after being placed on the charger ( next time bat_percent = 101 )
				reporter_perc_rem = CHARGING_IND_LEVEL;	//Change Local Percent Remaining to indicate Charging
				reporter_perc_update = true;
			}
			
			#if defined LTC4070
				if( charging_en == true ) 
				{	
					if ( charge_indicate_pin == CHARGE_PG_ACTIVE ) 
					{
						next_state = BATTERY_CHARGED;
					}
					else 
					{
						next_state = BATTERY_CHARGING;
					}
				}
				else 
				{	
					next_state = BATTERY_DISCHARGING;
				}
			#else
				if( charging_en == true ) 
				{	
					//Input Power is good, Charging or Charged
					if( bq25120_response == BQ_CHARGED )
					{
						reporter_perc_rem = CHARGING_IND_LEVEL;	//Change Local Percent Remaining to Indicate Charging (101%)
						perc_rem = 100;					//Inform calling function of 100% Charge
						next_state = BATTERY_CHARGED;	
					}
					else 
					{
						next_state = BATTERY_CHARGING;
					}
				}
				else
				{	//Charger not connected
					next_state = BATTERY_DISCHARGING;
				}
			#endif
			break;
		
		case BATTERY_CHARGED:
			reporter_perc_rem = CHARGING_IND_LEVEL;	//Change Local Percent Remaining to Indicate Charging (101%)
			perc_rem = 100;					//Inform calling function of 100% Charge
		
			if( charging_en == true ) 
			{	//Stay in Charged state until Charger is removed		
				next_state = BATTERY_CHARGED;
			}
			else
			{	
				next_state = BATTERY_DISCHARGING;
			}
			break;
		
		case BATTERY_LOW:
			if ( charging_en == true ) 
			{
				reporter_perc_rem = perc_rem;
				next_state = BATTERY_CHARGING;
			}
			else if( perc_rem > (LOW_BAT_THRESH+2) ) 
			{	//Voltage has somehow managed to climb it's way above hysteresis threshold. 
				reporter_perc_rem = perc_rem;
				next_state = BATTERY_DISCHARGING;
			}
			else
			{	//Stay in Low power State
				perc_rem = LOW_BAT_THRESH;
				next_state = BATTERY_LOW;	
			}
			break;
		
		case BATTERY_MONITOR_FAIL:
			if( temp_bat_mv > (SHUTOFF_VOLT_THRES+50) ) //50 mV of hysteresis
			{	//Voltage Reading has returned??? Get to go back to an Discharging State
				reporter_perc_rem = perc_rem;
				next_state = BATTERY_DISCHARGING;
			}
			else
			{	//Still looks bad news bears...
				#if defined LTC4070	
					if( charge_indicate_pin == CHARGE_PG_ACTIVE ) 
					{	//Show 100% when fully Charged
						perc_rem = 100;
					}
					else 
					{	//Show 3% when no longer fully charged
						perc_rem = LOW_BAT_THRESH+1;	//Set to lowest value that will keep Device alive
					}
				#else
					if( charging_en == true ) 
					{	//Show 100% when Charging
						perc_rem = 100;
					}
					else 
					{	//Show 3% when not charging
						perc_rem = LOW_BAT_THRESH+1;	//Set to lowest value that will keep Device alive
					}
				#endif
				
				if ( perc_rem != reporter_perc_rem) 
				{	//update calling functions battery level
					reporter_perc_rem = (uint8_t)perc_rem;
					reporter_perc_update = true;
					//app_trace_puts(DEBUG_LOW, "LEVEL UPDATE\r\n");
				}
				
				next_state = BATTERY_MONITOR_FAIL;
			}
			break;
		
		default:
			reporter_perc_rem = perc_rem;
			next_state = BATTERY_DISCHARGING;
			break;
	}
	
	//Battery State just changed
	if( battery.state != next_state )
	{	
		//Update Measurement Interval depending on new battery state
		battery_monitor_interval( next_state );
		
		//Debug print new battery Satus info
		if( battery_debug )
		{
			switch( next_state )
			{
				case BATTERY_LOW:
					s_state = s_bat_low;
					break;
				case BATTERY_CHARGING:
					s_state = s_bat_charging;
					break;
				case BATTERY_DISCHARGING:
					s_state = s_bat_discharging;
					break;
				case BATTERY_CHARGED:
					s_state = s_bat_charged;
					break;
				case BATTERY_MONITOR_FAIL:
					s_state = s_bat_mon_fail;
					break;
				default:
					s_state = s_bat_unknown;
					break;
			}
			app_trace_log(DEBUG_MED, "[BAT_UPDATE] S: %s %01umV %01u%% %01ucnt @%01u\r\n", s_state, temp_bat_mv, perc_rem, ad_cnt, getSystemTimeMs());
		}
	}
	else if( reporter_perc_update == true )
	{
		if( battery_debug ) 
		{	//Debug Print new Battery Percent Remaining:
			app_trace_log(DEBUG_MED, "[BAT_UPDATE] P: %s %01umV %01u%% %01ucnt @%01u\r\n", s_state, temp_bat_mv, perc_rem, ad_cnt, getSystemTimeMs());
			//app_trace_log(DEBUG_MED, "[BAT_STAT]: %01u%%, %01u mV, %01u cnt, state %01u \r\n", bat_per_rsp, bat_volt_mv, ad_cnt, bat_sv);
		}
	}
	else
	{
		//nothing has changed since last call
		static uint8_t last_pg = 1;
		
		if( last_pg != nrf_gpio_pin_read(CHARGE_PG_INPUT) )
		{
			last_pg = nrf_gpio_pin_read(CHARGE_PG_INPUT);
			app_trace_log(DEBUG_LOW, "[BAT_UPDATE]: PG %01u @%01u\r\n", last_pg, getSystemTimeMs());
		}
	}
	
	//Update battery info with the new info
	battery.state = next_state;
	battery.volt_mv = temp_bat_mv;
	battery.percent_rem = perc_rem;
}

//First callback from continous timer that initiates a Battery Voltage reading:
static uint8_t bat_measure_sv = 0;
static void battery_measurement_timer_handler(void * p_context)
{
	bat_measure_sv = 0;			//make sure we start back at first State
	battery_reading_sm( NULL );
}

//Second callback from One Shot Timer after delay has been met:
static void battery_sm_timer_handler( void * p_context )
{	
	battery_reading_sm( NULL );
}

//Third Callback to from AD Converter with Voltage Measurement:
static void battery_adc_callback(nrf_drv_saadc_evt_t const * p_event)
{	
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
		// Event generated when the buffer is filled with samples.		
		//if(battery_debug) app_trace_puts(DEBUG_LOW, "[BAT_AD_CB]: done\r\n");
		
		battery_reading_sm( p_event->data.done.p_buffer[0] );
	}
	else {
		app_trace_log(DEBUG_MED, "[BAT_ADC_CB]: %01u\r\n", p_event->type );
	}
}

static void battery_reading_sm( uint16_t adc_res )
{
	battery_dav = false;
	
	switch ( bat_measure_sv )
	{
		case 0:
			//if( battery_debug) app_trace_puts(DEBUG_LOW, "[BAT_TIME_CB]: start\r\n");
		
			// Turn on analog battery voltage divider, needs time to rise and settle
			nrf_gpio_pin_set(BAT_HALF_EN);
		
			//Set to Go Off again soon after the analog battery voltage has settled
			app_timer_start(m_BAT_MEASURE_SM_timer_id, BATTERY_MEASURE_SETTLE_TIME, NULL);

			bat_measure_sv = 1;
			break;
		
		case 1:		
			//Output has settled, Start AD conversions (burst mode will average readings for ~300 uSec)			
			nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
			nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
				
			bat_measure_sv = 2;
			break;
		
		case 2:
			//turn Off voltage divider to save power
			nrf_gpio_pin_clear(BAT_HALF_EN);
	
			//Pre-increment so ptr points directly to most recent reading:
			adc_sample.buf_ptr++;
			if( adc_sample.buf_ptr >= AD_BUF_LEN ) {
				adc_sample.buf_ptr = 0;
			}
			adc_sample.buf[adc_sample.buf_ptr] = adc_res;

			//set data available flag so main loop can synchronuously update the battery readings
			battery_dav = true;		
			
			bat_measure_sv = 0;
			break;
		
		default:
			//Set timer to quickly bring us back to handler in the proper state:
			bat_measure_sv = 0;
			break;
	}
}

//CHARGER_NOT_DETECT =		0x0001,
//BAT_MEASURE_CIRCUIT =		0x0002,
//BAT_MEASURE_SWITCH =		0x0004,
//BAT_NOT_INCLUDED =		0x0008,
//BAT_MANAGER_CHIP_FAULT = 	0x0010,
T_HW_FAILURE_FLAGS battery_self_test( bool test4chr )
{
	T_HW_FAILURE_FLAGS fail_flag = NO_FAILURE;
	
	app_trace_puts( DEBUG_MED, "[BAT_TEST]: Start\r\n" );
	
	battery_monitor_stop();
	
	do 
	{	
		// If requested, test that input charging voltage is present. If not, return failure:
		if( test4chr && !battery_chr_present() )
		{
			app_trace_log( DEBUG_MED, "[BAT_TEST]: Charger Not Detected\r\n" );
			fail_flag = CHARGER_NOT_DETECT;
			break;
		}
		
		// Check charger disabled Reading		
		if( battery_disable_charging() )
		{	//Charger Disable Issued
			//wait 250 ms for Disable to take effect (tested with no battery connected)
			nrf_gpio_pin_set(BAT_HALF_EN);	//Turn On Pulldown (if no battery connected this will pull the floating Vbat line low)
			nrf_delay_ms(100);				//Wait for Caps to drain (assuming No battery)
			
			battery_reading_sm( NULL );		//kick off a single reading	
			while( !battery_dav );			//wait for conversion to complete
			battery_update_status();
			battery_enable_charging();		//re-enable charging(after updating status, CD line is picky)
			if( battery.volt_mv < 3000 )
			{	//should not read less than 3000 mV if Battery is connected and good
				app_trace_log( DEBUG_MED, "[BAT_TEST]: Charge Disabled Reading: %01u \r\n", battery.volt_mv );
				fail_flag = BAT_NOT_INCLUDED;
				break;
			}
		}
		battery_enable_charging();	//re-enabled charger for remaining tests
		
		//Check the TI battery Charger Response to the Charger Disabled Reading:
		if( bq25120_response == BQ_FAULT || bq25120_response == BQ_READ_FAIL )
		{	//TI charger BQ25120 is detecting a problem
			app_trace_log( DEBUG_MED, "[BAT_TEST]: BQ25120 Response %01u\r\n", bq25120_response );
			fail_flag = BAT_MANAGER_CHIP_FAULT;
			break;
		}
		
		// Next Reading with Voltage Divider Off (sanity check of Voltage divider Control pin):
		nrf_gpio_cfg_input(BAT_HALF_EN, NRF_GPIO_PIN_PULLDOWN);
		battery_reading_sm( NULL );	//kick off a single reading			
		while( !battery_dav );		//wait for convertion to complete
		battery_update_status();
		nrf_gpio_pin_clear(BAT_HALF_EN);
		nrf_gpio_cfg_output(BAT_HALF_EN);
		if( battery.volt_mv > 100 )
		{	//should read less than 100 mV
			app_trace_log( DEBUG_MED, "[BAT_TEST]: %01u > 100 mV\r\n", battery.volt_mv );
			fail_flag = BAT_MEASURE_SWITCH;
			break;
		}
		
		//Take a Normal Reading:
		battery_reading_sm( NULL );	//kick off a single reading	
		while( !battery_dav );		//wait for convertion to complete
		battery_update_status();
		if( battery.volt_mv < 3000 || battery.volt_mv > 4250 )
		{	//should not read less than 3000 mV of more than 4250 mV
			app_trace_log( DEBUG_MED, "[BAT_TEST]: Normal Reading: %01u\r\n", battery.volt_mv );
			fail_flag = BAT_MEASURE_CIRCUIT;
			break;
		}
		
		//Test complete, break from loop:
		app_trace_log( DEBUG_MED, "[BAT_TEST]: Passed\r\n" );
		fail_flag = NO_FAILURE;
		break;
		
	} while( true );
	
	//take a final reading in a normal state, so as to reset the battery monitor status varaibles:
	battery_reading_sm( NULL );	//kick off a single reading	
	while( !battery_dav );		//wait for convertion to complete
	
	battery_monitor_start();
	
	return fail_flag;
}
