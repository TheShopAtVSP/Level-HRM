/*
 * battery.c
 *
 * Created: 10/10/2014 2:07:19 PM
 *  Author: richkl
 */ 

#include "global.h"
#include "battery.h"
#include "app_error_vsp.h"
#include "nrf_soc.h"
#include "nrf_saadc.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_gpio.h"

static nrf_saadc_value_t			sample;
static nrf_ppi_channel_t       		bat_ppi_channel;
static app_timer_id_t 				local_battery_timer_id;

static bool volatile battery_debug;
static volatile bool battery_dav;
static volatile uint8_t current_state;
bool monitor_en = false;
static volatile int g_battery_stable_counter;

////filter design from: http://www-users.cs.york.ac.uk/~fisher/mkfilter/racos.html
////Fcutoff = 0.1*Fsamp
//#define NZEROS			2
//#define AD_BUF_LEN		(NZEROS+1)
//#define AD_FILT_GAIN	2802
//static uint32_t xcoeffs[] = { 901, 1000, 901 };

#define AD_BUF_LEN 4
static struct {
	uint16_t buf[AD_BUF_LEN];
	uint8_t  buf_ptr;
} adc_sample = {0};

static uint32_t batvoltage_mv = 0;
static uint8_t bat_percent = 100;
static bool new_bat_percent = false;
#define CHARGING_VOLTAGE 	4250		//minimum voltage that needs to be applied by charger

#define	USE_BATTERY_LUT
#ifdef USE_BATTERY_LUT
const uint16_t battery_lut[] =
{
  0,
  3043,3269,3368,3437,3493,3536,3568,3596,3621,3636,3655,3659,3662,3666,3669,3672,3675,3678,3681,3684,
  3688,3691,3695,3698,3702,3706,3710,3714,3718,3722,3725,3729,3733,3737,3740,3744,3747,3751,3754,3757,
  3760,3763,3765,3768,3771,3773,3776,3778,3780,3783,3785,3788,3790,3793,3796,3798,3801,3805,3808,3811,
  3815,3819,3823,3827,3832,3836,3841,3846,3852,3857,3863,3868,3874,3880,3886,3893,3899,3905,3911,3917,
  3923,3929,3935,3941,3947,3953,3958,3964,3969,3975,3981,3986,3993,3999,4006,4014,4022,4031,4042,4050,
  0xFFFF
};
#else

#endif

bool battery_isdav(void)
{
	return(battery_dav);
}

uint8_t battery_get_current_state(void)
{
	return current_state;
}

uint16_t battery_get_voltage(void)
{
	return batvoltage_mv;
}

uint8_t battery_get_level( void )
{	
	return bat_percent;
}

bool battery_level_change( uint8_t * remain )
{
	if( new_bat_percent == true ) {
		new_bat_percent = false;
		*remain = bat_percent;
		return true;
	}
	
	return false;
}

void battery_monitor_start( void )
{
	monitor_en = true;
	
	adc_sample.buf_ptr = 0;
	
	battery_monitor_interval( BATTERY_DISCHARGING );	//Assume Discharge for now
}

void battery_monitor_stop( void )
{
	monitor_en = false;
	
	ret_code_t err_code = app_timer_stop(local_battery_timer_id);
	APP_ERROR_CHECK(err_code);
}

void battery_monitor_interval( uint bat_state )
{
	ret_code_t err_code;	
	
	if( monitor_en == true ) {
		err_code = app_timer_stop(local_battery_timer_id);
		APP_ERROR_CHECK(err_code);
		
		switch ( bat_state ) {
			case BATTERY_CHARGING:
				err_code = app_timer_start(local_battery_timer_id, BATTERY_MEASURE_INTERVAL_CHARGE, NULL);
				break;
			
			case BATTERY_CHARGED:
				err_code = app_timer_start(local_battery_timer_id, BATTERY_MEASURE_INTERVAL_CHARGE, NULL);
				break;
			
			case BATTERY_DISCHARGING:
				err_code = app_timer_start(local_battery_timer_id, BATTERY_MEASURE_INTERVAL_RUN, NULL);		
				break;
			
			default:
				err_code = app_timer_start(local_battery_timer_id, BATTERY_MEASURE_INTERVAL_LOW, NULL);	
				break;
		}		
		APP_ERROR_CHECK(err_code);
	}
}

void battery_measurement_timer_handler(void * p_context)
{
	battery_dav = false;
	
	DEBUG_PRINT(0,"bat_time_handler: start",battery_debug);	
	// switch in battery half
	nrf_gpio_pin_set(BAT_HALF_ON);
	
	// start sample
	nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
	nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
}


void battery_callback(nrf_drv_saadc_evt_t const * p_event)
{
	if(battery_debug) app_trace_s_msg("bat_callback: ");
	
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
		// Event generated when the buffer is filled with samples.
		adc_sample.buf_ptr++;
		if( adc_sample.buf_ptr >= AD_BUF_LEN ) {
			adc_sample.buf_ptr = 0;
		}
		adc_sample.buf[adc_sample.buf_ptr] = p_event->data.done.p_buffer[0];
			
		nrf_gpio_pin_clear(BAT_HALF_ON);	

		battery_dav = true;		
    }
	else {
		if(battery_debug) app_trace_log( "%01u", p_event->type );
	}
	
	if(battery_debug) app_trace_log("\r");
}

// battery subsystem data
void battery_status_level(uint8_t *level,uint8_t *state)
{
	uint8_t temp_state;
	volatile uint32_t i;
	volatile uint32_t charge_pin;
	volatile int32_t ad_cnt = adc_sample.buf[adc_sample.buf_ptr];
	//volatile uint32_t samp;
	volatile int32_t bat_left_per;
	volatile uint32_t temp_bat_mv;
	
	//adc_sample_ptr is post incremented, thus it's pointing to the oldest sample in the circular buffer.
//	samp = adc_sample_ptr;
//	for(i=0; i<AD_BUF_LEN; i++) 
//	{
//		filter_bat += (uint32_t) xcoeffs[i] * adc_sample_buf[samp];
//		samp++;
//		if( samp >= AD_BUF_LEN ) {
//			samp = 0;
//		}
//		__asm volatile ("nop");
//	}
//	filter_bat /= AD_FILT_GAIN;

	temp_bat_mv = ad_cnt * ADC_VREF * ADC_SCALE * RESISTOR_DIVIDER_SCALE;	
	temp_bat_mv /= ADC_RES;
	
#ifndef  USE_BATTERY_LUT	
	bat_left_per = 100*(temp_bat_mv - BATLEVEL_0);
	bat_left_per /= BAT_RANGE;
#else
	switch (g_battery_stable_counter) {
		case 1:
			bat_left_per = 100;
			new_bat_percent = true;			//Off of Charger, Remaining Percent is starting back at 100%
			g_battery_stable_counter++;
			break;
		
		case 2:
			for( bat_left_per = 1; bat_left_per<100; bat_left_per++ ) 
			{		
				if ( battery_lut[bat_left_per] >= temp_bat_mv ) break;	
			}

			g_battery_stable_counter++;
			break;			
		
		case 3:
			bat_left_per = bat_percent;
			// Percentage Remaining based off Lookup Table Curve Fit	
			// percentage never goes up
			while( temp_bat_mv < battery_lut[bat_left_per] ) {
				if( --bat_left_per <= 0 ) {
					break; 
				}
			}		
			break;	
		
		default:
			bat_left_per = bat_percent = 100;		//reset bat_percent
			g_battery_stable_counter++;
			break;
	}
#endif
	
	//Force to Stay within Bounds!
	if ( bat_left_per > 100 ) bat_left_per = 100;
	if ( bat_left_per < 0 ) bat_left_per = 0;
	
	// check if charged
	
	
	// ok... figure out what the battery is doing
	if ( temp_bat_mv > CHARGING_VOLTAGE ) {
		charge_pin = nrf_gpio_pin_read(CHARGE_INPUT);	
		if (charge_pin == 1U) {
			temp_state = BATTERY_CHARGED;
		}
		else {
			temp_state = BATTERY_CHARGING;
		}
		*level = 100;		
		// hold off % measurement until not charging
		g_battery_stable_counter = 0;
	}
	else {	//if (current_state == BATTERY_DISCHARGING) {
		if ( bat_left_per < bat_percent) {
			//update calling functions battery level
			bat_percent = (uint8_t)bat_left_per;
			new_bat_percent = true;
		}
		*level = bat_percent;
		
		if( bat_left_per == 0 ) temp_state = BATTERY_LOW;
		else temp_state = BATTERY_DISCHARGING; // default state
	}	

	*state =  temp_state;
	current_state = temp_state;
	batvoltage_mv = temp_bat_mv;	//save result for external use
	
	battery_dav = false;
	if (battery_debug) {
		app_trace_log("bat_status_level: %01u cnt, %01u volts, %01u left, %01u state\r",(int)ad_cnt,(int)temp_bat_mv,(int)bat_left_per, (int) *state);
	}
}

void battery_init(app_timer_id_t p_bat_timer, bool debug)
{
	battery_debug = debug;
	
	DEBUG_PRINT(0,"bat_init: start",battery_debug);

	local_battery_timer_id = p_bat_timer;
	
	// init sampling event
    ret_code_t err_code;

	// setup input pins
	nrf_gpio_cfg_input( CHARGE_INPUT, NRF_GPIO_PIN_NOPULL );	
	//nrf_gpio_pin_set(BAT_HALF_ON);	
	nrf_gpio_pin_clear(BAT_HALF_ON);
	nrf_gpio_cfg_output(BAT_HALF_ON);

	// set up ppi
	err_code = nrf_drv_ppi_init();
	APP_ERROR_CHECK(err_code);	
	err_code = nrf_drv_ppi_channel_alloc(&bat_ppi_channel);
	APP_ERROR_CHECK(err_code);	
	nrf_drv_ppi_channel_enable(bat_ppi_channel);
	
	// init saadc
	uint8_t channel_id = 0;
	nrf_saadc_channel_config_t channel_config = SAADC_BATTERY_CHANNEL_CONFIG(NRF_SAADC_INPUT_AIN0);
	
	nrf_drv_saadc_config_t  saadc_config = SAADC_BATTERY_CONFIG;
	err_code = nrf_drv_saadc_init(&saadc_config, battery_callback);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_channel_init( channel_id, &channel_config );
	APP_ERROR_CHECK(err_code);
	
	NRF_SAADC->CH[channel_id].CONFIG |= (1<<24);	//enable burst mode

	err_code = nrf_drv_saadc_buffer_convert(&sample, 1);
	APP_ERROR_CHECK(err_code);	

	// previous battery reading
	current_state = BATTERY_DISCHARGING;
	
	// start battery monitoring timer
	battery_monitor_start();
	g_battery_stable_counter = 0;
	
	DEBUG_PRINT(0,"bat_init: done",battery_debug);	
}

void battery_stop(void)
{
	//battery_monitor_stop();
	nrf_drv_ppi_channel_free(bat_ppi_channel);
	nrf_drv_saadc_uninit();
	nrf_saadc_disable();
	app_timer_stop( local_battery_timer_id );
}


	

