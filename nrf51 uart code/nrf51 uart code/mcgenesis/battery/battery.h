/*
 * battery.h
 *
 * Created: 8/1/2015 2:07:31 PM
 *  Author: richkl
 */ 


#ifndef BATTERY_H_
#define BATTERY_H_

#include "../global.h"
#include "nrf_drv_saadc.h"
#include "app_timer.h"
#include "nrf_drv_config.h"

// hardware pins
#define CHARGE_INPUT				9
#define BAT_HALF_ON					10

#define CHR_INTERVAL				1000
#define RUN_INTERVAL				1000
#define LOW_INTERVAL				5000

// battery defs
#define BATTERY_MEASURE_INTERVAL_CHARGE	(APP_TIMER_TICKS(CHR_INTERVAL, APP_TIMER_PRESCALER))
#define BATTERY_MEASURE_INTERVAL_RUN	(APP_TIMER_TICKS(RUN_INTERVAL, APP_TIMER_PRESCALER))
#define BATTERY_MEASURE_INTERVAL_LOW	(APP_TIMER_TICKS(LOW_INTERVAL, APP_TIMER_PRESCALER))
#if ( WDT_CONFIG_RELOAD_VALUE <= LOW_INTERVAL )
 //This event also makes sure the dog gets fed in hibernate mode
 #error "WDT Period must be greater than Low Power Battery measure interval"
#endif

#define BATLEVEL_100				4050		// Full battery in terms of milliVolts: 4.0 volts
#define BATLEVEL_05					3500		// Battery Curve Knee starts around this point
#define BATLEVEL_0					3050		// Empty battery: 3.05V * MATH_SCALE		

#define ADC_BIT_RES					14
#define ADC_CONFIG_RES				NRF_SAADC_RESOLUTION_14BIT
#define ADC_RES						(1<<ADC_BIT_RES)
#define ADC_SCALE					4
#define RESISTOR_DIVIDER_SCALE		2
#define ADC_VREF					600			// 0.60V * MATH_SCALE
#define BAT_RANGE					(BATLEVEL_100 - BATLEVEL_0)
#define DISCHARGE_SLOPE_1			(BATLEVEL_100 - BATLEVEL_05)
#define DISCHARGE_SLOPE_2			(BATLEVEL_05 - BATLEVEL_0)

#define BATTERY_LOW					0x00
#define BATTERY_CHARGING			0x01
#define BATTERY_DISCHARGING			0x02
#define BATTERY_CHARGED				0x04

#define SAADC_BATTERY_CHANNEL_CONFIG(PIN_P) \
{                                                      \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,         \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,         \
    .gain       = NRF_SAADC_GAIN1_4,				   \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,        \
    .acq_time   = NRF_SAADC_ACQTIME_10US,              \
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,         \
    .pin_p      = (nrf_saadc_input_t)(PIN_P),          \
    .pin_n      = NRF_SAADC_INPUT_DISABLED             \
}

#define SAADC_BATTERY_CONFIG                \
{                                                   \
    .resolution         = ADC_CONFIG_RES,  \
    .oversample         = NRF_SAADC_OVERSAMPLE_32X,	\
    .interrupt_priority = 3 \
}

void battery_init(app_timer_id_t p_bat_timer,Bool debug);
void battery_status_level(uint8_t *level,uint8_t *state);
void battery_convert(void);
bool battery_isdav(void);
void battery_monitor_start( void );
void battery_monitor_stop( void );
void battery_monitor_interval( uint );
void battery_callback(nrf_drv_saadc_evt_t const * p_event);
void battery_measurement_timer_handler(void * p_context);
void battery_stop(void);
uint8_t battery_get_current_state(void);
uint16_t battery_get_voltage(void);
uint8_t battery_get_level( void );
bool battery_level_change( uint8_t * remain );

#endif /* BATTERY_H_ */
