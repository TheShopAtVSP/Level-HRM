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
#include "manufacture_test.h"

#define BATLEVEL_100				4050		// Full battery in terms of milliVolts: 4.0 volts
#define BATLEVEL_05					3500		// Battery Curve Knee starts around this point
#define BATLEVEL_0					3050		// Empty battery: 3.05V * MATH_SCALE		

#define ADC_BIT_RES					14
#define ADC_CONFIG_RES				NRF_SAADC_RESOLUTION_14BIT
#define ADC_SCALE					4
#define ADC_RES						(1<<ADC_BIT_RES)
#define RESISTOR_DIVIDER_SCALE		3
#define ADC_VREF					600			// 0.60V * MATH_SCALE
#define BAT_RANGE					(BATLEVEL_100 - BATLEVEL_0)
#define DISCHARGE_SLOPE_1			(BATLEVEL_100 - BATLEVEL_05)
#define DISCHARGE_SLOPE_2			(BATLEVEL_05 - BATLEVEL_0)

#if defined LEVEL_1_0
	#define LTC4070
	#define CHARGE_PG_ACTIVE		1			//Charge Pin active high
	#define RES_DIV_TOP				100
	#define RES_DIV_BOT				100
#else
	#define BQ25120
	#define CHARGE_PG_ACTIVE		0			//PG Pin active low
	#define RES_DIV_TOP				499
	#define RES_DIV_BOT				300
#endif

//At 2 percent remaining, go into hibernate mode. This should keep the processor from resetting for about ~1 more day
#define LOW_BAT_THRESH				2
//Can not ship a battery by Air that is over 30% charged. So when in shipmode, bleed battery to less than 30% charge
#define SHIP_MODE_THRESH			30

typedef enum
{
	BATTERY_LOW =					0x00,
	BATTERY_CHARGING =				0x01,
	BATTERY_DISCHARGING =			0x02,
	BATTERY_CHARGED =				0x04,
	BATTERY_MONITOR_FAIL =			0x05,
	BATTERY_UNDEF_STATE =			0x0A,
} T_BATTERY_STATE;

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

ret_code_t battery_init( bool debug );
void battery_report( uint8_t * per_rem, uint8_t * state);
bool battery_shutoff( void );
bool battery_chr_present( void );
bool battery_isdav(void);
void battery_monitor_shutdown(void);
T_BATTERY_STATE battery_get_current_state(void);
uint16_t battery_get_voltage(void);
uint8_t battery_get_level( void );
bool battery_level_change( uint8_t * remain );
T_HW_FAILURE_FLAGS battery_self_test( bool test4chr );

#endif /* BATTERY_H_ */
