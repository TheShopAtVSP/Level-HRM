/*
 * battery.h
 *
 * Created: 08/25/2015 2:07:19 PM
 *  Author: richkl
 */


#ifndef LED_H_
#define LED_H_

#include "../global.h"
#include "app_timer.h"
#include "nrf_drv_config.h"
#include "nrf_gpio.h"

//
// leds
//
#if defined LEVEL_1_0
#define GRN_LED		20
#define RED_LED		19
#define BLU_LED		17
#define LED_ON		11
#else
#define GRN_LED		19
#define RED_LED		19
#define BLU_LED		19
#define LED_ON		20
#endif


void led_init( bool debug, app_timer_id_t p_timer );
void led_uninit( void );
void led_flash_timer_handler(void * p_context);
void led(uint8_t red, uint8_t green, uint8_t blue, int on, int off);
bool led_over_ride( uint16_t on_ms, uint16_t off_ms, uint16_t num_cycles );
	
#endif /* BATTERY_H_ */
