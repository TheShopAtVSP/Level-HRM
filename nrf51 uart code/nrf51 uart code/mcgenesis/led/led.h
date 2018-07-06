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

#define LED_FLASH_INTERVAL			(APP_TIMER_TICKS(50, APP_TIMER_PRESCALER))

//
// leds
//
#define GRN_LED				20
#define RED_LED				19
#define BLU_LED				17
#define ALT_BLU_LED			18		//The pin for the Blue Led on the Dev board
#define LED_ON				11



void led_init( bool debug );
void led_flash_timer_handler(void * p_context);
void led(uint8_t red, uint8_t green, uint8_t blue, int on, int off);
void led_control(void);
void ledraw(uint8_t red, uint8_t green, uint8_t blue);
bool led_start_flash_timer( app_timer_id_t );
bool led_stop_flash_timer( app_timer_id_t );
void led_light_show( bool on_off, app_timer_id_t p_timer );
	
#endif /* BATTERY_H_ */
