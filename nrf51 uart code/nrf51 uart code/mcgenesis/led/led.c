/*
 * led.c
 *
 * Created: 08/25/2015 2:07:19 PM
 *  Author: richkl
 *  5-2-16 added LED_ID - MM
 */ 

#include "led.h"
#include "app_error_vsp.h"

//#define 	USEPWM					

#ifdef USEPWM
#include "app_pwm.h"
APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER2.

/**@brief PWM instance default configuration (2 channels). */
#define LED_PWM_CONFIG_2CH(period_in_us, pin0, pin1)                           \
    {                                                                                  \
        .pins            = {pin0, pin1},                                               \
        .pin_polarity    = {APP_PWM_POLARITY_ACTIVE_LOW, APP_PWM_POLARITY_ACTIVE_LOW}, \
        .num_of_channels = 1,                                                          \
        .period_us       = period_in_us                                                \
    }
		
static volatile bool g_ready_flag;            // A flag indicating PWM status.
		
#endif

//static volatile app_timer_id_t 		local_led_timer_id = NULL;
static volatile bool led_initialied = false;
static volatile bool led_debug;
static volatile bool g_red;
static volatile bool g_grn;
static volatile bool g_blu;
static volatile int g_on;
static volatile int g_off;
static volatile int g_ontime;
static volatile int g_offtime;
static bool light_show = false;

#ifdef USEPWM
void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    g_ready_flag = true;
}

void led_flash_timer_handler(void * p_context)
{
}
#endif

#ifndef USEPWM
void led_flash_timer_handler(void * p_context)
{
	extern uint8_t led_id_key_numb;
		
	if( light_show == true ) {
		static uint8_t led_state = 0;
		static int i, repeat = 0;
		static uint8_t r[8], g[8], b[8];
		
		g_on--;
		if( g_on == 0 ) {
			g_on = g_ontime;
			
			switch ( led_state ) { 
				case 0:
						// do the led color decode just one time.
						if( repeat == 0 ) { 
							for (i = 0; i <= 7; i++){
								if( i % 2 == 0 ){
									r[i] = 0; g[i] = 0; b[i] = 0;
								}
								else {
									if((led_id_key_numb >> (7-i) & 0x03) == 0x00){
										r[i] = 1; g[i] = 1; b[i] = 1;
									}
									if((led_id_key_numb >> (7-i) & 0x03) == 0x01){
										r[i] = 1; g[i] = 0; b[i] = 1;
									}
									if((led_id_key_numb >> (7-i) & 0x03) == 0x02){
										r[i] = 1; g[i] = 0; b[i] = 0;
									}
									if((led_id_key_numb >> (7-i) & 0x03) == 0x03){
										r[i] = 1; g[i] = 1; b[i] = 0;
									}
								}
							}
						}
						
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 8 ) {    
								repeat = 0;                        
								led_state = 1; 
						}
						break;
				
				case 1:
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 10 ) {    
								repeat = 0;    
								led_state = 2;        
						}
						break;
						
				case 2:
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 2 ) {    
								repeat = 0;                        
								led_state = 3;        
						}
						break;
				
				case 3:
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 10 ) {    
								repeat = 0;    
								led_state = 4;        
						}
						break;
						
				case 4:
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 2 ) {    
								repeat = 0;                        
								led_state = 5;        
						}
						break;
				
				case 5:
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 10 ) {    
								repeat = 0;    
								led_state = 6;        
						}
						break;
						
				case 6:
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 2 ) {    
								repeat = 0;                        
								led_state = 7;        
						}
						break;
				
				case 7:
						ledraw(r[led_state], g[led_state], b[led_state]);
						if( ++repeat >= 10 ) {    
								repeat = 0;    
								led_state = 0;        
						}
						break;
				
				default:
					led_state = 0;
					break;
			}
		}
	}
	else if ( g_off == -1) {
		// -1 for off time overrides on times and sets leds off
		DEBUG_PRINT(0,"led_flash_timer_handler: all off",led_debug);
		nrf_gpio_pin_clear(LED_ON);	
	}
	else if ( g_on == -1 ) { 
		// -1 for on times sets leds on
		DEBUG_PRINT(0,"led_flash_timer_handler: all on",led_debug);		
		nrf_gpio_pin_set(LED_ON);			
		led_control();
	}

	else {
		if ( g_on != 0 ) {
			DEBUG_PRINT(0,"led_flash_timer_handler: on",led_debug);	
			led_control();			
			nrf_gpio_pin_set(LED_ON);
			g_on--;
			if ( g_on == 0) {
				nrf_gpio_pin_clear(LED_ON);		
				g_off = g_offtime;
			}
		}
		else if ( g_off != 0) {
			DEBUG_PRINT(0,"led_flash_timer_handler: off",led_debug);				
			nrf_gpio_pin_clear(LED_ON);	
			g_off--;
			if (g_off == 0) {
				g_on = g_ontime;
			}
		}
	
	}
}

void ledraw(uint8_t red, uint8_t green, uint8_t blue)
{
	g_red = (bool)red;
	g_grn = (bool)green;
	g_blu = (bool)blue;
	nrf_gpio_pin_set(LED_ON);
	led_control();
}

void led_control(void)
{
	if ( !g_red && !g_grn && !g_blu ) 
	{
		nrf_gpio_pin_clear(LED_ON);		
	}
	else {
		nrf_gpio_pin_set(LED_ON);		
		if ( g_red ) {
			nrf_gpio_cfg_output(RED_LED);
			nrf_gpio_pin_clear(RED_LED);
		}
		else {
			nrf_gpio_cfg_input(RED_LED, NRF_GPIO_PIN_NOPULL);
			nrf_gpio_pin_set(RED_LED);
		}
		if ( g_grn) {
			nrf_gpio_cfg_output(GRN_LED);
			nrf_gpio_pin_clear(GRN_LED);
		}
		else {
			nrf_gpio_cfg_input(GRN_LED, NRF_GPIO_PIN_NOPULL);
			nrf_gpio_pin_set(GRN_LED);
		}
		if ( g_blu ) {
			nrf_gpio_cfg_output(BLU_LED);
			nrf_gpio_pin_clear(BLU_LED);
	#ifdef ALT_BLU_LED
			nrf_gpio_cfg_output(ALT_BLU_LED);
			nrf_gpio_pin_clear(ALT_BLU_LED);
	#endif
		}
		else {
			nrf_gpio_cfg_input(BLU_LED, NRF_GPIO_PIN_NOPULL);
			nrf_gpio_pin_set(BLU_LED);
	#ifdef ALT_BLU_LED
			nrf_gpio_cfg_input(ALT_BLU_LED, NRF_GPIO_PIN_NOPULL);
			nrf_gpio_pin_set(ALT_BLU_LED);
	#endif
		}	
	}
}

#endif

void led(uint8_t red, uint8_t green, uint8_t blue, int on, int off)
{
	if( light_show == true ) return;	//Light show has control. Back Off!!!
	
	g_red = (bool)red;
	g_grn = (bool)green;
	g_blu = (bool)blue;
	g_on = g_ontime = on;
	g_off = g_offtime = off;
}

//void led_init(app_timer_id_t p_bat_timer,Bool debug, Bool dotimer)
void led_init( bool debug )
{
	led_debug = debug;
	led_initialied = false;	
		
	DEBUG_PRINT(0,"led_init: hw start",led_debug);

	// setup output pins
	nrf_gpio_cfg_output(LED_ON);	
	nrf_gpio_pin_clear(LED_ON);	
	nrf_gpio_cfg_output(RED_LED);
	nrf_gpio_pin_set(RED_LED);
	nrf_gpio_cfg_output(GRN_LED);
	nrf_gpio_pin_set(GRN_LED);
	nrf_gpio_cfg_output(BLU_LED);
	nrf_gpio_pin_set(BLU_LED);
#ifdef ALT_BLU_LED
	nrf_gpio_cfg_output(ALT_BLU_LED);
	nrf_gpio_pin_set(ALT_BLU_LED);
#endif

#ifdef USEPWM
	// 2-channel PWM, 200Hz, output on DK LED pins. 
	app_pwm_config_t pwm1_cfg = LED_PWM_CONFIG_2CH(5000L, RED_LED, GRN_LED);
	
	/* Initialize and enable PWM. */
	err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
	APP_ERROR_CHECK(err_code);
	app_pwm_enable(&PWM1);
	
	//app_pwm_channel_duty_set(&PWM1, 0, 12000);
	//app_pwm_channel_duty_set(&PWM1, 1, 50);
	g_ready_flag = false;
	while(!g_ready_flag);
	app_pwm_channel_duty_set(&PWM1, 1, 50);
#endif
	
#ifndef USEPWM
	g_red = false;
	g_grn = false;
	g_blu = false;
	led_control();
#endif
	
	led_initialied = true;
	
	DEBUG_PRINT(0,"led_init: done",led_debug);	
}

bool led_start_flash_timer( app_timer_id_t p_timer )
{
	uint32_t err_code;
	
	if( light_show == true ) return false;	//Light show has control. Back Off!!!
	
	err_code = app_timer_start(p_timer, LED_FLASH_INTERVAL , NULL);		
	APP_ERROR_CHECK(err_code); 
	if( err_code != NRF_SUCCESS ) return false;
	
	return true;
}

bool led_stop_flash_timer( app_timer_id_t p_timer )
{
	uint32_t err_code;
	
	if( light_show == true ) return false;	//Light show has control. Back Off!!!
	
	err_code = app_timer_stop(p_timer);		
	APP_ERROR_CHECK(err_code); 
	if( err_code != NRF_SUCCESS ) return false;
	
	return true;
}
	
void led_light_show( bool on_off, app_timer_id_t p_timer )
{
	if( on_off == true ) {
		led(0,0,0,4,0);
		led_start_flash_timer( p_timer );
		light_show = true;
	}
	else {
		light_show = false;
		ledraw(0,0,0);
	}
}


