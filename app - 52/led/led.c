/*
 * led.c
 *
 * Created: 08/25/2015 2:07:19 PM
 *  Author: richkl
 *  5-2-16 added LED_ID - MM
 */

#include "led.h"
#include "app_error_vsp.h"

//static volatile app_timer_id_t 		local_led_timer_id = NULL;
typedef struct {
	bool red;
	bool grn;
	bool blu;
	int on_time;
	int off_time;
	int cycles;
} T_LED_SET;
static T_LED_SET led_setting[2];
static uint8_t pri_lvl = 0;

static volatile bool led_initialied = false;
static volatile bool led_debug;
static volatile int g_on;
static volatile int g_off;
uint8_t led_id_key_numb = 0;
uint8_t led_id_shifter = 0;

#define LED_FLASH_PERIOD_MS		50
#define LED_FLASH_INTERVAL		(APP_TIMER_TICKS(LED_FLASH_PERIOD_MS, APP_TIMER_PRESCALER))
static app_timer_id_t local_led_timer_id = NULL;

static void led_control(void);
static bool led_start_flash_timer( void );
static bool led_stop_flash_timer( void );

//#define 	USEPWM
#ifdef USEPWM
#include "nrf_drv_pwm.h"

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
static nrf_drv_pwm_t m_pwm2 = NRF_DRV_PWM_INSTANCE(2);

static void init_pwm(void)
{
    uint32_t err_code;
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            GRN_LED | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            RED_LED | NRF_DRV_PWM_PIN_INVERTED, // channel 1
            BLU_LED | NRF_DRV_PWM_PIN_INVERTED, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED			
        },
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_demo1_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, demo1_handler);
    APP_ERROR_CHECK(err_code);

    m_demo1_seq_values.channel_0 = 0;
    m_demo1_seq_values.channel_1 = 0;
    m_demo1_seq_values.channel_2 = 0;
    m_demo1_seq_values.channel_3 = 0;
    m_demo1_phase = 0;

    nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1,
        NRF_DRV_PWM_FLAG_LOOP);
}
#endif

void led_init( bool debug, app_timer_id_t p_timer )
{
	led_debug = debug;
	led_initialied = false;
	
	local_led_timer_id = p_timer;

	app_trace_puts(DEBUG_LOW, "led_init: hw start\r\n");

	// setup output pins so LEDS are Off
	nrf_gpio_cfg_output(LED_ON);
	nrf_gpio_pin_clear(LED_ON);
	nrf_gpio_cfg_output(RED_LED);
	nrf_gpio_pin_set(RED_LED);
	nrf_gpio_cfg_output(GRN_LED);
	nrf_gpio_pin_set(GRN_LED);
	nrf_gpio_cfg_output(BLU_LED);
	nrf_gpio_pin_set(BLU_LED);

#ifdef USEPWM
	/* Initialize and enable PWM. */
#endif

	pri_lvl = 0;					//Start all settings in Off state
	led_setting[0].red = false;
	led_setting[0].grn = false;
	led_setting[0].blu = false;
	led_setting[0].on_time = 0;
	led_setting[0].off_time = -1;
	led_setting[0].cycles = -1;
	g_on = 0;
	g_off = -1;

	led_initialied = true;

	app_trace_puts(DEBUG_LOW, "led_init: done\r\n");
}

void led_uninit( void )
{
	led_stop_flash_timer();			//make sure timer is Off
	
	nrf_gpio_cfg_output(LED_ON);	//make sure LEDs are all Off
	nrf_gpio_pin_clear(LED_ON);
	nrf_gpio_cfg_output(RED_LED);
	nrf_gpio_pin_set(RED_LED);
	nrf_gpio_cfg_output(GRN_LED);
	nrf_gpio_pin_set(GRN_LED);
	nrf_gpio_cfg_output(BLU_LED);
	nrf_gpio_pin_set(BLU_LED);
	
	pri_lvl = 0;					//Reset all settings to Off
	led_setting[0].red = false;
	led_setting[0].grn = false;
	led_setting[0].blu = false;
	led_setting[0].on_time = 0;
	led_setting[0].off_time = -1;
	led_setting[0].cycles = -1;
	g_on = 0;
	g_off = -1;
}

void led_flash_timer_handler(void * p_context)
{	
	if( led_setting[pri_lvl].on_time < 0 || led_setting[pri_lvl].off_time < 0 )
	{	//Timer doesn't need to be On, LED is in a fixed State
		pri_lvl = 0;	//if timer is going Off, priority had better be lowest
		if ( led_setting[pri_lvl].off_time == -1)
		{
			// -1 for off time overrides on times and sets leds off
			app_trace_puts(DEBUG_LOW, "led_flash_timer_handler: stay off\r\n");
			nrf_gpio_pin_clear(LED_ON);
			led_stop_flash_timer();
		}
		else if ( led_setting[pri_lvl].on_time == -1 )
		{
			// -1 for on times sets leds on
			app_trace_puts(DEBUG_LOW, "led_flash_timer_handler: stay on\r\n");
			nrf_gpio_pin_set(LED_ON);
			led_control();
			led_stop_flash_timer();
		}
	}
	else
	{
		if ( g_on != 0 )
		{
			//app_trace_puts(DEBUG_LOW, "led_flash_timer_handler: on\r\n");
			led_control();
			nrf_gpio_pin_set(LED_ON);
			
			if( --g_on <= 0 )
			{	
				g_off = led_setting[pri_lvl].off_time;
			}
		}
		else if ( g_off != 0)
		{
			//app_trace_puts(DEBUG_LOW, "led_flash_timer_handler: off\r\n");
			nrf_gpio_pin_clear(LED_ON);
			
			if( --g_off <= 0 )
			{
				if( pri_lvl > 0 )
				{	
					if( --led_setting[pri_lvl].cycles <= 0 )
					{	//revert to lower priority settings
						pri_lvl = 0;	
					}
				}
				
				g_on = led_setting[pri_lvl].on_time;
			}
		}
	}
}

static void led_control(void)
{
	
	if ( !led_setting[pri_lvl].red && !led_setting[pri_lvl].grn && !led_setting[pri_lvl].blu )
	{
		nrf_gpio_pin_clear(LED_ON);
	}
	else
	{
		nrf_gpio_pin_set(LED_ON);
		if ( led_setting[pri_lvl].red )
		{
			nrf_gpio_cfg_output(RED_LED);
			nrf_gpio_pin_clear(RED_LED);
		}
		else
		{
			nrf_gpio_cfg_input(RED_LED, NRF_GPIO_PIN_NOPULL);
			nrf_gpio_pin_set(RED_LED);
		}
		if ( led_setting[pri_lvl].grn )
		{
			nrf_gpio_cfg_output(GRN_LED);
			nrf_gpio_pin_clear(GRN_LED);
		}
		else
		{
			nrf_gpio_cfg_input(GRN_LED, NRF_GPIO_PIN_NOPULL);
			nrf_gpio_pin_set(GRN_LED);
		}
		if ( led_setting[pri_lvl].blu )
		{
			nrf_gpio_cfg_output(BLU_LED);
			nrf_gpio_pin_clear(BLU_LED);
		}
		else
		{
			nrf_gpio_cfg_input(BLU_LED, NRF_GPIO_PIN_NOPULL);
			nrf_gpio_pin_set(BLU_LED);
		}
	}
}

//Sets the lowest priority settings only
void led(uint8_t red, uint8_t green, uint8_t blue, int on, int off)
{
	led_setting[0].red = (bool)red;
	led_setting[0].grn = (bool)green;
	led_setting[0].blu = (bool)blue;
	led_setting[0].on_time = on;
	led_setting[0].off_time = off;
	
	if( pri_lvl == 0 )
	{
		g_on = on;
		g_off = off;
		led_control();
	}
	
	led_start_flash_timer();
}

//Allow Led control to be usurped for a finite period:
bool led_over_ride( uint16_t on_ms, uint16_t off_ms, uint16_t num_cycles )
{
	uint32_t time_on = (on_ms / LED_FLASH_PERIOD_MS);	
	uint32_t time_off = (off_ms / LED_FLASH_PERIOD_MS);	
	
	//set higher priority led settings:
	pri_lvl = 1;
	led_setting[pri_lvl].red = true;
	led_setting[pri_lvl].grn = true;
	led_setting[pri_lvl].blu = true;
	g_on = led_setting[pri_lvl].on_time = time_on;
	g_off = led_setting[pri_lvl].off_time = time_off;
	led_setting[pri_lvl].cycles = num_cycles;
	
	led_start_flash_timer();
	
	return true;
}

static bool led_start_flash_timer( void )
{
	uint32_t err_code;

	if( app_timer_is_valid(local_led_timer_id) )
	{
		err_code = app_timer_start( local_led_timer_id, LED_FLASH_INTERVAL, NULL );
		if( err_code != NRF_SUCCESS ) 
		{	//Timer failed to start
			app_trace_log(DEBUG_HIGH, "[LED_TMR] Start Failed @%01u\r\r\n", getSystemTimeMs());
			return false;
		}
		else
		{
			app_trace_puts(DEBUG_LOW, "[LED_TMR] On\r\n");
		}
	}

	return true;
}

static bool led_stop_flash_timer( void )
{
	uint32_t err_code;

	if( app_timer_is_valid(local_led_timer_id) )
	{
		err_code = app_timer_stop( local_led_timer_id );
		if( err_code != NRF_SUCCESS ) 
		{	//Timer failed to stop
			app_trace_log(DEBUG_HIGH, "[LED_TMR] Stop Failed @%01u\r\r\n", getSystemTimeMs());
			return false;
		}
		else
		{
			app_trace_puts(DEBUG_LOW, "[LED_TMR] Off\r\n");
		}
	}

	return true;
}
