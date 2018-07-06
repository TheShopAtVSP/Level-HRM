/*
 * battery.c
 *
 * Created: 10/10/2014 2:07:19 PM
 *  Author: richkl
 */ 

#include "asf.h"
#include "battery.h"
#include "stdbool.h"

static struct adc_dev_inst g_adc_inst;
Bool battery_debug;
static uint8_t prev_level;
static Bool battery_dav;
static Bool conv_in_progress;

//filter design from: http://www-users.cs.york.ac.uk/~fisher/mkfilter/racos.html
//Sampling freq = 0.1Hz, cutoff = 0.01 Hz,
#define NZEROS		8
#define AD_BUF_LEN	NZEROS+1
#define GAIN		5396
static uint32_t xcoeffs[] = { 171, 422, 695, 914, 992, 914, 695, 422, 171 };
static uint16_t adc_sample_buf[AD_BUF_LEN];
static uint8_t adc_sample_ptr;

const uint16_t battery_lut[] =
{
	0000,3400,3472,3511,3546,3576,3602,3625,3644,3661,3675,3687,3697,3705,3712,3717,3722,3726,3728,3731,
	3733,3735,3736,3738,3739,3740,3742,3744,3746,3748,3750,3753,3756,3759,3763,3767,3771,3776,3780,3785,
	3791,3796,3802,3808,3814,3820,3827,3833,3840,3846,3853,3860,3866,3873,3880,3886,3893,3899,3906,3912,
	3919,3925,3931,3937,3944,3950,3956,3962,3968,3974,3980,3986,3992,3998,4004,4010,4016,4022,4028,4034,
	4040,4047,4053,4059,4065,4071,4077,4083,4089,4094,4097,4099,4101,4103,4105,4107,4109,4111,4114,4116,
	4118,65535,
};

Bool battery_isdav(void)
{
	return(battery_dav);
}

Bool battery_busy( void )
{
	return conv_in_progress;
}

status_code_t battery_monitor_enable( void )
{
	//Wait for Startup Timer
	//for(int i=0; i<50; i++);
	
	conv_in_progress = false;
		
	return ( adc_enable(&g_adc_inst) );
	//adc_enable_interrupt(&g_adc_inst, ADC_SEQ_SEOC);	
}

void battery_monitor_disable( void )
{
	adc_disable(&g_adc_inst);
	//adc_disable_interrupt(&g_adc_inst, ADC_SEQ_SEOC);
	
	//wait for adc to turn off
	while((adc_get_status(&g_adc_inst) & ADCIFE_SR_EN)) {
		asm volatile ("nop");
	}
}

static void battery_read_conv_result(void)
{
	// Check the ADC conversion status
    if ((adc_get_status(&g_adc_inst) & ADCIFE_SR_SEOC) == ADCIFE_SR_SEOC) {
		adc_sample_buf[adc_sample_ptr++] = adc_get_last_conv_value(&g_adc_inst);
		if( adc_sample_ptr >= AD_BUF_LEN ) adc_sample_ptr = 0; 
		adc_clear_status(&g_adc_inst, ADCIFE_SCR_SEOC);
		// indicate that battery data is available
		battery_dav = true;
	}	
	conv_in_progress = false;
}

// battery subsystem data
void battery_status_level(uint8_t *level,uint8_t *state)
{
	//TODO get data
	//*level = 50;
	int32_t itemp;
	uint32_t temp;
	uint16_t batvoltage_mV;
	int16_t bat_left_per;
	uint16_t filter_bat = 0;
	
	// get status
	ioport_get_pin_level(PIN_PA10);
	if ( ioport_get_pin_level(CHARGE_IN_PIN) ) {
		// still low? charging
		if (battery_debug) puts("battery_status_level: BATTERY_CHARGED\r");
		*state = BATTERY_CHARGING;		
	}
	//else if ( ioport_get_pin_level(BAT_LOW_PIN) ) {
		//if (battery_debug) puts("battery_status_level: BATTERY_LOW\r");
		//*state = BATTERY_DISCHARGING;	
	//}
	else {
		if (battery_debug) puts("battery_status_level: BATTERY_DISCHARGING\r");
		*state = BATTERY_DISCHARGING;
	}
	
	
	// battery level
	temp = 0;
	for(int i=0; i<AD_BUF_LEN; i++) 
	{
		temp += (xcoeffs[i] * adc_sample_buf[(adc_sample_ptr+i)%AD_BUF_LEN]);
	}
	filter_bat = temp/GAIN;

	temp = (uint32_t) filter_bat * VREF_ADC * ADC_SCALE * OP_AMP_SCALE;
	batvoltage_mV = temp/ADC_RES;
	
#define LINEAR_BAT
#ifdef LINEAR_BAT
	//Percentage Remaining based off Linearization of Battery Curve
	itemp = (batvoltage_mV - BATLEVEL_0) * 100;
	bat_left_per = itemp/BAT_RANGE;
	if ( bat_left_per > 100 ) bat_left_per = 100;
	if ( bat_left_per < 0 ) bat_left_per = 0;
#else
	//Percentage Remaining based off Lookup Table Curve Fit
	//bat_left_per = prev_level;
	//while( battery_lut[bat_left_per] > batvoltage_mV ) {
		//bat_left_per--;
	//}
	//while( battery_lut[bat_left_per+1] < batvoltage_mV ) {
		//bat_left_per++;
	//}
#endif
	
	*level = (uint8_t)bat_left_per;
	
	// dont bounce level 
	// this prevents the current reading from being greater/less than the previous if charging/discharging.
	// this must be disabled if testing with a p/s substitution of the battery.
#if 1
	if ( *state == BATTERY_DISCHARGING ) {
		if ( *level < prev_level ) prev_level = *level;				//if new reading has decreased, use it
		else if( *level > (prev_level + 25) ) prev_level = *level;	//new reading has increased significantly, a power supply is the only explanation... keep it
		else *level = prev_level;
	}
	else {
		if ( *level > prev_level) prev_level = *level;
		else *level = prev_level;		
	}
#endif
	
	battery_dav = false;
	if (battery_debug) printf("battery_status_level: %i counts %i volts %i left\n\r",(int)filter_bat,(int)batvoltage_mV,(int)bat_left_per);
}

void battery_convert(void)
{
	if( battery_monitor_enable() == STATUS_OK ) {
		battery_dav = false;
		conv_in_progress = true;
		//asm volatile ("nop");	//break point		
		adc_start_software_conversion( &g_adc_inst );
			
		//asm volatile ("nop");	//break point
	}
	else {
		conv_in_progress = false;
		if (battery_debug) puts("ADC error\r");
	}
}


void battery_init(bool debug)
{
	battery_debug = debug;
	
	if (battery_debug) puts("battery_init: start\r");
	
	// setup adc
	struct adc_config adc_cfg = {
		/* System clock division factor is 16 */
		.prescal = ADC_PRESCAL_DIV16,
		/* The APB clock is used */
		.clksel = ADC_CLKSEL_APBCLK,
		/* Max speed is 150K */
		.speed = ADC_SPEED_150K,
		/* ADC Reference voltage is VCC/2 */
		.refsel = ADC_REFSEL_VCC_1_2,
		/* Enables the Startup time */
		.start_up = CONFIG_ADC_STARTUP
	};	
	adc_init(&g_adc_inst, ADCIFE, &adc_cfg);
	adc_enable(&g_adc_inst);
	
	struct adc_seq_config adc_seq_cfg = {
		/* Select Vref for shift cycle */
		.zoomrange = ADC_ZOOMRANGE_0,
		/* Pad Ground */
		.muxneg = ADC_MUXNEG_7,
		/* DAC internal */
		.muxpos = BAT_ADC_MUXPOS,
		/* Enables the internal voltage sources */
		.internal = ADC_INTERNAL_2,
		/* Disables the ADC gain error reduction */
		.gcomp = ADC_GCOMP_DIS,
		/* Disables the HWLA mode */
		.hwla = ADC_HWLA_DIS,
		/* 12-bits resolution */
		.res = ADC_RES_12_BIT,
		/* Enables the single-ended mode */
		.bipolar = ADC_BIPOLAR_SINGLEENDED,
		// gain
		.gain = ADC_GAIN_HALF
	};
		
	struct adc_ch_config adc_ch_cfg = {
		.seq_cfg = &adc_seq_cfg,
		/* Internal Timer Max Counter */
		.internal_timer_max_count = 60,
		/* Window monitor mode is off */
		.window_mode = 0,
		.low_threshold = 0,
		.high_threshold = 0,
	};	
	adc_ch_set_config(&g_adc_inst, &adc_ch_cfg);
	
	adc_set_callback(&g_adc_inst, ADC_SEQ_SEOC, battery_read_conv_result, ADCIFE_IRQn, 1);
	
	prev_level = 100;
	battery_dav = false;
	
	//load the adc result buffer
	adc_sample_ptr = 0;
	
	if (battery_debug) puts("battery_init: done\r");
		
}

