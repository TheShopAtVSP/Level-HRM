/*
 * battery.h
 *
 * Created: 10/10/2014 2:07:31 PM
 *  Author: richkl
 */ 


#ifndef BATTERY_H_
#define BATTERY_H_

// battery defs
#define BATLEVEL_100		4120		//Full battery in terms of Voltage: 4.1V * MATH_SCALE
#define BATLEVEL_0			3400		//Empty battery: 3.4V * MATH_SCALE		
#define MATH_SCALE			1000
#define ADC_RES				4096
#define ADC_SCALE			2
#define OP_AMP_SCALE		2
#define VREF_ADC			2200/ADC_SCALE
#define BAT_RANGE			(BATLEVEL_100 - BATLEVEL_0)
#define AD_100_LEVEL		ADC_RES*BATLEVEL_100/(VREF_ADC*ADC_SCALE*OP_AMP_SCALE)
#define AD_0_LEVEL			ADC_RES*BATLEVEL_0/(VREF_ADC*ADC_SCALE*OP_AMP_SCALE)

#define BATTERY_CHARGING	0x01
#define BATTERY_DISCHARGING	0x02
#define BATTERY_CHARGED		0x04

void battery_init(bool debug);
void battery_status_level(uint8_t *level,uint8_t *state);
void battery_convert(void);
Bool battery_isdav(void);
Bool battery_busy( void );
status_code_t battery_monitor_enable( void );
void battery_monitor_disable( void );

#endif /* BATTERY_H_ */