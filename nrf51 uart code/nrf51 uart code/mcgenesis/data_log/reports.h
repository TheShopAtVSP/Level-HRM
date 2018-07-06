/*
 * reports.h
 *
 * Created: 3/2/2015 3:12:31 PM
 *  Author: matt
 */ 


#ifndef REPORTS_H_
#define REPORTS_H_

#include <stdint.h>
#include "../global.h"

typedef enum {
	NO_ERROR = 0,
	REPORT_ON_ERROR,
	INST_ERROR,
	DATA_TYPE_ERROR,
	REPORT_NOT_EMPTY,
}T_REPORT_ERR;

//the various types of records that can be logged
typedef enum {
	NO_TYPE = 0,
	ASCII,
	STEPS_TIME,
	ACCEL_RAW,
	GYRO_RAW,
	MAGNETO_RAW,
	ACCEL_FILT,
	GYRO_FILT,
	MAGNETO_FILT,
	ACCEL_VARIANCE,
	GYRO_VARIANCE,
	MAGNETO_VARIANCE,
	BATTERY_REMAINING,
	CYCLES_PER_TIME,
	TILT_ANGLE,
	BOARD_TEMP,
	EXPERIMENTAL,

	DATA_DESC_LEN,	//Last Element gives length of 
} T_DATA_DESC;

typedef enum {
	UNITLESS = 0,
	SECONDS = 1,
	MINUTES,
	SAMPLING_HZ,

	INDEP_DESC_LEN,	//Last Element gives length of 
} T_INDEP_DESC;

//use to quickly get the byte count of the dependent data type
typedef enum {
	T_CHAR = 0,
	T_UINT8,
	T_INT8,
	T_UINT16,
	T_INT16,
	T_UINT32,
	T_INT32,

	DATA_TYPE_LEN,	//Last Element gives length of enum
} T_DATA_TYPE;

#pragma anon_unions

typedef enum {
	_1_1BIT = 0,
	_10_1BIT,
	_100_1BIT,
	_4G,
	_8G,
	_16G,
	_500DPS,
	_1000DPS,
	_2000DPS,
	_4GAUSS,
	_8GAUSS,
	_12GAUSS,
	_16GAUSS,
	_48GAUSS,
	_16G_VAR,
	_64G_VAR,
	_256G_VAR,
	
	DATA_FSR_LEN,	//Last Element gives length of 
} T_DATA_FSR;

#define MOTION_MASK 0x0F
typedef __packed union {
	 __packed struct {
		Bool x:			1;
		Bool y:			1;
		Bool z:			1;
		Bool magnitude:	1;
		Bool nu_1:		1;
		Bool nu_2:		1;
		Bool nu_3:		1;
		Bool nu_4:		1;
	 };
	 uint8_t flags;
} T_DATA_FIELDS;

typedef struct {
	uint8_t atts_valid;		//use to flag when a set of attributes is valid and able to be enabled
	uint8_t enabled;
	uint16_t rec_len;		//current length of the record
	uint8_t sample_size;	//number of bytes per sample
	uint16_t rec_full_len;	//number of bytes when record needs to be stored
	uint16_t rep_len;		//number of unsent records in the report
	uint16_t timer_period;	//the time between Record Updates
	int16_t time_remain;	//the time left until the next Record Update
	uint16_t dep_scale;
	uint16_t data_fifo_tail;	//use to point to any data FIFOs
	uint8_t new_start;
	uint8_t decimator_cnt;		//the number of sumples to pass through the decimator
} T_REPORT_STATUS;

#pragma anon_unions

typedef __packed struct {
	uint8_t inst;
	T_INDEP_DESC independent_data:	8;
	uint8_t indep_scalar;
	T_DATA_DESC dependent_data:		8;
	T_DATA_TYPE dep_var_type:		8;
	T_DATA_FSR dep_scale:			8;
	T_DATA_FIELDS data_fields;		//8;
	uint16_t samples_rec;
	uint16_t records_rep;
}  T_REPORT_ATTRIBUTES;
#define ATTRIBUTE_SIZE sizeof(T_REPORT_ATTRIBUTES)

#define REP_CNT		8		//The number of Reporters that can be used

void init_reporters( void );
void reporter_time_rebase( uint32_t, uint32_t );
uint16_t active_data_types( void );
T_REPORT_ERR set_reporter(  T_REPORT_ATTRIBUTES *, uint8_t );
void  enable_reporters( uint8_t );
uint8_t get_active_reporters( void );
void check_reporters( void );
void inc_report_length( uint8_t );
void dec_report_length( uint8_t );
uint16_t get_report_record_cnt( uint8_t );
uint16_t get_total_record_cnt( void );

#endif /* REPORTS_H_ */
