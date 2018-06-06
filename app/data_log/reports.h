/*
 * reports.h
 *
 * Created: 3/2/2015 3:12:31 PM
 *  Author: matt
 */ 


#ifndef REPORTS_H_
#define REPORTS_H_

#include "mem_manager.h"

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
	ON_CHANGE,

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
	T_INT24,
	
	DATA_TYPE_LEN,	//Last Element gives length of enum
} T_DATA_TYPE;

#pragma anon_unions

typedef enum {
	_1_1BIT = 0,
	_10_1BIT,
	_100_1BIT,
	_1G,
	_2G,
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

	
	DATA_FSR_LEN,	//Last Element gives length of 
} T_DATA_FSR;

#define MOTION_MASK 0x0F
typedef __packed union {
	 __packed struct {
		bool x:			1;
		bool y:			1;
		bool z:			1;
		bool magnitude:	1;
		bool nu_1:		1;
		bool nu_2:		1;
		bool nu_3:		1;
		bool nu_4:		1;
	 };
	 uint8_t flags;
} T_DATA_FIELDS;

typedef struct {
	uint8_t atts_valid;			//use to flag when a set of attributes is valid and able to be enabled
	uint8_t enabled;
	uint16_t rec_data_len;		//current length of the record
	uint8_t sample_size;		//number of bytes per sample
	uint16_t rec_full_len;		//number of bytes when record needs to be stored
	uint16_t rep_len;			//number of unsent records in the report
	uint16_t timer_period;		//the time between Record Updates
	int16_t time_remain;		//the time left until the next Record Update
	uint16_t dep_scale;
	uint16_t data_fifo_tail;	//use to point to any data FIFOs
	uint8_t new_start;
	uint8_t rec_saved;			//flag that record was saved, a new one should be started
	uint8_t ack_require;		//True if an ACK is required from host on this Record type, otherwise false
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

void init_reporters( void );
bool rec_ack_req( uint8_t id );
void reporter_time_rebase( uint32_t, uint32_t );
uint16_t active_data_types( void );
T_REPORT_ERR set_reporter(  T_REPORT_ATTRIBUTES *, uint8_t );
void  enable_reporters( uint8_t );
uint8_t get_active_reporters( void );
void check_reporters( void );
void power_down_save_records( void );
void erase_reporter_config( void );
void flag_raw_data_sync( void );
void inc_report_length( uint8_t );
void dec_report_length( uint8_t );
uint16_t get_report_record_cnt( uint8_t );
uint32_t get_total_record_cnt( void );

#endif /* REPORTS_H_ */
