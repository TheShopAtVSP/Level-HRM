/*
 * reports.c
 *
 * Created: 3/2/2015 3:27:05 PM
 *  Author: matt
 */ 

#include "reports.h"
#include "../motion/step_detect.h"
#include "battery.h"
//#include "hal_imu.h"

#define RIDICULOUSLY_OLD	(3600*24L)		//1 day

extern T_CONFIG g_config;
extern bool gs_bdebug;
extern uint32_t get_unix_time( void ); 
extern T_3AXIS_FIFO accel_buf;
extern T_3AXIS_FIFO gyro_buf;
extern T_3AXIS_FIFO magnet_buf;
extern TMOTION Motion;

typedef struct {
	T_REPORT_ATTRIBUTES atts;
	T_REPORT_STATUS stat;
	T_RECORD rec;
}T_REPORTER;

typedef struct {
	uint32_t time_stamp;
	uint16_t cnt;
}T_STEP_ELEMENT;
 
static T_REPORTER reporter[REP_CNT];
static uint8_t active_reports = 0;

//To reduce the possibility of losing steps, the Step Reporter can only be turned off for upto 
//a maximum of 15 seconds. After that period it will automatically re-enable if the user hasn't
//already turned it back on.
#define MAX_STEP_REPORTER_OFF_TIME	(15000UL)	//Max Number of milliseconds that reporter can stay disabled

//elements needed by a step tracking reporter	
typedef struct{
	uint32_t start_time;
	uint32_t period;
	int16_t period_sum;			//temporary accumulator
	T_STEP_HISTORY * hist;		//pointer to recent peak history
	uint8_t hist_tail;
	T_STEP_ELEMENT data;		//number of steps detected during period X
	uint8_t lock;
	TTASK_TIMER re_enable;	//Do not allow report to stay off for longer than MAX Disable Time
} T_STEP_REPORTER;

static union {
	bool flag[REP_CNT];
	uint8_t all;
} rebase;

static T_STEP_REPORTER step_report;
static T_STEP_REPORTER alt_step_report;
static uint8_t ave_accel_lock = 0xFF;
static uint8_t raw_accel_lock = 0xFF;
static uint8_t raw_gyro_lock = 0xFF;
static uint8_t raw_magnet_lock = 0xFF;

bool accum_step_per_time( T_STEP_REPORTER * );
void accum_3axis_data( T_REPORTER * rep_inst, T_3AXIS_FIFO * buf );
void ave_accel_data( T_REPORTER * rep_inst, T_3AXIS_FIFO * buf );
uint16_t sampling_rate_fit( uint16_t req_rate, uint16_t actual_rate );
uint8_t number_fields( T_DATA_FIELDS );
void save_fields( T_REPORTER * rep, T_3AXIS * tail );
static bool buffer_record( T_REPORTER * );

///
//! \fn
/// \brief
/// \param
/// \return .
///
void init_reporters( void ) 
{
	uint32_t temp;
	
	active_reports = 0;
	
	//init locks
	step_report.lock = 0xFF;
	alt_step_report.lock = 0xFF;
	ave_accel_lock = 0xFF;
	raw_accel_lock = 0xFF;
	raw_gyro_lock = 0xFF;
	raw_magnet_lock = 0xFF;
	
	stop_task_timer( step_report.re_enable );		//make sure timer is Off
	
	for( uint i=0; i<REP_CNT; i++ )
	{			
		//if (gs_bdebug) app_trace_log(DEBUG_LOW, "Reporter %01u Init\r", i);
		
		//load attributes from config memory
		uint8_t * ptr = (uint8_t *) &reporter[i].atts;
		reporter[i].atts.inst = i;	//no need to explicitly save the instance number in the config space
		for( uint j=1; j<ATTRIBUTE_SIZE; j++ )
		{
			*(ptr+j) = g_config.report_atts[i][j-1];
		}	
		
		reporter[i].stat.enabled = false;
		reporter[i].stat.atts_valid = false;
		reporter[i].rec.preamble = NEW_REC_PREAMBLE;
		reporter[i].rec.hdr.report_inst = i;
		
		temp = reporter[i].stat.rep_len;
		reporter[i].stat.rep_len = 0;	//needs to be 0 for set_reporter() to work

		if( set_reporter( &reporter[i].atts, ATTRIBUTE_SIZE ) == NO_ERROR ) {
			if( reporter[i].atts.records_rep == 0 ) {
				//app_trace_log(DEBUG_LOW, "Reporter %01u On\r", i );
				//Unlimited length reporter
				if( reporter[i].atts.dependent_data == STEPS_TIME ) {
					//If no limit on Step Reporter, turn it on automatically
					active_reports |= 1<<i;
				}
				if( reporter[i].atts.dependent_data == EXPERIMENTAL ) {
					//Experiemntal Step Routine
					active_reports |= 1<<i;
				}
				if( reporter[i].atts.dependent_data == BATTERY_REMAINING ) {
					//If no limit on Reporter, turn it on automatically?
					active_reports |= 1<<i;
				}
				if( reporter[i].atts.dependent_data == ACCEL_FILT ) {
					//Average Accel Data to get General Activity and Orientation
					active_reports |= 1<<i;
				}
			}
		}
		else {
			if (gs_bdebug) app_trace_log(DEBUG_MED, "Reporter Init Failed!\r");
		}
		
		reporter[i].stat.rep_len = temp;
	}
		
	enable_reporters( active_reports );
	//recording.flags = 0;
		
	//Assign Step reporters to track specific algorithms 
	step_report.hist = step_history_2();
	alt_step_report.hist = step_history_1();
}

bool rec_ack_req( uint8_t rep_inst )
{
	bool res;
	
	if( rep_inst >= REP_CNT ) {
		if (gs_bdebug) app_trace_puts(DEBUG_MED, "REC_ACK: Instance Error\r");
		res = false;
	}
	else {
		res = (bool) reporter[rep_inst].stat.ack_require;
	}
	
	return res;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void erase_reporter_config( void ) 
{
	enable_reporters(0x00);					//start by turning all reporters Off
	for( int i=0; i<REP_CNT; i++ ) 
	{
		reporter[i].atts.dependent_data = NO_TYPE;
		reporter[i].stat.rep_len = 0;		//needs to be 0 for set_reporter() to work
		set_reporter( &reporter[i].atts, ATTRIBUTE_SIZE );
	}
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void reporter_time_rebase( uint32_t new_time, uint32_t old_time )
{
	int32_t time_shift = new_time - old_time;
	
	rebase.all = 0xFF&active_reports;	//flag reporters that are enabled
		
	//adjust Step Report start time to new base
	step_report.start_time += time_shift;
	//adjust Step History buffer timestamps
	uint16_t tail = step_report.hist_tail;
	while( tail != step_report.hist->head ) {
		int64_t time = step_report.hist->step[tail].time + time_shift;
		if( time < 0 ) time = 0;	//test to make sure no weirdness happens
		step_report.hist->step[tail].time = time;
		tail = (tail+1)&SH_BUF_MASK;
	}
	
	//Rebase Alternate Step Reporter
	alt_step_report.start_time += time_shift;
	//adjust Step History buffer timestamps
	tail = alt_step_report.hist_tail;
	while( tail != alt_step_report.hist->head ) {
		int64_t time = alt_step_report.hist->step[tail].time + time_shift;
		if( time < 0 ) time = 0;	//test to make sure no weirdness happens
		alt_step_report.hist->step[tail].time = time;
		tail = (tail+1)&SH_BUF_MASK;
	}
	
	//add any other objects that need time adjustments
}

///
//! \fn
/// \brief
/// \param
/// \return
///
T_REPORT_ERR set_reporter( T_REPORT_ATTRIBUTES * atts, uint8_t atts_len ) 
{
	uint8_t sample_byte_cnt = 1;
	uint16_t actual_rate;
	
	//Quick pass/fail tests 
	if( atts->inst >= REP_CNT ) {
		return INST_ERROR;
	}
	else if( atts_len != ATTRIBUTE_SIZE ) {
		//Incomplete list of attributes... Assume this must be a request for the existing attributes
		if (gs_bdebug) app_trace_log(DEBUG_LOW, "Return Current Atts\r" );
		for( uint i=0; i<ATTRIBUTE_SIZE; i++ )
		{
			*((uint8_t *)atts+i) = *((uint8_t *)&reporter[atts->inst].atts+i);
		}	
		return NO_ERROR;
	}
	else if( reporter[atts->inst].stat.enabled == true ) {
		//Reporter is already active. Don't allow it to change unless it is off
		return REPORT_ON_ERROR;
	}
	else if( reporter[atts->inst].stat.rep_len > 0 ) {
		//There are still records queued in memory for this report instance. If the reporter is changed
		//right now, that data will be misinterpreted.
		return REPORT_NOT_EMPTY;
	}
		
	switch( atts->dependent_data ) {
		case NO_TYPE:
			//Clear Reporter Attributes...
			atts->independent_data = UNITLESS;
			atts->indep_scalar = 0;
			//atts->dependent_data = NO_TYPE;	//redundant
			atts->dep_var_type = T_CHAR;
			atts->dep_scale = _1_1BIT;
			atts->data_fields.flags = 0;
			atts->samples_rec = 1;
			atts->records_rep = 1;
			
			sample_byte_cnt = 1;
			
			reporter[atts->inst].stat.time_remain= 0;				//trigger immediately
			reporter[atts->inst].stat.timer_period = 1;				//update messages every second
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = false;
			break;
			
		case ASCII:
			atts->independent_data = UNITLESS;
			atts->indep_scalar = 1;
			//atts->dependent_data = ASCII;	//redundant
			atts->dep_var_type = T_CHAR;
			atts->dep_scale = _1_1BIT;
			atts->data_fields.flags = 0;
						
			sample_byte_cnt = 1;
			
			reporter[atts->inst].stat.time_remain= 0;				//trigger immediately
			reporter[atts->inst].stat.timer_period = 1;				//update messages every second
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = false;
			break;
				
		case STEPS_TIME:
			atts->data_fields.flags = 0;
			atts->dep_scale = _1_1BIT;
		
			// Make sure the Independent data attribute makes sense
			if( atts->independent_data != SECONDS && atts->independent_data != MINUTES ) {
				atts->dep_var_type = T_UINT8;
				atts->independent_data = SECONDS;
				atts->indep_scalar = 15;
				sample_byte_cnt = 1;
			}
			
			//0 not allowed, minimum of 1 second time periods
			if( atts->indep_scalar == 0 ) atts->indep_scalar = 1;	
						
			switch( atts->dep_var_type ) {
				case T_CHAR:
				case T_INT8:
					atts->dep_var_type = T_UINT8;
				case T_UINT8:
					if( atts->independent_data == SECONDS && atts->indep_scalar > 60 ) {
						atts->dep_var_type = T_UINT16;
						sample_byte_cnt = 2;
					}
					else if( atts->independent_data == MINUTES && atts->indep_scalar > 1 ) {
						atts->dep_var_type = T_UINT16;
						sample_byte_cnt = 2;
					}
					else {
						sample_byte_cnt = 1;
					}
					break;
					
				case T_INT16:
					atts->dep_var_type = T_UINT16;
				case T_UINT16:
					sample_byte_cnt = 2;
					break;
					
				default:
					atts->dep_var_type = T_UINT8;
					atts->independent_data = SECONDS;
					atts->indep_scalar = 15;
					sample_byte_cnt = 1;
					break;
			}
			
			reporter[atts->inst].stat.timer_period = 1;				//check steps every second to update reporter		
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = true;		//Require an ACK fom the host so data cant be lost
			break;
				
		case ACCEL_RAW:
		
			atts->independent_data = SAMPLING_HZ;
			atts->data_fields.flags &= MOTION_MASK;
		
			switch ( atts->dep_var_type ) {

				case T_INT8:
					//atts->dep_var_type = T_INT8;	//redundant
					sample_byte_cnt = number_fields( atts->data_fields );
					break;
				
				default:
					atts->dep_var_type = T_INT16;
					sample_byte_cnt = 2*number_fields( atts->data_fields );
					break;
			}
			
			//figure out sampling rate situation
			actual_rate = get_accel_sample_rate();	//get the current sampling rate
			atts->indep_scalar = sampling_rate_fit( atts->indep_scalar, actual_rate );
		
			switch( atts->dep_scale ) {
				case _4G:
					reporter[atts->inst].stat.dep_scale = 4;
					break;
					
				//case _8G:
				//	reporter[atts->inst].stat.dep_scale = 8;
				//	break;
					
				//case _16G:
				//	reporter[atts->inst].stat.dep_scale = 16;
				//	break;
					
				default:
					atts->dep_scale = _4G;
					reporter[atts->inst].stat.dep_scale = 4;
					break;
			}
			
			//Minimum length for Raw data must be at least the amount of data created in 1 second 
			//(flash write call is made once every second to prevent constant flash writes)
			//atts->samples_rec = actual_rate;
			reporter[atts->inst].stat.timer_period = 1;		//update reporter every second (max update rate)
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = false;	//No ACK required to speed up data transfer
			break;
				
		case GYRO_RAW:
						
			atts->independent_data = SAMPLING_HZ;
			atts->data_fields.flags &= MOTION_MASK;
			if( atts->dep_var_type != T_INT8 ) {
				atts->dep_var_type = T_INT16;
				sample_byte_cnt = 2*number_fields( atts->data_fields );
			}
			else {
				//atts->dep_var_type = T_INT8;	//redundant
				sample_byte_cnt = number_fields( atts->data_fields );
			}
						
			//figure out sampling rate situation
			actual_rate = get_gyro_sample_rate();	//get the current sampling rate
			atts->indep_scalar = sampling_rate_fit( atts->indep_scalar, actual_rate );
						
			switch( atts->dep_scale ) {
				//case _500DPS:
				//	reporter[atts->inst].stat.dep_scale = 500;
				//	break;
							
				case _1000DPS:
					reporter[atts->inst].stat.dep_scale = 1000;
					break;
							
				//case _2000DPS:
				//	reporter[atts->inst].stat.dep_scale = 2000;
				//	break;
							
				default:
					atts->dep_scale = _1000DPS;
					reporter[atts->inst].stat.dep_scale = 1000;
					break;
			}
						
			//atts->samples_rec = actual_rate;			//For Raw data, record samples needs to be at least equal to the sampling rate
			reporter[atts->inst].stat.timer_period = 1;	//update reporter every second
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = false;	//No ACK required to speed up data transfer
			break;
				
		case MAGNETO_RAW:
		
			atts->independent_data = SAMPLING_HZ;
			atts->data_fields.flags &= MOTION_MASK;
			if( atts->dep_var_type != T_INT8 ) {
				atts->dep_var_type = T_INT16;
				sample_byte_cnt = 2*number_fields( atts->data_fields );
			}
			else {
				//atts->dep_var_type = T_INT8;	//redundant
				sample_byte_cnt = number_fields( atts->data_fields );
			}
						
			//figure out sampling rate situation
			actual_rate = get_compass_sample_rate();	//get the current sampling rate
			atts->indep_scalar = sampling_rate_fit( atts->indep_scalar, actual_rate );
						
			switch( atts->dep_scale ) {
				//case _4GAUSS:
				//	reporter[atts->inst].stat.dep_scale = 8;
				//	break;
							
				//case _8GAUSS:
				//	reporter[atts->inst].stat.dep_scale = 8;
				//	break;
							
				//case _12GAUSS:
				//	reporter[atts->inst].stat.dep_scale = 12;
				//	break;
				
				//case _16GAUSS:
				//	reporter[atts->inst].stat.dep_scale = 16;
				//	break;
				
				case _48GAUSS:
					reporter[atts->inst].stat.dep_scale = 4800;	//converted to uTesla
					break;
							
				default:
					atts->dep_scale = _48GAUSS;
					reporter[atts->inst].stat.dep_scale = 4800;	//converted to uTesla
					break;
			}

			reporter[atts->inst].stat.timer_period = 1;	//update reporter every second
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = false;	//No ACK required to speed up data transfer
			break;
			
		case ACCEL_FILT:
			atts->data_fields.flags = 0;				//This is going to store differently than how Raw data does
			atts->independent_data = SECONDS;
			atts->dep_var_type = T_INT16;
			//the 3 axes are stuffed into 2 bytes (5 bits each, the upper bit indicates sample periods when the accel is off)
			sample_byte_cnt = 2;
		
			switch( atts->dep_scale ) {				
				case _2G:
					reporter[atts->inst].stat.dep_scale = 2;
					break;
				default:
					atts->dep_scale = _1G;
					reporter[atts->inst].stat.dep_scale = 1;
					break;
			}

			if( atts->indep_scalar < 1 ) atts->indep_scalar = 1;	//minimum sample window of 1 second
			
			if( atts->samples_rec < 12 ) atts->samples_rec = 12;	//Don't allow to save too frequently
			reporter[atts->inst].stat.timer_period = 1;				//check data every second to update reporter	
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = true;
			break;
			
		case GYRO_FILT:
			reporter[atts->inst].stat.atts_valid= false; //data type not yet supported
			reporter[atts->inst].stat.ack_require = true;
			break;
		
		case MAGNETO_FILT:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			reporter[atts->inst].stat.ack_require = true;
			break;
		
		case ACCEL_VARIANCE:
			atts->independent_data = SAMPLING_HZ;
			atts->data_fields.flags &= MOTION_MASK;
		
			atts->dep_var_type = T_INT16;
			sample_byte_cnt = 2*number_fields( atts->data_fields );
			
			//figure out sampling rate situation
			actual_rate = get_accel_sample_rate();			//get the current sampling rate
			atts->indep_scalar = sampling_rate_fit( atts->indep_scalar, actual_rate );
	
			atts->dep_scale = _4G;
			reporter[atts->inst].stat.dep_scale = 4;
			
			//Minimum length for Linear data must be at least the amount of data created in 1 second 
			//(flash write call is made once every second to prevent constant flash writes)
			//atts->samples_rec = actual_rate;
			reporter[atts->inst].stat.timer_period = 1;		//update reporter every second (max update rate)
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = false;	//No ACK required to speed up data transfer
			break;
			
		case GYRO_VARIANCE:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			reporter[atts->inst].stat.ack_require = true;
			break;
			
		case MAGNETO_VARIANCE:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			reporter[atts->inst].stat.ack_require = true;
			break;
			
		case BATTERY_REMAINING:
			atts->data_fields.flags = 0;		
		
			//Setting to Save Battery Percent Remaining Periodically or on Level Change
			atts->dep_scale = _1_1BIT;			//1%/bit, 1mV/bit
			reporter[atts->inst].stat.dep_scale = 1;
		
			atts->dep_var_type = T_INT24;
			sample_byte_cnt = 3;										//Remaining Bet Percent (UINT8), Battery Voltage (INT16)
				
			if( atts->independent_data == MINUTES ) {
				//Save Remaining Percent Periodically
				if( atts->indep_scalar == 0 ) atts->indep_scalar = 1;		//0 not allowed, minimum of 1 sample period
				reporter[atts->inst].stat.timer_period = 60*atts->indep_scalar;	//check every X*60 second to update reporter	
			}
			else {
				//Save Remaining Percent when it changes
				atts->independent_data = ON_CHANGE;
				atts->indep_scalar = 1;									//Set value for simplicity
				atts->samples_rec = 1;									//Record is saved On change... Only 1 sample 
				reporter[atts->inst].stat.timer_period = 1;				//check every second to update reporter	
			}
				
			reporter[atts->inst].stat.atts_valid = true;	
			reporter[atts->inst].stat.ack_require = true;
			break;
		
		case CYCLES_PER_TIME:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			reporter[atts->inst].stat.ack_require = true;
			break;
			
		case TILT_ANGLE:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			reporter[atts->inst].stat.ack_require = true;
			break;

		case BOARD_TEMP:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			reporter[atts->inst].stat.ack_require = true;
			break;
		
		case EXPERIMENTAL:			
			
			atts->data_fields.flags = 0;
			atts->dep_scale = _1_1BIT;
		
			// Make sure the Independent data attribute makes sense
			if( atts->independent_data != SECONDS && atts->independent_data != MINUTES ) {
				atts->dep_var_type = T_UINT8;
				atts->independent_data = SECONDS;
				atts->indep_scalar = 15;
				sample_byte_cnt = 1;
			}
			
			//0 not allowed, minimum of 1 second time periods
			if( atts->indep_scalar == 0 ) atts->indep_scalar = 1;	
						
			switch( atts->dep_var_type ) {
				case T_CHAR:
				case T_INT8:
					atts->dep_var_type = T_UINT8;
				case T_UINT8:
					if( atts->independent_data == SECONDS && atts->indep_scalar > 60 ) {
						atts->dep_var_type = T_UINT16;
						sample_byte_cnt = 2;
					}
					else if( atts->independent_data == MINUTES && atts->indep_scalar > 1 ) {
						atts->dep_var_type = T_UINT16;
						sample_byte_cnt = 2;
					}
					else {
						sample_byte_cnt = 1;
					}
					break;
					
				case T_INT16:
					atts->dep_var_type = T_UINT16;
				case T_UINT16:
					sample_byte_cnt = 2;
					break;
					
				default:
					atts->dep_var_type = T_UINT8;
					atts->independent_data = SECONDS;
					atts->indep_scalar = 15;
					sample_byte_cnt = 1;
					break;
			}
			
			reporter[atts->inst].stat.timer_period = 1;				//check steps every second to update reporter		
			reporter[atts->inst].stat.atts_valid = true;
			reporter[atts->inst].stat.ack_require = true;
			break;
		
//		case HEADING:
//			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
//			reporter[atts->inst].stat.ack_require = true;
//			break;	
		
		default:
			reporter[atts->inst].stat.atts_valid = false; //data type not recognized
			break;
	}
	
	//Reporter Attributes cannot be used
	if( reporter[atts->inst].stat.atts_valid == false ) {
		return DATA_TYPE_ERROR;
	}
	
	//make sure the save length is plausible, and establish the other lengths that will be needed to track and save a record
	if( atts->samples_rec*sample_byte_cnt > REC_MAX_DATA_LEN ) {
		atts->samples_rec = REC_MAX_DATA_LEN/sample_byte_cnt;
	}
	else if( atts->samples_rec < 1 ) {
		atts->samples_rec = 1;	//At least 1 sample before saving
	}
	reporter[atts->inst].stat.sample_size = sample_byte_cnt;
	reporter[atts->inst].stat.rec_full_len = atts->samples_rec*sample_byte_cnt;
	
	//make sure the Maximum report length will fit within the limits of allocated memory
	uint32_t temp = (uint32_t)atts->records_rep*(reporter[atts->inst].stat.rec_full_len+REC_START_LEN);
	if( temp > get_log_region_size() ) {
		atts->records_rep = (get_log_region_size())/(reporter[atts->inst].stat.rec_full_len+REC_START_LEN);
	}

	reporter[atts->inst].stat.rec_data_len = 0;			//should be 0 already, but just in case
	
	//copy attributes
	uint8_t * ptr = (uint8_t *) &reporter[atts->inst].atts;
	for( uint i=0; i<ATTRIBUTE_SIZE; i++ )
	{
		*(ptr+i) = *((uint8_t *)atts+i);	//copy requested Atts into working ram
		if( i > 0 ) {
			//first byte not copied into nonvolatile memory (it is just the report instance number)
			g_config.report_atts[atts->inst][i-1] = *(ptr+i);
		}
	}
	
	//app_trace_log(DEBUG_LOW, "Attribute Size %02u\r", ATTRIBUTE_SIZE );
			
	return NO_ERROR;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void enable_reporters( uint8_t reports_enable ) 
{
	volatile uint8_t report_mask = 0;
	volatile uint8_t report_on_off = 0;
	volatile uint8_t act_reporters = 0;
	
	uint32_t start_time = get_unix_time();
		
	for( int i=0; i<REP_CNT; i++ ) {
	
		report_mask = 1<<i;
		if( reporter[i].stat.atts_valid != true ) {
			//don't let reporter be enabled if the set function has not be validated
			report_on_off = 0;
		}
		else {
			report_on_off = reports_enable&report_mask;
		}
			
		switch( reporter[i].atts.dependent_data ) {
			case NO_TYPE:
				//can only be off
				reporter[i].stat.enabled = false;
				break;
			
			case STEPS_TIME:
				if (gs_bdebug) app_trace_log(DEBUG_MED, "Step Reporter: ");
				if( report_on_off != 0 ) {
					if( reporter[i].stat.rep_len <= (unsigned int)(reporter[i].atts.records_rep-1) ) {	
						//Reporters defined with 0 length will stay on indefinitely, otherwise don't turn on if more
						//records are already stored than the reporter is defined to hold
						if( step_report.lock >= REP_CNT ) {
							//only 1 steps/time reporter can operate at a time. The accum_step_per_time() function
							//is not designed to support more than 1 calling reporter.
							if (gs_bdebug) app_trace_log(DEBUG_MED, "Enabled\r");
							
							reporter[i].stat.rec_data_len = 0;
							reporter[i].stat.time_remain = 0;	//set to trigger immediately
							reporter[i].stat.enabled = true;
						
							//init step tracking variables
							step_report.start_time = start_time;
							step_report.period = reporter[i].atts.indep_scalar;	//save data sample to record at every interval of this period
							if( reporter[i].atts.independent_data == MINUTES ) step_report.period *= 60;	//convert timer period to seconds
							step_report.period_sum = 0;							
							step_report.hist_tail = step_report.hist->head;
							rebase.flag[i] = false;	//clear the rebase flag bit
							stop_task_timer( step_report.re_enable );		//coming on, no need for an automatic re-enable
							//clear step History?
							
							step_report.lock = i;
						}
						else {
							if (gs_bdebug) app_trace_log(DEBUG_MED, "Already Enabled\r");
						}
					}
					else {
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Reporter Full\r");
					}
				}
				else { 
					if( step_report.lock == i ) {
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Disabled\r");
						//Requested to turn off Reporter
						step_report.lock = 0xFF;
						
						//Set to automatically re-enable the Step Reporter if the user forgets to
						start_task_timer( step_report.re_enable, MAX_STEP_REPORTER_OFF_TIME );
						
						//Push partial Record to RAM holding buffer
						if( reporter[i].stat.rec_data_len > 0 ) {
							buffer_record( &reporter[i] );
						}
					}
					else {
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Nothing to Disable\r");
					}
					
					reporter[i].stat.enabled = false;
				}
				break;
				
			case EXPERIMENTAL:
				if (gs_bdebug) app_trace_log(DEBUG_MED, "X Step Reporter:");
				if( report_on_off != 0 ) {
					if( reporter[i].stat.rep_len <= (unsigned int)(reporter[i].atts.records_rep-1) ) {	//allows reports of 0 length to stay on indef
						if( alt_step_report.lock >= REP_CNT ) {
							//only 1 steps/time reporter can operate at a time. The accum_step_per_time() function
							//is not designed to support more than 1 calling reporter.
							if (gs_bdebug) app_trace_log(DEBUG_MED, "Enabled\r");
							
							reporter[i].stat.rec_data_len = 0;
							reporter[i].stat.time_remain = 0;	//set to trigger immediately
							reporter[i].stat.enabled = true;
						
							//init step tracking variables
							alt_step_report.start_time = start_time;
							alt_step_report.period = reporter[i].atts.indep_scalar;	//save data sample to record at every interval of this period
							if( reporter[i].atts.independent_data == MINUTES ) alt_step_report.period *= 60;	//convert timer period to seconds
							alt_step_report.period_sum = 0;							
							alt_step_report.hist_tail = alt_step_report.hist->head;
							rebase.flag[i] = false;	//clear the rebase flag bit
							//clear step History?
							
							alt_step_report.lock = i;
						}
						else {
							if (gs_bdebug) app_trace_log(DEBUG_MED, "Already Enabled\r");
						}
					}
					else {
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Reporter Full\r");
					}
				}
				else { 
					if( alt_step_report.lock == i ) {
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Disabled\r");
						
						//Requested to turn off Reporter
						alt_step_report.lock = 0xFF;
						
						//Push partial Record to RAM holding buffer
						if( reporter[i].stat.rec_data_len > 0 ) {
							buffer_record( &reporter[i] );
						}
					}
					else {
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Nothing to Disable\r");
					}
					
					reporter[i].stat.enabled = false;
				}
				break;
			
			//the majority of cases have no special requirements and simply need to turn on
			default:
				
				//Check Data Types that can only be accumulated by 1 reporter at a time
				if( reporter[i].atts.dependent_data == ACCEL_FILT ) {
					if( ave_accel_lock >= REP_CNT ) {
						//function access is not currently assigned to a reporter
						if( report_on_off != 0 ) {
							ave_accel_lock = i;	//assign the only reporter that can use
						}
					}
					else if( ave_accel_lock != i ) {
						//Data type access is is locked to another reporter. Block this Reporter's Access!
						if( report_on_off != 0 ) {
							app_trace_log(DEBUG_MED, "Ave Accel Rep %01u Deny\r", i);
						}
						report_on_off = 0;
					}
					else {
						//function access is currently held by this reporter
						if( report_on_off == 0 ) {
							ave_accel_lock = 0xFF;	//release the lock
						}
					}
					if (gs_bdebug) app_trace_log(DEBUG_MED, "Ave Accel Lock: %01u\r", ave_accel_lock);
				}
				else if( reporter[i].atts.dependent_data == ACCEL_RAW) {
					if( raw_accel_lock >= REP_CNT ) {
						//function access is not currently assigned to a reporter
						if( report_on_off != 0 ) {
							raw_accel_lock = i;		//assign the only reporter that can use
							record_accel_on();		//keep Accelerometer active
						}
					}
					else if( raw_accel_lock != i ) {
						//Data type access is is locked to another reporter. Block this Reporter's Access!
						if( report_on_off != 0 ) {
							app_trace_log(DEBUG_MED, "Raw Accel Rep %01u Deny\r", i);
						}
						report_on_off = 0;
					}
					else {
						//function access is currently held by this reporter
						if( report_on_off == 0 ) {
							raw_accel_lock = 0xFF;	//release the lock
							record_accel_off();	
						}
					} 
					if (gs_bdebug) app_trace_log(DEBUG_MED, "Raw Accel Lock: %01u\r", raw_accel_lock);
				}
				else if( reporter[i].atts.dependent_data == GYRO_RAW) {
					if( raw_gyro_lock >= REP_CNT ) {
						//function access is not currently assigned to a reporter
						if( report_on_off != 0 ) {
							raw_gyro_lock = i;		//assign the only reporter that can use
							record_gyro_on();		//keep Gyrometer active
						}
					}
					else if( raw_gyro_lock != i ) {
						//Data type access is is locked to another reporter. Block this Reporter's Access!
						if( report_on_off != 0 ) {
							app_trace_log(DEBUG_MED, "Raw Gyro Rep %01u Deny\r", i);
						}
						report_on_off = 0;
					}
					else {
						//function access is currently held by this reporter
						if( report_on_off == 0 ) {
							raw_gyro_lock = 0xFF;	//release the lock
							record_gyro_off();
						}
					}
					if (gs_bdebug) app_trace_log(DEBUG_MED, "Raw Gyro Lock: %01u\r", raw_gyro_lock);
				}
				else if( reporter[i].atts.dependent_data == MAGNETO_RAW) {
					if( raw_magnet_lock >= REP_CNT ) {
						//function access is not currently assigned to a reporter
						if( report_on_off != 0 ) {
							raw_magnet_lock = i;	//assign the only reporter that can use
							record_compass_on();	//keep Magnetometer active
						}
					}
					else if( raw_magnet_lock != i ) {
						//Data type access is is locked to another reporter. Block this Reporter's Access!
						if( report_on_off != 0 ) {
							app_trace_log(DEBUG_MED, "Raw Compass Rep %01u Deny\r", i);
						}
						report_on_off = 0;
					}
					else {
						//function access is currently held by this reporter
						if( report_on_off == 0 ) {
							raw_magnet_lock = 0xFF;	//release the lock
							record_compass_off();
						}
					}
					if (gs_bdebug) app_trace_log(DEBUG_MED, "Raw Compass Lock: %01u\r", raw_magnet_lock);
				}
				
				if( report_on_off != 0 ) {
					if( reporter[i].stat.rep_len <= (unsigned int)(reporter[i].atts.records_rep-1) ) {	//allows reports of 0 length to stay on indef
						if( reporter[i].stat.enabled == false ) {
							if (gs_bdebug) app_trace_log(DEBUG_MED, "Reporter %01u Enabled\r", i);
							
							reporter[i].stat.rec_data_len = 0;
							reporter[i].stat.time_remain = 0;	//set to trigger immediately
							reporter[i].stat.new_start = true;
							reporter[i].stat.rec_saved = false;
							reporter[i].stat.enabled = true;
						
							rebase.flag[i] = false;
						}
					}
				}
				else {
					if( reporter[i].stat.enabled == true ) {
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Reporter %01u Disabled\r", i);
					}
					
					//Push partial Record to RAM holding buffer
					if( reporter[i].stat.rec_data_len > 0 ) {
						buffer_record( &reporter[i] );
					}

					reporter[i].stat.enabled = false;
				}
				break;
		}	//end switch
			
		if( reporter[i].stat.enabled == true ) {
			act_reporters |= report_on_off;	//indicate Report[i] is On
		}
	}	//end for loop
	
	active_reports = act_reporters;		//copy to global
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint8_t get_active_reporters( void )
{
	return active_reports;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
static void auto_enable_step_reporter( void )
{
	int i;
	
	for( i=0; i<REP_CNT; i++ ) {
		if( reporter[i].atts.dependent_data == STEPS_TIME ) {
			if (gs_bdebug) app_trace_log(DEBUG_MED, "Step Reporter Auto Restart\r");
			enable_reporters( active_reports | (1<<i) );		//turn Step reporter back on
			break;	
		}
	}
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void check_reporters( void ) 
{
	Union32 u32temp;
	uint32_t unix_time = get_unix_time();	//stamp coming into function
	
	//Check to prevent the Step Reporter froming being permanenatly disabled
	if( task_time( step_report.re_enable ) ) 
	{	//Step Reporter has been off for too long. Automatically turn it back on
		auto_enable_step_reporter();
		stop_task_timer( step_report.re_enable );
	}
	
	for( int i=0; i<REP_CNT; i++ ) {			
		if( reporter[i].stat.enabled == true ) {
			if( reporter[i].stat.time_remain-- <= 0 ) {
				reporter[i].stat.time_remain += reporter[i].stat.timer_period;
				
				switch( reporter[i].atts.dependent_data ) {
					case NO_TYPE:
						break;
				
					case ASCII:
						//no messages to add
						break;
								
					case STEPS_TIME:					
						//Check for step updates
						if( accum_step_per_time( &step_report ) == true ) 
						{		
							static uint8_t x_zero_cnt = 0;		

							//don't start a record unless steps are detected or a record has already started!!!						
							if( step_report.data.cnt > 0 || reporter[i].stat.rec_data_len > 0 ) {
								//time to log some data...
								//if (gs_bdebug) app_trace_log(DEBUG_LOW, "Append Step Data!!!\r");
								
								if( reporter[i].stat.rec_data_len == 0 ) {
									//start of new record
									x_zero_cnt = 0;
									reporter[i].rec.hdr.time = step_report.data.time_stamp;
								}
							
								//add data to record
								u32temp.u32 = step_report.data.cnt;
								for( int j=0; j<reporter[i].stat.sample_size; j++ ) {
									reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.u8[j];
								}
							
								//if 12 zeros in a row are recorded, save the partial record to conserve space.
								if( u32temp.u32 == 0 ) {
									x_zero_cnt += reporter[i].stat.sample_size;
									if(  x_zero_cnt >= REC_START_LEN ) {
										reporter[i].stat.rec_data_len -= x_zero_cnt;	//remove the last X zeros, no need to save them.
										if( buffer_record( &reporter[i] ) == false ) {
											//Did not Save
										}
									}
								}
								else {
									x_zero_cnt = 0;
								}
							}
						}
						rebase.flag[i] = false;	//time was rebased when rebase function was called
						break;
						
					case EXPERIMENTAL:
						// Testing Ground for new record types
						// Check for step updates
						if( accum_step_per_time( &alt_step_report ) == true ) {		
							static uint8_t zero_cnt = 0;		

							//don't start a record unless steps are detected or a record has already started!!!						
							if( alt_step_report.data.cnt > 0 || reporter[i].stat.rec_data_len > 0 ) {
								//time to log some data...
								//if (gs_bdebug) app_trace_log(DEBUG_LOW, "Append Step Data!!!\r");
								
								if( reporter[i].stat.rec_data_len == 0 ) {
									//start of new record
									zero_cnt = 0;
									reporter[i].rec.hdr.time = alt_step_report.data.time_stamp;
								}
							
								//add data to record
								u32temp.u32 = alt_step_report.data.cnt;
								for( int j=0; j<reporter[i].stat.sample_size; j++ ) {
									reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.u8[j];
								}
							
								//if 12 zeros in a row are recorded, save the partial record to conserve space.
								if( u32temp.u32 == 0 ) {
									zero_cnt += reporter[i].stat.sample_size;
									if(  zero_cnt >= REC_START_LEN ) {
										reporter[i].stat.rec_data_len -= zero_cnt;	//remove tht last X zeros, no need to save them.
										if( buffer_record( &reporter[i] ) == false ) {
											//Did not Save
										}
									}
								}
								else {
									zero_cnt = 0;
								}
							}
						
						}
						rebase.flag[i] = false;	//time was rebased when rebase function was called
						break;
								
					case ACCEL_RAW:											
						//Rebase unnecessarry for RAW data. If it is collecting, it will already be frequently saving.
						accum_3axis_data( &reporter[i], &accel_buf );
						break;
								
					case GYRO_RAW:
						accum_3axis_data( &reporter[i], &gyro_buf );
						break;
								
					case MAGNETO_RAW:
						accum_3axis_data( &reporter[i], &magnet_buf );
						break;
								
					case ACCEL_FILT:
						//If local time has been changed. Start a new Record to reflect new time
						if( rebase.flag[i] == true ) {
							if (gs_bdebug) app_trace_log(DEBUG_MED, "Time update. Saving Reporter\r");
							rebase.flag[i] = false;
							if( reporter[i].stat.rec_data_len > 0 ) {
								//save portion of record that has been created
								if( buffer_record( &reporter[i] ) == false ) {
									//Did not Save
								}
							}
						}
	
						ave_accel_data( &reporter[i], &accel_buf );
						break;
								
					case GYRO_FILT:
						break;
								
					case MAGNETO_FILT:
						break;
								
					case ACCEL_VARIANCE:							
						//When reporter first starts up, Zero out the recent data buffer. This will force
						//the Accel, Gyro, and Compass data to sync to the next data available Interrpt
						if( reporter[i].stat.new_start == true ) 
						{
							reporter[i].stat.new_start = false;
							reporter[i].stat.data_fifo_tail = Motion.head;	//zero out the buffer data
							reporter[i].stat.rec_data_len = 0;				//discard partial record
						}
						
						if( get_accel_en() == ON ) 
						{
							//Sensor On!
							//if (gs_bdebug) app_trace_log(DEBUG_LOW, "Save Linear Data!!!\r");
							
							//start of new record
							if( reporter[i].stat.rec_data_len == 0 )
							{
								reporter[i].rec.hdr.time = unix_time;
							}
							
							//abbreviate fifo_tail and get length
							int16_t tail = reporter[i].stat.data_fifo_tail;
							uint32_t len_rem = (Motion.head - tail)&MOT_BUF_MASK;
							while( 	len_rem > 0 ) 
							{	//Save data at subsampled rate.
								save_fields( &reporter[i], &Motion.accel[tail].lin );
								
								tail++;
								tail &= MOT_BUF_MASK;
								len_rem--;
								
								//check if buffer needs to be saved
								if( (reporter[i].stat.rec_data_len+reporter[i].stat.sample_size) > reporter[i].stat.rec_full_len ) {
									//Record is full, time to save...
									if( buffer_record( &reporter[i] ) == false ) {
										//Did not Save
									} 
								}
							}
							
							//save updated fifo_tail
							reporter[i].stat.data_fifo_tail = tail;
						}
						else {
							//Sensor is Off!
							reporter[i].stat.data_fifo_tail = Motion.head;
							reporter[i].stat.rec_data_len = 0;		//discard partial record
						}
						break;
								
					case GYRO_VARIANCE:
						break;
								
					case MAGNETO_VARIANCE:
						break;
								
					case BATTERY_REMAINING:
						
						if( reporter[i].stat.new_start == true ) {
							reporter[i].stat.new_start = false;
							battery_level_change( u32temp.u8 );		//call to clear flag
						}
						
						if( reporter[i].stat.rec_data_len == 0 ) {
							//start of new record
							reporter[i].rec.hdr.time = unix_time;
						}
						
						if( reporter[i].atts.independent_data == MINUTES ) {
							//Periodically save remaining Battery Percent 
							u32temp.u8[0] = battery_get_level();
							reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.u8[0];	
							
							//Periodicaly save Battery Voltage Reading
							u32temp.s32 = battery_get_voltage();
							//the only options that could have been accepted are INT16
							reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.s8[0];
							reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.s8[1];
						}
						else {
							//Save remaining Battery Percent when it changes
							if( battery_level_change( u32temp.u8 ) == true ) {
								reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.u8[0];
								
								u32temp.s32 = battery_get_voltage();
								//the only options that could have been accepted are INT16
								reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.s8[0];
								reporter[i].rec.data[reporter[i].stat.rec_data_len++] = u32temp.s8[1];
							}
						}

						break;
								
					case CYCLES_PER_TIME:
						break;
								
					case TILT_ANGLE:
						break;
								
					case BOARD_TEMP:
						break;
	
					default:
						break;
				}
			}
								
			if( (reporter[i].stat.rec_data_len+reporter[i].stat.sample_size) > reporter[i].stat.rec_full_len ) {
				//Record is full, time to save...
				//asm volatile ("nop");	//break point
				if( buffer_record( &reporter[i] ) == false ) {
					//Did not Save
				} 
			}
			else if( reporter[i].stat.rec_data_len > 0 ) {
				if( (unix_time - reporter[i].rec.hdr.time) > RIDICULOUSLY_OLD ) {
					//Reporter has not been saved in some time, let's put it out of it's misery and push the Record already
					if( buffer_record( &reporter[i] ) == false ) {
						//Did not Save
					}
				}
			}
		}
	}//end for()
	
	//Inform memory manager that all reporters have updated and buffered records may be pushed to nonvolatile backup
	reporter_update_cmplt();
}

//! \fn
/// \brief
/// \param
/// \return .
///
bool accum_step_per_time( T_STEP_REPORTER * rep )
{
	uint32_t time = get_unix_time();

	if( rep->hist_tail == rep->hist->head ){
		//no steps in history array
		if( time > (rep->start_time+rep->period) ) {
			rep->data.time_stamp = rep->start_time;	//capture the starting time of the step data element
			rep->start_time += rep->period;
			
			rep->data.cnt = rep->period_sum;
			rep->period_sum = 0;
			
			return true;	//indicate that the time period has been fully accumulated.
		}
		else if( time < rep->start_time ) {
			rep->start_time = time;
			
			if (gs_bdebug) app_trace_log(DEBUG_MED, "Curious?\r");
		}
	}
	else {
		//there are steps that need to be processed
		while( (rep->hist->step[rep->hist_tail].type != POSS_STEP) && (rep->hist_tail != rep->hist->head) ) {
			//if the time stamp is after the end of the time period, stop accumulating and
			//flag that the period is completed
			if( rep->hist->step[rep->hist_tail].time >= (rep->start_time+rep->period) ) {
				rep->data.time_stamp = rep->start_time;	//capture the starting time of the step data element
				rep->start_time += rep->period;
				
				rep->data.cnt = rep->period_sum;
				rep->period_sum = 0;
				
				return true;	//indicate that the time period has been fully accumulated
			}
			else if( rep->hist->step[rep->hist_tail].time >= rep->start_time ) {
				if( rep->hist->step[rep->hist_tail].type == TRUE_STEP ) {
					rep->period_sum += 1;
				}
			}
			else {
				//timestamp is from before the Record Start Time?
				if (gs_bdebug) app_trace_log(DEBUG_MED, "Curiouser???\r");
				
				if( rep->period_sum > 0 ) {
					rep->data.time_stamp = rep->start_time;							//capture the starting time of the step data element
					rep->start_time = rep->hist->step[rep->hist_tail].time;	//save new Start Time	
					
					rep->data.cnt = rep->period_sum;
					rep->period_sum = 0;
					
					return true;	//indicate that the previous time period has been fully accumulated
				}		
				else {
					rep->start_time = rep->hist->step[rep->hist_tail].time;	//save new Start Time	
					if( rep->hist->step[rep->hist_tail].type == TRUE_STEP ) {
						rep->period_sum += 1;
					}
				}
			}
			
			rep->hist_tail = (rep->hist_tail+1)&SH_BUF_MASK;
		}
	}
	
	return false;	//time period has not been fully evaluated
}

//Forces raw data Reporters to synchronize so those Records will start at the same time 
//(that time being the next data collection check period).
void flag_raw_data_sync( void ) 
{
	app_trace_log(DEBUG_MED, "[REP] 3 AXIS Sync @%01u\r", getSystemTimeMs());
	if( raw_accel_lock < REP_CNT ) reporter[raw_accel_lock].stat.new_start = true;
	if( raw_gyro_lock < REP_CNT ) reporter[raw_gyro_lock].stat.new_start = true;
	if( raw_magnet_lock < REP_CNT ) reporter[raw_magnet_lock].stat.new_start = true;
}

void accum_3axis_data( T_REPORTER * rep_inst, T_3AXIS_FIFO * buf )
{
	int16_t fifo_tail;
	uint32_t fifo_len_rem;
	uint32_t unix_time = get_unix_time();	//stamp coming into function
	
	if( buf->head > IMU_FIFO_MASK ) {
		app_trace_log(DEBUG_HIGH, "[REP] 3 AXIS Buffer Err!!!\r");
		return;
	}
	
	//When reporter first starts up, Zero out the recent data buffer. This will force
	//the Accel, Gyro, and Compass data to sync to the next data available Interrpt
	if( rep_inst->stat.new_start == true ) {
		rep_inst->stat.new_start = false;
		rep_inst->stat.data_fifo_tail = buf->head;	//zero out the FIFO data
		rep_inst->stat.rec_data_len = 0;			//discard partial record
	}
	
	if( buf->active_axis.flags > 0 ) {
		//Sensor On!
		//if (gs_bdebug) app_trace_log(DEBUG_LOW, "Save Compass Data!!!\r");
		
		//start of new record
		if( rep_inst->stat.rec_data_len == 0 ) {
			rep_inst->rec.hdr.time = unix_time;
		}
	
		//incorporate the sub sampling rate
		uint8_t sub_samp_rate = buf->rate/rep_inst->atts.indep_scalar;
		if( sub_samp_rate < 1 ) sub_samp_rate = 1;
		
		//abbreviate fifo_tail and get length
		fifo_tail = rep_inst->stat.data_fifo_tail;
		fifo_len_rem = (buf->head - fifo_tail)&IMU_FIFO_MASK;
		while(  fifo_len_rem >= sub_samp_rate &&
				(rep_inst->stat.rec_data_len + rep_inst->stat.sample_size) <= REC_MAX_LEN ) 
		{	//Save data at subsampled rate.
			T_3AXIS ave;
			int32_t x=0, y=0, z=0, m=0;
					
			//Pass through simple Decimation Filter
			for(int j=0; j<sub_samp_rate; j++){
				x += buf->fifo[fifo_tail].x;
				y += buf->fifo[fifo_tail].y;
				z += buf->fifo[fifo_tail].z;
				m += buf->fifo[fifo_tail].m;
				
				fifo_tail++;
				fifo_tail &= IMU_FIFO_MASK;
			}
			fifo_len_rem -= sub_samp_rate;
			
			//Determines average and transforms result to the User Requested Scale (could be different than generated scale)
			ave.x = buf->range*(x/sub_samp_rate)/rep_inst->stat.dep_scale;	
			ave.y = buf->range*(y/sub_samp_rate)/rep_inst->stat.dep_scale;
			ave.z = buf->range*(z/sub_samp_rate)/rep_inst->stat.dep_scale;
			ave.m = buf->range*(m/sub_samp_rate)/rep_inst->stat.dep_scale;
			
			save_fields( rep_inst, &ave );
			
			//check if buffer nees to be saved
			if( (rep_inst->stat.rec_data_len+rep_inst->stat.sample_size) > rep_inst->stat.rec_full_len ) {
				//Record is full, time to save...
				if( buffer_record( rep_inst ) == false ) {
					//Did not Save
				} 
			}
		}
		
		//save updated fifo_tail
		rep_inst->stat.data_fifo_tail = fifo_tail;
	}
	else {
		//Sensor is Off!
		rep_inst->stat.data_fifo_tail = buf->head;
		rep_inst->stat.rec_data_len = 0;		//discard partial record
	}
}

void ave_accel_data( T_REPORTER * rep_inst, T_3AXIS_FIFO * buf )
{
	static int32_t x=0, y=0, z=0;
	static uint32_t total_samples = 0;
	static int16_t period_update_time = 0;
	int16_t tail;
	uint32_t unix_time = get_unix_time();	//stamp coming into function
	
	//this reporter doesn't have access!!!!
	if( ave_accel_lock != rep_inst->atts.inst ) {
		app_trace_log(DEBUG_MED, "Ave Accel: Rep %01u Denied, Expect Rep %01u\r", rep_inst->atts.inst, ave_accel_lock);
		return;
	}
	
	//When reporter first starts up, Zero out the recent data buffer
	if( rep_inst->stat.new_start == true ) {
		app_trace_log(DEBUG_LOW, "Ave Accel Reporter %01u On\r", rep_inst->atts.inst);
		rep_inst->stat.new_start = false;
		rep_inst->stat.rec_data_len = 0;
		rep_inst->stat.data_fifo_tail = buf->head;	//zero out the FIFO data
		rep_inst->stat.rec_saved = false;
		
		x = y = z = 0;
		total_samples = 0;
		period_update_time = rep_inst->atts.indep_scalar + 1;	//going to be decremented immediately 
		rep_inst->rec.hdr.time = unix_time;
	}
	
	//Start a new Record
	if( rep_inst->stat.rec_saved == true ) {
		rep_inst->stat.rec_saved = false;
		
		app_trace_log(DEBUG_LOW, "New Ave Accel Rec\r");
		
		//start of new record
		x = y = z = 0;
		total_samples = 0;
		period_update_time = rep_inst->atts.indep_scalar;
		rep_inst->rec.hdr.time = unix_time - 1;		//Data goes back to previous call from 1 second ago
	}
	
	//Sum the axes to calc average at end of period
	tail = rep_inst->stat.data_fifo_tail;
	while ( tail != buf->head ) {
		x += buf->fifo[tail].x;
		y += buf->fifo[tail].y;
		z += buf->fifo[tail].z;
		
		total_samples++;
		
		tail++;
		tail &= IMU_FIFO_MASK;
	}
	rep_inst->stat.data_fifo_tail = tail;	
		
	if( --period_update_time <= 0 ) {
		Union32 store_val = {0};
		uint16_t samples_per_period = buf->rate*rep_inst->atts.indep_scalar;
		
		if( total_samples <= (samples_per_period/2) ) {	//need <= for when sample rate = 0
			//Accelerometer was off for the majority of this period. Store as a "motionless" period
			if( rep_inst->stat.rec_data_len < 2 ) {
				//First entry needs to be seeded with a negative value
				store_val.s16[0] = -1;	//Start needs to be negative
				rep_inst->rec.data[ rep_inst->stat.rec_data_len ] = store_val.u8[0];	//lower byte
				rep_inst->rec.data[ rep_inst->stat.rec_data_len+1 ] = store_val.u8[1];	//upper byte
				
				rep_inst->stat.rec_data_len += 2;
			}
			else {
				//Look at previous entry. If it is negative, we are going to accumulate to it. 
				int16_t prv_val = ( (uint16_t) ((uint16_t)rep_inst->rec.data[rep_inst->stat.rec_data_len-1] << 8) | rep_inst->rec.data[rep_inst->stat.rec_data_len-2] );
				if( prv_val < 0 && (prv_val-1) < 0 ) {	//negative and not about to rollover
					store_val.s16[0] = prv_val-1;
					rep_inst->rec.data[ rep_inst->stat.rec_data_len-2 ] = store_val.u8[0];		//lower byte
					rep_inst->rec.data[ rep_inst->stat.rec_data_len-1 ] = store_val.u8[1];		//upper byte
				}
				else {
					//Previous entry is not negative(or it's about to rollover). So Start accumulating in next entry
					store_val.s16[0] = -1;
					rep_inst->rec.data[ rep_inst->stat.rec_data_len ] = store_val.u8[0];		//lower byte
					rep_inst->rec.data[ rep_inst->stat.rec_data_len+1 ] = store_val.u8[1];		//upper byte
					
					rep_inst->stat.rec_data_len += 2;
				}
			}		
			
			//app_trace_log(DEBUG_LOW, "Ave Accel Save: %02u\r", -store_val.s16[0] );		//print positive value
		}
		else {
			//average value converted to user requested scale: +-1G or +-2G
			x = (x/total_samples)*(buf->range/rep_inst->stat.dep_scale);		
			y = (y/total_samples)*(buf->range/rep_inst->stat.dep_scale);
			z = (z/total_samples)*(buf->range/rep_inst->stat.dep_scale);
			
			//Force results to stay within bounds of the Full Scale Range:
			if( x > 0x7FFF )
			{
				x = 0x7FFF;
			}
			else if( x < -0x7FFF )
			{
				x = -0x7FFF;
			}
			if( y > 0x7FFF )
			{
				y = 0x7FFF;
			}
			else if( y < -0x7FFF )
			{
				y = -0x7FFF;
			}
			if( z > 0x7FFF )
			{
				z = 0x7FFF;
			}
			else if( z < -0x7FFF ) 
			{
				z = -0x7FFF;
			}
			
			//Save Results into 16bit holding variable
			store_val.u16[0] = ((z>>1)&0x7C00)|((y>>6)&0x03E0)|((x>>11)&0x001F);	//reposition the 5 MSB bits of each axis into one 16 bit holding variable
			rep_inst->rec.data[ rep_inst->stat.rec_data_len ] = store_val.u8[0];	//lower byte
			rep_inst->rec.data[ rep_inst->stat.rec_data_len+1 ] = store_val.u8[1];	//upper byte
			
			//Update Data Tail
			rep_inst->stat.rec_data_len += 2;
			
			if( false ) {
				x = store_val.u16[0]&0x1F;
				y = (store_val.u16[0]>>5)&0x1F;
				z = (store_val.u16[0]>>10)&0x1F;
				app_trace_puts(DEBUG_LOW, "Ave Accel (x,y,z): ");
				if( x > 15 ) 
				{
					app_trace_log(DEBUG_LOW, "-%02u, ", 32-x );
				}
				else 
				{
					app_trace_log(DEBUG_LOW, " %02u, ", x );
				}
				if( y > 15 ) 
				{
					app_trace_log(DEBUG_LOW, "-%02u, ", 32-y );
				}
				else 
				{
					app_trace_log(DEBUG_LOW, " %02u, ", y );
				}
				if( z > 15 ) 
				{
					app_trace_log(DEBUG_LOW, "-%02u\r", 32-z );
				}
				else 
				{
					app_trace_log(DEBUG_LOW, " %02u\r", z );
				}
			}
			
		}
		
		x = y = z = 0;
		total_samples = 0;
		period_update_time = rep_inst->atts.indep_scalar;
	}
	
	
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint8_t number_fields( T_DATA_FIELDS fields )
{
	uint8_t field_cnt =  0;
	
	if( fields.x > 0 ) field_cnt += 1;
	if( fields.y > 0 ) field_cnt += 1;
	if( fields.z > 0 ) field_cnt += 1;
	if( fields.magnitude > 0 ) field_cnt += 1;
	
	return field_cnt;
}

//! \fn
/// \brief
/// \param
/// \return .
///
void save_fields( T_REPORTER * rep, T_3AXIS * tail)
{
	Union32 u32temp;
	
	if( rep->atts.data_fields.x == 1 ) {
		u32temp.s32 = tail->x;
									
		if( rep->atts.dep_var_type == T_INT16 ) {
			//the only options that could have been accepted are INT16 or INT8
			rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[0];	//lower byte
		}
		rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[1];	//upper byte
	}
	if( rep->atts.data_fields.y == 1 ) {
		u32temp.s32 = tail->y;
									
		if( rep->atts.dep_var_type == T_INT16 ) {
			//the only options that could have been accepted are INT16 or INT8
			rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[0];
		}
		rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[1];
	}
	if( rep->atts.data_fields.z == 1 ) {
		u32temp.s32 = tail->z;
									
		if( rep->atts.dep_var_type == T_INT16 ) {
			//the only options that could have been accepted are INT16 or INT8
			rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[0];
		}
		rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[1];
	}
	if( rep->atts.data_fields.magnitude == 1 ) {
		u32temp.s32 = tail->m;
									
		if( rep->atts.dep_var_type == T_INT16 ) {
			//the only options that could have been accepted are INT16 or INT8
			rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[0];
		}
		rep->rec.data[rep->stat.rec_data_len++] = u32temp.s8[1];
	}
}

///
//! \fn
/// Fit the requested sampling rate into a multiple of the actual sampling Rate
/// \param
/// \return .
///
uint16_t sampling_rate_fit( uint16_t req_rate, uint16_t actual_rate )
{
	uint16_t return_rate;

	if( req_rate >= 3*actual_rate/4 ) {
		//requested sampling rate is at least 75% of the actual sampling rate.	
		return_rate = actual_rate;		//52Hz
	}
	else if( req_rate >= 3*actual_rate/8 ) {
		//requested sampling rate is between 37.5% and 75% of the actual sampling rate.
		return_rate = actual_rate/2;	//26Hz
	}
	else {
		//requested sampling rate is less than 37.5% of the actual sampling rate.
		return_rate = actual_rate/4;	//13Hz
	}
	
	if( return_rate > 4 ) {
		//we could do a rate of 2 or 4 Hz as well		
		if( req_rate <= 1 ) {
			return_rate = 1;
		}
		else if( req_rate <= 2 ) {
			return_rate = 2;
		}
		else if( req_rate <= 4 ) {
			return_rate = 4;
		}
	}
	else {
		if( return_rate == 0 ) return_rate = 1;	//force non-zero
	}
	
	return return_rate;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
static bool buffer_record( T_REPORTER * report ) 
{
	bool over_write_flag = false;
	
	//no limit to the report length, it can overwrite oldest unsent data
	if( report->atts.records_rep == 0 ) over_write_flag = true;
	
	if( (report->stat.rep_len < report->atts.records_rep) || (over_write_flag == true) ) {	
		report->rec.hdr.rec_len = report->stat.rec_data_len+REC_HEADER_LEN;
		if( add_record( &report->rec ) == false ) {
			//mem probably full, report discarded
			if (gs_bdebug) app_trace_log(DEBUG_MED, "Record Save Failed\r");
			return false;
		}
	}
	else {
		//automatically shut reporter Off, it has reached it's limit
		if (gs_bdebug) app_trace_log(DEBUG_MED, "Reporter Maxed, Auto-Off\r");
		active_reports &= ~(1<<report->atts.inst);	//indicate that reporter is Off
		report->stat.enabled = false;
	}
	
	report->stat.rec_data_len = 0;		//0 length, start next Record
	report->stat.rec_saved = true;		//flag to start a new record
	
	//BREAK_POINT;
	return true;
}

// Visible to external modules so they can inform when a record has been added
// (needed by init routine that finds records in memory that are unsent).
void inc_report_length( uint8_t rep_inst )
{
	if( rep_inst >= REP_CNT ) {
		if (gs_bdebug) app_trace_log(DEBUG_MED, "Reports: Instance Error\r");
	}
	else {
		reporter[rep_inst].stat.rep_len++;
	}
}

// Visible to external modules so they can inform when a record has been removed.
// (memory manager indicates when a record in marked as sent of has been thrown away).
void dec_report_length( uint8_t rep_inst )
{
	if( rep_inst >= REP_CNT ) {
		if (gs_bdebug) app_trace_puts(DEBUG_MED, "Reports: Instance Error\r");
	}
	else {
		
		if( reporter[rep_inst].stat.rep_len > 0 ) {
			reporter[rep_inst].stat.rep_len--;
		}
		else {
			if (gs_bdebug) app_trace_log(DEBUG_MED, "Report %01u: Length Error\r", rep_inst);
		}

	}
}

uint16_t get_report_record_cnt( uint8_t inst )
{
	return reporter[inst].stat.rep_len;
}

uint32_t get_total_record_cnt( void )
{
	uint32_t record_totals = 0;
	for( uint i=0; i<REP_CNT; i++ ) {
		record_totals += reporter[i].stat.rep_len;
	}
	
	return record_totals;
}

void power_down_save_records( void )
{
	int i;
	for( i=0; i<REP_CNT; i++ ) {			
		if( reporter[i].stat.rec_data_len > 0 ) {
			//Record has data. Save it before battery goes
			//asm volatile ("nop");	//break point
			if( buffer_record( &reporter[i] ) == false ) {
				//Did not Save
			}
		}
	}
	
	//Force any records that are sitting in RAM input buffer to be backed up in nv memory
	save_buffered_recs();
	
	//Backup Config Variables
	update_config();
}
