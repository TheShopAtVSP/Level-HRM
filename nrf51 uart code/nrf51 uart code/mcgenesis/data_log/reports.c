/*
 * reports.c
 *
 * Created: 3/2/2015 3:27:05 PM
 *  Author: matt
 */ 

#include "reports.h"
#include "mem_manager.h"
#include "../motion/step_detect.h"
#include "battery.h"

extern T_CONFIG g_config;
extern bool gs_bdebug;
extern uint32_t get_unix_time( void ); 
extern T_3AXIS_FIFO accel_buf;
extern T_3AXIS_FIFO gyro_buf;
extern T_3AXIS_FIFO magnet_buf;

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
static uint32_t total_record_cnt;

//elements needed by a step tracking reporter
typedef struct{
	uint32_t start_time;
	uint32_t period;
	int16_t period_sum;			//temporary accumulator
	T_STEP_HISTORY * hist;		//pointer to recent peak history
	uint8_t hist_tail;
	T_STEP_ELEMENT data;		//number of steps detected durig period X
} T_STEP_REPORTER;

static union {
	Bool flag[REP_CNT];
	uint8_t all;
} rebase;

static T_STEP_REPORTER step_report;
static T_STEP_REPORTER x_step_report;
bool accum_step_per_time( T_STEP_REPORTER * );
void accum_3axis_data( T_REPORTER * rep_inst, T_3AXIS_FIFO * buf );
uint16_t sampling_rate_fit( uint16_t req_rate, uint16_t actual_rate );
uint8_t number_fields( T_DATA_FIELDS );
void save_fields( T_REPORTER * rep, T_3AXIS * tail );
void store_record( T_REPORTER * );

///
//! \fn
/// \brief
/// \param
/// \return .
///
void init_reporters( void ) 
{
	uint32_t temp;
	total_record_cnt = 0;
	
	active_reports = 0;
	
	for( uint i=0; i<REP_CNT; i++ )
	{			
		//if (gs_bdebug) app_trace_log("Reporter %01u Init\r", i);
		
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
			if( reporter[i].atts.dependent_data == STEPS_TIME && reporter[i].atts.records_rep == 0 )
			{	//If no limit on Step Reporter, turn it on automatically?
				active_reports |= 1<<i;
			}
			if( reporter[i].atts.dependent_data == EXPERIMENTAL && reporter[i].atts.records_rep == 0 )
			{	//Experiemntal Step Routine
				active_reports |= 1<<i;
			}
			if( reporter[i].atts.dependent_data == BATTERY_REMAINING && reporter[i].atts.records_rep == 0 )
			{	//If no limit on Reporter, turn it on automatically?
				active_reports |= 1<<i;
			}
		}
		else {
			if (gs_bdebug) app_trace_log("Reporter Init Failed\r");
		}
		
		reporter[i].stat.rep_len = temp;
	}
		
	enable_reporters( active_reports );
	//recording.flags = 0;
	
	update_config();	//if backup memory attributes have changed, they will be stored in flash
	
	step_report.hist = step_history();
	x_step_report.hist = x_step_history();
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
	
	//Rebase x_step_report
	x_step_report.start_time += time_shift;
	//adjust Step History buffer timestamps
	tail = x_step_report.hist_tail;
	while( tail != x_step_report.hist->head ) {
		int64_t time = x_step_report.hist->step[tail].time + time_shift;
		if( time < 0 ) time = 0;	//test to make sure no weirdness happens
		x_step_report.hist->step[tail].time = time;
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
		if (gs_bdebug) app_trace_log( "Return Current Atts\r" );
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
			break;
				
		case ACCEL_RAW:
		
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
			
			atts->samples_rec = actual_rate;			//For Raw data, record samples needs to be at least equal to the sampling rate
			reporter[atts->inst].stat.timer_period = 1;	//update reporter every second
			reporter[atts->inst].stat.atts_valid = true;
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
						
			atts->samples_rec = actual_rate;			//For Raw data, record samples needs to be at least equal to the sampling rate
			reporter[atts->inst].stat.timer_period = 1;	//update reporter every second
			reporter[atts->inst].stat.atts_valid = true;
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
						
			atts->samples_rec = actual_rate;			//For Raw data, record samples needs to be at least equal to the sampling rate
			reporter[atts->inst].stat.timer_period = 1;	//update reporter every second
			reporter[atts->inst].stat.atts_valid = true;
			break;
			
		case ACCEL_FILT:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported 
			break;
			
		case GYRO_FILT:
			reporter[atts->inst].stat.atts_valid= false; //data type not yet supported
			break;
		
		case MAGNETO_FILT:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			break;
		
		case ACCEL_VARIANCE:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			break;
			
		case GYRO_VARIANCE:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			break;
			
		case MAGNETO_VARIANCE:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			break;
			
		case BATTERY_REMAINING:
			atts->data_fields.flags = 0;
		
			//Setting to Save Battery Percent Remaining Periodically or on Level Change
			atts->dep_scale = _1_1BIT;			//1%/bit, 1mV/bit
		
			atts->dep_var_type = T_UINT8;
			sample_byte_cnt = 3;										//Remaining Bet Percent (UINT8), Battery Voltage (INT16)
			if( atts->indep_scalar == 0 ) atts->indep_scalar = 1;		//0 not allowed, minimum of 1 time periods
				
			if( atts->independent_data == MINUTES ) {
				//Save Remaining Percent Periodically
				reporter[atts->inst].stat.timer_period = 60*atts->indep_scalar;	//check every X*60 second to update reporter	
			}
			else {
				//Save Remaining Percent when it changes
				atts->independent_data = UNITLESS;
				atts->samples_rec = 1;									//Record is saved On change... Only 1 sample 
				reporter[atts->inst].stat.timer_period = 1;				//check every second to update reporter	
			}
				
			reporter[atts->inst].stat.atts_valid = true;	
			break;
		
		case CYCLES_PER_TIME:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			break;
			
		case TILT_ANGLE:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
			break;

		case BOARD_TEMP:
			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
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
			break;
		
//		case HEADING:
//			reporter[atts->inst].stat.atts_valid = false; //data type not yet supported
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
	else if( atts->samples_rec == 0 ) {
		atts->samples_rec = 1;	//At least 1 sample before saving
	}
	reporter[atts->inst].stat.sample_size = sample_byte_cnt;
	reporter[atts->inst].stat.rec_full_len = atts->samples_rec*sample_byte_cnt;
	
	//make sure the Maximum report length will fit within the limits of allocated memory
	int32_t itemp = (int32_t)atts->records_rep*(reporter[atts->inst].stat.rec_full_len+REC_START_LEN);
	if( itemp > (log_region_size()-REC_FOOTER_LEN) ) {
		atts->records_rep = (log_region_size()-REC_FOOTER_LEN)/(reporter[atts->inst].stat.rec_full_len+REC_START_LEN);
	}

	reporter[atts->inst].stat.rec_len = 0;			//should be 0 already, but just in case
	
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
	
	//app_trace_log( "Attribute Size %02u\r", ATTRIBUTE_SIZE );
			
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
	volatile static uint8_t step_time_reporter = 0xFF;
	volatile static uint8_t xper_reporter = 0xFF;
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
				if( report_on_off > 0 ) {
					if( reporter[i].stat.rep_len <= (unsigned int)(reporter[i].atts.records_rep-1) ) {	//allows reports of 0 length to stay on indef
						if( step_time_reporter >= 0xFF ) {
							//only 1 steps/time reporter can operate at a time. The accum_step_per_time() function
							//is not designed to support more than 1 calling reporter.
							if (gs_bdebug) app_trace_log("Step Reporter On\r");
							
							reporter[i].stat.rec_len = 0;
							reporter[i].stat.time_remain = 0;	//set to trigger immediately
							reporter[i].stat.enabled = true;
						
							//init step tracking variables
							step_report.start_time = start_time;
							step_report.period = reporter[i].atts.indep_scalar;	//save data sample to record at every interval of this period
							if( reporter[i].atts.independent_data == MINUTES ) step_report.period *= 60;	//convert timer period to seconds
							step_report.period_sum = 0;							
							step_report.hist_tail = step_report.hist->head;
							rebase.flag[i] = false;	//clear the rebase flag bit
							//clear step History?
							
							step_time_reporter = i;
						}
					}
				}
				else { 
					if( step_time_reporter == i ) {
						//Requested to turn off Reporter
						step_time_reporter = 0xFF;
						
						//Can't save here, it will stall the processor and right now that is not a good idea
//						if( reporter[i].stat.rec_len > 0 ) {
//							//Save the partially created Record
//							store_record( &reporter[i] );
//						}
						reporter[i].stat.rec_len = 0;
					}
					
					reporter[i].stat.enabled = false;
				}
				break;
				
			case EXPERIMENTAL:
				if( report_on_off > 0 ) {
					if( reporter[i].stat.rep_len <= (unsigned int)(reporter[i].atts.records_rep-1) ) {	//allows reports of 0 length to stay on indef
						if( xper_reporter >= 0xFF ) {
							//only 1 steps/time reporter can operate at a time. The accum_step_per_time() function
							//is not designed to support more than 1 calling reporter.
							if (gs_bdebug) app_trace_log("Step Reporter On\r");
							
							reporter[i].stat.rec_len = 0;
							reporter[i].stat.time_remain = 0;	//set to trigger immediately
							reporter[i].stat.enabled = true;
						
							//init step tracking variables
							x_step_report.start_time = start_time;
							x_step_report.period = reporter[i].atts.indep_scalar;	//save data sample to record at every interval of this period
							if( reporter[i].atts.independent_data == MINUTES ) x_step_report.period *= 60;	//convert timer period to seconds
							x_step_report.period_sum = 0;							
							x_step_report.hist_tail = x_step_report.hist->head;
							rebase.flag[i] = false;	//clear the rebase flag bit
							//clear step History?
							
							xper_reporter = i;
						}
					}
				}
				else { 
					if( xper_reporter == i ) {
						//Requested to turn off Reporter
						xper_reporter = 0xFF;
						
						//Can't save here, it will stall the processor and right now that is not a good idea
//						if( reporter[i].stat.rec_len > 0 ) {
//							//Save the partially created Record
//							store_record( &reporter[i] );
//						}
						reporter[i].stat.rec_len = 0;
					}
					
					reporter[i].stat.enabled = false;
				}
				break;
			
			//the majority of cases have no special requirements and simply need to turn on
			default:
				if( report_on_off > 0 ) {
					if( reporter[i].stat.rep_len <= (unsigned int)(reporter[i].atts.records_rep-1) ) {	//allows reports of 0 length to stay on indef
						if( reporter[i].stat.enabled == false ) {
							reporter[i].stat.rec_len = 0;
							reporter[i].stat.time_remain = 0;	//set to trigger immediately
							reporter[i].stat.data_fifo_tail = 0;
							reporter[i].stat.new_start = true;
							reporter[i].stat.enabled = true;
							reporter[i].stat.decimator_cnt = 0;
						
							rebase.flag[i] = false;
						}
					}
				}
				else {
					//Can't save here, it will stall the processor and right now that is not a good idea
//					if( reporter[i].stat.rec_len > 0 ) {
//						//Save the partially created Record
//						store_record( &reporter[i] );
//					}
					reporter[i].stat.rec_len = 0;
					
					reporter[i].stat.enabled = false;
				}
				break;
		}	//end switch
			
		if( reporter[i].stat.enabled == true ) act_reporters |= report_on_off;	//indicate Report[i] is On
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
void check_reporters( void ) 
{
	Union32 u32temp;
	uint32_t unix_time = get_unix_time();	//stamp coming into function
	T_ACTIVE_SENSORS act_axes = { .accel.flags = 0 };
		
	get_active_axes( &act_axes );
	
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
						if( accum_step_per_time( &step_report ) == true ) {		
							static uint8_t zero_cnt = 0;		

							//don't start a record unless steps are detected or a record has already started!!!						
							if( step_report.data.cnt > 0 || reporter[i].stat.rec_len > 0 ) {
								//time to log some data...
								//if (gs_bdebug) app_trace_log("Append Step Data!!!\r");
								
								if( reporter[i].stat.rec_len == 0 ) {
									//start of new record
									zero_cnt = 0;
									reporter[i].rec.hdr.time = step_report.data.time_stamp;
								}
							
								//add data to record
								u32temp.u32 = step_report.data.cnt;
								for( int j=0; j<reporter[i].stat.sample_size; j++ ) {
									reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.u8[j];
								}
							
								//if 12 zeros in a row are recorded, save the partial record to conserve space.
								if( u32temp.u32 == 0 ) {
									zero_cnt += reporter[i].stat.sample_size;
									if(  zero_cnt >= REC_START_LEN ) {
										reporter[i].stat.rec_len -= zero_cnt;	//remove tht last X zeros, no need to save them.
										store_record( &reporter[i] );
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
						//If local time has been changed. Start a new Record to reflect new time
						if( rebase.flag[i] == true ) {
							if (gs_bdebug) app_trace_log("Time update. Saving Reporter\r");
							rebase.flag[i] = false;
							if( reporter[i].stat.rec_len > 0 ) {
								//save portion of record that has been created
								store_record( &reporter[i] );
							}
						}
	
						accum_3axis_data( &reporter[i], &accel_buf );
						break;
								
					case GYRO_RAW:
						//If local time has been changed. Start a new Record to reflect new time
						if( rebase.flag[i] == true ) {
							rebase.flag[i] = false;
							if( reporter[i].stat.rec_len > 0 ) {
								//save portion of record that has been created
								store_record( &reporter[i] );
							}
						}
						
						accum_3axis_data( &reporter[i], &gyro_buf );
						break;
								
					case MAGNETO_RAW:
						//If local time has been changed. Start a new Record to reflect new time
						if( rebase.flag[i] == true ) {
							rebase.flag[i] = false;
							if( reporter[i].stat.rec_len > 0 ) {
								//save portion of record that has been created
								store_record( &reporter[i] );
							}
						}
						
						accum_3axis_data( &reporter[i], &magnet_buf );
						break;
								
					case ACCEL_FILT:
						break;
								
					case GYRO_FILT:
						break;
								
					case MAGNETO_FILT:
						break;
								
					case ACCEL_VARIANCE:
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
						
						if( reporter[i].stat.rec_len == 0 ) {
							//start of new record
							reporter[i].rec.hdr.time = unix_time;
						}
						
						if( reporter[i].atts.independent_data == MINUTES ) {
							//Periodically save remaining Battery Percent 
							u32temp.u8[0] = battery_get_level();
							reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.u8[0];	
							
							//Periodicaly save Battery Voltage Reading
							u32temp.s32 = battery_get_voltage();
							//the only options that could have been accepted are INT16
							reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.s8[0];
							reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.s8[1];
						}
						else {
							//Save remaining Battery Percent when it changes
							if( battery_level_change( u32temp.u8 ) == true ) {
								reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.u8[0];
								
								//Periodicaly save Battery Voltage Reading
								u32temp.s32 = battery_get_voltage();
								//the only options that could have been accepted are INT16
								reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.s8[0];
								reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.s8[1];
							}
						}

						break;
								
					case CYCLES_PER_TIME:
						break;
								
					case TILT_ANGLE:
						break;
								
					case BOARD_TEMP:
						break;
					
					case EXPERIMENTAL:
						// Testing Ground for new record types
						//Check for step updates
						if( accum_step_per_time( &x_step_report ) == true ) {		
							static uint8_t x_zero_cnt = 0;		

							//don't start a record unless steps are detected or a record has already started!!!						
							if( x_step_report.data.cnt > 0 || reporter[i].stat.rec_len > 0 ) {
								//time to log some data...
								//if (gs_bdebug) app_trace_log("Append Step Data!!!\r");
								
								if( reporter[i].stat.rec_len == 0 ) {
									//start of new record
									x_zero_cnt = 0;
									reporter[i].rec.hdr.time = x_step_report.data.time_stamp;
								}
							
								//add data to record
								u32temp.u32 = x_step_report.data.cnt;
								for( int j=0; j<reporter[i].stat.sample_size; j++ ) {
									reporter[i].rec.data[reporter[i].stat.rec_len++] = u32temp.u8[j];
								}
							
								//if 12 zeros in a row are recorded, save the partial record to conserve space.
								if( u32temp.u32 == 0 ) {
									x_zero_cnt += reporter[i].stat.sample_size;
									if(  x_zero_cnt >= REC_START_LEN ) {
										reporter[i].stat.rec_len -= x_zero_cnt;	//remove tht last X zeros, no need to save them.
										store_record( &reporter[i] );
									}
								}
								else {
									x_zero_cnt = 0;
								}
							}
						
						}
						rebase.flag[i] = false;	//time was rebased when rebase function was called
						break;
	
					default:
						break;
				}
			}
								
			if( (reporter[i].stat.rec_len+reporter[i].stat.sample_size) > reporter[i].stat.rec_full_len ) {
				//Record is full, time to save...
				//asm volatile ("nop");	//break point
				store_record( &reporter[i] );
			}
		}
	}
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
			
			if (gs_bdebug) app_trace_log("Curious?\r");
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
				if (gs_bdebug) app_trace_log("Curiouser & Curiouser???\r");
				
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

void accum_3axis_data( T_REPORTER * rep_inst, T_3AXIS_FIFO * buf )
{
	uint32_t unix_time = get_unix_time();	//stamp coming into function
	
	//When reporter first starts up, Zero out the recent data buffer
	if( rep_inst->stat.new_start == true ) {
		rep_inst->stat.new_start = false;
		rep_inst->stat.data_fifo_tail = buf->head;	//zero out the FIFO data
	}
	
	if( buf->active_axis.flags > 0 ) {
		//Sensor On!
		//if (gs_bdebug) app_trace_log("Save Compass Data!!!\r");
		
		if( rep_inst->stat.rec_len == 0 ) {
			//start of new record
			rep_inst->rec.hdr.time = unix_time;
			
		}
	
		//incorporate the sub sampling rate
		uint8_t sub_samp_rate = buf->rate/rep_inst->atts.indep_scalar;
		if( sub_samp_rate < 1 ) sub_samp_rate = 1;
		
		rep_inst->stat.decimator_cnt = 0;
		while( ((buf->head - rep_inst->stat.data_fifo_tail)&IMU_FIFO_MASK) >= sub_samp_rate &&
				(rep_inst->stat.rec_len + rep_inst->stat.sample_size) <= REC_MAX_LEN ) 
		{
			//Only save the data every X number of samples
			rep_inst->stat.decimator_cnt++;
			//Save data at subsampled rate.
			if( rep_inst->stat.decimator_cnt == sub_samp_rate ) {
				T_3AXIS ave;
				int32_t x=0, y=0, z=0, m=0;
				int16_t tail;
				
				rep_inst->stat.decimator_cnt = 0;
					
				//Pass through simple Decimation Filter
				tail = rep_inst->stat.data_fifo_tail;
				for(int j=0; j<sub_samp_rate; j++){
					x += buf->fifo[tail].x;
					y += buf->fifo[tail].y;
					z += buf->fifo[tail].z;
					m += buf->fifo[tail].m;
					
					tail -= 1;
					tail &= IMU_FIFO_MASK;
				}
				
				//Determines average and transforms result to the User requested scale (could be different than recorded scale)
				ave.x = buf->range*(x/sub_samp_rate)/rep_inst->stat.dep_scale;	
				ave.y = buf->range*(y/sub_samp_rate)/rep_inst->stat.dep_scale;
				ave.z = buf->range*(z/sub_samp_rate)/rep_inst->stat.dep_scale;
				ave.m = buf->range*(m/sub_samp_rate)/rep_inst->stat.dep_scale;
				
				save_fields( rep_inst, &ave );
			}
			
			//Update Data Tail
			rep_inst->stat.data_fifo_tail += 1;
			rep_inst->stat.data_fifo_tail &= IMU_FIFO_MASK;
		}

	}
	else {
		//Sensor is Off!
		rep_inst->stat.data_fifo_tail = buf->head;
		rep_inst->stat.rec_len = 0;		//discard partial record
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
			rep->rec.data[rep->stat.rec_len++] = u32temp.s8[0];	//lower byte
		}
		rep->rec.data[rep->stat.rec_len++] = u32temp.s8[1];	//upper byte
	}
	if( rep->atts.data_fields.y == 1 ) {
		u32temp.s32 = tail->y;
									
		if( rep->atts.dep_var_type == T_INT16 ) {
			//the only options that could have been accepted are INT16 or INT8
			rep->rec.data[rep->stat.rec_len++] = u32temp.s8[0];
		}
		rep->rec.data[rep->stat.rec_len++] = u32temp.s8[1];
	}
	if( rep->atts.data_fields.z == 1 ) {
		u32temp.s32 = tail->z;
									
		if( rep->atts.dep_var_type == T_INT16 ) {
			//the only options that could have been accepted are INT16 or INT8
			rep->rec.data[rep->stat.rec_len++] = u32temp.s8[0];
		}
		rep->rec.data[rep->stat.rec_len++] = u32temp.s8[1];
	}
	if( rep->atts.data_fields.magnitude == 1 ) {
		u32temp.s32 = tail->m;
									
		if( rep->atts.dep_var_type == T_INT16 ) {
			//the only options that could have been accepted are INT16 or INT8
			rep->rec.data[rep->stat.rec_len++] = u32temp.s8[0];
		}
		rep->rec.data[rep->stat.rec_len++] = u32temp.s8[1];
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
		return_rate = actual_rate;
	}
	else if( req_rate >= 3*actual_rate/8 ) {
		//requested sampling rate is between 37.5% and 75% of the actual sampling rate.
		return_rate = actual_rate/2;
	}
	else {
		//requested sampling rate is less than 37.5% of the actual sampling rate.
		return_rate = actual_rate/4;
	}
	
	if( return_rate == 0 ) return_rate = 1;	//force non-zero
	
	return return_rate;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void store_record( T_REPORTER * report ) 
{
	Bool over_write_flag = false;
	
	//no limit to the report length, it can overwrite oldest unsent data
	if( report->atts.records_rep == 0 ) over_write_flag = true;
	
	if( (report->stat.rep_len < report->atts.records_rep) || (over_write_flag == true) ) {	
		report->rec.hdr.rec_len = report->stat.rec_len+REC_HEADER_LEN;
		if( push_record( &report->rec, over_write_flag ) == false ) {
			//mem probably full, report discarded
			if (gs_bdebug) app_trace_log("Record Discarded\r");
		}
	}
	else {
		//automatically shut reporter Off, it has reached it's limit
		if (gs_bdebug) app_trace_log("Reporter Maxed, Auto-Off\r");
		active_reports &= ~(1<<report->atts.inst);	//indicate that reporter is Off
		report->stat.enabled = false;
	}
	
	report->stat.rec_len = 0;	//0 length, start next Record
	
	//asm volatile ("nop");	//break point
}

void inc_report_length( uint8_t rep_inst )
{
	if( rep_inst >= REP_CNT ) {
		if (gs_bdebug) app_trace_log("Reports: Instance Error\r");
	}
	else {
		reporter[rep_inst].stat.rep_len++;
		total_record_cnt++;
	}
}

void dec_report_length( uint8_t rep_inst )
{
	if( rep_inst >= REP_CNT ) {
		if (gs_bdebug) app_trace_log("Reports: Instance Error\r");
	}
	else {
		
		if( reporter[rep_inst].stat.rep_len > 0 ) {
			reporter[rep_inst].stat.rep_len--;
		}
		else {
			if (gs_bdebug) app_trace_log("Reports: Length Error\r");
		}
		
		if( total_record_cnt > 0 ) {
			total_record_cnt--;
		}
		else {
			if (gs_bdebug) app_trace_log("Reports: Total Records Error\r");
		}
	}
}

uint16_t get_report_record_cnt( uint8_t inst )
{
	//check that the record count is operating correctly
	if( memory_empty() == true ) {
		if( total_record_cnt != 0 ) {
			//This should be 0
			if (gs_bdebug) app_trace_log("reports: RECORD COUNT ERROR!!!\r");
			total_record_cnt = 0;
			for( uint i=0; i<REP_CNT; i++ ) {
				reporter[i].stat.rep_len = 0;
			}
		}
	}
	
	return reporter[inst].stat.rep_len;
}

uint16_t get_total_record_cnt( void )
{
	//check that the record count is operating correctly
	if( memory_empty() == true ) {
		if( total_record_cnt != 0 ) {
			//This should be 0
			if (gs_bdebug) app_trace_log("reports: RECORD COUNT ERROR!!!\r");
			total_record_cnt = 0;
			for( uint i=0; i<REP_CNT; i++ ) {
				reporter[i].stat.rep_len = 0;
			}
		}
	}
	
	return total_record_cnt;
}
