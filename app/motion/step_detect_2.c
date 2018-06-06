/// \file step_detect.c
///
/// Algorithm looks to accelerometer data to determine when a person is moving
/// and keeps a record of the number of steps.
///
/// Copyright (C) 2014 VSP Global\n
///
/// \author Matt Workman \n

#include <stdlib.h>
#include <math.h>
#include "step_detect.h"

#define MIN_DELTA_MAG		4					//Minimum Change (as a % of 1g) to be considered a Step

//Step Signal Timing Defines
#define MIN_TIME_MAX2MAX    160L                //Shortest time to measure between Maximas
#define MIN_TIME_MIN2MIN    MIN_TIME_MAX2MAX    //Should be same as MAXIMA to MAXIMA
#define MAX_TIME_MAX2MAX    1500L               //Walking slower than this period is = to sleeping... or at least it should be, so ignore it!
#define MAX_TIME_MIN2MIN    MAX_TIME_MAX2MAX    //Same as MAXIMA to MAXIMA
#define MIN_TIME_MAX2MIN    10L					//Minima must come at least a short while after a Maxima
#define MIN_TIME_MIN2MAX    10L					//Maxima must come at least a short while after a Minima
#define MAX_TIME_MAX2MIN    (MAX_TIME_MAX2MAX - MIN_TIME_MIN2MAX)	//Minima must come before the next Maxima is expected
#define MAX_TIME_MIN2MAX    (MAX_TIME_MIN2MIN - MIN_TIME_MAX2MIN)	//Maxima must come before the next Minima is expected

//Make sure the MAXIMA to MINIMA timeout does not exceed the MAXIMA to MAXIMA timeout
#if MAX_TIME_MAX2MIN > MAX_TIME_MAX2MAX
#error "MAX_TIME_MAX2MIN >  MAX_TIME_MAX2MAX"
#endif
//Make sure the MINIMA to MAXIMA timeout does not exceed the MINIMA to MINIMA timeout
#if MAX_TIME_MIN2MAX > MAX_TIME_MIN2MIN
#error "MAX_TIME_MIN2MAX > MAX_TIME_MIN2MIN"
#endif

//Standard deviation values
#define STD_DEV_THRES		20		//Maximum Standard Deviation Threshold to consider peaks as "walking" (100 * sd)
#define BLOCK_ACCUM_CNT		4		//Prevent Steps from counting for at least x steps
#define FASTEST_CADENCE		222		//can only take so many steps/second (1/0.222 = 4.5 steps/sec)

extern uint32_t get_unix_time( void ); 
static bool x_mot_debug = true;
static uint32_t inst_per_amp_thres;
static uint32_t ave_per_amp_thres;
static uint32_t driving_noise_thres;
extern uint8_t hrm_step;

#define X_SAMP_SIZE	4
static struct {
	uint16_t u1gVal;			//sensitivity setting for accelerometer (+-2g, +-4g, +-8g, +-16g)
	uint32_t uDynamicDeltaMag;	//Dynamic Threshold of change in Magnitude to detect Min and Maxima
    uint32_t uMinDeltaMag;		//Smallest Value allowed for DynamicDeltaMag
	T_FEATURE Minima;
	T_FEATURE Maxima;
	T_FEATURE possMinima;
	T_FEATURE possMaxima;
	uint32_t BlockAccum;
	uint32_t Startup_Count;
	uint32_t Rel_Count;		//Relative to the last time count was cleared
	uint32_t Abs_Count;		//Never cleared, used to track Steps/Minute
	struct {
		uint32_t ptr;
		uint32_t periods[X_SAMP_SIZE];	//time between step peaks in milliseconds
		uint32_t amplitudes[X_SAMP_SIZE];	//highest value of Peak in g-scale
		uint32_t per_ave;				//average of the last X Step peak periods
		uint32_t per_var;				//the variance in those periods
		uint32_t per_sd;				//100 times the standard deviation of the last X Peak Periods
		uint32_t amp_ave;				//the average of the last X Step peak amplitudes
		int16_t qual_figure;			//Variable that tracks relative good vs. poor cadence
	} Cadence;
} Step = { 0 };		

static T_STEP_HISTORY recent_history;

static void clrRecentSteps( void );
static bool mrs_butterworth_bpf_init(float samp_freq, float l_cut_off, float h_cut_off);
static bool mrs_butterworth_filter(int32_t * x);


///
//! \fn
/// \brief
/// \param
/// \return .
///
void initStepDetect_2( bool debug )
{	
	uint32_t i;
	uint32_t sample_rate;
	x_mot_debug = debug;
	
    //initialize variables
	Step.u1gVal = get_accel_sens();
    Step.uMinDeltaMag = (uint32_t) ((uint32_t)Step.u1gVal*MIN_DELTA_MAG) / 100; //Convert to Percent of 1g
    Step.uDynamicDeltaMag = Step.uMinDeltaMag;
	inst_per_amp_thres = 0.02*Step.u1gVal;	//threshold used to weed out single small spurious peaks
	ave_per_amp_thres = 0.035*Step.u1gVal;	//threshold used to weed out a train of small spurious peaks
	driving_noise_thres = 0.3*Step.u1gVal;	//More than 99% of driving Peaks sit well below this threshold
	
	Step.possMinima.mag = 0;
	Step.possMaxima.mag = 0;
	Step.Minima.mag = 0;
    Step.Maxima.mag = 0;
	Step.Minima.time = 0;
    Step.Maxima.time = 0;
	Step.BlockAccum = BLOCK_ACCUM_CNT;
	Step.Startup_Count = 0;
	//Step.Rel_Count = 0;		//Only Clear on User Request
	//Step.Abs_Count = 0;		//Never Clear after Startup
	
	for(i=0; i<X_SAMP_SIZE; i++)
	{
		Step.Cadence.periods[i] = 1000;	//start all Periods at 0
		Step.Cadence.amplitudes[i] = 0;	//start all Amplitudes at 0
	}
	Step.Cadence.ptr = 0;
	Step.Cadence.per_ave = 1000;
	Step.Cadence.per_var = 0;		//Ave time between foot strikes
	Step.Cadence.per_sd = 0;
	Step.Cadence.amp_ave = 0;
	Step.Cadence.qual_figure = 0;

	clrRecentSteps();
	
	sample_rate = get_accel_sample_rate();
	mrs_butterworth_bpf_init(sample_rate, 0.33, 8);	//Bandpass Cutoff Fequencies: 0.33Hz, 8Hz
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
static void clrRecentSteps( void )
{
	for(int i=0; i<SH_BUF_MASK; i++ ){
		recent_history.step[i].type = FALSE_STEP;
		recent_history.step[i].time = 0;
	}
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
T_STEP_HISTORY * step_history_2( void ) 
{
	return &recent_history;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t getStartupCnt_2( void )
{
    return Step.Startup_Count;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t getStepCnt_2( void )
{
	return Step.Rel_Count;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void setStepCnt_2( uint32_t step )
{
	Step.Rel_Count = step;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t getAbsoluteCnt_2( void )
{
	return Step.Abs_Count;
}

// Note: This algorithm was largely developed and verified in the C# project Gen 
// Analysis. The algorithm developed there was tested against raw Accel and Gyro
// Data obtained from a 3rd party IMU device.
// Step Detection from Accel Magnitude Signal
/*               Step 1     Step 2               Step X
 *                /\        /\                   /\
 *               /  |      /  |                 /  |
 * -------|   /-/   |   /-/   |   /......|   /-/   |
 *        |  /      |  /      |  /       |  /      |
 *         \/        \/        \/         \/        \
 */
static TSTEP_DET_STATE stepDetectSV = INIT;
TSTEP_FEEDBACK stepStateMachine_2( uint32_t time, int32_t mag )
{
	TSTEP_FEEDBACK res = NOTHING;
	int32_t iDynamicThres;
	volatile TSTEP_DET_STATE copy_stepDetectSV = NA;
	static T_FEATURE prv = {0};
	int32_t delta_mag;
	uint32_t peak_period;
	uint32_t ave_amp = 0, ave_per = 0, squares_per = 0;
	uint32_t per_amp_check;
	int i;
	
	//Push Data through Bandpass filter
	mrs_butterworth_filter( &mag );
	
	delta_mag = mag - prv.mag;
	
	//Need to execute State Machine until the State variable is settled. A Maxima and
    //Minima inflection may come in back to back points, so the state variable may need
    //to make multiple transitions between adjacent sample points.
    while (copy_stepDetectSV != stepDetectSV)
    {
        copy_stepDetectSV = stepDetectSV;

        switch (stepDetectSV) {
            case INIT:
                delta_mag = 0;    			//Prevent the slope from being used until the next data point...
                res = STEP_FILTER_CLR;
				stepDetectSV = UPDATE;
                break;

            case FIND_SLOPE:
				if( time < Step.Minima.time || time < Step.Maxima.time ) {
					//Something has gone awry with the period timers (time overflows are possible: 49.7 days)...
					if (x_mot_debug) {
						app_trace_log(DEBUG_LOW, "step: Timers Invalid!\r");	
					}
					Step.Maxima.time = Step.Minima.time = 0;
				}
				
                //Determine which type of point we need to look for...
                if (delta_mag > 0) 
                {   // positive sloping
                    //Set Potential to Value less than the coming Maxima.
                    //+Derivative means currentMag is already > prevMag, so prevMag is not going to be the Maxima!
                    //Don't want to miss the real Maxima by guessing a value > whatever the Maxima ends up being.
                    Step.possMaxima.mag = prv.mag;
                    stepDetectSV = MAXIMA_HUNT; //now go wait for a possible Maxima!
                }
                else if (delta_mag < 0) 
                {   // negative sloping
                    //Set Potential to Value greater than the coming Minima.
                    //-Derivative means currentMag is already < prevMag, so prevMag is not going to be the Minima!
                    //Don't want to miss the real Minima by guessing a value < whatever the Minima ends up being.
                    Step.possMinima.mag = prv.mag;
                    stepDetectSV = MINIMA_HUNT; //now go wait for a possible Minima!
                }
                break;

            case MINIMA_HUNT: //Waiting for a Possible Minima Inflection
				iDynamicThres = -Step.uMinDeltaMag;
			
                if (delta_mag > 0) //increasing slope
                {   //Inflection found!
					if ( prv.mag < iDynamicThres )
					{
						//Save the lowest Inflection point that occurs between Maximas
						if (prv.mag <= Step.possMinima.mag)
						{
							Step.possMinima.mag = prv.mag; 		//keep new potential minima
							Step.possMinima.time = prv.time;	//previous mag time
							res = POSSIBLE_MINIMA;
						}

						stepDetectSV = VERIFY_MINIMA; //Wait for Signal to rise further than DynamicDeltaMagnitude or restarts falling
					}
					else if (Step.possMinima.mag < prv.mag)
					{
						stepDetectSV = VERIFY_MINIMA;
					}
					else
					{
						stepDetectSV = FIND_SLOPE; //Wait for Signal to rise further than DynamicDeltaMagnitude or restarts falling
					}
                }
                break;

            case MAXIMA_HUNT: //Waiting for a Possible Maxima Inflection
				iDynamicThres = Step.uMinDeltaMag;
			
				if (delta_mag < 0) //decreasing slope
				{   //Inflection found!
					if( prv.mag > iDynamicThres )
					{
						if( (prv.time - Step.possMaxima.time) <= 125 )
						{	//Peaks this close together tend to be double strikes of the same step. And if these
                            //peaks are of similar height, the algorithm can count either one (and only one) per step.
                            //This leads to a higher standard deviation, averaging these similar peaks can lower the sd.
							int32_t double_peak_test = 5 * (prv.mag - Step.possMaxima.mag);
							if (double_peak_test < 0)
							{
								double_peak_test *= -1;
							}

							if (prv.mag >= double_peak_test || Step.possMaxima.mag >= double_peak_test)
							{	//Difference between thes peaks is < 20%.  So average them together. This leads to a lower
                                //standard deviation for steps that exhibit a double peak
								Step.possMaxima.mag = ( prv.mag + Step.possMaxima.mag )/2;
								Step.possMaxima.time = ( prv.time + Step.possMaxima.time )/2;
								res = POSSIBLE_MAXIMA;
							}
							else if (prv.mag > Step.possMaxima.mag)
							{	//Most recent peak is a lot bigger, use it as the possible maxima
								Step.possMaxima.mag = prv.mag;
								Step.possMaxima.time = prv.time;
								res = POSSIBLE_MAXIMA;
							}
							else
							{	//Older peak is a lot bigger. Keep it as the possible maxima
								res = MAXIMA_TOO_SOON;
							}
						}
						else if ( prv.mag >= Step.possMaxima.mag )
						{	//Save the highest Inflection point that occurs between Minimas
							Step.possMaxima.mag = prv.mag;
							Step.possMaxima.time = prv.time;
							res = POSSIBLE_MAXIMA;
						}

						stepDetectSV = VERIFY_MAXIMA; //Wait for Signal to fall further than DynamicDeltaMagnitude or restarts rising
					}
					else
					{
						if (time >= (Step.Maxima.time + MAX_TIME_MAX2MAX))
						{   //Too long since last confirmed Peak. If there are any Startup steps, clear them and start fresh
							if (Step.Startup_Count != 0)
							{
								res = STEP_FILTER_CLR;
								stepDetectSV = UPDATE;
							}

						}
						else if (Step.possMaxima.mag > prv.mag)
						{
							//Go back to verifying previous Potential Maxima
							stepDetectSV = VERIFY_MAXIMA;
						}
						else
						{
							//res = TSTEP_DEBUG.MAXIMA_TOO_SMALL;
							stepDetectSV = FIND_SLOPE;
						}
					}
				}
                break;

            case VERIFY_MINIMA:
                iDynamicThres = 0;

				if (delta_mag < 0)
				{   //We're on our way back Down, return to looking for the new potential Minima.
					//There has been a local maxima that was not big enough to consider
					//a step, but maybe it was...

					stepDetectSV = MINIMA_HUNT;
				}
				else if (mag > iDynamicThres)
				{
					Step.Minima.mag = Step.possMinima.mag;	//looks like it was an actual Minima
					Step.Minima.time = Step.possMinima.time;
					res = MINIMA_VALIDATED;

					stepDetectSV = FIND_SLOPE; 				//Start looking for the coming Maxima!!!
				}
                break;

            case VERIFY_MAXIMA:				
				//Keep Accelerometer On for X seconds when significant motion is detected
				keep_accel_on();
				
				iDynamicThres = 0;

                if (delta_mag > 0) 
                {   //We're on our way back Up, return to looking for the new potential maxima
                    //There has been a local minima that may correspond to a missed step...
                    stepDetectSV = MAXIMA_HUNT;
                } 
				else if ( time > (Step.possMaxima.time + MAX_TIME_MAX2MIN) )
                {   
					if (x_mot_debug) {
						app_trace_log(DEBUG_LOW, "step: Max TO\r" );	
					}
					Step.possMaxima.mag = 0;		//accel.ave
					Step.possMaxima.time = time;
					
					res = STEP_FILTER_CLR;
                    stepDetectSV = UPDATE;
                }
                else if ( mag < iDynamicThres )
                {
					stepDetectSV = PEAK_STATS;
				}
				
				break;	
				
			case PEAK_STATS:
			
				//Start by checking if last 2 peaks match well
				peak_period = Step.possMaxima.time - Step.Maxima.time;

				if (peak_period > MAX_TIME_MAX2MAX)
				{
					if (x_mot_debug) {
						app_trace_log(DEBUG_LOW, "step: Too Long\r");	
					}
					//Way too long. Save new Stats and wait for next peak
					Step.Maxima.time = Step.possMaxima.time;   //update timestamp
					Step.Maxima.mag = Step.possMaxima.mag;
					Step.Cadence.amplitudes[Step.Cadence.ptr] = Step.Maxima.mag;
					Step.Cadence.periods[Step.Cadence.ptr] = MAX_TIME_MAX2MAX;  //Don't know the period yet, but assume the slowest
					
					res = STEP_FILTER_CLR;
					stepDetectSV = UPDATE;
					break;
				}

				if (peak_period < MIN_TIME_MAX2MAX)
				{
					if (x_mot_debug) {
						app_trace_log(DEBUG_LOW, "step: Too Soon\r");	
					}
					//Another Step Peak can't occur this fast. Likely part of the same step.
					if( Step.possMaxima.mag > Step.Maxima.mag )
					{	//This second peak is larger than one that was just saved. To get a more accurate representation of the Cadence,
						//the lasted saved step time and amplitude will be adjusted based to this new peak.
						uint32_t time_adjust = (Step.possMaxima.time - Step.Maxima.time) / 2;
						
						Step.Maxima.time += time_adjust;
						Step.Maxima.mag = Step.possMaxima.mag;
						Step.Cadence.amplitudes[Step.Cadence.ptr] = Step.Maxima.mag;
						Step.Cadence.periods[Step.Cadence.ptr] += time_adjust;
					}

					res = MAXIMA_TOO_SOON;
					stepDetectSV = FIND_SLOPE;
					break;
				}

				//peak_period is in mSec, have to account for the 2 factors of 1000
				per_amp_check = (1000*inst_per_amp_thres) / ((peak_period*peak_period)/(1000));
				if ((Step.possMaxima.mag) < per_amp_check)
				{
					if (x_mot_debug) {
						app_trace_log(DEBUG_LOW, "step: Too Small\r");	
					}
					//peak is too small to come this fast on the heels of another step peak
					res = MAXIMA_TOO_SMALL;
					stepDetectSV = FIND_SLOPE;
					break;
				}
				
				//Include Step Maxima In Cadence Stats
				//Make sure Gyro is on for at least the next few seconds to help with step detection
				keep_gyro_on();
				//keep_magnet_on();	//temporarily turn on Magnetometer for testing
				
				Step.Maxima.mag = Step.possMaxima.mag;
				Step.Maxima.time = Step.possMaxima.time;
				
				if( ++Step.Cadence.ptr >= X_SAMP_SIZE ) Step.Cadence.ptr = 0;
				Step.Cadence.amplitudes[ Step.Cadence.ptr ] = Step.Maxima.mag;
				Step.Cadence.periods[ Step.Cadence.ptr ] = peak_period;
				
				//Light Statical Analysis...
				for( i=0; i<X_SAMP_SIZE; i++ ) {
					ave_amp += Step.Cadence.amplitudes[i];
					ave_per += Step.Cadence.periods[i];
					squares_per += Step.Cadence.periods[i]*Step.Cadence.periods[i];
				}
				ave_amp /= X_SAMP_SIZE;
				ave_per /= X_SAMP_SIZE;
				squares_per /= X_SAMP_SIZE;
				Step.Cadence.amp_ave = ave_amp;
				Step.Cadence.per_ave = ave_per;
				Step.Cadence.per_var = squares_per - ave_per*ave_per;    //Variance = (Average of Squares) - (Square of the Average)
				Step.Cadence.per_sd = 100*fast_sqrt( Step.Cadence.per_var )/ave_per;	//100 * normalized Standard Deviation

				if (Step.Cadence.amp_ave > driving_noise_thres)
				{   //Have yet to see very high amplitude driving noise. So when the signal is strong, we don't have to 
					//rely solely on Standard Deviation as an indication of "Walking"
					Step.Cadence.qual_figure += STD_DEV_THRES;
				}
				else
				{
					//The SD value of 20 was empirical derived from the Driving data. Most of which rarely drops below 
					//0.2 Standard Deviations for long. Most Walking, on the other hand, has a low standard deviation
					//(often less than 0.05). So after a few steps the quality figure tends to grow to its max.
					if (Step.Cadence.per_sd > STD_DEV_THRES)
					{   //Reduce Figure much faster than it grows
						Step.Cadence.qual_figure -= 4*(Step.Cadence.per_sd - STD_DEV_THRES);
					}
					else
					{
						Step.Cadence.qual_figure += STD_DEV_THRES - Step.Cadence.per_sd;
					}
				}

				if( Step.Cadence.qual_figure > (4*STD_DEV_THRES) )
				{
					Step.Cadence.qual_figure = 4*STD_DEV_THRES;
				}
				else if( Step.Cadence.qual_figure < 0 )
				{
					Step.Cadence.qual_figure = 0;
				}
				
				//Try to Block False Steps by requiring X steps with a stable Cadence before starting the count
				if( Step.Cadence.qual_figure < (4*STD_DEV_THRES) )
				{   // The standard devaition in the step cadence is relatively high, block step accumulation
					Step.BlockAccum = BLOCK_ACCUM_CNT;
				}
				else if( Step.Cadence.amplitudes[Step.Cadence.ptr] < (Step.Cadence.amp_ave/2) )
				{
					//peak too much smaller than the recent history, briefly block step accumulation
					Step.BlockAccum = BLOCK_ACCUM_CNT;
				}

				stepDetectSV = PEAK_TRACK;
				//break;		//go straigt into the next state
					
			case PEAK_TRACK:
				

				//Don't Count Steps until Good Cadence and Minimum Successive Steps have been met
				if( Step.BlockAccum > 0 )
				{		
					Step.BlockAccum--;
					res = STEP_FILTER_ADD;
				}
				else
				{
					//Step.Cadence.per_ave is in mSec, have to account for the 2 factors of 1000
					uint32_t per_amp_check = (1000*ave_per_amp_thres) / ((Step.Cadence.per_ave*Step.Cadence.per_ave)/(1000));
					if( Step.Cadence.per_ave < FASTEST_CADENCE )
					{   //Frequency too fast for bipedal motion
						if (x_mot_debug) {
							app_trace_log(DEBUG_LOW, "step: Cadence Too Fast\r");	
						}
						res = STEP_FILTER_CLR;
					}
					else if( Step.Cadence.amp_ave < per_amp_check )
					{   //Amplitude is not high enough to match this frequency...
						//Removes some instances of false positives while driving
						if (x_mot_debug) {
							app_trace_log(DEBUG_LOW, "step: Cadence Too Small\r");	
						}
						res = STEP_FILTER_CLR;
					}
					else
					{	//all the conditions are met, count away!
						res = STEP_CNT_ADD;
					}
				}
				
				stepDetectSV = UPDATE;
                //break;		//go straigt into the next state
				
			case UPDATE:
				
				switch ( res )
				{
					case STEP_FILTER_CLR:
						if (x_mot_debug) app_trace_puts(DEBUG_LOW, "step: Clear\r");
						//Clear the startup requirements
						Step.Startup_Count = 0;
						Step.BlockAccum = BLOCK_ACCUM_CNT;
						Step.Cadence.qual_figure = 0;
					
						//go back and make possible steps false
						i = (recent_history.head-1)&SH_BUF_MASK;
						while( recent_history.step[i].type == POSS_STEP ) {
							recent_history.step[i].type = FALSE_STEP;
							i = (i-1)&SH_BUF_MASK;
						}
						break;
						
					case STEP_FILTER_ADD:
						if( Step.Startup_Count < SH_BUF_LEN )
						{
							//Too many possible Steps, oldest is going to be over-written!
							Step.Startup_Count++;
						}			
						
						recent_history.step[recent_history.head].time = get_unix_time();
						recent_history.step[recent_history.head].type = POSS_STEP;
						recent_history.head = (recent_history.head+1)&SH_BUF_MASK;
						break;
					
					case STEP_CNT_ADD:
						if (Step.Startup_Count > 0)
						{   //Add in the potential steps that were counting while the Cadence was unknown
							Step.Rel_Count += Step.Startup_Count;
							Step.Abs_Count += Step.Startup_Count;
							
							Step.Startup_Count = 0;
							
							//Set all previous Possible Steps to True, then add the next
							i = (recent_history.head-1)&SH_BUF_MASK;
							while( recent_history.step[i].type == POSS_STEP ) {
								recent_history.step[i].type = TRUE_STEP;
								i = (i-1)&SH_BUF_MASK;
							}
						}

						Step.Rel_Count++;   //new step!!!
						hrm_step = 1;
						Step.Abs_Count++;
						if (x_mot_debug) app_trace_log(DEBUG_MED, "[STEP] %01u\r", Step.Abs_Count);
						
						recent_history.step[recent_history.head].time = get_unix_time();
						recent_history.step[recent_history.head].type = TRUE_STEP;
						recent_history.head = (recent_history.head+1)&SH_BUF_MASK;
						break;

					default:
						//app_trace_log(DEBUG_MED, "step: nothing to update\r");
						break;
				}
				
				stepDetectSV = FIND_SLOPE;
				break;

            default:
                stepDetectSV = FIND_SLOPE;
                break;
        }   //end switch	
    }   //end while
	
	prv.mag = mag;
	prv.time = time;
	
	return (res);
}


// filter modified from: http://www.exstrom.com/journal/sigproc/bwbpf.c
//Variables to clculate 4th order Butterworth Bandpass filter
#define ORDER		4			//Filter order must be a multiple of 4!!!
#if ( (ORDER%4 != 0) || (ORDER == 0) )
	#error "BUTTERWORTH BPF Order Must be a Multiple of 4!!!"
#endif
#define N			ORDER/4
static int64_t g[N];
static int32_t w[5][N];
static int64_t d[4][N];
static bool butter_nit = false;
static bool mrs_butterworth_bpf_init(float samp_freq, float l_cut_off, float h_cut_off)
{
	int i;
	double a, a2, b, b2;
	double r;
	double PI = 3.14159265358979323846;
	double s = samp_freq;
	double fh = h_cut_off;
	double fl = l_cut_off;
	double float_g[N*sizeof(float)];
	double float_d[4][N*sizeof(float)];

	a = cos(PI*(fh + fl)/s) / cos(PI*(fh - fl)/s);
	a2 = a*a;
	b = tan(PI*(fh - fl)/s);
	b2 = b*b;
	
	for (i = 0; i < N; ++i)
	{
		r = sin(PI*(2*i + 1)/(4*N));
		s = b2 + 2*b*r + 1;
		
		w[0][i] = 0;
		w[1][i] = 0;
		w[2][i] = 0;
		w[3][i] = 0;
		w[4][i] = 0;
		
		float_g[i] = b2/s;
		float_d[0][i] = 4*a*(1 + b*r)/s;
		float_d[1][i] = 2*(b2 - 2*a2 - 1)/s;
		float_d[2][i] = 4*a*(1 - b*r)/s;
		float_d[3][i] = -(b2 - 2*b*r + 1)/s;
		
		g[i] = 65536*float_g[i];		//*(0x0001<<g_shift[i]);
		d[0][i] = 65536*float_d[0][i];	//*(0x0001<<d_shift[i]);
		d[1][i] = 65536*float_d[1][i];	//*(0x0001<<d_shift[i]);
		d[2][i] = 65536*float_d[2][i];	//*(0x0001<<d_shift[i]);
		d[3][i] = 65536*float_d[3][i];	//*(0x0001<<d_shift[i]);
	}

	butter_nit = true;
	return true;
}

static bool mrs_butterworth_filter(int32_t * x)
{
	int i;
	
	if( butter_nit == false ) {
		//error, need to initialize filter coefficients
		app_trace_log(DEBUG_MED, "[STEP_DET_2] Filter Not Init\r");
		return false;
	}

	for (i = 0; i < N; ++i)
	{
		int64_t temp = (d[0][i]*w[1][i] + d[1][i]*w[2][i] + d[2][i]*w[3][i] + d[3][i]*w[4][i])/65536;
		
		w[0][i] = *x + temp;
		*x = ((g[i]*(w[0][i] - 2*w[2][i] + w[4][i]))/65536);
		w[4][i] = w[3][i];
		w[3][i] = w[2][i];
		w[2][i] = w[1][i];
		w[1][i] = w[0][i];
	}

	return true;
}

