/// \file step_detect.c
///
/// Algorithm looks to accelerometer data to determine when a person is moving
/// and keeps a record of the number of steps.
///
/// Copyright (C) 2014 VSP Global\n
///
/// \author Matt Workman \n

#include <stdlib.h>
#include "step_detect.h"
#include "common.h"

#define MIN_DELTA_MAG       12					//Minimum Change (as a % of 1g) to be considered a Step

//Step Signal Timing Defines
#define MIN_TIME_MAX2MAX    150L                //Shortest time to measure between Maximas
#define MIN_TIME_MIN2MIN    MIN_TIME_MAX2MAX    //Should be same as MAXIMA to MAXIMA
#define MAX_TIME_MAX2MAX    1500L               //Walking slower than this period is = to sleeping... or at least it should be, so ignore it!
#define MAX_TIME_MIN2MIN    MAX_TIME_MAX2MAX    //Same as MAXIMA to MAXIMA
#define MIN_TIME_MAX2MIN    0L                  //Minima must come at least a short while after a Maxima
#define MIN_TIME_MIN2MAX    0L                  //Maxima must come at least a short while after a Minima
#define MAX_TIME_MAX2MIN    MAX_TIME_MAX2MAX-MIN_TIME_MIN2MAX    //Minima must come before the next Maxima is expected
#define MAX_TIME_MIN2MAX    MAX_TIME_MIN2MIN-MIN_TIME_MAX2MIN    //Maxima must come before the next Minima is expected

//Make sure the MAXIMA to MINIMA timeout does not exceed the MAXIMA to MAXIMA timeout
#if MAX_TIME_MAX2MIN > MAX_TIME_MAX2MAX
#error "MAX_TIME_MAX2MIN >  MAX_TIME_MAX2MAX"
#endif
//Make sure the MINIMA to MAXIMA timeout does not exceed the MINIMA to MINIMA timeout
#if MAX_TIME_MIN2MAX > MAX_TIME_MIN2MIN
#error "MAX_TIME_MIN2MAX > MAX_TIME_MIN2MIN"
#endif

//Standard deviation values
#define IN_STD_DEV_THRES	100		//Maxima standard Deviation Threshold to meet before peaks are included in Step Count (in milliseconds)
#define IN_VAR_THRES		IN_STD_DEV_THRES*IN_STD_DEV_THRES	//Variance = (Standard Deviation)^2
#define EX_STD_DEV_THRES	400		//Maxima standard Deviation Threshold that when exceeded will exclude peaks from Step Count (in msec)
#define EX_VAR_THRES		IN_STD_DEV_THRES*IN_STD_DEV_THRES	//Variance = (Standard Deviation)^2
#define BLOCK_ACCUM			4		//Prevent Steps from counting for at least x steps
#define HIGH_AMP_CADENCE	400		//A cadence of this time or less should have a high Amplitude associated with fast walking or running
#define FASTEST_CADENCE		222		//can only take so many steps/second... making limit 4.5 steps/sec

#define SAMP_SIZE	8
static struct {
	uint16_t u1gVal;			//sensitivity setting for accelerometer (+-2g, +-4g, +-8g, +-16g)
	uint32_t uDynamicDeltaMag;	//Dynamic Threshold of change in Magnitude to detect Min and Maxima
    uint32_t uMinDeltaMag;		//Smallest Value allowed for DynamicDeltaMag
	T_FEATURE Minima;
	T_FEATURE Maxima;
	T_FEATURE possMinima;
	T_FEATURE possMaxima;
	uint32_t PeriodPtr;
	uint32_t uPeriods[SAMP_SIZE];	//time between step peaks in milliseconds
	struct {
		uint32_t uAve;		//average of the last several step periods
		uint32_t uVar;		//the variance in those periods
	} Cadence;
	uint32_t avePeakAmp;	//Average of step Peaks
	uint32_t BlockAccum;
	uint32_t Startup_Count;
	uint32_t Rel_Count;		//Relative to the last time count was cleared
	uint32_t Abs_Count;		//Never cleared, used to track Steps/Minute
} Step = { 0 };

static T_STEP_HISTORY recent_history;
static bool mot_debug = false;

void motionCalculations(void);
static void clrRecentSteps( void );

///
//! \fn
/// \brief
/// \param
/// \return .
///
void initStepDetect_1( bool debug )
{	
	uint32_t i;
	
	mot_debug = debug; //WMMDB STEPS COUNT true=debug
	
    //initialize variables
	Step.u1gVal = get_accel_sens();
    Step.uMinDeltaMag = (uint32_t) Step.u1gVal*MIN_DELTA_MAG / 100; //Convert to Percent of 1g
    Step.uDynamicDeltaMag = Step.uMinDeltaMag;
	
	for(i=0; i<SAMP_SIZE; i++)
	{
		Step.uPeriods[i] = 0;	//start all Periods at 0
	}
	Step.Minima.mag = (uint32_t) Step.u1gVal; //Value of 1g
    Step.Maxima.mag = (uint32_t) Step.u1gVal; //Value of 1g
	Step.Minima.time = 0;
    Step.Maxima.time = 0;
	Step.PeriodPtr = 0;
	Step.Cadence.uVar = 0;
	Step.Cadence.uAve = 1000;		//Ave time between foot strikes
	Step.avePeakAmp = Step.u1gVal;
	Step.BlockAccum = true;
	Step.Startup_Count = 0;
	//Step.Rel_Count = 0;		//Only Clear on User Request
	//Step.Abs_Count = 0;		//Never Clear after Startup
	
	clrRecentSteps();
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
T_STEP_HISTORY * step_history_1( void ) 
{
	return &recent_history;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t getStartupCnt_1( void )
{
    return Step.Startup_Count;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t getStepCnt_1( void )
{
	return Step.Rel_Count;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
void setStepCnt_1( uint32_t step_cnt )
{
	Step.Rel_Count = step_cnt;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t getAbsoluteCnt_1( void )
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
TSTEP_FEEDBACK stepStateMachine_1( uint32_t time, uint32_t mag )
{
	TSTEP_FEEDBACK res = NOTHING;
	int32_t iDynamicThres;
	volatile TSTEP_DET_STATE copy_stepDetectSV = NA;
	static T_FEATURE prv;
	int32_t derivative = mag - prv.mag;	//Not divided by time, so this is really a scaled derivative
	
	//Need to execute State Machine until the State variable is settled. A Maxima and
    //Minima inflection may come in back to back points, so the state variable may need
    //to make multiple transitions between adjacent sample points.
    while (copy_stepDetectSV != stepDetectSV)
    {
        copy_stepDetectSV = stepDetectSV;

        switch (stepDetectSV) {
            case INIT:
                derivative = 0;    			//Prevent the slope from being used until the next data point...
                stepDetectSV = FIND_SLOPE;
                break;

            case FIND_SLOPE:
				if( time < Step.Minima.time || time < Step.Maxima.time ) {
					//Something has gone awry with the period timers (time overflows are possible: 49.7 days)...
					app_trace_log(DEBUG_MED, "Step_Algo: Timers Invalid!\r\n");	
					Step.Maxima.time = Step.Minima.time = 0;
				}
				
                //Determine which type of point we need to look for...
                if (derivative > 0) 
                {   // positive sloping
                    //Set Potential to Value less than the coming Maxima.
                    //+Derivative means currentMag is already > prevMag, so prevMag is not going to be the Maxima!
                    //Don't want to miss the real Maxima by guessing a value > whatever the Maxima ends up being.
                    Step.possMaxima.mag = prv.mag;
                    stepDetectSV = MAXIMA_HUNT; //now go wait for a possible Maxima!
                }
                else if (derivative < 0) 
                {   // negative sloping
                    //Set Potential to Value greater than the coming Minima.
                    //-Derivative means currentMag is already < prevMag, so prevMag is not going to be the Minima!
                    //Don't want to miss the real Minima by guessing a value < whatever the Minima ends up being.
                    Step.possMinima.mag = prv.mag;
                    stepDetectSV = MINIMA_HUNT; //now go wait for a possible Minima!
                }
                break;

            case MINIMA_HUNT: //Waiting for a Possible Minima Inflection
                if (derivative > 0) //increasing slope
                {   //Inflection found!
                    if (time < (Step.Minima.time + MIN_TIME_MIN2MIN) ||
                            time < (Step.Maxima.time + MIN_TIME_MAX2MIN)) 
                    {   //prevent a Minima from being considered if it comes too quickly after
                        //previous Maxima or Minima
                        res = MINIMA_TOO_SOON;
                        stepDetectSV = FIND_SLOPE;
                    }
                    else
                    {  
                        if (prv.mag < Step.possMinima.mag)
                        {	//Save the lowest Inflection point that occurs between Maximas
                            Step.possMinima.mag = prv.mag; //keep new potential minima
                            Step.possMinima.time = prv.time;	//previous mag time
                            res = POSSIBLE_MINIMA;
                        }
                        stepDetectSV = VERIFY_MINIMA; //Wait for Signal to rise further than DynamicDeltaMagnitude or restarts falling
                    }
                }
                break;

            case MAXIMA_HUNT: //Waiting for a Possible Maxima Inflection
                if (derivative < 0) //decreasing slope
                {   //Inflection found!
                    if (time < (Step.Maxima.time + MIN_TIME_MAX2MAX) ||
                            time < (Step.Minima.time + MIN_TIME_MIN2MAX)) 
                    {   //prevent a Maxima from being considered if it comes too quickly after
                        //previous Minima or Maxima
                        res = MAXIMA_TOO_SOON;
                        stepDetectSV = FIND_SLOPE;
                    }
                    else 
                    {   //Only consider a Potential Maxima that is greater than 1g threshold...																			
                        if (prv.mag > Step.possMaxima.mag)
                        {	//Save the highest Inflection point that occurs between Minimas
                            Step.possMaxima.mag = prv.mag; //keep new potential maxima
                            Step.possMaxima.time = prv.time;	//previous mag time
                        }
						
						if (Step.possMaxima.mag > ((uint32_t)Step.u1gVal + (Step.uMinDeltaMag/2)))
						{
							//if (mot_debug) app_trace_log(DEBUG_LOW, "Poss Max: %04u\r\n", (unsigned int)Step.possMaxima.mag);
                            res = POSSIBLE_MAXIMA;		//Motion is definitely occurring
							stepDetectSV = VERIFY_MAXIMA; //Wait for Signal to fall further than DynamicDeltaMagnitude or restarts rising
						}
						else
						{
							stepDetectSV = MINIMA_HUNT;
						}
					}
                }
                break;

            case VERIFY_MINIMA:
                iDynamicThres = Step.possMinima.mag + Step.uDynamicDeltaMag;

                if (derivative < 0) 
                {   //We're on our way back Down, return to looking for the new potential minima.
                    //There has been a local maxima that was not big enough to consider a step.

                    stepDetectSV = MINIMA_HUNT;
                } 
				else if (time > (Step.possMinima.time + MAX_TIME_MIN2MAX))
                {	//MINIMA has taken too long to rise above DynamicDeltaMag, null out Potential
                    //Minima parameters to allow a fresh value to be obtained
					Step.possMinima.mag = (uint32_t)Step.u1gVal;
					Step.possMinima.time = time;
                    res = MINIMA_TIMEOUT;

                    Step.BlockAccum = true;     //Have to see X Valid Steps before counting again
                    Step.uDynamicDeltaMag = Step.uMinDeltaMag;

                    stepDetectSV = FIND_SLOPE;
                }
                else if (mag > (uint32_t)iDynamicThres)
                {
                    Step.Minima.mag = Step.possMinima.mag; //looks like it was an actual minima
                    Step.Minima.time = Step.possMinima.time;
                    res = MINIMA_VALIDATED;

                    stepDetectSV = UPDATE; //Only update DynamicMag on Maxima Detect?
                }
				else if (time >= (Step.Minima.time + (3*Step.Cadence.uAve)/2))	//Last Minima + 1.5*Ave Cadence
				{   //Last Minima TimeStamp indicates a Step is overdue
					//Step.BlockAccum = true;
					Step.uDynamicDeltaMag = Step.uMinDeltaMag;
					Step.Maxima.mag = Step.Minima.mag = (uint32_t)Step.u1gVal; //force minimum DynamicDelta
					//res = MINIMA_TIMEOUT;
					//stepDetectSV = FIND_SLOPE;
				}
                break;

            case VERIFY_MAXIMA:				
				//Keep Accelerometer On for X seconds when significant motion is detected
				keep_accel_on();
				
				iDynamicThres = (int32_t)Step.possMaxima.mag - Step.uDynamicDeltaMag;
				if (iDynamicThres < (Step.u1gVal/2)) {
					iDynamicThres = (int32_t)Step.u1gVal/2;		//Limit the lower threshold
				}

                if (derivative > 0) 
                {   //We're on our way back Up, return to looking for the new potential maxima
                    //There has been a local minima that may correspond to a missed step...
                    stepDetectSV = MAXIMA_HUNT;
                } 
				else if ( time > (Step.possMaxima.time + MAX_TIME_MAX2MIN) )
                {   //MAXIMA has taken too long to fall below DynamicDeltaMag, null out Potential
                    //Maxima parameters to allow a fresh value to be obtained
					Step.possMaxima.mag = (uint32_t)Step.u1gVal;
					Step.possMaxima.time = time;
                    res = MAXIMA_TIMEOUT;

					//if (imu_debug) app_trace_log(DEBUG_LOW, "Step_Algo: Max TO\r\n");	
                    Step.BlockAccum = true;     //Have to see X Valid Steps before counting again
                    Step.uDynamicDeltaMag = Step.uMinDeltaMag;

                    stepDetectSV = FIND_SLOPE;
                }
                else if (mag < (uint32_t)iDynamicThres)
                {
                    uint32_t i;
                    uint32_t ave, squares;
					uint32_t stepPeriod, step2stepDiff;
					static uint32_t lastPeakTime = 0;	//time of last peak, whether it was deemed a step or not.
                    
                    stepPeriod = Step.possMaxima.time - lastPeakTime;
					lastPeakTime = Step.possMaxima.time;
					step2stepDiff = abs((int32_t)stepPeriod - Step.uPeriods[ Step.PeriodPtr ]);
                    if( stepPeriod > 2*MAX_TIME_MAX2MAX ) 
					{
						Step.BlockAccum = true;
						Step.Startup_Count = 0;
						//if (mot_debug) app_trace_log(DEBUG_LOW, "Step Clear\r\n");
						res = STEP_FILTER_CLR;
                    }
					else if (step2stepDiff > EX_STD_DEV_THRES)
					{	//Don't allow Peaks to add to Step Count if a low variance Cadence is not established...
						Step.BlockAccum = true;
						Step.Startup_Count = 1; //There always has to be 1 good step before this second step will match it
						Step.uPeriods[ Step.PeriodPtr ] = stepPeriod; //Save for next comparison...
						//if (mot_debug) app_trace_log(DEBUG_LOW, "Step Start\r\n");
						
						recent_history.step[recent_history.head].time = get_unix_time();
						recent_history.step[recent_history.head].type = POSS_STEP;
						recent_history.head = (recent_history.head+1)&SH_BUF_MASK;
							
						res = STEP_FILTER_1ST;
					}
					else
					{	//Looks like an actual Step Maxima!
						//Make sure Gyro is on for at least the next few seconds to help with step detection
						keep_gyro_on();
						
						//keep_magnet_on();	//temporarily turn on Magnetometer for testing
						
						Step.Maxima.mag = Step.possMaxima.mag;
						Step.Maxima.time = Step.possMaxima.time;
						
						Step.PeriodPtr = (Step.PeriodPtr+1)%SAMP_SIZE;  //update pointer
						Step.uPeriods[ Step.PeriodPtr ] = stepPeriod;
						
						//Light Statical Analysis...
						ave = squares = 0;
						for( i=0; i<SAMP_SIZE; i++ ) {
							ave += Step.uPeriods[i];
							squares += Step.uPeriods[i]*Step.uPeriods[i];
						}
						ave /= SAMP_SIZE;
						squares /= SAMP_SIZE;
						Step.Cadence.uVar = squares - ave*ave;    //Variance = (Average of Squares) - (Square of the Average)
						Step.Cadence.uAve = ave;

						//if Cadence.uiAvePeriod is short and uiDynamicDelta is low, the peaks are likely not due to stepping
						//Could use this to block peaks due to driving or other random oscillating motion.
						Step.avePeakAmp *= 3;
						Step.avePeakAmp += Step.Maxima.mag;
						Step.avePeakAmp /= 4;		//(1*new + 3*old)/4;

						//Don't allow Peaks to add to Step Count if a low variance Cadence is not established...
						if( Step.Cadence.uVar > IN_VAR_THRES ) 
						{
							Step.BlockAccum = true;
							//if (mot_debug) app_trace_log(DEBUG_LOW, "High Variance Motion\r\n");
						}

						//Don't Count Steps until Good Cadence and Minimum Successive Steps have been met
						if (Step.BlockAccum != false)
						{		
							if(++Step.Startup_Count == 1) {
								res = STEP_FILTER_1ST;
							}
							else {
								if (Step.Startup_Count >= BLOCK_ACCUM)
								{
									Step.BlockAccum = false;
								}
								res = STEP_FILTER_ADD;
							}
							
							recent_history.step[recent_history.head].time = get_unix_time();
							recent_history.step[recent_history.head].type = POSS_STEP;
							recent_history.head = (recent_history.head+1)&SH_BUF_MASK;
						}
						else
						{
							if ((Step.avePeakAmp < (((uint32_t)3*Step.u1gVal)>>1)) &&	// < 1.5*1gValue
								Step.Cadence.uAve < HIGH_AMP_CADENCE)	
							{   //Amplitude is not high enough to match the frequency of a person fast walking...
								//Removes some instances of false positives while driving
								Step.BlockAccum = true;
								Step.Startup_Count = 0;
								//if (mot_debug) app_trace_log(DEBUG_LOW, "Non Stepping Motion\r\n");
								res = STEP_FILTER_CLR;
							}
							else if (Step.Cadence.uAve < FASTEST_CADENCE)
							{   //Frequency too fast for bipedal motion
								Step.BlockAccum = true;
								Step.Startup_Count = 0;
								//if (mot_debug) app_trace_log(DEBUG_LOW, "Motion Too Fast\r\n");
								res = STEP_FILTER_CLR;
							}
							else
							{	//all the conditions are met, count away!
								if (Step.Startup_Count > 0)
								{   //Add in the potential steps that were counting while the Gait was unknown
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
								Step.Abs_Count++;
								
								if( mot_debug ) app_trace_log(DEBUG_MED, "[STEP] %01u\r\n", Step.Abs_Count);
								
								recent_history.step[recent_history.head].time = get_unix_time();
								recent_history.step[recent_history.head].type = TRUE_STEP;
								recent_history.head = (recent_history.head+1)&SH_BUF_MASK;
								
								res = STEP_CNT_ADD;
							}
						}
					}
					
					stepDetectSV = UPDATE;
                }
				else if ( time >= (Step.Maxima.time + (3*Step.Cadence.uAve)/2) )	//+ 1.5*AvePeriod
				{   //Last Maxima TimeStamp indicates a Step is overdue
					Step.BlockAccum = true;
					Step.uDynamicDeltaMag = Step.uMinDeltaMag;
					Step.Maxima.mag = Step.Minima.mag = (uint32_t)Step.u1gVal;	//force minimum DynamicDelta
					//res = MAXIMA_TIMEOUT;
					//stepDetectSV = FIND_SLOPE;
				}
                break;

            case UPDATE:
                //Adjust dynamic Step Amplitude threshold: NewThres = (3*CurrentThres + NewAmp/2)/4 ~= 1/2 Ave Amplitude
                Step.uDynamicDeltaMag *= 3;
                Step.uDynamicDeltaMag += (Step.Maxima.mag - Step.Minima.mag)/2; //add in 1/2 of the minima to maxima amplitude
                Step.uDynamicDeltaMag /= 4;
                if (Step.uDynamicDeltaMag < Step.uMinDeltaMag) {//Enforce minimum Change in magnitude to be considered a local min/maxima
                    Step.uDynamicDeltaMag = Step.uMinDeltaMag;
				}

                stepDetectSV = FIND_SLOPE;
                break;

            default:
                stepDetectSV = FIND_SLOPE;
                break;
        }   //end switch	
		
		
		//check for reset of step state, clear possible steps when this happens
		uint i = recent_history.head;
		switch ( res )
		{
			case STEP_FILTER_1ST:
				//go back and make possible steps false
				i = (i-2)&SH_BUF_MASK;	//go back 2 indices
				while( recent_history.step[i].type == POSS_STEP ) {
					recent_history.step[i].type = FALSE_STEP;
					i = (i-1)&SH_BUF_MASK;
				}
				break;
				
			case STEP_FILTER_CLR:
			case MAXIMA_TIMEOUT:
			case MINIMA_TIMEOUT:
				//go back and make possible steps false
				i = (i-1)&SH_BUF_MASK;
				while( recent_history.step[i].type == POSS_STEP ) {
					recent_history.step[i].type = FALSE_STEP;
					i = (i-1)&SH_BUF_MASK;
				}
				break;

			default:
				break;
		}
    }   //end while
	
	prv.mag = mag;
	prv.time = time;
	
	return (res);
}


