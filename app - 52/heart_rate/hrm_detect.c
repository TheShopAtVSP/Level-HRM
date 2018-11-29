//-----------------------------------------------------------------------------
// hrm_detect.c
//-----------------------------------------------------------------------------
// WMM 12-17
//-----------------------------------------------------------------------------

#include "nrf.h"
#include "led.h"
#include "hrm_detect.h"
#include "drv_si115x.h"
#include "comms.h"
#include "timing.h"

// ====================> Some HRM test globals <=======================
#define		HRM_RUNNING_AVG_SAMPLES		32 
#define		HRM_STITCH_TRESH	300
#define		HRM_ZERO_SEEKER_TRESH	10000
#define		HRM_AVG_SIZE	2
#define	    HRM_AVG_SAMPLES		4
#define     HRM_DP_DATA_SIZE 	156
#define     PD_DEBOUNCE_WIDTH 	7

uint16_t 	accel_x;
uint16_t 	accel_y;
uint16_t 	accel_z;
uint8_t 	uart_rev_buffer[1] = {0};
uint8_t 	uart_rx_ubyte;
T_PKT_TYPES tx_type;			//T_PKT_TYPES type;
uint8_t 	tx_payload[ MAX_PKT_PAYLOAD ];
int8_t 		tx_pay_len = -1;	//default no response
uint8_t 	current_hrm = 69;
uint8_t 	hrm_step = 0;
uint16_t 	run_get_hrms = 0;
int16_t 	hrm_chan1_raw[256], hrm_chan2_raw[256], hrm_chan3_raw[256], hrm_raw_index = 0, hrm_raw_index_old;
int32_t		hrm_chan_1_raw32, hrm_chan_2_raw32;
uint8_t		hrm_running_avg_indx = 0, hac_index = 0;
int32_t		hrm_chan1_running_avg[HRM_RUNNING_AVG_SAMPLES], hrm_chan2_running_avg[HRM_RUNNING_AVG_SAMPLES];
int32_t		hrm_chan1_running_avg_accm, hrm_chan2_running_avg_accm;
int32_t 	hrm_ch1_avg, hrm_ch2_avg;
int32_t 	hrm_chan1_fbs = 0, hrm_chan2_fbs = 0;
int32_t		hrm_avg_chan132[HRM_AVG_SIZE], hrm_avg_chan232[HRM_AVG_SIZE], hrm_avg_chan132_accm, hrm_avg_chan232_accm;
int16_t 	pd_emi_delta;
float		silly_ratio;
bool		hrm_stitch_flag = 0;
/// note default index = 8 = 0x12 = 50mA
uint8_t hrm_agc_led_current_idx1 = 29;
uint8_t hrm_agc_led_current_idx3 = 29;
uint8_t hrm_agc_led_current[30] = {0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x12, 0x21, 0x29, 0x31, 0x22, 0x39, 0x2A, 
									0x23, 0x32, 0x3A, 0x24, 0x33, 0x2C, 0x3B, 0x34, 0x2D, 0x3C, 0x35, 0x3D, 0x36, 0x3E, 0x3F};
	

uint8_t 	reg[2], timer_count_out = 0;
uint32_t 	hrm_timer_start, hrm_current_time, hrm_add_time, hrm_run_time;
static int16_t		num_emi_peaks, num_absop_peaks, emi_peaks_xpos[HRM_DP_DATA_SIZE+1], absop_peaks_xpos[HRM_DP_DATA_SIZE+1];
static int16_t		emi_peaks[HRM_DP_DATA_SIZE+1], absop_peaks[HRM_DP_DATA_SIZE+1];
int8_t 		i, j, k, dh, old_num_of_emi_peaks, old_num_of_absop_peaks, pd_reduction_loop;
static int16_t 	avg_x_diff, avg_x_rng;
//static int16_t 	pd_absop_delta;
//static int16_t 	hrm_chan1_raw_avg[256], hrm_chan2_raw_avg[256];
double  	emi_peaks_dfp[128], emi_peaks_xpos_dfp[128], absop_peaks_dfp[128], absop_peaks_xpos_dfp[128], intgtr_gyr_p[156], intgtr_gyr_y[156];
///static int16_t		last4hrmavg[8], last4hrmavgidx;

// Function Prototypes									
void hrm_read_cmd(void);
void Get_HRMs(void);
int16_t PeakDiff1D(const int16_t *arr, size_t length);
int8_t detect_peak
(
    const int16_t*  data,       /* the data */ 
    int8_t          data_count, /* row count of data */  
    int16_t         delta,      /* delta used for distinguishing peaks */
    int8_t          emi_first   /* search for emission or absorption peak first */
);
	
	
/*==============================================*/
/* 				 Host Commands 					*/
/*==============================================*/

/*UART buffer size.*/
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

uint8_t reg[2];
extern uint8_t uart_rx_ubyte;
extern uint8_t uart_rev_buffer[1];
	
void hrm_init( void )
{
	//Fire up detector
	Si1153_twi_init();
	Si1153_Init();
}

void SetIRLEDAnode (uint8_t led_switch)
{
	if(led_switch == 0) // Int IR LED
	{
		led(0,0,0,0,-1);
	}
	
	if(led_switch == 1) // Ext IR LED
	{
		led(1,1,1,-1,0);
	}
}

void hrm_read_cmd(void)
{		
	int time_out = 0;
	int t_o = 100;
	
	if(uart_rx_ubyte != 0x00)
	{	
		if(uart_rx_ubyte == 'a') // a = internal led
		{
			SetIRLEDAnode(1);
		}
		if(uart_rx_ubyte == 'b') // b = external led
		{
			SetIRLEDAnode(0);
		}
		
		if(uart_rx_ubyte == 'c') // c = led enable chan 0
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 'c') && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_MEASCONFIG0, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 'd') // d = led enable chan 1
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 'd') && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_MEASCONFIG1, uart_rx_ubyte);/// + 0x08);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 'e') // e = current LED 1
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 'e') && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_LED1_A, uart_rx_ubyte);
				Si115xParamSet(PARAM_LED1_B, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 'f') // f = current LED 2
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 'f') && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_LED2_A, uart_rx_ubyte);
				Si115xParamSet(PARAM_LED2_B, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 'g') // g = current LED 3
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 'g') && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_LED3_A, uart_rx_ubyte);
				Si115xParamSet(PARAM_LED3_B, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		uart_rx_ubyte = 0x00;
	}
}

//========================================================================================================
/*
		HRM Post Processing 				
*/
//========================================================================================================

void Get_HRMs(void)
{
	int16_t 	trialidx, trialerrorcnt[4], emiandabsoppd[4], total_emiandabsops; 
	//int16_t  	last_pvariance, pvariance;
	//double  	phase2sf, phase2sfcof;
	int16_t		ser_input_size = 120;
	//int16_t     justanextraint16;
	
	// Added a three step loop that processes chan 1, then chan 2 then subtracted channel while counting the total added peaks and 
	// doubles. after all three have been processed the lower error count determines which emi + absop peaks are to be used as the HR.
	// trialidx 0-2, trialerrorcnt(0-2), trialhr0-2(emi+absop).
	// This allows the prdominate LED to win a fight and auto switches to subtract mode without using a threshold.
	// Later add a frame invalid and use the last current_hrm if errors are too high or if sub and result P2P is greater than xxx.
	// also trying: trialerrorcnt[] will be evaluated by  taking the starting (oldemiandabsoppd) pd and abs old - new
	// 				trialerrorcnt[trialidx] = abs(oldemiandabsoppd[trialidx] - (num_emi_peaks + num_absop_peaks)); 	
	for(trialidx = 0; trialidx < 3; trialidx++)	
	{	
		trialerrorcnt[trialidx] = 0;
		
		
		if(trialidx == 2) // do this one last so that we can skip it if trialerrorcnt(0 or 1) is low.
		{
			/// Auto Subtract ///
			
			for(i = 0; i < ser_input_size; i++)
			{
				hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * (silly_ratio)));
			}
			
			
//			phase2sfcof = 0.5;
//			pvariance = PeakDiff1D(hrm_chan1_raw, ser_input_size);
//			justanextraint16 = PeakDiff1D(hrm_chan2_raw, ser_input_size);
//			phase2sf = fabs((double)justanextraint16 / phase2sf);
//			
//			last_pvariance = 30000;
//			/// printf("%f\n",phase2sf);
//			for(int ol=0;  ol < 1; ol++)  /// 3
//			{
//				if(ol==0) /// 1
//					phase2sfcof = 0.05; /// 0.05
//				if(ol==1) /// 2
//					phase2sfcof = 0.001; /// 0.005
//					
//				last_pvariance = 30000;	 

//				while(abs(last_pvariance) > abs(pvariance))
//				{
//					last_pvariance = pvariance;
//					phase2sf += phase2sfcof;
//					for(i = 0; i < ser_input_size; i++)
//					{
//						hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * phase2sf));
//					}
//					pvariance = PeakDiff1D(hrm_chan3_raw, ser_input_size);
//				}
//				
//				last_pvariance = 30000;
//			
//				while(abs(last_pvariance) > abs(pvariance))
//				{
//					last_pvariance = pvariance;
//					phase2sf -= phase2sfcof;
//					for(i = 0; i < ser_input_size; i++)
//					{
//						hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * phase2sf));
//					}
//					pvariance = PeakDiff1D(hrm_chan3_raw, ser_input_size);
//				}
//			}
//			for(i = 0; i < ser_input_size; i++)
//			{
//				hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * (phase2sf + phase2sfcof)));
//			}
		}
		else  // do not do subtract and load chan 3 for next step. trialidx=0 will be chan1 and trialidx=1 will be chan2.
		{	 	
			for(i = 0; i < hrm_raw_index ; i++)
			{
				if(trialidx == 0)
					hrm_chan3_raw[i] = hrm_chan1_raw[i];
				if(trialidx == 1)
					hrm_chan3_raw[i] = hrm_chan2_raw[i];
			}
		}


#if 1
		// dH Threshold control
		/// Eliminate bounce by increasing delta (dh). Basicaly acts as a auto peak detector threshold control.
		/// It looks like starting from low to high sometimes eliminates wanted peaks. 
		/// If we start from high and look for a minimum number of peaks we could post process after substracting a proprtional
		/// amount to insure that we get some of the dis-qualified peaks. De bounce would have to then take the form of the missing peaks
		/// routine.  Adding more averaging may yeild a completly different result.
		
		pd_emi_delta = 35;
		do
		{	
			detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1);
			pd_emi_delta -= 2;
		}while((num_emi_peaks < 4) || (num_absop_peaks < 4));
		pd_emi_delta -= 2; // means delta - 2. should be porpotional...
		if(pd_emi_delta < 4)
			pd_emi_delta = 4;
		
//		pd_emi_delta = 3; // To play with going low to high.
//		do
//		{	
//			detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1);
//			pd_emi_delta += 2;
//		}while((num_emi_peaks > 5) || (num_absop_peaks > 5));
//		pd_emi_delta -= 2; // means delta - 2. should be porpotional...
//		if(pd_emi_delta > 35)
//			pd_emi_delta = 35;
		
		detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1); 
#endif	
		
   #if 0	
		/// Eliminate bounce by increasing delta (dh). Basicaly acts as a auto peak detector threshold control. 
		/// This is used with the Stitcher and works rather well, but is #ifdefed out for this  architecture.
		if(pd_emi_auto)
			pd_emi_delta = 4;
	//	sprintf(out_str, "Eliminate bounce1: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks);
	//	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
		do
		{	
			detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1);
			dh = 0;
			int8_t pd_reduction_loop = num_emi_peaks;
			if(pd_reduction_loop > num_absop_peaks)
				pd_reduction_loop = num_absop_peaks;
			for(i=0; i<(pd_reduction_loop - 1); i++)
			{
				if( ((emi_peaks_xpos[i+1] - emi_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) 
					|| ((absop_peaks_xpos[i+1] - absop_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) )
				{
					dh++;
				}
			}
			
			if(dh > 0)
			{
				pd_emi_delta++;	
			}
			
			if(dh == 25)
				dh = 0;
		}while(dh > 0);
	//	 sprintf(out_str, "Eliminate bounce2: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks); 
	//	 SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
#else
	///	detect_peak(hrm_chan3_raw, hrm_raw_index, 8, 1);
#endif	
		
#if 1		
		/// Missing pulse correction:
		// run thru the absop_peaks_xpos[num_absop_peaks[0]]. find delta x for each. avg delta x. if 1 is double the rest add 1 to num_absop_peaks[0].
		
		int8_t old_num_of_emi_peaks;
		int8_t old_num_of_absop_peaks;

		if(num_emi_peaks > 63)  // Don't diddle those kids.
			num_emi_peaks = 63;
		if(num_absop_peaks > 63)
			num_absop_peaks = 63;
		
		// Add phantom end points to emi and absop arrays. 150 is used for debug and not processed.
		emi_peaks_xpos[num_emi_peaks] = 135;
		emi_peaks[num_emi_peaks] = 2;
		num_emi_peaks++;
		emi_peaks_xpos[num_emi_peaks] = 150;
		emi_peaks[num_emi_peaks] = 2;
		num_emi_peaks++;
		for(i=num_emi_peaks; i>0; i--)      
		{
			emi_peaks_xpos[i] = emi_peaks_xpos[i - 1];
			emi_peaks[i] = emi_peaks[i - 1];
		}
		emi_peaks_xpos[0] =  -16;
		emi_peaks[0] = 2;
		absop_peaks_xpos[num_absop_peaks] = 135;
		absop_peaks[num_absop_peaks] = -2;
		num_absop_peaks++;
		absop_peaks_xpos[num_absop_peaks] = 150;
		absop_peaks[num_absop_peaks] = -2;
		num_absop_peaks++;
		for(i=num_absop_peaks; i>0; i--)      
		{
			absop_peaks_xpos[i] = absop_peaks_xpos[i - 1];
			absop_peaks[i] = absop_peaks[i - 1];
		}
		absop_peaks_xpos[0] =  -6;
		absop_peaks[0] = -2;
		
		//Start emi
		for(k=0; k<1; k++)
		{
			int16_t avg_x_diff = 0;
			old_num_of_emi_peaks = num_emi_peaks;
			
			if(old_num_of_emi_peaks > 0)
			{
				for(i=0; i<(old_num_of_emi_peaks - 1); i++) // Why limit x axis range???
				//for(i=0; i<(old_num_of_emi_peaks); i++)   
				{
					avg_x_diff += (emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]);   
				}
				avg_x_diff = avg_x_diff / old_num_of_emi_peaks;
				avg_x_rng = avg_x_diff + (avg_x_diff / 2);// + (avg_x_diff / 4);
			}
			
			for(i=0; i<=(old_num_of_emi_peaks /*- 1*/); i++)   /// 10 turned into 18 because avg x rng was too small or travelling shift.
			{
				if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) > avg_x_rng) 
				{
					//for(j=old_num_of_emi_peaks; j<=(i+1); j--)
					for(j=num_emi_peaks + 0; j>(i); j--)
					{
						emi_peaks_xpos[j] = emi_peaks_xpos[j - 1];
						emi_peaks[j] = emi_peaks[j - 1];
						 
					}
					//emi_peaks_xpos[i] = emi_peaks_xpos[i] + (avg_x_diff / (old_num_of_emi_peaks +1));
					emi_peaks_xpos[j+1] = emi_peaks_xpos[j] + (avg_x_diff); 
					emi_peaks[j+1] = 0;
					num_emi_peaks++;
					trialerrorcnt[trialidx]++;
				}
			}
		}
		
		// Get rid of the phantom data
		j = 0;
		for(i=0; i<(num_emi_peaks); i++)
		{	 
			if(emi_peaks_xpos[i] < 120)
			{
				emi_peaks_xpos[i] = emi_peaks_xpos[i+1];
				emi_peaks[i] = emi_peaks[i+1];
				j++;
			}
		}
		num_emi_peaks = j - 1;
		
		//Start absop
		for(k=0; k<1; k++)
		{	
			if(num_emi_peaks > 63)
			num_emi_peaks = 63;
			if(num_absop_peaks > 63)
			num_absop_peaks = 63;
			avg_x_diff = 0;
			old_num_of_absop_peaks = num_absop_peaks;
			
			if(old_num_of_absop_peaks > 0)
			{
				avg_x_diff = 0;
				for(i=0; i<(old_num_of_absop_peaks - 1); i++)
				//for(i=0; i<(old_num_of_absop_peaks); i++)
				{
					avg_x_diff += (absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]);	
				}
				avg_x_diff = avg_x_diff / old_num_of_absop_peaks;
				avg_x_rng = avg_x_diff + (avg_x_diff / 2);// + (avg_x_diff / 4);
			}
			
			for(i=0; i<(old_num_of_absop_peaks /*- 1*/); i++)
			{
				if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) > avg_x_rng)
				{
					
					for(j=num_absop_peaks; j>(i); j--)
					//for(j=old_num_of_absop_peaks + 1; j<=(i+1); j--) 
					{
						absop_peaks_xpos[j] = absop_peaks_xpos[j-1];
						absop_peaks[j] = absop_peaks[j-1];
					}
					//absop_peaks_xpos[i] = absop_peaks_xpos[i] + (avg_x_diff / (old_num_of_emi_peaks +1));
					absop_peaks_xpos[j+1] = absop_peaks_xpos[j] + (avg_x_diff);
					absop_peaks[j+1] = 0; 
					num_absop_peaks++;
					trialerrorcnt[trialidx]++;
				}
			}
			
			// Get rid of the phantom data
			j = 0;
			for(i=0; i<(num_absop_peaks); i++)
			{	 
				if(absop_peaks_xpos[i] < 120)
				{
					absop_peaks_xpos[i] = absop_peaks_xpos[i+1];
					absop_peaks[i] = absop_peaks[i+1];
					j++;
				}
			}
			num_absop_peaks = j - 1;
		}
#endif

#if 1	
		/// Eliminate Bounce.
		old_num_of_emi_peaks = num_emi_peaks;
		///	avg_x_rng = avg_x_rng/3; // might be changed to depend on the last HR * some multipl
		avg_x_rng = current_hrm / 8;   // 60hbpm @ 20sps => 10 for half cycle. 6 * 10 * 2 = 120.  60/10=6  = quarter cycle
		for(i=0; i<num_emi_peaks; i++)
		{
			if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) < avg_x_rng)
			{
				old_num_of_emi_peaks--;
				i=i+1;
				trialerrorcnt[trialidx]++;
//				sprintf(out_str, "Found extra emi: emi_peaks_xpos[i] %d\n", emi_peaks_xpos[i]);
//				SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
			}
		}
		num_emi_peaks = old_num_of_emi_peaks;
		
		old_num_of_absop_peaks = num_absop_peaks;
		for(i=0; i<num_absop_peaks; i++)
		{
			if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) < avg_x_rng)
			{
				old_num_of_absop_peaks--;
				i=i+1;
				trialerrorcnt[trialidx]++;
//				sprintf(out_str, "Found extra absop: absop_peaks_xpos[i] %d\n", absop_peaks_xpos[i]);
//				SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
			}
		}
		num_absop_peaks = old_num_of_absop_peaks;
		
//		sprintf(out_str, "Eliminate bounce final: Dh = %d, avg_x_rng = %d, emip = %d, absopp = %d\n", pd_emi_delta, avg_x_rng, num_emi_peaks, num_absop_peaks); 
//		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);

		///	detect_peak(hrm_chan3_raw, hrm_raw_index, 8, 1);
#endif	
		emiandabsoppd[trialidx] = num_emi_peaks + num_absop_peaks;	
	} // end of trial loop 

	///============== Post processing =====================///

	/// Mode Selector
	int dp_mode = 0;

	//if((emiandabsoppd[0] >= emiandabsoppd[1]) && (emiandabsoppd[0] <= emiandabsoppd[2]))
	//	dp_mode = 0;
	//else if((emiandabsoppd[1] >= emiandabsoppd[0]) && (emiandabsoppd[1] <= emiandabsoppd[2]))
	//	dp_mode = 1;
	//else if((emiandabsoppd[2] >= emiandabsoppd[0]) && (emiandabsoppd[2] <= emiandabsoppd[1]))
	//	dp_mode = 2;
	//if((emiandabsoppd[0] >= emiandabsoppd[1]) && (emiandabsoppd[0] <= emiandabsoppd[2]))
	
	if(((emiandabsoppd[0] >= emiandabsoppd[1]) && (emiandabsoppd[0] <= emiandabsoppd[2])) || ((emiandabsoppd[0] <= emiandabsoppd[1]) && (emiandabsoppd[0] >= emiandabsoppd[2])))
		dp_mode = 0;
	//else if((emiandabsoppd[1] >= emiandabsoppd[0]) && (emiandabsoppd[1] <= emiandabsoppd[2]))
	else if(((emiandabsoppd[1] >= emiandabsoppd[0]) && (emiandabsoppd[1] <= emiandabsoppd[2])) || ((emiandabsoppd[1] <= emiandabsoppd[0]) && (emiandabsoppd[1] >= emiandabsoppd[2])))
		dp_mode = 1;
	else if(((emiandabsoppd[2] >= emiandabsoppd[0]) && (emiandabsoppd[2] <= emiandabsoppd[1])) || ((emiandabsoppd[2] <= emiandabsoppd[0]) && (emiandabsoppd[2] >= emiandabsoppd[1])))
		dp_mode = 2;

	//if((trialerrorcnt[0] <= trialerrorcnt[1]) && (trialerrorcnt[0] <= trialerrorcnt[2]))
	//	dp_mode = 0;
	//else if((trialerrorcnt[1] <= trialerrorcnt[0]) && (trialerrorcnt[1] <= trialerrorcnt[2]))
	//	dp_mode = 1;
	//else if((trialerrorcnt[2] <= trialerrorcnt[0]) && (trialerrorcnt[2] <= trialerrorcnt[1]))
	//	dp_mode = 2;
	//if(pvariance > 200)
	//	dp_mode = 3;

	total_emiandabsops = emiandabsoppd[dp_mode];
	//total_emiandabsops = (((double)(emiandabsoppd[0] * 5) + (double)(emiandabsoppd[1] * 5) + (double)(emiandabsoppd[2] * 5)) / 3);
	emiandabsoppd[3] = emiandabsoppd[dp_mode]; // save for skip frame //

    /// Average the hrm output.
	current_hrm = ((total_emiandabsops * 5) + (current_hrm * 3)) / 4;
}



/*========================================================================================================//

void Get_HRMs(void)
{
	/// CZ + MM => avg 4, delta 5, no width skip, stitching on, forward filter off. 
	int8_t 		i, j;
	int32_t		hrm_chan1_running_avg_accm, hrm_chan2_running_avg_accm;

#if 0
	/// Bias subtraction. ///
	for(i = 0; i < hrm_raw_index; i++)
	{
		hrm_chan1_raw[i] -= hrm_chan2_raw[i];
	}
#endif
	
	/// Average.
#if 1
	for(i = 0; i < (hrm_raw_index - HRM_AVG_SAMPLES); i++)
	{
		hrm_chan1_running_avg_accm = hrm_chan2_running_avg_accm = 0;
		for(j = 0; j < HRM_AVG_SAMPLES; j++)
		{
			hrm_chan1_running_avg_accm += hrm_chan1_raw[i+j];
			hrm_chan2_running_avg_accm += hrm_chan2_raw[i+j];
		}
		hrm_chan1_raw[i] = (int16_t)(hrm_chan1_running_avg_accm / HRM_AVG_SAMPLES);
		hrm_chan2_raw[i] = (int16_t)(hrm_chan2_running_avg_accm / HRM_AVG_SAMPLES);
	}
	
	/// Add nomalization here. ///
#endif
#if 1
	for(i = (hrm_raw_index - HRM_AVG_SAMPLES); i > (hrm_raw_index - HRM_AVG_SAMPLES - HRM_AVG_SAMPLES); i--)
	{
		hrm_chan1_running_avg_accm = hrm_chan2_running_avg_accm = 0;
		for(j = 0; j < HRM_AVG_SAMPLES; j++)
		{
			hrm_chan1_running_avg_accm += hrm_chan1_raw[i+j];
			hrm_chan2_running_avg_accm += hrm_chan2_raw[i+j];
		}
		hrm_chan1_raw[i] = (int16_t)(hrm_chan1_running_avg_accm / HRM_AVG_SAMPLES);
		hrm_chan2_raw[i] = (int16_t)(hrm_chan2_running_avg_accm / HRM_AVG_SAMPLES);
	}
	
	/// Add nomalization here. ///
#endif
				
	
#if 1
	/// Bias subtraction. ///
	for(i = 0; i < hrm_raw_index; i++)
	{
		hrm_chan1_raw[i] -= hrm_chan2_raw[i];
	}
#endif
	

#if 1	
	/// Eliminate bounce by increaseing delta (dh).
	pd_emi_delta = 4;
	do
	{	
		detect_peak(hrm_chan1_raw, hrm_raw_index, pd_emi_delta, 1);
		dh = 0;
		pd_reduction_loop = num_emi_peaks;
		if(pd_reduction_loop > num_absop_peaks)
			pd_reduction_loop = num_absop_peaks;
		for(i=0; i<(pd_reduction_loop - 1); i++)
		{
			if( ((emi_peaks_xpos[i+1] - emi_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) 
				|| ((absop_peaks_xpos[i+1] - absop_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) )
			{
				dh++;
			}
		}
		
		if(dh > 0)
		{
			pd_emi_delta++;	
		}
		
		if(dh == 25)
			dh = 0;
	}while(dh > 0);
#else
	detect_peak(hrm_chan1_raw, hrm_raw_index, 8, 1);
#endif	

#if 1		
	/// Missing pulse correction:
	// run thru the absop_peaks_xpos[num_absop_peaks[0]]. find delta x for each. avg delta x. if 1 is double the rest add 1 to num_absop_peaks[0].
	
	old_num_of_emi_peaks = num_emi_peaks;
	old_num_of_absop_peaks = num_absop_peaks;
	for(k=0; k<1; k++)
	{
		avg_x_diff = 0;
		for(i=0; i<(old_num_of_emi_peaks - 1); i++)
		{
			avg_x_diff += (emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]);	
		}
		avg_x_diff = avg_x_diff / old_num_of_emi_peaks;
		avg_x_rng = avg_x_diff + (avg_x_diff / 2) + (avg_x_diff / 4);
	
		for(i=0; i<(old_num_of_emi_peaks - 1); i++)
		{
			if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) > avg_x_rng) 
			{
				num_emi_peaks++;
				for(j=old_num_of_emi_peaks; j<=(i+1); j--)
				{
					emi_peaks_xpos[j] = emi_peaks_xpos[j-1];
				}
				emi_peaks_xpos[i] = emi_peaks_xpos[i] + avg_x_diff;
			}
		}
	
		avg_x_diff = 0;
		for(i=0; i<(old_num_of_absop_peaks - 1); i++)
		{
			avg_x_diff += (absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]);	
		}
		avg_x_diff = avg_x_diff / old_num_of_absop_peaks;
		avg_x_rng = avg_x_diff + (avg_x_diff / 2) + (avg_x_diff / 4);
	
		for(i=0; i<(old_num_of_absop_peaks - 1); i++)
		{
			if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) > avg_x_rng)
			{
				num_absop_peaks++;
				for(j=old_num_of_absop_peaks; j<=(i+1); j--)
				{
					absop_peaks_xpos[j] = absop_peaks_xpos[j-1];
				}
				absop_peaks_xpos[i] = absop_peaks_xpos[i] + avg_x_diff;
			}
		}
	}
#endif
	
    /// Average the hrm output.
	current_hrm = ((num_emi_peaks * 10) + (current_hrm * 10) + (num_absop_peaks * 10)) / 12;
///	current_hrm = ((num_emi_peaks * 10) + (current_hrm * 4) + (num_absop_peaks * 10)) / 6;
	//current_hrm = ((num_emi_peaks * 10) + (current_hrm * 3) + (num_absop_peaks * 0)) / 4;
}
/// ========================================================================================== */

int8_t detect_peak(
					const int16_t*   	data, 		/* the data */ 
					int8_t             	data_count, /* row count of data */  
					int16_t          	delta, 		/* delta used for distinguishing peaks */
					int8_t             	emi_first 	/* should we search emission peak first of absorption peak first? */
					)
{
    int     i;
    double  mx;
    double  mn;
    int     mx_pos = 0;
    int     mn_pos = 0;
    int     is_detecting_emi = emi_first;


    mx = data[0];
    mn = data[0];

    num_emi_peaks = 0;
    num_absop_peaks = 0;

    for(i = 1; i < data_count; ++i)
    {
        if(data[i] > mx)
        {
            mx_pos = i;
            mx = data[i];
        }
        if(data[i] < mn)
        {
            mn_pos = i;
            mn = data[i];
        }

        if(is_detecting_emi &&
                data[i] < mx - delta)
        {
            if(num_emi_peaks >= HRM_DP_DATA_SIZE) /* not enough spaces */
                return 1;

            emi_peaks[num_emi_peaks] = mx; //mx_pos
			emi_peaks_xpos[num_emi_peaks] = mx_pos;
			num_emi_peaks++;
			
            is_detecting_emi = 0;

            i = mx_pos - 1;

            mn = data[mx_pos];
            mn_pos = mx_pos;
        }
        else if((!is_detecting_emi) &&
                data[i] > mn + delta)
        {
            if(num_absop_peaks >= HRM_DP_DATA_SIZE)
                return 2;
			
			absop_peaks_xpos[num_absop_peaks] = mn_pos;
            absop_peaks[num_absop_peaks] = mn;
            num_absop_peaks++;

            is_detecting_emi = 1;
            
            i = mn_pos - 1;

            mx = data[mn_pos];
            mx_pos = mn_pos;
        }
    }

    return 0;
}


int16_t PeakDiff1D(const int16_t *arr, size_t length) 
{
    // returns the maximum value of array
    size_t i;
    int16_t maximum = arr[0];
	int16_t minimum = arr[0];
	int16_t peakdiff;
    for (i = 1; i < length; ++i) 
	{
        if (maximum < arr[i]) {
            maximum = arr[i];
        }
		if (minimum > arr[i]) {
            minimum = arr[i];
        }
    }
	peakdiff = maximum - minimum;
    return peakdiff;
}


/*  Proto 1153 HRM code                                 
	uint8_t		hrm_running_avg_indx = 0;
	int16_t	hrm_chan1_running_avg[5], hrm_chan2_running_avg[5];
	int32_t	hrm_chan1_running_avg_accm, hrm_chan2_running_avg_accm;
	int16_t hrm_chan1_fbs = 0, hrm_chan2_fbs = 0;	
*/
void hrm_loop( void )
{
	static expire_timer_t hrm_task = { 0, 0 };
	
	if( hrm_task.wait_a_ticks == 0 )
	{	//Get Timer Rolling
		get_expire_time( (50*1000UL), &hrm_task );
	}
		
	if( check_expiration(&hrm_task) ) 
	{	//time to get another sample
		get_expire_time( (50*1000UL), &hrm_task );
		
		hrm_chan_1_raw32 = 	(Si115xReadFromRegister(SI115x_REG_HOSTOUT0) << 16) | 
							(uint16_t)(Si115xReadFromRegister(SI115x_REG_HOSTOUT1) << 8) | 
							(uint8_t)Si115xReadFromRegister(SI115x_REG_HOSTOUT2);
		
		hrm_chan_2_raw32 = 	(Si115xReadFromRegister(SI115x_REG_HOSTOUT3) << 16) | 
							(uint16_t)(Si115xReadFromRegister(SI115x_REG_HOSTOUT4) << 8) | 
							(uint8_t)Si115xReadFromRegister(SI115x_REG_HOSTOUT5);
			
		Si115xWriteToRegister(SI115x_REG_COMMAND, 0x11); // Force cmd
		
#if 1			
		/// Running Signal Average
		hrm_avg_chan132[hac_index] = hrm_chan_1_raw32;
		hrm_avg_chan232[hac_index] = hrm_chan_2_raw32;
		hrm_avg_chan132_accm = hrm_avg_chan232_accm = 0;
		for(i = 0; i < HRM_AVG_SIZE; i++)
		{
			hrm_avg_chan132_accm += hrm_avg_chan132[hac_index];
			hrm_avg_chan232_accm += hrm_avg_chan232[hac_index];
		}
		hrm_chan_1_raw32 = hrm_avg_chan132_accm / HRM_AVG_SIZE;
		hrm_chan_2_raw32 = hrm_avg_chan232_accm / HRM_AVG_SIZE;
		hac_index++;
		if(hac_index == HRM_AVG_SIZE)
			hac_index = 0;
#endif		
		
		
#if 1
		/// Running Bias Average.
		// To make this better and save memory reduce HRM_RUNNING_AVG_SAMPLES[<120>>2] and spread the samples over the entire last 6 sec frame.
//			hrm_chan1_running_avg[hrm_running_avg_indx] = hrm_chan_1_raw32;
//			hrm_chan2_running_avg[hrm_running_avg_indx] = hrm_chan_2_raw32;
//			hrm_chan1_running_avg_accm = 0;
//			hrm_chan2_running_avg_accm = 0;
//			for(i=0; i<HRM_RUNNING_AVG_SAMPLES; i++)
//			{
//				hrm_chan1_running_avg_accm += hrm_chan1_running_avg[i];
//				hrm_chan2_running_avg_accm += hrm_chan2_running_avg[i];
//			}
//			hrm_chan1_fbs = hrm_chan1_running_avg_accm / HRM_RUNNING_AVG_SAMPLES;
//			hrm_chan2_fbs = hrm_chan2_running_avg_accm / HRM_RUNNING_AVG_SAMPLES;
	//	if(((hrm_raw_index >> 1) & 0x01) == 0x01)
	//	{
			hrm_chan1_running_avg[(hrm_raw_index >> 2)] = hrm_chan_1_raw32;//cheeper to assign than qualify every forth one.
			hrm_chan2_running_avg[(hrm_raw_index >> 2)] = hrm_chan_2_raw32;
	//		hrm_running_avg_indx++;
			//if(hrm_running_avg_indx == HRM_RUNNING_AVG_SAMPLES)
				//hrm_running_avg_indx = 0;
	//	}
//#else
//			hrm_chan1_fbs = hrm_chan2_fbs = 0;
#endif
		
		/// Done, throw it in the pile. Now that the bias has been subtracted we can move down to 16 bits
		hrm_chan_1_raw32 = hrm_chan_1_raw32 - hrm_chan1_fbs;
		hrm_chan_2_raw32 = hrm_chan_2_raw32 - hrm_chan2_fbs;
		hrm_chan1_raw[hrm_raw_index] = hrm_chan_1_raw32; //cast as (int16_t)
		hrm_chan2_raw[hrm_raw_index] = hrm_chan_2_raw32;
	
#if 0			
		/// Stitcher.
		/* 	Break this into two pices. Calc the new bias while timing a threshold rejection. 
			Throw away the new bias if we recover in xx seconds or adopt the new hrm_chanX_fbs 
			if the event lasts it was a level change. A problem will occur if the event happens 
			in the last xx seconds of the frame. */

		if( (hrm_chan_1_raw32 > HRM_STITCH_TRESH) || (hrm_chan_1_raw32 < -HRM_STITCH_TRESH) || 
			(hrm_chan_2_raw32 > HRM_STITCH_TRESH) || (hrm_chan_2_raw32 < -HRM_STITCH_TRESH) )
		{
			if(!hrm_stitch_flag)
				hrm_raw_index_old = hrm_raw_index;
			hrm_stitch_flag = 1;
			hrm_chan1_fbs = hrm_chan1_fbs + ((hrm_chan_1_raw32) / 2);
			hrm_chan2_fbs = hrm_chan2_fbs + ((hrm_chan_2_raw32) / 2);
		}
		else if(hrm_stitch_flag)
		{
			hrm_stitch_flag = 0;
			if(hrm_raw_index_old >= 2)
				hrm_raw_index = hrm_raw_index_old - 2;
			else
				hrm_raw_index = hrm_raw_index_old; /// make rolling!!!
			
//				for(i=0; i<HRM_AVG_SAMPLES; i++)
//				{
//					hrm_chan1_running_avg[i] = hrm_chan_1_raw32; 
//					hrm_chan2_running_avg[i] = hrm_chan_2_raw32; 
//				}	
		}				
#endif		
		
		/// Pump it out. Pass queued messages to the radio.
		if(!hrm_stitch_flag)
		{
			char silly_data[sizeof(float)];
			memcpy(silly_data, &silly_ratio, sizeof silly_ratio);
			tx_type = HRM_PKT;
			tx_pay_len = 11;
			tx_payload[0] = (uint16_t) silly_data[0];
			tx_payload[1] = (uint16_t) silly_data[1];
			tx_payload[2] = (uint16_t) silly_data[2];
			tx_payload[3] = (uint16_t) silly_data[3];
			//tx_payload[0] = hrm_ch1_avg >> 8;
			//tx_payload[1] = hrm_ch1_avg & 0xFF;
			//tx_payload[2] = (uint16_t)(hrm_chan_1_raw32 + hrm_chan1_fbs) >> 8; Si115xReadFromRegister(SI115x_REG_HOSTOUT0
			//tx_payload[2] = (uint16_t) (hrm_chan_1_raw32 & 0x00FF0000) >> 16;
			//tx_payload[3] = (uint16_t) (hrm_chan_2_raw32 & 0x00FF0000) >> 16;
			//tx_payload[2] = (uint16_t) pd_emi_delta;
			//tx_payload[3] = (uint16_t)(hrm_chan_1_raw32 + hrm_chan1_fbs) & 0xFF;
			tx_payload[4] = current_hrm;
			tx_payload[5] = pd_emi_delta;
			//tx_payload[6] = (uint16_t) (hrm_chan_1_raw32 & 0x0000FF00) >> 8;;
			//tx_payload[7] = (uint16_t) (hrm_chan_1_raw32 & 0x000000FF);
			tx_payload[6] = (hrm_chan1_raw[hrm_raw_index]) >> 8;
			tx_payload[7] = hrm_chan1_raw[hrm_raw_index]  ;
			tx_payload[8] = (hrm_chan2_raw[hrm_raw_index]) >> 8;
			tx_payload[9] = hrm_chan2_raw[hrm_raw_index]  ;
			queue_packet_wrapper( tx_type, tx_payload, tx_pay_len );
			ble_transfer_packets_wrapper();		
			run_get_hrms++;
			
			if(run_get_hrms == 6 * 20) // 6sec * 20 sps
			{
				hrm_chan1_running_avg_accm = 0;
				hrm_chan2_running_avg_accm = 0;
				for(i = 0; i < 30; i++)
				{
					hrm_chan1_running_avg_accm += hrm_chan1_running_avg[i];
					hrm_chan2_running_avg_accm += hrm_chan2_running_avg[i];
				}
				hrm_chan1_fbs = hrm_chan1_running_avg_accm / 30;//(int32_t)(hrm_running_avg_indx);
				hrm_chan2_fbs = hrm_chan2_running_avg_accm / 30;//(int32_t)(hrm_running_avg_indx);
				silly_ratio = (float)hrm_chan1_fbs / (float)hrm_chan2_fbs;
				hrm_running_avg_indx = 0;
				
				Get_HRMs();
				
				app_trace_log(DEBUG_MED, "HRM: %01u @%01u\r\n", current_hrm, getSystemTimeMs());
				
				hrm_raw_index = 0;
				run_get_hrms = 0;
#if 0
				/// LED Current Control (AGC).
				hrm_agc_led_current_idx1 = 14 - (uint8_t)((abs(hrm_chan1_fbs) >> 8) - 25) / 3; // 32000 = 29(354mA), 1000 = 0(5.5mA), 14=100 mA
				if(hrm_agc_led_current_idx1 < 0)
					hrm_agc_led_current_idx1 = 0; // (5.5mA)
				if(hrm_agc_led_current_idx1 > 14)
					hrm_agc_led_current_idx1 = 14; // (14 = 100 mA limit)
				Si115xParamSet(PARAM_LED1_A, hrm_agc_led_current[hrm_agc_led_current_idx1]);
				///hrm_agc_led_current_idx1 = (uint8_t)(hrm_chan1_fbs >> 8);
				hrm_agc_led_current_idx3 = 14 - (uint8_t)((abs(hrm_chan2_fbs) >> 8) - 25) / 3; // 32000 = 29(354mA), 1000 = 0(5.5mA), 14=100 mA
				if(hrm_agc_led_current_idx3 < 0)
					hrm_agc_led_current_idx3 = 0; // (5.5mA)
				if(hrm_agc_led_current_idx3 > 14)
					hrm_agc_led_current_idx3 = 14; // (14 = 100 mA limit)
				Si115xParamSet(PARAM_LED3_A, hrm_agc_led_current[hrm_agc_led_current_idx3]);
				
#endif	
#if 0
				/// LED Current Control (AGC). 
				//  chan1 = 2.6k(OPEN), 8.2k(OD29), 16.2k(OF29)   		
				//  chan2 = 500(OPEN), 16.3k(OD29), 21.4k(OF29)
				if( (hrm_chan1_fbs < 10000) && (hrm_agc_led_current_idx1 > 15)) // limit to 100mA
				{
					hrm_agc_led_current_idx1--;
					Si115xParamSet(PARAM_LED1_A, hrm_agc_led_current[hrm_agc_led_current_idx1]);
					//Si115xParamSet(PARAM_LED1_B, hrm_agc_led_current[hrm_agc_led_current_idx]);
				}
				
				if( (hrm_chan1_fbs > 16000) && (hrm_agc_led_current_idx1 < 29)) // limit to 5.5 mA
				{
					hrm_agc_led_current_idx1++;
					Si115xParamSet(PARAM_LED1_A, hrm_agc_led_current[hrm_agc_led_current_idx1]);
					//Si115xParamSet(PARAM_LED1_B, hrm_agc_led_current[hrm_agc_led_current_idx]);
				}
				
				if( (hrm_chan2_fbs < 10000) && (hrm_agc_led_current_idx3 > 15))
				{
					hrm_agc_led_current_idx3--;
					Si115xParamSet(PARAM_LED3_A, hrm_agc_led_current[hrm_agc_led_current_idx3]);
					//Si115xParamSet(PARAM_LED1_B, hrm_agc_led_current[hrm_agc_led_current_idx]);
				}
				
				if( (hrm_chan2_fbs > 16000) && (hrm_agc_led_current_idx3 < 29))
				{
					hrm_agc_led_current_idx3++;
					Si115xParamSet(PARAM_LED3_A, hrm_agc_led_current[hrm_agc_led_current_idx3]);
					//Si115xParamSet(PARAM_LED1_B, hrm_agc_led_current[hrm_agc_led_current_idx]);
				}
#endif						
			}
			hrm_raw_index++;
		}
		
		/// See if the host wants anything.
		hrm_read_cmd(); 
	}
	///else    if(!hrm_stitch_flag)
}
