/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Peak Detector.c                                  			         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Include files                                                             */
/*---------------------------------------------------------------------------*/
#include <analysis.h>
#include <ansi_c.h>
#include <math.h>
#include <cvirte.h>
#include <userint.h>
#include <rs232.h>
#include <utility.h>
#include <formatio.h>
//#include <string.h>
#include <toolbox.h>
#include "Activity Discriminator vars.h" 
#include "Activity Discriminator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int CVICALLBACK Peak_Detector_CB (int panel, int control, int event,
								  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		
			Apply_Peak_Detector();
			
					break;
	}
	return 0;
}


void Apply_Peak_Detector(void)
{
	double 	pd_threshold_val;
	int		i, j, pd_sel, pd_auto_threshold, pd_threshold_width, pd_source, one_shot;
	char	tmp_str[64];
	double 	hrm_7seg_out;
	int 	dh, pd_emi_auto, pd_absop_auto, pd_emi_delta, pd_absop_delta, pd_emi_width;
	double	long_hart_fft[2048], raw_hart_fft[4100];

	GetCtrlVal(mainpnl, MAINPNL_ONE_SHOT, &one_shot);
	if(one_shot)
		SetCtrlVal(mainpnl, MAINPNL_GETRTDATA, 0); 
	
	GetCtrlVal(mainpnl, MAINPNL_PEAK_DETECTOR_SEL, &pd_sel);
	GetCtrlVal(mainpnl, MAINPNL_PD_THRESH, &pd_threshold_val);
	GetCtrlVal(mainpnl, MAINPNL_PD_THRESH_AUTO, &pd_auto_threshold);
	GetCtrlVal(mainpnl, MAINPNL_PD_THRESH_W, &pd_threshold_width);
	if(pd_threshold_width < 3)
		pd_threshold_width = 3;
	if(pd_auto_threshold)
		pd_threshold_val = ssgp;	
	
		GetCtrlVal(mainpnl, MAINPNL_PD_SOURCE, &pd_source);
		GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time);
		if(pd_peak_cnt > 2)
			hrm_7seg_out = (double)( ((double)pd_peak_cnt + 0.0) / run_time ) * 60.0;
		else
			hrm_7seg_out = 0;
		SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_2, hrm_7seg_out);
		/// ================================================================== ///
		DeleteGraphPlot (mainpnl, MAINPNL_SIG4GRAPH, -1, VAL_IMMEDIATE_DRAW);
///		Peak_MM(raw_hart, ser_input_size, pd_threshold_val, pd_threshold_width); 
//int detect_peak(
//        const double*   data, /* the data */ 
//        int             data_count, /* row count of data */ 
//        int*            emi_peaks, /* emission peaks will be put here */ 
//        int*            num_emi_peaks, /* number of emission peaks found */
//        int             max_emi_peaks, /* maximum number of emission peaks */ 
//        int*            absop_peaks, /* absorption peaks will be put here */ 
//        int*            num_absop_peaks, /* number of absorption peaks found */
//        int             max_absop_peaks, /* maximum number of absorption peaks
//                                            */ 
//        double          delta, /* delta used for distinguishing peaks */
//        int             emi_first /* should we search emission peak first of
//                                     absorption peak first? */
//        )
#if 0    /* formerly excluded lines */
/// =========> PD
		int pdreturn = detect_peak(raw_hart, ser_input_size, emi_peaks, num_emi_peaks, 64, absop_peaks, num_absop_peaks, 64, (double)pd_threshold_width, 1);
		
		hrm_7seg_out = (double)( ((double)num_emi_peaks[0]) / run_time ) * 60.0; 
		sprintf(tmp_str, "pdreturn %d, num_emi_peaks %d, hrm = %d ppm\n", pdreturn, num_emi_peaks[0], (int)(hrm_7seg_out));
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);
		hrm_7seg_out = (double)( ((double)num_absop_peaks[0]) / run_time ) * 60.0; 
		sprintf(tmp_str, "pdreturn %d, num_absop_peaks %d, hrm = %d ppm\n", pdreturn, num_absop_peaks[0], (int)(hrm_7seg_out));
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);
		
		//PlotXY (mainpnl, MAINPNL_SIG4GRAPH, peak_position, peak_ampl, pd_peak_cnt, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_RED);
		//if( pdcount > 0)
		for(i=0; i<num_emi_peaks[0]; i++)
		{
			emi_peaks_dfp[i] = (double)emi_peaks[i]; 
			absop_peaks_dfp[i] = (double)absop_peaks[i];
			emi_peaks_xpos_dfp[i] = (double)emi_peaks_xpos[i]; 
			absop_peaks_xpos_dfp[i] = (double)absop_peaks_xpos[i];
		}
		if(num_emi_peaks[0] >= 1)
			PlotXY (mainpnl, MAINPNL_SIG4GRAPH, emi_peaks_xpos_dfp, emi_peaks_dfp, num_emi_peaks[0], VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_RED);
		if(num_absop_peaks[0] >= 1)
			PlotXY (mainpnl, MAINPNL_SIG4GRAPH, absop_peaks_xpos_dfp, absop_peaks_dfp, num_absop_peaks[0], VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_BLUE);
		
#endif   /* formerly excluded lines */
		/// double	final_hrm_avg[64], final_hrm_avg_old, final_hrm_avg_second_diff ///
		/// int		final_hrm_avg_index, final_hrm_avg_valid_index, fo_avg_len, fo_start; ///
		fo_avg_len = 2;  // zero based
		final_hrm_avg_second_diff = 0.85;
		final_hrm_avg_valid_index = 0;
		
		if(fo_start)
		{
			for(i=0; i <= fo_avg_len; i++)
			{
				final_hrm_avg[i] = hrm_7seg_out;
			}
			///final_hrm_avg = hrm_7seg_out;
			final_hrm_avg_index = fo_avg_len;
			fo_start = 0;
			final_hrm_avg_old = hrm_7seg_out;
		}
		
		for(i=final_hrm_avg_index - fo_avg_len; i <= final_hrm_avg_index; i++)
		{
			if( (final_hrm_avg[i] > (final_hrm_avg_old * final_hrm_avg_second_diff)) && (final_hrm_avg[i] < (final_hrm_avg_old * (2 - final_hrm_avg_second_diff))) )
			{
				hrm_7seg_out += final_hrm_avg[i];
				fo_hit++;
			}
			else
			{
				hrm_7seg_out += final_hrm_avg_old; 
				fo_miss++;
			}
			 SetCtrlVal(mainpnl, MAINPNL_HIT, fo_hit/fo_avg_len); 
			 SetCtrlVal(mainpnl, MAINPNL_MISS, fo_miss/fo_avg_len);
			final_hrm_avg_valid_index++;   
		}
		
		 hrm_7seg_out = hrm_7seg_out / (double)(final_hrm_avg_valid_index + 1);
		 SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_3, hrm_7seg_out);    
		
		 final_hrm_avg_index++;
		 final_hrm_avg[final_hrm_avg_index] = hrm_7seg_out; 
		 final_hrm_avg_old = hrm_7seg_out;
		 
		 
		Update_Sig_Plots();
		
		
/// ===========================================================================================================================		
/// =========> PD
	int16_t 	k;
	int32_t		hrm_chan1_running_avg_accm, hrm_chan2_running_avg_accm;
	int			HRM_AVG_SAMPLES, PD_DEBOUNCE_WIDTH;
	
	GetCtrlVal(mainpnl, MAINPNL_GYRAVGFV, &HRM_AVG_SAMPLES);
	GetCtrlVal(mainpnl, MAINPNL_PD_EMI_WIDTH, &PD_DEBOUNCE_WIDTH);
	GetCtrlVal(mainpnl, MAINPNL_PD_EMI_DELTA, &pd_emi_delta);
	GetCtrlVal(mainpnl, MAINPNL_PD_EMI_THRESH_AUTO, &pd_emi_auto);
	
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
		hrm_chan1_raw_avg[i] = (int16_t)(hrm_chan1_running_avg_accm / HRM_AVG_SAMPLES);
		hrm_chan2_raw_avg[i] = (int16_t)(hrm_chan2_running_avg_accm / HRM_AVG_SAMPLES);
	}
	/// Add nomalization here. ///
#endif
#if 1
	for(i = (hrm_raw_index); i > (hrm_raw_index - HRM_AVG_SAMPLES - HRM_AVG_SAMPLES); i--)
	{
		hrm_chan1_running_avg_accm = hrm_chan2_running_avg_accm = 0;
		for(j = 0; j < HRM_AVG_SAMPLES; j++)
		{
			hrm_chan1_running_avg_accm += hrm_chan1_raw[i-j];
			hrm_chan2_running_avg_accm += hrm_chan2_raw[i-j];
		}
		hrm_chan1_raw_avg[i] = (int16_t)(hrm_chan1_running_avg_accm / HRM_AVG_SAMPLES);
		hrm_chan2_raw_avg[i] = (int16_t)(hrm_chan2_running_avg_accm / HRM_AVG_SAMPLES);
	}
#endif
				
	
#if 1
	/// Quick Bias subtraction. ///
	
	int16_t bais_sub_c1 = hrm_chan1_raw_avg[1];
	int16_t bais_sub_c2 = hrm_chan2_raw_avg[1];

	
	for(i = 0; i < hrm_raw_index; i++)
	{
		hrm_chan1_raw_avg[i] -=  bais_sub_c1;
		hrm_chan2_raw_avg[i] -=  bais_sub_c2; 
	}
#endif

/// Auto Subtract ///
	int  	run_auto_p2sf;
	double  last_pvariance, pvariance, pmax, pmin, pmaxi, pmini, pmean=0;
	char 	out_str[128];
	
		for(j=0; j<(ser_input_size + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES); j++)
		{
			intgtr_gyr_p[j] = (double)hrm_chan1_raw[j];
			intgtr_gyr_y[j] = (double)hrm_chan2_raw[j];
		}
		/// Variance ( intgtr_gyr_p, ser_input_size ,&pmean , &pvariance );
		MaxMin1D ( intgtr_gyr_p, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
		pvariance = pmax - pmin;
		sprintf(out_str, "pvariance %f, pmean %f\n", pvariance, pmean);
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
		
		if(pvariance > 200.0)
		{
			GetCtrlVal(mainpnl, MAINPNL_AUTO_P2SF, &run_auto_p2sf);
			if(run_auto_p2sf)
			{
//				for(j=0; j<(ser_input_size + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES); j++)
//				{
//					intgtr_gyr_p[j] = (double)hrm_chan1_raw[j];
//					intgtr_gyr_y[j] = (double)hrm_chan2_raw[j];
//				}
//				Variance ( intgtr_gyr_p, ser_input_size ,&pmean , &pvariance );
				phase2sf = 0.0;
				phase2sfcof = 0.5;
					MaxMin1D ( intgtr_gyr_p, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
					pvariance = pmax - pmin;
					MaxMin1D ( intgtr_gyr_y, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
					phase2sf = fabs(pvariance / (pmax - pmin));
					last_pvariance = 1000000.0;
					/// printf("%f\n",phase2sf);
				for(int ol=0;  ol < 1; ol++)  /// 3
				{
					if(ol==0) /// 1
						phase2sfcof = 0.05; /// 0.05
					if(ol==1) /// 2
						phase2sfcof = 0.001; /// 0.005
						
					last_pvariance = 1000000.0;	 

					while(fabs(last_pvariance) > fabs(pvariance))
					{
						last_pvariance = pvariance;
						phase2sf += phase2sfcof;
						SetCtrlVal(mainpnl, MAINPNL_PHASE2SF, phase2sf);
					
						for(i=0; i<(ser_input_size  + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES); i++)
						{
							hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw_avg[i] - ((double)hrm_chan2_raw_avg[i] * phase2sf));
						}
						for(j=0; j<(ser_input_size  + HRM_AVG_SAMPLES); j++)
						{
							intgtr_gyr_p[j] = (double)hrm_chan3_raw[j];
						}
						MaxMin1D ( intgtr_gyr_p, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
						if(pmax > pmin)
							pvariance = pmax - pmin;
						else
							pvariance = pmin - pmax;
//						sprintf(out_str, "phase2sf=%f,  +pvariance %f < %f\n", phase2sf, pvariance, last_pvariance);
//						SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
					}
					
					last_pvariance = 1000000.0;
					///phase2sf -= 0.1;
					while(fabs(last_pvariance) > fabs(pvariance))
					{
						last_pvariance = pvariance;
						phase2sf -= phase2sfcof;
						SetCtrlVal(mainpnl, MAINPNL_PHASE2SF, phase2sf);
					
						for(i=0; i<(ser_input_size  + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES); i++)
						{
							hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw_avg[i] - ((double)hrm_chan2_raw_avg[i] * phase2sf));
						}
					
						for(j=0; j<(ser_input_size  + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES); j++)
						{
							intgtr_gyr_p[j] = (double)hrm_chan3_raw[j];
						}
						MaxMin1D ( intgtr_gyr_p, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
						if(pmax > pmin)
							pvariance = pmax - pmin;
						else
							pvariance = pmin - pmax;	
//						sprintf(out_str, "phase2sf=%f,  -pvariance %f, <%f\n",phase2sf, pvariance, last_pvariance);
//						SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
					}
				}
				for(i=0; i<(ser_input_size  + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES); i++)
				{
					hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw_avg[i] - ((double)hrm_chan2_raw_avg[i] * (phase2sf + phase2sfcof)));
				}
			
				sprintf(out_str, "=========================\n");
				SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
			}
			else
			{
				
				for(i = 0; i < hrm_raw_index + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES; i++)
				{
					hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw_avg[i] - ((double)hrm_chan2_raw_avg[i] * phase2sf));
				}
			}
		}	
		else
		{	 	
			for(i = 0; i < hrm_raw_index + HRM_AVG_SAMPLES + HRM_AVG_SAMPLES; i++)
			{
				hrm_chan3_raw[i] = hrm_chan1_raw_avg[i];
			}
		}
		
#if 1	
	/// Eliminate bounce by increasing delta (dh). Basicaly acts as a auto peak detector threshold control.
	/// It looks like starting from low to high sometimes eliminates wanted peaks. 
	/// If we start from high and look for a minimum number of peaks we could post process after substracting a proprtional
	/// amount to insure that we get some of the dis-qualified peaks. De bounce would have to then take the form of the missing peaks
	/// routine.  Adding more averaging may yeild a completly different result.
	
	pd_emi_delta = 40;
	sprintf(out_str, "Eliminate bounce1: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks);
	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	if(pd_emi_auto)
	{
		do
		{	
			detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1);
			pd_emi_delta--;
		}while((num_emi_peaks < 5) || (num_absop_peaks < 5));
		sprintf(out_str, "Eliminate bounce2: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks); 
		 SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	 
		pd_emi_delta -= 2; // means delta - 2. should be porpotional...
	}
	detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1); 
	
	 sprintf(out_str, "Eliminate bounce3: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks); 
	 SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
#else
///	detect_peak(hrm_chan3_raw, hrm_raw_index, 8, 1);
#endif	
	
#if 0	
	/// Eliminate bounce by increasing delta (dh). Basicaly acts as a auto peak detector threshold control.
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
	int track;
	
	sprintf(out_str, "Missing pulse correction1: emip = %d, absopp = %d\n", num_emi_peaks, num_absop_peaks);
	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	if(num_emi_peaks > 63)
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

//for(track=0; track<num_emi_peaks; track++)
//{
//	printf("in %d ex = %d, ey = %d\n", track, emi_peaks_xpos[track], emi_peaks[track]); 	
//}

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
	
	
	for(k=0; k<2; k++)
	{
		int16_t avg_x_diff = 0;
		old_num_of_emi_peaks = num_emi_peaks;
		old_num_of_absop_peaks = num_absop_peaks;
		
		if(old_num_of_emi_peaks > 0)
		{
			for(i=0; i<(old_num_of_emi_peaks - 1); i++) // Why limit x axis range???
			//for(i=0; i<(old_num_of_emi_peaks); i++)   
			{
				avg_x_diff += (emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]);   
			}
			avg_x_diff = avg_x_diff / old_num_of_emi_peaks; /// <================================
			avg_x_rng = avg_x_diff + (avg_x_diff / 2);// + (avg_x_diff / 4);
		
//printf("k%d START: avg ex = %d, avg_x_rng = %d, old_num_of_emi_peaks = %d, \n", k, avg_x_diff, avg_x_rng, old_num_of_emi_peaks); 
		
		}
		
//for(track=0; track<num_emi_peaks; track++)
//{
//	printf("in %d ex = %d, ey = %d\n", track, emi_peaks_xpos[track], emi_peaks[track]); 	
//}
		
		for(i=0; i<=(old_num_of_emi_peaks /*- 1*/); i++)   /// 10 turned into 18 because avg x rng was too small or travelling shift.
		{
			if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) > avg_x_rng) 
			{
				//for(j=old_num_of_emi_peaks; j<=(i+1); j--)
				for(j=num_emi_peaks + 0; j>(i); j--)
				{
//printf("i = %d, J = %d ex = %d, ey = %d\n", i, j, emi_peaks_xpos[j], emi_peaks_xpos[j-1]);
					emi_peaks_xpos[j] = emi_peaks_xpos[j - 1];
					emi_peaks[j] = emi_peaks[j - 1];
					 
				}
				//emi_peaks_xpos[i] = emi_peaks_xpos[i] + (avg_x_diff / (old_num_of_emi_peaks +1));
				emi_peaks_xpos[j+1] = emi_peaks_xpos[j] + (avg_x_diff); 
				emi_peaks[j+1] = 0;
				num_emi_peaks++;
				//sprintf(out_str, "Found missing emi: emi_peaks_xpos[i] %d\n", emi_peaks_xpos[i]);
				//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
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
		
//for(track=0; track<num_emi_peaks; track++)
//{
//	printf("out %d ex = %d, ey = %d\n", track, emi_peaks_xpos[track], emi_peaks[track]); 	
//}
	
		sprintf(out_str, "=>Missing pulse correction2: emixrng = %d, emip = %d, ", avg_x_rng, num_emi_peaks);
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	for(k=0; k<2; k++)
	{	
		if(num_emi_peaks > 63)
		num_emi_peaks = 63;
		if(num_absop_peaks > 63)
		num_absop_peaks = 63;
		avg_x_diff = 0;
		old_num_of_emi_peaks = num_emi_peaks;
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
				sprintf(out_str, "Found missing absop: absop_peaks_xpos[i] %d\n", absop_peaks_xpos[i]);
				SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
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
		
		
		
		sprintf(out_str, "=>Missing pulse correction3: absopxrng = %d, absopp = %d\n", avg_x_rng, num_absop_peaks);
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	}
#endif

#if 1	
	/// Eliminate Bounce.
	old_num_of_emi_peaks = num_emi_peaks;
	avg_x_rng = avg_x_rng/3;
	for(i=0; i<num_emi_peaks; i++)
	{
		if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) < avg_x_rng)
		{
			old_num_of_emi_peaks--;
			sprintf(out_str, "Found extra emi: emi_peaks_xpos[i] %d\n", emi_peaks_xpos[i]);
			SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
		}
	}
	num_emi_peaks = old_num_of_emi_peaks;
	
	old_num_of_absop_peaks = num_absop_peaks;
	for(i=0; i<num_absop_peaks; i++)
	{
		if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) < avg_x_rng)
		{
			old_num_of_absop_peaks--;
			sprintf(out_str, "Found extra absop: absop_peaks_xpos[i] %d\n", absop_peaks_xpos[i]);
			SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
		}
	}
	num_absop_peaks = old_num_of_absop_peaks;
	
 	sprintf(out_str, "Eliminate bounce2: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks); 
 	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);

///	detect_peak(hrm_chan3_raw, hrm_raw_index, 8, 1);
#endif	
	
	
/// 7 SEG Output ///
	//current_hrm = (((num_emi_peaks * 10) + (num_absop_peaks * 10)) / 2);  
	SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_9, (unsigned char)(((num_emi_peaks * 10) + (num_absop_peaks * 10)) / 2));
    /// Average the hrm output.
	/// current_hrm = ((num_emi_peaks * 10) + (current_hrm * 10) + (num_absop_peaks * 10)) / 12;
	current_hrm = ((num_emi_peaks * 10) + (current_hrm * 4) + (num_absop_peaks * 10)) / 6;
	//current_hrm = ((num_emi_peaks * 10) + (current_hrm * 3) + (num_absop_peaks * 0)) / 4;
	//current_hrm = ((num_emi_peaks * 10) + (num_absop_peaks * 10)) / 2;  
	SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG, (double)current_hrm); //
	SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_3 , (((double)current_hrm + (double)hrm_52 ) / 2)) ;
	
/// CVI Stuff: ///
	
	double 	ymin, ymax;
	char	scaledeltastr[32];
	/// Plot  raw
	for(i = 0; i < hrm_raw_index; i++)
	{
		raw_hart[i] = (double)hrm_chan1_raw[i];
	}
	DeleteGraphPlot (mainpnl, MAINPNL_SIG2GRAPH, -1, VAL_IMMEDIATE_DRAW);
	PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_hart, hrm_raw_index, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
	GetAxisScalingMode (mainpnl, MAINPNL_SIG2GRAPH, VAL_LEFT_YAXIS, NULL, &ymin, &ymax);
	sprintf(scaledeltastr, "%f", ymax - ymin);
	PlotText (mainpnl, MAINPNL_SIG2GRAPH, 0.0, raw_hart[2], scaledeltastr, VAL_APP_META_FONT, VAL_BLACK, VAL_TRANSPARENT);
	for(i = 0; i < hrm_raw_index; i++)
	{
		raw_hart[i] = (double)hrm_chan2_raw[i];
	}
	DeleteGraphPlot (mainpnl, MAINPNL_SIG3GRAPH, -1, VAL_IMMEDIATE_DRAW);
	PlotY (mainpnl, MAINPNL_SIG3GRAPH, raw_hart, hrm_raw_index, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
	GetAxisScalingMode (mainpnl, MAINPNL_SIG3GRAPH, VAL_LEFT_YAXIS, NULL, &ymin, &ymax);
	sprintf(scaledeltastr, "%f", ymax - ymin);
	PlotText (mainpnl, MAINPNL_SIG3GRAPH, 0.0, raw_hart[2], scaledeltastr, VAL_APP_META_FONT, VAL_BLACK, VAL_TRANSPARENT);
	/// Plot subtract
	GetCtrlVal(mainpnl, MAINPNL_PHASE2SF, &phase2sf);
	if(phase2sf)
	{
		for(i = 0; i < hrm_raw_index; i++)
		{
			raw_hart[i] = (double)(hrm_chan3_raw[i]);
		}
		DeleteGraphPlot (mainpnl, MAINPNL_SIG4GRAPH, -1, VAL_IMMEDIATE_DRAW);
		PlotY (mainpnl, MAINPNL_SIG4GRAPH, raw_hart, hrm_raw_index, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		GetAxisScalingMode (mainpnl, MAINPNL_SIG4GRAPH, VAL_LEFT_YAXIS, NULL, &ymin, &ymax);
		sprintf(scaledeltastr, "%f", ymax - ymin);
		PlotText (mainpnl, MAINPNL_SIG4GRAPH, 0.0, raw_hart[2], scaledeltastr, VAL_APP_META_FONT, VAL_BLACK, VAL_TRANSPARENT);
	}

	/// Plot peaks
#if 1
//		printf("===================================\n");	
		for(i=0; i<num_emi_peaks; i++)
		{
			emi_peaks_dfp[i] = (double)emi_peaks[i]; 
			emi_peaks_xpos_dfp[i] = (double)emi_peaks_xpos[i]; 
//			printf("\nindx %d, expos = %d, ey= %d\n", i, emi_peaks_xpos[i], emi_peaks[i]);
		}
//		printf("ABSORPSION\n");
		for(i=0; i<num_absop_peaks; i++)
		{											 
			absop_peaks_dfp[i] = (double)absop_peaks[i];
			absop_peaks_xpos_dfp[i] = (double)absop_peaks_xpos[i];
//			printf("indx %d, axpos = %d, ay= %d\n", i, absop_peaks_xpos[i], absop_peaks[i]);
		}
		if(num_emi_peaks >= 1)
			PlotXY (mainpnl, MAINPNL_SIG4GRAPH, emi_peaks_xpos_dfp, emi_peaks_dfp, num_emi_peaks, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_GREEN);
		if(num_absop_peaks >= 1)
			PlotXY (mainpnl, MAINPNL_SIG4GRAPH, absop_peaks_xpos_dfp, absop_peaks_dfp, num_absop_peaks, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_BLUE);

#endif
/// ===========================================================================================================================	  
/// =========> PD 	
	

#if 0
		for(i=look_back_starting_point; i<(look_back_starting_point + ser_input_size); i++)
		{
			 raw_hart[i] =  intgtr_gyr_p[i - look_back_starting_point] + 0; //100
		}
		
		look_back_starting_point += ser_input_size; 
		j = 0;
		for(i = (look_back_starting_point - (20 * 30)); i < look_back_starting_point; i++)
		{
			if(i >= 0)
			{
				long_hart[j] = raw_hart[i];
				j++;
			}
		}
		
		if(look_back_starting_point > 3000)
			look_back_starting_point = (20 * 30);
		
		total_run_time = j / 20;
		last_data_stream_cir_buff_idx += data_stream_cir_buff_idx;
		
		DeleteGraphPlot (mainpnl, MAINPNL_HRM_DAY_GRAPH, -1, VAL_IMMEDIATE_DRAW); 
		PlotY (mainpnl, MAINPNL_HRM_DAY_GRAPH, long_hart, j, VAL_DOUBLE, VAL_BASE_ZERO_VERTICAL_BAR, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_CYAN);
		
		PeakDetector (long_hart, j, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
		
		hrm_7seg_out = (double)( ((double)pd_peak_cnt) / total_run_time ) * 60.0;
		SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG, hrm_7seg_out);
		sprintf(tmp_str, "%3.1f / Min", hrm_7seg_out);
		SetCtrlVal(mainpnl, MAINPNL_HART_BEAT_DISP, tmp_str);
			
		Apply_Histogram_Gyro();
		Apply_Histogram_Acc();
		Apply_Histogram_Mag();
		Determine_Activity();
		
		frequency = ser_input_size / run_time;
		delta_t=1/frequency;
	    delta_f=1/(j*delta_t);
	    frequency_array[0]=0.0;
		for (i=0;i<j;i++)
	    {
	        frequency_array[i]=i*delta_f*60;
	    }
	
		Copy1D (long_hart, j, long_hart_fft);
		Spectrum (long_hart_fft, j);
		for(i=0; i<11; i++)
		{
			long_hart_fft[i] = 0;
		}
		DeleteGraphPlot (mainpnl, MAINPNL_HRM_FFT_GRAPH, -1, VAL_IMMEDIATE_DRAW); 
	    PlotXY (mainpnl, MAINPNL_HRM_FFT_GRAPH, frequency_array, long_hart_fft, j,
	            VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_EMPTY_SQUARE,
	            VAL_SOLID,1, VAL_RED); 
		
		frequency = ser_input_size / run_time;
		delta_t=1/frequency;
	    delta_f=1/(ser_input_size*delta_t);
		
	    frequency_array[0]=0.0;
		for (i=0;i<ser_input_size;i++)
	    {
	        frequency_array[i]=i*delta_f*60;
	    }
		
		Subset1D (raw_hart, look_back_starting_point, (look_back_starting_point - ser_input_size), ser_input_size, raw_hart_fft);
		Spectrum (raw_hart_fft, ser_input_size);
		for(i=0; i<11; i++)
		{
			raw_hart_fft[i] = 0;
		}
		DeleteGraphPlot (mainpnl, MAINPNL_HRM_FFT_GRAPH_RT, -1, VAL_IMMEDIATE_DRAW); 
	    PlotXY (mainpnl, MAINPNL_HRM_FFT_GRAPH_RT, frequency_array, raw_hart_fft, ser_input_size,
	            VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_EMPTY_SQUARE,
	            VAL_SOLID,1, VAL_RED); 
#endif		
	   
}


int CVICALLBACK ReStart_FO (int panel, int control, int event,
							void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			final_hrm_avg_index = 0;
			fo_start = 1;
			fo_hit = fo_miss = 0;
			break;
	}
	return 0;
}

/// ===============================PD===================================================///

int8_t detect_peak(
					const int16_t*   	data, 		/* the data */ 
					int8_t             	data_count, /* row count of data */  
					int16_t          	delta, 		/* delta used for distinguishing peaks */
					int8_t             	emi_first 	/* should we search emission peak first of
														absorption peak first? */
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
	// Add 1 to Num because it was an index, not a count.
///	num_emi_peaks++;
///    num_absop_peaks++;
	return 0;
}


//int detect_peak(
//        const double*   data, /* the data */ 
//        int             data_count, /* row count of data */ 
//        int*            emi_peaks, /* emission peaks will be put here */ 
//        int*            num_emi_peaks, /* number of emission peaks found */
//        int             max_emi_peaks, /* maximum number of emission peaks */ 
//        int*            absop_peaks, /* absorption peaks will be put here */ 
//        int*            num_absop_peaks, /* number of absorption peaks found */
//        int             max_absop_peaks, /* maximum number of absorption peaks
//                                            */ 
//        double          delta, /* delta used for distinguishing peaks */
//        int             emi_first /* should we search emission peak first of
//                                     absorption peak first? */
//        )
//{
//    int     i;
//    double  mx;
//    double  mn;
//    int     mx_pos = 0;
//    int     mn_pos = 0;
//    int     is_detecting_emi = emi_first;
//
//
//    mx = data[0];
//    mn = data[0];
//
//    *num_emi_peaks = 0;
//    *num_absop_peaks = 0;
//
//    for(i = 1; i < data_count; ++i)
//    {
//        if(data[i] > mx)
//        {
//            mx_pos = i;
//            mx = data[i];
//        }
//        if(data[i] < mn)
//        {
//            mn_pos = i;
//            mn = data[i];
//        }
//
//        if(is_detecting_emi &&
//                data[i] < mx - delta)
//        {
//            if(*num_emi_peaks >= max_emi_peaks) /* not enough spaces */
//                return 1;
//
//            emi_peaks[*num_emi_peaks] = mx; //mx_pos
//			emi_peaks_xpos[*num_emi_peaks] = mx_pos;
//			++ (*num_emi_peaks);
//			
//            is_detecting_emi = 0;
//
//            i = mx_pos - 1;
//
//            mn = data[mx_pos];
//            mn_pos = mx_pos;
//        }
//        else if((!is_detecting_emi) &&
//                data[i] > mn + delta)
//        {
//            if(*num_absop_peaks >= max_absop_peaks)
//                return 2;
//			
//			absop_peaks_xpos[*num_absop_peaks] = mn_pos;
//            absop_peaks[*num_absop_peaks] = mn;
//            ++ (*num_absop_peaks);
//
//            is_detecting_emi = 1;
//            
//            i = mn_pos - 1;
//
//            mx = data[mn_pos];
//            mx_pos = mn_pos;
//        }
//    }
//
//    return 0;
//}
