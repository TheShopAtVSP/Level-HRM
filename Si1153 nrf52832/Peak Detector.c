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
	int		pd_sel, pd_auto_threshold, pd_threshold_width, pd_source;
	char	tmp_str[64];
	double 	hrm_7seg_out;
	int 	i, j, dh, pd_emi_auto, pd_absop_auto, pd_emi_delta, pd_absop_delta, pd_emi_width;
	double	long_hart_fft[2048], raw_hart_fft[4100];

	GetCtrlVal(mainpnl, MAINPNL_PEAK_DETECTOR_SEL, &pd_sel);
	GetCtrlVal(mainpnl, MAINPNL_PD_THRESH, &pd_threshold_val);
	GetCtrlVal(mainpnl, MAINPNL_PD_THRESH_AUTO, &pd_auto_threshold);
	GetCtrlVal(mainpnl, MAINPNL_PD_THRESH_W, &pd_threshold_width);
	if(pd_threshold_width < 3)
		pd_threshold_width = 3;
	if(pd_auto_threshold)
		pd_threshold_val = ssgp;	
	
	GetCtrlVal(mainpnl, MAINPNL_PD_SOURCE, &pd_source);
	
	if(pd_sel == 1)
	{
		if(pd_source == 1)
		  	PeakDetector (filt_acl_x, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 2)
		  	PeakDetector (filt_acl_y, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 3)
		  	PeakDetector (filt_acl_z, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 4)
		{
		  	PeakDetector (filt_gyr_p, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
		}
		if(pd_source == 5)
		  	PeakDetector (filt_gyr_y, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 6)
		  	PeakDetector (filt_gyr_r, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
		
		if(pd_source == 7)
		  	PeakDetector (intgtr_acl_x, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 8)
		  	PeakDetector (intgtr_acl_y, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 9)
		  	PeakDetector (intgtr_acl_z, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 10)
		  	PeakDetector (intgtr_gyr_p, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 11)
		  	PeakDetector (intgtr_gyr_y, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
	
		if(pd_source == 12)
		  	PeakDetector (intgtr_gyr_r, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
		
		if(pd_source == 13)
		{
			Get_Manitude(0);
		  	PeakDetector (acc_mag, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv);
		}
		
		if(pd_source == 14)
		{
			Get_Manitude(0);
		  	PeakDetector (gyr_mag, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv);
		}
		
		if(pd_source == 15)
		{
			Get_Manitude(1);
		  	PeakDetector (acc_mag, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv);
		}
		
		if(pd_source == 16)
		{
			Get_Manitude(1);
		  	PeakDetector (gyr_mag, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv);
		}
		
		if(pd_source == 17)
		{
		  	PeakDetector (raw_hart, ser_input_size + pd_threshold_width, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, \
					  		ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
			int hart_rate = pd_peak_cnt * (60 / run_time );
			sprintf(tmp_str, "%d / Min", hart_rate);
			SetCtrlVal(mainpnl, MAINPNL_HART_BEAT_DISP, tmp_str); 
		//	Peak_MM(raw_hart, ser_input_size, pd_threshold_val, pd_threshold_width);
		}
		
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
		
		
		
/// =========> PD
		for(i=0; i<(ser_input_size); i++)
		{
			 raw_hart[i] =  intgtr_gyr_p[i];
		}
		
		GetCtrlVal(mainpnl, MAINPNL_PD_EMI_WIDTH, &pd_emi_width);
		GetCtrlVal(mainpnl, MAINPNL_PD_EMI_DELTA, &pd_emi_delta);
		GetCtrlVal(mainpnl, MAINPNL_PD_EMI_THRESH_AUTO, &pd_emi_auto);
		if(pd_emi_auto)
			pd_emi_delta = 2;
		do
		{	
			detect_peak(raw_hart, ser_input_size, emi_peaks, num_emi_peaks, 64, absop_peaks, num_absop_peaks, 64, (double)pd_emi_delta, 1);
			dh = 0;
			for(i=0; i<(num_emi_peaks[0] - 1); i++)
			{
				if((emi_peaks_xpos[i+1] - emi_peaks_xpos[i]) < pd_emi_width)
				{
					dh++;
				}
			}
			
			if((dh > 0) && pd_emi_auto)
			{
				pd_emi_delta++;
				SetCtrlVal(mainpnl, MAINPNL_PD_EMI_DELTA, pd_emi_delta);	
			}
			
			if(!pd_emi_auto || (dh == 25))
				dh = 0;
		}while(dh > 0);
		hrm_7seg_out = (double)( ((double)num_emi_peaks[0]) * 60.0 / run_time ); 
		sprintf(tmp_str, "num_emi_peaks %d, hrm = %d ppm\n", num_emi_peaks[0], (int)(hrm_7seg_out));
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);
		SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_7, (unsigned char)hrm_7seg_out);
		
		GetCtrlVal(mainpnl, MAINPNL_PD_ABSOP_DELTA, &pd_absop_delta);
		GetCtrlVal(mainpnl, MAINPNL_PD_ABSOP_THRESH_AUTO, &pd_absop_auto);
		if(pd_absop_auto)
			pd_absop_delta = 2;
		do
		{	
			detect_peak(raw_hart, ser_input_size, emi_peaks, num_emi_peaks, 64, absop_peaks, num_absop_peaks, 64, (double)pd_absop_delta, 1);
			dh = 0;
			for(i=0; i<(num_absop_peaks[0] - 1); i++)
			{
				if((absop_peaks_xpos[i+1] - absop_peaks_xpos[i]) < pd_emi_width)
				{
					dh++;
				}
			}
			
			if((dh > 0) && pd_absop_auto)
			{
				pd_absop_delta++;
				SetCtrlVal(mainpnl, MAINPNL_PD_ABSOP_DELTA, pd_absop_delta);	
			}
			 /// IF dh cannot = 0 then increase allowed dh and substract it from  num_absop_peaks[0] 
			if(!pd_absop_auto || (dh == 25))
				dh = 0;
		}while(dh > 0);	  
		hrm_7seg_out = (double)( ((double)num_absop_peaks[0])  * 60.0 / run_time ); 
		sprintf(tmp_str, "num_absop_peaks %d, hrm = %d ppm\n", num_absop_peaks[0], (int)(hrm_7seg_out));
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);
		SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_8, (unsigned char)hrm_7seg_out);
		
		/// missing pulse correction:
		// run thru the absop_peaks_xpos[num_absop_peaks[0]]. find delta x for each. avg delta x. if 1 is double the rest add 1 to num_absop_peaks[0].
		int avg_x_diff = 0;
		int avg_x_rng = 0;
		for(int k=0; k<2; k++)
		{
			for(i=0; i<(num_emi_peaks[0] - 1); i++)
			{
				avg_x_diff += (emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]);	
			}
			avg_x_diff = avg_x_diff / num_emi_peaks[0];
			avg_x_rng = avg_x_diff + (avg_x_diff / 2);// + (avg_x_diff / 4);
		
			for(i=0; i<(num_emi_peaks[0] - 1); i++)
			{
				if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) > avg_x_rng) 
				{
					num_emi_peaks[0]++;
					sprintf(tmp_str, "Added a peak\n");
					SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);
					for(j=num_emi_peaks[0]; j<=(i+1); j--)
					{
						emi_peaks_xpos[j] = emi_peaks_xpos[j-1];
					}
					emi_peaks_xpos[i] = emi_peaks_xpos[i] + avg_x_diff;
					PlotPoint (mainpnl, MAINPNL_SIG4GRAPH, (double)emi_peaks_xpos[i], 0.0, VAL_BOLD_X, VAL_MAGENTA);	  	
				}
			}
		
			avg_x_diff = 0;
			for(i=0; i<(num_absop_peaks[0] - 1); i++)
			{
				avg_x_diff += (absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]);	
			}
			avg_x_diff = avg_x_diff / num_absop_peaks[0];
			avg_x_rng = avg_x_diff + (avg_x_diff / 2);// + (avg_x_diff / 4);
		
			for(i=0; i<(num_absop_peaks[0] - 1); i++)
			{
				if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) > avg_x_rng)
				{
					num_absop_peaks[0]++;
					sprintf(tmp_str, "Added a valley\n");
					SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);
					for(j=num_absop_peaks[0]; j<=(i+1); j--)
					{
						absop_peaks_xpos[j] = absop_peaks_xpos[j-1];
					}
					absop_peaks_xpos[i] = absop_peaks_xpos[i] + avg_x_diff;
					PlotPoint (mainpnl, MAINPNL_SIG4GRAPH, (double)absop_peaks_xpos[i], 0.0, VAL_BOLD_X, VAL_CYAN);
				}
			}
		}
		
		
		hrm_7seg_out = ((double)( ((double)num_absop_peaks[0]) * 60.0 / run_time )) + ((double)( ((double)num_emi_peaks[0]) * 60.0 / run_time )); 
		hrm_7seg_out = hrm_7seg_out / 2;
		SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_9, (unsigned char)hrm_7seg_out);
		
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
			PlotXY (mainpnl, MAINPNL_SIG4GRAPH, emi_peaks_xpos_dfp, emi_peaks_dfp, num_emi_peaks[0], VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_GREEN);
		if(num_absop_peaks[0] >= 1)
			PlotXY (mainpnl, MAINPNL_SIG4GRAPH, absop_peaks_xpos_dfp, absop_peaks_dfp, num_absop_peaks[0], VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_BLUE);
/// =========> PD 
		
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
		
	}   
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

/// PeakDetector (long_hart, j, pd_threshold_val, pd_threshold_width, DETECT_PEAKS, ANALYSIS_TRUE, ANALYSIS_TRUE, &pd_peak_cnt, &peak_position, &peak_ampl, &peak_deriv); 
/*void Peak_MM(double * y_data, int size, double pd_threshold_val, int pd_threshold_width)
{
	int i, pdcount = 0, pdindex = 0, pdhit=0;
	double x_max, y_max, peak_position[64], peak_ampl[64];  
	
	for(i = 0; i < size - pd_threshold_width; i += pd_threshold_width)
	{
		pdhit +=  polymax(y_data, i, size, &x_max, &y_max) ;
		if(pdhit)
		{
			pdcount++;
			peak_position[pdindex] = x_max;
			peak_ampl[pdindex] = y_max;
			pdindex++;
		}
	}
	//PlotXY (mainpnl, MAINPNL_SIG4GRAPH, peak_position, peak_ampl, pd_peak_cnt, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_RED);
	if( pdcount > 0)
		PlotXY (mainpnl, MAINPNL_SIG4GRAPH, peak_position, peak_ampl, pdcount, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_CIRCLE, VAL_SOLID, 1, VAL_RED);
}


// Fits *y from x_start to (x_start + window) with a parabola and returns x_max and y_max
int polymax(double * y_data, int x_start, int window, double *x_max, double *y_max)
{
    double sum[10],mat[3][4],temp=0,temp1=0,a1,a2,a3;
    int i,j;

    double x[window];
    for(i = 0; i < window; i++)
        x[i] = (double)i;

    double y[window];
    for(i = 0; i < window; i++)
        y[i] = (double)(y_data[x_start + i] - y_data[x_start]);

    for(i = 0; i < window; i++)
    {
        temp=temp+x[i];
        temp1=temp1+y[i];
    }
    sum[0]=temp;
    sum[1]=temp1;
    sum[2]=sum[3]=sum[4]=sum[5]=sum[6]=0;

    for(i = 0;i < window;i++)
    {
        sum[2]=sum[2]+(x[i]*x[i]);
        sum[3]=sum[3]+(x[i]*x[i]*x[i]);
        sum[4]=sum[4]+(x[i]*x[i]*x[i]*x[i]);
        sum[5]=sum[5]+(x[i]*y[i]);
        sum[6]=sum[6]+(x[i]*x[i]*y[i]);
    }
    mat[0][0]=window;
    mat[0][1]=mat[1][0]=sum[0];
    mat[0][2]=mat[1][2]=mat[2][0]=sum[2];
    mat[1][2]=mat[2][3]=sum[3];
    mat[2][2]=sum[4];
    mat[0][3]=sum[1];
    mat[1][3]=sum[5];
    mat[2][3]=sum[6];

    temp=mat[1][0]/mat[0][0];
    temp1=mat[2][0]/mat[0][0];
	//i = 0; 
    for(j = 0; j <= 3; j++)  
    {
        mat[1][j]=mat[1][j]-(mat[0][j]*temp);
        mat[2][j]=mat[2][j]-(mat[0][j]*temp1);
    }

//    temp=mat[2][4]/mat[1][5];
//    temp1=mat[0][6]/mat[1][7];
//    for(i = 1,j = 0; j < (3 + 1); j++)
//    {
//        mat[i+1][j]=mat[i+1][j]-(mat[i][j]*temp);
//        mat[i-1][j]=mat[i-1][j]-(mat[i][j]*temp1);
//    }

    temp=mat[0][2]/mat[2][2];
    temp1=mat[1][2]/mat[2][2];
	//i = 0;
    for(j = 0; j <= 3; j++)
    {
        mat[0][j]=mat[0][j]-(mat[2][j]*temp);
        mat[1][j]=mat[1][j]-(mat[2][j]*temp1);
    }

    a3 = mat[2][3]/mat[2][2];
    a2 = mat[1][3]/mat[1][1];
    a1 = mat[0][3]/mat[0][0];

    // zX^2 + yX + x

    if (a3 < 0)
    {
        temp = - a2 / (2*a3);
        *x_max = temp + x_start;
        *y_max = (a3*temp*temp + a2*temp + a1) + y_data[x_start];
        return 1;
    }
    else
       return 0;
}  */

/// ==================================================================================///

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

//void print_help(void)
//{
//    fprintf(stderr,
//            "Usage: peakdetect [OPTIONS]\n"
//            "Peak detection in a wave\n"
//            "\n"
//            "Options:\n"
//            "-i inputfile \t\tInput file.\n"
//            "             \t\tThe input file should be a csv format file, "
//            "whose first\n"
//            "             \t\tcolumn is X and second column is Y.\n"
//            "-o outfile   \t\tOutput file.\n"
//            "             \t\tEmission peaks will be output first, "
//            "followed by\n"
//            "             \t\tabsorption peaks with an empty line seperated."
//            "\n"
//            "-d deltavalue\t\tDelta, a parameter used to determine peaks.\n"
//            "-m mode      \t\tDetecting mode, "
//            "could be either \"a\" (detect absorption peak\n"
//            "             \t\tfirst) or \"e\" (detect emission peak first).\n"
//            "             \t\tDefault value is \"a\".\n"
//            "--version    \t\tDisplay version information.\n"
//            "--help       \t\tShow this help information.\n"
//            "\n"
//            "e.g.\n"
//            "peakdetect -i input.csv -o output.csv -d 1e-7 -m a\n"
//            "peakdetect <input.csv -d 0.1 -m e | tee out.csv\n");
//    exit(0);
//}
//
//void print_version(void)
//{
//    fprintf(stderr,
//            "peakdetect version 0.1.3\n"
//            "Copyright (C) 2011 Hong Xu <xuphys@gmail.com>\n"
//            "Originally inspired by Eli Billauer\'s peakdet for MATLAB:\n"
//            "http://billauer.co.il/peakdet.html\n"
//            "\n"
//            "See the README file for license information.\n");
//    exit(0);
//}
//
int detect_peak(
        const double*   data, /* the data */ 
        int             data_count, /* row count of data */ 
        int*            emi_peaks, /* emission peaks will be put here */ 
        int*            num_emi_peaks, /* number of emission peaks found */
        int             max_emi_peaks, /* maximum number of emission peaks */ 
        int*            absop_peaks, /* absorption peaks will be put here */ 
        int*            num_absop_peaks, /* number of absorption peaks found */
        int             max_absop_peaks, /* maximum number of absorption peaks
                                            */ 
        double          delta, /* delta used for distinguishing peaks */
        int             emi_first /* should we search emission peak first of
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

    *num_emi_peaks = 0;
    *num_absop_peaks = 0;

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
            if(*num_emi_peaks >= max_emi_peaks) /* not enough spaces */
                return 1;

            emi_peaks[*num_emi_peaks] = mx; //mx_pos
			emi_peaks_xpos[*num_emi_peaks] = mx_pos;
			++ (*num_emi_peaks);
			
            is_detecting_emi = 0;

            i = mx_pos - 1;

            mn = data[mx_pos];
            mn_pos = mx_pos;
        }
        else if((!is_detecting_emi) &&
                data[i] > mn + delta)
        {
            if(*num_absop_peaks >= max_absop_peaks)
                return 2;
			
			absop_peaks_xpos[*num_absop_peaks] = mn_pos;
            absop_peaks[*num_absop_peaks] = mn;
            ++ (*num_absop_peaks);

            is_detecting_emi = 1;
            
            i = mn_pos - 1;

            mx = data[mn_pos];
            mx_pos = mn_pos;
        }
    }

    return 0;
}
//
//int main(int argc, const char *argv[])
//{
//#define INITIAL_ROW_COUNT       1500
//#define ROW_COUNT_INCREASEMENT  3000
//    double*     data[2];
//    double      row[2];
//#define MAX_PEAK    200
//    int         emi_peaks[MAX_PEAK];
//    int         absorp_peaks[MAX_PEAK];
//    int         emi_count = 0;
//    int         absorp_count = 0;
//#define LINE_BUFFER_SIZE    120
//    char        line[LINE_BUFFER_SIZE];
//    FILE*       out =   stdout;
//    FILE*       in  =   stdin;
//    int         i;
//    double      delta = 1e-6;
//    int         emission_first = 0;
//    int         idummy;
//
//    /*
//     * argument parsing
//     */
//    {
//        int flag_delta = 0;
//        int flag_in = 0;
//        int flag_out = 0;
//        int flag_mode = 0;
//        for(i = 1; i < argc; ++i)
//        {
//            if(flag_delta)
//            {
//                delta = atof(argv[i]);
//                flag_delta = 0;
//            }
//            else if(flag_in)
//            {
//                in = fopen(argv[i], "r");
//                if(!in)
//                {
//                    fprintf(stderr, "Failed to open file \"");
//                    fprintf(stderr, argv[i]);
//                    fprintf(stderr, "\".\n");
//                    exit(2);
//                }
//                flag_in = 0;
//            }
//            else if(flag_out)
//            {
//                out = fopen(argv[i], "w");
//                if(!out)
//                {
//                    fprintf(stderr, "Failed to open file \"");
//                    fprintf(stderr, argv[i]);
//                    fprintf(stderr, "\".\n");
//                    exit(2);
//                }
//                flag_out = 0;
//
//            }
//            else if(flag_mode)
//            {
//                if(!strcmp(argv[i], "a"))
//                    emission_first = 0;
//                else if(!strcmp(argv[i], "e"))
//                    emission_first = 1;
//                else
//                {
//                    fprintf(stderr,
//                            "Argument parsing error: Unknown mode \"");
//                    fprintf(stderr, argv[i]);
//                    fprintf(stderr, "\"\n");
//                    exit(4);
//                }
//
//                flag_mode = 0;
//            }
//            else if(!strcmp(argv[i], "-d"))
//                flag_delta = 1;
//            else if(!strcmp(argv[i], "-i"))
//                flag_in = 1;
//            else if(!strcmp(argv[i], "-o"))
//                flag_out = 1;
//            else if(!strcmp(argv[i], "-m"))
//                flag_mode = 1;
//            else if(!strcmp(argv[i], "--help"))
//                print_help();
//            else if(!strcmp(argv[i], "--version"))
//                print_version();
//            else
//            {
//                fprintf(stderr, "Unknown option \"");
//                fprintf(stderr, argv[i]);
//                fprintf(stderr, "\".\n");
//                exit(3);
//            }
//
//        }
//    }
//
//    data[0] = (double*) malloc(sizeof(double) * INITIAL_ROW_COUNT);
//    data[1] = (double*) malloc(sizeof(double) * INITIAL_ROW_COUNT);
//
//    /* read data */
//    i = 0;
//    while(fgets(line, LINE_BUFFER_SIZE, in))
//    {
//        /* when the buffer is not large enough, increase the buffer size */
//        idummy = i - INITIAL_ROW_COUNT;
//        if(idummy >= 0 && idummy % ROW_COUNT_INCREASEMENT == 0)
//        {
//            double*     tmp;
//            int         j;
//
//            for(j = 0; j < 2; ++j)
//            {
//                tmp = (double*) malloc(
//                        sizeof(double) * (i + ROW_COUNT_INCREASEMENT));
//                memcpy(tmp, data[j], i * sizeof(double));
//                free(data[j]);
//                data[j] = tmp;
//            }
//        }
//
//        sscanf(line, "%lf,%lf", row, row + 1);
//        data[0][i] = row[0];
//        data[1][i] = row[1];
//        ++ i;
//    }
//
//    if(detect_peak(data[1], i,
//                emi_peaks, &emi_count, MAX_PEAK,
//                absorp_peaks, &absorp_count, MAX_PEAK,
//                delta, emission_first))
//    {
//        fprintf(stderr, "There are too many peaks.\n");
//        exit(1);
//    }
//
//    for(i = 0; i < emi_count; ++i)
//        fprintf(out, "%e,%e\n", data[0][emi_peaks[i]], data[1][emi_peaks[i]]);
//    puts("");
//    for(i = 0; i < absorp_count; ++i)
//        fprintf(out, "%e,%e\n", data[0][absorp_peaks[i]],
//                data[1][absorp_peaks[i]]);
//
//    return 0;
//}
