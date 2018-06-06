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
#include <string.h>
#include <toolbox.h>
#include "Activity Discriminator vars.h" 
#include "Activity Discriminator.h"


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
	char	tmp_str[32];
	double 	hrm_7seg_out;
	int 	i, j;
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
		}
		
		GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time);
		if(pd_peak_cnt > 2)
			hrm_7seg_out = (double)( ((double)pd_peak_cnt + 0.0) / run_time ) * 60.0;
		else
			hrm_7seg_out = 0;
		SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_2, hrm_7seg_out);
		
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
		
		for(i=look_back_starting_point; i<(look_back_starting_point + ser_input_size); i++)
		{
			 raw_hart[i] =  intgtr_gyr_p[i - look_back_starting_point] + 100;
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
		
		total_run_time = j / 24;
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
