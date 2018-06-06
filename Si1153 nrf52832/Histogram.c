/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Histogram.c                                  			         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Include files                                                             */
/*---------------------------------------------------------------------------*/
#include <ansi_c.h>
#include <math.h>
#include <analysis.h>
#include <cvirte.h>
#include <userint.h>
#include <rs232.h>
#include <utility.h>
#include <formatio.h>
#include <string.h>
#include <toolbox.h>
#include "Activity Discriminator vars.h" 
#include "Activity Discriminator.h"


void Apply_Histogram_Mag(void)
{
	double 		axis_array[100], max_val, min_val;
	ssize_t  	hist_array[100];
	int			intervals = 10, max_pos, min_pos;
	
	if( pd_peak_cnt == 0) 
	{
		DeleteGraphPlot (mainpnl, MAINPNL_MAG_HIST_GRAPH, -1, VAL_IMMEDIATE_DRAW); 
		return;
	}
	
	MaxMin1D (peak_ampl, pd_peak_cnt, &max_val, &max_pos, &min_val, &min_pos);
	Histogram(peak_ampl, pd_peak_cnt, 0.0, max_val, hist_array, axis_array, intervals);
	
	PlotXY (mainpnl, MAINPNL_MAG_HIST_GRAPH, axis_array,  hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_VERTICAL_BAR, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
}

void Apply_Histogram_Acc(void)
{
	double 		axis_array[100], max_val, min_val, last_max_val, last_min_val;
	ssize_t  	hist_array[100];
	int			intervals = 10, max_pos, min_pos;
	
	//DeleteGraphPlot (mainpnl, MAINPNL_ACC_HIST_GRAPH, -1, VAL_IMMEDIATE_DRAW);
	if( pd_peak_cnt == 0) return;
	
	MaxMin1D (filt_acl_x, (ser_input_size + 64), &last_max_val, &max_pos, &last_min_val, &min_pos);
	MaxMin1D (filt_acl_y, (ser_input_size + 64), &max_val, &max_pos, &min_val, &min_pos);
	if( min_val < last_min_val ) last_min_val = min_val;
	if( max_val > last_max_val ) last_max_val = max_val;
	MaxMin1D (filt_acl_z, (ser_input_size + 64), &max_val, &max_pos, &min_val, &min_pos);
	if( min_val < last_min_val ) last_min_val = min_val;
	if( max_val > last_max_val ) last_max_val = max_val;
	if(last_min_val == last_max_val) last_min_val = last_max_val - 0.1;
	SetAxisScalingMode (mainpnl, MAINPNL_ACC_HIST_GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, last_min_val, last_max_val);
	SetAxisScalingMode (mainpnl, MAINPNL_ACC_HIST_GRAPH, VAL_LEFT_YAXIS, VAL_AUTOSCALE, 0, 100); 
	
	//Histogram(filt_acl_x, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	//PlotXY (mainpnl, MAINPNL_ACC_HIST_GRAPH, axis_array,  hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_SCATTER, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
	//Histogram(filt_acl_y, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	//PlotXY (mainpnl, MAINPNL_ACC_HIST_GRAPH, axis_array,  hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_SCATTER, VAL_EMPTY_CIRCLE, VAL_SOLID, 1, VAL_GREEN);
	Histogram(filt_acl_z, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	PlotXY (mainpnl, MAINPNL_ACC_HIST_GRAPH, axis_array,  hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_SCATTER, VAL_EMPTY_DIAMOND, VAL_SOLID, 1, VAL_WHITE);
	//Histogram(acc_mag, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	//PlotXY (mainpnl, MAINPNL_ACC_HIST_GRAPH, axis_array, hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_SCATTER, VAL_ASTERISK, VAL_SOLID, 1, VAL_MAGENTA);
}

void Apply_Histogram_Gyro(void)
{
	double 		axis_array[100], max_val, min_val, last_max_val, last_min_val;
	ssize_t  	hist_array[100];
	int			intervals = 10, max_pos, min_pos;
	
	DeleteGraphPlot (mainpnl, MAINPNL_HRM_HIST_GRAPH, -1, VAL_IMMEDIATE_DRAW);
	if( pd_peak_cnt == 0) return;
	
	MaxMin1D (filt_gyr_p, (ser_input_size + 64), &last_max_val, &max_pos, &last_min_val, &min_pos);
	MaxMin1D (filt_gyr_y, (ser_input_size + 64), &max_val, &max_pos, &min_val, &min_pos);
	if( min_val < last_min_val ) last_min_val = min_val;
	if( max_val > last_max_val ) last_max_val = max_val;
	MaxMin1D (filt_gyr_r, (ser_input_size + 64), &max_val, &max_pos, &min_val, &min_pos);
	if( min_val < last_min_val ) last_min_val = min_val;
	if( max_val > last_max_val ) last_max_val = max_val;
	if(last_min_val == last_max_val) last_min_val = last_max_val - 0.1;
	//SetAxisScalingMode (mainpnl, MAINPNL_HRM_HIST_GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, last_min_val, last_max_val);
	//SetAxisScalingMode (mainpnl, MAINPNL_HRM_HIST_GRAPH, VAL_LEFT_YAXIS, VAL_AUTOSCALE, 0, 100); 
	
	Histogram(filt_gyr_p, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	PlotXY (mainpnl, MAINPNL_HRM_HIST_GRAPH, axis_array,  hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_SCATTER, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
	Histogram(filt_gyr_y, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	PlotXY (mainpnl, MAINPNL_HRM_HIST_GRAPH, axis_array,  hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_SCATTER, VAL_EMPTY_CIRCLE, VAL_SOLID, 1, VAL_GREEN);
	//Histogram(filt_gyr_r, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	//PlotXY (mainpnl, MAINPNL_HRM_HIST_GRAPH, axis_array,  hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_CONNECTED_POINTS, VAL_EMPTY_DIAMOND, VAL_SOLID, 1, VAL_BLUE);
	//Histogram(gyr_mag, ser_input_size, last_min_val, last_max_val, hist_array, axis_array, intervals);
	//PlotXY (mainpnl, MAINPNL_HRM_HIST_GRAPH, axis_array, hist_array, intervals, VAL_DOUBLE, VAL_SSIZE_T, VAL_SCATTER, VAL_ASTERISK, VAL_SOLID, 1, VAL_MAGENTA);
}
