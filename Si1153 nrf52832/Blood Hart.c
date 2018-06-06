/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Blood Hart.c                                  			         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Include files                                                             */
/*---------------------------------------------------------------------------*/
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



/* Integrator Pieces */

/*
	int		j, raw_filt_sel, aclsftfilt_val, gyrsftfilt_val, aclavgfilt_val, gyravgfilt_val;
	int 	threshold_val, auto_threshold;
    double 	acl_x_avgfilt_accm = 0.0, acl_y_avgfilt_accm = 0.0, acl_z_avgfilt_accm = 0.0;
	long 	acl_x_shift_filt_ravg = 0, acl_y_shift_filt_ravg = 0, acl_z_shift_filt_ravg = 0;
	double 	gyro_p_avgfilt_accm = 0.0, gyro_y_avgfilt_accm = 0.0, gyro_r_avgfilt_accm = 0.0;
	long 	gyro_p_shift_filt_ravg = 0, gyro_y_shift_filt_ravg = 0, gyro_r_shift_filt_ravg = 0;
	double 	running_avg = 0;   
	
	GetCtrlVal(mainpnl, MAINPNL_ACLSFTFV, &aclsftfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_GYRSFTFV, &gyrsftfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_ACLAVGFV, &aclavgfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_GYRAVGFV, &gyravgfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_PDTHRESH, &threshold_val);
	GetCtrlVal(mainpnl, MAINPNL_HBFAUTOTHRESH, &auto_threshold);
*/



/* Basic Integrator or both
	if((raw_intgtr_sel == 1) || (raw_intgtr_sel == 4) && !ss_integrator_sel) 
	{    
		
		Integrate(intgtr_acl_x,(ser_input_size + 64) , 1.0, intgtr_acl_x[0], 0.0, intgtr_acl_x);
		Integrate(intgtr_acl_y,(ser_input_size + 64) , 1.0, intgtr_acl_y[0], 0.0, intgtr_acl_y);
		Integrate(intgtr_acl_z,(ser_input_size + 64) , 1.0, intgtr_acl_z[0], 0.0, intgtr_acl_z);
		Integrate(intgtr_gyr_p,(ser_input_size + 64) , 1.0, intgtr_gyr_p[0], 0.0, intgtr_gyr_p);
		Integrate(intgtr_gyr_y,(ser_input_size + 64) , 1.0, intgtr_gyr_y[0], 0.0, intgtr_gyr_y);  
		Integrate(intgtr_gyr_r,(ser_input_size + 64) , 1.0, intgtr_gyr_r[0], 0.0, intgtr_gyr_r); 
	/*
		Integrate(intgtr_acl_x,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_x);
		Integrate(intgtr_acl_y,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_y);
		Integrate(intgtr_acl_z,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_z);
		Integrate(intgtr_gyr_p,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_p);
		Integrate(intgtr_gyr_y,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_y);  
		Integrate(intgtr_gyr_r,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_r); 
	*/ 
/*	}
*/

			/* 
			running_avg_acc_x += intgtr_acl_x[i];
			running_avg_acc_y += intgtr_acl_y[i];
			running_avg_acc_z += intgtr_acl_z[i];
			running_avg_gyr_p += intgtr_gyr_p[i];
			running_avg_gyr_y += intgtr_gyr_y[i];
			running_avg_gyr_r += intgtr_gyr_r[i];
			*/


/*
		for(i=0; i<(ser_input_size + 64); i++)
		{
			intgtr_acl_x[i] = filt_acl_x[i];
			intgtr_acl_y[i] = filt_acl_y[i];
			intgtr_acl_z[i] = filt_acl_z[i];
			intgtr_gyr_p[i] = filt_gyr_p[i];
			intgtr_gyr_y[i] = filt_gyr_y[i];
			intgtr_gyr_r[i] = filt_gyr_r[i];
		}
*/


