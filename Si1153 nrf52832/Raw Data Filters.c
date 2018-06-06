/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Raw Data Filters.c                                  			 */
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



int CVICALLBACK Apply_Raw_Data_Filtering_CB (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			//Apply_Raw_Data_Filtering();
			Apply_Peak_Detector(); 
			break;
	}
	return 0;
}

void Apply_Raw_Data_Filtering(void)
{
	int 	i, j, raw_filt_sel, aclsftfilt_val, gyrsftfilt_val, aclavgfilt_val, gyravgfilt_val;
	int 	auto_threshold;
    double 	acl_x_avgfilt_accm = 0.0, acl_y_avgfilt_accm = 0.0, acl_z_avgfilt_accm = 0.0;
	long 	acl_x_shift_filt_ravg = 0, acl_y_shift_filt_ravg = 0, acl_z_shift_filt_ravg = 0;
	double 	gyro_p_avgfilt_accm = 0.0, gyro_y_avgfilt_accm = 0.0, gyro_r_avgfilt_accm = 0.0;
	long 	gyro_p_shift_filt_ravg = 0, gyro_y_shift_filt_ravg = 0, gyro_r_shift_filt_ravg = 0;
	double 	acc_threshold_val, running_avg = 0;   
	
	
	GetCtrlVal(mainpnl, MAINPNL_RAW_FILTER, &raw_filt_sel);
	GetCtrlVal(mainpnl, MAINPNL_ACLSFTFV, &aclsftfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_GYRSFTFV, &gyrsftfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_ACLAVGFV, &aclavgfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_GYRAVGFV, &gyravgfilt_val);
	GetCtrlVal(mainpnl, MAINPNL_ACC_THRESHHOLD, &acc_threshold_val);
	GetCtrlVal(mainpnl, MAINPNL_HBFAUTOTHRESH, &auto_threshold);
	
	
	
	// apply threshold
	running_avg = 0;
	for(i=0; i<(ser_input_size + 64); i++)
	{
		/*
		if(fabs(raw_acl_x[i]) > acc_threshold_val)
			filt_acl_x[i] = raw_acl_x[i];
		else
			filt_acl_x[i] = 0.0;
		if(fabs(raw_acl_y[i]) > acc_threshold_val)
			filt_acl_y[i] = raw_acl_y[i];
		else
			filt_acl_y[i] = 0.0;
		if(fabs(raw_acl_z[i]) > acc_threshold_val)
			filt_acl_z[i] = raw_acl_z[i];
		else
			filt_acl_z[i] = 0.0;
		*/
		
		filt_acl_x[i] = raw_acl_x[i];
		filt_acl_y[i] = raw_acl_y[i];
		filt_acl_z[i] = raw_acl_z[i];
		
		if(0)
		{
			filt_gyr_p[i] = sqrt(fabs(raw_gyr_p[i]));
			if(raw_gyr_p[i] < 0)
				filt_gyr_p[i] = -filt_gyr_p[i];	
		}
		else
			filt_gyr_p[i] = raw_gyr_p[i];
		
		if(0)
		{
			filt_gyr_y[i] = sqrt(fabs(raw_gyr_y[i]));
			if(raw_gyr_y[i] < 0)
				filt_gyr_y[i] = -filt_gyr_y[i];	
		}
		else
			filt_gyr_y[i] = raw_gyr_y[i];
		
		filt_gyr_r[i] = raw_gyr_r[i];
	///	running_avg +=  wave1[i]; // for HB
	}
	///	running_avg =  running_avg / (i+1);

	
	// bishift or both
	/// shift_filt_ravg = (((shift_filt_ravg) + ((long)wave1[i] * hbf_val) + 4) >> 3);
	/// running_avg_x = (((running_avg_x) + ((long)gyro_x * 3) + 2) >> 2); (((running_avg_y * 7) + (gyro_y + 4)) >> 3);
	if((raw_filt_sel == 1) || (raw_filt_sel == 4)) 
	{
		
		for(i=0; i<(ser_input_size + 64); i++)
		{
			acl_x_shift_filt_ravg = (((acl_x_shift_filt_ravg) + (long)(filt_acl_x[i] * (double)aclsftfilt_val * 1000) + (aclsftfilt_val - 1)) >> (aclsftfilt_val - 1));
			filt_acl_x[i] = (double)acl_x_shift_filt_ravg / 1000.0;
			
			acl_y_shift_filt_ravg = (((acl_y_shift_filt_ravg) + (long)(filt_acl_y[i] * (double)aclsftfilt_val * 1000) + (aclsftfilt_val - 1)) >> (aclsftfilt_val - 1));
			filt_acl_y[i] = (double)acl_y_shift_filt_ravg / 1000.0;
			
			acl_z_shift_filt_ravg = (((acl_z_shift_filt_ravg) + (long)(filt_acl_z[i] * (double)aclsftfilt_val * 1000) + (aclsftfilt_val - 1)) >> (aclsftfilt_val - 1));
			filt_acl_z[i] = (double)acl_z_shift_filt_ravg / 1000.0;
			
			gyro_p_shift_filt_ravg = (((gyro_p_shift_filt_ravg * 100) + ((long)raw_gyr_p[i] * gyrsftfilt_val) + (gyrsftfilt_val - 1)) >> (gyrsftfilt_val - 1));
			filt_gyr_p[i] = (double)gyro_p_shift_filt_ravg; //////////////////////// / 100.0;
			gyro_p_shift_filt_ravg = gyro_p_shift_filt_ravg / 100;
			
			gyro_y_shift_filt_ravg = (((gyro_y_shift_filt_ravg * 100) + ((long)raw_gyr_y[i] * gyrsftfilt_val) + (gyrsftfilt_val - 1)) >> (gyrsftfilt_val - 1));
			filt_gyr_y[i] = (double)gyro_y_shift_filt_ravg; //////////////////////// / 100.0;
			gyro_y_shift_filt_ravg = gyro_y_shift_filt_ravg / 100;
			
			gyro_r_shift_filt_ravg = (((gyro_r_shift_filt_ravg * 100) + ((long)raw_gyr_r[i] * gyrsftfilt_val) + (gyrsftfilt_val - 1)) >> (gyrsftfilt_val - 1));
			filt_gyr_r[i] = (double)gyro_r_shift_filt_ravg / 100.0;
			gyro_r_shift_filt_ravg = gyro_r_shift_filt_ravg / 100;
		}
	}
	
	// average or both
	if((raw_filt_sel == 2) || (raw_filt_sel == 4))  
	{
		
		for(i=0; i<(ser_input_size + 64); i++)
		{
			for(j=0; j<aclavgfilt_val; j++)
			{
				acl_x_avgfilt_accm = acl_x_avgfilt_accm + filt_acl_x[i+j];
				acl_y_avgfilt_accm = acl_y_avgfilt_accm + filt_acl_y[i+j]; 
				acl_z_avgfilt_accm = acl_z_avgfilt_accm + filt_acl_z[i+j]; 
			}
			filt_acl_x[i] = acl_x_avgfilt_accm / aclavgfilt_val;
			filt_acl_y[i] = acl_y_avgfilt_accm / aclavgfilt_val;
			filt_acl_z[i] = acl_z_avgfilt_accm / aclavgfilt_val;
			acl_x_avgfilt_accm = acl_y_avgfilt_accm = acl_z_avgfilt_accm = 0;
			
			for(j=0; j<gyravgfilt_val; j++)
			{															  
				gyro_p_avgfilt_accm = gyro_p_avgfilt_accm + filt_gyr_p[i+j]; 
				gyro_y_avgfilt_accm = gyro_y_avgfilt_accm + filt_gyr_y[i+j]; 
				gyro_r_avgfilt_accm = gyro_r_avgfilt_accm + filt_gyr_r[i+j]; 
			}
			filt_gyr_p[i] = gyro_p_avgfilt_accm / gyravgfilt_val; 
			filt_gyr_y[i] = gyro_y_avgfilt_accm / gyravgfilt_val;
			filt_gyr_r[i] = gyro_r_avgfilt_accm / gyravgfilt_val;
			gyro_p_avgfilt_accm = gyro_y_avgfilt_accm = gyro_r_avgfilt_accm = 0;
		}   	
	}
	
//	running_avg = 0;
//	for(i=0; i<ser_input_size; i++)
//	{
//		running_avg +=  wave1[i];
//	}
//	running_avg =  running_avg / (i + 1)/2;
	
//	for(i=0; i<ser_input_size; i++)
//	{
//		if(sqra_on)
//		{
//			if(filt_gyr_p[i] < 0) 
//			{
//				filt_gyr_p[i] = -filt_gyr_p[i];
//				filt_gyr_p[i] = sqrt(filt_gyr_p[i]);
//				filt_gyr_p[i] = -filt_gyr_p[i];	
//			}
//			else
//			{
//				filt_gyr_p[i] = sqrt(filt_gyr_p[i]);	
//			}
//		}
//	
//		if(sqrb_on)
//		{
//			if(filt_gyr_y[i] < 0) 
//			{
//				filt_gyr_y[i] = -filt_gyr_y[i];
//				filt_gyr_y[i] = sqrt(filt_gyr_y[i]);
//				filt_gyr_y[i] = -filt_gyr_y[i];	
//			}
//			else
//			{
//				filt_gyr_y[i] = sqrt(filt_gyr_y[i]);	
//			}
//		}
//	}
	
	Apply_Position_Integrator();
}



int CVICALLBACK Sqr_A (int panel, int control, int event,
					   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_SQRA, &sqra_on);
			break;
	}
	return 0;
}

int CVICALLBACK Sqr_B (int panel, int control, int event,
					   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_SQRB, &sqrb_on);
			break;
	}
	return 0;
}
