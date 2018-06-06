/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Sig Graph.c                                  					 */
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


int CVICALLBACK UpdateSG1_CB (int panel, int control, int event,
								  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			UpdateSG1();
			Update_PD_Plots();
			break;
	}
	return 0;
}

void UpdateSG1(void)
{
	int	s1g_sel, mag_source_sel;
	return;
	GetCtrlVal(mainpnl, MAINPNL_S1G_SEL, &s1g_sel);
	GetCtrlVal(mainpnl, MAINPNL_S1GX, &s1gx);
	GetCtrlVal(mainpnl, MAINPNL_S1GY, &s1gy);
	GetCtrlVal(mainpnl, MAINPNL_S1GZ, &s1gz);
	GetCtrlVal(mainpnl, MAINPNL_S1GM, &s1gm);
	
	DeleteGraphPlot (mainpnl, MAINPNL_SIG1GRAPH, -1, VAL_IMMEDIATE_DRAW);  
	if(ser_input_size > 0)
	{
		if(s1g_sel == 1) // raw acel
		{
			if(s1gx)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s1gy)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s1gz)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s1g_sel == 2) // raw gyro
		{
			if(s1gx)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s1gy)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s1gz)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s1g_sel == 3) // filt acel
		{
			if(s1gx)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, filt_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s1gy)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, filt_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s1gz)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, filt_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s1g_sel == 4) // filt gyro
		{
			if(s1gx)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, filt_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s1gy)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, filt_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s1gz)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, filt_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s1g_sel == 5) // Sum v/t Acel
		{
			if(s1gx)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, intgtr_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s1gy)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, intgtr_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s1gz)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, intgtr_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s1g_sel == 6) // Sum v/t Gyro
		{
			if(s1gx)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, intgtr_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s1gy)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, intgtr_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s1gz)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, intgtr_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s1gm)
		{
			if(s1g_sel < 5) mag_source_sel = 0; else mag_source_sel = 1;
			Get_Manitude(mag_source_sel);
			if((s1g_sel & 0x01) == 1)
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, acc_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
			else
				PlotY (mainpnl, MAINPNL_SIG1GRAPH, gyr_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
		}
		
		if(s1g_sel == 7) // raw hart
		{
			SetCtrlAttribute (mainpnl, MAINPNL_SIG1GRAPH, ATTR_YLOOSE_FIT_AUTOSCALING, 1);
			SetCtrlAttribute (mainpnl, MAINPNL_SIG1GRAPH, ATTR_YLOOSE_FIT_AUTOSCALING_UNIT, 2);
			PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_hart, ser_input_size, VAL_DOUBLE, VAL_FAT_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
		}
	}
}


int CVICALLBACK UpdateSG2_CB (int panel, int control, int event,
								  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			UpdateSG2();
			Update_PD_Plots();
			break;
	}
	return 0;
}

void UpdateSG2(void)
{
	int	s2g_sel, mag_source_sel;
	
	GetCtrlVal(mainpnl, MAINPNL_S2G_SEL, &s2g_sel);   
	GetCtrlVal(mainpnl, MAINPNL_S2GX, &s2gx);
	GetCtrlVal(mainpnl, MAINPNL_S2GY, &s2gy);
	GetCtrlVal(mainpnl, MAINPNL_S2GZ, &s2gz);
	GetCtrlVal(mainpnl, MAINPNL_S2GM, &s2gm);
	
	DeleteGraphPlot (mainpnl, MAINPNL_SIG2GRAPH, -1, VAL_IMMEDIATE_DRAW);  
	if(ser_input_size > 0)
	{
		///printf("update sg2 %d %f rhcbi %d\n", ser_input_size, raw_gyr_p[100], raw_hart_cir_buff_idx);
		if(s2g_sel == 1) // raw acel
		{
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s2gz)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s2g_sel == 2) // raw gyro
		{
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s2gz)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s2g_sel == 3) // filt acel
		{
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, filt_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, filt_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s2gz)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, filt_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s2g_sel == 4) // filt gyro
		{
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, filt_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, filt_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s2gz)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, filt_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s2g_sel == 5) // Sum v/t Acel
		{
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, intgtr_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, intgtr_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s2gz)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, intgtr_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s2g_sel == 6) // Sum v/t Gyro
		{
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, intgtr_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, intgtr_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s2gz)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, intgtr_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s2gm)
		{
			if(s2g_sel < 5) mag_source_sel = 0; else mag_source_sel = 1;
			Get_Manitude(mag_source_sel);
			if((s2g_sel & 0x01) == 1)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, acc_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
			else
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, gyr_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
		}
	}
		
}


int CVICALLBACK UpdateSG3_CB (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			UpdateSG3();
			Update_PD_Plots();
			break;
	}
	return 0;
}

void UpdateSG3(void)
{
	int	s3g_sel, mag_source_sel;
	
	GetCtrlVal(mainpnl, MAINPNL_S3G_SEL, &s3g_sel);   
	GetCtrlVal(mainpnl, MAINPNL_S3GX, &s3gx);
	GetCtrlVal(mainpnl, MAINPNL_S3GY, &s3gy);
	GetCtrlVal(mainpnl, MAINPNL_S3GZ, &s3gz);
	GetCtrlVal(mainpnl, MAINPNL_S3GM, &s3gm);
	
	DeleteGraphPlot (mainpnl, MAINPNL_SIG3GRAPH, -1, VAL_IMMEDIATE_DRAW);  
	if(ser_input_size > 0)
	{
		if(s3g_sel == 1) // raw acel
		{
			if(s3gx)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, raw_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s3gy)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, raw_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s3gz)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, raw_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s3g_sel == 2) // raw gyro
		{
			if(s3gx)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, raw_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s3gy)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, raw_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s3gz)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, raw_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s3g_sel == 3) // filt acel
		{
			if(s3gx)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, filt_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s3gy)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, filt_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s3gz)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, filt_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s3g_sel == 4) // filt gyro
		{
			if(s3gx)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, filt_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s3gy)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, filt_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s3gz)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, filt_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s3g_sel == 5) // Sum v/t Acel
		{
			if(s3gx)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, intgtr_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s3gy)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, intgtr_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s3gz)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, intgtr_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s3g_sel == 6) // Sum v/t Gyro
		{
			if(s3gx)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, intgtr_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s3gy)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, intgtr_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s3gz)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, intgtr_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s3gm)
		{
			if(s3g_sel < 5) mag_source_sel = 0; else mag_source_sel = 1;
			Get_Manitude(mag_source_sel);
			if((s3g_sel & 0x01) == 1)
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, acc_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
			else
				PlotY (mainpnl, MAINPNL_SIG3GRAPH, gyr_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
		}
	}
		
}


int CVICALLBACK UpdateSG4_CB (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			UpdateSG4();
			Update_PD_Plots();
			break;
	}
	return 0;
}

void UpdateSG4(void)
{
	int	s4g_sel, mag_source_sel;
	
	GetCtrlVal(mainpnl, MAINPNL_S4G_SEL, &s4g_sel);   
	GetCtrlVal(mainpnl, MAINPNL_S4GX, &s4gx);
	GetCtrlVal(mainpnl, MAINPNL_S4GY, &s4gy);
	GetCtrlVal(mainpnl, MAINPNL_S4GZ, &s4gz);
	GetCtrlVal(mainpnl, MAINPNL_S4GM, &s4gm);
	//SetCtrlAttribute (mainpnl, MAINPNL_SIG4GRAPH, ATTR_ACTIVE_XAXIS, VAL_TOP_XAXIS);
	SetAxisScalingMode (mainpnl, MAINPNL_SIG4GRAPH, VAL_TOP_XAXIS, VAL_MANUAL, 0.0, run_time);
	//SetCtrlAttribute (mainpnl, MAINPNL_SIG4GRAPH, ATTR_ACTIVE_XAXIS, VAL_BOTTOM_XAXIS);
	
	/// DeleteGraphPlot (mainpnl, MAINPNL_SIG4GRAPH, -1, VAL_IMMEDIATE_DRAW);  
	if(ser_input_size > 0)
	{
		if(s4g_sel == 1) // raw acel
		{
			if(s4gx)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, raw_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s4gy)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, raw_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s4gz)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, raw_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s4g_sel == 2) // raw gyro
		{
			if(s4gx)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, raw_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s4gy)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, raw_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s4gz)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, raw_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s4g_sel == 3) // filt acel
		{
			if(s4gx)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, filt_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s4gy)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, filt_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s4gz)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, filt_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s4g_sel == 4) // filt gyro
		{
			if(s4gx)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, filt_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s4gy)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, filt_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s4gz)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, filt_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s4g_sel == 5) // Sum v/t Acel
		{
			if(s4gx)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, intgtr_acl_x, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s4gy)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, intgtr_acl_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s4gz)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, intgtr_acl_z, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s4g_sel == 6) // Sum v/t Gyro
		{
			//if(s4gx)
			//	PlotY (mainpnl, MAINPNL_SIG4GRAPH, intgtr_gyr_p, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
			if(s4gy)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, intgtr_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			if(s4gz)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, intgtr_gyr_r, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
		}
		
		if(s4gm)
		{
			if(s4g_sel < 5) mag_source_sel = 0; else mag_source_sel = 1;
			Get_Manitude(mag_source_sel);
			if((s4g_sel & 0x01) == 1)
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, acc_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
			else
				PlotY (mainpnl, MAINPNL_SIG4GRAPH, gyr_mag, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);
		}
	}
	ProcessDrawEvents();
}


void Update_PD_Plots(void)
{
	int		pd_sel, pd_dest;
	char	pd_str[128];
	
	GetCtrlVal(mainpnl, MAINPNL_PEAK_DETECTOR_SEL, &pd_sel);
	if((pd_sel != 0) && (pd_peak_cnt != 0)) 
	{															   
		IsListItemChecked(mainpnl, MAINPNL_PD_PLOT, 0, &pd_dest); 
		if(pd_dest)
		{
			UpdateSG1();
			PlotXY (mainpnl, MAINPNL_SIG1GRAPH, peak_position, peak_ampl, pd_peak_cnt, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_BLACK);
		}
	
		IsListItemChecked(mainpnl, MAINPNL_PD_PLOT, 1, &pd_dest); 
		if(pd_dest)
		{
			UpdateSG2();
			PlotXY (mainpnl, MAINPNL_SIG2GRAPH, peak_position, peak_ampl, pd_peak_cnt, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_BLACK);
		}
	
		IsListItemChecked(mainpnl, MAINPNL_PD_PLOT, 2, &pd_dest); 
		if(pd_dest)
		{
			UpdateSG3();
			PlotXY (mainpnl, MAINPNL_SIG3GRAPH, peak_position, peak_ampl, pd_peak_cnt, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_BLACK);
		}
	
		IsListItemChecked(mainpnl, MAINPNL_PD_PLOT, 3, &pd_dest); 
		if(pd_dest)
		{
			UpdateSG4();
			PlotXY (mainpnl, MAINPNL_SIG4GRAPH, peak_position, peak_ampl, pd_peak_cnt, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_BLACK);
		}
		
		sprintf(pd_str, "%d Steps", (int)pd_peak_cnt);
		SetCtrlVal(mainpnl, MAINPNL_STEPS_DISP, pd_str);
		sprintf(pd_str, "%d Stps/Min", (int)(60.0 * (double)pd_peak_cnt / (double)ser_input_size * 100.0));
		SetCtrlVal(mainpnl, MAINPNL_PACE_DISP, pd_str); 
	}	
}

void Update_Sig_Plots(void)
{
//	UpdateSG1();
	UpdateSG2();
	UpdateSG3();
	UpdateSG4();
	Update_PD_Plots();
//	Update_Cursors ();
//	AX3dPlot();
//	Apply_Histogram_Mag();
//	Apply_Histogram_Acc();
//	Apply_Histogram_Gyro();
//	Determine_Activity();
}

int CVICALLBACK Accel_and_Gyro_Cursor_Update (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			Update_Cursors ();
			break;
	}
	return 0;
}

void Update_Cursors (void)
{
///	double throwaway_y = 0.0;
	
///	GetGraphCursor (mainpnl, MAINPNL_SIG1GRAPH, 1,&acc_plot_index ,&throwaway_y );
///	SetGraphCursor (mainpnl, MAINPNL_SIG2GRAPH, 1, acc_plot_index ,0.0);
///	SetGraphCursor (mainpnl, MAINPNL_SIG3GRAPH, 1, acc_plot_index ,0.0);
///	SetGraphCursor (mainpnl, MAINPNL_SIG4GRAPH, 1, acc_plot_index ,0.0);
///	Get_Gravity_Vector();
}
