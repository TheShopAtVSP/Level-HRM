/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    RT serial sensor acquisition.c                                   */
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


/*
int CVICALLBACK RT_Scan_Loop_CB (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	int auto_ss;
	
	switch (event)
	{
		case EVENT_COMMIT:
		
			GetCtrlVal(mainpnl, MAINPNL_SS_AUTO, &auto_ss);
			if(auto_ss)
				Self_Steady();
			Uno_Scan_Loop();
			Apply_Raw_Data_Filtering();
		
		break;
	}
	return 0;
}
*/
void RT_Scan_Loop(void)
{
	unsigned char 	byte_in_h = 0, byte_in_l = 0;
	char			fetch_str[4];
	int  			i = 0, sil;
	double			start_time, sps, g_scale = 256.0, a_scale = 256.0, double_dac_out;
	unsigned short 	si_ax, si_ay, si_az, si_gp, si_gy, si_gr, si_hart, dac_out;
	char 			out_str[256];
	int				update_size;
	
	//update_size = ser_input_size;
	update_size = 300; 

	///if(raw_hart_cir_buff_idx >= update_size + 64)
		///raw_hart_cir_buff_idx = 0;
	if(!selfsteadyhasrun)
	{
		ssgp = 0;
		ssgy = 0;
	}
	
	Serial_Get_Byte();
	while(Serial_Get_Byte() != 0x69){}
	
	byte_in_h = Serial_Get_Byte(); 
	byte_in_l = Serial_Get_Byte(); 
	si_ax = (short)(byte_in_h << 8) + byte_in_l;
	raw_acl_x[raw_hart_cir_buff_idx] = (double)(si_ax) / a_scale;
		
	byte_in_h = Serial_Get_Byte(); 
	byte_in_l = Serial_Get_Byte(); 
	si_ay = (short)(byte_in_h << 8) + byte_in_l;
	raw_acl_y[raw_hart_cir_buff_idx] = (double)(si_ay) / a_scale;

	byte_in_h = Serial_Get_Byte(); 
	byte_in_l = Serial_Get_Byte(); 
	si_az = (short)(byte_in_h << 8) + byte_in_l;
	raw_acl_z[raw_hart_cir_buff_idx] = (double)(si_az) / a_scale;

	byte_in_h = Serial_Get_Byte(); 
	byte_in_l = Serial_Get_Byte();
	si_gp = (short)(byte_in_h << 8) + byte_in_l;
	raw_gyr_p[raw_hart_cir_buff_idx] = (double)si_gp;
	raw_gyr_p[raw_hart_cir_buff_idx] -= ssgp;
	raw_hart_cir_buff[raw_hart_cir_buff_idx] = (double)si_gp;// / g_scale;
	raw_hart_cir_buff[raw_hart_cir_buff_idx] -= ssgp;

	byte_in_h = Serial_Get_Byte(); 
	byte_in_l = Serial_Get_Byte();
	si_gy = (short)(byte_in_h << 8) + byte_in_l;
	raw_gyr_y[raw_hart_cir_buff_idx] =  (double)si_gy;
	raw_gyr_y[raw_hart_cir_buff_idx] -= ssgy; 

	if(!selfsteadyhasrun)
	{
		if(raw_hart_cir_buff_idx == 9)
		{
			 Mean (raw_gyr_p, 10 , &raw_hart_cir_buff_mean );
			 ssgp =  raw_hart_cir_buff_mean;
			 Mean (raw_gyr_y, 10 , &raw_hart_cir_buff_mean );
			 ssgy =  raw_hart_cir_buff_mean;
			 selfsteadyhasrun = 1;
		}
		raw_hart_cir_buff_idx++; 
	}
	else
	{
	
		if((raw_hart_cir_buff[raw_hart_cir_buff_idx] < (1000) ) && (raw_hart_cir_buff[raw_hart_cir_buff_idx] > (-1000) ))
		{
			PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_hart_cir_buff[raw_hart_cir_buff_idx]);
			raw_hart_cir_buff_idx++;
		}
		
		if( (raw_hart_cir_buff_idx > 5) &&
		((raw_gyr_p[raw_hart_cir_buff_idx] > (raw_gyr_p[raw_hart_cir_buff_idx-5] + 100)) ||
		(raw_gyr_p[raw_hart_cir_buff_idx] < (raw_gyr_p[raw_hart_cir_buff_idx-5] - 100))) )
		{
			ssgp = ((ssgp * 10) + (double)si_gp) / 11;
		}
		
		if( (raw_hart_cir_buff_idx > 5) &&
		((raw_gyr_y[raw_hart_cir_buff_idx] > (raw_gyr_y[raw_hart_cir_buff_idx-5] + 100)) ||
		(raw_gyr_y[raw_hart_cir_buff_idx] < (raw_gyr_y[raw_hart_cir_buff_idx-5] - 100))) )
		{
			ssgy = ((ssgy * 10) + (double)si_gy) / 11;
		}
			
			
		if(raw_hart_cir_buff_idx > update_size)
		{
			ser_input_size = update_size;
			Apply_Raw_Data_Filtering();
			/*
			GetCtrlVal(mainpnl, MAINPNL_S2GX, &s2gx);
			GetCtrlVal(mainpnl, MAINPNL_S2GY, &s2gy);
			DeleteGraphPlot (mainpnl, MAINPNL_SIG2GRAPH, -1, VAL_IMMEDIATE_DRAW);  
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_hart_cir_buff, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED); 
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
				//Apply_Raw_Data_Filtering();
				///		printf("RDF %d\n", ser_input_size);
			*/
			raw_hart_cir_buff_idx = 0;
		}	
			
			
			
			
	}
	
	
	/*
	if( (raw_hart_cir_buff_idx > 5) &&
		(raw_hart_cir_buff[raw_hart_cir_buff_idx] < (raw_hart_cir_buff[raw_hart_cir_buff_idx-5] + 100)) &&
		(raw_hart_cir_buff[raw_hart_cir_buff_idx] > (raw_hart_cir_buff[raw_hart_cir_buff_idx-5] - 100)))
	{
		PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_hart_cir_buff[raw_hart_cir_buff_idx]);
		if(raw_hart_cir_buff_idx > update_size)
		{
			ser_input_size = update_size; 
			//for(int i=0; i < ser_input_size; i++)
			//{
			//	raw_gyr_p[i] = raw_hart_cir_buff[i];
			//}
			GetCtrlVal(mainpnl, MAINPNL_S2GX, &s2gx);
			GetCtrlVal(mainpnl, MAINPNL_S2GY, &s2gy);
			DeleteGraphPlot (mainpnl, MAINPNL_SIG2GRAPH, -1, VAL_IMMEDIATE_DRAW);  
			if(s2gx)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_hart_cir_buff, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED); 
			if(s2gy)
				PlotY (mainpnl, MAINPNL_SIG2GRAPH, raw_gyr_y, ser_input_size, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
			//Apply_Raw_Data_Filtering();
	///		printf("RDF %d\n", ser_input_size);
			raw_hart_cir_buff_idx = 0;
		}
		raw_hart_cir_buff_idx++;
	}
	
	if(raw_hart_cir_buff_idx >= 5)
	{
		Mean (raw_hart_cir_buff, raw_hart_cir_buff_idx , &raw_hart_cir_buff_mean );
		ssgp = raw_hart_cir_buff_mean;
		sprintf(out_str, "ssgp %f, si_gp %f\n", ssgp, (double)si_gp);
		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	}
*/
	
	
/*	
	if( (raw_hart_cir_buff_idx > 5) &&
		((raw_hart_cir_buff[raw_hart_cir_buff_idx] > (raw_hart_cir_buff[raw_hart_cir_buff_idx-5] + 100)) ||
		(raw_hart_cir_buff[raw_hart_cir_buff_idx] < (raw_hart_cir_buff[raw_hart_cir_buff_idx-5] - 100))))
	{
		ssgp = ((ssgp * 10) + (double)si_gp) / 11;
		///			sprintf(out_str, "ssgp %f, si_gp %f\n\n", ssgp, (double)si_gp);
		///			SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	}
*/

/*
	if(raw_hart_cir_buff_idx <= 5) 
	   raw_hart_cir_buff_idx++;
*/
	
/*	
//	else
//		raw_hart_cir_buff_idx++;
	
	if( (raw_hart_cir_buff_idx > 5) &&
		(raw_gyr_y[raw_hart_cir_buff_idx] < (raw_gyr_y[raw_hart_cir_buff_idx-5] + 100)) &&
		(raw_gyr_y[raw_hart_cir_buff_idx] > (raw_gyr_y[raw_hart_cir_buff_idx-5] - 100)))
	{
		ssgy = ((ssgy * 10) + (double)si_gy) / 11;
		///			sprintf(out_str, "ssgy %f, si_gy %f\n\n", ssgy, (double)si_gy);
		///			SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	}
*/
/*
	if((raw_hart_cir_buff_idx <= 5) &&(!self_steady) && ((raw_hart_cir_buff[raw_hart_cir_buff_idx] < (1000) ) && (raw_hart_cir_buff[raw_hart_cir_buff_idx] > (-1000) )))
	{
		PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_hart_cir_buff[raw_hart_cir_buff_idx]);
		raw_hart_cir_buff_idx++;
	////////	///if( (raw_hart_cir_buff_idx >= update_size) && (raw_hart_cir_buff_idx <= update_size+4) )
	}
*/
	return; 	
}
//

int CVICALLBACK RT_Timer_CB (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:
			
			RT_Scan_Loop();
			break;
	}
	return 0;
}
