/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Uno serial sensor acquisition.c                                  */
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

/*

int CVICALLBACK Uno_Scan_Loop_CB (int panel, int control, int event,
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

void Uno_Scan_Loop(void)
{
	unsigned char 	byte_in_h = 0, byte_in_l = 0;
	char			fetch_str[4];
	int  			i, sil;
	double			start_time, sps, g_scale = 256.0, a_scale = 256.0, double_dac_out;
	unsigned short 			si_ax, si_ay, si_az, si_gp, si_gy, si_gr, si_hart, dac_out;
///	int				dbg_cntr = 0; /// 
///	char 			tmp_str[128];   /// 
///	double			db_start_time;  ///
	
	GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time);
	GetCtrlVal(mainpnl, MAINPNL_SAMP_PER_SEC, &sps);
	ser_input_size = run_time * sps;
	a_scale = 1;
	
	if(self_steady)
	{
		GetCtrlVal(mainpnl, MAINPNL_SS_SAMP_NUMB, &ser_input_size);/// moved up
	}
	else
	{
		GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time);  /// moved up
		ser_input_size = run_time * sps;
	}
	sig1points = ser_input_size;
	
	for(sil=0; sil<ser_input_size + 64; sil +=  self_steady)
	{
		//FlushInQ (ser_com_port); 
		Serial_Get_Byte();
		while(Serial_Get_Byte() != 0x69){}
		
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte(); 
		si_ax = (short)(byte_in_h << 8) + byte_in_l;
		raw_acl_x[sil] = (double)(si_ax) / a_scale;
			
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte(); 
		si_ay = (short)(byte_in_h << 8) + byte_in_l;
		raw_acl_y[sil] = (double)(si_ay) / a_scale;
	
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte(); 
		si_az = (short)(byte_in_h << 8) + byte_in_l;
		raw_acl_z[sil] = (double)(si_az) / a_scale;
	
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gp = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_p[sil] = (double)(si_gp);// / g_scale;
		
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gy = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_y[sil] = (double)(si_gy);// / g_scale;
		
		raw_acl_x[sil] -= ssax;
		raw_acl_y[sil] -= ssay;
		raw_acl_z[sil] -= ssaz;
		raw_gyr_p[sil] -= ssgp;
		raw_gyr_y[sil] -= ssgy;
		
		if((!self_steady) && ((raw_gyr_p[sil] < (1000) ) && (raw_gyr_p[sil] > (-1000) )))
		{
			PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_gyr_p[sil]);
			sil++;
		}
		raw_acl_x[sil] = (double)(si_ax) / a_scale;
		raw_acl_y[sil] = (double)(si_ay) / a_scale;
		raw_acl_z[sil] = (double)(si_az) / a_scale;
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gp = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_p[sil] = (double)(si_gp);// / g_scale;
		raw_acl_x[sil] -= ssax;
		raw_acl_y[sil] -= ssay;
		raw_acl_z[sil] -= ssaz;
		raw_gyr_p[sil] -= ssgp;
		if((!self_steady) && ((raw_gyr_p[sil] < (1000) ) && (raw_gyr_p[sil] > (-1000) )))
		{
			PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_gyr_p[sil]);
			sil++;
		}
		raw_acl_x[sil] = (double)(si_ax) / a_scale;
		raw_acl_y[sil] = (double)(si_ay) / a_scale;
		raw_acl_z[sil] = (double)(si_az) / a_scale;
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gp = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_p[sil] = (double)(si_gp);// / g_scale;
		raw_acl_x[sil] -= ssax;
		raw_acl_y[sil] -= ssay;
		raw_acl_z[sil] -= ssaz;
		raw_gyr_p[sil] -= ssgp;
		if((!self_steady) && ((raw_gyr_p[sil] < (1000) ) && (raw_gyr_p[sil] > (-1000) )))
		{
			PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_gyr_p[sil]);
			sil++;
		}
		raw_acl_x[sil] = (double)(si_ax) / a_scale;
		raw_acl_y[sil] = (double)(si_ay) / a_scale;
		raw_acl_z[sil] = (double)(si_az) / a_scale;
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gp = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_p[sil] = (double)(si_gp);// / g_scale;
		raw_acl_x[sil] -= ssax;
		raw_acl_y[sil] -= ssay;
		raw_acl_z[sil] -= ssaz;
		raw_gyr_p[sil] -= ssgp;
		if((!self_steady) && ((raw_gyr_p[sil] < (1000) ) && (raw_gyr_p[sil] > (-1000) )))
		{
			PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_gyr_p[sil]);
			sil++;
			ssgp = ((ssgp * 20) + (double)si_gp) / 21;
		}
	//	else
	//		ssgp = ((ssgp * 100) + (double)si_gp) / 101;
	}

	return;
	
	
	
	GetCtrlVal(mainpnl, MAINPNL_DAC_SWITCH, &daconoff);
	GetCtrlVal(mainpnl, MAINPNL_AX_GAIN, &axdacgain);
	GetCtrlVal(mainpnl, MAINPNL_AY_GAIN, &aydacgain);
	GetCtrlVal(mainpnl, MAINPNL_AZ_GAIN, &azdacgain);
	GetCtrlVal(mainpnl, MAINPNL_DAC_GAIN, &dacgain);   
	GetCtrlVal(mainpnl, MAINPNL_INVERT_AX_SWITCH, &invdacax);
	if(!invdacax) invdacax = -1;
	GetCtrlVal(mainpnl, MAINPNL_INVERT_AY_SWITCH, &invdacay);
	if(!invdacay) invdacay = -1;
	GetCtrlVal(mainpnl, MAINPNL_INVERT_AZ_SWITCH, &invdacaz);
	if(!invdacaz) invdacaz = -1;
	
	GetCtrlVal(mainpnl, MAINPNL_SAMP_PER_SEC, &sps); 
	if(self_steady)
	{
		GetCtrlVal(mainpnl, MAINPNL_SS_SAMP_NUMB, &ser_input_size);/// moved up
	}
	else
	{
		GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time);  /// moved up
		ser_input_size = run_time * sps;
	}
	sig1points = ser_input_size;
	
	strcpy(fetch_str, "R");
	FlushInQ (ser_com_port);
	ComWrtByte (ser_com_port, fetch_str[0]); 
///	db_start_time = Timer();
	
	for(sil=0; sil<ser_input_size + 64; sil++)
	{
		start_time = Timer();				  	
		ComWrtByte (ser_com_port, fetch_str[0]);
			
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte(); 
		si_ax = (short)(byte_in_h << 8) + byte_in_l;
		raw_acl_x[sil] = (double)(si_ax) - ssax;/// / a_scale;
		
	//	PlotY (mainpnl, MAINPNL_SIG1GRAPH, raw_acl_x, 1, VAL_DOUBLE,VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
		PlotStripChartPoint (mainpnl, MAINPNL_SIG1GRAPH, raw_acl_x[sil]);
	
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte(); 
		si_ay = (short)(byte_in_h << 8) + byte_in_l;
		raw_acl_y[sil] = (double)(si_ay) / a_scale;
	
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte(); 
		si_az = (short)(byte_in_h << 8) + byte_in_l;
		raw_acl_z[sil] = (double)(si_az) / a_scale;
	
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gp = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_p[sil] = (double)(si_gp) / g_scale;
	
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gy = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_y[sil] = (double)(si_gy) / g_scale;
	
		byte_in_h = Serial_Get_Byte(); 
		byte_in_l = Serial_Get_Byte();
		si_gr = (short)(byte_in_h << 8) + byte_in_l;
		raw_gyr_r[sil] = (double)(si_gr) / g_scale;
	
		byte_in_h = Serial_Get_Byte();
		byte_in_l = Serial_Get_Byte();
		si_hart =  (short)(byte_in_h << 8) + byte_in_l; 
		/// si_hart =  si_hart - 250;
		raw_hart[sil] = (double)si_hart;  
		
		/// dbg_cntr++;
		SetCtrlVal(mainpnl, MAINPNL_SER_XFER_LED, (sil / 20) & 0x01);
		while((Timer() - start_time) < (1.0 / sps)) { Delay(0.001); } //dbg_cntr++; }
		
		// axdacgain, aydacgain, azdacgain, invdacax, invdacay, invdacaz, dacgain, daconoff;
		if(daconoff)
		{
			double_dac_out = ( ((raw_acl_x[sil] - ssax) * (double)invdacax * axdacgain * dacgain * 1) + \
			  				   ((raw_acl_y[sil] - ssay) * (double)invdacay * aydacgain * dacgain * 1) + \
			  				   ((raw_acl_z[sil] - ssaz) * (double)invdacaz * azdacgain * dacgain * 1) + 1860);
			if(double_dac_out > 4095.0) double_dac_out = 4095.0;
			if(double_dac_out < 0.0) double_dac_out = 0.0;
			dac_out = (short)double_dac_out;
			raw_gyr_r[sil] = double_dac_out;
		}
		else
			dac_out = 1860;
		
		byte_in_h = (dac_out >> 8) & 0x0F;
		ComWrtByte (ser_com_port, byte_in_h);
		byte_in_l = dac_out & 0xFF;
		ComWrtByte (ser_com_port, byte_in_l);
	}
	
	// apply ss values
	for(i=0; i<ser_input_size + 64; i++)
	{
		raw_acl_x[i] -= ssax;
		raw_acl_y[i] -= ssay;
		raw_acl_z[i] -= ssaz;
		raw_gyr_p[i] -= ssgp;
		raw_gyr_y[i] -= ssgy;
		raw_gyr_r[i] -= ssgr;

	}
///	sprintf(tmp_str, "%f sec dbc = %d\n", Timer() - db_start_time, dbg_cntr); /// 
///	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);							  /// 
	SetCtrlVal(mainpnl, MAINPNL_SER_XFER_LED, 0);	
}


*/
