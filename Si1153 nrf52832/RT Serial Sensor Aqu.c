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


void RT_Scan_Loop(void)
{
	unsigned char 	byte_in_h = 0, byte_in_l = 0;
	char			fetch_str[4];
	int  			i = 0, sil, gyravgfilt_val, stitch_thresh_val, one_shot;
	double			start_time, sps, a_scale = 1.0;
	signed short 	si_ax, si_ay, si_az, si_gp, si_gy, si_gr, si_hart, dac_out;
	char 			out_str[256];
	double			pgy[4], ggprdavgaccm, ggyrdavgaccm;
	
	GetCtrlVal(mainpnl, MAINPNL_GYRAVGFV, &gyravgfilt_val); 
	GetCtrlVal(mainpnl, MAINPNL_STITCH_TH, &stitch_thresh_val);
	GetCtrlVal(mainpnl, MAINPNL_ONE_SHOT, &one_shot);
	
	if(raw_hart_cir_buff_idx > 4000)
	{
		raw_hart_cir_buff_idx = 0;
		data_stream_cir_buff_idx = last_data_stream_cir_buff_idx = 0;
		total_run_time = total_peaks = 0;
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
	SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_4, byte_in_h);
	SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_5, byte_in_h);
	byte_in_l = Serial_Get_Byte();
	SetCtrlVal(mainpnl, MAINPNL_HRM_7SEG_6, byte_in_l);
	si_az = (short)(byte_in_h << 8) + byte_in_l;
	raw_acl_z[raw_hart_cir_buff_idx] = (double)(si_az) / a_scale;

	byte_in_h = Serial_Get_Byte(); 
	byte_in_l = Serial_Get_Byte();
	si_gp = (short)(byte_in_h << 8) + byte_in_l;
		
	byte_in_h = Serial_Get_Byte(); 
	byte_in_l = Serial_Get_Byte();
	si_gy = (short)(byte_in_h << 8) + byte_in_l;
	
	/// int16_t 	hrm_chan1_raw[156], hrm_chan2_raw[156], hrm_raw_index = 0, hrm_raw_index_old;
	
	hrm_chan1_raw[hrm_raw_index] = (int16_t)si_gp;
	hrm_chan2_raw[hrm_raw_index] = (int16_t)si_gy;
	hrm_raw_index++;
	if(hrm_raw_index == 120 + gyravgfilt_val)
	{
		/// Apply_Raw_Data_Filtering();
		hrm_raw_index -= gyravgfilt_val;
		ser_input_size = hrm_raw_index;
		Apply_Peak_Detector();
		if(!one_shot)
			hrm_raw_index = 0;  
	}
	
	///////   if(!selfsteadyhasrun)
	if(1) /// <=============== debug
	{
		raw_hart_cir_buff[raw_hart_cir_buff_idx] = (double)si_gp;
		 raw_hart_cir_buff[raw_hart_cir_buff_idx] -= ssgp;
		raw_gyr_p[raw_hart_cir_buff_idx] =  (double)si_gp;
		 raw_gyr_p[raw_hart_cir_buff_idx] -= ssgp; 
		raw_gyr_y[raw_hart_cir_buff_idx] =  (double)si_gy;
		 raw_gyr_y[raw_hart_cir_buff_idx] -= ssgy; 
	}
	else
	{
		raw_hart_cir_buff[raw_hart_cir_buff_idx] = (double)si_gp;
		/// raw_hart_cir_buff[raw_hart_cir_buff_idx] -= ssgp;
		/// if((raw_hart_cir_buff[raw_hart_cir_buff_idx] < stitch_thresh_val ) && (raw_hart_cir_buff[raw_hart_cir_buff_idx] > -stitch_thresh_val ) && glasses_on)
		if( ((raw_hart_cir_buff[raw_hart_cir_buff_idx] < stitch_thresh_val ) && (raw_hart_cir_buff[raw_hart_cir_buff_idx] > -stitch_thresh_val ) && glasses_on) \
			|| ((raw_gyr_y[raw_hart_cir_buff_idx] < stitch_thresh_val ) && (raw_gyr_y[raw_hart_cir_buff_idx] > -stitch_thresh_val ) && glasses_on) )
		{
			ggprdavg[ggrdavgindex] = (double)si_gp;  - ssgp;
			ggyrdavg[ggrdavgindex] = (double)si_gy;  - ssgy;
			ggrdavgindex++;
			if(ggrdavgindex >= gyravgfilt_val)
				ggrdavgindex = 0;
		
			ggprdavgaccm = ggyrdavgaccm = 0;
			for(i=0; i<gyravgfilt_val; i++)
			{
				ggprdavgaccm += ggprdavg[i];
				ggyrdavgaccm += ggyrdavg[i]; 
			}
			raw_gyr_p[data_stream_cir_buff_idx] = ggprdavgaccm / (double)gyravgfilt_val;
			raw_gyr_y[data_stream_cir_buff_idx] = ggyrdavgaccm / (double)gyravgfilt_val;
		}
	}

	if(!selfsteadyhasrun) 
	{
		if(raw_hart_cir_buff_idx == 25)
		{
			 for(i=0; i<10; i++)
			 {
				  ssgp += raw_gyr_p[i+15];
				  ssgy += raw_gyr_y[i+15];
			 }
			  ssgp = ssgp/10;
			  ssgy = ssgy/10; 
			 
			 for(i=0; i<25; i++)
			 {
				 ggprdavg[i] = 0;
				 ggyrdavg[i] = 0;
			 }
			 
			 selfsteadyhasrun = 1;
			 
		 	raw_hart_cir_buff_idx = 0;
			data_stream_cir_buff_idx = 0;
			last_data_stream_cir_buff_idx = 0;
			look_back_starting_point = 0;
			aquisition_add_time = 0;
			GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time);
			total_run_time = total_peaks = 0;
			FlushInQ (ser_com_port);
			aquisition_start_time = Timer();
		 //	sprintf(out_str, "ssgp %f\n",  ssgp);
		 //	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
		}
		else
			raw_hart_cir_buff_idx++; 
	}
	else
	{
		if( ((raw_hart_cir_buff[raw_hart_cir_buff_idx] < stitch_thresh_val ) && (raw_hart_cir_buff[raw_hart_cir_buff_idx] > -stitch_thresh_val ) && glasses_on) \
			|| ((raw_gyr_y[raw_hart_cir_buff_idx] < stitch_thresh_val ) && (raw_gyr_y[raw_hart_cir_buff_idx] > -stitch_thresh_val ) && glasses_on) )
		{
			if(stitch_rear_edge_dect && (data_stream_cir_buff_idx > 1) && (raw_hart_cir_buff_idx > 1))
			{
				raw_hart_cir_buff_idx--;
				data_stream_cir_buff_idx--;
				raw_hart_cir_buff_idx--;
				data_stream_cir_buff_idx--;
#if 0				
				
				raw_hart_cir_buff[raw_hart_cir_buff_idx] = (double)si_gp;
				raw_hart_cir_buff[raw_hart_cir_buff_idx] -= ssgp;
				raw_gyr_p[raw_hart_cir_buff_idx] =  (double)si_gp;
				raw_gyr_p[raw_hart_cir_buff_idx] -= ssgp; 
				raw_gyr_y[raw_hart_cir_buff_idx] =  (double)si_gy;
				raw_gyr_y[raw_hart_cir_buff_idx] -= ssgy;
#endif				
				
				for(i=0; i<gyravgfilt_val; i++)
				{
					ggprdavg[i] = raw_gyr_p[data_stream_cir_buff_idx];
					ggyrdavg[i] = raw_gyr_y[data_stream_cir_buff_idx]; 
				}
				FlushInQ (ser_com_port); 
				stitch_rear_edge_dect = 0;
			}
			
			pgy[0] = raw_gyr_p[data_stream_cir_buff_idx];
			pgy[1] = raw_gyr_y[data_stream_cir_buff_idx];
			pgy[2] = raw_gyr_p[data_stream_cir_buff_idx];
			pgy[3] = raw_gyr_y[data_stream_cir_buff_idx];
			PlotStripChart (mainpnl, MAINPNL_SIG1GRAPH, pgy , 4, 0, 0, VAL_DOUBLE);
			data_stream_cir_buff_idx++;
			run_time += aquisition_add_time;
			aquisition_add_time = 0;
			last_good_time = Timer();        
		}
		else if( data_stream_cir_buff_idx > 5)
		{
			pgy[0] = 0;
			pgy[1] = 0;
			pgy[2] = raw_gyr_p[data_stream_cir_buff_idx];
			pgy[3] = raw_gyr_y[data_stream_cir_buff_idx];
			PlotStripChart (mainpnl, MAINPNL_SIG1GRAPH, pgy , 4, 0, 0, VAL_DOUBLE);
			stitch_rear_edge_dect = 1;
			aquisition_add_time = Timer() - last_good_time;
		}
		
		if( (raw_hart_cir_buff_idx > 5) &&
		((raw_hart_cir_buff[raw_hart_cir_buff_idx] > (raw_hart_cir_buff[raw_hart_cir_buff_idx-5] + 100)) ||
		(raw_hart_cir_buff[raw_hart_cir_buff_idx] < (raw_hart_cir_buff[raw_hart_cir_buff_idx-5] - 100))) )
		{
			 ssgp = ssgp + (((double)si_gp - ssgp) / 2.0) ;
		}
		
		// we need a second sensor var array for rt ssgy 
		if( (raw_hart_cir_buff_idx > 5) &&  
		((raw_gyr_y[raw_hart_cir_buff_idx] > (raw_gyr_y[raw_hart_cir_buff_idx-5] + 100)) ||
		(raw_gyr_y[raw_hart_cir_buff_idx] < (raw_gyr_y[raw_hart_cir_buff_idx-5] - 100))) )
		{
			 ssgy = ssgy + (((double)si_gy - ssgy) / 2.0) ;
		}
		
		 ssgp = ssgp + (((double)si_gp - ssgp) / 20.0) ; 
		 ssgy = ssgy + (((double)si_gy - ssgy) / 20.0) ; 
		
		raw_hart_cir_buff_idx++;
		
		/// Glasses on/off detector ///
		if(ssgp < 10)
		{
			SetCtrlVal(mainpnl, MAINPNL_GLASSESONMSG, "Glasses Off");
			SetCtrlAttribute (mainpnl, MAINPNL_GLASSESONMSG, ATTR_TEXT_COLOR, VAL_RED);
			glasses_on = 1; /// 0
		}
		else
		{
			SetCtrlVal(mainpnl, MAINPNL_GLASSESONMSG, "Glasses On");
			SetCtrlAttribute (mainpnl, MAINPNL_GLASSESONMSG, ATTR_TEXT_COLOR, VAL_GREEN);
			glasses_on = 1;
		}
		
		/// Sample Time Done and Finish Up ///
		if(Timer() > (run_time + aquisition_start_time + aquisition_add_time))
		{
			GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time); 
			ser_input_size = data_stream_cir_buff_idx;     
		///	Apply_Raw_Data_Filtering();
			sprintf(out_str, "ssgp %f, ssgy %f\n", ssgp, ssgy);
			SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
			
			//sprintf(out_str, "timer %f, aquisition_start_time %f run_time %f\n", Timer(), aquisition_start_time, run_time);
			//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
			//SetCtrlVal(mainpnl, MAINPNL_SAMP_PER_SEC, raw_hart_cir_buff_idx / (Timer() - aquisition_start_time)); 
			SetCtrlVal(mainpnl, MAINPNL_SAMP_PER_SEC, raw_hart_cir_buff_idx / run_time);    
			
			for(i=0; i<ser_input_size+64; i++)
			{
				 fio_array[i*6+0] = raw_acl_x[i];
				 fio_array[i*6+1] = raw_acl_y[i];
				 fio_array[i*6+2] = raw_acl_z[i];
				 fio_array[i*6+3] = raw_gyr_p[i];
				 fio_array[i*6+4] = raw_gyr_y[i];
				 fio_array[i*6+5] = raw_gyr_r[i];
			}
															  
			raw_hart_cir_buff_idx = data_stream_cir_buff_idx = 0;
			GetCtrlVal(mainpnl, MAINPNL_ONE_SHOT, &one_shot);
			if(one_shot)
				SetCtrlVal(mainpnl, MAINPNL_GETRTDATA, 0); 
			stitch_rear_edge_dect = 1;
			FlushInQ (ser_com_port); 
			aquisition_start_time = Timer();
		}
	}
	return; 	
}

unsigned char Serial_Get_Byte(void)
{
	unsigned char 	byte_in = 0;
	int  			ser_to = 0;
/// int				dbg_cntr=0;
	
	while( GetInQLen(ser_com_port) == 0 ) 
	{ 
		if(ser_to == 100)
		{
			return 0;
			CloseCom (ser_com_port);
			Delay(0.10);
			RS232Error = OpenComConfig (ser_com_port, "COM7", 115200, 0, 8, 1, 512, 512);
			Delay(2.00);
			ser_to = 0;
			/// dbg_cntr++;
		}
		ser_to++;
		Delay(0.001);
	}
	/// printf("%d\n", dbg_cntr);
	if( GetInQLen(ser_com_port) != 0 ) 
	byte_in = (unsigned char)ComRdByte (ser_com_port);
		
	return byte_in;
}

// ======================== CallBacks ===============================================
int CVICALLBACK RT_Timer_CB (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	int go_baby;
	
	switch (event)
	{
		case EVENT_TIMER_TICK:
			GetCtrlVal(mainpnl, MAINPNL_GETRTDATA, &go_baby);
			if(go_baby)
				RT_Scan_Loop();
			else
			{
				raw_hart_cir_buff_idx = data_stream_cir_buff_idx = 0;
				last_data_stream_cir_buff_idx = look_back_starting_point = 0;
				aquisition_add_time = 0;
				GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time);
				///selfsteadyhasrun = 0;
				total_run_time = total_peaks = 0;
				FlushInQ (ser_com_port);
				aquisition_start_time = Timer();
				current_hrm = 69;
			}
			break;
	}
	return 0;
}

int CVICALLBACK SetAquisisionTime (int panel, int control, int event,
								   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time); 
			break;
	}
	return 0;
}


int CVICALLBACK ClearStripChartCB (int panel, int control, int event,
								   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			ClearStripChart (mainpnl, MAINPNL_SIG1GRAPH);
			break;
	}
	return 0;
}

int CVICALLBACK Restart_Aquisition (int panel, int control, int event,
									void *callbackData, int eventData1, int eventData2)
{
	int startdataedge;
	
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_GETRTDATA, &startdataedge);
			if(startdataedge)
				hrm_raw_index = 0; 
			break;
	}
	return 0;
}
