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
	unsigned char 	byte_in = 0;
	char			ser_str[128], ser_char_in[8];
	int  			i, sil;
	double			start_time, sps;
	

	GetCtrlVal(mainpnl, MAINPNL_NRTS, &ser_input_size); 
	GetCtrlVal(mainpnl, MAINPNL_SAMP_PER_SEC, &sps);
	sig1points = ser_input_size;
	FlushInQ (ser_com_port);
	for(sil=0; sil<512; sil++)
		byte_in = Serial_Get_Byte();
	
	for(sil=0; sil<ser_input_size + 64; sil++)
	{
		start_time = Timer();
		
		while( byte_in != 65 )
		{
			byte_in = Serial_Get_Byte(); 
		}
		strcpy(ser_str, "");
		
		while(1)
		{
			byte_in = Serial_Get_Byte(); 
			if( byte_in == 66)
				break;
			sprintf(ser_char_in, "%c", byte_in);
			strcat(ser_str, ser_char_in);
		}
		raw_acl_x[sil] = atof(ser_str);
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, ser_str);
		
		strcpy(ser_str, "");
		while(1)
		{
			byte_in = Serial_Get_Byte(); 
			if( byte_in == 67)
				break;
			sprintf(ser_char_in, "%c", byte_in);
			strcat(ser_str, ser_char_in);
		}
		raw_acl_y[sil] = atof(ser_str);
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, ser_str);
		
		strcpy(ser_str, "");
		while(1)
		{
			byte_in = Serial_Get_Byte(); 
			if( byte_in == 68)
				break;
			sprintf(ser_char_in, "%c", byte_in);
			strcat(ser_str, ser_char_in);
		}
		raw_acl_z[sil] = atof(ser_str);
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, ser_str);
		
		strcpy(ser_str, "");
		while(1)
		{
			byte_in = Serial_Get_Byte(); 
			if( byte_in == 69)
				break;
			sprintf(ser_char_in, "%c", byte_in);
			strcat(ser_str, ser_char_in);
		}
		raw_gyr_p[sil] = atof(ser_str);
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, ser_str);
		
		strcpy(ser_str, "");
		while(1)
		{
			byte_in = Serial_Get_Byte(); 
			if( byte_in == 70)
				break;
			sprintf(ser_char_in, "%c", byte_in);
			strcat(ser_str, ser_char_in);
		}
		raw_gyr_y[sil] = atof(ser_str);
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, ser_str);
		
		strcpy(ser_str, "");
		while(1)
		{
			byte_in = Serial_Get_Byte(); 
			if( byte_in == 71)
				break;
			sprintf(ser_char_in, "%c", byte_in);
			strcat(ser_str, ser_char_in);
		}
		raw_gyr_r[sil] = atof(ser_str);
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, ser_str);
		//sprintf(ser_str, "\n");
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, ser_str);
		
		while((Timer() - start_time) < 1.0 / sps) {}
	}
	
	for(i=0; i<ser_input_size + 64; i++) // apply ss values
	{
		raw_acl_x[i] -= ssax;
		raw_acl_y[i] -= ssay;
		raw_acl_z[i] -= ssaz;
		raw_gyr_p[i] -= ssgp;
		raw_gyr_y[i] -= ssgy;
		raw_gyr_r[i] -= ssgr;

	}
		
}

unsigned char Serial_Get_Byte(void)
{
	unsigned char 	byte_in;
	int  			ser_to = 0;
	
	while( GetInQLen(ser_com_port) == 0 ) 
	{ 
		if(ser_to == 100)
		{
			CloseCom (ser_com_port);
			Delay(0.10);
			RS232Error = OpenComConfig (ser_com_port, "COM3", 28800, 0, 8, 1, 32, 32);
			Delay(0.10);
			ser_to = 0;
		}
		ser_to++;
		Delay(0.025);
	}
	byte_in = (char)ComRdByte (ser_com_port);
	
		return byte_in;
}
