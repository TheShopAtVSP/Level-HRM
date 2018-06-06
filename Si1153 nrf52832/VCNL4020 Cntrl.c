/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    VCNL4020 Control.c                                  			         */
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





int CVICALLBACK Set_LED_Current (int panel, int control, int event,
								 void *callbackData, int eventData1, int eventData2)
{
	unsigned char led_current;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:

			GetCtrlVal(mainpnl, MAINPNL_LED_CURRENT, &led_current); 
			strcpy(ser_str, "c"); 
			ComWrt (ser_com_port, ser_str, 1);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.20);
			ComWrtByte (ser_com_port, led_current);
			ComWrtByte (ser_com_port, 0x000A);
				
			break;
	}
	return 0;
}


int CVICALLBACK Set_LED_Source (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	int led_source;
	
	switch (event)
	{
		case EVENT_COMMIT:

			GetCtrlVal(mainpnl, MAINPNL_LED_RING, &led_source); 
			ComWrtByte (ser_com_port, led_source);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.01);
			break;
	}
	return 0;
}


int CVICALLBACK Set_Sample_Rate (int panel, int control, int event,
								 void *callbackData, int eventData1, int eventData2)
{
	int samp_rate;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:

			GetCtrlVal(mainpnl, MAINPNL_RATE_RING, &samp_rate); 
			strcpy(ser_str, "r"); 
			ComWrt (ser_com_port, ser_str, 1);
			Delay(0.010);
			ComWrtByte (ser_com_port, samp_rate);
			
			if(samp_rate == 1) samp_rate = 4;
			else if(samp_rate == 2) samp_rate = 8;
			else if(samp_rate == 3) samp_rate = 16;
			else if(samp_rate == 4) samp_rate = 31;
			else if(samp_rate == 5) samp_rate = 62;
			else if(samp_rate == 6) samp_rate = 125;
			else if(samp_rate == 7) samp_rate = 250;
			SetCtrlVal(mainpnl, MAINPNL_SAMP_PER_SEC, (double)samp_rate);
			break;
	}
	return 0;
}

/*
int CVICALLBACK Motor_Control_CB (int panel, int control, int event,
								  void *callbackData, int eventData1, int eventData2)
{
	int 	motor_run, motor_forward;
	char	ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:

			GetCtrlVal(mainpnl, MAINPNL_MOTOR_RUN, &motor_run);
			GetCtrlVal(mainpnl, MAINPNL_MOTOR_FWD, &motor_forward);
			
			if(motor_run)
			{
				if(motor_forward)
				{
					strcpy(ser_str, "F"); 
					ComWrt (ser_com_port, ser_str, 1);
					Set_Motor_Speed();
				}
				else
				{
					strcpy(ser_str, "B"); 
					ComWrt (ser_com_port, ser_str, 1);
					Set_Motor_Speed();
				}
			}
			else
			{
				strcpy(ser_str, "S"); 
				ComWrt (ser_com_port, ser_str, 1);	
			}
			
			break;
	}
	return 0;
}

int CVICALLBACK Motor_Speed_CB (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			Set_Motor_Speed();
			break;
	}
	return 0;
}

void  Set_Motor_Speed(void)
{
	double			motor_speed;
	char			ser_str[8];
	unsigned char	mspeed;
	int 			dir;
	
	GetCtrlVal(mainpnl, MAINPNL_RPS, &motor_speed);
	GetCtrlVal(mainpnl, MAINPNL_MOTOR_FWD, &dir);
	if(motor_speed < 0.5) motor_speed = 0.5;
	if(!dir)
		mspeed = (unsigned char) (85.0 + ((motor_speed - 1) * 170.0));   // ~90 = 1rps(F)
	else
		mspeed = (unsigned char) (85.0 + ((motor_speed - 1) * 170.0));   // ~210 = 1rps(B)
	strcpy(ser_str, "V"); 
	ComWrt (ser_com_port, ser_str, 1); // Guard Against serial noise
	strcpy(ser_str, "S");
	ComWrt (ser_com_port, ser_str, 1);
	ComWrtByte (ser_com_port,  mspeed);
}
*/
