/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Motor Control.c                                  			         */
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
