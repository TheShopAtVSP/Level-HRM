/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Si1153 Control.c                                  			         */
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


//============================== Callbacks =========================================

int CVICALLBACK Set_LED_Source (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	int led_source;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:

			GetCtrlVal(mainpnl, MAINPNL_LED_RING, &led_source);
			if(led_source)
				strcpy(ser_str, "b");   // Int
			else
				strcpy(ser_str, "a");   // Ext
			ComWrt (ser_com_port, ser_str, 1);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.10);
			break;
	}
	return 0;
}


int CVICALLBACK Chan_1_LED_Enable (int panel, int control, int event,
								   void *callbackData, int eventData1, int eventData2)
{
	unsigned char c1_led_en = 0;
	int c1_butt_check;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_CH1_LEDA_EN, &c1_butt_check);
			if(c1_butt_check)
				c1_led_en = 0x01;	
			GetCtrlVal(mainpnl, MAINPNL_CH1_LEDB_EN, &c1_butt_check);
			if(c1_butt_check)
				c1_led_en = c1_led_en | 0x04;
			GetCtrlVal(mainpnl, MAINPNL_CH1_LEDC_EN, &c1_butt_check);
			if(c1_butt_check)
				c1_led_en = c1_led_en | 0x02;	
			strcpy(ser_str, "c"); 
			ComWrt (ser_com_port, ser_str, 1);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.10);
			ComWrtByte (ser_com_port, (unsigned char)c1_led_en);
			ComWrtByte (ser_com_port, 0x000A);
			break;
	}
	return 0;
}


int CVICALLBACK Chan_2_LED_Enable (int panel, int control, int event,
								   void *callbackData, int eventData1, int eventData2)
{
	unsigned char c2_led_en = 0;
	int c2_butt_check;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_CH2_LEDA_EN, &c2_butt_check);
			if(c2_butt_check)
				c2_led_en = 0x01;	
			GetCtrlVal(mainpnl, MAINPNL_CH2_LEDB_EN, &c2_butt_check);
			if(c2_butt_check)
				c2_led_en = c2_led_en | 0x04;
			GetCtrlVal(mainpnl, MAINPNL_CH2_LEDC_EN, &c2_butt_check);
			if(c2_butt_check)
				c2_led_en = c2_led_en | 0x02;
			//c2_led_en = c2_led_en + 0xF8;
			strcpy(ser_str, "d"); 
			ComWrt (ser_com_port, ser_str, 1);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.10);
			ComWrtByte (ser_com_port, (unsigned char)c2_led_en);
			ComWrtByte (ser_com_port, 0x000A);
			break;
	}
	return 0;
}


int CVICALLBACK LED_A_Current (int panel, int control, int event,
							   void *callbackData, int eventData1, int eventData2)
{
	unsigned char led_current;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_LED_A_I, &led_current); 
			strcpy(ser_str, "e");   //LED 1
			ComWrt (ser_com_port, ser_str, 1);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.10);
			ComWrtByte (ser_com_port, led_current);
			ComWrtByte (ser_com_port, 0x000A);
			break;
	}
	return 0;
}


int CVICALLBACK LED_B_Current (int panel, int control, int event,
							   void *callbackData, int eventData1, int eventData2)
{
	unsigned char led_current;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_LED_B_I, &led_current); 
			strcpy(ser_str, "f"); //LED 2
			ComWrt (ser_com_port, ser_str, 1);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.10);
			ComWrtByte (ser_com_port, led_current);
			ComWrtByte (ser_com_port, 0x000A);
			break;
	}
	return 0;
}

int CVICALLBACK LED_C_Current (int panel, int control, int event,
							   void *callbackData, int eventData1, int eventData2)
{
	unsigned char led_current;
	char ser_str[8];
	
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_LED_C_I, &led_current); 
			strcpy(ser_str, "g"); //LED 3
			ComWrt (ser_com_port, ser_str, 1);
			ComWrtByte (ser_com_port, 0x000A);
			Delay(0.10);
			ComWrtByte (ser_com_port, led_current);
			ComWrtByte (ser_com_port, 0x000A);
			break;
	}
	return 0;
}
