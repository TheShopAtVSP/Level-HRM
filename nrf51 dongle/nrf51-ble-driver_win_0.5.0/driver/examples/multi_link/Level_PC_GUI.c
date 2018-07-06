//==============================================================================
//
// Title:		Level_PC_GUI.c
// Purpose:		A short description of the application.
//
// Created on:	7/7/2016 at 3:34:35 PM by Mitch_Mason.
// Copyright:	CPI. All Rights Reserved.
//
//==============================================================================

//==============================================================================
// Include files

#include <ansi_c.h>
#include <cvirte.h>		
#include <userint.h>
#include "toolbox.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "Level_PC.h"
#include "Level PC Includes.h"

//==============================================================================
// Constants

//==============================================================================
// Types

//==============================================================================
// Static global variables

//==============================================================================
// Static functions

//==============================================================================
// Global variables

//==============================================================================
// Global functions and callbacks

/// led_id_index




int CVICALLBACK RedButton (int panel, int control, int event,
						   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			if(led_id_index == 0)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_0, ATTR_ON_COLOR, VAL_RED);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_0, 1);
			}
			if(led_id_index == 1)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_1, ATTR_ON_COLOR, VAL_RED);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_1, 1);
			}
			if(led_id_index == 2)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_2, ATTR_ON_COLOR, VAL_RED);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_2, 1);
			}
			if(led_id_index == 3)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_3, ATTR_ON_COLOR, VAL_RED);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_3, 1);
			}
			
			led_id_index++;
			if(led_id_index == 4)
				led_id_index = 0;
			//UART CALL
			break;
	}
	return 0;
}

int CVICALLBACK GreenButton (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			if(led_id_index == 0)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_0, ATTR_ON_COLOR, VAL_YELLOW);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_0, 1);
			}
			if(led_id_index == 1)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_1, ATTR_ON_COLOR, VAL_YELLOW);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_1, 1);
			}
			if(led_id_index == 2)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_2, ATTR_ON_COLOR, VAL_YELLOW);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_2, 1);
			}
			if(led_id_index == 3)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_3, ATTR_ON_COLOR, VAL_YELLOW);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_3, 1);
			}
			
			led_id_index++;
			if(led_id_index == 4)
				led_id_index = 0;
			//UART CALL
			break;
	}
	return 0;
}

int CVICALLBACK WhiteButton (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			if(led_id_index == 0)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_0, ATTR_ON_COLOR, VAL_WHITE);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_0, 1);
			}
			if(led_id_index == 1)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_1, ATTR_ON_COLOR, VAL_WHITE);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_1, 1);
			}
			if(led_id_index == 2)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_2, ATTR_ON_COLOR, VAL_WHITE);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_2, 1);
			}
			if(led_id_index == 3)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_3, ATTR_ON_COLOR, VAL_WHITE);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_3, 1);
			}
			
			led_id_index++;
			if(led_id_index == 4)
				led_id_index = 0;
			//UART CALL
			break;
	}
	return 0;
}

int CVICALLBACK PurpleButton (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			if(led_id_index == 0)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_0, ATTR_ON_COLOR, VAL_MAGENTA);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_0, 1);
			}
			if(led_id_index == 1)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_1, ATTR_ON_COLOR, VAL_MAGENTA);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_1, 1);
			}
			if(led_id_index == 2)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_2, ATTR_ON_COLOR, VAL_MAGENTA);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_2, 1);
			}
			if(led_id_index == 3)
			{
				SetCtrlAttribute (panelHandle, PANEL_B2L_LED_3, ATTR_ON_COLOR, VAL_MAGENTA);
				SetCtrlVal(panelHandle, PANEL_B2L_LED_3, 1);
			}
			
			led_id_index++;
			if(led_id_index == 4)
				led_id_index = 0;
			//UART CALL
			break;
	}
	return 0;
}
