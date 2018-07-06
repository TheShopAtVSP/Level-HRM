//==============================================================================
//
// Title:		Level_PC_Main
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
char target_list[TARGET_LIST_LENGTH][TARGET_LIST_NAME_MAX_LENGTH] = {
    "Level 7A5B"
    };
//==============================================================================
// Global functions

/// The main entry-point function.
int main (int argc, char *argv[])
{
	uint32_t error_code;
    char* serial_port;
	
	/* initialize and load resources */
	if (InitCVIRTE (0, argv, 0) == 0)
        return -1;
	if ((panelHandle = LoadPanel (0, "Level_PC.uir", PANEL)) < 0)
        return -1;
	
	/* display the panel and run the user interface */
	SetSleepPolicy (VAL_SLEEP_NONE);
	DisplayPanel (panelHandle);
	
	// Init and main loop
	led_id_index = 0;
	
	intro_message_print();

    serial_port = UART_PORT_NAME;
    sprintf(Console_str, "Serial port used: %s\n", serial_port); 
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    sd_rpc_serial_port_name_set(serial_port);
    sd_rpc_serial_baud_rate_set(115200);
    sd_rpc_log_handler_set(log_handler);
    sd_rpc_evt_handler_set(ble_evt_dispatch);
    error_code = sd_rpc_open(); // can hang here. Hit reset if it does.
	sprintf(Console_str, "Past sd_rpc_open().\n");
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    if (error_code != NRF_SUCCESS)
    {
        sprintf(Console_str, "Failed to open the nRF51 ble driver.\n");
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
        return error_code;
    }

    error_code = ble_stack_init();
    if (error_code != NRF_SUCCESS)
        return error_code;

    scan_start();
	
	RunUserInterface();
	
	return 0;
}


//=============================== Functions ====================================

void intro_message_print()
{
    int i;

    sprintf(Console_str, "Level PC Interface.\n\n");
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    sprintf(Console_str, "The Master Device is using jrcp on port %s.\n", UART_PORT_NAME);
	SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    sprintf(Console_str, "This program will connect to devices with the following names:\n");
    SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);

    for (i = 0; i < TARGET_LIST_LENGTH; ++i)
    {
        sprintf(Console_str, "  * %s\n", target_list[i]); 
		SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
    }
}


//==============================================================================
// UI callback function prototypes

/// HIFN Exit when the user dismisses the panel.
int CVICALLBACK panelCB (int panel, int event, void *callbackData,
		int eventData1, int eventData2)
{
	int error_code;
	
	/* clean up and exit */
	if (event == EVENT_CLOSE)
	{
		error_code = sd_ble_gap_scan_stop();
	    if (error_code != NRF_SUCCESS)
	    {
			sprintf(Console_str, "Failed to stop scanning. Reason: %d\n", error_code); 
			SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
	    }
		
		//error_code = sd_ble_gap_disconnect(uint16_t conn_handle, uint8_t hci_status_code));
		//sprintf(Console_str, "Failed to enable BLE stack.\n"); 
		//SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
	
		error_code = sd_rpc_close();
	    if (error_code != NRF_SUCCESS)
	    {
			sprintf(Console_str, "Failed to close the nRF51 ble driver.\n"); 
			SetCtrlVal(panelHandle, PANEL_CONSOLE, Console_str);
	        return error_code;
	    }
	
		if (panelHandle > 0)
			DiscardPanel (panelHandle);
	
		QuitUserInterface (0);
	}
	    
	return 0;
}

int CVICALLBACK Select_Device (int panel, int control, int event,
							   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:

			break;
	}
	return 0;
}


