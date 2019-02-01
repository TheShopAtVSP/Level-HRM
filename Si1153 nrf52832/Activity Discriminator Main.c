/*****************************************************************/
/*                                                               */
/*  Activity Discriminator					                     */
/*  WMM                                         				 */
/*  Feb, 2016                                                    */
/*                                                               */
/*****************************************************************/
#include <windows.h>
#include <rs232.h>
#include <GL\gl.h>
#include <GL\glu.h>
#include "3DGraphCtrl.h"
#include <utility.h>
#include <ansi_c.h>
#include <cvirte.h>     
#include <analysis.h>
#include <userint.h>
#include <inifile.h>
#include "cviogl.h"
#include "Activity Discriminator.h"
#include "Activity Discriminator vars.h"

/*****************************************************************/
/*                                                               */
/*  Main         												 */
/*                                                               */
/*****************************************************************/

int main (int argc, char *argv[])
{
	int fileError;
	char com_str[8]; 
	
    if (InitCVIRTE (0, argv, 0) == 0)    
        return -1;    /* out of memory */
    
    if ((mainpnl = LoadPanel (0, "Activity Discriminator.uir", MAINPNL)) < 0)
        return -1;

/*****************************************************************/
/* Inits and Setups                                          	 */
/*****************************************************************/	
	
	CheckListItem (mainpnl, MAINPNL_PD_PLOT, 3, 1);
	
	// create object for holding the value/tag pairs
	IniText iniText = Ini_New (TRUE);	// TRUE for automatic sorting 
	// read in the tag/value pairs 
	fileError = Ini_ReadFromFile (iniText, "./myconfig.ini"); 
	if( fileError < 0 )
	{   //File Not read
		return -1;
	}
	// create the in–memory tag/value pairs
	fileError = Ini_GetInt (iniText, "section 1", "com_port", &ser_com_port); 
	if( fileError < 0 )
	{   //Value Not read
		return -1;
	}
	// dispose of the in–memory tag/value pairs
	Ini_Dispose (iniText);
	
	// check that the com port makes sense:
	if( ser_com_port > 99 || ser_com_port < 0 )
	{	// Com port out of bounds
		return -1;	
	}
	sprintf( com_str, "COM%01u", ser_com_port );
	RS232Error = OpenComConfig (ser_com_port, com_str, 115200, 0, 8, 1, 16, 16); 
	
	selfsteadyhasrun = 0;
	self_steady = 0;
	ssax = ssay = ssaz = 0;
	ssgp = ssgy = ssgr = 0;
	raw_hart_cir_buff_idx = data_stream_cir_buff_idx = 0;
	GetCtrlVal(mainpnl, MAINPNL_NRTS, &run_time); 
	aquisition_start_time = Timer();
	GetCtrlVal(mainpnl, MAINPNL_PHASE2SF, &phase2sf);
	sqra_on = sqrb_on = 0;
	
	autoGenerateGraphs = 1;
	gv_firstSample = 1;
	gv_acc_plot_index_old = 0;
	pd_peak_cnt = 0;
	ggrdavgindex = 0;
	glasses_on = 0;
	aquisition_add_time = 0;
	stitch_rear_edge_dect = 0;
	total_run_time = total_peaks = last_data_stream_cir_buff_idx = look_back_starting_point = 0;
	final_hrm_avg_index = 0;
	fo_start = 1;
	fo_hit = fo_miss = 0;
	hrm_raw_index = 0;
	current_hrm = 69;
	memset(last4hrmavg, 71, 8);
	for(int i = 0; i < 8; i++)
		last4hrmavg[i] = 71;
	last4hrmavgidx = 0;
	

//	GetScreenSize (NULL, &screen_width);
//	if(screen_width == 1600)
//		SetPanelSize (mainpnl, 800, 1600);
	
	// Convert the CVI picture control to an OGL control
    ///OGLControlPanel = OGLConvertCtrl (mainpnl, MAINPNL_GV_IMAGE);
    // Initialize the OGL control
    ///InitOGLControl();
	///OGLRefreshGraph(mainpnl, OGLControlPanel);
	
	
    // For HB Testing
    //if (autoGenerateGraphs) GenerateGraphs();
    DisplayPanel (mainpnl);
    RunUserInterface ();
    DiscardPanel (mainpnl);
    return 0;
}


/*****************************************************************/
/* Quit Button Callback                                          */
/*****************************************************************/
int CVICALLBACK Quit (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            QuitUserInterface (0);
            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Quit Program", "Quits the program.");

            break;
    }
    return 0;
}


/*****************************************************************/
/* Help Button Callback                                          */
/*****************************************************************/
int CVICALLBACK HelpCallback (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_RIGHT_CLICK:
        case EVENT_COMMIT:
            MessagePopup ("Convolve and Deconvolve Example Program",HELP_MSG);
        break;
    }
    return 0;
}


