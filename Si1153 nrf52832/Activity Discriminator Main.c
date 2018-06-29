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
	int screen_width;
	
    if (InitCVIRTE (0, argv, 0) == 0)    
        return -1;    /* out of memory */
    
    if ((mainpnl = LoadPanel (0, "Activity Discriminator.uir", MAINPNL)) < 0)
        return -1;

/*****************************************************************/
/* Inits and Setups                                          	 */
/*****************************************************************/	
	
	CheckListItem (mainpnl, MAINPNL_PD_PLOT, 3, 1);
	ser_com_port =6;
	OpenComConfig (ser_com_port, "COM6", 115200, 0, 8, 1, 16, 16);
	//ser_com_port =3;
	//OpenComConfig (ser_com_port, "COM3", 115200, 0, 8, 1, 16, 16);
	//ser_com_port =7;
	//OpenComConfig (ser_com_port, "COM7", 115200, 0, 8, 1, 16, 16);
	//ser_com_port =8;
	//OpenComConfig (ser_com_port, "COM8", 115200, 0, 8, 1, 16, 16);
//	ComWrtByte (ser_com_port, 0x56);
//	ComWrtByte (ser_com_port, 0);
	
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


