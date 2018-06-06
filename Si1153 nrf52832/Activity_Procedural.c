/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Activity_Procedural.c                                  			         */
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


void Determine_Activity(void)
{
	double 		mean_val, min_val, max_val;
	int			max_pos, min_pos;
	char		tmp_str[64];
	
	
	if( pd_peak_cnt == 0) return;
	
	//Mean (filt_acl_z, ser_input_size, &mean_val);
	MaxMin1D (filt_acl_z, (ser_input_size), &max_val, &max_pos, &min_val, &min_pos);
	mean_val = max_val - min_val;
	sprintf(tmp_str, "mean_val %f\n", mean_val);
	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, tmp_str);
	
	if(mean_val < 5000)
	{
		SetCtrlVal(mainpnl, MAINPNL_LED_RESTING, ON);
		SetCtrlVal(mainpnl, MAINPNL_LED_WALKING, OFF);
		SetCtrlVal(mainpnl, MAINPNL_LED_RUNNING, OFF);
		///AUTO_P2SF
	//	SetCtrlVal(mainpnl, MAINPNL_AUTO_P2SF, 0);
		///PHASE2SF
	//	SetCtrlVal(mainpnl, MAINPNL_PHASE2SF, 0.0);
	}
	
	if( (mean_val >= 5000) && (mean_val < 1000) )
	{
		SetCtrlVal(mainpnl, MAINPNL_LED_RESTING, OFF);
		SetCtrlVal(mainpnl, MAINPNL_LED_WALKING, ON);
		SetCtrlVal(mainpnl, MAINPNL_LED_RUNNING, OFF);
	//	SetCtrlVal(mainpnl, MAINPNL_AUTO_P2SF, 1); 
	}
	
	if(mean_val >= 10000)
	{
		SetCtrlVal(mainpnl, MAINPNL_LED_RESTING, OFF);
		SetCtrlVal(mainpnl, MAINPNL_LED_WALKING, OFF);
		SetCtrlVal(mainpnl, MAINPNL_LED_RUNNING, ON);
	//	SetCtrlVal(mainpnl, MAINPNL_AUTO_P2SF, 1);
	}
}

