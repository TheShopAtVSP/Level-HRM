/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Self Steady.c                                  			         */
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


int CVICALLBACK Self_Steady_CB (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			
		//	Self_Steady();

			break;
	}
	return 0;
}
/*
void Self_Steady(void)
{
	int i, numb_ss_samp;
	char out_str[256];
	double ssax_accm = 0, ssay_accm = 0, ssaz_accm = 0;
	double ssgp_accm = 0, ssgy_accm = 0, ssgr_accm = 0;
	
	ssax = ssay = ssaz = 0;
	ssgp = ssgy = ssgr = 0;
	self_steady = 1;
	Uno_Scan_Loop();
	self_steady = 0;
	
	GetCtrlVal(mainpnl, MAINPNL_SS_SAMP_NUMB, &numb_ss_samp); 
	
	for(i=0; i<numb_ss_samp; i++)
	{
		ssax_accm += raw_acl_x[i];
		ssay_accm += raw_acl_y[i];
		ssaz_accm += raw_acl_z[i];
		ssgp_accm += raw_gyr_p[i];
		ssgy_accm += raw_gyr_y[i];
		ssgr_accm += raw_gyr_r[i];
		
	}
	
	ssax = ssax_accm / numb_ss_samp;
	ssay = ssay_accm / numb_ss_samp;
	ssaz = ssaz_accm / numb_ss_samp;
	ssgp = ssgp_accm / numb_ss_samp;
	ssgy = ssgy_accm / numb_ss_samp;
	ssgr = ssgr_accm / numb_ss_samp;
	
	sprintf(out_str, "ssax %f, ssay %f, ssaz %f\n", ssax, ssay, ssaz);
	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	sprintf(out_str, "ssgp %f, ssgy %f, ssgr %f\n\n", ssgp, ssgy, ssgr);
	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
}
 */
