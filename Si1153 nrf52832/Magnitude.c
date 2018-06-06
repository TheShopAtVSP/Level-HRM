/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Magnitude.c                                  			         */
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

void Get_Manitude(int source_sel)
{
	int i;
	
	if(source_sel == 0)
	{
		for(i=0; i<(ser_input_size + 64); i++)
		{
			acc_mag[i] = (filt_acl_x[i] * filt_acl_x[i])\
						+(filt_acl_y[i] * filt_acl_y[i])\
						+(filt_acl_z[i] * filt_acl_z[i]);
			acc_mag[i] = sqrt(acc_mag[i]);
		
			gyr_mag[i] = (filt_gyr_p[i] * filt_gyr_p[i])\
						+(filt_gyr_y[i] * filt_gyr_y[i])\ 
						+(filt_gyr_r[i] * filt_gyr_r[i]);
			gyr_mag[i] = sqrt(gyr_mag[i]);
		}
	}
	
	if(source_sel == 1)
	{
		for(i=0; i<(ser_input_size + 64); i++)
		{
			acc_mag[i] = (intgtr_acl_x[i] * intgtr_acl_x[i])\
						+(intgtr_acl_y[i] * intgtr_acl_y[i])\
						+(intgtr_acl_z[i] * intgtr_acl_z[i]);
			acc_mag[i] = sqrt(acc_mag[i]);
		
			gyr_mag[i] = (intgtr_gyr_p[i] * intgtr_gyr_p[i])\
						+(intgtr_gyr_y[i] * intgtr_gyr_y[i])\ 
						+(intgtr_gyr_r[i] * intgtr_gyr_r[i]);
			gyr_mag[i] = sqrt(gyr_mag[i]);
		}
	}
}
