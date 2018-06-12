/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Position Integrator.c                                  			         */
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


int CVICALLBACK Position_Integrator_CB (int panel, int control, int event,
										void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			Apply_Position_Integrator();
			break;
	}
	return 0;
}


void Apply_Position_Integrator(void)
{
	char 	out_str[64];
	int 	i, raw_intgtr_sel, ss_integrator_sel, integ_span, run_auto_p2sf, pmaxi, pmini;
	double	acc_integ_gain, gyr_integ_gain, pvariance, last_pvariance , pmean, pmax, pmin;
	double 	subtractor;		   
	int		polynomial_order, frame_size; 
	
	GetCtrlVal(mainpnl, MAINPNL_POS_INTGTR_SEL, &raw_intgtr_sel); 
	GetCtrlVal(mainpnl, MAINPNL_ACC_SSI, &acc_integ_gain);
	GetCtrlVal(mainpnl, MAINPNL_GYR_SSI, &gyr_integ_gain);  
	GetCtrlVal(mainpnl, MAINPNL_INT_SPAN, &integ_span);
	if(integ_span <= 0) integ_span = 0.01;
	
	// Init intgrator array
	for(i=0; i<(ser_input_size + 64); i++)
	{
		intgtr_acl_x[i] = filt_acl_x[i];
		intgtr_acl_y[i] = filt_acl_y[i];
		intgtr_acl_z[i] = filt_acl_z[i];
		intgtr_gyr_p[i] = filt_gyr_p[i];
		intgtr_gyr_y[i] = filt_gyr_y[i];
		intgtr_gyr_r[i] = filt_gyr_r[i];
	}
	
	
	// SS Self Steady
	GetCtrlVal(mainpnl, MAINPNL_SS_INTEGRATOR, &ss_integrator_sel);
//	if(((raw_intgtr_sel == 1) || (raw_intgtr_sel == 2)) && ss_integrator_sel)
	{
		running_avg_acc_x = running_avg_acc_y = running_avg_acc_z = 0;
		running_avg_gyr_p = running_avg_gyr_y = running_avg_gyr_r = 0; 
		for(i=0; i<(ser_input_size + 64); i++)
		{  
			running_avg_acc_x += filt_acl_x[i];
			running_avg_acc_y += filt_acl_y[i];
			running_avg_acc_z += filt_acl_z[i];
			running_avg_gyr_p += filt_gyr_p[i];
			running_avg_gyr_y += filt_gyr_y[i];
			running_avg_gyr_r += filt_gyr_r[i];
			
		}
	
		running_avg_acc_x /= ((double)(ser_input_size + 64) * acc_integ_gain);
		running_avg_acc_y /= ((double)(ser_input_size + 64) * acc_integ_gain);
		running_avg_acc_z /= ((double)(ser_input_size + 64) * acc_integ_gain);
		running_avg_gyr_p /= ((double)(ser_input_size + 64) * gyr_integ_gain);
		running_avg_gyr_y /= ((double)(ser_input_size + 64) * gyr_integ_gain);
		running_avg_gyr_r /= ((double)(ser_input_size + 64) * gyr_integ_gain);
	
		// ============================================================================
		if( ser_input_size == 0)
			return;
		
//		Variance ( intgtr_gyr_p, ser_input_size ,&pvariance , &pmean );
		//sprintf(out_str, "pvariance %f, pmean %f\n", pvariance, pmean);
		//SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
		
///		if(fabs(pvariance) < 1.0)
		if(0)
		{
			for(i=0; i<(ser_input_size); i++)
			{
				intgtr_gyr_p[i] = 0.0 ;
			}
		}
		else
		{
			
			for(i=0; i<(ser_input_size ); i++)
			{
				intgtr_gyr_r[i] = filt_gyr_y[i];
				intgtr_gyr_y[i] = filt_gyr_y[i]; 
			}
			
			
#if 0    /* formerly excluded lines */
			GetCtrlVal(mainpnl, MAINPNL_SG_PO, &polynomial_order);
			GetCtrlVal(mainpnl, MAINPNL_SG_FS, &frame_size);
			SavitzkyGolayFiltering (filt_gyr_y, ser_input_size, polynomial_order, frame_size, NULL, intgtr_gyr_r);
			
			
			for(i=0; i<(ser_input_size ); i++)
			{
				intgtr_gyr_y[i] -= intgtr_gyr_r[i];  /// -=
			}
#endif   /* formerly excluded lines */
			
#if 0    /* formerly excluded lines */
			for(i=0; i<(ser_input_size ); i++)
			{
				intgtr_gyr_r[i] = filt_gyr_p[i];
				intgtr_gyr_p[i] = filt_gyr_p[i]; 
			}
			
			
			GetCtrlVal(mainpnl, MAINPNL_SG_PO, &polynomial_order);
			GetCtrlVal(mainpnl, MAINPNL_SG_FS, &frame_size);
			SavitzkyGolayFiltering (filt_gyr_p, ser_input_size, polynomial_order, frame_size, NULL, intgtr_gyr_r);
			
			
			for(i=0; i<(ser_input_size ); i++)
			{
				intgtr_gyr_p[i] -= intgtr_gyr_r[i]; 
			}
#endif   /* formerly excluded lines */
			
			
			/// SQR Geo Desimation ///
			for(i=0; i<ser_input_size; i++)
			{
				if(sqra_on)
				{
					if(intgtr_gyr_p[i] < 0) 
					{
						intgtr_gyr_p[i] = -intgtr_gyr_p[i];
						intgtr_gyr_p[i] = sqrt(intgtr_gyr_p[i]);
						intgtr_gyr_p[i] = -intgtr_gyr_p[i];	
					}
					else
					{
						intgtr_gyr_p[i] = sqrt(intgtr_gyr_p[i]);	
					}
				}
	
				if(sqrb_on)
				{
					if(intgtr_gyr_y[i] < 0) 
					{
						intgtr_gyr_y[i] = -intgtr_gyr_y[i];
						intgtr_gyr_y[i] = sqrt(intgtr_gyr_y[i]);
						intgtr_gyr_y[i] = -intgtr_gyr_y[i];	
					}
					else
					{
						intgtr_gyr_y[i] = sqrt(intgtr_gyr_y[i]);	
					}
				}
			}
			
			/// Final Cleanup before Chan A B scaled Subtraction ///
			for(i=0; i<(ser_input_size ); i++)
			{
				///intgtr_acl_x[i] -= running_avg_acc_x;
				///intgtr_acl_y[i] -= running_avg_acc_y;
				///intgtr_acl_z[i] -= running_avg_acc_z;
				//intgtr_gyr_p[i] -= running_avg_gyr_p;
			//	intgtr_gyr_r[i] = filt_gyr_p[i] - (filt_gyr_y[i] * phase2sf) ;
				
				//AutoCorrelate (x, n, ALGORITHM_CORCOR_NO_NORMALIZATION, rxx);
///////				AutoCorrelate (intgtr_gyr_p, ser_input_size, ALGORITHM_CONCOR_BIASED_NORMALIZATION, intgtr_gyr_r);
///////				NormalizedCorrelate (filt_gyr_p, ser_input_size, filt_gyr_y, ser_input_size, ALGORITHM_CONCOR_FREQ_DOMAIN, ALGORITHM_CONCOR_BIASED_NORMALIZATION, intgtr_gyr_r);
				
				
				//intgtr_gyr_y[i] -= running_avg_gyr_y;
				//intgtr_gyr_r[i] -= running_avg_gyr_r;
				gyr_tmp[i] = intgtr_gyr_p[i];
			}
		
			/// Auto Subtract ///
			GetCtrlVal(mainpnl, MAINPNL_AUTO_P2SF, &run_auto_p2sf);
			/// if(run_auto_p2sf)
			if(0)
			{
				phase2sf = 0.0;
				phase2sfcof = 0.5;
				for(int ol=0;  ol < 3; ol++)
				{
					if(ol==1)
						phase2sfcof = 0.05;
					if(ol==2)
						phase2sfcof = 0.005;
						
					last_pvariance = 1000000.0;	 
//					MaxMin1D ( intgtr_gyr_p, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
//					pvariance = pmax - pmin;
//					MaxMin1D ( intgtr_gyr_y, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
//					phase2sf = fabs(pvariance / (pmax - pmin));
					/// acually, it might be better to do seperate max and min ratios and apply the largest. whatch out for signs too...
					while(fabs(last_pvariance) > fabs(pvariance))
					{
						last_pvariance = pvariance;
						phase2sf += phase2sfcof;
						SetCtrlVal(mainpnl, MAINPNL_PHASE2SF, phase2sf);
					
						for(i=0; i<(ser_input_size ); i++)
						{

							//intgtr_gyr_p[i] = filt_gyr_p[i] - (filt_gyr_y[i] * phase2sf) ;
							  intgtr_gyr_p[i] = gyr_tmp[i] - (filt_gyr_y[i] * phase2sf) ;
						}
					
						MaxMin1D ( intgtr_gyr_p, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
						pvariance = pmax;/// - pmin;
						sprintf(out_str, "+pvariance %f, last_pvariance %f\n", pvariance, last_pvariance);
						SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
					}
				
					last_pvariance = 1000000.0;
					///phase2sf -= 0.1;
					while(fabs(last_pvariance) > fabs(pvariance))
					{
						last_pvariance = pvariance;
						phase2sf -= phase2sfcof;
						SetCtrlVal(mainpnl, MAINPNL_PHASE2SF, phase2sf);
					
						for(i=0; i<(ser_input_size ); i++)
						{

							//intgtr_gyr_p[i] = filt_gyr_p[i] - (filt_gyr_y[i] * phase2sf) ;
							intgtr_gyr_p[i] = gyr_tmp[i] - (filt_gyr_y[i] * phase2sf) ;
						}
					
						MaxMin1D ( intgtr_gyr_p, ser_input_size, &pmax , &pmaxi, &pmin, &pmini );
						pvariance = pmax;/// - pmin;
						sprintf(out_str, "-pvariance %f, last_pvariance %f\n", pvariance, last_pvariance);
						SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
					}
				}
			}
//			else
//			{
//				for(i=0; i<(ser_input_size ); i++)
//				{
//
//					/// intgtr_gyr_p[i] = intgtr_gyr_p[i] - (intgtr_gyr_y[i] * phase2sf) ;
//					//  intgtr_gyr_p[i] = gyr_tmp[i] - (filt_gyr_y[i] * phase2sf) ;
//				}
//			}
		}
	}
	
	// Basic Integrator 
	if(raw_intgtr_sel == 1) 
	{
		Basic_Integrate(intgtr_acl_x,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_x);
		Basic_Integrate(intgtr_acl_y,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_y);
		Basic_Integrate(intgtr_acl_z,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_z);
		//Basic_Integrate(intgtr_gyr_p,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_p);
		//Basic_Integrate(intgtr_gyr_y,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_y);  
		//Basic_Integrate(intgtr_gyr_r,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_r);  
	}
	
	// Simpson Integrator
	if(raw_intgtr_sel == 2) 
	{
		Simpson_Integrate(intgtr_acl_x,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_x);
		Simpson_Integrate(intgtr_acl_y,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_y);
		Simpson_Integrate(intgtr_acl_z,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_acl_z);
		//Simpson_Integrate(intgtr_gyr_p,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_p);
		//Simpson_Integrate(intgtr_gyr_y,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_y);  
		//Simpson_Integrate(intgtr_gyr_r,(ser_input_size + 64) , integ_span, 0.0, 0.0, intgtr_gyr_r);  
	}
	
	Apply_Peak_Detector();
}

  
void Basic_Integrate(const double x[], ssize_t n, int dt, double xInit, double xFinal, double y[])
{
	double 	temp_sum = 0.0;	
  	int 	i;
  
	dbl_tmp[0] = temp_sum = xInit;
	for(i=1;i<n;i++)
	{
		temp_sum += x[i];
		dbl_tmp[i] = temp_sum;
	}
	dbl_tmp[i] = temp_sum + xFinal;
	
	y[0] = dbl_tmp[0];
	for(i=1;i<n;i++)
	{
		y[i] = dbl_tmp[i];
	}
}	


 // Y[i] = SUM[j=0:i] ( X[j-1]  + 4X[j] +X[j+1] ) * dt/6
 // Integrate (double Input_Array[], ssize_t Number_of_Elements, double Sampling_Interval, double Initial_Condition, double Final_Condition, double Output_Array[]);
void Simpson_Integrate(const double x[], ssize_t n, int dt, double xInit, double xFinal, double y[])
{
	double 	temp_sum = 0.0;	
  	int 	i, j;
  
	dbl_tmp[0] = temp_sum = xInit;
	for(i=1;i<n+10;i+=dt)
	{
		temp_sum += ( x[i-1]  + 4*x[i] +x[i+1] ) * (double)dt/6;
		dbl_tmp[i] = temp_sum;
	}
	dbl_tmp[i] = temp_sum + xFinal;
	
	y[0] = dbl_tmp[0];
	for(i=1;i<n+10;i+=dt)
	{
		for(j=0;j<dt;j++)
		{
			y[i + j] = dbl_tmp[i];
		}
	}
}


int CVICALLBACK Phase2ScaleFactor (int panel, int control, int event,
								   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(mainpnl, MAINPNL_PHASE2SF, &phase2sf);
			Apply_Position_Integrator(); 
			break;
	}
	return 0;
}
