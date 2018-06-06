/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    AX3dPlot.c                                  			         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Include files                                                             */
/*---------------------------------------------------------------------------*/
#include <windows.h> 
#include <ansi_c.h>
#include <math.h>
#include <cvirte.h>
#include <userint.h>
#include <rs232.h>
#include <utility.h>
#include <formatio.h>
#include <string.h>
//#include <toolbox.h>
#include "toolbox.h"
#include "3DGraphCtrl.h"
#include "cviogl.h"
#include <analysis.h>
#include "Activity Discriminator vars.h" 
#include "Activity Discriminator.h"



/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
//static CAObjHandle      gSurfaceGraph;
CAObjHandle      gSurfaceGraph;
CAObjHandle 	 plot3dHandle = 0;
CAObjHandle 	 plot3dHandle_org = 0;
CAObjHandle      gAxis;
CAObjHandle      axes = 0;

int              gEditingLevels,
                 gEditingInterval,
                 gEditingAnchor,
                 gRefreshing;
int              gColorTable[] = { 0, 0x0000FFL, 0x00FF00L, 0xFF0000L }; 

VARIANT 		vxArray, vyArray, vzArray;




/*****************************************************************************/
/* AX3dPlot                                                                  */
/*****************************************************************************/
void AX3dPlot(void)
{
	int    		err, acc_plot3d_sel, gyr_plot3d_sel, autoscale_plot3d_sel;
	char 		str_tmp[128];
	double		xmin=0, xmax=0, ymin=0, ymax=0, zmin=0, zmax=0;
	int			minpos, maxpos;
	

	GetCtrlVal(mainpnl, MAINPNL_PLOT3D_ACC_ON, &acc_plot3d_sel);
	if(acc_plot3d_sel)
	{
		err = GetObjHandleFromActiveXCtrl(mainpnl, MAINPNL_GRAPH3D, &gSurfaceGraph);
		if(err == 0)
		{
			err = CW3DGraphLib__DCWGraph3DGetPlots (gSurfaceGraph, NULL, &plot3dHandle_org);
			if(err == 0)
				CW3DGraphLib_CWPlots3DItem (plot3dHandle_org, NULL, CA_VariantInt(1), &plot3dHandle);
			CW3DGraphLib_CWPlot3DSetStyle (plot3dHandle, NULL, CW3DGraphLibConst_cwPoint);
			CW3DGraphLib_CWPlot3D_CISetPointColor (plot3dHandle, NULL, VAL_BLUE);
		}
		
		// Note: The XYZ is rotated in the visuization plane so Y2=Z3(up/down) X1=X1(frwd/bck) Z3=Y2(in/out).
		CW3DGraphLib__DCWGraph3DGetAxes(gSurfaceGraph, NULL, &axes);
		CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(1), &gAxis);
		strcpy(str_tmp, "Accel X"); CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
		CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(3), &gAxis);
		strcpy(str_tmp, "Accel Y"); CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
		CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(2), &gAxis);
		strcpy(str_tmp, "Accel Z"); CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
	
		CA_VariantSet1DArray (&vxArray, CAVT_DOUBLE, ser_input_size, &intgtr_acl_x);
		CA_VariantSet1DArray (&vyArray, CAVT_DOUBLE, ser_input_size, &intgtr_acl_y);
		CA_VariantSet1DArray (&vzArray, CAVT_DOUBLE, ser_input_size, &intgtr_acl_z);
																									    
		CW3DGraphLib_CWPlot3DPlot3DCurve(plot3dHandle, NULL, vyArray, vzArray, vxArray, CA_DEFAULT_VAL);
	
		CA_VariantClear (&vxArray);
		CA_VariantClear (&vyArray);
		CA_VariantClear (&vzArray);
	}
	
	GetCtrlVal(mainpnl, MAINPNL_PLOT3D_GYRO_ON, &gyr_plot3d_sel);
	if(gyr_plot3d_sel)
	{
	///	if(!acc_plot3d_sel)
	///	{
			err = GetObjHandleFromActiveXCtrl(mainpnl, MAINPNL_GRAPH3D, &gSurfaceGraph);
			if(err == 0)
			{
				err = CW3DGraphLib__DCWGraph3DGetPlots (gSurfaceGraph, NULL, &plot3dHandle_org);
				if(err == 0)
					CW3DGraphLib_CWPlots3DItem (plot3dHandle_org, NULL, CA_VariantInt(1), &plot3dHandle);
				CW3DGraphLib_CWPlot3DSetStyle (plot3dHandle, NULL, CW3DGraphLibConst_cwPoint); 
				CW3DGraphLib_CWPlot3D_CISetPointColor (plot3dHandle, NULL, VAL_RED);
			}
	///	}
	
		// Note: The XYZ is rotated in the visuization plane so Y2=Z3(up/down) X1=X1(frwd/bck) Z3=Y2(in/out).
		CW3DGraphLib__DCWGraph3DGetAxes(gSurfaceGraph, NULL, &axes);
		CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(1), &gAxis);
		strcpy(str_tmp, "Gyro Pitch"); CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
		CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(3), &gAxis);
		strcpy(str_tmp, "Gyro Yaw"); CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
		CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(2), &gAxis);
		strcpy(str_tmp, "Gyro Roll"); CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
	
		CA_VariantSet1DArray (&vxArray, CAVT_DOUBLE, ser_input_size, &intgtr_gyr_p);
		CA_VariantSet1DArray (&vyArray, CAVT_DOUBLE, ser_input_size, &intgtr_gyr_y);
		CA_VariantSet1DArray (&vzArray, CAVT_DOUBLE, ser_input_size, &intgtr_gyr_r);

		CW3DGraphLib_CWPlot3DPlot3DCurve(plot3dHandle, NULL, vyArray, vzArray, vxArray, CA_DEFAULT_VAL);
	
		CA_VariantClear (&vxArray);
		CA_VariantClear (&vyArray);
		CA_VariantClear (&vzArray);
	}
	
	GetCtrlVal(mainpnl, MAINPNL_PLOT3D_SCALE_ON, &autoscale_plot3d_sel);
	if(!autoscale_plot3d_sel)
	{
		if(acc_plot3d_sel || gyr_plot3d_sel)
		{
			if(acc_plot3d_sel)
			{
				MaxMin1D (intgtr_acl_x, ser_input_size, &xmax, &minpos, &xmin, &maxpos);
				MaxMin1D (intgtr_acl_y, ser_input_size, &ymax, &minpos, &ymin, &maxpos);
				MaxMin1D (intgtr_acl_z, ser_input_size, &zmax, &minpos, &zmin, &maxpos);
			}
			if(gyr_plot3d_sel)
			{
				MaxMin1D (intgtr_gyr_p, ser_input_size, &xmax, &minpos, &xmin, &maxpos);
				MaxMin1D (intgtr_gyr_y, ser_input_size, &ymax, &minpos, &ymin, &maxpos);
				MaxMin1D (intgtr_gyr_r, ser_input_size, &zmax, &minpos, &zmin, &maxpos);
			}
			if(ymin<xmin) xmin=ymin;
			if(zmin<xmin) xmin=zmin;
			if(ymax>xmax) xmax=ymax;
			if(zmax>xmax) xmax=zmax;
		
			CW3DGraphLib__DCWGraph3DGetAxes(gSurfaceGraph, NULL, &axes);
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(1), &gAxis);
			CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VFALSE);
			CW3DGraphLib_CWAxis3DSetMinimum (gAxis, NULL, CA_VariantInt((int)(floor(xmin))));
			CW3DGraphLib_CWAxis3DSetMaximum (gAxis, NULL, CA_VariantInt((int)(ceil(xmax))));
		
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(2), &gAxis);
			CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VFALSE);
			CW3DGraphLib_CWAxis3DSetMinimum (gAxis, NULL, CA_VariantInt((int)(floor(xmin))));
			CW3DGraphLib_CWAxis3DSetMaximum (gAxis, NULL, CA_VariantInt((int)(ceil(xmax))));
		
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(3), &gAxis);
			CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VFALSE);
			CW3DGraphLib_CWAxis3DSetMinimum (gAxis, NULL, CA_VariantInt((int)(floor(xmin))));
			CW3DGraphLib_CWAxis3DSetMaximum (gAxis, NULL, CA_VariantInt((int)(ceil(xmax))));
		}
		else
		{
			CW3DGraphLib__DCWGraph3DGetAxes(gSurfaceGraph, NULL, &axes);
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(1), &gAxis);
			CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);	
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(2), &gAxis);
			CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(3), &gAxis);
			CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
		}
	}
}



int CVICALLBACK Plot_3D_CB (int panel, int control, int event,
							void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			AX3dPlot();
			break;
	}
	return 0;
}

int CVICALLBACK GYR_Plot_3D_CB (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			SetCtrlVal(mainpnl, MAINPNL_PLOT3D_ACC_ON, 0);
			AX3dPlot();
			break;
	}
	return 0;
}

int CVICALLBACK ACC_Plot_3D_CB (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			SetCtrlVal(mainpnl, MAINPNL_PLOT3D_GYRO_ON, 0);
			AX3dPlot();
			break;
	}
	return 0;
}
