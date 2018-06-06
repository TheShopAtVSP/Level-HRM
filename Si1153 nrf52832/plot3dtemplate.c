/*----------------------------------------------------------------------------*/
/*                                                                            */
/* FILE:    3DGraph.c                                                         */
/*                                                                            */
/* PURPOSE: This example demonstrates using the Measurement Studio 3D Graph   */
/*          ActiveX control in a CVI panel.  It uses a 3D Surface plot of     */
/*          random sine patterns to demonstrate some of the plot options      */
/*          														          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Include files                                                              */
/*----------------------------------------------------------------------------*/
#include <cviauto.h>
#include "3DGraphCtrl.h"
#include <analysis.h>
#include <cvirte.h>		
#include <userint.h>
#include "3DGraph.h"

static int panelHandle;
static CAObjHandle graphHandle = 0;
static CAObjHandle plotHandle = 0;
static int running = 0;
static int projections = 0;
static int projectionsOnly = 0;

int main (int argc, char *argv[])
{
	CAObjHandle plotsHandle = 0;
	
	if (InitCVIRTE (0, argv, 0) == 0)
		return -1;	/* out of memory */
	if ((panelHandle = LoadPanel (0, "3DGraph.uir", PANEL)) < 0)
		return -1;

	//Get Handle of Graph and first Plot from the ActiveX control
	GetObjHandleFromActiveXCtrl (panelHandle, PANEL_3DGRAPH, &graphHandle);
	CW3DGraphLib__DCWGraph3DGetPlots (graphHandle, NULL, &plotsHandle);

	CW3DGraphLib_CWPlots3DItem (plotsHandle, NULL, CA_VariantInt(1),
								&plotHandle);

	DisplayPanel (panelHandle);
	RunUserInterface ();
	CA_DiscardObjHandle (plotsHandle);
	DiscardPanel (panelHandle);
	return 0;
}

int CVICALLBACK ShowProjections (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			//Toggle Projections Mode
			if(!projections) {
				CW3DGraphLib_CWPlot3DSetProjectionXY (plotHandle, NULL, VTRUE);
				projections = 1;
				return 0;
			}
			CW3DGraphLib_CWPlot3DSetProjectionXY (plotHandle, NULL, VFALSE);
			projections = 0;
			break;
		}
	return 0;
}

int CVICALLBACK ProjectionsOnly (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			//Toggle Graph to Show only Projections
			if(!projectionsOnly) {
				CW3DGraphLib_CWPlot3DSetShowProjectionsOnly (plotHandle, NULL, VTRUE);
				projectionsOnly = 1;
				return 0;
			}
			CW3DGraphLib_CWPlot3DSetShowProjectionsOnly (plotHandle, NULL, VFALSE);
			projectionsOnly = 0;
			break;
		}
	return 0;
}

int CVICALLBACK ChangeStyle (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	int style;
	switch (event)
		{
		case EVENT_COMMIT:
			//Set Plot Style to selected mode
			GetCtrlVal (panelHandle, PANEL_STYLE, &style);
			switch (style)
				{
				case 1:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwLine);
				break;
				case 2:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwPoint);
				break;
				case 3:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwLinePoint);
				break;
				case 4:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwHiddenLine);
				break;
				case 5:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwSurface);
				break;
				case 6:;
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwSurfaceLine);
				break;
				case 7:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwSurfaceNormal);
				break;
				case 8:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwContourLine);
				break;
				case 9:
					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL,
												   CW3DGraphLibConst_cwSurfaceContour);
				break;
			}				
			break;
		}
	return 0;
}

int CVICALLBACK StartPlotting (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			SetCtrlAttribute (panelHandle, PANEL_TIMER, ATTR_ENABLED, 1);
			running = 1;
			break;
		}
	return 0;
}

int CVICALLBACK Stop (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
 			SetCtrlAttribute (panelHandle, PANEL_TIMER, ATTR_ENABLED, 0);
			running = 0;
			break;
		}
	return 0;
}

int CVICALLBACK QuitCallback (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			if(running)
			{
				SetCtrlAttribute (panelHandle, PANEL_TIMER, ATTR_ENABLED, 0);
				ProcessSystemEvents();
			}
			CA_DiscardObjHandle (plotHandle);
			QuitUserInterface (0);
			break;
		}
	return 0;
}

int CVICALLBACK SetTransparency (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	int transparency;
	
	switch (event)
		{
		case EVENT_COMMIT:
			//Set Transparency level of plot
			GetCtrlVal (panelHandle, PANEL_TRANSPARENCY, &transparency);
			CW3DGraphLib_CWPlot3DSetTransparency (plotHandle, NULL, transparency);
			break;
		}
	return 0;
}

int CVICALLBACK Tick (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	double data[20][128];
	int i;
	VARIANT vArray;
	
	switch (event)
		{
		case EVENT_TIMER_TICK:
			//Generate 20 random sine waves for plot surface
			for(i=0;i<20;i++) {
				SinePattern (128, (double)rand()/(double)RAND_MAX, 0.0, 1.0, data[i]);
				ProcessDrawEvents();
			}

			//Put 2D array in VARIANT and plot surface
			CA_VariantSet2DArray (&vArray, CAVT_DOUBLE, 20, 128, &data);
			CW3DGraphLib_CWPlot3DPlot3DSimpleSurface (plotHandle, NULL, vArray,
													  CA_DEFAULT_VAL);
			CA_VariantClear (&vArray);
			break;
			
		}
	return 0;
}
