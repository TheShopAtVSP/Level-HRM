/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Sig Generatior.c                                  			     */
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


static double hbphase;
static double heart_beat[1016] = {93,93,94,94,94,93,94,93,92,90,91,96,114,153,223,\
298,411,537,667,794,909,977,1000,1013,1019,1021,1021,1016,998,965,919,874,815,755,693,634,575,518,461,416,364,316,274,242,222,211,210,217,225,240,257,274,\
288,302,311,317,316,314,308,299,285,270,255,241,229,219,207,195,182,170,159,152,147,142,138,135,131,129,126,123,121,121,119,119,119,118,116,113,109,108,108,\
119,146,196,274,379,502,631,757,850,947,992,1010,1017,1020,1021,1015,999,972,927,872,812,748,685,622,560,509,448,390,337,292,258,233,221,219,222,232,247,\
262,279,295,312,326,331,334,331,321,310,295,279,261,242,227,211,197,185,174,165,158,152,145,140,136,133,127,123,119,117,116,115,115,116,119,120,120,119,115,\
112,111,117,133,168,228,317,429,552,653,778,892,970,1001,1014,1019,1021,1020,1015,996,963,914,856,794,733,673,626,568,512,457,405,359,319,285,263,244,\
236,236,243,254,268,282,297,305,315,321,325,324,316,303,286,270,252,236,221,207,192,176,162,148,139,129,122,116,115,113,112,109,108,105,104,104,106,107,\ 
108,109,112,113,115,115,113,111,108,107,109,121,140,185,257,360,485,620,758,889,959,997,1012,1018,1021,1022,1022,1020,1009,989,950,896,837,778,719,662,607,\ 
563,509,456,405,357,316,283,261,246,240,238,243,252,263,275,286,293,295,295,292,288,281,270,259,246,230,216,199,183,167,154,143,134,127,121,114,108,102,\ 
95,88,82,77,74,72,73,76,78,82,84,89,92,96,100,101,103,104,101,98,97,98,102,119,157,225,323,444,575,711,816,930,985,1007,1016,1020,1022,1022,1018,1007,977,\ 
934,879,819,756,695,635,578,533,477,423,371,323,282,247,221,206,198,198,206,218,233,246,259,269,275,281,283,280,273,265,254,240,228,212,196,181,168,154,\ 
142,131,125,120,117,115,113,109,104,98,93,89,87,83,82,82,84,86,92,98,102,104,105,103,103,103,105,106,106,105,104,102,105,119,155,220,319,418,556,696,834,\
942,990,1009,1017,1020,1022,1022,1020,1007,976,928,871,806,752,688,627,567,509,452,398,348,312,276,247,227,216,212,217,228,244,256,271,286,299,307,310,307,\ 
302,294,284,272,259,244,230,215,201,186,175,162,151,141,133,126,122,116,113,110,107,103,99,94,91,90,90,91,94,96,98,99,100,97,94,90,88,86,90,104,138,200,\ 
294,415,522,659,794,915,980,1005,1015,1019,1021,1022,1018,1003,969,919,859,792,725,670,605,541,481,423,371,323,283,250,232,215,207,207,213,223,237,251,\ 
263,277,289,297,301,300,294,284,272,261,245,228,212,197,182,167,154,144,133,121,111,103,96,88,83,78,76,73,73,72,71,70,70,69,69,70,73,76,81,82,83,82,80,\ 
78,76,76,88,115,167,251,370,482,630,779,912,979,1004,1015,1020,1021,1022,1022,1020,1008,979,934,878,815,750,699,636,575,518,461,405,351,304,272,240,215,\ 
200,192,193,199,209,221,231,243,253,260,266,266,265,261,256,246,233,218,203,187,172,156,141,131,118,106,100,95,91,88,85,84,83,82,82,82,83,84,87,88,91,92,\
93,93,94,94,94,93,94,93,92,90,91,96,114,153,223,303,431,574,723,867,960,997,1012,1017,1020,1022,1022,1022,1019,1003,971,923,879,818,757,696,637,579,524,\ 
470,431,383,340,299,264,237,219,210,209,215,228,243,258,273,286,295,299,299,294,284,272,259,247,234,220,206,196,181,167,154,142,130,120,109,102,97,91,86,\ 
83,81,81,78,76,74,70,64,59,56,56,58,62,66,72,77,83,88,95,101,105,107,110,115,134,175,249,358,492,637,784,894,971,1001,1014,1019,1021,1022,1022,1021,1015,\ 
993,954,900,841,778,715,654,607,549,493,439,388,338,296,259,238,218,209,208,215,226,240,255,271,282,292,297,297,291,281,270,258,248,237,225,214,202,189,\ 
174,160,147,142,133,127,120,115,112,110,107,108,106,104,101,99,96,94,95,97,101,105,107,109,108,107,104,105,107,111,121,145,190,265,370,497,633,744,874,\ 
962,998,1012,1018,1021,1021,1018,1013,991,949,895,830,763,693,623,570,505,442,383,327,279,239,208,187,182,179,185,196,213,232,253,271,286,297,301,299,\
292,284,273,258,240,225,206,186,169,155,143,133,124,119,113,107,103,99,97,95,94,93,94,93,95,98,103,108,113,117,121,121,119,116,113,110,107,105,108,116,\ 
136,179,252,357,485,623,761,866,959,997,1012,1018,1021,1022,1021,1016,1003,968,919,861,799,735,673,610,564,505,450,396,346,302,267,238,217,207,200,201\  
209,222,238,255,270,282,291,298,300,296,290,279,268,254,245,230,215,197,179,160,143,126,111,104,96,92,91,90,88,88,88,89,89,89,91,};


/*****************************************************************/
/*                                                               */
/*  Generatesig1 Function Description                            */
/*                                                               */
/*  The Generatesig1 function is called when the user clicks on  */
/*  the Generate button.  This will call GenerateGraphs          */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Generatesig1 (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GenerateGraphs();
            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Generate Waveforms",
                          "Generates the two signals and their convolve.");

            break;
    }
    return 0;
}


/*****************************************************************/
/*  This function determines which type of waves                 */
/*  waves have been selected and generates each signal according */
/*  to the selected parameters.  If noise has been enabled for   */
/*  either or both signals, the function then adds Gaussian      */
/*  noise to the appropriate signal(s).  The function then plots */
/*  the two signals on the upper two graphs.  The convolve is */
/*  then calculated and plotted on the lower graph.              */
/*****************************************************************/
void GenerateGraphs(void)
{
    int i, showDeconvWarning;
	int threshold_val, auto_threshold;
    double running_avg = 0;
	
	
///	GetCtrlVal(mainpnl, MAINPNL_PDTHRESH, &threshold_val);
	GetCtrlVal(mainpnl, MAINPNL_HBFAUTOTHRESH, &auto_threshold);
	
    switch (sig1wavetype) {
        case 0:
            SinePattern (sig1points, 7.0, sig1phase, sig1cycles, wave1);
            break;
        case 1:
            SquareWave (sig1points, 7.0, (sig1cycles/sig1points), &sig1phase,
                        50.0, wave1);
            break;
        case 2:
            TriangleWave (sig1points, 7.0, (sig1cycles/sig1points), &sig1phase,
                          wave1);
            break;
		case 3:
			ArbitraryWave (sig1points, 0.1, 7.8125E-3, &hbphase, heart_beat, 1016, LINEAR_INTERPOLATION, 
						   wave1);
			break;
    }
    
    if (sig1noise == 1) {
        GaussNoise (sig1points, sig1noiseamp, seed, noisewave1);
        for (i=0; i<sig1points; i++) {
            wave1[i] = wave1[i] + noisewave1[i];
        }
    }
    
    DeleteGraphPlot (mainpnl, MAINPNL_SIG1GRAPH, -1, VAL_IMMEDIATE_DRAW);
    PlotY (mainpnl, MAINPNL_SIG1GRAPH, wave1, sig1points, VAL_DOUBLE,
           VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
/*	
	GetCtrlVal(mainpnl, MAINPNL_RAW_FILTER, &raw_filt_sel);
	if(raw_filt_sel == 0)
	{
		running_avg = 0;
		for(i=0; i<sig1points; i++)
		{
			running_avg +=  wave1[i];
		}
		running_avg =  running_avg / (i + 1);
	}
	
	/// running_avg_x = (((running_avg_x) + ((long)gyro_x * 3) + 2) >> 2); (((running_avg_y * 7) + (gyro_y + 4)) >> 3);
	if((raw_filt_sel == 2) || (raw_filt_sel == 4))
	{
		GetCtrlVal(mainpnl, MAINPNL_HPBSFV, &hbf_val);
		for(i=0; i<sig1points; i++)
		{
			shift_filt_ravg = (((shift_filt_ravg) + ((long)wave1[i] * hbf_val) + (hbf_val - 1)) >> (hbf_val - 1));
			//shift_filt_ravg = (((shift_filt_ravg) + ((long)wave1[i] * hbf_val) + 4) >> 3);
			wave1[i] = (double)shift_filt_ravg;
		}
		
		PlotY (mainpnl, MAINPNL_SIG1GRAPH, wave1, sig1points, VAL_DOUBLE,
           VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_DK_BLUE);
		
		running_avg = 25;
		for(i=0; i<sig1points; i++)
		{
			running_avg +=  wave1[i];
		}
		running_avg =  running_avg / (i + 1)/2;
	}
	
	if((raw_filt_sel == 1) || (raw_filt_sel == 4))
	{
		GetCtrlVal(mainpnl, MAINPNL_HPFV, &hbf_val);
		for(i=0; i<sig1points; i++)
		{
			for(j=0; j<hbf_val; j++)
			{
				hbf_accm = hbf_accm + wave1[i+j];
			}
			wave1[i] = hbf_accm / hbf_val;
			hbf_accm = 0;
		}
		PlotY (mainpnl, MAINPNL_SIG1GRAPH, wave1, sig1points, VAL_DOUBLE,
           VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_DK_RED);
		
		running_avg = 25;
		for(i=0; i<sig1points; i++)
		{
			running_avg +=  wave1[i];
		}
		running_avg =  running_avg / (i + 1)/2;
	}
	*/

	//PeakDetector (wave1, sig1points, running_avg * 2.0, 3, DETECT_PEAKS, ANALYSIS_TRUE, ANALYSIS_TRUE, &hart_peak_cnt, &hart_peak_pos, &hart_peak_amp, &hart_peak_deriv);
///	if(!auto_threshold)
///		PeakDetector (wave1, sig1points, threshold_val, 3, DETECT_PEAKS, ANALYSIS_TRUE, ANALYSIS_TRUE, &hart_peak_cnt, &hart_peak_pos, &hart_peak_amp, &hart_peak_deriv);
///	else
///		PeakDetector (wave1, sig1points, running_avg * 2.0, 3, DETECT_PEAKS, ANALYSIS_TRUE, ANALYSIS_TRUE, &hart_peak_cnt, &hart_peak_pos, &hart_peak_amp, &hart_peak_deriv);
	
///	DeleteGraphPlot (mainpnl, MAINPNL_SIG3GRAPH, -1, VAL_IMMEDIATE_DRAW);
///	SetAxisScalingMode (mainpnl, MAINPNL_SIG3GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, (double)sig1points);
///	SetCtrlVal(mainpnl, MAINPNL_HART_PEAK_COUNT, hart_peak_cnt);
	for(i=0; i<hart_peak_cnt; i++)
	{
		PlotPoint (mainpnl, MAINPNL_SIG3GRAPH, hart_peak_pos[i], hart_peak_amp[i], VAL_SOLID_DIAMOND, VAL_BLACK);
	}
	
            
    switch (sig2wavetype) {
        case 0:
            SinePattern (sig2points, 7.0, sig2phase, sig2cycles, wave2);
            break;
        case 1:
            SquareWave (sig2points, 7.0, (sig2cycles/sig2points), &sig2phase,
                        50.0, wave2);
            break;
        case 2:
            TriangleWave (sig2points, 7.0, (sig2cycles/sig2points), &sig2phase,
                          wave2);
            break;
    }
    
    if (sig2noise == 1) {
        GaussNoise (sig2points, sig2noiseamp, seed, noisewave2);
        for (i=0; i<sig2points; i++) {
            wave2[i] = wave2[i] + noisewave2[i];
        }
    }
    
    DeleteGraphPlot (mainpnl, MAINPNL_SIG2GRAPH, -1, VAL_IMMEDIATE_DRAW);
    PlotY (mainpnl, MAINPNL_SIG2GRAPH, wave2, sig2points, VAL_DOUBLE,
           VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
           
    if ((sig1noise == 1) || (sig2noise == 1)) {
       seed = seed + 1;
       if (seed > 50) {
          seed = 1;
       }
    }
    Convolve (wave1, sig1points, wave2, sig2points, waveconv);
    DeleteGraphPlot (mainpnl, MAINPNL_SIG2GRAPH, -1, VAL_IMMEDIATE_DRAW);
    PlotY (mainpnl, MAINPNL_SIG2GRAPH, waveconv,
           (sig1points + sig2points -1), VAL_DOUBLE, VAL_THIN_LINE,
           VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_MAGENTA);

	/*************************************************************/
	/* The result of the deconvolution might be  unstable if any */
	/* of the values of the FFT of the signal are 0. 			 */
	/*************************************************************/
	memcpy (wave1Real, wave1, sig1points);
	ReFFT (wave1Real, wave1Imaginary, sig1points);
	showDeconvWarning = 0;
	for (i = 0; i < sig1points; ++i) {
		if ((fabs (wave1Real[i]) < DBL_EPSILON) && (fabs (wave1Imaginary[i]) < DBL_EPSILON)) {
			showDeconvWarning = 1;
			break;
		}
	}
    
	SetCtrlAttribute (mainpnl, MAINPNL_WARNINGMSG, ATTR_VISIBLE, showDeconvWarning);    
	DeleteGraphPlot (mainpnl, MAINPNL_SIG4GRAPH, -1, VAL_IMMEDIATE_DRAW);
   	Deconvolve (waveconv, (sig1points + sig2points -1), wave1, sig1points, wavedeconv);
    PlotY (mainpnl, MAINPNL_SIG4GRAPH, wavedeconv, (sig2points), VAL_DOUBLE, VAL_THIN_LINE,
           VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_GREEN);
	
}           
            

/*****************************************************************/
/*                                                               */
/*  Getsig1wavetype Function Description                         */
/*                                                               */
/*  The Getsig1wavetype function is called whenever the user     */
/*  changes the signal type for the first signal.  The function  */
/*  gets the new value of the control and stores it in the       */
/*  sig1wavetype variable.                                       */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig1wavetype (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG1WAVETYPE, &sig1wavetype);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Set Signal 1 Wave Type",
                          "Selects the type of wave for the first signal - sine, square, or triangular.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig1noise Function Description                            */
/*                                                               */
/*  The Getsig1noise function is called whenever the user        */
/*  changes the value of the noise enable switch for the first   */
/*  signal.  The function gets the new value of the control and  */
/*  puts it in the sig1noise variable.  If the user has disabled */
/*  the noise, the function then disables the noise amplitude    */
/*  control for the first signal.  If the user enables noise,    */
/*  the function enables the noise amplitude control for the     */
/*  first signal.                                                */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig1noise (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG1NOISE, &sig1noise);
            if (sig1noise == 0) {
                SetInputMode (mainpnl, MAINPNL_SIG1NOISEAMP, 0);
                
            }
            else {
                SetInputMode (mainpnl, MAINPNL_SIG1NOISEAMP, 1);
                
            }
            
            if (autoGenerateGraphs) GenerateGraphs();
              
            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Set Signal 1 Noise",
                          "Enables or disables noise addition to the first signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig1cycles Function Description                           */
/*                                                               */
/*  The Getsig1cycles function is called whenever the user       */
/*  changes the value of the control for the number of cycles in */
/*  the first signal.  The function gets the new value of the    */
/*  control and puts it in the sig1cycles variable.              */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig1cycles (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG1CYCLES, &sig1cycles);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Select Signal 1 Cycles",
                          "Selects the number of cycles in the first signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig1phase Function Description                            */
/*                                                               */
/*  The Getsig1phase function is called whenever the user        */
/*  changes the vaule of the phase of the first signal.  The     */
/*  function gets the new value of the control and puts it in    */
/*  the sig1phase variable.                                      */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig1phase (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG1PHASE, &sig1phase);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Select Signal 1 Phase",
                          "Selects the phase offset for the first signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig1points Function Description                           */
/*                                                               */
/*  The Getsig1points function is called whenever the user       */
/*  changes the number of points in the first signal.  The       */
/*  function gets the new value of the control and puts it in    */
/*  the sig1points variable.                                     */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig1points (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG1POINTS, &sig1points);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Set Signal 1 Points",
                          "Selects the number of points in the first signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Quit Function Description                                    */
/*                                                               */
/*  The Quit function is called when the user clicks on the Quit */
/*  button on the main panel.  The function quits the program.   */
/*                                                               */
/*****************************************************************/

/*****************************************************************/
/*                                                               */
/*  Getsig1noiseamp Function Description                         */
/*                                                               */
/*  The Getsig1noiseamp function is called whenever the user     */
/*  changes the value of the control for the noise amplitude for */
/*  the first signal.  The function gets the new value of the    */
/*  control and puts it in the sig1noiseamp variable.            */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig1noiseamp (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG1NOISEAMP, &sig1noiseamp);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Select Signal 1 Noise Amplitude",
                          "Sets the standard deviation of the Gaussian noise added to the first signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig2points Function Description                           */
/*                                                               */
/*  The Getsig2points function is called whenever the user       */
/*  changes the value of the control for the number of points    */
/*  in the second signal.  The function gets the new value of    */
/*  the control and puts it in the sig2points variable.          */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig2points (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG2POINTS, &sig2points);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Select Signal 2 Points",
                          "Selects the number of points in the second signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig2cycles Function Description                           */
/*                                                               */
/*  The Getsig2cycles function is called whenever the user       */
/*  changes the value of the control for the number of cycles in */
/*  the second signal.  The function gets the new value of the   */
/*  control and puts it in the sig2cycles variable.              */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig2cycles (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG2CYCLES, &sig2cycles);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Set Signal 2 Cycles",
                          "Sets the number of cycles in the second signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig2phase Function Description                            */
/*                                                               */
/*  The Getsig2phase function is called whenever the user        */
/*  changes the value of the control of the phase of the second  */
/*  signal.  The function gets the new value of the control and  */
/*  puts it in the sig2phase variable.                           */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig2phase (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG2PHASE, &sig2phase);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Set Signal 2 Phase",
                          "Sets the phase offset of the second signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig2noise Function Description                            */
/*                                                               */
/*  The Getsig2noise function is called whenever the user        */
/*  changes the value of the control for the enable/disable      */
/*  switch for the noise on the second signal.  The function     */
/*  gets the new value of the control and puts it in the         */
/*  sig2noise variable.  If the noise is disabled, the function  */
/*  disables the noise level control for the second signal.  If  */
/*  the noise is enabled, the function enables the noise level   */
/*  control.                                                     */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig2noise (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG2NOISE, &sig2noise);
            if (sig2noise == 0) {
               SetInputMode (mainpnl, MAINPNL_SIG2NOISEAMP, 0);
            }
            else {
               SetInputMode (mainpnl, MAINPNL_SIG2NOISEAMP, 1);
            }   
            if (autoGenerateGraphs) GenerateGraphs();
            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Enable/Disable Signal 2 Noise",
                          "Enables and disables the noise added to the second signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*                                                               */
/*  Getsig2noiseamp Function Description                         */
/*                                                               */
/*  The Getsig2noiseamp function is called whenever the user     */
/*  changes the value of the control for the amplitude of the    */
/*  noise added to the second signal.  The function gets the new */
/*  value of the control and puts it in the sig2noiseamp         */
/*  variable.                                                    */
/*                                                               */
/*****************************************************************/

int CVICALLBACK Getsig2noiseamp (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG2NOISEAMP, &sig2noiseamp);
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Select Signal 2 Noise Amplitude",
                          "Sets the standard deviation of the Gaussian noise added to the second signal.");

            break;
    }
    return 0;
}

/*****************************************************************/
/*  Getsig2wavetype Function Description                         */
/*                                                               */
/*  The Getsig2wavetype function is called whenever the user     */
/*  changes the value of the control selecting the type of wave  */
/*  used for the second signal.  The function gets the new value */
/*  of the control and puts it in the sig2wavetype variable.     */
/*****************************************************************/

int CVICALLBACK Getsig2wavetype (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            GetCtrlVal (mainpnl, MAINPNL_SIG2WAVETYPE, &sig2wavetype);
            if (autoGenerateGraphs) GenerateGraphs();
            if (autoGenerateGraphs) GenerateGraphs();

            break;
        case EVENT_RIGHT_CLICK:
            MessagePopup ("Select Signal 2 Wave Type",
                          "Selects the type of wave used for the second signal - sine, square, or triangle.");

            break;
    }
    return 0;
}


int CVICALLBACK SetAutoGenerate (int panel, int control, int event,
        void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_VAL_CHANGED:
            GetCtrlVal(panel, control ,&autoGenerateGraphs);

            break;
    }
    return 0;
}
