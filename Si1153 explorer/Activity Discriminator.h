/**************************************************************************/
/* LabWindows/CVI User Interface Resource (UIR) Include File              */
/*                                                                        */
/* WARNING: Do not add to, delete from, or otherwise modify the contents  */
/*          of this include file.                                         */
/**************************************************************************/

#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

     /* Panels and Controls: */

#define  MAINPNL                          1
#define  MAINPNL_GRAPH3D                  2       /* control type: activeX, callback function: (none) */
#define  MAINPNL_HELP                     3       /* control type: command, callback function: HelpCallback */
#define  MAINPNL_QUIT                     4       /* control type: command, callback function: Quit */
#define  MAINPNL_SIG4GRAPH                5       /* control type: graph, callback function: (none) */
#define  MAINPNL_SIG2GRAPH                6       /* control type: graph, callback function: (none) */
#define  MAINPNL_SIG3GRAPH                7       /* control type: graph, callback function: (none) */
#define  MAINPNL_WARNINGMSG               8       /* control type: textMsg, callback function: (none) */
#define  MAINPNL_SAMP_PER_SEC             9       /* control type: numeric, callback function: (none) */
#define  MAINPNL_PEAK_DETECTOR_SEL        10      /* control type: ring, callback function: Peak_Detector_CB */
#define  MAINPNL_GRAVITY_VECT             11      /* control type: ring, callback function: Gravity_Vector_CB */
#define  MAINPNL_POS_INTGTR_SEL           12      /* control type: ring, callback function: Position_Integrator_CB */
#define  MAINPNL_RAW_FILTER               13      /* control type: ring, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_GYRSFTFV                 14      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_ACLSFTFV                 15      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_GYR_SSI                  16      /* control type: numeric, callback function: Position_Integrator_CB */
#define  MAINPNL_ACC_SSI                  17      /* control type: numeric, callback function: Position_Integrator_CB */
#define  MAINPNL_STITCH_TH                18      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_GYRAVGFV                 19      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_ACLAVGFV                 20      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_PD_THRESH                21      /* control type: scale, callback function: Peak_Detector_CB */
#define  MAINPNL_PD_THRESH_W              22      /* control type: scale, callback function: Peak_Detector_CB */
#define  MAINPNL_GV_ADJUST                23      /* control type: scale, callback function: Apply_Gravity_Vector_CB */
#define  MAINPNL_PD_W_THRESH_AUTO         24      /* control type: radioButton, callback function: Apply_Gravity_Vector_CB */
#define  MAINPNL_PD_THRESH_AUTO           25      /* control type: radioButton, callback function: Apply_Gravity_Vector_CB */
#define  MAINPNL_HBFAUTOTHRESH_3          26      /* control type: radioButton, callback function: Apply_Gravity_Vector_CB */
#define  MAINPNL_INT_SPAN                 27      /* control type: scale, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_HBFAUTOTHRESH_2          28      /* control type: radioButton, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_ACC_THRESHHOLD           29      /* control type: scale, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_HBFAUTOTHRESH            30      /* control type: radioButton, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_HRM_FFT_GRAPH            31      /* control type: graph, callback function: (none) */
#define  MAINPNL_HRM_FFT_GRAPH_RT         32      /* control type: graph, callback function: (none) */
#define  MAINPNL_HRM_HIST_GRAPH           33      /* control type: graph, callback function: (none) */
#define  MAINPNL_ACC_HIST_GRAPH           34      /* control type: graph, callback function: (none) */
#define  MAINPNL_DECORATION_5             35      /* control type: deco, callback function: (none) */
#define  MAINPNL_MAG_HIST_GRAPH           36      /* control type: graph, callback function: (none) */
#define  MAINPNL_ACCEL_GRAPH_1            37      /* control type: graph, callback function: (none) */
#define  MAINPNL_DECORATION_3             38      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_4             39      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_6             40      /* control type: deco, callback function: (none) */
#define  MAINPNL_TEXTBOX                  41      /* control type: textBox, callback function: (none) */
#define  MAINPNL_LOAD_LAST_DATA           42      /* control type: command, callback function: Load_Last_Data */
#define  MAINPNL_LOAD_DATA                43      /* control type: command, callback function: Load_Data */
#define  MAINPNL_SAVE_DATA                44      /* control type: command, callback function: Save_Data */
#define  MAINPNL_GETRTDATA                45      /* control type: textButton, callback function: (none) */
#define  MAINPNL_DECORATION_8             46      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_9             47      /* control type: deco, callback function: (none) */
#define  MAINPNL_NRTS                     48      /* control type: numeric, callback function: SetAquisisionTime */
#define  MAINPNL_S4GM                     49      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S4GZ                     50      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S4GY                     51      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S4GX                     52      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S3GM                     53      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S3GZ                     54      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S3GY                     55      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S3GX                     56      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S2GM                     57      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S2GZ                     58      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S2GY                     59      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S2GX                     60      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S1GM                     61      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S1GZ                     62      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S1GY                     63      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S1GX                     64      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S4G_SEL                  65      /* control type: ring, callback function: UpdateSG4_CB */
#define  MAINPNL_S3G_SEL                  66      /* control type: ring, callback function: UpdateSG3_CB */
#define  MAINPNL_S2G_SEL                  67      /* control type: ring, callback function: UpdateSG2_CB */
#define  MAINPNL_S1G_SEL                  68      /* control type: ring, callback function: UpdateSG1_CB */
#define  MAINPNL_SS_INTEGRATOR            69      /* control type: radioButton, callback function: Position_Integrator_CB */
#define  MAINPNL_CALOR_DISP_4             70      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_DISP_5             71      /* control type: string, callback function: (none) */
#define  MAINPNL_STEPS_DISP               72      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_DISP_3             73      /* control type: string, callback function: (none) */
#define  MAINPNL_PACE_DISP                74      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_TOT_DISP           75      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_DISP               76      /* control type: string, callback function: (none) */
#define  MAINPNL_HART_BEAT_DISP_3         77      /* control type: string, callback function: (none) */
#define  MAINPNL_HART_BEAT_DISP_4         78      /* control type: string, callback function: (none) */
#define  MAINPNL_HART_BEAT_DISP_2         79      /* control type: string, callback function: (none) */
#define  MAINPNL_HART_BEAT_DISP           80      /* control type: string, callback function: (none) */
#define  MAINPNL_DECORATION_13            81      /* control type: deco, callback function: (none) */
#define  MAINPNL_LED_5                    82      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_6                    83      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_7                    84      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_8                    85      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_4                    86      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_RUNNING              87      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_WALKING              88      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_RESTING              89      /* control type: LED, callback function: (none) */
#define  MAINPNL_DECORATION_17            90      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_7             91      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_16            92      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION               93      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_10            94      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_11            95      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_14            96      /* control type: deco, callback function: (none) */
#define  MAINPNL_PD_PLOT                  97      /* control type: listBox, callback function: Peak_Detector_CB */
#define  MAINPNL_DECORATION_18            98      /* control type: deco, callback function: (none) */
#define  MAINPNL_PD_SOURCE                99      /* control type: ring, callback function: Peak_Detector_CB */
#define  MAINPNL_SER_XFER_LED             100     /* control type: LED, callback function: (none) */
#define  MAINPNL_DECORATION_12            101     /* control type: deco, callback function: (none) */
#define  MAINPNL_PLOT3D_SCALE_ON          102     /* control type: textButton, callback function: Plot_3D_CB */
#define  MAINPNL_PLOT3D_GYRO_ON           103     /* control type: textButton, callback function: GYR_Plot_3D_CB */
#define  MAINPNL_PLOT3D_ACC_ON            104     /* control type: textButton, callback function: ACC_Plot_3D_CB */
#define  MAINPNL_LED_RING                 105     /* control type: ring, callback function: Set_LED_Source */
#define  MAINPNL_HRM_7SEG_2               106     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_3               107     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG                 108     /* control type: numeric, callback function: (none) */
#define  MAINPNL_RT_TIMER                 109     /* control type: timer, callback function: RT_Timer_CB */
#define  MAINPNL_GV_IMAGE                 110     /* control type: picture, callback function: OGLCallback */
#define  MAINPNL_PHASE2SF                 111     /* control type: numeric, callback function: Phase2ScaleFactor */
#define  MAINPNL_AUTO_P2SF                112     /* control type: textButton, callback function: Phase2ScaleFactor */
#define  MAINPNL_SQRB                     113     /* control type: textButton, callback function: Sqr_B */
#define  MAINPNL_SQRA                     114     /* control type: textButton, callback function: Sqr_A */
#define  MAINPNL_LED_C_I                  115     /* control type: ring, callback function: LED_C_Current */
#define  MAINPNL_LED_B_I                  116     /* control type: ring, callback function: LED_B_Current */
#define  MAINPNL_LED_A_I                  117     /* control type: ring, callback function: LED_A_Current */
#define  MAINPNL_CH2_LEDA_EN              118     /* control type: radioButton, callback function: Chan_2_LED_Enable */
#define  MAINPNL_CH2_LEDC_EN              119     /* control type: radioButton, callback function: Chan_2_LED_Enable */
#define  MAINPNL_CH2_LEDB_EN              120     /* control type: radioButton, callback function: Chan_2_LED_Enable */
#define  MAINPNL_CH1_LEDC_EN              121     /* control type: radioButton, callback function: Chan_1_LED_Enable */
#define  MAINPNL_CH1_LEDB_EN              122     /* control type: radioButton, callback function: Chan_1_LED_Enable */
#define  MAINPNL_CH1_LEDA_EN              123     /* control type: radioButton, callback function: Chan_1_LED_Enable */
#define  MAINPNL_GLASSESONMSG             124     /* control type: textMsg, callback function: (none) */
#define  MAINPNL_SIG1GRAPH                125     /* control type: strip, callback function: Accel_and_Gyro_Cursor_Update */
#define  MAINPNL_HRM_DAY_GRAPH            126     /* control type: graph, callback function: (none) */
#define  MAINPNL_DECORATION_15            127     /* control type: deco, callback function: (none) */
#define  MAINPNL_SG_FS                    128     /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_SG_PO                    129     /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_RESTART_FO               130     /* control type: command, callback function: ReStart_FO */
#define  MAINPNL_CLR_SCART                131     /* control type: command, callback function: ClearStripChartCB */
#define  MAINPNL_ONE_SHOT                 132     /* control type: radioButton, callback function: (none) */
#define  MAINPNL_MISS                     133     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HIT                      134     /* control type: numeric, callback function: (none) */
#define  MAINPNL_DECORATION_19            135     /* control type: deco, callback function: (none) */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK ACC_Plot_3D_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Accel_and_Gyro_Cursor_Update(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Apply_Gravity_Vector_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Apply_Raw_Data_Filtering_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Chan_1_LED_Enable(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Chan_2_LED_Enable(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ClearStripChartCB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Gravity_Vector_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK GYR_Plot_3D_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK HelpCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LED_A_Current(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LED_B_Current(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LED_C_Current(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Load_Data(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Load_Last_Data(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK OGLCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Peak_Detector_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Phase2ScaleFactor(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Plot_3D_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Position_Integrator_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Quit(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ReStart_FO(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK RT_Timer_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Save_Data(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Set_LED_Source(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SetAquisisionTime(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Sqr_A(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Sqr_B(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK UpdateSG1_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK UpdateSG2_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK UpdateSG3_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK UpdateSG4_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
