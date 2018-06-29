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
#define  MAINPNL_HELP                     2       /* control type: command, callback function: HelpCallback */
#define  MAINPNL_QUIT                     3       /* control type: command, callback function: Quit */
#define  MAINPNL_SIG4GRAPH                4       /* control type: graph, callback function: (none) */
#define  MAINPNL_SIG2GRAPH                5       /* control type: graph, callback function: (none) */
#define  MAINPNL_SIG3GRAPH                6       /* control type: graph, callback function: (none) */
#define  MAINPNL_WARNINGMSG               7       /* control type: textMsg, callback function: (none) */
#define  MAINPNL_SAMP_PER_SEC             8       /* control type: numeric, callback function: (none) */
#define  MAINPNL_PEAK_DETECTOR_SEL        9       /* control type: ring, callback function: Peak_Detector_CB */
#define  MAINPNL_POS_INTGTR_SEL           10      /* control type: ring, callback function: Position_Integrator_CB */
#define  MAINPNL_RAW_FILTER               11      /* control type: ring, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_GYRSFTFV                 12      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_ACLSFTFV                 13      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_GYR_SSI                  14      /* control type: numeric, callback function: Position_Integrator_CB */
#define  MAINPNL_ACC_SSI                  15      /* control type: numeric, callback function: Position_Integrator_CB */
#define  MAINPNL_STITCH_TH                16      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_GYRAVGFV                 17      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_ACLAVGFV                 18      /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_PD_THRESH                19      /* control type: scale, callback function: Peak_Detector_CB */
#define  MAINPNL_PD_ABSOP_DELTA           20      /* control type: scale, callback function: Peak_Detector_CB */
#define  MAINPNL_PD_EMI_DELTA             21      /* control type: scale, callback function: Peak_Detector_CB */
#define  MAINPNL_PD_THRESH_W              22      /* control type: scale, callback function: Peak_Detector_CB */
#define  MAINPNL_PD_ABSOP_THRESH_AUTO     23      /* control type: radioButton, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_PD_EMI_THRESH_AUTO       24      /* control type: radioButton, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_PD_THRESH_AUTO           25      /* control type: radioButton, callback function: (none) */
#define  MAINPNL_INT_SPAN                 26      /* control type: scale, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_HBFAUTOTHRESH_2          27      /* control type: radioButton, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_ACC_THRESHHOLD           28      /* control type: scale, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_HBFAUTOTHRESH            29      /* control type: radioButton, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_HRM_FFT_GRAPH            30      /* control type: graph, callback function: (none) */
#define  MAINPNL_HRM_FFT_GRAPH_RT         31      /* control type: graph, callback function: (none) */
#define  MAINPNL_HRM_HIST_GRAPH           32      /* control type: graph, callback function: (none) */
#define  MAINPNL_ACC_HIST_GRAPH           33      /* control type: graph, callback function: (none) */
#define  MAINPNL_DECORATION_5             34      /* control type: deco, callback function: (none) */
#define  MAINPNL_MAG_HIST_GRAPH           35      /* control type: graph, callback function: (none) */
#define  MAINPNL_DECORATION_3             36      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_4             37      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_6             38      /* control type: deco, callback function: (none) */
#define  MAINPNL_TEXTBOX                  39      /* control type: textBox, callback function: (none) */
#define  MAINPNL_LOAD_LAST_DATA           40      /* control type: command, callback function: Load_Last_Data */
#define  MAINPNL_LOAD_DATA                41      /* control type: command, callback function: Load_Data */
#define  MAINPNL_SAVE_DATA                42      /* control type: command, callback function: Save_Data */
#define  MAINPNL_GETRTDATA                43      /* control type: textButton, callback function: Restart_Aquisition */
#define  MAINPNL_DECORATION_8             44      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_9             45      /* control type: deco, callback function: (none) */
#define  MAINPNL_NRTS                     46      /* control type: numeric, callback function: SetAquisisionTime */
#define  MAINPNL_S4GM                     47      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S4GZ                     48      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S4GY                     49      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S4GX                     50      /* control type: radioButton, callback function: UpdateSG4_CB */
#define  MAINPNL_S3GM                     51      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S3GZ                     52      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S3GY                     53      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S3GX                     54      /* control type: radioButton, callback function: UpdateSG3_CB */
#define  MAINPNL_S2GM                     55      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S2GZ                     56      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S2GY                     57      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S2GX                     58      /* control type: radioButton, callback function: UpdateSG2_CB */
#define  MAINPNL_S1GM                     59      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S1GZ                     60      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S1GY                     61      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S1GX                     62      /* control type: radioButton, callback function: UpdateSG1_CB */
#define  MAINPNL_S4G_SEL                  63      /* control type: ring, callback function: UpdateSG4_CB */
#define  MAINPNL_S3G_SEL                  64      /* control type: ring, callback function: UpdateSG3_CB */
#define  MAINPNL_S2G_SEL                  65      /* control type: ring, callback function: UpdateSG2_CB */
#define  MAINPNL_S1G_SEL                  66      /* control type: ring, callback function: UpdateSG1_CB */
#define  MAINPNL_SS_INTEGRATOR            67      /* control type: radioButton, callback function: Position_Integrator_CB */
#define  MAINPNL_CALOR_DISP_4             68      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_DISP_5             69      /* control type: string, callback function: (none) */
#define  MAINPNL_STEPS_DISP               70      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_DISP_3             71      /* control type: string, callback function: (none) */
#define  MAINPNL_PACE_DISP                72      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_TOT_DISP           73      /* control type: string, callback function: (none) */
#define  MAINPNL_CALOR_DISP               74      /* control type: string, callback function: (none) */
#define  MAINPNL_HART_BEAT_DISP           75      /* control type: string, callback function: (none) */
#define  MAINPNL_LED_5                    76      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_6                    77      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_7                    78      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_8                    79      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_4                    80      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_RUNNING              81      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_WALKING              82      /* control type: LED, callback function: (none) */
#define  MAINPNL_LED_RESTING              83      /* control type: LED, callback function: (none) */
#define  MAINPNL_DECORATION_17            84      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_7             85      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_16            86      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION               87      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_10            88      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_11            89      /* control type: deco, callback function: (none) */
#define  MAINPNL_DECORATION_14            90      /* control type: deco, callback function: (none) */
#define  MAINPNL_PD_PLOT                  91      /* control type: listBox, callback function: Peak_Detector_CB */
#define  MAINPNL_DECORATION_18            92      /* control type: deco, callback function: (none) */
#define  MAINPNL_PD_SOURCE                93      /* control type: ring, callback function: Peak_Detector_CB */
#define  MAINPNL_SER_XFER_LED             94      /* control type: LED, callback function: (none) */
#define  MAINPNL_DECORATION_12            95      /* control type: deco, callback function: (none) */
#define  MAINPNL_LED_RING                 96      /* control type: ring, callback function: Set_LED_Source */
#define  MAINPNL_HRM_7SEG_2               97      /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_3               98      /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_9               99      /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_Y2              100     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_Y1              101     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_Y0              102     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_6               103     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG_4               104     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HRM_7SEG                 105     /* control type: numeric, callback function: (none) */
#define  MAINPNL_RT_TIMER                 106     /* control type: timer, callback function: RT_Timer_CB */
#define  MAINPNL_PHASE2SF                 107     /* control type: numeric, callback function: Phase2ScaleFactor */
#define  MAINPNL_AUTO_P2SF                108     /* control type: textButton, callback function: Phase2ScaleFactor */
#define  MAINPNL_SQRB                     109     /* control type: textButton, callback function: Sqr_B */
#define  MAINPNL_SQRA                     110     /* control type: textButton, callback function: Sqr_A */
#define  MAINPNL_LED_C_I                  111     /* control type: ring, callback function: LED_C_Current */
#define  MAINPNL_LED_B_I                  112     /* control type: ring, callback function: LED_B_Current */
#define  MAINPNL_LED_A_I                  113     /* control type: ring, callback function: LED_A_Current */
#define  MAINPNL_CH2_LEDA_EN              114     /* control type: radioButton, callback function: Chan_2_LED_Enable */
#define  MAINPNL_CH2_LEDC_EN              115     /* control type: radioButton, callback function: Chan_2_LED_Enable */
#define  MAINPNL_CH2_LEDB_EN              116     /* control type: radioButton, callback function: Chan_2_LED_Enable */
#define  MAINPNL_CH1_LEDC_EN              117     /* control type: radioButton, callback function: Chan_1_LED_Enable */
#define  MAINPNL_CH1_LEDB_EN              118     /* control type: radioButton, callback function: Chan_1_LED_Enable */
#define  MAINPNL_CH1_LEDA_EN              119     /* control type: radioButton, callback function: Chan_1_LED_Enable */
#define  MAINPNL_GLASSESONMSG             120     /* control type: textMsg, callback function: (none) */
#define  MAINPNL_SIG1GRAPH                121     /* control type: strip, callback function: Accel_and_Gyro_Cursor_Update */
#define  MAINPNL_HRM_DAY_GRAPH            122     /* control type: graph, callback function: (none) */
#define  MAINPNL_DECORATION_15            123     /* control type: deco, callback function: (none) */
#define  MAINPNL_SG_FS                    124     /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_SG_PO                    125     /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_RESTART_FO               126     /* control type: command, callback function: ReStart_FO */
#define  MAINPNL_CLR_SCART                127     /* control type: command, callback function: ClearStripChartCB */
#define  MAINPNL_ONE_SHOT                 128     /* control type: radioButton, callback function: (none) */
#define  MAINPNL_MISS                     129     /* control type: numeric, callback function: (none) */
#define  MAINPNL_HIT                      130     /* control type: numeric, callback function: (none) */
#define  MAINPNL_DECORATION_19            131     /* control type: deco, callback function: (none) */
#define  MAINPNL_PD_EMI_WIDTH             132     /* control type: numeric, callback function: Apply_Raw_Data_Filtering_CB */
#define  MAINPNL_DECORATION_2             133     /* control type: deco, callback function: (none) */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK Accel_and_Gyro_Cursor_Update(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Apply_Raw_Data_Filtering_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Chan_1_LED_Enable(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Chan_2_LED_Enable(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ClearStripChartCB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK HelpCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LED_A_Current(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LED_B_Current(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LED_C_Current(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Load_Data(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Load_Last_Data(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Peak_Detector_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Phase2ScaleFactor(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Position_Integrator_CB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Quit(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Restart_Aquisition(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
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
