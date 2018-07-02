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

#define  PANEL                            1       /* callback function: panelCB */
#define  PANEL_SCAN_LIST                  2       /* control type: textBox, callback function: Select_Device */
#define  PANEL_CONSOLE                    3       /* control type: textBox, callback function: (none) */
#define  PANEL_B2L_SUCESS_LED             4       /* control type: LED, callback function: (none) */
#define  PANEL_START_B2L_LED              5       /* control type: LED, callback function: (none) */
#define  PANEL_KEY_RING_EXISTS_LED        6       /* control type: LED, callback function: (none) */
#define  PANEL_READ_KEY_RING_LED          7       /* control type: LED, callback function: (none) */
#define  PANEL_FIRST_KEY_LED              8       /* control type: LED, callback function: (none) */
#define  PANEL_BONDED_LED                 9       /* control type: LED, callback function: (none) */
#define  PANEL_CONNET_LED                 10      /* control type: LED, callback function: (none) */
#define  PANEL_PURPLE_BUTTON              11      /* control type: pictButton, callback function: PurpleButton */
#define  PANEL_WHITE_BUTTON               12      /* control type: pictButton, callback function: WhiteButton */
#define  PANEL_GREEN_BUTTON               13      /* control type: pictButton, callback function: GreenButton */
#define  PANEL_RED_BUTTON                 14      /* control type: pictButton, callback function: RedButton */
#define  PANEL_B2L_LED_3                  15      /* control type: LED, callback function: (none) */
#define  PANEL_B2L_LED_2                  16      /* control type: LED, callback function: (none) */
#define  PANEL_B2L_LED_1                  17      /* control type: LED, callback function: (none) */
#define  PANEL_B2L_LED_0                  18      /* control type: LED, callback function: (none) */
#define  PANEL_DECORATION                 19      /* control type: deco, callback function: (none) */
#define  PANEL_DECORATION_2               20      /* control type: deco, callback function: (none) */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK GreenButton(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK panelCB(int panel, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK PurpleButton(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK RedButton(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Select_Device(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK WhiteButton(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
