/*
 * bsp.h
 *
 * Created: 
 *  Author: rklo
 */ 
 
#ifndef BSP_H__
#define BSP_H__

#include "global.h"

// IMU Related Pins
#define AXIS_INT1				5
#define AXIS_INT2				6
#define MAG_RDY					7
#define MAG_INT					8
#define IMU_INT					AXIS_INT1

//Uart Logging
#define MAX_TEST_DATA_BYTES     (15U)			/**< max number of test bytes to be used for tx and rx. */
#define RX_PIN_NUMBER  			21				//Not Used. Connect to Reset Pin
#define TX_PIN_NUMBER  			12				//12 for Dev Board, 14 for the prototype production board
#define RTS_PIN_NUMBER  		0xFF			//Not Used
#define CTS_PIN_NUMBER  		0xFF			//Not Used
#define LOG_BUAD				UART_BAUDRATE_BAUDRATE_Baud115200

// Test Point X on Dev PCB
#define TEST_IO		32

static uint8_t m_uart_data;
static bool m_uart_has_input;

#endif
