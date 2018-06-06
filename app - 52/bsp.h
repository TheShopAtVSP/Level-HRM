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

// Battery Monitor and Charger Related Pins
#define BQ_INT_PIN				7
#define CD_OUT_PIN				8
#define CHARGE_PG_INPUT			9
#define BAT_HALF_EN				10
#define BAT_HALF_IN				NRF_SAADC_INPUT_AIN0

// Uart Logging
#define MAX_TEST_DATA_BYTES     (15U)			/**< max number of test bytes to be used for tx and rx. */
#define RX_PIN_NUMBER  			21				//Not Used. Connect to Reset Pin
#define TX_PIN_NUMBER  			12				//12 for Dev Board, 14 for the prototype production board
#define RTS_PIN_NUMBER  		0xFF			//Not Used
#define CTS_PIN_NUMBER  		0xFF			//Not Used
#define LOG_BUAD				UART_BAUDRATE_BAUDRATE_Baud115200

// SPI Flash
#define SPI_SCK_PIN         	15
#define SPI_MOSI_PIN        	16
#define SPI_MISO_PIN        	12
#define SPI_CS_PIN				13
#define SPI_HOLD_PIN			14

// Test Point X on Dev PCB
#define TEST_IO		32

static uint8_t m_uart_data;
static bool m_uart_has_input;

#endif
