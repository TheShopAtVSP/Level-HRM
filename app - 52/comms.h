
/*
 * comms.h
 *
 * Created: 4/14/2015 5:26:18 PM
 *  Author: Matt Workman
 */ 


#ifndef COMMS_H_
#define COMMS_H_

#include <stdint.h>
#include "../global.h"
#include "app_timer.h"

typedef enum {
	NACK = 				0x00,
	ACK = 				0x01,
	BOOT_SET_ADDRESS =	0x02,
	BOOT_WRITE_DATA =	0x03,
	BOOT_WRITE_CMPLT =	0x04,
	START_OF_RECORD =	0x05,
	CONTINUE_RECORD =	0x06,
	REPORT_ATTRIBUTES =	0x07,
	REPORT_CONTROL =	0x08,
	TRANSMIT_CONTROL =	0x09,
	TIMERD = 			0x0A,
	TIMEWR = 			0x0B,	
	USERUUIDRD = 		0x0C,
	USERUUIDWR = 		0x0D,	
	FRAMERD = 			0x0E,
	FRAMEWR = 			0x0F,	
	BLE_TX_POWER = 		0x10,
	LED_ID_RX = 		0x11,
	NUKE_BOND =			0x12,
	NUKE_RECORDS =		0x13,
	CHECK_KEY =			0x14,
	TRACE_CONTROL =		0x15,
	MANUFACT_CONTROL =	0x16, 
	SECURITY_KEY_WR =	0x17,
	HRM_PKT			=	0x69,
	
	NUM_PACKET_TYPES,
} T_PKT_TYPES;
//
//typedef enum {
	//ADDR_ACCEPTED = 1,
	//WRITE_SUCCESS = 2,
	//BOOT_COMPLETE = 3,
//} TACK_CODES;

typedef enum {
	ADDR_ERROR = 1,
	PKT_SEQ_ERROR = 2,
	DATA_LEN_ERROR = 3,
	WRITES_DISABLED = 4,
	PKT_TYPE_UNRECOG = 5,
	PKT_LEN_ERROR = 6,
	WRITE_FAILURE = 7,
	WRITE_ABORTED = 8,
	ATTRIBUTE_ERROR = 9,
	INVALID_CODE = 10,
	PKT_TX_ERR = 11,
} TNACK_CODES;


#define PKT_HEADER_LEN		2
#define MAX_PKT_PAYLOAD    20
typedef __packed struct {	
	uint8_t id;
	T_PKT_TYPES type;			//T_PKT_TYPES type;
	uint8_t payload[MAX_PKT_PAYLOAD];
	
	uint8_t pkt_len;			//only used in transmitting packets
} T_PACKET;

/*
// Received Message
//max of 20 bytes: 1 byte(command) + 1 byte(len&seq) + 18 bytes(data)
//The Bluetooth module will currently only send up to 20 bytes at a time
#define HEADER_LEN  2
typedef struct BLE_RXMSG {
	T_PKT_TYPES command;              //Received byte 1
	unsigned char seq;              //3 LSbits of received byte 2
	unsigned char len;              //5 MSbits of received byte 2
	unsigned char data[MAX_DATA];   //Recieved byte 3 thru len
}__attribute__ ((packed)) TBLE_RXMSG;

// Transmitted Message (structure is different from RXMSG per Rich: "Just to keep it interesting")
//max of 31 bytes: 1 byte(seq&command) + 1 byte(len) + 29 bytes(data)
typedef union BLE_TXMSG {
	uint8_t txarray[MAX_DATA+2];
	struct {
		T_PKT_TYPES command: 5;           //5 LSbits of transmit byte 1
		char seq: 3;                    //3 MSbits of transmit byte 1
		unsigned char len;              //Transmit byte 2
		unsigned char data[MAX_DATA];   //Transmit byte 3 thru len
	};
}__attribute__ ((packed)) TBLE_TXMSG;
*/

void init_comms( app_timer_id_t p_detach_timer );
void detach_handler(void * p_context );
void flag_new_comms( void );
void parse_msg( T_PACKET * );
void ble_send_nus_data( void );
bool secure_connection( void );
bool queue_packet_wrapper( T_PKT_TYPES pkt_type, uint8_t * data , uint8_t data_len );
bool ble_transfer_packets_wrapper( void );

#endif /* COMMS_H_ */



