/*
 * comms.c
 *
 * Created: 4/14/2015 1:51:13 PM
 *  Author: matt
 */

#include "reports.h"
#include "comms.h"
#include "mem_manager.h"
#include "battery.h"
#include "ble_gap.h"
#include "led.h"
#include "ble_err.h"
#include "hal_nvm.h"
#include "manufacture_test.h"
#include "app_util_platform.h"

#define PACKET_TIMEOUT	5000

// Local State Variable Enums
typedef enum {
	NO_CONNECTION,
	GET_KEY_STATE,
	SECURE_CONNECTION
} T_SECURITY_STATE;

typedef enum {
	ACK_IDLE,
	WAITING,
	RECEIVED,
	ID_ACK_FAIL
} T_ACK_STATE;

typedef enum {
	TX_OFF,
	TX_START,
	TX_WAIT_ACK,
	TX_NO_ACK
} T_TX_STATE;

// Local and Global Variables
extern T_CONFIG g_config;
extern bool gs_bdebug;
extern uint32_t get_unix_time( void );
extern void set_unix_time( uint32_t );
extern bool erase_records_flag;
extern uint8_t key_ring_index;
extern uint8_t key_ring[7];
extern uint8_t uart_rx_ubyte;

struct {
	T_ACK_STATE state;
	uint32_t pkt_id;
} rec_ack = { .state = ACK_IDLE, .pkt_id = 0xFFFF };

#define PKT_FIFO_MASK	0x3F
T_PACKET packet[PKT_FIFO_MASK+1];		//allow up to 64 packets to queue (also saves a partial history of recent_pkts = ((PKT_FIFO_MASK+1)-(pkt_head-pkt_tail)&PKT_FIFO_MASK)
bool new_connection = false;
bool record_queue_en = false;
bool remote_trace_en = false;
static uint8_t g_pkt_head = 0, g_pkt_tail = 0;

#define DETACH_0x88_TO							(APP_TIMER_TICKS(12000, APP_TIMER_PRESCALER))
#define DETACH_KEY_TO							(APP_TIMER_TICKS(6000, APP_TIMER_PRESCALER))
static app_timer_id_t local_detach_timer_id = 	NULL;
#define START_SEC_CODE 							0x88
uint8_t	connected_and_bonded = 					0; //in DevMagPer.c as extern for lightshow semiphore
static T_SECURITY_STATE sec_con_state = 		NO_CONNECTION;

extern	uint8_t tx_payload[ MAX_PKT_PAYLOAD ];
extern	int8_t tx_pay_len;	//default no response
extern 	T_PKT_TYPES tx_type;	//T_PKT_TYPES type;

//function prototypes
extern bool ble_uart_tx( uint8_t *, uint8_t );
extern void terminate_connection(void);
extern void delete_all_bonds(void);
extern void delete_current_bond(void);

static bool queue_packet( T_PKT_TYPES pkt_type, uint8_t * data , uint8_t data_len );
static bool ble_transfer_packets( void );
static bool packetize_record( void );
static void flush_queue( void );
static void update_queue( void );
static uint8_t get_fifo_length( void );
static void secure_connect_sm( uint8_t * data, uint8_t len );

//! \fn
/// \brief
/// \param
/// \return .
///
void init_comms( app_timer_id_t p_detach_timer )
{
	local_detach_timer_id = p_detach_timer;
}

//! \fn
/// \brief
/// \param
/// \return .
///
void detach_handler(void * p_context )
{
	// If someone connects to us and does not send the Start Code within the specified time
	// then disconnect, it's not our App!!! Also disconnect if the KEY does not come shortly
	// after receiving the Start Code.
	app_trace_puts(DEBUG_MED, "[DETACH] Timeout\r");
	terminate_connection();
}


//! \fn
/// \brief
/// \param
/// \return .
///
void flag_new_comms( void )
{
	new_connection = true;
	connected_and_bonded = false;
	record_queue_en = false;
	remote_trace_en = false;
	rec_ack.state = ACK_IDLE;
	sec_con_state = NO_CONNECTION;	//restart state machine on new connection
	app_timer_stop(local_detach_timer_id);
	app_timer_start(local_detach_timer_id, DETACH_0x88_TO, NULL);	
	flush_queue();
	if( manufacture_mode() )
	{	//Operating without an encrypted connection, force User to present Access Key:
		key_ring[key_ring_index] = 0;
	}
	app_timer_stop(local_detach_timer_id);
	sec_con_state = SECURE_CONNECTION;
}

//
//! parse_msg()
/// Data has been received on the on the UART_OVER_BTLE_UART_RX_RX service. Decode the meaning and queue response
/// \param
/// \return .
///
void parse_msg( T_PACKET * rx_msg )
{
	static uint8_t nextID = 0xFF;
	T_PKT_TYPES tx_type;			//T_PKT_TYPES type;tx_type;
//	uint8_t tx_payload[ MAX_PKT_PAYLOAD ];
//	int8_t tx_pay_len = -1;	//default no response
	int8_t rx_pay_len = rx_msg->pkt_len - PKT_HEADER_LEN;
	Union32 temp;

	uart_rx_ubyte = rx_msg->id;
	app_trace_log(DEBUG_MED, "HRM1 CMD = %01u\r", rx_msg->id);
	
	if( rx_msg->pkt_len < PKT_HEADER_LEN )
	{	//pkt length is shorter than the minimum message length: 2
		if (gs_bdebug) app_trace_puts(DEBUG_MED, "[PARSE] Pkt Len Error\r");
		tx_type = NACK;	//any undefined case will return a NACK
		tx_payload[0] = PKT_LEN_ERROR;
		tx_pay_len = 1;
	}
	else
	{
		//check that the ID sequence is correct
		if( new_connection == true )
		{
			//Re-base sequence number
			nextID = rx_msg->id;
			new_connection = false;
		}
		else if( rx_msg->id != ++nextID )
		{
			//a packet was likely dropped!!!!
			if (gs_bdebug) app_trace_log(DEBUG_MED, "[PARSE] ID Seq Err: %01u != %01u\r", nextID, rx_msg->id );
			
			//tx_type = NACK;
			//tx_payload[0] = PKT_SEQ_ERROR;
			//tx_payload[1] = nextID;
			//tx_pay_len = 2;
			//queue_packet( tx_type, tx_payload, tx_pay_len );

			nextID = rx_msg->id;
		}
//		uart_rx_ubyte = rx_msg->payload[0];
//		app_trace_log(DEBUG_MED, "HRM1 CMD = %01u\r", rx_msg->payload[0]);
		switch( rx_msg->type )
		{
			case NACK:	//Something didn't Work
				switch ( rx_msg->payload[0] )
				{
					case PKT_SEQ_ERROR:
						{
							if (gs_bdebug) app_trace_log(DEBUG_MED, "[NACK] Resend Packet %01u", rx_msg->payload[1]);
							uint16_t queue_index = rx_msg->payload[1]&PKT_FIFO_MASK;
							if( packet[queue_index].id == rx_msg->payload[1] )
							{
								ret_code_t res = ble_uart_tx( (uint8_t *) &(packet[queue_index]), packet[queue_index].pkt_len );
								if( res != NRF_SUCCESS )
								{
									if (gs_bdebug) app_trace_log(DEBUG_MED, ": Send Err %01u\r\r", res);
									tx_type = NACK;
									tx_payload[0] = PKT_TX_ERR;
									tx_payload[1] = rx_msg->id;
									tx_pay_len = 2;
								}
								else
								{
									if (gs_bdebug) app_trace_puts(DEBUG_MED, "\r\r");
								}
							}
							else
							{
								//Packet number no longer exists
								if (gs_bdebug) app_trace_puts(DEBUG_MED, ": No Longer Exists\r\r");
								tx_type = NACK;
								tx_payload[0] = PKT_SEQ_ERROR;
								tx_payload[1] = rx_msg->id;
								tx_pay_len = 2;
							}
						}
						break;

					case PKT_LEN_ERROR:
						//Host Device believes the packet length was in error
						//not sure what todo:
						if (gs_bdebug) app_trace_puts(DEBUG_MED, "[NACK] Packet Length Error\r");
						break;

					case PKT_TYPE_UNRECOG:
						//Host Device did not recognize the Packet Type
						//not sure what todo:
						if (gs_bdebug) app_trace_puts(DEBUG_MED, "[NACK] Packet Type Error\r");
						break;

					default:
						if (gs_bdebug) app_trace_puts(DEBUG_MED, "[NACK] Unknown\r");
						break;
				}

				break;

			case ACK:	//Peer Device is ACKing transmitted Data
				if( rec_ack.state == WAITING )
				{
					if( rx_msg->payload[0] == rec_ack.pkt_id )
					{
						//ACKing the final packet in a Record
						rec_ack.state = RECEIVED;
						if (gs_bdebug) app_trace_log(DEBUG_LOW, "[ACK] Record Accepted @%01u\r", getSystemTimeMs());
					}
					else
					{
						rec_ack.state = ID_ACK_FAIL;
						if (gs_bdebug) app_trace_log(DEBUG_MED, "[ACK] ID %01u Does Not Match %01u\r", rx_msg->payload[0], (unsigned int)rec_ack.pkt_id);
					}
				}
				else
				{
					if (gs_bdebug) app_trace_puts(DEBUG_LOW, "[ACK] ???\r");
				}

				break;

			case REPORT_ATTRIBUTES:
				{
					if(sec_con_state != SECURE_CONNECTION)
					{
						terminate_connection();
						break;
					}
					if (gs_bdebug)
					{
						if( rx_pay_len == ATTRIBUTE_SIZE )
						{
							app_trace_log(DEBUG_LOW, "Reporter %01u Atts Request:\r", rx_msg->payload[0]);	// id
							app_trace_log(DEBUG_LOW, "  Ind_Des:	%01u\r", rx_msg->payload[1]);	// T_INDEP_DESC
							app_trace_log(DEBUG_LOW, "  Ind_Scl:	%01u\r", rx_msg->payload[2]);	// count
							app_trace_log(DEBUG_LOW, "  Data_Des:	%01u\r", rx_msg->payload[3]);	// T_DATA_DESC
							app_trace_log(DEBUG_LOW, "  Type: 	%01u\r", rx_msg->payload[4]);		// T_DATA_TYPE
							app_trace_log(DEBUG_LOW, "  FSR:  	%01u\r", rx_msg->payload[5]);		// T_DATA_FSR
							app_trace_log(DEBUG_LOW, "  Fields:	%01u\r", rx_msg->payload[6]); 	// T_DATA_FIELDS
							app_trace_log(DEBUG_LOW, "  Samples:	%01u\r", rx_msg->payload[7]|(rx_msg->payload[8]<<8));	// count
							app_trace_log(DEBUG_LOW, "  Recs_Rep:	%01u\r", rx_msg->payload[9]|(rx_msg->payload[10]<<8));	// count
						}
					}
					/* 		
							Reporter 0 Atts Request:
							Ind_Des:	1
							Ind_Scl:	15
							Data_Des:	2
							Type: 	1
							FSR:  	0
							Fields:	0
							Samples:	4
							Recs_Rep:	0
					*/
					T_REPORT_ERR rep_err = set_reporter( (T_REPORT_ATTRIBUTES *)rx_msg->payload, rx_pay_len );
					if( rep_err == NO_ERROR )
					{
						if( rx_pay_len == ATTRIBUTE_SIZE )
						{
							//New atts were Requested and Accepted
							set_config_update_flag();	//store new values in flash when we return to main
						}

						if (gs_bdebug)
						{
							app_trace_puts(DEBUG_MED, "Report Atts Are: ");
							for(uint i=0; i<ATTRIBUTE_SIZE; i++ )
							{
								app_trace_log(DEBUG_MED, "%02u, ", rx_msg->payload[i]);
							}
							app_trace_puts(DEBUG_MED, "\r");
						}

						tx_type = REPORT_ATTRIBUTES;
						for( uint i=0; i<ATTRIBUTE_SIZE; i++ )
						{
							tx_payload[i] = rx_msg->payload[i];	//copy the accepted attributes to send back
						}
						tx_pay_len = ATTRIBUTE_SIZE;
					}
					else
					{
						if (gs_bdebug) app_trace_log(DEBUG_MED, "Attribute Update Failed: %01u\r", rep_err);
						//Something failed, return the Error Code
						tx_type = NACK;
						tx_payload[0] = ATTRIBUTE_ERROR;
						tx_payload[1] = rx_msg->id;
						tx_payload[2] = rep_err;
						tx_pay_len = 3;
					}
				}
				break;

			case REPORT_CONTROL:	//Host Device is Requesting Reporters to be turn on/off
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				if (gs_bdebug) app_trace_puts(DEBUG_LOW, "Reporters Req: ");
				if( rx_pay_len > 0 )
				{
					//turn on/off the Reporters that are indicated by data byte 0
					if (gs_bdebug) app_trace_log(DEBUG_LOW, "0x%02X\r", rx_msg->payload[0]);
					enable_reporters(rx_msg->payload[0]);	//return the Active/Inactive flags
				}
				else
				{
					if(gs_bdebug) app_trace_puts(DEBUG_LOW, "\r");
				}

				//reply with active/inactive Reporters
				tx_type = REPORT_CONTROL;
				tx_payload[0] = get_active_reporters();
				if (gs_bdebug) app_trace_log(DEBUG_MED, "Reporters Active: 0x%02X\r", tx_payload[0]);
				tx_pay_len = 1;

				break;

			case TRANSMIT_CONTROL:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				
				//Only modify transmit flow control if the payload is exactly 1 byte, otherwise only return data
				if( rx_pay_len == 1 )
				{
					if (gs_bdebug) app_trace_log(DEBUG_MED, "Transmit Ctrl Req: 0x%02X\r", rx_msg->payload[0]);
					//turn on/off the Reporters that are indicated by data byte 0
					if(rx_msg->payload[0] == true )
					{
						//start sending records
						record_queue_en = true;
					}
					else
					{
						//stop record transmission
						record_queue_en = false;
					}
				}
				else 
				{
					//length is incorrect to adjust the Flow Control Flag. Do not touch it, just return lengths
					if (gs_bdebug) app_trace_log(DEBUG_MED, "Transmit Ctrl Stat: %01u\r", record_queue_en);
				}

				//send back the total number of records used in memory
				tx_type = TRANSMIT_CONTROL; 
				temp.u32 = get_total_record_cnt();	//number of queued Records in memory
				tx_payload[0] = temp.u8[0];
				tx_payload[1] = temp.u8[1];

				temp.u32 = records_byte_cnt();	//transmit byte cnt of queued Records.
				tx_payload[2] = temp.u8[0];
				tx_payload[3] = temp.u8[1];
				tx_payload[4] = temp.u8[2];
				tx_payload[5] = temp.u8[3];
				tx_pay_len = 6;
				if (gs_bdebug)
				{
					app_trace_puts(DEBUG_LOW, "TC Response: ");
					for( int i=0; i<tx_pay_len; i++ )
					{
						app_trace_log(DEBUG_LOW, "0x%02X, ", tx_payload[i]);
					}
					app_trace_puts(DEBUG_LOW, "\r");
				}

				//send back the individual lengths of all 8 reporters
				//for( uint i=0; i<REP_CNT; i++ ) {
				//temp.u16 = get_report_record_cnt( i );
				//tx_payload[2*i] = temp.u8[0];
				//tx_payload[2*i+1]= temp.u8[1];
				//}
				//tx_pay_len = 2*REP_CNT;

				break;

			case TIMERD:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				if (gs_bdebug) app_trace_puts(DEBUG_MED, "Time Read\r");

				//Send back current time
				tx_type = TIMERD;
				temp.u32 = get_unix_time();
				for (int i = 0; i<sizeof(uint32_t); i++)
				{
					tx_payload[i] = temp.u8[i];
				}
				tx_pay_len = sizeof(uint32_t);

				break;

			case TIMEWR:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				if (gs_bdebug) app_trace_puts(DEBUG_MED, "Time Set\r");

				for (int i = 0; i<sizeof(uint32_t); i++)
				{
					temp.u8[i] = rx_msg->payload[i];
					tx_payload[i] = temp.u8[i];
				}

				if( temp.u32 != get_unix_time() )
				{
					reporter_time_rebase( temp.u32, get_unix_time() );
					set_unix_time( temp.u32 );
					//unix2Clock( temp.u32 );
				}

				//Respond with new time
				tx_type = TIMEWR;
				tx_pay_len = sizeof(uint32_t);

				break;

			case USERUUIDRD:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				if (gs_bdebug) app_trace_puts(DEBUG_LOW, "User UUID RD\r");

				// reading
				tx_type = USERUUIDRD;
				for (int i = 0; i<user_UUID_LEN; i++)
				{
					tx_payload[i] = g_config.user_UUID[i];
				}
				tx_pay_len = user_UUID_LEN;
				break;

			case USERUUIDWR:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				if (gs_bdebug) app_trace_puts(DEBUG_MED, "User UUID WR\r");

				// writing
				for (int i = 0; i<user_UUID_LEN; i++)
				{
					g_config.user_UUID[i] = rx_msg->payload[i];
					tx_payload[i] = g_config.user_UUID[i];
				}

				set_config_update_flag();	//store new values in flash when we return to main

				//Respond with new UUID
				tx_type = USERUUIDWR;
				tx_pay_len = user_UUID_LEN;

				break;

			case FRAMERD:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				if (gs_bdebug) app_trace_puts(DEBUG_LOW, "Frame RD\r");

				// reading
				tx_type = FRAMERD;
				for (int i = 0; i<device_FRAMEID_LEN; i++)
				{
					tx_payload[i] = g_config.device_frame_ID[i];
				}
				tx_pay_len = device_FRAMEID_LEN;

				break;

			case FRAMEWR:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				if (gs_bdebug) app_trace_puts(DEBUG_MED, "Frame WR\r");

				// writing
				for (int i = 0; i<device_FRAMEID_LEN; i++)
				{
					g_config.device_frame_ID[i] = rx_msg->payload[i];
					tx_payload[i] = g_config.device_frame_ID[i];
				}

				set_config_update_flag();	//store new values in flash when we return to main

				//Respond with new UUID
				tx_type = FRAMEWR;
				tx_pay_len = device_FRAMEID_LEN;

				break;

			case BLE_TX_POWER:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				
				//adjustment to BLE transmit power
				//accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm.
				//(-40 dBm only for NRF52, -30 dBm only for NRF51).
				if( (int8_t)rx_msg->payload[0] >= 2 ) temp.s32 = 4;
				else if( (int8_t)rx_msg->payload[0] >= -2 ) temp.s32 = 0;
				else if( (int8_t)rx_msg->payload[0] >= -6 ) temp.s32 = -4;
				else if( (int8_t)rx_msg->payload[0] >= -10 ) temp.s32 = -8;
				else if( (int8_t)rx_msg->payload[0] >= -14 ) temp.s32 = -12;
				else if( (int8_t)rx_msg->payload[0] >= -18 ) temp.s32 = -16;
				else if( (int8_t)rx_msg->payload[0] >= -25 ) temp.s32 = -20;
				else temp.s32 = -40;

				sd_ble_gap_tx_power_set( temp.s8[0] );

				if (gs_bdebug) app_trace_log(DEBUG_MED, "New TX Power: %01u dBm\r", temp.s8[0]);

				//reply with active/inactive Reporters
				tx_type = BLE_TX_POWER;
				tx_payload[0] = temp.u8[0];
				tx_pay_len = 1;

				break;

			case LED_ID_RX:

				if (gs_bdebug) app_trace_puts(DEBUG_LOW, "LED_ID_RX\r");
			
				if( rx_pay_len == (KEY_LEN+1) )
				{	//message is from manufacturing App
					g_config.ship_mode = rx_msg->payload[KEY_LEN];
					rx_pay_len -= 1;	//secure_connect_sm expects len = KEY_LEN
				}
				else if( rx_pay_len == KEY_LEN )
				{	//message is from User App	
					if( g_config.ship_mode != 0x00 )
					{
						app_trace_puts(DEBUG_MED, "Ship Flag Cleared\r");
						g_config.ship_mode = 0x00;
						set_config_update_flag();
					}
				}
				
				secure_connect_sm(rx_msg->payload, rx_pay_len);

				//ack with key
				tx_type = LED_ID_RX;
				tx_pay_len = 0;
				break;

			case NUKE_BOND:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				
				if ( rx_msg->payload[0] == 22 ) {
					delete_all_bonds();
				
					//ack with key
					tx_type = NUKE_RECORDS;
					tx_pay_len = 0;
				}
				else {
					tx_type = NACK;
					tx_payload[0] = INVALID_CODE;
					tx_payload[1] = rx_msg->id;
					tx_pay_len = 2;
				}
				break;
				
			case NUKE_RECORDS:
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}
				
				app_trace_puts(DEBUG_HIGH, "Records Nuked!!!\r");
				
				if( rx_pay_len != 1 ) {
					//payload size is not correct
					tx_type = NACK;
					tx_payload[0] = DATA_LEN_ERROR;
					tx_payload[1] = rx_msg->id;
					tx_pay_len = 2;
				}
				else if ( rx_msg->payload[0] == 22 ) {
					record_queue_en = false;	//Stop sending records
					erase_reporter_config(); 
					
					//Flag to destroy records already saved to memory. This takes too long to execute all at once. So
					//it will be executed through main() while the processor is not busy.
					erase_records_flag = true;
		
					//ack with type
					tx_type = NUKE_RECORDS;
					tx_pay_len = 0;
				}
				else {
					tx_type = NACK;
					tx_payload[0] = INVALID_CODE;
					tx_payload[1] = rx_msg->id;
					tx_pay_len = 2;
				}
				break;
				
			case CHECK_KEY:
				if (gs_bdebug) app_trace_puts(DEBUG_LOW, "Check Key Read\r");
				app_trace_puts(DEBUG_LOW, "LEDID State 0\r");
			
				//#define JUST_CONNECT_ALREADY	//todo: remove after testing
				#ifdef JUST_CONNECT_ALREADY
					app_timer_stop(local_detach_timer_id);
					sec_con_state = SECURE_CONNECTION;
					tx_type = CHECK_KEY;
					tx_payload[0] = 1;
					tx_pay_len = 1;
				#else
					//Send back current key state for this bond	
					if( rx_pay_len == 1 ) 
					{		
						if ( rx_msg->payload[0] == 0 )
						{				
							tx_type = CHECK_KEY;
							tx_payload[0] = key_ring[key_ring_index];
							tx_pay_len = 1;
							app_trace_log(DEBUG_MED, "[KR]: NUS Xmit key_ring[%01u] = 0x%02X @\r", key_ring_index, key_ring[key_ring_index], getSystemTimeMs() );
						}
						else
						{
							tx_type = NACK;
							tx_payload[0] = PKT_TYPE_UNRECOG;
							tx_payload[1] = rx_msg->id;
							tx_pay_len = 2;
							app_trace_log(DEBUG_MED, "INVALID CMD");
						}
					}
					else
					{
						tx_type = NACK;
						tx_payload[0] = PKT_LEN_ERROR;
						tx_payload[1] = rx_msg->id;
						tx_pay_len = 2;
						app_trace_log(DEBUG_MED, "INVALID CHK LEN");
					}
				#endif
				break;
			
			case TRACE_CONTROL:
				//turn remote trace on/off
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}

				if (gs_bdebug) app_trace_log(DEBUG_LOW, "Trace Ctrl: %01u\r", rx_msg->payload[0]);

				if(rx_msg->payload[0] == true )
				{	//start sending trace logs
					remote_trace_en = true;
				}
				else
				{	//stop sending logs
					remote_trace_en = false;
				}
				break;
				
			case MANUFACT_CONTROL:
				{
					if(sec_con_state != SECURE_CONNECTION)
					{
						terminate_connection();
						break;
					}		
										
					if( rx_pay_len == 1 )
					{
						if ( rx_msg->payload[0] == 1 )
						{
							//kickoff test procedure:
							//tests take some time, so flag main loop to do it
							set_hw_test_request();
						}
					}
					
					//get copy of Test Status
					T_MANUFACT_TEST_RESULTS res = get_hw_test_status();
					
					tx_type = MANUFACT_CONTROL;
					tx_payload[0] = res.state;
					tx_payload[1] = (uint8_t) res.hw_flags;
					tx_payload[2] = (uint8_t) (res.hw_flags>>8);
					tx_pay_len = 3;
					
					app_trace_log(DEBUG_MED, "Manu Ctrl: St-%01u Fl-0x%02X%02X\r", tx_payload[0], tx_payload[2], tx_payload[1]);
				}
				break;
				
			case SECURITY_KEY_WR:
				//turn remote trace on/off
				if(sec_con_state != SECURE_CONNECTION)
				{
					terminate_connection();
					break;
				}

				if (gs_bdebug) 
				{	
					app_trace_puts(DEBUG_LOW, "SK Data:");
					for( int i=0; i<rx_pay_len; i++ )
					{
						app_trace_log(DEBUG_LOW, " %01u", rx_msg->payload[i]);
					}
					app_trace_puts( DEBUG_LOW, "\r");
				}

				if( rx_pay_len == KEY_LEN )
				{	//update Security Key
					for( int i=0; i<KEY_LEN; i++ )
					{
						tx_payload[i] = copy_security_key[i] = g_config.security_key[i] = rx_msg->payload[i];
					}
					
					//invalidate all bonds that came before now
					delete_all_bonds();			
					
					//store new values in flash after returning to main
					set_config_update_flag();	
					
					tx_type = SECURITY_KEY_WR;	//0x17
					tx_pay_len = KEY_LEN;
				}
				else
				{
					tx_type = NACK;
					tx_payload[0] = PKT_LEN_ERROR;
					tx_payload[1] = rx_msg->id;
					tx_pay_len = 2;
					app_trace_log(DEBUG_MED, "INVALID KEY LEN");
				}
				break;
				
			default:    //Where's The Fun?
				if (gs_bdebug) app_trace_puts(DEBUG_MED, "Respond: Where's The Fun?\r");
				tx_type = NACK;
				tx_payload[0] = PKT_TYPE_UNRECOG;
				tx_payload[1] = rx_msg->id;
				tx_pay_len = 2;

				break;
		} //end switch
	} //end else

	if( tx_pay_len >= 0 )
	{
		queue_packet( tx_type, tx_payload, tx_pay_len );
		ble_transfer_packets();		//pass queued messages to the radio
	}
}

//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t record_pkts2_send = 0;
bool packetize_record( void )
{
	bool res = false;
	T_RECORD rec = { .hdr = { .rec_len = 0 } };
	
	if( get_next_record( &rec ) != true )
	{
		if (gs_bdebug) app_trace_puts(DEBUG_MED, "[Queue] Rec Pop Fail\r");
		res = false;
	}
	else if( rec.hdr.rec_len > 0 )
	{
		uint8_t * rec_ptr = (uint8_t *) &rec.hdr;	//point to first transmitted byte in Record
		bool start_packet = true;
		T_PKT_TYPES pkt_type;
		uint8_t payload_len;
		uint16_t remain_len;

		uint8_t num_pkts_2_add = (rec.hdr.rec_len/MAX_PKT_PAYLOAD) + 1;

		if( (get_fifo_length() + num_pkts_2_add) > PKT_FIFO_MASK )
		{
			if (gs_bdebug) app_trace_puts(DEBUG_MED, "[Queue] Too many packets!\r");
			rec_ack.pkt_id = 0xFFFF;	//prevent an ACK from accidentally updating the Log pointers
			return false;	//not enough space in the FIFO to add this record
		}

		//force record into sub-packets and queue piecemeal transmission
		remain_len = rec.hdr.rec_len;
		while( remain_len > 0 )
		{
			//first packet type is different to indicate the beginning of a record
			if( start_packet == true )
			{
				start_packet = false;
				pkt_type = START_OF_RECORD;
			}
			else
			{
				pkt_type = CONTINUE_RECORD;
			}

			//queue packets until record is complete
			if( remain_len >= MAX_PKT_PAYLOAD )
			{	//send next 18 bytes of record
				payload_len = MAX_PKT_PAYLOAD;
			}
			else
			{	//send final bytes of Record
				payload_len = remain_len;
			}

			if( queue_packet( pkt_type, rec_ptr, payload_len) != true )
			{
				if (gs_bdebug) app_trace_puts(DEBUG_MED, "[Queue] Full\r");
				//no more room in queue (This is already checked for, so it shouldn't occur. But better safe than sorry)
				rec_ack.pkt_id = 0xFFFF;	//prevent an ACK from accidentally updating the Record pointers
				return res;
			}

			rec_ptr += payload_len;
			remain_len -= payload_len;
		}
		
		//Print the first X,Y,Z out of memory
		//app_trace_log(DEBUG_MED, "REC %02u: 0x%04X, 0x%04X, 0x%04X\r", rec.hdr.id, *((int16_t *)rec.data), *((int16_t *)rec.data+2), *((int16_t *)rec.data+4) );

		//Check if an ACK is required for this record
		if( rec_ack_req(rec.hdr.report_inst) ) {
			//host must ACK the final packet in the transmitted Record in order for the next Record to be queued.
			rec_ack.state = WAITING;
			rec_ack.pkt_id = packet[(g_pkt_head-1)&PKT_FIFO_MASK].id;	//the packet id of the last packet in the record.
		}
		else {
			//No ACK required to speed up data transfer
			rec_ack.state = ACK_IDLE;
		}
		
		res = true;
	}
	else
	{
		if (gs_bdebug) app_trace_puts(DEBUG_LOW, "[Queue] No Records\r");
	}

	return res;
}

//! \fn
/// \brief
/// \param
/// \return .
///
#define RECORD_RETRY	5
void ble_send_nus_data( void )
{
	static TTASK_TIMER timeout = { 0, PACKET_TIMEOUT };
	static T_TX_STATE rec_queue_state = TX_OFF;
	static int8_t retries = RECORD_RETRY;

	if( record_queue_en == false )
	{	//requested to stop
		rec_queue_state = TX_OFF;
	}
			
	//Handle the sending of a full Record
	switch( rec_queue_state )
	{
		case TX_OFF:	//OFF
			rec_ack.state = ACK_IDLE;
			if( record_queue_en == true )
			{
				rec_queue_state = TX_START;
			}
			break;
			
		case TX_NO_ACK:	
			//Wait for Packets to send (No need for ACK, looking for a high data rate)
			//When Queue is near empty, start loading the next packets
			if( get_fifo_length() < 6 ) {
				//Queue near empty, load it up again
				rec_queue_state = TX_START;
			}
			break;

		case TX_START:	//Queue up first Record
			if( !data_log_empty() )
			{
				if( packetize_record() == true )
				{
					if( gs_bdebug ) app_trace_puts(DEBUG_LOW, "[COMMS] Record Queued to TX\r");
					
					if( rec_ack.state == WAITING ) 
					{
						//Need to wait for an ACK from the Host
						re_arm_task_timer( timeout );
						retries = RECORD_RETRY;
						rec_queue_state = TX_WAIT_ACK;
					}
					else 
					{
						//Wait til packet goes out, but don't wait for Host to ACK. Update tail now, since we don't need
						//to wait for the ACK
						if( inform_rec_tranceived() != true ) 
						{
							//update failed!
						}
						
						rec_queue_state = TX_NO_ACK;
					}
				}
			}
			else
			{
				//Nothing left to transmit. Turn Transmit Control Off? Per Andrew: "NO!"
				//record_queue_en = false;
				//rec_queue_state = TX_OFF;
			}
			break;

		case TX_WAIT_ACK:	//Wait to Queue next Record
			//Must wait for Data Records to be ACK'd to prevent losing data when sending to
			//a "dead" host (BLE transport is active, but the App has crashed).
			switch( rec_ack.state )
			{
				case ACK_IDLE:
					//if any data goes into memory before the timeout, send it
					if( retries <= 0 )
					{
						if( gs_bdebug ) app_trace_puts(DEBUG_MED, "[COMMS] Lack of ACK, TC Off.\r");
						record_queue_en = false;	//5 failures in a row. Turn off transmit control
						rec_ack.state = ACK_IDLE;
						rec_queue_state = TX_START;
					}
					else if( !data_log_empty() )
					{
						if( packetize_record() == true )
						{
							re_arm_task_timer( timeout );
							
							if( gs_bdebug ) app_trace_log(DEBUG_LOW, "[COMMS] Record Queued @%01u\r", getSystemTimeMs());
							//tx_timestamp = systime_ms;
							
							if( rec_ack.state != WAITING ) {
								//Wait til packet goes out, but don't wait for Host to ACK
								rec_queue_state = TX_NO_ACK;
							}
						}
					}
					else {
						//Nothing left to transmit
					}
					break;

				case WAITING:
					if( task_time( timeout ) )
					{
						//turn off TX control
						//record_queue_en = false;
						retries--;
						rec_ack.state = ACK_IDLE;
						if( gs_bdebug ) app_trace_log(DEBUG_MED, "[COMMS] No Record ACK @%01u\r", getSystemTimeMs());
					}
					break;

				case RECEIVED:
					//If a Record Receipt has been received, update the tail pointer and prepare the next packet
					if( inform_rec_tranceived() == true )
					{
						if( retries != RECORD_RETRY )
						{
							//successful sent Record, reset number of retries
							//some messages were lost and needed to resend
							retries = RECORD_RETRY;
						}
						rec_ack.state = ACK_IDLE;

						if( !data_log_empty() )
						{
							if( packetize_record() == true )
							{
								re_arm_task_timer( timeout );
								
								if( gs_bdebug ) app_trace_puts(DEBUG_LOW, "[COMMS] Next Record Queued\r");
								//tx_timestamp = systime_ms;
								
								if( rec_ack.state != WAITING ) {
									//Wait til packet goes out, but don't wait for Host to ACK
									rec_queue_state = TX_NO_ACK;
								}
							}
						}
					}
					break;

				case ID_ACK_FAIL:
					retries--;
					rec_ack.state = ACK_IDLE;
					if( gs_bdebug ) app_trace_puts(DEBUG_MED, "[COMMS] ACK Not Valid\r");
					break;

				default:
					rec_ack.state = ACK_IDLE;
					break;
			}
			break;

		default:
			rec_queue_state = TX_OFF;
			break;
	}
	
	//If remote console is enabled, queue trace messages to send
	if( remote_trace_en == true ) 
	{
		int msg_len;
		char tx_msg[ MAX_PKT_PAYLOAD ];
		
		if( get_fifo_length() < PKT_FIFO_MASK )
		{	//there is space available in the queue
			msg_len = app_trace_remote( tx_msg, MAX_PKT_PAYLOAD );
			if( msg_len > 0 )
			{
				queue_packet( TRACE_CONTROL, (uint8_t *)tx_msg, msg_len );
			}
		}
	}

	// Send Off as many of the queued packets as the radio will take
	if( ble_transfer_packets() == false )
	{	//packets are not transferring, queue was flushed:
		remote_trace_en = false;
		record_queue_en = false;	//Turn off transmit control
		rec_ack.state = ACK_IDLE;	//reset state variables
		rec_queue_state = TX_START;
	}
		
	return;
}

//! \fn
/// \brief
/// \param
/// \return .
///
bool secure_connection( void )
{
	if( sec_con_state == SECURE_CONNECTION )
		return true;
	else
		return false;
}

//! \fn
/// \brief
/// \param
/// \return .
///
static void secure_connect_sm( uint8_t * data, uint8_t d_len )
{
	bool key_valid = true;
	extern uint8_t device_index_ring;
	extern uint8_t gu_prev_battery_state;
	
	// Start the LED light show if connected and LED_ID_KEY has been sent
	switch ( sec_con_state )
	{
		case NO_CONNECTION:
			if ( data[0] == START_SEC_CODE )
			{
				app_trace_puts(DEBUG_LOW, "[SEC_STATE] Rx 0x88\r");
				app_trace_log(DEBUG_LOW, "[KR]: connected_and_bonded = 0x%02X, key_ring[%01u] = 0x%02X\r", connected_and_bonded, key_ring_index, key_ring[key_ring_index] );
				app_timer_stop(local_detach_timer_id);
				if((connected_and_bonded == 1) && (key_ring[key_ring_index] == 1)) // this is a bonded connect so enable services and we are done
				{
					// Previously Bonded with Device
					app_trace_puts(DEBUG_LOW, "[SEC_STATE] Bonded\r");
					connected_and_bonded = 0;
					sec_con_state = SECURE_CONNECTION;
				}
				else
				{	//give more time for KEY to be trasnsferred		
					app_timer_start(local_detach_timer_id, DETACH_KEY_TO, NULL);		
					sec_con_state = GET_KEY_STATE;
				}
			}
			else
			{
				//delete_current_bond();
				app_trace_log(DEBUG_MED, "[SEC_STATE] Wrong Code 0x%02X\r", data[0]);
				terminate_connection();
			}
			break;

		case GET_KEY_STATE:	// time out routine run while waiting for LED_ID_NUMB
			app_trace_puts(DEBUG_LOW, "[SEC_STATE] Key\r");
		
			// Code and Key have been received, Detach Timer can be stopped
			app_timer_stop(local_detach_timer_id);
		
			//Test Received Key
			if( d_len == KEY_LEN ) 
			{
				int i;
				for( i=0; i<KEY_LEN; i++ )
				{
					if( copy_security_key[i] != data[i] )
					{
						app_trace_log(DEBUG_MED, "[SEC_STATE] [%01u]: %01u != %01u\r", i, data[i], copy_security_key[i]);
						key_valid = false;
					}
				}
			}
			else
			{
				app_trace_puts(DEBUG_MED, "[SEC_STATE] Incorrect Len\r");
				key_valid = false;
			}
				
			if( key_valid )
			{
				app_trace_puts(DEBUG_LOW, "[SEC_STATE] COMPLETE\r");
				gu_prev_battery_state = BATTERY_UNDEF_STATE; // Reset the led state to it's former magenta glory
				sec_con_state = SECURE_CONNECTION;
				
				//update config variables
				if(device_index_ring != 0)
					key_ring_index = device_index_ring - 1;
				else
					key_ring_index = 6;
				key_ring[key_ring_index] = 1;
				g_config.key_ring[key_ring_index] = key_ring[key_ring_index];
				g_config.device_index_ring = device_index_ring;
				set_config_update_flag();	//save new data next time through main()
				app_trace_log(DEBUG_LOW, "[KR]: NUS Xmit key_ring[%01u] = 0x%02X, device_index_ring = %01u\r", key_ring_index, key_ring[key_ring_index], device_index_ring );
			}
			else
			{
				app_trace_puts(DEBUG_MED, "[SEC_STATE] KEY failed\r");
				//delete_current_bond();
				terminate_connection();
			}
			break;
			
		case SECURE_CONNECTION: // Finished. Enable services. Now looking for disconnect.

			//app_trace_log(DEBUG_LOW, "LEDID State 4\r");
			break;
		
		default:
			sec_con_state = NO_CONNECTION;
			break;
	}
}

//! \fn
/// \brief
/// \param
/// \return .
///
static uint8_t packet_id = 0;
static int8_t unsent_fifo_length = 0;
static bool queue_packet( T_PKT_TYPES pkt_type, uint8_t * data , uint8_t data_len )
{	
	int i;
	uint8_t pkt_len = data_len + 1;
	
	
	if( unsent_fifo_length > PKT_FIFO_MASK )
	{	//queue is full
		if( !remote_trace_en ) app_trace_log(DEBUG_MED, "[QUEUE] Insufficient Space\r");
		return false;	//can't add anymore to queue
	}
	else if( data_len > MAX_PKT_PAYLOAD )
	{	//packet too big!
		if( !remote_trace_en ) app_trace_log(DEBUG_MED, "[QUEUE] Len Error %01u\r", data_len);
		return false;	
	}
		
	//Block ISR during packet update ( parse ISR can also call queue_packet )
	CRITICAL_REGION_ENTER();
	
	packet[g_pkt_head].id = packet_id++;
	packet[g_pkt_head].type = pkt_type;
	for(i=0; i<data_len; i++)
	{
		packet[g_pkt_head].payload[i] = *(data+i);
	}
	packet[g_pkt_head].pkt_len =  pkt_len;
	
	if( !remote_trace_en ) 
	{
//		app_trace_log(DEBUG_MED, "[QUEUE] ID:%01u LN:%01u", packet_id, pkt_len);
//		for( i=0; i<data_len; i++ )
//		{
//			app_trace_log(DEBUG_LOW, " 0x%02X", packet[g_pkt_head].payload[i]);
//		}
//		app_trace_puts(DEBUG_MED, "\r");
	}

	g_pkt_head = (g_pkt_head+1)&PKT_FIFO_MASK;
	unsent_fifo_length++;
	
	CRITICAL_REGION_EXIT();

	return true;
}

bool queue_packet_wrapper( T_PKT_TYPES pkt_type, uint8_t * data , uint8_t data_len )
{
	bool err; 
	err = queue_packet( pkt_type, data , data_len ); 
	return err;
}
#define TXFER_RETRY	5
static bool ble_transfer_packets( void )
{
	uint32_t res;
	static int8_t send_fail_cnt = 0;
	
	while( unsent_fifo_length > 0 ) 
	{
		//Block ISR during packet update ( parse ISR can also call transfer_packet )
		CRITICAL_REGION_ENTER();
		res = ble_uart_tx((uint8_t *)&packet[g_pkt_tail], packet[g_pkt_tail].pkt_len);
		if( res == NRF_SUCCESS ) update_queue();
		CRITICAL_REGION_EXIT();

		//React to result of sending packet to SD
		if( res == NRF_SUCCESS )
		{
			//Packet accepted by the Soft Device
			send_fail_cnt = 0;
		}
		else if( res == BLE_ERROR_NO_TX_PACKETS ) 
		{
			//No more SD Connection Packets Available
			return true;
		}
		else
		{
			//Sumpin failed in adding msg to transmit queue
			if( send_fail_cnt++ >= TXFER_RETRY )
			{
				flush_queue();
				if( gs_bdebug ) app_trace_puts(DEBUG_MED, "[COMMS] Queue Flushed\r");
				return false;
			}
			return true;	//wait for several retries before declaring invalid
		}
	}

	return true;
}

bool ble_transfer_packets_wrapper( void )
{
	bool err;
	ble_transfer_packets();
	return err;
}
static void update_queue( void ) 
{
	//if( !remote_trace_en ) app_trace_puts(DEBUG_LOW, "Packet: Passed to SD\r");
	g_pkt_tail = (g_pkt_tail+1)&PKT_FIFO_MASK;
	unsent_fifo_length--;
}

//! \fn
/// \brief
/// \param
/// \return .
///
static void flush_queue( void )
{
	packet_id = 0;
	g_pkt_head = g_pkt_tail = 0;
	unsent_fifo_length = 0;
}

//! \fn
/// \brief
/// \param
/// \return .
///
static uint8_t get_fifo_length( void )
{
	return unsent_fifo_length;
}

