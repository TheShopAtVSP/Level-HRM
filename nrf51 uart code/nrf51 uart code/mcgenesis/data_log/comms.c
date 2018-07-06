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

extern Bool gs_bdebug;
extern uint32_t get_unix_time( void );
extern void set_unix_time( uint32_t );

#define PACKET_TIMEOUT	5000

typedef enum {
	ACK_IDLE,
	WAITING,
	RECIEVED,
	ID_ACK_FAIL,
}T_ACK_STATE;

typedef enum {
	TX_OFF,
	TX_START,
	TX_CONTINUE,
} T_TX_STATE;

struct{
	T_ACK_STATE state;
	uint32_t pkt_id;
} rec_ack = { .state = ACK_IDLE, .pkt_id = 0xFFFF };
	
#define PKT_FIFO_MASK	0x3F
T_PACKET packet[PKT_FIFO_MASK+1];		//allow up to 64 packets to queue (also saves a partial history of recent_pkts = ((PKT_FIFO_MASK+1)-(pkt_head-pkt_tail)&PKT_FIFO_MASK)
T_PACKET * resend_pkt;
Bool priority_msg = false;
Bool new_connection = false;
Bool record_queue_en = false;
static uint8_t g_pkt_head = 0, g_pkt_tail = 0;

extern Bool ble_uart_tx( uint8_t *, uint8_t );
extern T_CONFIG g_config;
extern void disconnect_and_advertize(void);

Bool packetize_record( void );
Bool queue_packet( T_PKT_TYPES pkt_type, uint8_t * data , uint8_t data_len );
void flush_queue( void );
uint8_t get_fifo_length( void );

//! \fn
/// \brief
/// \param
/// \return .
///
void flag_new_comms( void )
{
	new_connection = true;
	record_queue_en = false;
	rec_ack.state = ACK_IDLE;
	flush_queue();
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
	extern uint8_t led_id_key_state;
	T_PKT_TYPES tx_type;
	uint8_t tx_payload[ MAX_PKT_PAYLOAD ];
	int8_t tx_pay_len = -1;	//default no response
	int8_t rx_pay_len = rx_msg->pkt_len - PKT_HEADER_LEN;
	T_REPORT_ERR res;
	Union32 temp;
	
	//check that the ID sequence is correct
	if( rx_msg->pkt_len >= PKT_HEADER_LEN ) {
		if( new_connection == true ) {
			//Re-base sequence number
			nextID = rx_msg->id;
			new_connection = false;
		}
		else if( rx_msg->id != ++nextID ) {
			//a packet was likely dropped!!!!		
			
			//tx_type = NACK;
			//tx_payload[0] = PKT_SEQ_ERROR;
			//tx_payload[1] = nextID;
			//tx_pay_len = 2;
			//queue_packet( tx_type, tx_payload, tx_pay_len );
			
			nextID = rx_msg->id;
			if (gs_bdebug) app_trace_log("Parse: ID Seq Err\r");
		}
	}
	else {
		//pkt length is shorter than the minimum message length: 2
		if (gs_bdebug) app_trace_log("Parse: Msg Incomplete\r");
		tx_type = NACK;	//any undefined case will return a NACK
		tx_payload[0] = PKT_LEN_ERROR;
		tx_pay_len = 1;
		queue_packet( tx_type, tx_payload, tx_pay_len );
		return;
	}
	
	switch( rx_msg->type )
	{
		case NACK:	//Something didn't Work
			switch ( rx_msg->payload[0] ) {
				case PKT_SEQ_ERROR:
					if (gs_bdebug) app_trace_log("Parse: Host Requests Packet %01u Resent\r", rx_msg->payload[1]);
					if( packet[rx_msg->payload[1]&PKT_FIFO_MASK].id == rx_msg->payload[1] ){
						if (gs_bdebug) app_trace_log("Resending Packet %01u\r", packet[rx_msg->payload[1]&PKT_FIFO_MASK].id );
						resend_pkt = &packet[rx_msg->payload[1]&PKT_FIFO_MASK];
						priority_msg = true;
					}
					else {
						//Packet number no longer exists
						if (gs_bdebug) app_trace_log("Packet %01u No Longer Exists\r", rx_msg->payload[1]);
						tx_type = NACK;
						tx_payload[0] = PKT_SEQ_ERROR;
						tx_payload[1] = rx_msg->id;
						tx_pay_len = 2;
					}
					break;
					
				case PKT_LEN_ERROR:
					//Host Device believes the packet length was in error
					//not sure what todo: 
					if (gs_bdebug) app_trace_log("Parse: NACK- Packet Length Error\r");
					break;
					
				case PKT_TYPE_UNRECOG:
					//Host Device did not recognize the Packet Type
					//not sure what todo:
					if (gs_bdebug) app_trace_log("Parse: NACK- Packet Type Error\r");
					break;
				
				default:
					if (gs_bdebug) app_trace_log("Parse: NACK- Unknown\r");
					break;
			}	
			
			break;
		
		case ACK:	//Peer Device is ACKing transmitted Data	
			if (gs_bdebug) app_trace_log("Parse: ACK- ");
			if( rec_ack.state == WAITING ) {
				if( rx_msg->payload[0] == rec_ack.pkt_id ) {	
					//ACKing the final packet in a Record
					rec_ack.state = RECIEVED;
					if (gs_bdebug) app_trace_log("Record Accepted\r");
				}
				else {
					rec_ack.state = ID_ACK_FAIL;
					if (gs_bdebug) app_trace_log("ID %01u Does Not Match %01u\r", rx_msg->payload[0], (unsigned int)rec_ack.pkt_id);
				}
			}
			else {
				if (gs_bdebug) app_trace_log("???\r");
			}

			break;
		
		case REPORT_ATTRIBUTES:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) {		
				if( rx_pay_len == ATTRIBUTE_SIZE ) {
					app_trace_log("Report Atts Request: \r");
					app_trace_log("  Inst: 	%01u\r", rx_msg->payload[0]);		// id
					app_trace_log("  Ind_Des:	%01u\r", rx_msg->payload[1]);	// T_INDEP_DESC 
					app_trace_log("  Ind_Scl:	%01u\r", rx_msg->payload[2]);	// count
					app_trace_log("  Data_Des:	%01u\r", rx_msg->payload[3]);	// T_DATA_DESC
					app_trace_log("  Type: 	%01u\r", rx_msg->payload[4]);		// T_DATA_TYPE
					app_trace_log("  FSR:  	%01u\r", rx_msg->payload[5]);		// T_DATA_FSR
					app_trace_log("  Fields:	%01u\r", rx_msg->payload[6]); 	// T_DATA_FIELDS
					app_trace_log("  Samples:	%01u\r", rx_msg->payload[7]|(rx_msg->payload[8]<<8));	// count
					app_trace_log("  Recs_Rep:	%01u\r", rx_msg->payload[9]|(rx_msg->payload[10]<<8));	// count
				}
			}
			res = set_reporter( (T_REPORT_ATTRIBUTES *)rx_msg->payload, rx_pay_len );
			if( res == NO_ERROR ) {
				if( rx_pay_len == ATTRIBUTE_SIZE ) {
					//New atts were Requested and Accepted
					flag_config_update();	//store new values in flash when we return to main
				}
				
				if (gs_bdebug) {
					app_trace_log("Report Atts Set: ");
					for(uint i=0; i<ATTRIBUTE_SIZE; i++ ) {
						app_trace_log("%02u, ", rx_msg->payload[i]);
					}
					app_trace_log("\r");
				}

				tx_type = REPORT_ATTRIBUTES;
				for( uint i=0; i<ATTRIBUTE_SIZE; i++ ) {
					tx_payload[i] = rx_msg->payload[i];	//copy the accepted attributes to send back
				}
				tx_pay_len = ATTRIBUTE_SIZE;
			}
			else {
				if (gs_bdebug) app_trace_log("Attribute Update Failed: %01u\r", res);
				//Something failed, return the Error Code
				tx_type = NACK;
				tx_payload[0] = ATTIBUTE_ERROR;
				tx_payload[1] = rx_msg->id;
				tx_payload[2] = res;
				tx_pay_len = 3;
			}	
	
			break;
			
		case REPORT_CONTROL:	//Host Device is Requesting Reporters to be turn on/off
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) app_trace_log("Reporters Req: ");
			if( rx_pay_len > 0 ) {
				//turn on/off the Reporters that are indicated by data byte 0
				if (gs_bdebug) app_trace_log("0x%02X\r", rx_msg->payload[0]);
				enable_reporters(rx_msg->payload[0]);	//return the Active/Inactive flags
			}
			else {
				if(gs_bdebug) app_trace_log("\r");
			}
			
			//reply with active/inactive Reporters
			tx_type = REPORT_CONTROL;
			tx_payload[0] = get_active_reporters();
			if (gs_bdebug) app_trace_log("Reporters Active: 0x%02X\r", tx_payload[0]);
			tx_pay_len = 1;
			
			break;
			
		case TRANSMIT_CONTROL:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			//if no payload is defined, assume the ue is supposed to stay off
			if( rx_pay_len <= 0 ) rx_msg->payload[0] = false;

			if (gs_bdebug) app_trace_log("Transmit Ctrl: 0x%02X\r", rx_msg->payload[0]);
		
			//turn on/off the Reporters that are indicated by data byte 0
			if(rx_msg->payload[0] == true ) {
				//start sending records
				record_queue_en = true;
			}
			else {
				//stop record transmission
				record_queue_en = false;
			}
			
			//send back the total number of records used in memory
			tx_type = TRANSMIT_CONTROL;
			temp.u32 = get_total_record_cnt();	//number of queued Records in memory
			tx_payload[0] = temp.u8[0];	
			tx_payload[1] = temp.u8[1];
			temp.u32 = log_byte_size() - REC_PREAMBLE_LEN*temp.u32;	//transmit size of queued Records
			tx_payload[2] = temp.u8[0];	
			tx_payload[3] = temp.u8[1];
			tx_payload[4] = temp.u8[2];
			tx_payload[5] = temp.u8[3];
			tx_pay_len = 6;
			if (gs_bdebug) {
				app_trace_log("TC Response: ");
				for( int i=0; i<tx_pay_len; i++ ) {
					app_trace_log("0x%02X, ", tx_payload[i]);
				}
				app_trace_log("\r");
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
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) app_trace_log("Time Read\r");

			//Send back current time
			tx_type = TIMERD;
			temp.u32 = get_unix_time();
			for (int i = 0; i<sizeof(uint32_t); i++) {
				tx_payload[i] = temp.u8[i];
			}
			tx_pay_len = sizeof(uint32_t);
	
			break;	
			
		case TIMEWR:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) app_trace_log("Time Set\r");
		
			for (int i = 0; i<sizeof(uint32_t); i++) {
				temp.u8[i] = rx_msg->payload[i];
				tx_payload[i] = temp.u8[i];
			}
			
			if( temp.u32 != get_unix_time() ) {
				reporter_time_rebase( temp.u32, get_unix_time() );
				set_unix_time( temp.u32 );							
				//unix2Clock( temp.u32 );
			}
			
			//Respond with new time
			tx_type = TIMEWR;
			tx_pay_len = sizeof(uint32_t);
	
			break;
				
		case USERUUIDRD:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) app_trace_log("User UUID RD\r");			
			
		  // reading
			tx_type = USERUUIDRD;
			for (int i = 0; i<user_UUID_LEN; i++) {
				tx_payload[i] = g_config.user_UUID[i];
			}
			tx_pay_len = user_UUID_LEN;
	
			break;
	
		case USERUUIDWR:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) app_trace_log("User UUID WR\r");			
			
			// writing
			for (int i = 0; i<user_UUID_LEN; i++) {
				g_config.user_UUID[i] = rx_msg->payload[i];
				tx_payload[i] = g_config.user_UUID[i];
			}
			
			flag_config_update();	//store new values in flash when we return to main
			
			//Respond with new UUID
			tx_type = USERUUIDWR;
			tx_pay_len = user_UUID_LEN;

			break;

		case FRAMERD:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) app_trace_log("Frame RD\r");			
			
		  // reading
			tx_type = FRAMERD;
			for (int i = 0; i<device_FRAMEID_LEN; i++) {
				tx_payload[i] = g_config.device_frame_ID[i];
			}
			tx_pay_len = device_FRAMEID_LEN;
	
			break;
	
		case FRAMEWR:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}
			if (gs_bdebug) app_trace_log("Frame WR\r");			
			
			// writing
			for (int i = 0; i<device_FRAMEID_LEN; i++) {
				g_config.device_frame_ID[i] = rx_msg->payload[i];
				tx_payload[i] = g_config.device_frame_ID[i];
			}
			
			flag_config_update();	//store new values in flash when we return to main
			
			//Respond with new UUID
			tx_type = FRAMEWR;
			tx_pay_len = device_FRAMEID_LEN;

			break;
			
		case BLE_TX_POWER:
			if(led_id_key_state != 4){ 
				disconnect_and_advertize();
				break;
			}			
			//adjustment to BLE transmit power
			//accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm.
			//(-40 dBm will not actually give -40 dBm, but will instead be remapped to -30 dBm).
			if( (int8_t)rx_msg->payload[0] >= 2 ) temp.s32 = 4;	
			else if( (int8_t)rx_msg->payload[0] >= -2 ) temp.s32 = 0;
			else if( (int8_t)rx_msg->payload[0] >= -6 ) temp.s32 = -4;
			else if( (int8_t)rx_msg->payload[0] >= -10 ) temp.s32 = -8;
			else if( (int8_t)rx_msg->payload[0] >= -14 ) temp.s32 = -12;
			else if( (int8_t)rx_msg->payload[0] >= -18 ) temp.s32 = -16;
			else if( (int8_t)rx_msg->payload[0] >= -25 ) temp.s32 = -20;
			else temp.s32 = -30;
		
			sd_ble_gap_tx_power_set( temp.s8[0] );
		
			if (gs_bdebug) app_trace_log("New TX Power: %01u dBm\r", temp.s8[0]);
		
			//reply with active/inactive Reporters
			tx_type = BLE_TX_POWER;
			tx_payload[0] = temp.u8[0];
			tx_pay_len = 1;
	
			break;
			
		default:    //Where's The Fun?
			if (gs_bdebug) app_trace_log("Respond: Where's The Fun?\r");
	
			tx_type = NACK;
			tx_payload[0] = PKT_TYPE_UNRECOG;
			tx_payload[1] = rx_msg->id;
			tx_pay_len = 2;
		
			break;
	} //end switch
	
	if( tx_pay_len >= 0 ) {
		queue_packet( tx_type, tx_payload, tx_pay_len );
	}
}

//! \fn
/// \brief
/// \param
/// \return .
///
uint32_t record_pkts2_send = 0;
Bool packetize_record( void )
{		
	Bool res = false;
	T_RECORD rec = { .hdr.rec_len = 0 };
			
	if( pop_record( &rec ) == true ) {
		uint8_t * rec_ptr = (uint8_t *) &rec.hdr;
		Bool start_packet = true;
		T_PKT_TYPES pkt_type;
		uint8_t payload_len;
		uint16_t remain_len;
		
		uint8_t num_pkts_2_add = (rec.hdr.rec_len/MAX_PKT_PAYLOAD) + 1;
		
		if( (get_fifo_length() + num_pkts_2_add) > PKT_FIFO_MASK ) {
			if (gs_bdebug) app_trace_log("Queue: Too many packets!\r");
			rec_ack.pkt_id = 0xFFFF;	//prevent an ACK from accidentally updating the Log pointers
			return res;	//not enough space in the FIFO to add this record
		}
		
		//force record into sub-packets and queue piecemeal transmission
		remain_len = rec.hdr.rec_len;
		while( remain_len > 0 )
		{
			//first packet type is different to indicate the beginning of a record
			if( start_packet == true ) {
				start_packet = false;
				pkt_type = START_OF_RECORD;
			}
			else {
				pkt_type = CONTINUE_RECORD;
			}
			
			//queue packets until record is complete	
			if( remain_len >= MAX_PKT_PAYLOAD ) {
				payload_len = MAX_PKT_PAYLOAD;
			}
			else {
				payload_len = remain_len;
			}
			remain_len -= payload_len;

			if( queue_packet( pkt_type, rec_ptr, payload_len) != true ) {
				if (gs_bdebug) app_trace_log("Queue: Full\r");
				//no more room in queue (This is already checked for, so it shouldn't occur. But better safe than sorry)
				rec_ack.pkt_id = 0xFFFF;	//prevent an ACK from accidentally updating the Record pointers
				return res;
			}
			
			rec_ptr += payload_len;	
		}
		
		//host must ACK the final packet in the transmitted Record in order for the next Record to be queued.
		rec_ack.state = WAITING;
		rec_ack.pkt_id = packet[(g_pkt_head-1)&PKT_FIFO_MASK].id;	//the packet id of the last packet in the record.
		res = true;
	}
	
	return res;
}

//! \fn
/// \brief
/// \param
/// \return .
///
static uint8_t unsent_fifo_length = 0;
#define PKT_RETRY	5
void send_ble_data( void ) 
{	
	static TTASK_TIMER timeout = { .timer = 0, .period = PACKET_TIMEOUT };
	static T_TX_STATE rec_queue_state = TX_OFF;
	static int8_t retries = PKT_RETRY;
	static int8_t send_fail = PKT_RETRY;
	
	//Handle the sending of recorded data
	switch( rec_queue_state ) {
		case TX_OFF:	//OFF
			rec_ack.state = ACK_IDLE;
			if( record_queue_en == true ) {
				rec_queue_state = TX_START;
			}
			break;
			
		case TX_START:	//Queue up first Packet
			if( record_queue_en == false ) {
				//requested to stop
				rec_queue_state = TX_OFF;
			}
			else if( memory_empty() == false ) {
				if( packetize_record() == true ) {
					if( gs_bdebug ) app_trace_log("Record Queued to TX\r");
					re_arm_timer( timeout );
					retries = PKT_RETRY;
					rec_ack.state = WAITING;
					rec_queue_state = TX_CONTINUE;
				}
			}
			else {
				//Nothing left to transmit. Turn Transmit Control Off? Per Andrew: "NO!"
				//record_queue_en = false;
				//rec_queue_state = TX_OFF;
			}
			break;
			
		case TX_CONTINUE:	//Wait to Queue next packet
			if( record_queue_en == false ) {
				//requested to stop
				rec_ack.state = ACK_IDLE;
				rec_queue_state = TX_OFF;
			}
			else {
				switch( rec_ack.state ) {
					case ACK_IDLE:
						//if any data goes into memory before the timeout, send it
						if( retries <= 0 ) {
							record_queue_en = false;	//5 failures in a row. Turn off transmit control
							rec_ack.state = ACK_IDLE;
							rec_queue_state = TX_START;
						}
						else if( memory_empty() == false ) {
							if( packetize_record() ) {
								if( gs_bdebug ) app_trace_log("Record Queued\r");
								//tx_timestamp = systime_ms;
								rec_ack.state = WAITING;
							}
						}
						break;
				
					case WAITING:
						if( task_time( timeout ) ) {
							//turn off TX control
							//record_queue_en = false;
							retries--;
							rec_ack.state = ACK_IDLE;
							if( gs_bdebug ) app_trace_log("No Record ACK\r");
						}
						break;
					
					case RECIEVED:
						//If a Record Receipt has been received, update the tail pointer and prepare the next packet
						update_log_tail();	
						if( retries != PKT_RETRY ) {
							//successful sent packet, reset number of retries
							//some messages were lost and needed to resend
							retries = PKT_RETRY;
						}
						rec_ack.state = ACK_IDLE;
				
						if( memory_empty() == false ) {
							if( packetize_record() ) {
								if( gs_bdebug ) app_trace_log("Next Record Queued\r");
								//tx_timestamp = systime_ms;
								rec_ack.state = WAITING;
							}
						}
						break;
					
					case ID_ACK_FAIL:
						retries--;
						rec_ack.state = ACK_IDLE;
						if( gs_bdebug ) app_trace_log("ACK Not Valid\r");
						break;
					
					default:
						rec_ack.state = ACK_IDLE;
						break;
				}
			}
			break;
			
		default:
			rec_queue_state = TX_OFF;
			break;
	}
	
	//Send any data that has already been put in the transmit buffer
	if( priority_msg == true ) {
		//a packet requires immediate re-sending
		if( ble_uart_tx( (uint8_t *)resend_pkt, resend_pkt->pkt_len) == true ) {
			//accepted by the BLE device
			re_arm_timer( timeout );
			priority_msg = false;
		}
		else {
			//sumpin failed in adding msg to transmit queue
			if( gs_bdebug ) app_trace_log("Packet: Priority Msg Failed\r");
		}
	}
	else if( unsent_fifo_length > 0 ) {
		if( ble_uart_tx((uint8_t *)&packet[g_pkt_tail], packet[g_pkt_tail].pkt_len) == true ) {
			//accepted by the BLE device
			if( gs_bdebug ) app_trace_log("Packet: Passed to Soft Device\r");
			re_arm_timer( timeout );
			g_pkt_tail = (g_pkt_tail+1)&PKT_FIFO_MASK;
			unsent_fifo_length--;
			send_fail = PKT_RETRY;
		}
		else {
			//sumpin failed in adding msg to transmit queue
			if( gs_bdebug ) app_trace_log("Packet: TX Failed\r");
			if( send_fail-- <= 0 ) {
				record_queue_en = false;	//5 failures in a row. Turn off transmit control
				rec_ack.state = ACK_IDLE;
				rec_queue_state = TX_START;
				flush_queue();
			}
		}
	}
}

//! \fn
/// \brief
/// \param
/// \return .
///
static uint8_t packet_id = 0;
Bool queue_packet( T_PKT_TYPES pkt_type, uint8_t * data , uint8_t data_len )
{	
	if( unsent_fifo_length > PKT_FIFO_MASK ) {
		//queue is full
		return false;	//can't add anymore to queue
	}
	
	packet[g_pkt_head].id = packet_id++;
	packet[g_pkt_head].type = pkt_type;
	for(int i=0; i<data_len; i++) {
		packet[g_pkt_head].payload[i] = *(data+i);
	}
	packet[g_pkt_head].pkt_len = data_len + 2;	
	
	if (gs_bdebug) app_trace_log("Packet: %01u Queued\r", packet[g_pkt_head].id);
	g_pkt_head = (g_pkt_head+1)&PKT_FIFO_MASK;
	unsent_fifo_length++;
	
	return true;
}

//! \fn
/// \brief
/// \param
/// \return .
///
void flush_queue( void ) 
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
uint8_t get_fifo_length( void )
{
	return unsent_fifo_length;
}
