/*
 * TI BQ25120 i2c driver and Init
 * Created: 4/25/2017 7:41:00 AM
 *  Author: WMM
 */

#include "global.h"
#include "bq25120.h"
#include "hal_twim.h"

#define bq25120_I2C_ADDR	0x6A

// BQ25120 Control Registers Addresses
#define bq25120_STATUS_SHIPMODE_ADDR				(0x00)
	#define bq25120_SHIPMODE_MASK					(0x20)
#define bq25120_FAULTS_ADDR							(0x01)
#define bq25120_TSCNTL_ADDR							(0x02)
#define bq25120_FAST_CHARGE_ADDR					(0x03)
#define bq25120_TERM_PRECHARGE_ADDR					(0x04)
#define bq25120_BATT_VOLT_ADDR						(0x05)
#define bq25120_SYS_OUT_ADDR						(0x06)
#define bq25120_LOADSW_LDO_ADDR						(0x07)
#define bq25120_PB_ADDR								(0x08)
#define bq25120_ILIM_UVLO_ADDR						(0x09)
#define bq25120_BATT_MON_ADDR						(0x0A)
#define bq25120_VIN_DPM_TIMERS_ADDR					(0x0B)
#define bq25120_INIT_MASK							(0xFF)
 
typedef int (*bq25120_comm_t) (uint8_t reg_addr, uint8_t length, uint8_t *data);

ret_code_t bq25120_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply );
ret_code_t bq25120_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data);

const bq25120_comm_t bq25120_read = (bq25120_comm_t) &bq25120_i2c_read;
const bq25120_comm_t bq25120_write = (bq25120_comm_t) &bq25120_i2c_write;

static bool disable_charge = false;

ret_code_t bq25120_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply )
{
	T_TWIM_PACKET packet;
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t res;
		
	packet.i2c_id = bq25120_I2C_ADDR;
	packet.reg_addr = reg_addr;
	packet.buffer = reply;	      //pointer to return data
	packet.length = len;
	res = hal_twim_read( &packet );
	if( res != NRF_SUCCESS ){
		if( prv_res != res )
		{
			app_trace_log(DEBUG_HIGH, "[BQ25120_I2C] Rd Failed: %01d\r", res);
		}
		prv_res = res;
	}
	return res;
}

ret_code_t bq25120_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	T_TWIM_PACKET packet;
	static ret_code_t prv_res = NRF_SUCCESS;
	ret_code_t res;
	
	packet.i2c_id = bq25120_I2C_ADDR;
	packet.reg_addr = reg_addr;
	packet.buffer = data;	      //pointer to data to write
	packet.length = len;
	res = hal_twim_write( &packet );
	if( res != NRF_SUCCESS ){
		if( prv_res != res )
		{
			app_trace_log(DEBUG_HIGH, "[BQ25120_I2C] Wr Failed: %01d\r", res);
		}
		prv_res = res;
	}
	return res;
}

static ret_code_t bq25120_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data)
{
	ret_code_t err;
	uint8_t new_data = 0x00, old_data = 0x00;
	volatile uint8_t shift = 0;

	err = bq25120_read(reg_addr, 1, &old_data);
	if (err != NRF_SUCCESS) return err;
		
	//Find first Set bit in mask, data needs to be shifted to this point
	while( shift < 7 ) {
		if( ((mask>>shift)&0x01) == 0x01 ) {
			break;
		}
		shift++;
	}

	new_data = ((old_data & (~mask)) | ((data<<shift) & mask));

	if (new_data == old_data)
	{	//No need to write value, return
		return NRF_SUCCESS;
	}

	return bq25120_write(reg_addr, 1, &new_data);
}

ret_code_t bq25120_init( void )
{
	ret_code_t res = NRF_SUCCESS;
	
	//Set Up BQ25120
	app_trace_puts(DEBUG_MED, "[BQ25120_INIT] ");
	
	//Must toggle Charge Disable Pin to wake up device
	nrf_gpio_cfg_output(CD_OUT_PIN);
	nrf_gpio_pin_clear(CD_OUT_PIN);
	nrf_gpio_pin_set(CD_OUT_PIN);
	nrf_delay_us(250);	//have to wait for IC to start up
	
	//Initialize Registers:
	res = bq25120_write_data_with_mask( bq25120_STATUS_SHIPMODE_ADDR, bq25120_INIT_MASK, 0x00 );	//(B5: 0)Normal Mode
	res = bq25120_write_data_with_mask( bq25120_FAULTS_ADDR, bq25120_INIT_MASK, 0x06 );				//(B0: 1)Mask OC|(B1: 1)Mask UVLO|(B2: 1)Mask UV|(B3: 1)Mask OV|(B7-4)Read Only: clears Faults
	res = bq25120_write_data_with_mask( bq25120_TSCNTL_ADDR, bq25120_INIT_MASK, 0x09);				//(B0: 1)Mask TMR Fault|(B3: 1)EN_INT|(B7: 0) TS DIS
	
	//50 mA charge current, 4.15V, 10 mA termination: full charge cycle took 53 minutes
	res = bq25120_write_data_with_mask( bq25120_FAST_CHARGE_ADDR, bq25120_INIT_MASK, 0x84 );		//(B0: 0)Not Hi-Z|(B1: 0)Chr En|(B7-2: 100001)Fast Chr=50mA
	res = bq25120_write_data_with_mask( bq25120_TERM_PRECHARGE_ADDR, bq25120_INIT_MASK, 0x92 );		//(B1: 1)En Chr Term Cur|(B7-2: 100100)Term Cur=10mA
	res = bq25120_write_data_with_mask( bq25120_BATT_VOLT_ADDR, bq25120_INIT_MASK, 0x6E );			//Charge Voltage = 4.15V
	
	res = bq25120_write_data_with_mask( bq25120_SYS_OUT_ADDR, bq25120_INIT_MASK, 0xAA );			//SYS = 1.8V
	res = bq25120_write_data_with_mask( bq25120_LOADSW_LDO_ADDR, bq25120_INIT_MASK, 0x7C );			//(B0: 0)RST !MR time met|(B6-2: 11111)LDO Passthrough
	res = bq25120_write_data_with_mask( bq25120_PB_ADDR, bq25120_INIT_MASK, 0x68 );					//(B4-3: 01)!MR 8 SEC|(B5: 1)HI-Z after RST|(B6: 1)!MR wake time
	res = bq25120_write_data_with_mask( bq25120_ILIM_UVLO_ADDR, bq25120_INIT_MASK, 0x1F );			//(B2-0: 111)UVLO = OFF(110=2.2V)|(B5-3: 011)ILIM = 200mA
	res = bq25120_write_data_with_mask( bq25120_BATT_MON_ADDR, bq25120_INIT_MASK, 0x7C );			//(B7: 0)No Reading
	res = bq25120_write_data_with_mask( bq25120_VIN_DPM_TIMERS_ADDR, bq25120_INIT_MASK, 0x82 );		//(B2-1: 01)3 hour fast charge|(B3: 0)TIMER !slow|(B6-4: 000)DPM=4.2V|(B7: 0)EN_DPM
	
	nrf_gpio_pin_clear(CD_OUT_PIN);

	return res;
}

ret_code_t bq25120_en_shipmode( void )
{
	ret_code_t res = NRF_SUCCESS;
	
	//Must toggle Charge Disable Pin to wake up device
	if( bq25120_i2c_wakeup() )
	{
		nrf_delay_us(150);	//have to wait for IC to start up
	}
	
	res = bq25120_write_data_with_mask( bq25120_STATUS_SHIPMODE_ADDR, bq25120_SHIPMODE_MASK, 1 );	//(B5: 1)Ship Mode En
	
	return res;
}

bool bq25120_i2c_wakeup( void )
{
	bool wait = false;
	
	if( nrf_gpio_pin_read(CHARGE_PG_INPUT) == 1 )
	{	//Valid Charger Voltage not detected. Assert wakeup to read status
		nrf_gpio_pin_clear(CD_OUT_PIN);
		nrf_gpio_pin_set(CD_OUT_PIN);
		
		wait = true;				//Part will not respond for ~120 uSec
	}
	
	return wait;
}

void bq25120_charge_disable( bool terminate )
{
	if( terminate )
	{
		disable_charge = true;
		nrf_gpio_pin_set(CD_OUT_PIN);
	}
	else
	{
		disable_charge = false;
		nrf_gpio_pin_clear(CD_OUT_PIN);
	}
}

//	BQ25120 Status bits:
//	0 = Communication error
//	1 = Discharging = No charger in (VIN_UV = status2 0x40) = 1 , LED off
//	2 = Charging = charger in (VIN_UV = status2 0x40) = 0 and V<95%, LED Flashing 
//	3 = Charge Done = charger in (VIN_UV = status2 0x40) = 0 V>95%, LED Steady
//	4 = Fault
T_BQ25120_STATUS bq25120_read_status( void )
{
	T_BQ25120_STATUS ret_val;
	ret_code_t err;
	bool new_value = false;
	uint8_t status, fault;
	static uint8_t prv_status = 0, prv_fault = 0;
	
	//It is assumed that wakeup was called Wake up has already been called and 
	//the timing requirement met.
//	if( bq25120_i2c_wakeup() )
//	{
//		nrf_delay_us(150);	//have to wait for IC to start up
//	}
	
	err = bq25120_read( bq25120_STATUS_SHIPMODE_ADDR, 1, &status );
	err = bq25120_read( bq25120_FAULTS_ADDR, 1, &fault );
	
	if( disable_charge ) nrf_gpio_pin_set(CD_OUT_PIN);		//Pin asserted prevents charging
	else nrf_gpio_pin_clear(CD_OUT_PIN);					//Pin must be cleared to enable charging
	
	if( status != prv_status || fault != prv_fault ) 
	{
		prv_status = status;
		prv_fault = fault;
		new_value = true;
	}
	
	if( err == NRF_SUCCESS )	
	{	
		//Print Status Result
		if( (status & 0xC0) == 0x40 )
		{
			ret_val = BQ_CHARGING;
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Charging, ");
		}
		else if( (status & 0xC0) == 0x80 )
		{
			ret_val = BQ_CHARGED;
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Charge Done, ");
		}
		else if( (status & 0xC0) == 0xC0 )
		{
			ret_val = BQ_FAULT;
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Fault, ");
		}
		else
		{
			ret_val = BQ_READY;
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Ready, ");
		}
		
		//Print all fault bits set
		if( (fault & 0x80) == 0x80 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "VIN_OV, ");
		}
		if( (fault & 0x40) == 0x40 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "VIN_UV, ");
		}
		if( (fault & 0x20) == 0x20 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "BAT_UVLO, ");
		}
		if( (fault & 0x10) == 0x10 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "BAT_OCP, ");
		}
		if( new_value ) app_trace_log(DEBUG_MED, "SB=0x%02X, FB=0x%02X\r", status, fault);
	}
	else
	{
		if( new_value ) app_trace_log( DEBUG_HIGH, "[BQ25120] Err: 0x%02X, 0x%02X\r", status, fault );
		ret_val = BQ_READ_FAIL;	//error reading the bq25120 status
	}

	return ret_val;
}
