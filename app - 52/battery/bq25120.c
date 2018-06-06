/*
 * TI BQ25120 i2c driver and Init
 * Created: 4/25/2017 7:41:00 AM
 *  Author: WMM
 */

#include "global.h"
#include "bq25120.h"
#include "battery.h"
#include "hal_twim.h"

#ifdef BQ25120

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

static ret_code_t bq25120_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply );
static ret_code_t bq25120_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data);

static bool disable_charge = false;
static bool charger_present = false;

static ret_code_t bq25120_i2c_read(uint8_t reg_addr, uint8_t len, uint8_t *reply )
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
	}
	prv_res = res;
	
	return res;
}

static ret_code_t bq25120_i2c_write(uint8_t reg_addr, uint8_t len, uint8_t *data)
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
	}
	prv_res = res;
	
	return res;
}

static ret_code_t bq25120_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data)
{
	ret_code_t err;
	uint8_t new_data = 0x00, old_data = 0x00;
	volatile uint8_t shift = 0;

	err = bq25120_i2c_read(reg_addr, 1, &old_data);
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

	return bq25120_i2c_write(reg_addr, 1, &new_data);
}

static ret_code_t bq2510_reg_defaults( void )
{
	ret_code_t res = NRF_SUCCESS;
	
	//Write the necessary values in order to issue a Reset:
	res = bq25120_write_data_with_mask( bq25120_ILIM_UVLO_ADDR, 0x80, 1 );							//Issue Soft Reset
	nrf_delay_us(25);		//Datasheet doesn't appear to recommend any minimum time here, but I assume like all things, it could use a moment right now
	
	//Initialize Registers:
	res = bq25120_write_data_with_mask( bq25120_STATUS_SHIPMODE_ADDR, bq25120_INIT_MASK, 0x00 );	//(B5: 0)Normal Mode (all other bits are READ ONLY)
	res = bq25120_write_data_with_mask( bq25120_FAULTS_ADDR, bq25120_INIT_MASK, 0x06 );				//(B0: 1)Mask OC|(B1: 1)Mask UVLO|(B2: 1)Mask UV|(B3: 1)Mask OV|(B7-4)Read Only: clears Faults
	res = bq25120_write_data_with_mask( bq25120_TSCNTL_ADDR, bq25120_INIT_MASK, 0x09);				//(B0: 1)Mask TMR Fault|(B3: 1)EN_INT|(B7: 0) TS DIS
	
	//50 mA charge current, 4.15V, 10 mA termination: full charge cycle took 53 minutes
	res = bq25120_write_data_with_mask( bq25120_FAST_CHARGE_ADDR, bq25120_INIT_MASK, 0x84 );		//(B0: 0)Not Hi-Z|(B1: 0)Chr En|(B7-2: 100001)Fast Chr=50mA
	res = bq25120_write_data_with_mask( bq25120_TERM_PRECHARGE_ADDR, bq25120_INIT_MASK, 0x92 );		//(B1: 1)En Chr Term Cur|(B7-2: 100100)Term Cur=10mA
	res = bq25120_write_data_with_mask( bq25120_BATT_VOLT_ADDR, bq25120_INIT_MASK, 0x6E );			//Charge Voltage = 4.15V
	
	res = bq25120_write_data_with_mask( bq25120_SYS_OUT_ADDR, bq25120_INIT_MASK, 0xAA );			//0xAA = SYS = 1.8V 0xBC = 2.7V for HRM
	res = bq25120_write_data_with_mask( bq25120_LOADSW_LDO_ADDR, bq25120_INIT_MASK, 0x7D );//7C		//(B0: 1)RST !MR time & VIN Good|(B6-2: 11111)LDO Passthrough
	res = bq25120_write_data_with_mask( bq25120_PB_ADDR, bq25120_INIT_MASK, 0x60 );//68				//(B2: 0)Output PG|(B4-3: 00)!MR 5 SEC|(B5: 1)HI-Z after RST|(B6: 1)!MR wake time
	res = bq25120_write_data_with_mask( bq25120_ILIM_UVLO_ADDR, bq25120_INIT_MASK, 0x1F );			//(B2-0: 111)UVLO = OFF(110=2.2V)|(B5-3: 011)ILIM = 200mA
	res = bq25120_write_data_with_mask( bq25120_BATT_MON_ADDR, bq25120_INIT_MASK, 0x7C );			//(B7: 0)No Reading
	res = bq25120_write_data_with_mask( bq25120_VIN_DPM_TIMERS_ADDR, bq25120_INIT_MASK, 0x86 );		//(B2-1: 11)3 disable safety timer|(B3: 0)TIMER !slow|(B6-4: 000)DPM=4.2V|(B7: 0)EN_DPM
	/////=> disable bq25120_VIN_DPM_TIMERS_ADDR Safty timer using 0x86 and bq25120_TSCNTL_ADDR bit 0 = 1 = 0x09.
	
	return res;
}

ret_code_t bq25120_init( void )
{	//Set Up BQ25120
	ret_code_t res = NRF_SUCCESS;

	//Must toggle Charge Disable Pin to wake up device
	nrf_gpio_cfg_output(CD_OUT_PIN);
	if( bq25120_i2c_wakeup() )
	{
		nrf_delay_us(150);	//have to wait for IC to start up
	}
	
	#define PRINT_BQ_REG_VALUES
	#ifdef PRINT_BQ_REG_VALUES
		uint8_t reg[12];
		//Read & Print Register contents registers before manipulating them:
		bq25120_i2c_read( 0x00, sizeof(reg), reg );
		app_trace_puts(DEBUG_MED, "[BQ25120_INIT] B: ");
		for( int i=0; i<sizeof(reg); i++ )
		{
			app_trace_log(DEBUG_MED, "0x%02X ", reg[i]);
		}
		app_trace_puts(DEBUG_MED, "\r");
		
		//Reset values of Registers: 			0xC1 0x00 0xA8 0x14 0x0E 0x78 0xAA 0x7C 0x68 0x0A 0x00 0x42 
		//Values Read from Jason v1.04:			B: 0xC1 0x03 0xA7 0x26 0xA6 0xF4 0xD6 0x7D 0x60 0x1F 0x44 0x82 
		//										A: 0xC1 0x06 0x29 0x84 0x92 0x6E 0xAA 0x7D 0x60 0x1F 0x44 0x82
	#endif	
	
	res = bq2510_reg_defaults();
		
	#ifdef PRINT_BQ_REG_VALUES
		//Read and Print new Register contents:
		bq25120_i2c_read( 0x00, sizeof(reg), reg );
		app_trace_puts(DEBUG_MED, "[BQ25120_INIT] A: ");
		for( int i=0; i<sizeof(reg); i++ )
		{
			app_trace_log(DEBUG_MED, "0x%02X ", reg[i]);
		}
		app_trace_puts(DEBUG_MED, "\r");
	#endif
	
	if( res == NRF_SUCCESS ) 
	{
		app_trace_puts(DEBUG_MED, "[BQ25120_INIT] complete\r");
	}
	else 
	{
		app_trace_log(DEBUG_HIGH, "[BQ25120_INIT] Failed: %01d\r", res);
	}
	
	//Enable Charging:
	bq25120_charge_disable( false );

	return res;
}

//Use to check if the BQ25120 is reporting a charger voltage is present
bool bq25120_charger_connected( void )
{
	return charger_present;
}

bool bq25120_i2c_wakeup( void )
{
	bool wait = false;
	
	if( nrf_gpio_pin_read(CHARGE_PG_INPUT) == 1 )
	{	
		//Valid Charger Voltage not detected. Assert wakeup to read status
		nrf_gpio_pin_clear(CD_OUT_PIN);
		nrf_gpio_pin_set(CD_OUT_PIN);	//Charging is Off while pin is asserted. Call i2c_idle once communication is complete
		
		wait = true;				//Part will not respond for ~120 uSec
	}
	
	return wait;
}

static void bq25120_i2c_idle( void )
{
	//When Communication is complete, return I2C to idle state which allows Charging,
	//unless charging hsa been disabled:
	if( disable_charge ) nrf_gpio_pin_set(CD_OUT_PIN);		//Pin asserted prevents charging
	else nrf_gpio_pin_clear(CD_OUT_PIN);					//Pin must be cleared to enable charging
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

ret_code_t bq25120_en_shipmode( void )
{
	ret_code_t res = NRF_SUCCESS;
	
	//Must toggle Charge Disable Pin to wake up device
	if( bq25120_i2c_wakeup() )
	{
		nrf_delay_us(150);	//have to wait for IC to start up
	}
	
	//Have to activate Charge Disable Line for Ship Mode to activate:
	bq25120_charge_disable( true );
	
	res = bq25120_write_data_with_mask( bq25120_STATUS_SHIPMODE_ADDR, bq25120_SHIPMODE_MASK, 1 );	//(B5: 1)Ship Mode En
	
	bq25120_i2c_idle();		//Now that Charge Disable is active we will not be returning to idle mode, so this is really just for completeness
	
	return res;
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
	uint8_t reg[12] = {0};
	uint8_t * status = reg;
	uint8_t * fault = reg+1;
	uint8_t len = 2;
	static uint8_t prv_status = 0, prv_fault = 0;
	
	if( bq25120_i2c_wakeup() )
	{	//must wait for BQ to come out of Hi-Z and be ready to communicate
		nrf_delay_us(150);	
	}
	
	//Read the first 2 registers to see any any faults are present
	err = bq25120_i2c_read( bq25120_STATUS_SHIPMODE_ADDR, len, reg );
	if( err != NRF_SUCCESS )
	{	//Try again
		app_trace_puts(DEBUG_MED, "[BQ25120] Re-read\r");
		
		//Force I2C wakeup condition:
		nrf_gpio_pin_clear(CD_OUT_PIN);
		nrf_gpio_pin_set(CD_OUT_PIN);	//Charging is Off while pin is asserted. Call i2c_idle once communication is complete
		nrf_delay_us(150);
		
		err = bq25120_i2c_read( bq25120_STATUS_SHIPMODE_ADDR, len, reg );
	}
	
	//Handle faults that won't clear
	if( (*status & 0xC0) == 0xC0 )
	{	//Fault reported
		static uint8_t retries = 0;
		
		//read all registers for debug
		bq25120_i2c_read( bq25120_TSCNTL_ADDR, 10, &reg[2] );
		len = 12;
		
		if( *status == prv_status )
		{	//fault was not cleared by previous read, more drastic measures needed:
			if( retries < 3 )
			{
				app_trace_log(DEBUG_MED, "[BQ25120] %sRe-Init Registers%s @%01u\r", FG_RED, FG_RESET, getSystemTimeMs());
				if( bq2510_reg_defaults() != NRF_SUCCESS )
				{
					app_trace_puts(DEBUG_MED, "[BQ25120] Re-Init Failed\r");
				}
			}
			retries = (retries+1)%30;	//As long as fault remains unchanged, retry re-initializing the part every 30th pass (=30 seconds)
		}
		else
		{
			retries = 0;
		}
	}
	
	bq25120_i2c_idle();
	
	//BQ has had a change, allow update to print
	if( *status != prv_status || *fault != prv_fault ) 
	{
		prv_status = *status;
		prv_fault = *fault;
		new_value = true;
	}
	
	if( err == NRF_SUCCESS )	
	{	
		//Parse Status Result
		if( (*status & 0xC0) == 0x40 )
		{
			ret_val = BQ_CHARGING;
			charger_present = true;
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Charging, ");
		}
		else if( (*status & 0xC0) == 0x80 )
		{
			ret_val = BQ_CHARGED;
			charger_present = true;
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Charge Done, ");
		}
		else if( (*status & 0xC0) == 0xC0 )
		{
			ret_val = BQ_FAULT;
			charger_present = false;	//assume no charger
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Fault, ");
		}
		else
		{
			ret_val = BQ_READY;
			charger_present = false;
			if( new_value ) app_trace_puts(DEBUG_MED, "[BQ25120] Ready, ");
		}
		
		//Print all fault bits set
		if( (*fault & 0x80) == 0x80 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "VIN_OV, ");
		}
		if( (*fault & 0x20) == 0x20 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "BAT_UVLO, ");
		}
		if( (*fault & 0x10) == 0x10 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "BAT_OCP, ");
		}
		if( (*fault & 0x40) == 0x40 )
		{
			if( new_value ) app_trace_puts(DEBUG_MED, "VIN_UV, ");
		}
		
	}
	else
	{
		ret_val = BQ_READ_FAIL;	//error reading the bq25120 status
		//Comms down, fallback to PG Pin for charger present
		if( nrf_gpio_pin_read(CHARGE_PG_INPUT) == 0 )
		{	//Valid Charger Voltage is present
			charger_present = true;
		}
		else
		{
			charger_present = false;
		}
		if( new_value ) app_trace_log( DEBUG_HIGH, "[BQ25120] Comms Err: " );
	}
	
	//On change, finish by printing out all the registers that were read:
	if( new_value ) 
	{
		for( int i=0; i<len; i++ )
		{
			app_trace_log(DEBUG_MED, "0x%02X ", reg[i]);
		}
		app_trace_log(DEBUG_MED, "@%01u\r", getSystemTimeMs());
	}
	
	//Debug the possible situation where we are not Charging and/or indicating Charging when we should be:
	if( (ret_val == BQ_READY) && charger_present )
	{	//Status should indicate Charging or Charge Done.
		//This condition is expected when charging has been disabled
		if( !disable_charge )
		{	
			app_trace_log(DEBUG_MED, "[BQ25120] %sChr_Present Mismatch!!!%s\r", FG_GREEN, FG_RESET );
		}
	}

	return ret_val;
}

#endif

