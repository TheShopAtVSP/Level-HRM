/*
 * lis3mdl.c
 *
 * Created: 8/20/2015 11:52:26 AM
 *  Author: matt
 */ 

#include "lis3mdl.h"
#include <stdlib.h>

//Data Rates when FAST_ODR is disabled
enum {
	LIS3MDL_ODR_0_625HZ_VAL =	0x00,
	LIS3MDL_ODR_1_25HZ_VAL =	0x01,
	LIS3MDL_ODR_2_5HZ_VAL =		0x02,
	LIS3MDL_ODR_5HZ_VAL =		0x03,
	LIS3MDL_ODR_10HZ_VAL =		0x04,
	LIS3MDL_ODR_20HZ_VAL =		0x05,
	LIS3MDL_ODR_40HZ_VAL =		0x06,
	LIS3MDL_ODR_80HZ_VAL =		0x07,
	
	LIS3MDL_ODR_TABLE_LEN =		8
};

typedef struct {
	uint32_t hz;
	uint8_t reg_val;
} T_LIS3MDL_ODR;

T_LIS3MDL_ODR odr_lut[LIS3MDL_ODR_TABLE_LEN] = {
	[0] = { .hz = 0, .reg_val = LIS3MDL_ODR_0_625HZ_VAL },
	[1] = { .hz = 1, .reg_val = LIS3MDL_ODR_1_25HZ_VAL },
	[2] = { .hz = 2, .reg_val = LIS3MDL_ODR_2_5HZ_VAL },
	[3] = { .hz = 5, .reg_val = LIS3MDL_ODR_5HZ_VAL },
	[4] = { .hz = 10, .reg_val = LIS3MDL_ODR_10HZ_VAL },
	[5] = { .hz = 20, .reg_val = LIS3MDL_ODR_20HZ_VAL },
	[6] = { .hz = 40, .reg_val = LIS3MDL_ODR_40HZ_VAL },
	[7] = { .hz = 80, .reg_val = LIS3MDL_ODR_80HZ_VAL },
};

//Full Scale Range Control Bits
enum {
	LIS3MDL_FS_4GAUSS_VAL =		0x00,
	LIS3MDL_FS_8GAUSS_VAL =		0x01,
	LIS3MDL_FS_12GAUSS_VAL =	0x02,
	LIS3MDL_FS_16GAUSS_VAL =	0x03,
	
	LIS3MDL_FS_TABLE_LEN =		4
};

//Full Scale Range Gain (LSBs/Gauss)
enum {
	GAIN_4GAUSS =	6842,
	GAIN_8GAUSS =	3421,
	GAIN_12GAUSS =	2281,
	GAIN_16GAUSS =	1711
};

typedef struct {
	uint8_t fsr;
	uint8_t reg_val;
	uint16_t gain;
} T_LIS3MDL_FSR;

const T_LIS3MDL_FSR fsr_lut[LIS3MDL_FS_TABLE_LEN] = {
	[0] = { .fsr = 4, .reg_val = LIS3MDL_FS_4GAUSS_VAL, .gain = GAIN_4GAUSS },
	[1] = { .fsr = 8, .reg_val = LIS3MDL_FS_8GAUSS_VAL, .gain = GAIN_8GAUSS },
	[2] = { .fsr = 12, .reg_val = LIS3MDL_FS_12GAUSS_VAL, .gain = GAIN_12GAUSS },
	[3] = { .fsr = 16, .reg_val = LIS3MDL_FS_16GAUSS_VAL, .gain = GAIN_16GAUSS },
};

//Performance Modes
enum {
	LIS3MDL_LOW_POWER_VAL =		0x00,
	LIS3MDL_MED_PERFORM_VAL =	0x01,
	LIS3MDL_HIGH_PERFORM_VAL =	0x02,
	LIS3MDL_ULTRA_PERFORM_VAL =	0x03,

	LIS3MDL_PERFORMANCE_LEN =	4
};	


static struct {
	uint8_t p_mode;		//performance mode
	uint16_t fsr;		//full scale range
	uint16_t odr_hz;	//sampling rate
} magnet_cfg =	{
	.p_mode = LIS3MDL_LOW_POWER_VAL,
	.fsr = DEF_MAGNET_FSR,
	.odr_hz = DEF_MAGNET_OSR,
};

static status_code_t lis3mdl_i2c_write( uint8_t reg_addr, uint8_t len, uint8_t *data );
static status_code_t lis3mdl_i2c_read( uint8_t reg_addr, uint8_t len, uint8_t *reply );
static int lis3mdl_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data);
uint16_t lis3mdl_read_data( uint8_t * data );
bool lis3mdl_self_test( void );

static status_code_t lis3mdl_i2c_read( uint8_t reg_addr, uint8_t len, uint8_t *reply )
{
	T_TWIM_PACKET packet;
	status_code_t stat;

	packet.i2c_id = LIS3MDL_I2C_ADDRESS;
	packet.reg_addr = reg_addr;
	packet.buffer = reply;	      //pointer to return data
	packet.length = len;
	stat = hal_twim_read(&packet);
	if( stat != ERR_NONE ){
		if (imu_debug) app_trace_puts(DEBUG_LOW, "Mag Read Failed\r");
	}
	return stat;
}

static status_code_t lis3mdl_i2c_write( uint8_t reg_addr, uint8_t len, uint8_t *data )
{
	T_TWIM_PACKET packet;
	status_code_t stat;
	
	packet.i2c_id = LIS3MDL_I2C_ADDRESS;
	packet.reg_addr = reg_addr;
	packet.buffer = data;	      //pointer to data to write
	packet.length = len;
	stat = hal_twim_write(&packet);
	if( stat != ERR_NONE ){
		if (imu_debug) app_trace_puts(DEBUG_LOW, "Mag Write Failed\r");
	}
	return stat;
}

static int lis3mdl_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data)
{
	int err;
	uint8_t new_data = 0x00, old_data = 0x00;
	volatile uint8_t shift = 0;

	err = lis3mdl_i2c_read(reg_addr, 1, &old_data);
	if (err != ERR_NONE) {
		return err;
	}
	
	//Find first Set bit in mask, data needs to be shifted to this point
	while( shift < 7 ) {
		if( ((mask>>shift)&0x01) == 0x01 ) {
			break;
		}
		shift++;
	}

	new_data = ((old_data & (~mask)) | ((data<<shift) & mask));

//	if (new_data == old_data) {
//		//Skip the write. We're not changing anything
//		return ERR_NONE;	
//	}

	return lis3mdl_i2c_write(reg_addr, 1, &new_data);
}

int lis3mdl_init( void )
{
	int i;
	int err;
	uint8_t reply[10];
//	uint8_t data = 0;
	
	if(imu_debug) app_trace_puts(DEBUG_LOW, "Initializing LIS3MDL\r");
	
	nrf_delay_ms(1);
	
	err = lis3mdl_i2c_read( LIS3MDL_WHO_AM_I, 1, reply );
	if( err != ERR_NONE ) {
		//retry
		err = lis3mdl_i2c_read( LIS3MDL_WHO_AM_I, 1, reply );
		if( err != ERR_NONE ) {
			if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL not Found\r");
			return err;
		}
	}	
	app_trace_log(DEBUG_LOW, "LIS3MDL Who Am I: 0x%02X\r", reply[0]);
		
//	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG2, LIS3MDL_REBOOT_MASK, 1 );
//	nrf_delay_ms(50);
	
	err = lis3mdl_i2c_read( LIS3MDL_CTRL_REG1, 5, reply );
	app_trace_log(DEBUG_LOW, "Control Reg Defaults: ");
	for( i = 0; i < 4; i++ )
	{
		app_trace_log(DEBUG_LOW, "0x%02X, ", reply[i]);
	}
	app_trace_log(DEBUG_LOW, "0x%02X\r", reply[i]);
	
	err = lis3mdl_i2c_read( LIS3MDL_INT_CFG, 4, reply );
	app_trace_log(DEBUG_LOW, "Int Reg Defaults: ");
	for( i = 0; i < 3; i++ )
	{
		app_trace_log(DEBUG_LOW, "0x%02X, ", reply[i]);
	}
	app_trace_log(DEBUG_LOW, "0x%02X\r", reply[i]);
	
	err = lis3mdl_i2c_read( LIS3MDL_STATUS_REG, 1, reply );
	app_trace_log(DEBUG_LOW, "Status Reg: 0x%02X\r", reply[0]);
		
	//Set up default parameters
//	err = lis3mdl_set_pm( magnet_cfg.p_mode );
//	err = lis3mdl_set_odr( magnet_cfg.odr_hz );
//	err = lis3mdl_set_fs( magnet_cfg.fsr );
//	err = lis3mdl_set_pm( LIS3MDL_LOW_POWER_VAL );
//	err = lis3mdl_set_odr( LIS3MDL_ODR_0_625HZ_VAL );
//	err = lis3mdl_set_fs( LIS3MDL_FS_4GAUSS_VAL );
//	data = 0;
//	err = lis3mdl_i2c_write( LIS3MDL_INT_CFG, 1, &data );	//Clear Int Cfg Register
		
	//Block Data Updates in the middle of a 16 bit word Read operation
//	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG5, LIS3MDL_BDU_MASK, 1 );
	
	lis3mdl_read_data( reply );
	
	//Power Down Mode
	err = lis3mdl_enable( false );
	if( err != ERR_NONE ) return err;
	
	return ERR_NONE;
}

int lis3mdl_enable( bool on )
{
	int err;
	
	if( on == true ) {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL On\r");
		err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG3, LIS3MDL_MD_MASK, LIS3MDL_MD_CONTINUOS_OP );
	}
	else {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL Off\r");
		err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG3, LIS3MDL_MD_MASK, LIS3MDL_MD_SINGLE_CONV );	//Takes 1 reading and goes to idle mode
	}
	
	if( err != ERR_NONE ) {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL Enable Failed!!!\r");
		return err;
	}
	
	return ERR_NONE;
}

int lis3mdl_set_odr( uint16_t odr )
{
	int i, err;
	 
	for(i = 0; i < LIS3MDL_ODR_TABLE_LEN; i++) {
		if( odr_lut[i].hz >= odr ) {
			break;
		}
	}
	if (i >= LIS3MDL_ODR_TABLE_LEN) return ERR_INVALID_ARG;
	 
	//Set Date Rate
	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG1, LIS3MDL_DO_MASK, odr_lut[i].reg_val );
	if( err != ERR_NONE ) {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL ODR Set Failed!!!\r");
		return err;
	}
	
	//Value Successfully Updated. Now update configured data rate to match implemented data rate
	if( magnet_cfg.odr_hz != odr_lut[i].hz ) magnet_cfg.odr_hz = odr_lut[i].hz;
	 
	return ERR_NONE;
}

int lis3mdl_set_fs( uint16_t fsr )
{
	int i, err;
	
	for(i = 0; i < LIS3MDL_FS_TABLE_LEN; i++) {
		if( fsr_lut[i].fsr >= fsr ) {
			break;
		}
	}
	if (i >= LIS3MDL_FS_TABLE_LEN) return ERR_INVALID_ARG;
	
	//Set Date Rate
	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG2, LIS3MDL_FS_MASK, fsr_lut[i].reg_val );
	if( err != ERR_NONE ) {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL FSR Set Failed!!!\r");
		return err;
	}
	
	//Value Successfully Updated. Now update configured fsr to match implemented fsr
	if( magnet_cfg.fsr != fsr_lut[i].fsr ) magnet_cfg.fsr = fsr_lut[i].fsr;
	
	return ERR_NONE;
}

// Set the Performance Modes for the XY Axes and the Z axis
int lis3mdl_set_pm( uint8_t mode )
{
	int err;
	
	if( mode > LIS3MDL_PERFORMANCE_LEN ) return ERR_INVALID_ARG;
	
	//Set Mode for XY Axes
	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG1, LIS3MDL_OM_MASK, mode );
	if( err != ERR_NONE ) {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL XY Mode Select Failed!!!\r");
		return err;
	}
	
	//Set Mode for Z Axis
	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG4, LIS3MDL_OMZ_MASK, mode );
	if( err != ERR_NONE ) {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL Z Mode Select Failed!!!\r");
		return err;
	}
	
	//Value Successfully Updated. Now update configured performance mode to match implemented mode
	if( magnet_cfg.p_mode != mode ) magnet_cfg.p_mode = mode;
	
	return ERR_NONE;
}

uint16_t lis3mdl_get_odr( void ) 
{
	return magnet_cfg.odr_hz;
}

uint16_t lis3mdl_get_fsr( void ) 
{
	return magnet_cfg.fsr;
}

uint8_t lis3mdl_get_pm( void ) 
{
	return magnet_cfg.p_mode;
}

uint16_t lis3mdl_get_sens( void ) 
{
	uint16_t sensi;
	
	if( magnet_cfg.fsr != 0 ) {
		sensi = 32768/magnet_cfg.fsr;
	}
	else {
		sensi = DEF_MAGNET_SENS;
	}
	
	return sensi;
}

uint16_t lis3mdl_read_data( uint8_t * data ) 
{
	int err;

	err = lis3mdl_i2c_read( LIS3MDL_OUT_X_L, 6, data );
	if( err != ERR_NONE ) {
		if(imu_debug) app_trace_puts(DEBUG_MED, "LIS3MDL Read Error\r");
		return err;
	}	
	
	return err;
}

#define SELF_TEST_SAMPLES	5
// Keep the device still during self-test procedure
bool lis3mdl_self_test( void )
{
	uint8_t data[6];
	int err;
	uint8_t read_cnt = 0;
	int32_t out_nost[3] = {0};
	int32_t out_st[3] = {0};
	uint32_t diff[3];
	uint16_t min_st[3] = { 1.0*GAIN_12GAUSS, 1.0*GAIN_12GAUSS, 0.1*GAIN_12GAUSS };
	uint16_t max_st[3] = { 3.0*GAIN_12GAUSS, 3.0*GAIN_12GAUSS, 1.0*GAIN_12GAUSS };
	
	app_trace_puts(DEBUG_MED, "LIS3MDL Self Test: ");
	
	//write 0x1C to Control Register 1	//Initialize Sensor
	data[0] = 0x1C;
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG1, 1, data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Write Err 1-%01u\r", err);
		return false;
	}
	
	//write 0x40 to Control Register 2	//FS=+-12Gauss, 80Hz 
	data[0] = 0x40;
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG2, 1, data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Write Err 2-%01u\r", err);
		return false;
	}
	
	//Put the Remaining Control Registers into Known State
	data[0] = 0;
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG4, 1, data );	//Clear Register
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG5, 1, data );	//Clear Register
	err = lis3mdl_i2c_write( LIS3MDL_INT_CFG, 1, data );	//Clear Int Cfg Register
	
	//wait 20 ms
	nrf_delay_ms( 20 );
	
	//write 0x00 to Control Register 3	//Turn on sensor, Continuous Mode
	data[0] = 0x00;
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG3, 1, data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Write Err 3-%01u\r", err);
		return false;
	}
	
	//wait 20 ms						//Wait for stable output
	nrf_delay_ms( 20 );
	
	//Wait for first Sample
	data[0] = 0;
	while( (data[0]&LIS3MDL_ZYXDA_MASK) == 0 ) {
		nrf_delay_ms(1);	//slow down read process
		err = lis3mdl_i2c_read( LIS3MDL_STATUS_REG, 1, data );
		if( err != ERR_NONE ) {
			app_trace_log(DEBUG_MED, "Fail, Read Err 1-%01u\r", err);
			return err;
		}	
	}		
	
	//Read OUTX, OUTY, OUTZ				//Read to Clear ZYXDA Bit, discard first Reading
	err = lis3mdl_read_data( data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Read Err 2-%01u\r", err);
		return err;
	}
	
	//Take the next five readings
	read_cnt = 0;
	while( read_cnt < SELF_TEST_SAMPLES )
	{
		//wait for sample to become Ready
		data[0] = 0;
		while( (data[0]&LIS3MDL_ZYXDA_MASK) == 0 ) {
			nrf_delay_ms(1);	//slow down read process
			err = lis3mdl_i2c_read( LIS3MDL_STATUS_REG, 1, data );
			if( err != ERR_NONE ) {
				app_trace_log(DEBUG_MED, "Fail, Read Err 3-%01u\r", err);
				return err;
			}	
		}
		
		//Read OUTX, OUTY, OUTZ	
		err = lis3mdl_read_data( data );
		if( err != ERR_NONE ) {
			app_trace_log(DEBUG_MED, "Fail, Read Err 4-%01u\r", err);
			return err;
		}
		out_nost[0] += ((uint16_t) data[1]<<8 | data[0]);
		out_nost[1] += ((uint16_t) data[3]<<8 | data[2]);
		out_nost[2] += ((uint16_t) data[5]<<8 | data[4]);
		
		read_cnt++;
	}
	
	//average the 5 readings
	out_nost[0] /= SELF_TEST_SAMPLES;
	out_nost[1] /= SELF_TEST_SAMPLES;
	out_nost[2] /= SELF_TEST_SAMPLES;
	
	//write 0x1D to Control Register 1	//Enable Self Test
	data[0] = 0x1D;
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG1, 1, data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Write Err 4-%01u\r", err);
		return false;
	}
	
	//wait 60 ms
	nrf_delay_ms(60);
	
	//Wait for first Sample
	data[0] = 0;
	while( (data[0]&LIS3MDL_ZYXDA_MASK) == 0 ) {
		nrf_delay_ms(1);	//slow down read process
		err = lis3mdl_i2c_read( LIS3MDL_STATUS_REG, 1, data );
		if( err != ERR_NONE ) {
			app_trace_log(DEBUG_MED, "Fail, Read Err 5-%01u\r", err);
			return err;
		}	
	}		
	
	//Read OUTX, OUTY, OUTZ				//Read to Clear ZYXDA Bit, discard first Reading
	err = lis3mdl_read_data( data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Read Err 6-%01u\r", err);
		return err;
	}
	
	//Take the next five readings
	read_cnt = 0;
	while( read_cnt < SELF_TEST_SAMPLES )
	{
		//wait for sample to become Ready
		data[0] = 0;
		while( (data[0]&LIS3MDL_ZYXDA_MASK) == 0 ) {
			nrf_delay_ms(1);	//slow down read process
			err = lis3mdl_i2c_read( LIS3MDL_STATUS_REG, 1, data );
			if( err != ERR_NONE ) {
				app_trace_log(DEBUG_MED, "Fail, Read Err 7-%01u\r", err);
				return err;
			}	
		}
		
		//Read OUTX, OUTY, OUTZ	
		err = lis3mdl_read_data( data );
		if( err != ERR_NONE ) {
			app_trace_log(DEBUG_MED, "Fail, Read Err 8-%01u\r", err);
			return err;
		}
		out_st[0] += ((uint16_t) data[1]<<8 | data[0]);
		out_st[1] += ((uint16_t) data[3]<<8 | data[2]);
		out_st[2] += ((uint16_t) data[5]<<8 | data[4]);
		
		read_cnt++;
	}
	
	//average the 5 readings
	out_st[0] /= SELF_TEST_SAMPLES;
	out_st[1] /= SELF_TEST_SAMPLES;
	out_st[2] /= SELF_TEST_SAMPLES;
	
	//write 0x1C to Control Register 1	//Disable Self Test
	data[0] = 0x1C;
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG1, 1, data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Write Err 5-%01u\r", err);
		return false;
	}
	
	//write 0x03 to Control Register 3	//Power Down Mode
	data[0] = 0x03;
	err = lis3mdl_i2c_write( LIS3MDL_CTRL_REG3, 1, data );
	if( err != ERR_NONE ) {
		app_trace_log(DEBUG_MED, "Fail, Write Err 6-%01u\r", err);
		return false;
	}	
	
	//Calculate the |differences| in the 2 average readings
	int16_t itemp = out_st[0]-out_nost[0];
	diff[0] = abs( itemp );
	itemp = out_st[1]-out_nost[1];
	diff[1] = abs( itemp );
	itemp = out_st[2]-out_nost[2];
	diff[2] = abs( itemp );
	
	if( (diff[0] <= min_st[0]) || (diff[0] >= max_st[0]) ) {
		//fail
		app_trace_log(DEBUG_MED, "Fail X : %01u\r", diff[0]);
		return false;
	}
	if( (diff[1] < min_st[1]) || (diff[1] > max_st[1]) ) {
		//fail
		app_trace_log(DEBUG_MED, "Fail Y : %01u\r", diff[1]);
		return false;
	}
	if( (diff[2] < min_st[2]) || (diff[2] > max_st[2]) ) {
		//fail
		app_trace_log(DEBUG_MED, "Fail Z : %01u\r", diff[2]);
		return false;
	}
	
	app_trace_puts(DEBUG_MED, "Pass\r");
	
	return true;
}

