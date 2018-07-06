/*
 * lis3mdl.c
 *
 * Created: 8/20/2015 11:52:26 AM
 *  Author: matt
 */ 

#include "lis3mdl.h"

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

//Full Scale Ranges
enum {
	LIS3MDL_FS_4GAUSS_VAL =		0x00,
	LIS3MDL_FS_8GAUSS_VAL =		0x01,
	LIS3MDL_FS_12GAUSS_VAL =	0x02,
	LIS3MDL_FS_16GAUSS_VAL =	0x03,
	
	LIS3MDL_FS_TABLE_LEN =		4
};

typedef struct {
	uint8_t fsr;
	uint8_t reg_val;
} T_LIS3MDL_FSR;

T_LIS3MDL_FSR fsr_lut[LIS3MDL_FS_TABLE_LEN] = {
	[0] = { .fsr = 4, .reg_val = LIS3MDL_FS_4GAUSS_VAL },
	[1] = { .fsr = 8, .reg_val = LIS3MDL_FS_8GAUSS_VAL },
	[2] = { .fsr = 12, .reg_val = LIS3MDL_FS_12GAUSS_VAL },
	[3] = { .fsr = 16, .reg_val = LIS3MDL_FS_16GAUSS_VAL },
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
		if (imu_debug) app_trace_log("Mag Read Failed\r");
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
		if (imu_debug) app_trace_log("Mag Write Failed\r");
	}
	return stat;
}

static int lis3mdl_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data)
{
	int err;
	uint8_t new_data = 0x00, old_data = 0x00;
	volatile uint8_t shift = 0;

	err = lis3mdl_i2c_read(reg_addr, 1, &old_data);
	if (err != 0) return err;
	
	//Find first Set bit in mask, data needs to be shifted to this point
	while( shift < 7 ) {
		if( ((mask>>shift)&0x01) == 0x01 ) {
			break;
		}
		shift++;
	}

	new_data = ((old_data & (~mask)) | ((data<<shift) & mask));

	if (new_data == old_data)
	return 1;

	return lis3mdl_i2c_write(reg_addr, 1, &new_data);
}

int lis3mdl_init( void )
{
	int err;
	
	//Set up default parameters
	err = lis3mdl_set_pm( magnet_cfg.p_mode );
	err = lis3mdl_set_odr( magnet_cfg.odr_hz );
	err = lis3mdl_set_fs( magnet_cfg.fsr );
	
	//Block Data Updates in the middle of a 16 bit word Read operation
	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG5, LIS3MDL_BDU_MASK, 1 );
	
	//Put into Idle Mode
	err = lis3mdl_enable( false );
	if( err != 0 ) return err;
	
	return 0;
}

int lis3mdl_enable( bool on )
{
	int err;
	
	if( on == true ) {
		err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG3, LIS3MDL_MD_MASK, LIS3MDL_MD_CONTINUOS_OP );
	}
	else {
		err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG3, LIS3MDL_MD_MASK, LIS3MDL_MD_POWER_DOWN );
	}
	if( err != 0 ) return err;
	
	return 0;
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
	if( err != 0 ) {
		if(imu_debug) app_trace_log("LIS3MDL ODR Set Failed!!!\r");
		return err;
	}
	
	//Value Successfully Updated. Now update configured data rate to match implemented data rate
	if( magnet_cfg.odr_hz != odr_lut[i].hz ) magnet_cfg.odr_hz = odr_lut[i].hz;
	 
	return 0;
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
	if( err != 0 ) {
		if(imu_debug) app_trace_log("LIS3MDL FSR Set Failed!!!\r");
		return err;
	}
	
	//Value Successfully Updated. Now update configured fsr to match implemented fsr
	if( magnet_cfg.fsr != fsr_lut[i].fsr ) magnet_cfg.fsr = fsr_lut[i].fsr;
	
	return 0;
}

// Set the Performance Modes for the XY Axes and the Z axis
int lis3mdl_set_pm( uint8_t mode )
{
	int err;
	
	if( mode > LIS3MDL_PERFORMANCE_LEN ) return ERR_INVALID_ARG;
	
	//Set Mode for XY Axes
	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG1, LIS3MDL_OM_MASK, mode );
	if( err != 0 ) {
		if(imu_debug) app_trace_log("LIS3MDL Mode Select Failed!!!\r");
		return err;
	}
	
	//Set Mode for Z Axis
	err = lis3mdl_write_data_with_mask( LIS3MDL_CTRL_REG4, LIS3MDL_OMZ_MASK, mode );
	if( err != 0 ) {
		if(imu_debug) app_trace_log("LIS3MDL Mode Select Failed!!!\r");
		return err;
	}
	
	//Value Successfully Updated. Now update configured performance mode to match implemented mode
	if( magnet_cfg.p_mode != mode ) magnet_cfg.p_mode = mode;
	
	return 0;
}

uint16_t lis3mdl_get_odr( void ) {
	return magnet_cfg.odr_hz;
}

uint16_t lis3mdl_get_fsr( void ) {
	return magnet_cfg.fsr;
}

uint8_t lis3mdl_get_pm( void ) {
	return magnet_cfg.p_mode;
}

uint16_t lis3mdl_get_sens( void ) {
	uint16_t sensi;
	
	if( magnet_cfg.fsr != 0 ) {
		sensi = 32768/magnet_cfg.fsr;
	}
	else {
		sensi = DEF_MAGNET_SENS;
	}
	
	return sensi;
}
