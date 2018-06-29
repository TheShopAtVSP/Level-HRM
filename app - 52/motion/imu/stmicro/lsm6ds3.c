/*
 * Started with STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 *
 * Ported to C: 6/29/2015 2:25:07 PM
 */

#include "lsm6ds3.h"

#ifdef POLLING_MODE
#define LSM6DSW3_POLLING_MODE 
#else 
#define LSM6DS3_FIFO_MODE	//Enable Code to use FIFO
#endif

#define FIFO_DEBUG	false

#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)				(((a) < (b)) ? (a) : (b))
#endif

static struct {
	uint16_t sensor_en;
	struct {
		uint16_t fsr;		//full scale range
		uint16_t odr_hz;	//sampling rate
	}accel;
	struct {
		uint16_t fsr;		//full scale range
		uint16_t odr_hz;	//sampling rate
	}gyro;
	struct {
		uint16_t odr_hz;
		uint16_t buf_byte_thres;
		uint16_t data_set_thres;
	}fifo;
	bool motion_event_ready;
} imu_cfg =	{
	.sensor_en = 0,
	.accel = {
		.fsr = DEF_ACCEL_FSR,
		.odr_hz = DEF_ACCEL_OSR,
	},
	.gyro = {
		.fsr = DEF_GYRO_FSR,
		.odr_hz = DEF_GYRO_OSR,
	},
	.fifo = {
		.odr_hz = 0,
		.buf_byte_thres = 0,
		.data_set_thres = 0,
	},
	.motion_event_ready = false,
};

static struct {
	uint8_t list[ FIFO_MAX_PATTERN ];
	uint16_t samps_pat[ FIFO_DATA_MAX_SRC_CNT ];
} fifo_pattern;

static const struct lsm6ds3_odr_table{
	uint8_t addr[2];
	uint8_t mask[2];
	T_LSM6DS3_ODR_REG odr[ODR_CNT];
} lsm6ds3_odr_table = {
	.addr[LSM6DS3_ACCEL] = LSM6DS3_CTRL1_XL,
	.mask[LSM6DS3_ACCEL] = CTRL1_XL_ODR_MASK,
	.addr[LSM6DS3_GYRO] = LSM6DS3_CTRL2_G,
	.mask[LSM6DS3_GYRO] = CTRL2_GYRO_ODR_MASK,
	.odr[0] = { .hz = 0, .reg_val = ODR_POWER_OFF_VAL },
	.odr[1] = { .hz = 13, .reg_val = ODR_13HZ_VAL },
	.odr[2] = { .hz = 26, .reg_val = ODR_26HZ_VAL },
	.odr[3] = { .hz = 52, .reg_val = ODR_52HZ_VAL },
	.odr[4] = { .hz = 104, .reg_val = ODR_104HZ_VAL },
	.odr[5] = { .hz = 208, .reg_val = ODR_208HZ_VAL },
	.odr[6] = { .hz = 416, .reg_val = ODR_416HZ_VAL },
};

#define LSM6DS3_FS_LIST_NUM		4
static const struct lsm6ds3_fs_table{
	uint8_t addr;
	uint8_t mask;
	T_LSM6DS3_FS_REG fs_avl[LSM6DS3_FS_LIST_NUM];
} lsm6ds3_fs_table[2] = {
	[LSM6DS3_ACCEL] = {
		.addr = LSM6DS3_CTRL1_XL,
		.mask = CTRL1_XL_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_ACCEL_FS_2G_GAIN,
					.value = LSM6DS3_ACCEL_FS_2G_VAL,
					.urv = 2, },
		.fs_avl[1] = { .gain = LSM6DS3_ACCEL_FS_4G_GAIN,
					.value = LSM6DS3_ACCEL_FS_4G_VAL,
					.urv = 4, },
		.fs_avl[2] = { .gain = LSM6DS3_ACCEL_FS_8G_GAIN,
					.value = LSM6DS3_ACCEL_FS_8G_VAL,
					.urv = 8, },
		.fs_avl[3] = { .gain = LSM6DS3_ACCEL_FS_16G_GAIN,
					.value = LSM6DS3_ACCEL_FS_16G_VAL,
					.urv = 16, },
	},
	[LSM6DS3_GYRO] = {
		.addr = LSM6DS3_CTRL2_G,
		.mask = CTRL2_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_GYRO_FS_245_GAIN,
					.value = LSM6DS3_GYRO_FS_245_VAL,
					.urv = 245, },
		.fs_avl[1] = { .gain = LSM6DS3_GYRO_FS_500_GAIN,
					.value = LSM6DS3_GYRO_FS_500_VAL,
					.urv = 500, },
		.fs_avl[2] = { .gain = LSM6DS3_GYRO_FS_1000_GAIN,
					.value = LSM6DS3_GYRO_FS_1000_VAL,
					.urv = 1000, },
		.fs_avl[3] = { .gain = LSM6DS3_GYRO_FS_2000_GAIN,
					.value = LSM6DS3_GYRO_FS_2000_VAL,
					.urv = 2000, },
	}
};

//include function pointers to the I2C or SPI communication routines
extern const lsm6ds3_comm_t lsm6ds3_read;
extern const lsm6ds3_comm_t lsm6ds3_write;

//prototypes
void (* save_gyro_data) ( uint8_t *data ) = NULL;
void (* save_accel_data) ( uint8_t *data ) = NULL;
void (* save_extern_data) ( uint8_t *data ) = NULL;
void (* save_steps_data) ( uint8_t *data ) = NULL;
static ret_code_t lsm6ds3_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data);
static ret_code_t lsm6ds3_set_fs( T_LSM6DS3_SENSOR_TYPE s_type, int32_t fsr );
static ret_code_t lsm6ds3_set_odr( T_LSM6DS3_SENSOR_TYPE s_type, uint16_t * odr_hz);
static ret_code_t lsm6ds3_set_irq( T_LSM6DS3_SENSOR_TYPE s_type, bool on_off );
//static ret_code_t lsm6ds3_configure_pedo( void );
static ret_code_t lsm6ds3_enable_pedometer( bool enable );
static ret_code_t lsm6ds3_get_step_data( uint16_t *steps );
static ret_code_t lsm6ds3_enable_embedded_func( bool enable, T_SENSOR_ODR_CODE odr );
bool write_external_sensor( uint8_t id, uint8_t reg_addr, uint8_t data );
bool lsm6ds3_init_lis3mdl( bool load_calibration );
static ret_code_t lsm6ds3_set_fifo_mode( T_FIFO_MODES fm );
static ret_code_t lsm6ds3_set_fifo_decimators_and_threshold( uint16_t * fifo_len );
static ret_code_t lsm6ds3_reconfigure_fifo( void );
static ret_code_t lsm6ds3_read_fifo( bool check_fifo_len );
static void lsm6ds3_parse_fifo_data( uint8_t * data_buf, uint16_t read_len, uint16_t first_pattern );	

void gyro_save_init( save_fifo_func_ptr save_gyro_func )
{
	save_gyro_data = save_gyro_func;
}

void accel_save_init( save_fifo_func_ptr save_accel_func )
{
	save_accel_data = save_accel_func;
}

void extern_save_init( save_fifo_func_ptr save_extern_func )
{
	save_extern_data = save_extern_func;
}

void steps_save_init( save_fifo_func_ptr save_steps_func )
{
	save_steps_data = save_steps_func;
}

uint16_t lsm6ds3_active_sensors(void)
{	
	return imu_cfg.sensor_en;
}

ret_code_t lsm6ds3_init_sensors( void )
{
	ret_code_t err = NRF_SUCCESS;
	uint8_t temp_reg_val = 0;
	
	imu_cfg.sensor_en = 0;
	
	//Make sure Sampling Rates are within Possible Bounds
	if( imu_cfg.accel.odr_hz < 1 ) {
		//can't be done
		imu_cfg.accel.odr_hz = 1;	//minimum rate
	}
	else if( imu_cfg.accel.odr_hz > LSM6DS3_MAX_ODR_HZ ) {
		imu_cfg.accel.odr_hz = LSM6DS3_MAX_ODR_HZ;	//maximum rate
	}
	if( imu_cfg.gyro.odr_hz < 1 ) {
		//can't be done
		imu_cfg.gyro.odr_hz = 1;	//minimum rate
	}
	else if( imu_cfg.gyro.odr_hz > LSM6DS3_MAX_ODR_HZ ) {
		imu_cfg.gyro.odr_hz = LSM6DS3_MAX_ODR_HZ;	//maximum rate
	}

	// Issue Software Reset to make sure registers are at Default Settings
	err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL3_C, CTRL3_SW_RESET_MASK, LSM6DS3_EN_BIT);
	if( err != 0 ) return err;
	do {
		//Stall while device runs its Boot routine. Bit will autoclear when complete
		delay_ms( 1 );		
		err = lsm6ds3_read( LSM6DS3_CTRL3_C, 1, &temp_reg_val);
		if( err != 0 ) return err;
	}
	while( (temp_reg_val&CTRL3_SW_RESET_MASK) != 0x00 );
	
	//Check Hardware Who Am I Register. Expected Return: LSM6DS3_DEVICE_ID
	err = lsm6ds3_read( LSM6DS3_WHO_AM_I_REG, 1, &temp_reg_val );
	if (imu_debug) app_trace_log(DEBUG_MED, "[INIT_XL]: Who: 0x%02X\r", temp_reg_val);
	
	//Read and Print Control Register to check default state
	//err = lsm6ds3_read( LSM6DS3_CTRL8_XL_ADDR, 1, &temp_reg_val);
	//if( imu_debug ) app_trace_log(DEBUG_LOW, "Ctrl 8: 0x%02X\r", temp_reg_val);
	
	//Set Up Accelerometer
	err = lsm6ds3_set_fs( LSM6DS3_ACCEL, DEF_ACCEL_FSR );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_CTRL6_G, CTRL6_XL_HPERF_MODE_MASK, LSM6DS3_HPERF_DISABLE );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_WAKE_UP_THS, WAKE_UP_THS_MASK, 0x01 );			//Wake Threshold = X*(Accel FSR)/64
	err = lsm6ds3_write_data_with_mask( LSM6DS3_WAKE_UP_DUR, WAKE_UP_DUR_WAKE_DUR_MASK, 0x01 );	//Wake Duration = X*(Accel Samp Rate)
	err = lsm6ds3_write_data_with_mask( LSM6DS3_TAP_CFG, TAP_CFG_SLOPE_FDS_MASK, 0 );			//0: Slope Filter, 1: HP Filter for Wake Up
	if ( err != NRF_SUCCESS ) return err;
	
	//Set up Gyrometer
	err = lsm6ds3_set_fs( LSM6DS3_GYRO, DEF_GYRO_FSR );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_CTRL7_G, CTRL7_GYRO_HPERF_MODE_MASK, LSM6DS3_HPERF_DISABLE );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_CTRL7_G, CTRL7_GYRO_HPF_CFG_MASK, LSM6DS3_GYRO_HPF_0_0324_HZ );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_CTRL7_G, CTRL7_GYRO_HPF_EN_MASK, LSM6DS3_EN_BIT );
	if ( err != NRF_SUCCESS ) return err;

	//Interrupt Sources latch until read to clear
	err = lsm6ds3_write_data_with_mask( LSM6DS3_TAP_CFG, TAP_CFG_LATCH_INT_MASK, LSM6DS3_EN_BIT );
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_INT_HLACTIVE_ADDR, LSM6DS3_INT_HLACTIVE_MASK, LSM6DS3_DIS_BIT );	//Active High when Disabled (default active High)
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_INT_PP_OD_ADDR, LSM6DS3_INT_PP_OD_MASK, LSM6DS3_DIS_BIT );	//Open Drain or Push-Pull when Disabled (default Push-Pull)
	if ( err != NRF_SUCCESS ) return err;
	
	//Block Data Registers from loading new readings between the reading of the LSB and MSB
	err = lsm6ds3_write_data_with_mask( LSM6DS3_CTRL3_C, CTRL3_BDU_MASK, LSM6DS3_EN_BIT );
	if ( err != NRF_SUCCESS ) return err;
	
	//Enable Circular Buffering of Data Registers
	err = lsm6ds3_write_data_with_mask( LSM6DS3_CTRL7_G, CTRL7_ROUNDING_STAT_MASK, LSM6DS3_EN_BIT );
	if ( err != NRF_SUCCESS ) return err;
	
	//All Enabled Interrupt2 Sources indicate on Int1 Output
	err = lsm6ds3_write_data_with_mask( LSM6DS3_CTRL4_C, CTRL4_INT2_ON_INT1_MASK, LSM6DS3_EN_BIT );
	if ( err != NRF_SUCCESS ) return err;
	
	//Route the desired Function Interrupts to indicate on the Int1 pin
	temp_reg_val = MD1_CFG_INT1_WAKE_UP;
	err = lsm6ds3_write( LSM6DS3_MD1_CFG, 1, &temp_reg_val );
	if (err != NRF_SUCCESS) return err;

#if defined( LIS3MDL )	
	if( lsm6ds3_init_lis3mdl( false ) == false ) {
		if (imu_debug) app_trace_log(DEBUG_MED, "Compass Init fail\r");
	}
#endif

#if defined (LSM6DS3_FIFO_MODE)
	err = lsm6ds3_reconfigure_fifo();
	if (err != NRF_SUCCESS) return err;
#endif
	
	//Enable Interrupts
	err = lsm6ds3_write_data_with_mask( LSM6DS3_TAP_CFG, TAP_CFG_FUNC_INT_EN_MASK, LSM6DS3_EN_BIT );
	if ( err != NRF_SUCCESS ) return err;
	
	return err;
}

#define MIN_ST_A			((90*32768L)/2000)		//Reading that corresonds to 90 milliG when FS = +-2000 mG (1474)
#define MAX_ST_A			((1700*32768L)/2000)	//Reading that corresonds to 1700 milliG when FS = +-2000 mG (27852)
#define MIN_ST_G_2000DPS	((150*32768L)/2000)		//Reading that corresonds to 150 dps when FS = +-2000 dps (2457)
#define MAX_ST_G_2000DPS	((700*32768L)/2000)		//Reading that corresonds to 700 dps when FS = +-2000 dps (11468)
#define AVE_CNT				5						//Going to Average 5 readings
static ret_code_t lsm6ds3_self_test( void ) 
{
	ret_code_t err = NRF_SUCCESS;
	uint8_t xl_test_setting[10] = { 0x30, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00 };
	uint8_t gy_test_setting[10] = { 0x00, 0x5C, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38 };
	uint8_t wr_val;
	uint8_t read_cnt;
	int16_t data[3];
	int32_t accum_nost[3];
	int32_t accum_st[3];
	int32_t res[3];
	TTASK_TIMER to;
	
	//Accel Self Test: Enable Ax/Ay/Az, Set BDU = 1, ODR = 52Hz, FS = +-2G
	err = lsm6ds3_write( LSM6DS3_CTRL1_XL, 10, xl_test_setting );
	if( err != NRF_SUCCESS )
	{
		app_trace_log(DEBUG_HIGH, "[ST]: XL Test failed 0x%02X\r", err);
		return err;
	}
	
	//Turn on Accel Data Ready Output on Int1 Pin
	wr_val = INT1_CTRL_DRDY_ACCEL_MASK;
	lsm6ds3_write( LSM6DS3_INT1_CTRL, 1, &wr_val );
	
	//Wait 200ms for Stable Output:
	nrf_delay_ms( 200 );
	
	//Read and discard the data, Reading Accel XYZ clears the XLDA bit in the Status Register and Int1
	lsm6ds3_read( LSM6DS3_OUTX_L_XL, 6, (uint8_t *)data );
	
	//Acumulate next 5 readings of Ax, Ay, Az
	accum_nost[0] = accum_nost[1] = accum_nost[2] = 0;
	read_cnt = 0;
	start_task_timer( to, 250);		//wait upto 250 ms for transfer to complete
	while( read_cnt < AVE_CNT )
	{
		if( nrf_gpio_pin_read(AXIS_INT1) == LSM6DS3_INT_ACTIVE )
		{	//Interrupt Pin Set, Reading Data should clear it
			lsm6ds3_read( LSM6DS3_OUTX_L_XL, 6, (uint8_t *)data );
			accum_nost[0] += data[0];
			accum_nost[1] += data[1];
			accum_nost[2] += data[2];
			//app_trace_log(DEBUG_MED, "[ST]: Rd%01u X:%01d Y:%01d Z:%01d @%01u\r", read_cnt, data[0], data[1], data[2], getSystemTimeMs());
			read_cnt++;	
			
			if( nrf_gpio_pin_read(AXIS_INT1) == LSM6DS3_INT_ACTIVE )
			{	//Should be inactive after reading
				app_trace_log(DEBUG_HIGH, "[ST]:  XL_NOST INT1 ERR\r");
				return NRF_ERROR_INVALID_STATE;
			}
		}
		
		if( task_time(to) ) {
			app_trace_log(DEBUG_HIGH, "[ST]: XL_NOST Timeout\r");
			return NRF_ERROR_TIMEOUT;
		}
	}	
	
	//Enable Self Test
	wr_val = 0x01;	
	lsm6ds3_write( LSM6DS3_CTRL5_C, 1, &wr_val );
	
	//Wait 200ms for Stable Output:
	nrf_delay_ms( 200 );
	
	//Read and discard the data, Reading Accel XYZ clears the XLDA bit in the Status Register
	lsm6ds3_read( LSM6DS3_OUTX_L_XL, 6, (uint8_t *)data );
	
	//Acumulate next 5 readings of Ax, Ay, Az
	accum_st[0] = accum_st[1] = accum_st[2] = 0;
	read_cnt = 0;
	start_task_timer( to, 250);		//wait upto 250 ms for transfer to complete
	while( read_cnt < AVE_CNT )
	{
		if( nrf_gpio_pin_read(AXIS_INT1) == LSM6DS3_INT_ACTIVE )
		{	//Interrupt Pin Set, Reading Data should clear it
			lsm6ds3_read( LSM6DS3_OUTX_L_XL, 6, (uint8_t *)data );
			accum_st[0] += data[0];
			accum_st[1] += data[1];
			accum_st[2] += data[2];
			//app_trace_log(DEBUG_MED, "[ST]: Rd%01u X:%01d Y:%01d Z:%01d @%01u\r", read_cnt, data[0], data[1], data[2], getSystemTimeMs());
			read_cnt++;	
			
			if( nrf_gpio_pin_read(AXIS_INT1) == LSM6DS3_INT_ACTIVE )
			{	//Should be inactive after reading
				app_trace_log(DEBUG_HIGH, "[ST]: XL_ST INT1 ERR\r");
				return NRF_ERROR_INVALID_STATE;
			}
		}
		
		if( task_time(to) ) {
			app_trace_log(DEBUG_HIGH, "[ST]: XL_ST Timeout\r");
			return NRF_ERROR_TIMEOUT;
		}
	}	
	
	//Write 0x00 to CTRL1_XL	//Disable Sensor
	wr_val = 0;
	lsm6ds3_write( LSM6DS3_CTRL1_XL, 1, &wr_val );
	//Write 0x00 to CTRL5_C		//Disable Self Test
	wr_val = 0;
	lsm6ds3_write( LSM6DS3_CTRL5_C, 1, &wr_val );
	
	//Calculate difference between the sets of Readings
	res[0] = (accum_st[0] - accum_nost[0]) / AVE_CNT;
	if( res[0] < 0 ) res[0] *= -1;
	res[1] = (accum_st[1] - accum_nost[1]) / AVE_CNT;
	if( res[1] < 0 ) res[1] *= -1;
	res[2] = (accum_st[2] - accum_nost[2]) / AVE_CNT;
	if( res[2] < 0 ) res[2] *= -1;
	
	if( res[0] < MIN_ST_A || res[0] > MAX_ST_A )
	{
		//Fail
		app_trace_log(DEBUG_HIGH, "[ST_A X]: 0x%04X\r", res[0]);
		return NRF_ERROR_INVALID_DATA;
	}
	else if( res[1] < MIN_ST_A || res[1] > MAX_ST_A )
	{
		//Fail
		app_trace_log(DEBUG_HIGH, "[ST_A Y]: 0x%04X\r", res[1]);
		return NRF_ERROR_INVALID_DATA;
	}
	else if( res[2] < MIN_ST_A || res[2] > MAX_ST_A )
	{
		//Fail
		app_trace_log(DEBUG_HIGH, "[ST_A Z]: 0x%04X\r", res[2]);
		return NRF_ERROR_INVALID_DATA;
	}
	else
	{
		//Accel Pass
		app_trace_log(DEBUG_MED, "[ST]: A Pass: %01d, %01d, %01d\r", res[0], res[1], res[2]);
	}
		
	//GYRO Self Test: Enable Gx/Gy/Gz, Set BDU = 1, ODR = 208Hz, FS = +-2000dps
	lsm6ds3_write( LSM6DS3_CTRL1_XL, 10, gy_test_setting );
	
	//Turn on Gyro Data Ready Output on Int1 Pin
	wr_val = INT2_CTRL_DRDY_GYRO_MASK;
	lsm6ds3_write( LSM6DS3_INT2_CTRL, 1, &wr_val );
	
	//Wait 800ms for Stable Output:
	nrf_delay_ms( 800 );
	
	//Read and discard the data, Reading Gyro XYZ clears the GDA bit in the Status Register
	lsm6ds3_read( LSM6DS3_OUTX_L_G, 6, (uint8_t *)data );
	
	//Acumulate next 5 readings of Gx, Gy, Gz
	accum_nost[0] = accum_nost[1] = accum_nost[2] = 0;
	read_cnt = 0;
	start_task_timer( to, 250);		//wait upto 250 ms for transfer to complete
	while( read_cnt < AVE_CNT )
	{
		if( nrf_gpio_pin_read(AXIS_INT2) == LSM6DS3_INT_ACTIVE )
		{	//Interrupt Pin Set, Reading Data should clear it
			lsm6ds3_read( LSM6DS3_OUTX_L_G, 6, (uint8_t *)data );
			accum_nost[0] += data[0];
			accum_nost[1] += data[1];
			accum_nost[2] += data[2];
			//app_trace_log(DEBUG_MED, "[ST]: Rd%01u X:%01d Y:%01d Z:%01d @%01u\r", read_cnt, data[0], data[1], data[2], getSystemTimeMs());
			read_cnt++;	
			
			if( nrf_gpio_pin_read(AXIS_INT2) == LSM6DS3_INT_ACTIVE )
			{	//Should be inactive after reading
				app_trace_log(DEBUG_HIGH, "[ST]: G_NOST INT1 ERR\r");
				return NRF_ERROR_INVALID_STATE;
			}
		}
		
		if( task_time(to) ) {
			app_trace_log(DEBUG_HIGH, "[ST]: G_NOST Timeout\r");
			return NRF_ERROR_TIMEOUT;
		}
	}	
	
	//Enable Self Test
	wr_val = 0x04;	
	lsm6ds3_write( LSM6DS3_CTRL5_C, 1, &wr_val );
	
	//Wait 60ms for Stable Output:
	nrf_delay_ms( 60 );
	
	//Read and discard the data, Reading Gyro XYZ clears the GDA bit in the Status Register
	lsm6ds3_read( LSM6DS3_OUTX_L_G, 6, (uint8_t *)data );
	
	//Acumulate next 5 readings of Gx, Gy, Gz
	accum_st[0] = accum_st[1] = accum_st[2] = 0;
	read_cnt = 0;
	start_task_timer( to, 250);		//wait upto 250 ms for transfer to complete
	while( read_cnt < AVE_CNT )
	{
		if( nrf_gpio_pin_read(AXIS_INT2) == LSM6DS3_INT_ACTIVE )
		{	//Interrupt Pin Set, Reading Data should clear it
			lsm6ds3_read( LSM6DS3_OUTX_L_G, 6, (uint8_t *)data );
			accum_st[0] += data[0];
			accum_st[1] += data[1];
			accum_st[2] += data[2];
			//app_trace_log(DEBUG_MED, "[ST]: Rd%01u X:%01d Y:%01d Z:%01d @%01u\r", read_cnt, data[0], data[1], data[2], getSystemTimeMs());
			read_cnt++;	
			
			if( nrf_gpio_pin_read(AXIS_INT2) == LSM6DS3_INT_ACTIVE )
			{	//Should be inactive after reading
				app_trace_log(DEBUG_HIGH, "[ST]: G_ST INT1 ERR\r");
				return NRF_ERROR_INVALID_STATE;
			}
		}
		
		if( task_time(to) ) {
			app_trace_log(DEBUG_HIGH, "[ST]: G_ST Timeout\r");
			return NRF_ERROR_TIMEOUT;
		}
	}
	
	//Write 0x00 to CTRL2_G		//Disable Sensor
	wr_val = 0;
	lsm6ds3_write( LSM6DS3_CTRL2_G, 1, &wr_val );
	//Write 0x00 to CTRL5_C		//Disable Self Test
	wr_val = 0;
	lsm6ds3_write( LSM6DS3_CTRL5_C, 1, &wr_val );
	
	//Calculate difference between the sets of Readings
	res[0] = (accum_st[0] - accum_nost[0]) / AVE_CNT;
	if( res[0] < 0 ) res[0] *= -1;
	res[1] = (accum_st[1] - accum_nost[1]) / AVE_CNT;
	if( res[1] < 0 ) res[1] *= -1;
	res[2] = (accum_st[2] - accum_nost[2]) / AVE_CNT;
	if( res[2] < 0 ) res[2] *= -1;
	
	if( res[0] < MIN_ST_G_2000DPS || res[0] > MAX_ST_G_2000DPS )
	{
		//Fail
		app_trace_log(DEBUG_HIGH, "[ST_G X]: 0x%04X\r", res[0]);
		return NRF_ERROR_INVALID_DATA;
	}
	else if( res[1] < MIN_ST_G_2000DPS || res[1] > MAX_ST_G_2000DPS )
	{
		//Fail
		app_trace_log(DEBUG_HIGH, "[ST_G Y]: 0x%04X\r", res[1]);
		return NRF_ERROR_INVALID_DATA;
	}
	else if( res[2] < MIN_ST_G_2000DPS || res[2] > MAX_ST_G_2000DPS )
	{
		//Fail
		app_trace_log(DEBUG_HIGH, "[ST_G Z]: 0x%04X\r", res[2]);
		return NRF_ERROR_INVALID_DATA;
	}
	else
	{
		//Gyro Pass
		app_trace_log(DEBUG_MED, "[ST]: G Pass: %01d, %01d, %01d\r", res[0], res[1], res[2]);
	}
	
	return err;
}

ret_code_t lsm6ds3_run_self_test( void )
{
	ret_code_t err;
	ret_code_t res;
	uint8_t cntrl_copy[10];
	uint8_t int_copy[2];
	uint8_t md_copy[2];
	uint8_t clear[2] = { 0, 0 };
	
	//Get copies of any registers that will be modified:
	err = lsm6ds3_read( LSM6DS3_MD1_CFG, 2, md_copy );
	err |= lsm6ds3_read( LSM6DS3_INT1_CTRL, 2, int_copy );
	err |= lsm6ds3_read( LSM6DS3_CTRL1_XL, 10, cntrl_copy );
	if( err != NRF_SUCCESS )
	{
		app_trace_log(DEBUG_HIGH, "[ST]: Copy failed 0x%02X\r", err);
		return err;
	}
	lsm6ds3_write( LSM6DS3_MD1_CFG, 2, clear );		//turn off wake on motion Interrupt
	lsm6ds3_write( LSM6DS3_INT1_CTRL, 2, clear );
	
	res = lsm6ds3_self_test();
	
	//Restore those registers to their previous values:
	err = lsm6ds3_write( LSM6DS3_CTRL1_XL, 10, cntrl_copy );
	err |= lsm6ds3_write( LSM6DS3_MD1_CFG, 2, md_copy );
	err |= lsm6ds3_write( LSM6DS3_INT1_CTRL, 2, int_copy );
	if( err != NRF_SUCCESS )
	{
		app_trace_log(DEBUG_HIGH, "[ST]: Restore failed 0x%02X\r", err);
		return err;
	}
	
	return res;
}

static ret_code_t lsm6ds3_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data)
{
	int err;
	uint8_t new_data = 0x00, old_data = 0x00;
	volatile uint8_t shift = 0;

	err = lsm6ds3_read(reg_addr, 1, &old_data);
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
		return 0;

	return lsm6ds3_write(reg_addr, 1, &new_data);
}

uint16_t lsm6ds3_get_accel_fsr( void ) 
{
	return imu_cfg.accel.fsr;
}

uint16_t lsm6ds3_get_gyro_fsr( void ) 
{
	return imu_cfg.gyro.fsr;
}

uint16_t lsm6ds3_get_accel_samp( void ) 
{	
	return imu_cfg.accel.odr_hz;
}

uint16_t lsm6ds3_get_gyro_samp( void ) 
{
	return imu_cfg.gyro.odr_hz;
}

uint16_t lsm6ds3_get_accel_sens( void ) 
{
	static uint16_t sensi = 0;
	static uint16_t fsr = 0;
	
	if( imu_cfg.accel.fsr != 0 ) {
		if( fsr != imu_cfg.accel.fsr  ) {
			//number needs recalculating
			fsr = imu_cfg.accel.fsr;
			sensi = 32768/fsr;
		}
	}
	else {
		sensi = DEF_ACCEL_SENS;
	}
	
	return sensi;
}

float lsm6ds3_get_gyro_sens( void ) 
{
	static float sensi;
	static uint16_t fsr = 0;
	
	if( imu_cfg.gyro.fsr != 0 ) {
		if( fsr != imu_cfg.gyro.fsr  ) {
			//number needs recalculating
			fsr = imu_cfg.gyro.fsr;
			sensi = 32768.0/fsr;
		}
	}
	else {
		sensi = DEF_GYRO_SENS;
	}
	
	return sensi;
}

ret_code_t lsm6ds3_read_accel_reg( uint8_t *data ) 
{
	return lsm6ds3_read(LSM6DS3_OUTX_L_XL, DATA_OUT_XYZ_SIZE, data);
}

ret_code_t lsm6ds3_read_gyro_reg( uint8_t *data ) 
{
	return lsm6ds3_read(LSM6DS3_OUTX_L_G, DATA_OUT_XYZ_SIZE, data);
}

ret_code_t lsm6ds3_read_sens_hub_reg( uint8_t *data ) 
{
	return lsm6ds3_read(LSM6DS3_SENSORHUB1_REG, DATA_OUT_XYZ_SIZE, data);
}

ret_code_t lsm6ds3_get_step_data( uint16_t *steps )
{
	uint16_t data;
	ret_code_t err = NRF_SUCCESS;
	
	err = lsm6ds3_read(LSM6DS3_STEP_COUNTER_L, 2, (uint8_t *)&data);
	if (err == NRF_SUCCESS) {
		*steps = data;
	}
	else {
		*steps = 0;
	}

	return err;
}

ret_code_t lsm6ds3_wake_on_motion( bool en_wom ) 
{
	ret_code_t err = NRF_SUCCESS;
	uint8_t dummy_read;
	
	if( en_wom == LSM6DS3_EN_BIT ) {
		lsm6ds3_disable_all();	//make sure everything that draws power is shutdown
		
		//Accel needs to be on at minimum data rate to run Wake on Motion Detection
		uint16_t temp_odr = LSM6DS3_MIN_OP_ODR_HZ;
		lsm6ds3_set_odr( LSM6DS3_ACCEL, &temp_odr);
		
		//Read the Wake up source to clear it
		lsm6ds3_read(LSM6DS3_WAKE_UP_SRC, 1, &dummy_read);	
		
		//Route Wake Up Interrupt to INT1
		err = lsm6ds3_write_data_with_mask(LSM6DS3_MD1_CFG, MD1_CFG_INT1_WAKE_UP, LSM6DS3_EN_BIT);	
		
		//Indicate that WOM running
		imu_cfg.sensor_en |= MOTION_WAKE_EN_MASK;		

		//app_trace_log(DEBUG_LOW, "Wake On Motion Enabled\r");
	}
	else {
		//turn Off Wake on Motion Int
		err = lsm6ds3_write_data_with_mask(LSM6DS3_MD1_CFG, MD1_CFG_INT1_WAKE_UP, LSM6DS3_DIS_BIT);	
		imu_cfg.sensor_en &= ~MOTION_WAKE_EN_MASK;
		
		//Is Accel supposed to be On or Off?
		if( (imu_cfg.sensor_en&ACCEL_EN_MASK) == 0 ) {
			err = lsm6ds3_disable_sensors( LSM6DS3_ACCEL );
		}
		
		//Force Gyro into normal operating mode
		//err = lsm6ds3_enable_sensors( LSM6DS3_GYRO );
	}
	
	return err;
}

// function should be periodically called to double check that the lsm6ds3 is still in low power wake on motion
// mode. If it were to somehow exit this mode, the application will be waiting forever...or until the battery dies.
ret_code_t lsm6ds3_check_wom( void ) 
{
	ret_code_t err;
	uint8_t read_val[2];
	
	//Reading the master config seems to keep the device from getting permaneantly lost on FIFO overflows
	err = lsm6ds3_read( LSM6DS3_MASTER_CFG, 1, read_val);
	if (err != NRF_SUCCESS) return err;	
	//app_trace_log(DEBUG_LOW, "Master Config: 0x%02X\r", read_val[0] );
	
	err = lsm6ds3_read( LSM6DS3_CTRL1_XL, 2, read_val);		//Read Accel and Gyro Control Registers
	if (err != NRF_SUCCESS) return err;
	if( (read_val[0]&CTRL1_XL_ODR_MASK) != (ODR_13HZ_VAL<<ODR_REG_OFFSET) ) {
		app_trace_log(DEBUG_MED, "Accel ODR: 0x%02X, Not 0x%02X\r", (read_val[0]&CTRL1_XL_ODR_MASK), (ODR_13HZ_VAL<<ODR_REG_OFFSET) );
		return NRF_ERROR_INVALID_STATE;
	}
	if( (read_val[1]&CTRL2_GYRO_ODR_MASK) != (ODR_POWER_OFF_VAL<<ODR_REG_OFFSET) ) {
		app_trace_log(DEBUG_MED, "Gyro ODR: 0x%02X, Not 0\r", (read_val[1]&CTRL2_GYRO_ODR_MASK) );
		return NRF_ERROR_INVALID_STATE;
	}
	
	err = lsm6ds3_read( LSM6DS3_MD1_CFG, 1, read_val);	
	if (err != NRF_SUCCESS) return err;
	if( (read_val[0]&MD1_CFG_INT1_WAKE_UP) != MD1_CFG_INT1_WAKE_UP ) {
		app_trace_log(DEBUG_MED, "WOM not Enabled\r");
		
		imu_cfg.sensor_en &= ~MOTION_WAKE_EN_MASK;
		
		return NRF_ERROR_INVALID_STATE;
	}
	
	return NRF_SUCCESS;
}

bool lsm6ds3_check_wakeup_evt( void )
{
	ret_code_t err;
	uint8_t read_val = 0;

	err = lsm6ds3_read(LSM6DS3_WAKE_UP_SRC, 1, &read_val);
	if( err != NRF_SUCCESS ) {
		//give a retry
		app_trace_log(DEBUG_LOW, "[WAKE_SRC]: retry");
		err = lsm6ds3_read(LSM6DS3_WAKE_UP_SRC, 1, &read_val);
		if( err == NRF_SUCCESS ) {
			app_trace_log(DEBUG_LOW, "\r");
		}
		else {
			app_trace_log(DEBUG_LOW, " failed\r");
			return false;
		}
	}
	
	if( (read_val&WAKE_UP_SRC_WU_IA_MASK) > 0 ) return true;
	
	return false;
}

ret_code_t lsm6ds3_set_irq( T_LSM6DS3_SENSOR_TYPE s_type, bool on_off )
{
	uint8_t reg_addr, mask, data;
	uint8_t dummy_read;
	ret_code_t err = NRF_SUCCESS;

	if( on_off == LSM6DS3_EN_BIT ) data = LSM6DS3_EN_BIT;
	else data = LSM6DS3_DIS_BIT;

	switch (s_type) {
		case LSM6DS3_ACCEL:			
			//uint8_t data[6];
			//lsm6ds3_read_accel_reg( data );

#if defined (LSM6DS3_FIFO_MODE)
			reg_addr = LSM6DS3_INT1_CTRL;
			mask = INT1_CTRL_FIFO_THRS_MASK;
			
			if( (imu_cfg.sensor_en&GYRO_EN_MASK) > 0 ) {
				data = LSM6DS3_EN_BIT;		//IRQ needs to stay on, Gyro is still active
			}
#else
			reg_addr = LSM6DS3_INT1_CTRL_ADDR;
			mask = LSM6DS3_ACCEL_DRDY_IRQ_MASK;
#endif	
			break;
			
		case LSM6DS3_GYRO:

#if defined (LSM6DS3_FIFO_MODE)
			reg_addr = LSM6DS3_INT1_CTRL;
			mask = INT1_CTRL_FIFO_THRS_MASK;
			
			if( (imu_cfg.sensor_en&ACCEL_EN_MASK) > 0 ) {
				data = LSM6DS3_EN_BIT;		//IRQ needs to stay on, Accel is still active
			}
#else
			reg_addr = LSM6DS3_INT1_CTRL_ADDR;
			mask = LSM6DS3_GYRO_DRDY_IRQ_MASK;
#endif
			break;
			
		case LSM6DS3_STEP_COUNTER:			
			//Read Status Register to Clear it Out
			lsm6ds3_read(LSM6DS3_FUNC_SRC1, 1, &dummy_read);
			
			reg_addr = LSM6DS3_INT2_CTRL;
			mask = INT1_CTRL_STEP_DETECT_MASK;		//Step Counter Delta Time Int Enable
			break;

		case LSM6DS3_SIG_MOTION:
			//Read Status Register to Clear it Out
			lsm6ds3_read(LSM6DS3_FUNC_SRC1, 1, &dummy_read);
			
			reg_addr = LSM6DS3_INT1_CTRL;
			mask = INT1_CTRL_SIG_MOTION_MASK;
			break;
			
		case LSM6DS3_STEP_DETECTOR:
			//Read Status Register to Clear it Out
			lsm6ds3_read(LSM6DS3_FUNC_SRC1, 1, &dummy_read);
			
			reg_addr = LSM6DS3_INT1_CTRL;
			mask = INT1_CTRL_STEP_DETECT_MASK;
			break;
		
		case LSM6DS3_TILT:
			//Read Status Register to Clear it Out
			lsm6ds3_read(LSM6DS3_FUNC_SRC1, 1, &dummy_read);
			
			reg_addr = LSM6DS3_MD1_CFG;
			mask = MD1_CFG_INT1_TILT;
			break;
			
		case LSM6DS3_EXTERN_SENS:
			return NRF_SUCCESS;		//no IRQ associated with this sensor. Data is sampled based on Accel Data Ready (and potentially INT source 2...ignored).
			
		case LSM6DS3_MOTION_WAKE:
			return NRF_SUCCESS;		//no IRQ associated with this sensor. Data is sampled based on Accel Data Ready (and potentially INT source 2...ignored).
			
		default:
			return NRF_ERROR_INVALID_PARAM;
	}
	
	//Enable Data Int Source
	err = lsm6ds3_write_data_with_mask(reg_addr, mask, data);
	
	return err;
}

static ret_code_t lsm6ds3_set_fs( T_LSM6DS3_SENSOR_TYPE s_type, int32_t fsr )
{
	ret_code_t err;
	int	i;
	uint8_t read_data;
	const struct lsm6ds3_fs_table *fs_table = &lsm6ds3_fs_table[s_type];

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++) {
		if (fs_table->fs_avl[i].urv >= fsr) {
			break;
		}
	}
	if (i >= LSM6DS3_FS_LIST_NUM) return NRF_ERROR_INVALID_PARAM;
	
	do {
		err = lsm6ds3_write_data_with_mask(fs_table->addr, fs_table->mask, fs_table->fs_avl[i].value);
		err = lsm6ds3_read(fs_table->addr, 1, &read_data);
		if( err != NRF_SUCCESS ) return err;
		if( (read_data&fs_table->mask) != (fs_table->fs_avl[i].value<<2) ) {
			delay_ms( 5 );
		}
		else {
			break;
		}
	} while( 1 );
	
	if( s_type == LSM6DS3_ACCEL ) {
		imu_cfg.accel.fsr = fs_table->fs_avl[i].urv;
	}
	else if( s_type == LSM6DS3_GYRO ) {
		imu_cfg.gyro.fsr = fs_table->fs_avl[i].urv;
	}

	return NRF_SUCCESS;
}

static ret_code_t lsm6ds3_set_odr( T_LSM6DS3_SENSOR_TYPE s_type, uint16_t * odr_hz )
{
	ret_code_t err = NRF_SUCCESS;
	uint i;
	
	if( s_type >= LSM6DS3_SENSORS_NUMB ) return NRF_ERROR_INVALID_PARAM;			//sensor type does not exist
	if( s_type != LSM6DS3_ACCEL && s_type != LSM6DS3_GYRO ) return NRF_ERROR_NOT_SUPPORTED;	//sensor data rate cannot not be modified
	
	//Fit the Requested Data Rate into the appropriate category
	for( i=0; i<ODR_CNT; i++ ) {
		if( *odr_hz <= lsm6ds3_odr_table.odr[i].hz ) {
			break;
		}
	}
	if (i >= ODR_CNT) return NRF_ERROR_INVALID_PARAM;
	
	err = lsm6ds3_write_data_with_mask(lsm6ds3_odr_table.addr[s_type], lsm6ds3_odr_table.mask[s_type], lsm6ds3_odr_table.odr[i].reg_val);
	if (err != NRF_SUCCESS) return err;
	
	//Value Successfully Updated
	if( lsm6ds3_odr_table.odr[i].hz != *odr_hz ) *odr_hz = lsm6ds3_odr_table.odr[i].hz;	//update requested frequency to match implemented frequency

	return NRF_SUCCESS;
}

ret_code_t lsm6ds3_enable_sensors( T_LSM6DS3_SENSOR_TYPE s_type )
{
	ret_code_t err;
	uint16_t temp_odr;
	
	if (s_type >= LSM6DS3_SENSORS_NUMB) return NRF_ERROR_INVALID_PARAM;	//sensor type does not exist

	//if( (imu_cfg.sensor_en&(1<<s_type)) != 0 ) return 0;	//sensor already on

	switch (s_type) {
		case LSM6DS3_ACCEL:
			temp_odr = imu_cfg.accel.odr_hz;
			err = lsm6ds3_set_odr( LSM6DS3_ACCEL, &temp_odr );
			if( temp_odr != imu_cfg.accel.odr_hz ) {
				//different sampling rate assigned
				if (imu_debug) app_trace_log(DEBUG_LOW, "Accel ODR Off: %0u Hz\r", temp_odr);
			}
			if (err != NRF_SUCCESS) return err;
			
			imu_cfg.sensor_en |= ACCEL_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if (err != NRF_SUCCESS) return err;
#endif
			break;
			
		case LSM6DS3_GYRO:
			temp_odr = imu_cfg.gyro.odr_hz;
			err = lsm6ds3_set_odr( LSM6DS3_GYRO, &temp_odr );	
			if( temp_odr != imu_cfg.gyro.odr_hz ) {
				//different sampling rate assigned
				if (imu_debug) app_trace_log(DEBUG_LOW, "Gyro ODR Off: %0u Hz\r", temp_odr);
			}
			if (err != NRF_SUCCESS) return err;
			
			imu_cfg.sensor_en |= GYRO_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if (err != NRF_SUCCESS) return err;
#endif
			break;
			
		case LSM6DS3_SIG_MOTION:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_SIG_MOTION_EN_MASK, LSM6DS3_EN_BIT);
			if (err != NRF_SUCCESS) return err;

			imu_cfg.sensor_en |= SIG_MOTION_EN_MASK;
			imu_cfg.motion_event_ready = true;
			
			err = lsm6ds3_enable_embedded_func( true, ODR_13HZ_VAL );
			if (err != NRF_SUCCESS) return err;
			break;
			
		case LSM6DS3_STEP_COUNTER:
		case LSM6DS3_STEP_DETECTOR:
			err = lsm6ds3_enable_pedometer(true);
			if (err != NRF_SUCCESS) return err;
			
			imu_cfg.sensor_en |= (1<<s_type);
			
			err = lsm6ds3_enable_embedded_func(true, ODR_26HZ_VAL);
			if (err != NRF_SUCCESS) return err;
			break;
			
		case LSM6DS3_TILT:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_TILT_EN_MASK, LSM6DS3_EN_BIT);
			if (err != NRF_SUCCESS) return err;
			
			imu_cfg.sensor_en |= TILT_EN_MASK;
			
			err = lsm6ds3_enable_embedded_func(true, ODR_26HZ_VAL);
			if (err != NRF_SUCCESS) return err;
			break;
			
		case LSM6DS3_EXTERN_SENS:
			//Turn on External Senor Monitoring
			
			imu_cfg.sensor_en |= EXTERN_SENS_EN_MASK;
			break;
		
		case LSM6DS3_MOTION_WAKE:
			//Not handled here			
		default:
			return NRF_ERROR_INVALID_PARAM;
	}

	//Turn on the Interrupt source for this sensor
	err = lsm6ds3_set_irq( s_type, true );
	if (err != NRF_SUCCESS) return err;

	//uint8_t stat_reg;
	//err = lsm6ds3_read( LSM6DS3_STATUS_ADDR, 1, &stat_reg);
	//if (imu_debug) app_trace_log(DEBUG_LOW, "Stat Register: 0x%02X\r", stat_reg);

	return err;
}
	
ret_code_t lsm6ds3_disable_sensors( T_LSM6DS3_SENSOR_TYPE s_type )
{
	ret_code_t err = NRF_SUCCESS;
	uint16_t odr_off_hz = 0;

	if (s_type >= LSM6DS3_SENSORS_NUMB) return NRF_ERROR_INVALID_PARAM;	//sensor type does not exist
	//if( (imu_cfg.sensor_en&(1<<s_type)) == 0 ) return NRF_SUCCESS;	//sensor already off

	switch (s_type) {
		case LSM6DS3_ACCEL:
			if ( (imu_cfg.sensor_en&EMBEDDED_FUNC_EN_MASK) != 0 ) {
				//Accelerometer needs to stay on at a minimum of 26 Hz to keep these functions running
				odr_off_hz = LSM6DS3_MIN_FUNC_ODR_HZ;
			}
			else {
				odr_off_hz = 0;
			}
			
			err = lsm6ds3_set_odr( LSM6DS3_ACCEL, &odr_off_hz );
			if (err != NRF_SUCCESS) return err;
			
			//Sensor in no longer On, clear indication
			imu_cfg.sensor_en &= ~ACCEL_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if( err != NRF_SUCCESS ) return err;
#endif
			break;
			
		case LSM6DS3_GYRO:
			odr_off_hz = 0;
			err = lsm6ds3_set_odr( LSM6DS3_GYRO, &odr_off_hz );
			if (err != NRF_SUCCESS) return err;
			
			//Sensor in no longer On, clear indication
			imu_cfg.sensor_en &= ~GYRO_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if( err != NRF_SUCCESS ) return err;
#endif
			break;
			
		case LSM6DS3_SIG_MOTION:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_SIG_MOTION_EN_MASK, LSM6DS3_DIS_BIT);
			if (err != NRF_SUCCESS) return err;

			imu_cfg.sensor_en &= ~SIG_MOTION_EN_MASK;
			imu_cfg.motion_event_ready = false;

			break;
			
		case LSM6DS3_STEP_COUNTER:
		case LSM6DS3_STEP_DETECTOR:
			err = lsm6ds3_enable_pedometer( false );
			if (err != NRF_SUCCESS) return err;
			
			imu_cfg.sensor_en &= ~(1<<s_type);
			
			break;
			
		case LSM6DS3_TILT:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_TILT_EN_MASK, LSM6DS3_DIS_BIT);
			if (err != NRF_SUCCESS) return err;
			
			imu_cfg.sensor_en &= ~TILT_EN_MASK;

			break;
			
		case LSM6DS3_EXTERN_SENS:
			//Turn off External Sensor Monitoring
			
			imu_cfg.sensor_en &= ~EXTERN_SENS_EN_MASK;
			break;
		
		case LSM6DS3_MOTION_WAKE:

			err = lsm6ds3_wake_on_motion( LSM6DS3_DIS_BIT );	
			break;
		
		default:
			return NRF_ERROR_INVALID_PARAM;
	}

	//Turn Off the Interrupt Source for this Sensor
	err = lsm6ds3_set_irq( s_type, false );
	if( err != NRF_SUCCESS ) return err;
	
	err = lsm6ds3_enable_embedded_func( false, ODR_POWER_OFF_VAL);
	if( err != NRF_SUCCESS ) return err;

	return  err;
}

ret_code_t lsm6ds3_disable_all( void ) 
{
	static ret_code_t prv_err = NRF_SUCCESS;
	ret_code_t err = NRF_SUCCESS;
	T_LSM6DS3_SENSOR_TYPE s_type;
	
	for( s_type=LSM6DS3_ACCEL; s_type<LSM6DS3_SENSORS_NUMB; s_type++ ) {
		if( (imu_cfg.sensor_en&(0x0001<<s_type)) > 0 ) {
			err = lsm6ds3_disable_sensors( s_type );
			if( err != NRF_SUCCESS ) {
				//shutoff failed, retry once
				err = lsm6ds3_disable_sensors( s_type );
				if( err != NRF_SUCCESS ) {
					//failed again, issue Error
					if( prv_err != err) app_trace_log(DEBUG_MED, "Stop IMU Sensor %01u Failed: %02i\r", s_type, err);
				}
			}
			prv_err = err;
		}
	}
	
	return err;
}

//Following functions are Related to the IMU Special Features
ret_code_t lsm6ds3_irq_management( void )
{
	uint8_t src_value = 0x00;
	ret_code_t err;

	err = lsm6ds3_read(LSM6DS3_FUNC_SRC1, 1, &src_value);
	if( err != NRF_SUCCESS ) return err;

	if (src_value & FUNC_SRC1_STEP_DELTA_IA_MASK) {
		uint16_t steps_c;
		err = lsm6ds3_get_step_data(&steps_c);
		if (err != NRF_SUCCESS) {
			//dev_err(cdata->dev, "error while reading step counter data\n");
			//enable_irq(cdata->irq);

			return err;
		}

		//lsm6ds3_report_single_event(&cdata->sensors[LSM6DS3_STEP_COUNTER], steps_c, cdata->sensors[LSM6DS3_STEP_COUNTER].timestamp);
	}

	if (src_value & FUNC_SRC1_STEP_DETECTED_MASK) {
		//sdata = &cdata->sensors[LSM6DS3_STEP_DETECTOR];
		//sdata->timestamp = cdata->timestamp;
		//lsm6ds3_report_single_event(sdata, 1, sdata->timestamp);

		if (imu_cfg.motion_event_ready) {
			//sdata = &cdata->sensors[LSM6DS3_SIGN_MOTION];
			//sdata->timestamp = cdata->timestamp;
			//lsm6ds3_report_single_event(sdata, 1, sdata->timestamp);
			imu_cfg.motion_event_ready = false;
			err = lsm6ds3_disable_sensors(LSM6DS3_STEP_DETECTOR);
		}
	}

	if (src_value & FUNC_SRC1_TILT_IA_MASK) {
		//sdata = &cdata->sensors[LSM6DS3_TILT];
		//sdata->timestamp = cdata->timestamp;
		//lsm6ds3_report_single_event(sdata, 1, sdata->timestamp);
	}
	
	if( err != NRF_SUCCESS ) return err;
	
	//enable_irq(cdata->irq);
	return err;
}

static ret_code_t lsm6ds3_enable_embedded_func( bool enable, T_SENSOR_ODR_CODE odr )
{
	static bool em_func_en = false;
	ret_code_t err = NRF_SUCCESS;

	if (enable) {
		if( em_func_en == false ) {
			err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_FUNC_EN_MASK, LSM6DS3_EN_BIT);
			if (err != NRF_SUCCESS) return err;
			em_func_en = true;
		}
	}
	else if ( (imu_cfg.sensor_en&EMBEDDED_FUNC_EN_MASK) == 0 ) {
		if( em_func_en == true ) {
			err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_FUNC_EN_MASK, LSM6DS3_DIS_BIT);
			if (err != NRF_SUCCESS) return err;
			em_func_en = false;
		}
	}

	//Accelerometer needs to be On for the embedded functions to operate.  
	if( (imu_cfg.sensor_en&ACCEL_EN_MASK) == 0 ) {
		//No other functions are using the acclerometer, it can be set to the rate requested
		err = lsm6ds3_write_data_with_mask(lsm6ds3_odr_table.addr[LSM6DS3_ACCEL], lsm6ds3_odr_table.mask[LSM6DS3_ACCEL], odr);
		if (err != NRF_SUCCESS) return err;
	}

	return err;
}

//static ret_code lsm6ds3_configure_pedo( void )
//{
//	ret_code err = NRF_SUCCESS;
//	
////	//Enable access to the embedded function Registers
////	lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS, LSM6DS3_FUNC_CFG_EN_MASK, LSM6DS3_EN_BIT );

////	//Configure Pedometer:
////	temp_reg_val = 0;
////	lsm6ds3_write_data_with_mask( LSM6DS3_PEDO_DEB_REG, PEDO_DEB_TIME_MASK, &temp_reg_val );

////	//Disable access to the embedded function Registers
////	do{
////		// Need to turn Embedded Register Access Off. If this remains on and we right the wrong register, 
////		// it could cause permanent damage to the device as per page 82 of datasheet.
////		lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS, LSM6DS3_FUNC_CFG_EN_MASK, LSM6DS3_DIS_BIT );
////		err = lsm6ds3_read( LSM6DS3_FUNC_CFG_ACCESS, 1, &temp_reg_val );
////		//if( err != NRF_SUCCESS ) return err;	//Don't proceed if we haven't verified that Access is Off
////	}
////	while( (temp_reg_val&LSM6DS3_FUNC_CFG_EN_MASK) != 0 );
//	
//	//Reset Steps Register
//	err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_PEDO_RESET_MASK, LSM6DS3_EN_BIT);
//	if (err != NRF_SUCCESS) return err;
//	
//	return err;
//}

static ret_code_t lsm6ds3_enable_pedometer( bool enable )
{
	ret_code_t err = NRF_SUCCESS;
	uint8_t value;

	if (enable)	value = LSM6DS3_EN_BIT;
	else value = LSM6DS3_DIS_BIT;
	
#if defined(LSM6DS3_FIFO_MODE)
	err = lsm6ds3_write_data_with_mask(LSM6DS3_FIFO_CTRL2, FIFO_CTRL2_PEDO_EN_MASK, value);
	if (err != NRF_SUCCESS) return err;
#endif

	err = lsm6ds3_write_data_with_mask(LSM6DS3_CTRL10_C, CTRL10_PEDO_EN_MASK, value);
	if (err != NRF_SUCCESS) return err;
	
	return err;
}

ret_code_t lsm6ds3_fifo_management( void )
{
#if defined(LSM6DS3_FIFO_MODE)
	uint8_t src_fifo = 0x00;
	ret_code_t err;
	uint8_t attempts = 0;
	
	//*data_avl = false;
	
	do {
		err = lsm6ds3_read(LSM6DS3_FIFO_STATUS2, 1, &src_fifo);
		if( err != NRF_SUCCESS ) return err;

		if (src_fifo & FIFO_STATUS2_WATER_M_MASK) {
			//Amount of Data in FIFO is between Watermark and full
			if ( src_fifo & FIFO_STATUS2_OVR_MASK ) {
				lsm6ds3_set_fifo_mode(FIFO_BYPASS);		//Setting to BYPASS flushes buffer
				delay_us(30);							//After setting BYPASS, have to wait at least 30us before a new mode
				err = lsm6ds3_set_fifo_mode(FIFO_CONTINUOUS);
				app_trace_log(DEBUG_MED, "Data FIFO overrun!\r");
			}
			else {
				err = lsm6ds3_read_fifo(true);
				//*data_avl = true;
			}
			if( err != NRF_SUCCESS ) return err;
		}
	}
	while( (src_fifo & FIFO_STATUS2_WATER_M_MASK) && (attempts++ < 4) );
#endif

	return err;
}

static ret_code_t lsm6ds3_read_fifo( bool check_fifo_len )
{
	ret_code_t err = NRF_SUCCESS;
	Union32 fifo_stat;
	uint16_t fifo_len = 0;
	uint16_t next_data_set, temp_len;
	volatile uint16_t read_len;
	uint8_t fifo_read[FIFO_MAX_READ];

	if( check_fifo_len ) {
		err = lsm6ds3_read( LSM6DS3_FIFO_STATUS1, 4, &fifo_stat.u8[0] );	//Read the 4 FIFO Status Registers
		if( err != NRF_SUCCESS ) {
			app_trace_log(DEBUG_MED, "FIFO - Status Read Error\r");
			return err;
		}
			
		if( fifo_stat.u8[1]&FIFO_STATUS2_OVR_MASK ) {
			app_trace_log(DEBUG_MED, "FIFO - Overrun\r");
		}

		//The next axis of sensor data is specified in Status Registers 3&4 (bits 9:0)
		next_data_set = fifo_stat.u16[1]&LSM6DS3_FIFO_PATTERN_MASK;
		next_data_set /= LSM6DS3_FIFO_SAMPLES_PER_DATA_SET;
		
		//The FIFO length is contained in the Status Registers 1&2  (bits 11:0)		
		fifo_len = fifo_stat.u16[0];
		fifo_len &= LSM6DS3_FIFO_LENGTH_MASK;
		fifo_len *= LSM6DS3_FIFO_BYTES_PER_SAMPLE;
		if( FIFO_DEBUG ) app_trace_log(DEBUG_LOW, "FIFO Byte Count: %02u DataSet: %02u\r", fifo_len, next_data_set);
		
		if (fifo_len == 0)	{
			//Fifo Empty
			return NRF_SUCCESS;
		}
		else if( fifo_len > imu_cfg.fifo.buf_byte_thres ) {
			//Fifo larger than this 1 read call. Limit the data read.
			read_len = imu_cfg.fifo.buf_byte_thres;
		}
		else {
			read_len = fifo_len;
		}
	}
	else {
		read_len = imu_cfg.fifo.buf_byte_thres;
		next_data_set = 0;
	}
	
	// make sure the read length does not exceed the size of the holding array
	if( read_len > FIFO_MAX_READ ) read_len = FIFO_MAX_READ;
	
	temp_len = 0;
	while( temp_len < read_len ) {		
		if( (read_len-temp_len) < LSM6DS3_RX_TX_MAX_LENGTH) {
			err = lsm6ds3_read( LSM6DS3_FIFO_DATA_OUT_L, (read_len-temp_len), &fifo_read[temp_len] );
		}
		else {
			err = lsm6ds3_read( LSM6DS3_FIFO_DATA_OUT_L, LSM6DS3_RX_TX_MAX_LENGTH, &fifo_read[temp_len] );
		}
		
		if (err != NRF_SUCCESS) {
			app_trace_log(DEBUG_LOW, "FIFO - Buffer Read Error\r");
			return err;
		}
		
		temp_len += LSM6DS3_RX_TX_MAX_LENGTH;
	}

	lsm6ds3_parse_fifo_data( fifo_read, read_len, next_data_set );
	
	return err;
}

static void lsm6ds3_parse_fifo_data( uint8_t * data_buffer, uint16_t read_len, uint16_t first_data_set )
{
	uint16_t fifo_offset = 0;
	uint16_t data_set_offset = first_data_set;
	
	if( FIFO_DEBUG ) app_trace_puts(DEBUG_LOW, "FIFO Parse:");
	
	while( (fifo_offset+LSM6DS3_FIFO_BYTES_PER_DATA_SET) <= read_len) {
		
		switch ( fifo_pattern.list[data_set_offset] ) {
			case FIFO_GYRO:
				if( FIFO_DEBUG ) app_trace_puts(DEBUG_LOW, " G");
				save_gyro_data( &data_buffer[fifo_offset] );
				break;
				
			case FIFO_ACCEL:
				if( FIFO_DEBUG ) app_trace_puts(DEBUG_LOW, " A");
				save_accel_data( &data_buffer[fifo_offset] );
				break;
				
			case FIFO_EXTERN:
				if( FIFO_DEBUG ) app_trace_puts(DEBUG_LOW, " E");
				//save_extern_data( &data_buffer[fifo_offset] );
				break;
				
			case FIFO_STEPS:
				if( FIFO_DEBUG ) app_trace_puts(DEBUG_LOW, " S");
				//save_steps_data( &data_buffer[fifo_offset] );
				break;
				
			default:
				break;
		}
		
		fifo_offset += LSM6DS3_FIFO_BYTES_PER_DATA_SET;
		if( ++data_set_offset >= imu_cfg.fifo.data_set_thres ) {
			data_set_offset = 0;
		}
	}
	
	if( FIFO_DEBUG ) app_trace_puts(DEBUG_LOW, "\r");

	return;
}

static ret_code_t lsm6ds3_set_fifo_mode( T_FIFO_MODES fm )
{
	ret_code_t err;

	//Make sure that a possible mode was requested
	switch (fm) {
		case FIFO_BYPASS:
		case FIFO_FIFO:
		case FIFO_CONTIN_FIFO:
		case FIFO_BYPASS_CONTIN:
		case FIFO_CONTINUOUS:
			//Valid selection, proceed
			break;
			
		default:
			//Selection not supported, return
			return NRF_ERROR_INVALID_PARAM;
			//break;
	}

	err = lsm6ds3_write_data_with_mask(LSM6DS3_FIFO_CTRL5, FIFO_CTRL5_MODE_MASK, fm);
	if (err != NRF_SUCCESS) return err;
	
	return err;
}

/// The order for setting up the FIFO:
/// 1. Choose the decimation factor for each sensor through the decimation bits in the FIFO_CTRL3 and FIFO_CTRL4 registers;
/// 2. Choose the FIFO ODR through the ODR_FIFO_[3:0] bits in the FIFO_CTRL5 register;
/// 3. Set the FIFO_MODE_[2:0] bits in the FIFO_CTRL5 register to 110b to enable FIFO Continuous mode.
static ret_code_t lsm6ds3_reconfigure_fifo( void )
{
	static bool fifo = OFF;
	ret_code_t err = NRF_SUCCESS;
	uint16_t fifo_len = 0;
	
	lsm6ds3_read_fifo(true);

	//Turn FIFO Off, Flushes Buffer, and allows changes to be made
	err = lsm6ds3_set_fifo_mode(FIFO_BYPASS);
	if( err == NRF_SUCCESS ) {
		
		err = lsm6ds3_set_fifo_decimators_and_threshold( &fifo_len );
		if( err != NRF_SUCCESS ) return err;
			
		if (fifo_len > 0) {
			int i;
			
			//After setting BYPASS, have to wait at least 30us before a new mode
			delay_us(30);
			
			//Set the FIFO Update Rate at or above the maximum sensor rate
			for( i=0; i<ODR_CNT; i++ ) {
				if( imu_cfg.fifo.odr_hz <= lsm6ds3_odr_table.odr[i].hz ) {
					//i is now the appropriate offset to determine the Register value to write. Stop incrementing it.
					break;
				}
			}
			err = lsm6ds3_write_data_with_mask(LSM6DS3_FIFO_CTRL5, FIFO_CTRL5_ODR_MASK, lsm6ds3_odr_table.odr[i].reg_val);
			if( err != NRF_SUCCESS ) return err;
			
			imu_cfg.fifo.odr_hz = lsm6ds3_odr_table.odr[i].hz;
			//app_trace_log(DEBUG_LOW, "FIFO Rate: %01u\r", imu_cfg.fifo.odr_hz );
			
			//Turn FIFO On
			err = lsm6ds3_set_fifo_mode(FIFO_CONTINUOUS);
			if( err != NRF_SUCCESS ) return err;
			
			if( fifo == OFF ) {
				fifo = ON;
				if( FIFO_DEBUG ) app_trace_log(DEBUG_LOW, "FIFO On\r");
			}
		}
		else {
			//Set FIFO Update Rate to Off
			err = lsm6ds3_write_data_with_mask(LSM6DS3_FIFO_CTRL5, FIFO_CTRL5_ODR_MASK, ODR_POWER_OFF_VAL);
			if( err != NRF_SUCCESS ) return err;
			
			imu_cfg.fifo.odr_hz = 0;
			//app_trace_log(DEBUG_LOW, "FIFO Rate: %01u\r", imu_cfg.fifo.odr_hz );
			
			if( fifo == ON ) {
				fifo = OFF;
				if( FIFO_DEBUG ) app_trace_log(DEBUG_LOW, "FIFO Off\r");
			}
		}
	}

	return err;
}
  
static ret_code_t lsm6ds3_set_fifo_decimators_and_threshold( uint16_t * fifo_len )
{
	ret_code_t err = NRF_SUCCESS;
	volatile uint8_t decimator = 0;
	volatile uint16_t min_odr_hz = LSM6DS3_MAX_ODR_HZ, max_odr_hz = 0;
	volatile uint16_t pattern_len = 0;
	volatile uint16_t fifo_byte_len = 0, fifo_threshold;
	volatile uint16_t pattern_iterations;
	volatile uint16_t min_pattern_iter = LSM6DS3_FIFO_SIZE / LSM6DS3_FIFO_BYTES_PER_DATA_SET;
	
	//Clear return length
	*fifo_len = 0;
	
	//Determine the min and max data rates of the data that can be stored in the FIFO
	if( (imu_cfg.sensor_en&ACCEL_EN_MASK) != 0 ) {
		if( imu_cfg.accel.odr_hz == 0 ) {
			//can't be done
			imu_cfg.accel.odr_hz = 1;	//minimum rate
		}
		else if( imu_cfg.accel.odr_hz > LSM6DS3_MAX_ODR_HZ ) {
			imu_cfg.accel.odr_hz = LSM6DS3_MAX_ODR_HZ;	//maximum rate
		}
		min_odr_hz = MIN(min_odr_hz, imu_cfg.accel.odr_hz);
		max_odr_hz = MAX(max_odr_hz, imu_cfg.accel.odr_hz);
	}
	if( (imu_cfg.sensor_en&GYRO_EN_MASK) != 0 ) {
		if( imu_cfg.gyro.odr_hz == 0 ) {
			//can't be done
			imu_cfg.gyro.odr_hz = 1;	//minimum rate
		}
		else if( imu_cfg.gyro.odr_hz > LSM6DS3_MAX_ODR_HZ ) {
			imu_cfg.gyro.odr_hz = LSM6DS3_MAX_ODR_HZ;	//maximum rate
		}
		min_odr_hz = MIN(min_odr_hz, imu_cfg.gyro.odr_hz);
		max_odr_hz = MAX(max_odr_hz, imu_cfg.gyro.odr_hz);
	}
	//if( (imu_cfg.sensor_en&FIFO_SET_3) != 0 ) {
	//	if( set3_rate == 0 ) {
	//		//can't be done
	//	}
	//	min_odr_hz = MIN(min_odr_hz, set3_rate);
	//	max_odr_hz = MAX(max_odr_hz, set3_rate);
	//}
	//if( (imu_cfg.sensor_en&STEP_COUNTER_EN_MASK) != 0 ) {
	//	if( step_rate == 0 ) {
	//		//can't be done
	//	}
	//	min_odr_hz = MIN(min_odr_hz, step_rate);
	//}

	if( (imu_cfg.sensor_en&ACCEL_EN_MASK) ) {
		fifo_pattern.samps_pat[FIFO_ACCEL] = ( imu_cfg.accel.odr_hz/min_odr_hz );	//result will be 1 or greater
		if( fifo_pattern.samps_pat[FIFO_ACCEL] == 0 ){
			if( imu_debug ) app_trace_puts(DEBUG_MED, "FIFO: Not Possible\r");
			fifo_pattern.samps_pat[FIFO_ACCEL] = 1;
		}
		pattern_len += fifo_pattern.samps_pat[FIFO_ACCEL];
		pattern_iterations = MAX( (imu_cfg.accel.odr_hz/4)/fifo_pattern.samps_pat[FIFO_ACCEL], 1 );	//Set threshold for the number of readings in 0.25 seconds
		min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );
		//imu_cfg.accel.deltatime_ns = ( 1000000000ULL/imu_cfg.accel.odr_hz );
		decimator = MAX( max_odr_hz/imu_cfg.accel.odr_hz, 1 );
	} 
	else {
		fifo_pattern.samps_pat[FIFO_ACCEL] = 0;
		decimator = 0;
	}
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_CTRL3, FIFO_CTRL3_DEC_ACCEL_MASK, decimator );
	if (err != NRF_SUCCESS) return err;

	if( (imu_cfg.sensor_en&GYRO_EN_MASK) ) {
		fifo_pattern.samps_pat[FIFO_GYRO] = ( imu_cfg.gyro.odr_hz/min_odr_hz );
		if( fifo_pattern.samps_pat[FIFO_GYRO] == 0 ){
			if( imu_debug ) app_trace_puts(DEBUG_MED, "FIFO: Not Possible\r");
			fifo_pattern.samps_pat[FIFO_GYRO] = 1;
		}
		pattern_len += fifo_pattern.samps_pat[FIFO_GYRO];
		pattern_iterations = MAX( (imu_cfg.gyro.odr_hz/4)/fifo_pattern.samps_pat[FIFO_GYRO], 1 );	//Set threshold for the number of readings in 0.25 seconds
		min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );	
		//imu_cfg.gyro.deltatime_ns = ( 1000000000ULL/imu_cfg.gyro.odr_hz );
		decimator = MAX( max_odr_hz/imu_cfg.gyro.odr_hz, 1 );
	} 
	else {
		fifo_pattern.samps_pat[FIFO_GYRO] = 0;
		decimator = 0;
	}
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_CTRL3, FIFO_CTRL3_DEC_GYRO_MASK, decimator );
	if (err != NRF_SUCCESS) return err;
	
	//if( fifo_set_3 ) {
		//imu_cfg.fifo.samps_in_pattern[FIFO_EXTERN] = ( set3_odr/min_odr_hz );
		//pattern_len += imu_cfg.fifo.samps_in_pattern[FIFO_EXTERN];
		//pattern_iterations = MAX( set3_fifo_length/imu_cfg.fifo.samps_in_pattern[FIFO_EXTERN], 1 );
		//min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );
		////deltatime.set3 = ( 1000000000ULL/set3_odr );
		//decimator = MAX( max_odr_hz/set3_odr, 1 );
	//} 
	//else {
		//fifo_pattern.samps_pat[FIFO_EXTERN] = 0;
		//decimator = 0;
	//}
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_3_4_DECIMATOR_ADDR, LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK, decimator );
	//if (err != 0) return err;

	//if( (imu_cfg.sensor_en&STEP_COUNTER_EN_MASK) ) {
		//imu_cfg.fifo.samps_in_pattern[FIFO_STEPS] = ( step_odr/min_odr_hz );
		//pattern_len += imu_cfg.fifo.samps_in_pattern[FIFO_STEPS];
		//pattern_iterations = MAX( sdata_step_c->fifo_length/imu_cfg.fifo.samps_in_pattern[FIFO_STEPS], 1 );
		//min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );
		////sdata_step_c->deltatime = ( 1000000000ULL/step_odr );
		//decimator = MAX( max_odr_hz/step_odr, 1 );
	//} else {
		//fifo_pattern.samps_pat[FIFO_STEPS] = 0;
		//decimator = 0; 
	//}
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_3_4_DECIMATOR_ADDR, LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK, decimator );
	//if (err != 0) return err;
	
	if( pattern_len > FIFO_MAX_PATTERN ) {
		//Not enough memory reserved for FIFO pattern
		if( imu_debug ) app_trace_puts(DEBUG_MED, "FIFO Pattern Too Long\r");
		return NRF_ERROR_NO_MEM;
	}
	
	fifo_threshold = pattern_len*min_pattern_iter*LSM6DS3_FIFO_SAMPLES_PER_DATA_SET;
	fifo_byte_len = fifo_threshold*LSM6DS3_FIFO_BYTES_PER_SAMPLE;

	if (fifo_byte_len > 0) {
		err = lsm6ds3_write( LSM6DS3_FIFO_CTRL1, 1, (uint8_t *) &fifo_threshold );
		if (err != NRF_SUCCESS) return err;

		err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_CTRL2, FIFO_CTRL2_THRS_H_MASK,*((uint8_t *)&fifo_threshold+1) );
		if (err != NRF_SUCCESS) return err;
		
		int pat_id = 0, iteration = 0;
		while( pat_id < pattern_len ) {
			int data_set;
			//create a list of the data set pattern so the fifo data can easily be parsed
			for( data_set=0; data_set<FIFO_DATA_MAX_SRC_CNT; data_set++ ) {
				if( ((int)fifo_pattern.samps_pat[data_set] - iteration) > 0 ) {
					fifo_pattern.list[pat_id] = data_set;
					pat_id++;
				}
			}
			iteration++;
		}
		
		if( FIFO_DEBUG ) {
			app_trace_log(DEBUG_LOW, "FIFO Pattern: ");
			for( pat_id=0 ; pat_id<(pattern_len-1); pat_id++ ) {
				app_trace_log(DEBUG_LOW, "%02u, ", fifo_pattern.list[pat_id]);
			}
			app_trace_log(DEBUG_LOW, "%02u\r", fifo_pattern.list[pat_id]);
		}
	}

	imu_cfg.fifo.data_set_thres = pattern_len;
	imu_cfg.fifo.buf_byte_thres = fifo_byte_len;	//number of bytes when Interrupt occurs
	imu_cfg.fifo.odr_hz = max_odr_hz;
	*fifo_len = fifo_byte_len;
	
	//if(imu_debug) app_trace_log(DEBUG_LOW, "FIFO Thr: %02u, Acc Pat: %02u, Gyr Pat: %02u\r", imu_cfg.fifo.buf_byte_thres, imu_cfg.fifo.samps_in_pattern[FIFO_ACCEL], imu_cfg.fifo.samps_in_pattern[FIFO_GYRO]);
	
	return NRF_SUCCESS;
}

bool write_external_sensor( uint8_t id, uint8_t reg_addr, uint8_t data )
{
	ret_code_t err;
	uint8_t write_val, read_val = 0;
	
	//todo: Embedded functions may need disabling before accessing
	
	// Step 1: Enable access to the embedded function Registers
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS, FUNC_CFG_EN_MASK, LSM6DS3_EN_BIT );
	if( err != NRF_SUCCESS ) return false;

	// Step 2: Write 0x38 into SLV0_ADD (LIS3MDL slave address = 0011100b (if SDO=0). Enable write operation (rw_0=0))
	write_val = id;
	lsm6ds3_write( LSM6DS3_SLV0_ADD, 1, &write_val );
	
	// Step 3: Write 0x22 into SLV0_SUBADD (0x22 is the LIS3MDL register to be written)
	write_val = reg_addr;
	lsm6ds3_write( LSM6DS3_SLV0_SUBADD, 1, &write_val );
	
	// Step 4: Write 0x00 into DATAWRITE_SRC_MODE_SUB_SLV0 (0x00 is the value to be written in register 0x22 of LIS3MDL
	//			to configure it in continuous conversion mode)
	write_val = data;
	lsm6ds3_write( LSM6DS3_D_WR_MODE_SUB_SLV0, 1, &write_val );

	// Step 5: Disable access to the embedded function Registers
	do{
		lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS, FUNC_CFG_EN_MASK, LSM6DS3_DIS_BIT );
		err = lsm6ds3_read( LSM6DS3_FUNC_CFG_ACCESS, 1, &read_val );
		if( err != NRF_SUCCESS ) return false;
	}
	while( (read_val&FUNC_CFG_EN_MASK) != 0 );
	
	// Step 6: Enable Embedded functions
	lsm6ds3_write_data_with_mask( LSM6DS3_CTRL10_C, CTRL10_FUNC_EN_MASK, LSM6DS3_EN_BIT );
	
	// Step 7: Write 0x09 into MASTER_CONFIG (Enable internal pull-up on SDx/SCx lines, Sensor hub trigger signal is
	//			XL Data Ready, Enable auxiliary I2C master)
	lsm6ds3_write_data_with_mask( LSM6DS3_MASTER_CFG, MASTER_CFG_I2C_PULLUP_MASK|MASTER_CFG_I2C_MASTER_MASK, 0x09 );
	
	// Step 8: Write 0x50 into CTRL1_XL	( Turn-on the accelerometer (for trigger signal))
	lsm6ds3_write_data_with_mask( LSM6DS3_CTRL1_XL, CTRL1_XL_ODR_MASK, 0x05 );
	
	//Step 9: wait for message to send out on i2c lines
	delay_ms(10);
	
	//todo: make sure new values have been written?
	//todo: Slave 0 registers may need to be restored
	
	// Step 10: Disable auxiliary I2C master (todo: May need to stay on)
	lsm6ds3_write_data_with_mask( LSM6DS3_MASTER_CFG, MASTER_CFG_I2C_MASTER_MASK, LSM6DS3_DIS_BIT );
	
	// Step 11: Return Embedded Function to whatever it was before this call
	lsm6ds3_enable_embedded_func( false, ODR_POWER_OFF_VAL );
	
	// Step 12: Return Accel to whatever it was before this call
	uint16_t temp_odr = imu_cfg.accel.odr_hz;
	err = lsm6ds3_set_odr( LSM6DS3_ACCEL, &temp_odr );
	if( err != NRF_SUCCESS ) return false;
	
	return true;
}

#if defined( LIS3MDL )
//If the LIS3MDL Magnetometer is connected, then it can be initialized by turning on the i2c Pass Through 
//mode. Without Pass Through, the LIS3MDL control registers must be written in a trickier way using the 
//Slave 0 registers of the LSM6DS3... Pass through is much cleaner.
bool lsm6ds3_init_lis3mdl( bool load_calibration )
{
	ret_code_t err;
	uint8_t write_val, read_val = 0;
	
#if defined( LSM6DS3_I2C_INTERFACE )
	//Turn Pull Up to Keep Lines High from here on
//	lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PULLUP_MASK, LSM6DS3_EN_BIT );
//	nrf_delay_ms(5);
	
	//Enable i2c Pass Through to initialize the external magnetometer sensor
	err = lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PASSTHR_MASK, LSM6DS3_EN_BIT );
	if( err != NRF_SUCCESS ) return false;
	
	err = lis3mdl_init();
	if( err != NRF_SUCCESS ) {
		lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PASSTHR_MASK, LSM6DS3_DIS_BIT );
		return false;
	}

	err = lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PASSTHR_MASK, LSM6DS3_DIS_BIT );
	if( err != NRF_SUCCESS ) return false;
#else
	//Can't turn on Pass Through, have to write the Control Registers indirectly through the Slave 0 Registers
	err = write_external_sensor( (LIS3MDL_I2C_ADDRESS<<1), LIS3MDL_CTRL_REG3, 0x03 );	//Put LIS3MDL into Power Down Mode
	if( err != NRF_SUCCESS ) return false;
#endif
	
	
	
	
	//Don't turn on embedded functions at this time
	return true;
	
	
	
		
	// Enable access to the embedded function Registers
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS_ADDR, LSM6DS3_FUNC_CFG_REG2_MASK, LSM6DS3_EN_BIT );
	if( err != NRF_SUCCESS ) return false;
	
	// Write 0x39 into SLV0_ADD (LIS3MDL slave address = 0011100b (if SDO=0). Enable read operation (rw_0=1))
	write_val = (LIS3MDL_I2C_ADDRESS<<1)|0x01;
	lsm6ds3_write( LSM6DS3_SLAVE_0_ADDR, 1, &write_val );
	
	// Write 0x28 into SLV0_SUBADD (0x28 is the first LIS3MDL output register to be read)
	write_val = LIS3MDL_OUT_X_L;
	lsm6ds3_write( LSM6DS3_SLAVE_0_SUBADDR, 1, &write_val );

	// Write 0x06 into SLAVE0_CONFIG (No decimation. 1 external sensor connected. Number of registers to read = 6)
	write_val = 0x06;
	lsm6ds3_write( LSM6DS3_SLAVE_0_CONFIG, 1, &write_val );
	
	// Load Magnetic Corrections if Requested
	if( load_calibration ) {
		//Write 0xF7 into MAG_OFFX_H // X offset value initialization
		//Write 0x08 into MAG_OFFX_L // X offset value initialization
		//Write 0x03 into MAG_OFFY_H // Y offset value initialization
		//Write 0x61 into MAG_OFFY_L // Y offset value initialization
		//Write 0xFC into MAG_OFFZ_H // Z offset value initialization
		//Write 0xEF into MAG_OFFZ_L // Z offset value initialization
		//Write 0x0A into MAG_SI_XX // XX soft-iron element
		//Write 0x01 into MAG_SI_XY // XY soft-iron element
		//Write 0x00 into MAG_SI_XZ // XZ soft-iron element
		//Write 0x01 into MAG_SI_YX // YX soft-iron element
		//Write 0x08 into MAG_SI_YY // YY soft-iron element
		//Write 0x81 into MAG_SI_YZ // YZ soft-iron element
		//Write 0x00 into MAG_SI_ZX // ZX soft-iron element
		//Write 0x81 into MAG_SI_ZY // ZY soft-iron element
		//Write 0x0A into MAG_SI_ZZ // ZZ soft-iron element
	}

	// Disable access to the embedded function Registers
	do{
		lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS_ADDR, LSM6DS3_FUNC_CFG_REG2_MASK, LSM6DS3_DIS_BIT );
		err = lsm6ds3_read( LSM6DS3_FUNC_CFG_ACCESS_ADDR, 1, &read_val );
		if( err != NRF_SUCCESS ) return false;
	}
	while( (read_val&LSM6DS3_FUNC_CFG_REG2_MASK) != 0 );
	
	//Have to wait for Embedded Function Register Access to be disabled before attempting to write Regular Registers
	if( load_calibration ) {
		//lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_IRON_CORRECT_MASK, LSM6DS3_EN_BIT );	//Enable Hard-iron correction
		//lsm6ds3_write_data_with_mask( LSM6DS3_CTRL9_XL_ADDR, 0x01, LSM6DS3_EN_BIT );	// Enable Soft-iron correction
	}
	
	// Enable Embedded functions
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_EN_ADDR, LSM6DS3_FUNC_EN_MASK, LSM6DS3_EN_BIT );
	if( err != NRF_SUCCESS ) return false;
	
	err = lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_MASTER_MASK, LSM6DS3_EN_BIT );
	if( err != NRF_SUCCESS ) return false;
	
	return true;
}
#endif
