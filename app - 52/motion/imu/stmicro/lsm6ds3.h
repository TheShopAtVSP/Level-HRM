/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 *
 * Ported to C: 6/29/2015 2:25:07 PM
 */

#ifndef LSM6DS3_H_
#define LSM6DS3_H_

#include "../hal_imu.h"
#include "lis3mdl.h"
#include "sdk_errors.h"

//Determine which communication interface to use, can't use both...
#define LSM6DS3_I2C_INTERFACE 
//#define LSM6DS3_SPI_INTERFACE

#if defined LEVEL_1_0
	#define LSM6DS3_I2C_ADDR				0x6A
	#define LSM6DS3_DEVICE_ID				0x69
#else
	//Address needed to Change due to Battery Charger IC having same ID
	#define LSM6DS3_I2C_ADDR				0x6B	
	#define LSM6DS3_DEVICE_ID				0x6A
#endif

/************** Device Register Defines  *******************/
#define LSM6DS3_TEST_PAGE					0x00
#define LSM6DS3_FUNC_CFG_ACCESS				0x01
	#define FUNC_CFG_EN_MASK				0x80
	#define FUNC_CFG_EN_B_MASK				0x20	
#define LSM6DS3_SENSOR_SYNC_TIME			0x04
	#define SENSOR_SYNC_TIME_MASK			0x0F
#define LSM6DS3_SENSOR_SYNC_EN				0x05
	#define SENSOR_SYNC_EN_MASK				0x03
#define LSM6DS3_FIFO_CTRL1					0x06
	#define IFO_CTRL1_THRS_L_MASK			0xFF
#define LSM6DS3_FIFO_CTRL2					0x07
	#define FIFO_CTRL2_PEDO_EN_MASK			0x80
	#define FIFO_CTRL2_PEDO_RDY_MASK		0x40
	#define FIFO_CTRL2_TEMP_EN_MASK			0x08
	#define FIFO_CTRL2_THRS_H_MASK			0x07
#define LSM6DS3_FIFO_CTRL3					0x08	
	#define FIFO_CTRL3_DEC_GYRO_MASK		0x38
	#define FIFO_CTRL3_DEC_ACCEL_MASK		0x07
#define LSM6DS3_FIFO_CTRL4					0x09
	#define FIFO_CTRL4_STOP_ON_FTH_MASK		0x80
	#define FIFO_CTRL4_ONLY_HIGH_BYTE_MASK	0x40
	#define FIFO_CTRL4_DEC_DS4_MASK			0x38
	#define FIFO_CTRL4_DEC_DS3_MASK			0x07
#define LSM6DS3_FIFO_CTRL5					0x0A
	#define FIFO_CTRL5_ODR_MASK				0x78
	#define FIFO_CTRL5_MODE_MASK			0x07	
#define LSM6DS3_DRDY_PULSE_CFG_G			0x0B
	#define DRDY_PULSE_CFG_G_MASK			0x80
	#define DRDY_PULSE_INT2_WRIST_TILT_MASK	0x01
#define LSM6DS3_REFERENCE_G					0x0C
#define LSM6DS3_INT1_CTRL					0x0D
	#define INT1_CTRL_STEP_DETECT_MASK		0x80
	#define INT1_CTRL_SIG_MOTION_MASK		0x40
	#define INT1_CTRL_FULL_FLAG_MASK		0x20
	#define INT1_CTRL_FIFO_OVR_MASK			0x10
	#define INT1_CTRL_FIFO_THRS_MASK		0x08
	#define INT1_CTRL_BOOT_MASK				0x04
	#define INT1_CTRL_DRDY_GYRO_MASK		0x02
	#define INT1_CTRL_DRDY_ACCEL_MASK		0x01
#define LSM6DS3_INT2_CTRL					0x0E
	#define INT2_CTRL_STEP_DELTA_MASK		0x80
	#define INT2_CTRL_STEP_CNT_OVR_MASK		0x40
	#define INT2_CTRL_FULL_FLAG_MASK		0x20
	#define INT2_CTRL_FIFO_OVR_MASK			0x10
	#define INT2_CTRL_FIFO_THRS_MASK		0x08
	#define INT2_CTRL_DRDY_TEMP_MASK		0x04
	#define INT2_CTRL_DRDY_GYRO_MASK		0x02
	#define INT2_CTRL_DRDY_ACCEL_MASK		0x01
#define LSM6DS3_WHO_AM_I_REG				0x0F
#define LSM6DS3_CTRL1_XL					0x10
	#define CTRL1_XL_ODR_MASK				0xF0
	#define CTRL1_XL_FS_MASK				0x0C
	#define CTRL1_XL_LPF1_BW_MASK			0x02
#define LSM6DS3_CTRL2_G						0x11
	#define CTRL2_GYRO_ODR_MASK				0xF0
	#define CTRL2_GYRO_FS_MASK				0x0C
	#define CTRL2_GYRO_FS_125_MASK			0x02
#define LSM6DS3_CTRL3_C						0x12
	#define CTRL3_BOOT_MASK					0x80
	#define CTRL3_BDU_MASK					0x40
	#define CTRL3_INT_HLACTIVE_MASK			0x20
	#define CTRL3_INT_PP_OD_MASK			0x10
	#define CTRL3_SIM_MASK					0x08
	#define CTRL3_IF_INC_MASK				0x04
	#define CTRL3_BLE_MASK					0x02
	#define CTRL3_SW_RESET_MASK				0x01
#define LSM6DS3_CTRL4_C						0x13
	#define CTRL4_DEN_XL_EN_MASK			0x80
	#define CTRL4_SLEEP_G_MASK				0x40
	#define CTRL4_INT2_ON_INT1_MASK			0x20
	#define CTRL4_DEN_DRDY_INT1_MASK		0x10
	#define CTRL4_DRDY_MASK					0x08
	#define CTRL4_I2C_DIS_MASK				0x04
	#define CTRL4_LPF1_G_EN_MASK			0x02
#define LSM6DS3_CTRL5_C						0x14
	#define CTRL5_ROUNDING_MASK				0xE0
	#define CTRL5_DEN_LH_MASK				0x10
	#define CTRL5_SELFTEST_GYRO_MASK		0x0C
	#define CTRL5_SELFTEST_ACCEL_MASK		0x03
#define LSM6DS3_CTRL6_G 					0x15
	#define CTRL6_TRIG_EN_MASK				0x80
	#define CTRL6_LVL_EN_MASK				0x40
	#define CTRL6_LVL2_EN_MASK				0x20
	#define CTRL6_XL_HPERF_MODE_MASK		0x10
	#define CTRL6_SUSR_OFF_W_MASK			0x08
	#define CTRL6_GYRO_LPF1_BW_MASK			0x03
#define LSM6DS3_CTRL7_G						0x16
	#define CTRL7_GYRO_HPERF_MODE_MASK		0x80
	#define CTRL7_GYRO_HPF_EN_MASK			0x40
	#define CTRL7_GYRO_HPF_CFG_MASK			0x30
	#define CTRL7_ROUNDING_STAT_MASK		0x04
#define LSM6DS3_CTRL8_XL					0x17
	#define CTRL8_LPF2_XL_EN_MASK			0x80
	#define CTRL8_HP_LPF2_CFG_MASK			0x60
	#define CTRL8_HP_REF_MODE_MASK			0x10
	#define CTRL8_INPUT_COMPOSITE_MASK		0x08
	#define CTRL8_HP_SLOPE_XL_EN_MASK		0x04
	#define CTRL8_LP_ON_6D_MASK				0x01
#define LSM6DS3_CTRL9_XL					0x18
	#define CTRL9_DEN_X_MASK				0x80
	#define CTRL9_DEN_Y_MASK				0x40
	#define CTRL9_DEN_Z_MASK				0x20
	#define CTRL9_DEN_XL_GYRO_MASK			0x10
	#define CTRL9_DSOFT_EN_MASK				0x04
#define LSM6DS3_CTRL10_C					0x19
	#define CTRL10_WRIST_TILT_EN_MASK		0x80
	#define CTRL10_TIMER_EN_MASK			0x20
	#define CTRL10_PEDO_EN_MASK				0x10
	#define CTRL10_TILT_EN_MASK				0x08
	#define CTRL10_FUNC_EN_MASK				0x04
	#define CTRL10_PEDO_RESET_MASK			0x02
	#define CTRL10_SIG_MOTION_EN_MASK		0x01
#define LSM6DS3_MASTER_CFG					0x1A
	#define MASTER_CFG_DRDY_INT1_MASK		0x80
	#define MASTER_CFG_DAT_VAL_FIFO_MASK	0x40
	#define MASTER_CFG_START_CONFIG_MASK	0x10
	#define MASTER_CFG_I2C_PULLUP_MASK		0x08
	#define MASTER_CFG_I2C_PASSTHR_MASK		0x04
	#define MASTER_CFG_IRON_CORRECT_MASK	0x02
	#define MASTER_CFG_I2C_MASTER_MASK		0x01
#define LSM6DS3_WAKE_UP_SRC					0x1B
	#define WAKE_UP_SRC_FF_IA_MASK			0x20
	#define WAKE_UP_SRC_SLEEP_IA_MASK		0x10
	#define WAKE_UP_SRC_WU_IA_MASK			0x08
	#define WAKE_UP_SRC_X_WU_MASK			0x04
	#define WAKE_UP_SRC_Y_WU_MASK			0x02
	#define WAKE_UP_SRC_Z_WU_MASK			0x01
#define LSM6DS3_TAP_SRC						0x1C
	#define TAP_SRC_TAP_IA_MASK				0x40
	#define TAP_SRC_SINGLE_TAP_MASK			0x20
	#define TAP_SRC_DOUBLE_TAP_MASK			0x10
	#define TAP_SRC_TAP_SIGN_MASK			0x08
	#define TAP_SRC_X_TAP_MASK				0x04
	#define TAP_SRC_Y_TAP_MASK				0x02
	#define TAP_SRC_Z_TAP_MASK				0x01
#define LSM6DS3_D6D_SRC						0x1D
	#define D6D_SRC_DEN_DRDY_MASK			0x80
	#define D6D_SRC_D6D_IA_MASK				0x40
	#define D6D_SRC_ZH_MASK					0x20
	#define D6D_SRC_ZL_MASK					0x10
	#define D6D_SRC_YH_MASK					0x08
	#define D6D_SRC_YL_MASK					0x04
	#define D6D_SRC_XH_MASK					0x02
	#define D6D_SRC_XL_MASK					0x01
#define LSM6DS3_STATUS_REG					0x1E
	#define STATUS_TEMP_DA_MASK				0x04
	#define STATUS_GYRO_DA_MASK				0x02
	#define STATUS_XL_DA_MASK				0x01
#define LSM6DS3_OUT_TEMP_L					0x20
#define LSM6DS3_OUT_TEMP_H					0x21
#define LSM6DS3_OUTX_L_G  					0x22
#define LSM6DS3_OUTX_H_G  					0x23
#define LSM6DS3_OUTY_L_G  					0x24
#define LSM6DS3_OUTY_H_G  					0x25
#define LSM6DS3_OUTZ_L_G  					0x26
#define LSM6DS3_OUTZ_H_G  					0x27
#define LSM6DS3_OUTX_L_XL  					0x28
#define LSM6DS3_OUTX_H_XL  					0x29
#define LSM6DS3_OUTY_L_XL  					0x2A
#define LSM6DS3_OUTY_H_XL  					0x2B
#define LSM6DS3_OUTZ_L_XL  					0x2C
#define LSM6DS3_OUTZ_H_XL  					0x2D
#define LSM6DS3_SENSORHUB1_REG				0x2E
#define LSM6DS3_SENSORHUB2_REG				0x2F
#define LSM6DS3_SENSORHUB3_REG				0x30
#define LSM6DS3_SENSORHUB4_REG				0x31
#define LSM6DS3_SENSORHUB5_REG  			0x32
#define LSM6DS3_SENSORHUB6_REG  			0x33
#define LSM6DS3_SENSORHUB7_REG  			0x34
#define LSM6DS3_SENSORHUB8_REG  			0x35
#define LSM6DS3_SENSORHUB9_REG  			0x36
#define LSM6DS3_SENSORHUB10_REG  			0x37
#define LSM6DS3_SENSORHUB11_REG  			0x38
#define LSM6DS3_SENSORHUB12_REG  			0x39
#define LSM6DS3_FIFO_STATUS1  				0x3A
	#define FIFO_STATUS1_DIFF_FIFO_L_MASK	0xFF
#define LSM6DS3_FIFO_STATUS2  				0x3B
	#define FIFO_STATUS2_WATER_M_MASK		0x80
	#define FIFO_STATUS2_OVR_MASK			0x40
	#define FIFO_STATUS2_FIFO_FULL_MASK		0x20
	#define FIFO_STATUS2_FIFO_EMPTY_MASK	0x10
	#define FIFO_STATUS2_DIFF_FIFO_H_MASK	0x07	
#define LSM6DS3_FIFO_STATUS3  				0x3C
	#define FIFO_STATUS3_PATTERN_L_MASK		0xFF
#define LSM6DS3_FIFO_STATUS4  				0x3D
	#define FIFO_STATUS4_PATTERN_H_MASK		0x03	
#define LSM6DS3_FIFO_DATA_OUT_L  			0x3E
#define LSM6DS3_FIFO_DATA_OUT_H  			0x3F
#define LSM6DS3_TIMESTAMP0_REG  			0x40
#define LSM6DS3_TIMESTAMP1_REG  			0x41
#define LSM6DS3_TIMESTAMP2_REG  			0x42
#define LSM6DS3_STEP_COUNTER_L  			0x4B
#define LSM6DS3_STEP_COUNTER_H  			0x4C
#define LSM6DS3_SENSORHUB13_REG				0x4D
#define LSM6DS3_SENSORHUB14_REG				0x4E
#define LSM6DS3_SENSORHUB15_REG				0x4F
#define LSM6DS3_SENSORHUB16_REG				0x50
#define LSM6DS3_SENSORHUB17_REG				0x51
#define LSM6DS3_SENSORHUB18_REG				0x52
#define LSM6DS3_FUNC_SRC1  					0x53
	#define FUNC_SRC1_STEP_DELTA_IA_MASK	0x80
	#define FUNC_SRC1_SIG_MOTION_IA_MASK	0x40
	#define FUNC_SRC1_TILT_IA_MASK			0x20
	#define FUNC_SRC1_STEP_DETECTED_MASK	0x10
	#define FUNC_SRC1_STEP_OVR_MASK			0x08
	#define FUNC_SRC1_HI_FAIL_MASK			0x04
	#define FUNC_SRC1_SI_FAIL_MASK			0x02
	#define FUNC_SRC1_HUB_END_OP_MASK		0x01	
#define LSM6DS3_FUNC_SRC2  					0x54
	#define FUNC_SRC2_SLAVE3_NACK_MASK		0x40
	#define FUNC_SRC2_SLAVE2_NACK_MASK		0x20
	#define FUNC_SRC2_SLAVE1_NACK_MASK		0x10
	#define FUNC_SRC2_SLAVE0_NACK_MASK		0x08
	#define FUNC_SRC2_WRIST_TILT_IA_MASK	0x01
#define LSM6DS3_WRIST_TILT_IA				0x55
	#define WRIST_TILT_X_POS_IA_MASK		0x80
	#define WRIST_TILT_X_NEG_IA_MASK		0x40
	#define WRIST_TILT_Y_POS_IA_MASK		0x20
	#define WRIST_TILT_Y_NEG_IA_MASK		0x10
	#define WRIST_TILT_Z_POS_IA_MASK		0x08
	#define WRIST_TILT_Z_NEG_IA_MASK		0x04
#define LSM6DS3_TAP_CFG  					0x58
	#define TAP_CFG_FUNC_INT_EN_MASK		0x80
	#define TAP_CFG_INACT_EN_MASK			0x60
	#define TAP_CFG_SLOPE_FDS_MASK			0x10
	#define TAP_CFG_TAP_X_EN_MASK			0x08
	#define TAP_CFG_TAP_Y_EN_MASK			0x04
	#define TAP_CFG_TAP_Z_EN_MASK			0x02
	#define TAP_CFG_LATCH_INT_MASK			0x01	
#define LSM6DS3_TAP_THS_6D  				0x59
	#define TAP_THS_6D_D4D_EN_MASK			0x80
	#define TAP_THS_6D_6D_THS_MASK			0x60
	#define TAP_THS_6D_TAP_THS_MASK			0x1F
#define LSM6DS3_INT_DUR2  					0x5A
	#define INT_DUR2_DUR_MASK				0xF0
	#define INT_DUR2_QUIET_MASK				0x0C
	#define INT_DUR2_SHOCK_MASK				0x03
#define LSM6DS3_WAKE_UP_THS  				0x5B
	#define WAKE_UP_THS_S_DO_TAP_MASK		0x80
	#define WAKE_UP_THS_MASK				0x3F
#define LSM6DS3_WAKE_UP_DUR  				0x5C
	#define WAKE_UP_DUR_FF_DUR_MASK			0x80
	#define WAKE_UP_DUR_WAKE_DUR_MASK		0x60
	#define WAKE_UP_DUR_TIMER_HR_MASK		0x10
	#define WAKE_UP_DUR_SLEEP_DUR_MASK		0x0F
#define LSM6DS3_FREE_FALL  					0x5D
	#define FREE_FALL_FF_DUR_MASK			0xF8
	#define FREE_FALL_FF_THS_MASK			0x07
#define LSM6DS3_MD1_CFG  					0x5E
	#define MD1_CFG_INT1_INACTIVITY			0x80
	#define MD1_CFG_INT1_SINGLE_TAP			0x40
	#define MD1_CFG_INT1_WAKE_UP			0x20
	#define MD1_CFG_INT1_FREEFALL			0x10
	#define MD1_CFG_INT1_DOUBLE_TAP			0x08
	#define MD1_CFG_INT1_6D					0x04
	#define MD1_CFG_INT1_TILT				0x02
	#define MD1_CFG_INT1_TIMER_ROLL			0x01
#define LSM6DS3_MD2_CFG  					0x5F
	#define MD2_CFG_INT2_INACTIVITY			0x80
	#define MD2_CFG_INT2_SINGLE_TAP			0x40
	#define MD2_CFG_INT2_WAKE_UP			0x20
	#define MD2_CFG_INT2_FREEFALL			0x10
	#define MD2_CFG_INT2_DOUBLE_TAP			0x08
	#define MD2_CFG_INT2_6D					0x04
	#define MD2_CFG_INT2_TILT				0x02
	#define MD2_CFG_INT2_IRON				0x01
#define LSM6DS3_MASTER_CMD_CODE  			0x60
#define LSM6DS3_SENS_SYNC_SPI_ERROR_CODE	0x61
#define LSM6DS3_OUT_MAG_RAW_X_L				0x66
#define LSM6DS3_OUT_MAG_RAW_X_H				0x67
#define LSM6DS3_OUT_MAG_RAW_Y_L				0x68
#define LSM6DS3_OUT_MAG_RAW_Y_H				0x69
#define LSM6DS3_OUT_MAG_RAW_Z_L				0x6A
#define LSM6DS3_OUT_MAG_RAW_Z_H				0x6B
#define LSM6DS3_X_OFS_USR					0x73
#define LSM6DS3_Y_OFS_USR					0x74
#define LSM6DS3_Z_OFS_USR					0x75

/************** Access Device RAM  *******************/
#define LSM6DS3_ADDR0_TO_RW_RAM				0x62
#define LSM6DS3_ADDR1_TO_RW_RAM				0x63
#define LSM6DS3_DATA_TO_WR_RAM				0x64
#define LSM6DS3_DATA_RD_FROM_RAM			0x65
	#define LSM6DS3_RAM_SIZE				4096

/************** Embedded functions register mapping  *******************/
#define LSM6DS3_SLV0_ADD					0x02
#define LSM6DS3_SLV0_SUBADD					0x03
#define LSM6DS3_SLAVE0_CONFIG				0x04
	#define SLAVE0_CFG_RATE_MASK			0xC0
	#define SLAVE0_CFG_AUX_SENS_ON			0x30
	#define SLAVE0_CFG_SRC_MODE				0x80
	#define SLAVE0_CFG_NUM_OPS				0x07
#define LSM6DS3_SLV1_ADD					0x05
#define LSM6DS3_SLV1_SUBADD					0x06
#define LSM6DS3_SLAVE1_CONFIG				0x07
	#define SLAVE1_CFG_RATE_MASK			0xC0
	#define SLAVE1_CFG_WR_ONCE_MASK			0x20
	#define SLAVE1_CFG_NUM_OPS				0x07
#define LSM6DS3_SLV2_ADD					0x08
#define LSM6DS3_SLV2_SUBADD					0x09
#define LSM6DS3_SLAVE2_CONFIG				0x0A
	#define SLAVE2_CFG_RATE_MASK			0xC0
	#define SLAVE2_CFG_NUM_OPS				0x07
#define LSM6DS3_SLV3_ADD					0x0B
#define LSM6DS3_SLV3_SUBADD					0x0C
#define LSM6DS3_SLAVE3_CONFIG				0x0D
	#define SLAVE3_CFG_RATE_MASK			0xC0
	#define SLAVE3_CFG_NUM_OPS				0x07
#define LSM6DS3_D_WR_MODE_SUB_SLV0			0x0E
#define LSM6DS3_CONFIG_PEDO_THS_MIN			0x0F
	#define PEDO_THS_MIN_PEDO_FS_MASK		0x80
	#define PEDO_THS_MIN_THS_MIN_MASK		0x1F
#define LSM6DS3_CONFIG_TILT_IIR				0x10
#define LSM6DS3_CONFIG_TILT_ACOS			0x11
#define LSM6DS3_CONFIG_TILT_WTIME			0x12
#define LSM6DS3_SM_STEP_THS					0x13
#define LSM6DS3_PEDO_DEB_REG				0x14
	#define PEDO_DEB_TIME_MASK				0xF8
	#define PEDO_DEB_THS_MASK				0x07
#define LSM6DS3_STEP_COUNT_DELTA			0x15
#define LSM6DS3_MAG_SI_XX					0x24
#define LSM6DS3_MAG_SI_XY					0x25
#define LSM6DS3_MAG_SI_XZ					0x26
#define LSM6DS3_MAG_SI_YX					0x27
#define LSM6DS3_MAG_SI_YY					0x28
#define LSM6DS3_MAG_SI_YZ					0x29
#define LSM6DS3_MAG_SI_ZX					0x2A
#define LSM6DS3_MAG_SI_ZY					0x2B
#define LSM6DS3_MAG_SI_ZZ					0x2C
#define LSM6DS3_MAG_OFFX_L					0x2D
#define LSM6DS3_MAG_OFFX_H 					0x2E
#define LSM6DS3_MAG_OFFY_L 					0x2F
#define LSM6DS3_MAG_OFFY_H					0x30
#define LSM6DS3_MAG_OFFZ_L 					0x31
#define LSM6DS3_MAG_OFFZ_H					0x32
#define LSM6DS3_A_WRIST_TILT_LAT			0x50
#define LSM6DS3_A_WRIST_TILT_THS			0x54
#define LSM6DS3_A_WRIST_TILT_MASK_REG		0x59

/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define LSM6DS3_EN_BIT		0x01
#define LSM6DS3_DIS_BIT		0x00

#define LSM6DS3_ACCEL_FS_2G_VAL				0x00
#define LSM6DS3_ACCEL_FS_4G_VAL				0x02
#define LSM6DS3_ACCEL_FS_8G_VAL				0x03
#define LSM6DS3_ACCEL_FS_16G_VAL			0x01
#define LSM6DS3_ACCEL_FS_2G_GAIN			61
#define LSM6DS3_ACCEL_FS_4G_GAIN			122
#define LSM6DS3_ACCEL_FS_8G_GAIN			244
#define LSM6DS3_ACCEL_FS_16G_GAIN			488

#define LSM6DS3_GYRO_FS_245_VAL				0x00
#define LSM6DS3_GYRO_FS_500_VAL				0x01
#define LSM6DS3_GYRO_FS_1000_VAL			0x02
#define LSM6DS3_GYRO_FS_2000_VAL			0x03
#define LSM6DS3_GYRO_FS_245_GAIN			8750
#define LSM6DS3_GYRO_FS_500_GAIN			17500
#define LSM6DS3_GYRO_FS_1000_GAIN			35000
#define LSM6DS3_GYRO_FS_2000_GAIN			70000	

#define LSM6DS3_HPERF_ENABLE				(0x00)
#define LSM6DS3_HPERF_DISABLE				(0x01)	
#define LSM6DS3_GYRO_HPF_0_0081_HZ			0
#define LSM6DS3_GYRO_HPF_0_0324_HZ			1
#define LSM6DS3_GYRO_HPF_2_07_HZ			2
#define LSM6DS3_GYRO_HPF_16_32_HZ			3
#define LSM6DS3_GYRO_HPF_EN_MASK			0x40

#define LSM6DS3_MIN_OP_ODR_HZ				13			//Lowest Data Rate that the part can operate at
#define LSM6DS3_MIN_FUNC_ODR_HZ				26			//Lowest Data Rate that will support the Embedded Functions
#define LSM6DS3_MAX_ODR_HZ					416			//Capping at the highest rate for low power mode

#define LSM6DS3_TIMER_RESET					0xAA

//Self Test Defines
#define LSM6DS3_SELFTEST_DISABLED_VAL		0x00
#define LSM6DS3_SELFTEST_POS_SIGN_VAL		0x01
#define LSM6DS3_SELFTEST_NEG_ACCEL_SIGN_VAL	0x02
#define LSM6DS3_SELFTEST_NEG_GYRO_SIGN_VAL	0x03

//Configurables for the accelerometer
#define DEF_ACCEL_OSR		52							//52 HZ
#define DEF_GYRO_OSR		52							//52 HZ
#define DEF_GYRO_FSR		1000						//+-1000 DPS
#define DEF_ACCEL_FSR		4							//+-4g
#define DEF_ACCEL_SENS		(32768/DEF_ACCEL_FSR)
#define DEF_GYRO_SENS		(32768.0f/DEF_GYRO_FSR)

//FIFO related Values
#define LSM6DS3_FIFO_LENGTH_MASK			((FIFO_STATUS2_DIFF_FIFO_H_MASK<<8)|FIFO_STATUS1_DIFF_FIFO_L_MASK)
#define LSM6DS3_FIFO_PATTERN_MASK			((FIFO_STATUS4_PATTERN_H_MASK<<8)|FIFO_STATUS3_PATTERN_L_MASK)
#define LSM6DS3_FIFO_SAMPLES_PER_DATA_SET	3
#define LSM6DS3_FIFO_BYTES_PER_SAMPLE		2
#define LSM6DS3_FIFO_BYTES_PER_DATA_SET		LSM6DS3_FIFO_SAMPLES_PER_DATA_SET*LSM6DS3_FIFO_BYTES_PER_SAMPLE
#define LSM6DS3_FIFO_SIZE					(8 * 1024)
#define FIFO_MAX_PATTERN					256
#define FIFO_MAX_READ						(FIFO_MAX_PATTERN*LSM6DS3_FIFO_BYTES_PER_DATA_SET)
typedef enum
{
	FIFO_GYRO = 0,
	FIFO_ACCEL = 1,
	FIFO_EXTERN = 2,
	FIFO_STEPS = 3,
	
	FIFO_DATA_MAX_SRC_CNT,
} T_FIFO_DATA_SRC;

//Values that program the specified Data Rates
typedef enum
{
	FIFO_BYPASS = 0,
	FIFO_FIFO = 1,
	FIFO_CONTIN_FIFO = 3,
	FIFO_BYPASS_CONTIN = 4,
	FIFO_CONTINUOUS = 6,
} T_FIFO_MODES;

#define ODR_REG_OFFSET	4
typedef enum
{
	ODR_POWER_OFF_VAL =	0x00,
	ODR_13HZ_VAL =		0x01,
	ODR_26HZ_VAL =		0x02,
	ODR_52HZ_VAL =		0x03,
	ODR_104HZ_VAL =		0x04,
	ODR_208HZ_VAL =		0x05,
	ODR_416HZ_VAL =		0x06,
	
	ODR_CNT =			7,
} T_SENSOR_ODR_CODE;

typedef struct 
{
	uint32_t hz;
	uint8_t reg_val;
} T_LSM6DS3_ODR_REG;

typedef struct
{
	unsigned int gain;
	uint8_t value;
	int urv;
} T_LSM6DS3_FS_REG;

typedef enum 
{
	LSM6DS3_ACCEL = 0,
	LSM6DS3_GYRO,
	LSM6DS3_SIG_MOTION,
	LSM6DS3_STEP_COUNTER,
	LSM6DS3_STEP_DETECTOR,
	LSM6DS3_TILT,
	LSM6DS3_EXTERN_SENS,
	LSM6DS3_MOTION_WAKE,
	
	LSM6DS3_SENSORS_NUMB,		//Total Number of Sensor types
} T_LSM6DS3_SENSOR_TYPE;

#define ACCEL_EN_MASK			(1<<LSM6DS3_ACCEL)
#define GYRO_EN_MASK			(1<<LSM6DS3_GYRO)
#define SIG_MOTION_EN_MASK		(1<<LSM6DS3_SIG_MOTION)
#define STEP_COUNTER_EN_MASK	(1<<LSM6DS3_STEP_COUNTER)
#define STEP_DETECTOR_EN_MASK	(1<<LSM6DS3_STEP_DETECTOR)
#define TILT_EN_MASK			(1<<LSM6DS3_TILT)
#define EXTERN_SENS_EN_MASK		(1<<LSM6DS3_EXTERN_SENS)
#define MOTION_WAKE_EN_MASK		(1<<LSM6DS3_MOTION_WAKE)

#define EMBEDDED_FUNC_EN_MASK	(SIG_MOTION_EN_MASK|STEP_COUNTER_EN_MASK|STEP_DETECTOR_EN_MASK|TILT_EN_MASK|EXTERN_SENS_EN_MASK)

#define DEF_ZERO					(0x00)

/* Sensitivity Acc */
#define SENSITIVITY_ACC_2G			(61)	/** ug/LSB */
#define SENSITIVITY_ACC_4G			(122)	/** ug/LSB */
#define SENSITIVITY_ACC_8G			(244)	/** ug/LSB */
#define SENSITIVITY_ACC_16G			(488)	/** ug/LSB */
/* Sensitivity Gyr */
#define SENSITIVITY_GYR_125			(437)	/** 10udps/LSB */
#define SENSITIVITY_GYR_245			(875)	/** 10udps/LSB */
#define SENSITIVITY_GYR_500			(1750)	/** 10udps/LSB */
#define SENSITIVITY_GYR_1000		(3500)	/** 10udps/LSB */
#define SENSITIVITY_GYR_2000		(7000)	/** 10udps/LSB */

#define LSM6DS3_RX_TX_MAX_LENGTH	(0xFE)
#define DATA_OUT_XYZ_SIZE			6

#define LSM6DS3_INT_ACTIVE			1

//Functions are defined for whichever comms module is included in project: I2C or SPI
typedef int (*lsm6ds3_comm_t) (uint8_t reg_addr, uint8_t length, uint8_t *data);	
typedef void (*save_fifo_func_ptr) ( uint8_t * );

ret_code_t lsm6ds3_init_sensors( void );
ret_code_t lsm6ds3_run_self_test( void );
uint16_t lsm6ds3_active_sensors (void );
ret_code_t lsm6ds3_read_accel_reg( uint8_t *data );
ret_code_t lsm6ds3_read_gyro_reg( uint8_t *data );
ret_code_t lsm6ds3_read_sens_hub_reg( uint8_t *data );
uint16_t lsm6ds3_get_accel_fsr( void );
uint16_t lsm6ds3_get_gyro_fsr( void );
uint16_t lsm6ds3_get_accel_samp( void );
uint16_t lsm6ds3_get_gyro_samp( void );
uint16_t lsm6ds3_get_accel_sens( void );
float lsm6ds3_get_gyro_sens( void );
ret_code_t lsm6ds3_enable_sensors( T_LSM6DS3_SENSOR_TYPE s_type );
ret_code_t lsm6ds3_disable_sensors( T_LSM6DS3_SENSOR_TYPE s_type );
ret_code_t lsm6ds3_disable_all( void );
ret_code_t lsm6ds3_wake_on_motion( bool turn_on );
ret_code_t lsm6ds3_check_wom( void );
bool lsm6ds3_check_wakeup_evt( void );
ret_code_t lsm6ds3_fifo_management( void );
ret_code_t lsm6ds3_irq_management( void );
void gyro_save_init( save_fifo_func_ptr save_gyro_func );
void accel_save_init( save_fifo_func_ptr save_accel_func );
void extern_save_init( save_fifo_func_ptr save_extern_func );
void steps_save_init( save_fifo_func_ptr save_steps_func );

#endif /* LSM6DS3_H_ */
