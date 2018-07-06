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

// The Addresses of the Control Registers
#define LSM6DS3_FIFO_CTRL1_ADDR				(0x06)
#define LSM6DS3_FIFO_CTRL2_ADDR				(0x07)
#define LSM6DS3_FIFO_CTRL3_ADDR				(0x08)
#define LSM6DS3_FIFO_CTRL4_ADDR				(0x09)
#define LSM6DS3_FIFO_CTRL5_ADDR				(0x0A)
#define LSM6DS3_INT1_CTRL_ADDR				(0x0D)
#define LSM6DS3_INT2_CTRL_ADDR				(0x0E)
#define LSM6DS3_CTRL1_XL_ADDR				(0x10)
#define LSM6DS3_CTRL2_G_ADDR				(0x11)
#define LSM6DS3_CTRL3_C_ADDR				(0x12)
#define LSM6DS3_CTRL4_C_ADDR				(0x13)
#define LSM6DS3_CTRL5_C_ADDR				(0x14)
#define LSM6DS3_CTRL6_C_ADDR				(0x15)
#define LSM6DS3_CTRL7_G_ADDR				(0x16)
#define LSM6DS3_CTRL8_XL_ADDR				(0x17)
#define LSM6DS3_CTRL9_XL_ADDR				(0x18)
#define LSM6DS3_CTRL10_C_ADDR				(0x19)

//Addresses and Masks of Configuration Registers
#define LSM6DS3_MCFG_ADDR					(0x1A)
#define LSM6DS3_MCFG_DAT_RDY_MASK			0x80
#define LSM6DS3_MCFG_DAT_VAL_FIFO_MASK		0x40
#define LSM6DS3_MCFG_START_CONFIG_MASK		0x10
#define LSM6DS3_MCFG_I2C_PULLUP_MASK		0x08
#define LSM6DS3_MCFG_I2C_PASSTHR_MASK		0x04
#define LSM6DS3_MCFG_IRON_CORRECT_MASK		0x02
#define LSM6DS3_MCFG_I2C_MASTER_MASK		0x01
#define LSM6DS3_MD1_CFG_ADDR				(0x5E)
#define LSM6DS3_MD2_CFG_ADDR				(0x5F)
#define LSM6DS3_MD1_CFG_TIMER_ROLL			0x01
#define LSM6DS3_MD2_CFG_IRON				0x01
#define LSM6DS3_MDx_CFG_TILT				0x02
#define LSM6DS3_MDx_CFG_6D					0x04
#define LSM6DS3_MDx_CFG_DOUBLE_TAP			0x08
#define LSM6DS3_MDx_CFG_FREEFALL			0x10
#define LSM6DS3_MDx_CFG_WAKE_UP				0x20
#define LSM6DS3_MDx_CFG_SINGLE_TAP			0x40
#define LSM6DS3_MDx_CFG_INACTIVITY			0x80
#define LSM6DS3_FUNC_CFG_ACCESS_ADDR		(0x01)
#define LSM6DS3_FUNC_CFG_ACCESS_MASK		0x01
#define LSM6DS3_FUNC_CFG_ACCESS_MASK2		0x04
#define LSM6DS3_FUNC_CFG_REG2_MASK			0x80
#define LSM6DS3_FUNC_CFG_START1_ADDR		0x62
#define LSM6DS3_FUNC_CFG_START2_ADDR		0x63

/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define LSM6DS3_WHO_AM_I_ADDR				0x0F
#define LSM6DS3_WHO_AM_I_DEF				0x69
#define LSM6DS3_AXIS_EN_MASK				0x38
#define LSM6DS3_INT1_FULL					0x20
#define LSM6DS3_INT1_FTH					0x08
#define LSM6DS3_ODR_LIST_NUM				5
#define LSM6DS3_BDU_ADDR					LSM6DS3_CTRL3_C_ADDR
#define LSM6DS3_BDU_MASK					0x40
#define LSM6DS3_INT_HLACTIVE_ADDR			LSM6DS3_CTRL3_C_ADDR	//Control Register for Active High or Low Int pins
#define LSM6DS3_INT_HLACTIVE_MASK			0x20
#define LSM6DS3_INT_PP_OD_ADDR				LSM6DS3_CTRL3_C_ADDR	//Control Register for Push Pull or Open Drain Int Pins
#define LSM6DS3_INT_PP_OD_MASK				0x10
#define LSM6DS3_FUNC_EN_ADDR				LSM6DS3_CTRL10_C_ADDR
#define LSM6DS3_FUNC_EN_MASK				0x04
#define LSM6DS3_SELFTEST_ADDR				LSM6DS3_CTRL5_C_ADDR
#define LSM6DS3_SELFTEST_ACCEL_MASK			0x03
#define LSM6DS3_SELFTEST_GYRO_MASK			0x0c
#define LSM6DS3_SELFTEST_DISABLED_VAL		0x00
#define LSM6DS3_SELFTEST_POS_SIGN_VAL		0x01
#define LSM6DS3_SELFTEST_NEG_ACCEL_SIGN_VAL	0x02
#define LSM6DS3_SELFTEST_NEG_GYRO_SIGN_VAL	0x03
#define LSM6DS3_INT_LATCH_ADDR				0x58
#define LSM6DS3_INT_LATCH_MASK				0x01
#define LSM6DS3_TIMER_EN_ADDR				0x58
#define LSM6DS3_TIMER_EN_MASK				0x80
#define LSM6DS3_TIMER_UPPER_BYTE_ADDR		0x42
#define LSM6DS3_TIMER_RESET					0xAA
#define LSM6DS3_INT2_ON_INT1_ADDR			LSM6DS3_CTRL4_C_ADDR
#define LSM6DS3_INT2_ON_INT1_MASK			0x20
#define LSM6DS3_MIN_DURATION_MS				1638
#define LSM6DS3_ROUNDING_ADDR				LSM6DS3_CTRL7_G_ADDR
#define LSM6DS3_ROUNDING_MASK				0x04
#define LSM6DS3_GYRO_HPF_ADDR				LSM6DS3_CTRL7_G_ADDR
#define LSM6DS3_GYRO_HPF_CFG_MASK			0x30
#define LSM6DS3_GYRO_HPF_0_0081_HZ			0
#define LSM6DS3_GYRO_HPF_0_0324_HZ			1
#define LSM6DS3_GYRO_HPF_2_07_HZ			2
#define LSM6DS3_GYRO_HPF_16_32_HZ			3
#define LSM6DS3_GYRO_HPF_EN_MASK			0x40
#define LSM6DS3_WAKE_UP_INT_ADDR			LSM6DS3_MD1_CFG_ADDR
#define LSM6DS3_WAKE_UP_THS_ADDR			0x5B
#define LSM6DS3_WAKE_UP_THS_MASK			0x3F
#define LSM6DS3_WAKE_UP_DUR_ADDR			0x5C
#define LSM6DS3_WAKE_UP_DUR_MASK			0x60
#define LSM6DS3_WAKE_UP_SRC_ADDR			0x1B
#define LSM6DS3_WAKE_UP_SRC_MASK			0x0F
#define LSM6DS3_INTERFACE_STAT_ADDR			LSM6DS3_CTRL4_C_ADDR
#define LSM6DS3_INTERFACE_I2C_DIS_MASK		0x04	
#define DATA_OUT_XYZ_SIZE					6

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define LSM6DS3_ACCEL_ODR_ADDR				LSM6DS3_CTRL1_XL_ADDR
#define LSM6DS3_ACCEL_ODR_MASK				0xF0
#define LSM6DS3_ACCEL_FS_ADDR				LSM6DS3_CTRL1_XL_ADDR
#define LSM6DS3_ACCEL_FS_MASK				0x0c
#define LSM6DS3_ACCEL_FS_2G_VAL				0x00
#define LSM6DS3_ACCEL_FS_4G_VAL				0x02
#define LSM6DS3_ACCEL_FS_8G_VAL				0x03
#define LSM6DS3_ACCEL_FS_16G_VAL			0x01
#define LSM6DS3_ACCEL_FS_2G_GAIN			61
#define LSM6DS3_ACCEL_FS_4G_GAIN			122
#define LSM6DS3_ACCEL_FS_8G_GAIN			244
#define LSM6DS3_ACCEL_FS_16G_GAIN			488
#define LSM6DS3_ACCEL_AXIS_EN_ADDR			LSM6DS3_CTRL9_XL_ADDR
#define LSM6DS3_ACCEL_AXIS_EN_MASK			0x38
#define LSM6DS3_ACCEL_XYZ_EN				0x07
#define LSM6DS3_ACCEL_DRDY_IRQ_MASK			0x01
#define LSM6DS3_ACCEL_STD					1
#define LSM6DS3_ACCEL_STD_FROM_PD			2
#define LSM6DS3_ACCEL_OUT_X_L_ADDR			0x28
#define LSM6DS3_ACCEL_OUT_Y_L_ADDR			0x2a
#define LSM6DS3_ACCEL_OUT_Z_L_ADDR			0x2c

/* CUSTOM VALUES FOR GYRO SENSOR */
#define LSM6DS3_GYRO_ODR_ADDR				LSM6DS3_CTRL2_G_ADDR
#define LSM6DS3_GYRO_ODR_MASK				0xF0
#define LSM6DS3_GYRO_FS_ADDR				LSM6DS3_CTRL2_G_ADDR
#define LSM6DS3_GYRO_FS_MASK				0x0c
#define LSM6DS3_GYRO_FS_245_VAL				0x00
#define LSM6DS3_GYRO_FS_500_VAL				0x01
#define LSM6DS3_GYRO_FS_1000_VAL			0x02
#define LSM6DS3_GYRO_FS_2000_VAL			0x03
#define LSM6DS3_GYRO_FS_245_GAIN			8750
#define LSM6DS3_GYRO_FS_500_GAIN			17500
#define LSM6DS3_GYRO_FS_1000_GAIN			35000
#define LSM6DS3_GYRO_FS_2000_GAIN			70000
#define LSM6DS3_GYRO_AXIS_EN_ADDR			LSM6DS3_CTRL10_C_ADDR
#define LSM6DS3_GYRO_AXIS_EN_MASK			0x38
#define LSM6DS3_GYRO_XYZ_EN					0x07
#define LSM6DS3_GYRO_DRDY_IRQ_MASK			0x02
#define LSM6DS3_GYRO_STD					6
#define LSM6DS3_GYRO_STD_FROM_PD			2
#define LSM6DS3_GYRO_OUT_X_L_ADDR			0x22
#define LSM6DS3_GYRO_OUT_Y_L_ADDR			0x24
#define LSM6DS3_GYRO_OUT_Z_L_ADDR			0x26

// Sensor Hub Registers
#define LSM6DS3_HUB01_OUT_ADDR				0x2E
// Continuous between
#define LSM6DS3_HUB12_OUT_ADDR				0x39
#define LSM6DS3_HUB13_OUT_ADDR				0x4D
// Continuous between
#define LSM6DS3_HUB18_OUT_ADDR				0x52


// VALUES FOR HIGH PERFORMANCE MODES (increased performance/increased current consumption)
#define LSM6DS3_HPERF_GYR_ADDR		LSM6DS3_CTRL7_G_ADDR
#define LSM6DS3_HPERF_GYR_MASK		(0x80)
#define LSM6DS3_HPERF_ACC_ADDR		LSM6DS3_CTRL6_C_ADDR
#define LSM6DS3_HPERF_ACC_MASK		(0x10)
#define LSM6DS3_HPERF_ENABLE		(0x00)
#define LSM6DS3_HPERF_DISABLE		(0x01)			//Anti-Aliasing Filter Disabled (saves power)

/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define LSM6DS3_SIGN_MOTION_EN_ADDR			LSM6DS3_CTRL10_C_ADDR
#define LSM6DS3_SIGN_MOTION_EN_MASK			0x01
#define LSM6DS3_SIGN_MOTION_DRDY_IRQ_MASK	0x40

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define LSM6DS3_PEDOMETER_EN_ADDR			0x58
#define LSM6DS3_PEDOMETER_EN_MASK			0x40
#define LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK	0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK	0x80
#define LSM6DS3_STEP_COUNTER_OUT_L_ADDR		0x4b
#define LSM6DS3_STEP_COUNTER_OUT_SIZE		2
#define LSM6DS3_STEP_COUNTER_RES_ADDR		LSM6DS3_CTRL10_C_ADDR
#define LSM6DS3_STEP_COUNTER_RES_MASK		0x06
#define LSM6DS3_STEP_COUNTER_RES_ALL_EN		0x03
#define LSM6DS3_STEP_COUNTER_RES_FUNC_EN	0x02
#define LSM6DS3_STEP_COUNTER_DURATION_ADDR	0x15

/* CUSTOM VALUES FOR TILT SENSOR */
#define LSM6DS3_TILT_EN_ADDR				0x58
#define LSM6DS3_TILT_EN_MASK				0x20
#define LSM6DS3_TILT_DRDY_IRQ_MASK			0x02

/* Embedded Function Registers */
#define LSM6DS3_SLAVE_0_ADDR				0x02
#define LSM6DS3_SLAVE_0_SUBADDR				0x03
#define LSM6DS3_SLAVE_0_CONFIG				0x04
#define LSM6DS3_SLAVE_1_ADDR				0x05
#define LSM6DS3_SLAVE_1_SUBADDR				0x06
#define LSM6DS3_SLAVE_1_CONFIG				0x07
#define LSM6DS3_SLAVE_2_ADDR				0x08
#define LSM6DS3_SLAVE_2_SUBADDR				0x09
#define LSM6DS3_SLAVE_2_CONFIG				0x0A
#define LSM6DS3_SLAVE_3_ADDR				0x0B
#define LSM6DS3_SLAVE_3_SUBADDR				0x0C
#define LSM6DS3_SLAVE_3_CONFIG				0x0D
#define LSM6DS3_DATAWR_MODE_SUB_SLAVE_0		0x0E
#define LSM6DS3_SM_THS						0x13
#define LSM6DS3_STEP_CNT_DELTA				0X15

// FIFO related Values
#define LSM6DS3_FIFO_MODE_ADDR				LSM6DS3_FIFO_CTRL5_ADDR
#define LSM6DS3_FIFO_MODE_MASK				0x07
#define LSM6DS3_FIFO_MODE_BYPASS			0x00
#define LSM6DS3_FIFO_MODE_CONTINUOS			0x06
#define LSM6DS3_FIFO_INT_ADDR				LSM6DS3_INT1_CTRL_ADDR
#define LSM6DS3_FIFO_THRESHOLD_IRQ_MASK		0x08
#define LSM6DS3_FIFO_OVR_IRQ_MASK			0x10
#define LSM6DS3_FIFO_FULL_IRQ_MASK			0x20
#define LSM6DS3_FIFO_ODR_ADDR				LSM6DS3_FIFO_CTRL5_ADDR
#define LSM6DS3_FIFO_ODR_MASK				0x78
#define LSM6DS3_FIFO_ODR_MAX				0x07
#define LSM6DS3_FIFO_ODR_MAX_HZ				800
#define LSM6DS3_FIFO_ODR_OFF				0x00
#define LSM6DS3_FIFO_A_G_DECIMATOR_ADDR		LSM6DS3_FIFO_CTRL3_ADDR	
#define LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK	0x07
#define LSM6DS3_FIFO_GYRO_DECIMATOR_MASK	0x38
#define LSM6DS3_FIFO_AG_HIGH_BYTE_ONLY_ADDR LSM6DS3_FIFO_CTRL4_ADDR
#define LSM6DS3_FIFO_AG_HIGH_BYTE_ONLY_MASK	0x40
#define LSM6DS3_FIFO_3_4_DECIMATOR_ADDR		LSM6DS3_FIFO_CTRL4_ADDR
#define LSM6DS3_FIFO_DATA3_DECIMATOR_MASK	0x07
#define LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK	0x38
#define LSM6DS3_FIFO_THR_L_ADDR				LSM6DS3_FIFO_CTRL1_ADDR
#define LSM6DS3_FIFO_THR_H_ADDR				LSM6DS3_FIFO_CTRL2_ADDR
#define LSM6DS3_FIFO_THR_H_MASK				0x0f
#define LSM6DS3_FIFO_PEDO_E_ADDR			LSM6DS3_FIFO_CTRL2_ADDR
#define LSM6DS3_FIFO_PEDO_E_MASK			0x80
#define LSM6DS3_FIFO_STEP_C_FREQ			25

#define LSM6DS3_FIFO_STATUS_1_ADDR			(0x3A)
#define LSM6DS3_FIFO_STATUS_2_ADDR			(0x3B)
#define LSM6DS3_FIFO_STATUS_3_ADDR			(0x3C)
#define LSM6DS3_FIFO_STATUS_4_ADDR			(0x3D)
#define LSM6DS3_FIFO_DIFF_L_ADDR			LSM6DS3_FIFO_STATUS_1_ADDR
#define LSM6DS3_FIFO_DIFF_H_ADDR			LSM6DS3_FIFO_STATUS_2_ADDR
#define LSM6DS3_FIFO_DIFF_MASK				0x0fff
#define LSM6DS3_FIFO_DATA_OUT_L_ADDR		0x3E
#define LSM6DS3_FIFO_SAMPLES_PER_DATA_SET	3
#define LSM6DS3_FIFO_BYTES_PER_SAMPLE		2
#define LSM6DS3_FIFO_BYTES_PER_DATA_SET		LSM6DS3_FIFO_SAMPLES_PER_DATA_SET*LSM6DS3_FIFO_BYTES_PER_SAMPLE
#define LSM6DS3_FIFO_SIZE					(8 * 1024)
#define LSM6DS3_FIFO_DATA_OVR_2REGS			0x4000
#define LSM6DS3_FIFO_DATA_OVR				0x40
#define LSM6DS3_FIFO_DATA_AVL_ADDR			LSM6DS3_FIFO_STATUS_2_ADDR
#define LSM6DS3_FIFO_DATA_AVL				0x80
#define LSM6DS3_FIFO_PATTERN_L_ADDR			LSM6DS3_FIFO_STATUS_3_ADDR
#define LSM6DS3_FIFO_PATTERN_H_ADDR			LSM6DS3_FIFO_STATUS_4_ADDR
#define LSM6DS3_FIFO_PATTERN_MASK			0x03ff
enum fifo_sets{
	FIFO_GYRO = 0,
	FIFO_ACCEL = 1,
	FIFO_EXTERN = 2,
	FIFO_STEPS = 3,
	
	FIFO_DATA_SET_CNT,
};
						
// Status Related Values
#define LSM6DS3_STATUS_ADDR					0x1E
#define LSM6DS3_BOOT_RUNNING_MASK			0x08
#define LSM6DS3_TEMP_DATA_AVL_MASK			0x04
#define LSM6DS3_GYRO_DATA_AVL_MASK			0x02
#define LSM6DS3_ACCEL_DATA_AVL_MASK			0x01
#define LSM6DS3_SRC_FUNC_ADDR				0x53
#define LSM6DS3_SRC_SIGN_MOTION_DATA_AVL	0x40
#define LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL	0x10
#define LSM6DS3_SRC_TILT_DATA_AVL			0x20
#define LSM6DS3_SRC_STEP_COUNTER_DATA_AVL	0x80

// Reset
#define LSM6DS3_RESET_ADDR					LSM6DS3_CTRL3_C_ADDR
#define LSM6DS3_RESET_MASK					0x01

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

#define FIFO_MAX_PATTERN	256
#define FIFO_MAX_READ		(FIFO_MAX_PATTERN*LSM6DS3_FIFO_BYTES_PER_DATA_SET)
struct {
	uint8_t list[ FIFO_MAX_PATTERN ];
	uint16_t samps_pat[ FIFO_DATA_SET_CNT ];
} fifo_pattern;

typedef struct {
	uint32_t hz;
	uint8_t reg_val;
} lsm6ds3_odr_reg_t;

//Values that program the specified Data Rates
#define ODR_REG_OFFSET	4
enum {
	ODR_POWER_OFF_VAL =	0x00,
	ODR_13HZ_VAL =		0x01,
	ODR_26HZ_VAL =		0x02,
	ODR_52HZ_VAL =		0x03,
	ODR_104HZ_VAL =		0x04,
	ODR_208HZ_VAL =		0x05,
	ODR_416HZ_VAL =		0x06,
	
	ODR_CNT =			7,
};

static const struct lsm6ds3_odr_table{
	uint8_t addr[2];
	uint8_t mask[2];
	lsm6ds3_odr_reg_t odr[ODR_CNT];
} lsm6ds3_odr_table = {
	.addr[LSM6DS3_ACCEL] = LSM6DS3_ACCEL_ODR_ADDR,
	.mask[LSM6DS3_ACCEL] = LSM6DS3_ACCEL_ODR_MASK,
	.addr[LSM6DS3_GYRO] = LSM6DS3_GYRO_ODR_ADDR,
	.mask[LSM6DS3_GYRO] = LSM6DS3_GYRO_ODR_MASK,
	.odr[0] = { .hz = 0, .reg_val = ODR_POWER_OFF_VAL },
	.odr[1] = { .hz = 13, .reg_val = ODR_13HZ_VAL },
	.odr[2] = { .hz = 26, .reg_val = ODR_26HZ_VAL },
	.odr[3] = { .hz = 52, .reg_val = ODR_52HZ_VAL },
	.odr[4] = { .hz = 104, .reg_val = ODR_104HZ_VAL },
	.odr[5] = { .hz = 208, .reg_val = ODR_208HZ_VAL },
	.odr[6] = { .hz = 416, .reg_val = ODR_416HZ_VAL },
};

#define LSM6DS3_MIN_OP_ODR_HZ			13			//Lowest Data Rate that the part can operate at
#define LSM6DS3_MIN_FUNC_ODR_HZ			26			//Lowest Data Rate that will support the Embedded Functions

typedef struct  {
	unsigned int gain;
	uint8_t value;
	int urv;
} lsm6ds3_fs_reg_t;

#define LSM6DS3_FS_LIST_NUM		4
static const struct lsm6ds3_fs_table{
	uint8_t addr;
	uint8_t mask;
	lsm6ds3_fs_reg_t fs_avl[LSM6DS3_FS_LIST_NUM];
} lsm6ds3_fs_table[2] = {
	[LSM6DS3_ACCEL] = {
		.addr = LSM6DS3_ACCEL_FS_ADDR,
		.mask = LSM6DS3_ACCEL_FS_MASK,
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
		.addr = LSM6DS3_GYRO_FS_ADDR,
		.mask = LSM6DS3_GYRO_FS_MASK,
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
static int lsm6ds3_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data);
static int lsm6ds3_set_fs( T_LSM6DS3_SENSOR_TYPE s_type, int32_t fsr );
int lsm6ds3_set_irq( T_LSM6DS3_SENSOR_TYPE s_type, bool on_off );
static int lsm6ds3_enable_pedometer( bool enable );
static int lsm6ds3_reset_steps( void );
int lsm6ds3_get_step_data( uint16_t *steps );
static int lsm6ds3_enable_embedded_func( bool enable );
bool write_external_sensor( uint8_t id, uint8_t reg_addr, uint8_t data );
bool lsm6ds3_init_lis3mdl( bool load_calibration );
void (* save_gyro_data) ( uint8_t *data ) = NULL;
void (* save_accel_data) ( uint8_t *data ) = NULL;
void (* save_extern_data) ( uint8_t *data ) = NULL;
void (* save_steps_data) ( uint8_t *data ) = NULL;
int lsm6ds3_set_fifo_mode( enum fifo_mode fm );
static int lsm6ds3_set_fifo_decimators_and_threshold( void );
static int lsm6ds3_set_fifo_enable( bool on_off );
int lsm6ds3_reconfigure_fifo( void );
int lsm6ds3_read_fifo( bool check_fifo_len );
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

int lsm6ds3_init_sensors( void )
{
	volatile int err = 0;
	uint8_t temp_reg_val = 0;
	
	imu_cfg.sensor_en = 0;
	
	//Make sure Sampling Rates are within Possible Bounds
	if( imu_cfg.accel.odr_hz < 1 ) {
		//can't be done
		imu_cfg.accel.odr_hz = 1;	//minimum rate
	}
	else if( imu_cfg.accel.odr_hz > 416 ) {
		imu_cfg.accel.odr_hz = 416;	//maximum rate
	}
	if( imu_cfg.gyro.odr_hz < 1 ) {
		//can't be done
		imu_cfg.gyro.odr_hz = 1;	//minimum rate
	}
	else if( imu_cfg.gyro.odr_hz > 416 ) {
		imu_cfg.gyro.odr_hz = 416;	//maximum rate
	}

	// Issue Software Reset to make sure registers are at Default Settings
	err = lsm6ds3_write_data_with_mask(LSM6DS3_RESET_ADDR, LSM6DS3_RESET_MASK, LSM6DS3_EN_BIT);
	if( err != 0 ) return err;
	
	//Wait for Device to finish Reboot
	do {
		delay_ms( 10 );		//Stall while device runs its Boot routine
		err = lsm6ds3_read( LSM6DS3_STATUS_ADDR, 1, &temp_reg_val);
		if( err != 0 ) return err;
	}
	while( (temp_reg_val&LSM6DS3_BOOT_RUNNING_MASK) != 0x00 );
	
	//Read and Print Control Register to check default state
	//err = lsm6ds3_read( LSM6DS3_CTRL3_C_ADDR, 1, &temp_reg_val);
	//if( imu_debug ) app_trace_log("CTRL Reg 3: 0x%02X\r", temp_reg_val);
	
	//Set Up Accelerometer
	err = lsm6ds3_write_data_with_mask( LSM6DS3_ACCEL_AXIS_EN_ADDR, LSM6DS3_ACCEL_AXIS_EN_MASK, LSM6DS3_ACCEL_XYZ_EN );
	err = lsm6ds3_set_fs( LSM6DS3_ACCEL, DEF_ACCEL_FSR );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_HPERF_ACC_ADDR, LSM6DS3_HPERF_ACC_MASK, LSM6DS3_HPERF_DISABLE );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_WAKE_UP_THS_ADDR, LSM6DS3_WAKE_UP_THS_MASK, 0x02 );
	if (err != 0) return err;
	
	//Set up Gyrometer
	err = lsm6ds3_write_data_with_mask( LSM6DS3_GYRO_AXIS_EN_ADDR, LSM6DS3_GYRO_AXIS_EN_MASK, LSM6DS3_GYRO_XYZ_EN );
	err = lsm6ds3_set_fs( LSM6DS3_GYRO, DEF_GYRO_FSR );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_HPERF_GYR_ADDR, LSM6DS3_HPERF_GYR_MASK, LSM6DS3_HPERF_DISABLE );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_GYRO_HPF_ADDR, LSM6DS3_GYRO_HPF_CFG_MASK, LSM6DS3_GYRO_HPF_0_0324_HZ );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_GYRO_HPF_ADDR, LSM6DS3_GYRO_HPF_EN_MASK, LSM6DS3_EN_BIT );
	if (err != 0) return err;

	//Interrupt Sources latch until read to clear
	err = lsm6ds3_write_data_with_mask( LSM6DS3_INT_LATCH_ADDR, LSM6DS3_INT_LATCH_MASK, LSM6DS3_EN_BIT );
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_INT_HLACTIVE_ADDR, LSM6DS3_INT_HLACTIVE_MASK, LSM6DS3_DIS_BIT );	//Active High when Disabled
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_INT_PP_OD_ADDR, LSM6DS3_INT_PP_OD_MASK, LSM6DS3_DIS_BIT );	//Open Push-Pull when Disabled
	if (err != 0) return err;
	
	//Block Data Registers from loading new readings between the reading of the LSB and MSB
	err = lsm6ds3_write_data_with_mask( LSM6DS3_BDU_ADDR, LSM6DS3_BDU_MASK, LSM6DS3_EN_BIT );
	if (err != 0) return err;
	
	//Enable Circular Buffering of Data Registers
	err = lsm6ds3_write_data_with_mask( LSM6DS3_ROUNDING_ADDR, LSM6DS3_ROUNDING_MASK, LSM6DS3_EN_BIT );
	if (err != 0) return err;
	
	//All Enabled Interrupt2 Sources indicate on Int1 Output
	err = lsm6ds3_write_data_with_mask( LSM6DS3_INT2_ON_INT1_ADDR, LSM6DS3_INT2_ON_INT1_MASK, LSM6DS3_EN_BIT );
	if (err != 0) return err;

	// Start the time stamp counter
	temp_reg_val = LSM6DS3_TIMER_RESET;
	err = lsm6ds3_write( LSM6DS3_TIMER_UPPER_BYTE_ADDR, 1, &temp_reg_val );
	err = lsm6ds3_write_data_with_mask( LSM6DS3_TIMER_EN_ADDR, LSM6DS3_TIMER_EN_MASK, LSM6DS3_EN_BIT );
	if (err != 0) return err;

	//Enable access to the embedded function Registers
	lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS_ADDR, LSM6DS3_FUNC_CFG_REG2_MASK, LSM6DS3_EN_BIT );

	temp_reg_val = 0;
	lsm6ds3_write( LSM6DS3_STEP_COUNTER_DURATION_ADDR, 1, &temp_reg_val );

	//Disable access to the embedded function Registers
	do{
		// Need to turn Embedded Register Access Off. If this remains on and we right the wrong register, 
		// it could cause permanent damage to the device as per page 82 of datasheet.
		lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS_ADDR, LSM6DS3_FUNC_CFG_REG2_MASK, LSM6DS3_DIS_BIT );
		err = lsm6ds3_read( LSM6DS3_FUNC_CFG_ACCESS_ADDR, 1, &temp_reg_val );
		if( err != 0 ) return err;
	}
	while( (temp_reg_val&LSM6DS3_FUNC_CFG_REG2_MASK) != 0 );
	
	//Route the desired Function Interrupts to indicate on the Int1 pin
	temp_reg_val = LSM6DS3_MDx_CFG_WAKE_UP;
	err = lsm6ds3_write( LSM6DS3_MD1_CFG_ADDR, 1, &temp_reg_val );
	if (err != 0) return err;
	
	//Clear IMU Step Count
	err = lsm6ds3_reset_steps();

#if defined( LIS3MDL )	
	if( lsm6ds3_init_lis3mdl( false ) == false ) {
		if (imu_debug) app_trace_log("Compass Init fail\r");
	}
#endif

#if defined (LSM6DS3_FIFO_MODE)
	err = lsm6ds3_reconfigure_fifo();
	if (err != 0) return err;
#endif
	
	return err;
}

static int lsm6ds3_write_data_with_mask(uint8_t reg_addr, uint8_t mask, uint8_t data)
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

int lsm6ds3_read_accel_reg( uint8_t *data ) 
{
	return lsm6ds3_read(LSM6DS3_ACCEL_OUT_X_L_ADDR, DATA_OUT_XYZ_SIZE, data);
}

int lsm6ds3_read_gyro_reg( uint8_t *data ) 
{
	return lsm6ds3_read(LSM6DS3_GYRO_OUT_X_L_ADDR, DATA_OUT_XYZ_SIZE, data);
}

int lsm6ds3_read_sens_hub_reg( uint8_t *data ) 
{
	return lsm6ds3_read(LSM6DS3_HUB01_OUT_ADDR, DATA_OUT_XYZ_SIZE, data);
}

int lsm6ds3_get_step_data( uint16_t *steps )
{
	uint16_t data;
	int err = 0;
	
	err = lsm6ds3_read(LSM6DS3_STEP_COUNTER_OUT_L_ADDR, LSM6DS3_STEP_COUNTER_OUT_SIZE, (uint8_t *)&data);
	if (err == 0) {
		*steps = data;
	}
	else {
		*steps = 0;
	}

	return err;
}

int lsm6ds3_wake_on_motion( bool en_wom ) 
{
	int err = 0;
	uint8_t dummy_read;
	
	if( en_wom  == LSM6DS3_EN_BIT ) {
		lsm6ds3_disable_all();	//make sure everything that draws power is shutdown
		
		//Accel needs to be on at minimum data rate to run Wake on Motion Detection
		uint16_t temp_odr = LSM6DS3_MIN_OP_ODR_HZ;
		lsm6ds3_set_odr( LSM6DS3_ACCEL, &temp_odr);
		
		//Read the Wake up source to clear it
		lsm6ds3_read(LSM6DS3_WAKE_UP_SRC_ADDR, 1, &dummy_read);	
		
		//turn on Wake on Motion Int
		err = lsm6ds3_write_data_with_mask(LSM6DS3_WAKE_UP_INT_ADDR, LSM6DS3_MDx_CFG_WAKE_UP, LSM6DS3_EN_BIT);	
		
		//Indicate that WOM running
		imu_cfg.sensor_en |= MOTION_WAKE_EN_MASK;
	}
	else {
		//turn Off Wake on Motion Int
		err = lsm6ds3_write_data_with_mask(LSM6DS3_WAKE_UP_INT_ADDR, LSM6DS3_MDx_CFG_WAKE_UP, LSM6DS3_DIS_BIT);	
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
int lsm6ds3_check_wom( void ) 
{
	int err;
	uint8_t read_val;

	err = lsm6ds3_read( LSM6DS3_ACCEL_ODR_ADDR, 1, &read_val);	
	if (err != 0) return err;
	if( (read_val&LSM6DS3_ACCEL_ODR_MASK) != (ODR_13HZ_VAL<<ODR_REG_OFFSET) ) {
		if (imu_debug) app_trace_log("ODR Compare: 0x%02X, 0x%02X\r", (read_val&LSM6DS3_ACCEL_ODR_MASK), (ODR_13HZ_VAL<<ODR_REG_OFFSET) );
		return (-1);
	}
	
	err = lsm6ds3_read( LSM6DS3_WAKE_UP_INT_ADDR, 1, &read_val);	
	if (err != 0) return err;
	if( (read_val&LSM6DS3_MDx_CFG_WAKE_UP) != LSM6DS3_MDx_CFG_WAKE_UP ) {
		if (imu_debug) app_trace_log("WOM not Enabled\r");
		
		imu_cfg.sensor_en &= ~MOTION_WAKE_EN_MASK;
		
		return (-1);
	}
	
	return 0;
}

bool lsm6ds3_check_wakeup_evt( void )
{
	int err;
	uint8_t read_val = 0;

	err = lsm6ds3_read(LSM6DS3_WAKE_UP_SRC_ADDR, 1, &read_val);
	if( err != 0 ) return false;
	
	if( (read_val&LSM6DS3_WAKE_UP_SRC_MASK) > 0 ) return true;
	
	return false;
}

int lsm6ds3_set_irq( T_LSM6DS3_SENSOR_TYPE s_type, bool on_off )
{
	uint8_t reg_addr, mask, data;
	uint8_t dummy_read;
	int err = 0;

	if( on_off == LSM6DS3_EN_BIT ) data = LSM6DS3_EN_BIT;
	else data = LSM6DS3_DIS_BIT;

	switch (s_type) {
		case LSM6DS3_ACCEL:			
			//uint8_t data[6];
			//lsm6ds3_read_accel_reg( data );

#if defined (LSM6DS3_FIFO_MODE)
			reg_addr = LSM6DS3_FIFO_INT_ADDR;
			mask = LSM6DS3_FIFO_THRESHOLD_IRQ_MASK;
			
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
			reg_addr = LSM6DS3_FIFO_INT_ADDR;
			mask = LSM6DS3_FIFO_THRESHOLD_IRQ_MASK;
			
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
			lsm6ds3_read(LSM6DS3_SRC_FUNC_ADDR, 1, &dummy_read);
			
			reg_addr = LSM6DS3_INT2_CTRL_ADDR;
			mask = LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK;		//Step Counter Delta Time Int Enable
			break;

		case LSM6DS3_SIG_MOTION:
			//Read Status Register to Clear it Out
			lsm6ds3_read(LSM6DS3_SRC_FUNC_ADDR, 1, &dummy_read);
			
			reg_addr = LSM6DS3_INT1_CTRL_ADDR;
			mask = LSM6DS3_SIGN_MOTION_DRDY_IRQ_MASK;
			break;
			
		case LSM6DS3_STEP_DETECTOR:
			//Read Status Register to Clear it Out
			lsm6ds3_read(LSM6DS3_SRC_FUNC_ADDR, 1, &dummy_read);
			
			reg_addr = LSM6DS3_INT1_CTRL_ADDR;
			mask = LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
			break;
		
		case LSM6DS3_TILT:
			//Read Status Register to Clear it Out
			lsm6ds3_read(LSM6DS3_SRC_FUNC_ADDR, 1, &dummy_read);
			
			reg_addr = LSM6DS3_MD1_CFG_ADDR;
			mask = LSM6DS3_TILT_DRDY_IRQ_MASK;
			break;
			
		case LSM6DS3_EXTERN_SENS:
			return 0;		//no IRQ associated with this sensor. Data is sampled based on Accel Data Ready (and potentially INT source 2...ignored).
			
		case LSM6DS3_MOTION_WAKE:
			return 0;		//no IRQ associated with this sensor. Data is sampled based on Accel Data Ready (and potentially INT source 2...ignored).
			
		default:
			return ERR_INVALID_ARG;
	}
	
	//Enable Data Int Source
	lsm6ds3_write_data_with_mask(reg_addr, mask, data);
	
	return err;
}

static int lsm6ds3_set_fs( T_LSM6DS3_SENSOR_TYPE s_type, int32_t fsr )
{
	int err, i;
	uint8_t read_data;
	const struct lsm6ds3_fs_table *fs_table = &lsm6ds3_fs_table[s_type];

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++) {
		if (fs_table->fs_avl[i].urv >= fsr) {
			break;
		}
	}
	if (i >= LSM6DS3_FS_LIST_NUM) return ERR_INVALID_ARG;
	
	do {
		err = lsm6ds3_write_data_with_mask(fs_table->addr, fs_table->mask, fs_table->fs_avl[i].value);
		err = lsm6ds3_read(fs_table->addr, 1, &read_data);
		if( err != 0 ) return err;
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

	return 0;
}

int lsm6ds3_enable_sensors( T_LSM6DS3_SENSOR_TYPE s_type )
{
	int err;
	uint16_t temp_odr;
	
	if (s_type >= LSM6DS3_SENSORS_NUMB) return ERR_INVALID_ARG;	//sensor type does not exist

	//if( (imu_cfg.sensor_en&(1<<s_type)) != 0 ) return 0;	//sensor already on

	switch (s_type) {
		case LSM6DS3_ACCEL:
			temp_odr = imu_cfg.accel.odr_hz;
			err = lsm6ds3_set_odr( LSM6DS3_ACCEL, &temp_odr );
			if( temp_odr != imu_cfg.accel.odr_hz ) {
				//different sampling rate assigned
				if (imu_debug) app_trace_log("Accel ODR Off: %0u Hz\r", temp_odr);
			}
			if (err != 0) return err;
			
			imu_cfg.sensor_en |= ACCEL_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if (err != 0) return err;
#endif
			break;
			
		case LSM6DS3_GYRO:
			temp_odr = imu_cfg.gyro.odr_hz;
			err = lsm6ds3_set_odr( LSM6DS3_GYRO, &temp_odr );	
			if( temp_odr != imu_cfg.gyro.odr_hz ) {
				//different sampling rate assigned
				if (imu_debug) app_trace_log("Gyro ODR Off: %0u Hz\r", temp_odr);
			}
			if (err != 0) return err;
			
			imu_cfg.sensor_en |= GYRO_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if (err != 0) return err;
#endif
			break;
			
		case LSM6DS3_SIG_MOTION:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_SIGN_MOTION_EN_ADDR, LSM6DS3_SIGN_MOTION_EN_MASK, LSM6DS3_EN_BIT);
			if (err != 0) return err;

			imu_cfg.sensor_en |= SIG_MOTION_EN_MASK;
			imu_cfg.motion_event_ready = true;
			
			err = lsm6ds3_enable_embedded_func(true);
			if (err != 0) return err;
			break;
			
		case LSM6DS3_STEP_COUNTER:
		case LSM6DS3_STEP_DETECTOR:
			err = lsm6ds3_enable_pedometer(true);
			if (err != 0) return err;
			
			imu_cfg.sensor_en |= (1<<s_type);
			
			err = lsm6ds3_enable_embedded_func(true);
			if (err != 0) return err;
			break;
			
		case LSM6DS3_TILT:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_TILT_EN_ADDR, LSM6DS3_TILT_EN_MASK, LSM6DS3_EN_BIT);
			if (err != 0) return err;
			
			imu_cfg.sensor_en |= TILT_EN_MASK;
			
			err = lsm6ds3_enable_embedded_func(true);
			if (err != 0) return err;
			break;
			
		case LSM6DS3_EXTERN_SENS:
			//Turn on External Senor Monitoring
			
			imu_cfg.sensor_en |= EXTERN_SENS_EN_MASK;
			break;
		
		case LSM6DS3_MOTION_WAKE:
			//Not handled here			
		default:
			return ERR_INVALID_ARG;
	}

	//Turn on the Interrupt source for this sensor
	err = lsm6ds3_set_irq( s_type, true );
	if (err != 0) return err;

	//uint8_t stat_reg;
	//err = lsm6ds3_read( LSM6DS3_STATUS_ADDR, 1, &stat_reg);
	//if (imu_debug) app_trace_log("Stat Register: 0x%02X\r", stat_reg);

	return 0;
}
	
int lsm6ds3_disable_sensors( T_LSM6DS3_SENSOR_TYPE s_type )
{
	int err = 0;
	uint16_t odr_off_hz = 0;

	if (s_type >= LSM6DS3_SENSORS_NUMB) return ERR_INVALID_ARG;	//sensor type does not exist

	//if( (imu_cfg.sensor_en&(1<<s_type)) == 0 ) return 0;	//sensor already off

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
			if (err != 0) return err;
			
			//Sensor in no longer On, clear indication
			imu_cfg.sensor_en &= ~ACCEL_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if( err != 0 ) return err;
#endif
			break;
			
		case LSM6DS3_GYRO:
			odr_off_hz = 0;
			err = lsm6ds3_set_odr( LSM6DS3_GYRO, &odr_off_hz );
			if (err != 0) return err;
			
			//Sensor in no longer On, clear indication
			imu_cfg.sensor_en &= ~GYRO_EN_MASK;
			
#if defined (LSM6DS3_FIFO_MODE)
			err = lsm6ds3_reconfigure_fifo();
			if( err != 0 ) return err;
#endif
			break;
			
		case LSM6DS3_SIG_MOTION:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_SIGN_MOTION_EN_ADDR, LSM6DS3_SIGN_MOTION_EN_MASK, LSM6DS3_DIS_BIT);
			if (err != 0) return err;

			imu_cfg.sensor_en &= ~SIG_MOTION_EN_MASK;
			imu_cfg.motion_event_ready = false;

			break;
			
		case LSM6DS3_STEP_COUNTER:
		case LSM6DS3_STEP_DETECTOR:
			err = lsm6ds3_enable_pedometer( false );
			if (err != 0) return err;
			
			imu_cfg.sensor_en &= ~(1<<s_type);
			
			break;
			
		case LSM6DS3_TILT:
			err = lsm6ds3_write_data_with_mask(LSM6DS3_TILT_EN_ADDR, LSM6DS3_TILT_EN_MASK, LSM6DS3_DIS_BIT);
			if (err != 0) return err;
			
			imu_cfg.sensor_en &= ~TILT_EN_MASK;

			break;
			
		case LSM6DS3_EXTERN_SENS:
			//Turn off External Senor Monitoring
			
			imu_cfg.sensor_en &= ~EXTERN_SENS_EN_MASK;
			break;
		
		case LSM6DS3_MOTION_WAKE:

			err = lsm6ds3_wake_on_motion( LSM6DS3_DIS_BIT );	
			break;
		
		default:
			return ERR_INVALID_ARG;
	}

	//Turn Off the Interrupt Source for this Sensor
	err = lsm6ds3_set_irq( s_type, false );
	if( err != 0 ) return err;
	
	err = lsm6ds3_enable_embedded_func( false );
	if( err != 0 ) return err;

	return 0;
}

int lsm6ds3_disable_all( void ) {
	int err = 0;
	T_LSM6DS3_SENSOR_TYPE s_type;
	
	for( s_type=LSM6DS3_ACCEL; s_type<LSM6DS3_SENSORS_NUMB; s_type++ ) {
		if( (imu_cfg.sensor_en&(0x0001<<s_type)) > 0 ) {
			err = lsm6ds3_disable_sensors( s_type );
			if( err != 0 ) {
				//shutoff failed, retry once
				err = lsm6ds3_disable_sensors( s_type );
				if( err != 0 ) {
					//failed again, issue Error
					if (imu_debug) app_trace_log("Stop IMU Sensor %01u Failed: %02i\r", s_type, err);
				}
			}
		}
	}
	
	return err;
}

int lsm6ds3_set_odr( T_LSM6DS3_SENSOR_TYPE s_type, uint16_t * odr_hz )
{
	int err = 0;
	uint i;
	
	if (s_type >= LSM6DS3_SENSORS_NUMB) return ERR_INVALID_ARG;			//sensor type does not exist
	if( s_type != LSM6DS3_ACCEL && s_type != LSM6DS3_GYRO ) return ERR_UNSUPPORTED_DEV;	//sensor data rate cannot not be modified
	
	//Fit the Requested Data Rate into the appropriate category
	for( i=0; i<ODR_CNT; i++ ) {
		if( *odr_hz <= lsm6ds3_odr_table.odr[i].hz ) {
			break;
		}
	}
	if (i >= ODR_CNT) return ERR_INVALID_ARG;
	
	err = lsm6ds3_write_data_with_mask(lsm6ds3_odr_table.addr[s_type], lsm6ds3_odr_table.mask[s_type], lsm6ds3_odr_table.odr[i].reg_val);
	if (err != 0) return err;
	
	//Value Successfully Updated
	if( lsm6ds3_odr_table.odr[i].hz != *odr_hz ) *odr_hz = lsm6ds3_odr_table.odr[i].hz;	//update requested frequency to match implemented frequency

	return 0;
}

//Following functions are Related to the IMU Special Features
int lsm6ds3_fifo_management( void )
{
#if defined(LSM6DS3_FIFO_MODE)
	uint8_t src_fifo = 0x00;
	int err;

	err = lsm6ds3_read(LSM6DS3_FIFO_DATA_AVL_ADDR, 1, &src_fifo);
	if( err != 0 ) return err;

	if (src_fifo & LSM6DS3_FIFO_DATA_AVL) {
		if (src_fifo & LSM6DS3_FIFO_DATA_OVR) {
			lsm6ds3_set_fifo_mode(BYPASS);		//Setting to BYPASS flushes buffer
			err = lsm6ds3_set_fifo_mode(CONTINUOS);
			if (imu_debug) app_trace_log("Data FIFO overrun!\r");
		}
		else {
			err = lsm6ds3_read_fifo(true);
		}
		if( err != 0 ) return err;
	}
#endif

	return 0;
}

int lsm6ds3_irq_management( void )
{
	uint8_t src_value = 0x00;
	int err;

	err = lsm6ds3_read(LSM6DS3_SRC_FUNC_ADDR, 1, &src_value);
	if( err != 0 ) return err;

	if (src_value & LSM6DS3_SRC_STEP_COUNTER_DATA_AVL) {
		uint16_t steps_c;
		err = lsm6ds3_get_step_data(&steps_c);
		if (err != 0) {
			//dev_err(cdata->dev, "error while reading step counter data\n");
			//enable_irq(cdata->irq);

			return err;
		}

		//lsm6ds3_report_single_event(&cdata->sensors[LSM6DS3_STEP_COUNTER], steps_c, cdata->sensors[LSM6DS3_STEP_COUNTER].timestamp);
	}

	if (src_value & LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL) {
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

	if (src_value & LSM6DS3_SRC_TILT_DATA_AVL) {
		//sdata = &cdata->sensors[LSM6DS3_TILT];
		//sdata->timestamp = cdata->timestamp;
		//lsm6ds3_report_single_event(sdata, 1, sdata->timestamp);
	}
	
	if( err != 0 ) return err;
	
	//enable_irq(cdata->irq);
	return 0;
}

static int lsm6ds3_enable_embedded_func( bool enable )
{
	static bool em_func_en = false;
	int err;

	if (enable) {
		if( em_func_en == false ) {
			err = lsm6ds3_write_data_with_mask(LSM6DS3_FUNC_EN_ADDR, LSM6DS3_FUNC_EN_MASK, LSM6DS3_EN_BIT);
			if (err != 0) return err;
			em_func_en = true;
		}
	}
	else if ( (imu_cfg.sensor_en&EMBEDDED_FUNC_EN_MASK) == 0 ) {
		if( em_func_en == true ) {
			err = lsm6ds3_write_data_with_mask(LSM6DS3_FUNC_EN_ADDR, LSM6DS3_FUNC_EN_MASK, LSM6DS3_DIS_BIT);
			if (err != 0) return err;
			em_func_en = false;
		}
	}

	//Accelerometer needs to be On for the embedded functions to operate.  
	if( (imu_cfg.sensor_en&ACCEL_EN_MASK) == 0 ) {
		//It is not currently on for motion monitoring...
		if (em_func_en == true) {
			//Make sure it is On at the minimum rate for the embedded functions (26 Hz)
			err = lsm6ds3_write_data_with_mask(lsm6ds3_odr_table.addr[LSM6DS3_ACCEL], lsm6ds3_odr_table.mask[LSM6DS3_ACCEL], ODR_26HZ_VAL);
			if (err != 0) return err;
		} 
		else {
			//It is not needed for the embedded functions, kill it
			err = lsm6ds3_write_data_with_mask(lsm6ds3_odr_table.addr[LSM6DS3_ACCEL], lsm6ds3_odr_table.mask[LSM6DS3_ACCEL], ODR_POWER_OFF_VAL);
			if (err != 0) return err;
		}
	}

	return 0;
}

static int lsm6ds3_enable_pedometer( bool enable )
{
	int err = 0;
	uint8_t value;

	if (enable)	value = LSM6DS3_EN_BIT;
	else value = LSM6DS3_DIS_BIT;
	
#if defined(LSM6DS3_FIFO_MODE)
	err = lsm6ds3_write_data_with_mask(LSM6DS3_FIFO_PEDO_E_ADDR, LSM6DS3_FIFO_PEDO_E_MASK, value);
	if (err != 0) return err;
#endif

	err = lsm6ds3_write_data_with_mask(LSM6DS3_PEDOMETER_EN_ADDR, LSM6DS3_PEDOMETER_EN_MASK, value);
	if (err != 0) return err;
	
	return 0;
}

static int lsm6ds3_reset_steps( void )
{
	int err;
	uint8_t reg_value = 0x00;

	err = lsm6ds3_read(LSM6DS3_STEP_COUNTER_RES_ADDR, 1, &reg_value);
	if (err != 0) return err;

	if (reg_value & LSM6DS3_FUNC_EN_MASK) {
		reg_value = LSM6DS3_STEP_COUNTER_RES_FUNC_EN;
	}
	else {
		reg_value = LSM6DS3_DIS_BIT;
	}

	err = lsm6ds3_write_data_with_mask(LSM6DS3_STEP_COUNTER_RES_ADDR, LSM6DS3_STEP_COUNTER_RES_MASK, reg_value);
	if (err != 0) return err;

	return 0;
}

//Fifo related functions:
static int lsm6ds3_set_fifo_enable(bool on_off)
{
	volatile int err;
	uint8_t reg_value;

	if (on_off) {
		reg_value = LSM6DS3_FIFO_ODR_MAX;
	}
	else {
		reg_value = LSM6DS3_FIFO_ODR_OFF;
	}

	err = lsm6ds3_write_data_with_mask(LSM6DS3_FIFO_ODR_ADDR, LSM6DS3_FIFO_ODR_MASK, reg_value);
	
	return err;
}

int lsm6ds3_read_fifo( bool check_fifo_len )
{
	int err;
	Union32 fifo_stat;
	uint16_t fifo_len = 0;
	uint16_t next_data_set, temp_len;
	volatile uint16_t read_len;
	uint8_t fifo_read[FIFO_MAX_READ];

	if( check_fifo_len ) {
		err = lsm6ds3_read( LSM6DS3_FIFO_STATUS_1_ADDR, 4, &fifo_stat.u8[0] );	//Read the 4 FIFO Status Registers
		if( err != 0 ) {
			if(imu_debug) app_trace_log("FIFO - Status Read Error\r");
			return err;
		}
			
		if( fifo_stat.u8[1]&LSM6DS3_FIFO_DATA_OVR ) {
			if(imu_debug) app_trace_log("FIFO - Overrun\r");
		}

		//The next axis of sensor data is specified in Status Registers 3&4 (bits 9:0)
		next_data_set = fifo_stat.u16[1]&LSM6DS3_FIFO_PATTERN_MASK;
		next_data_set /= LSM6DS3_FIFO_SAMPLES_PER_DATA_SET;
		
		//The FIFO length is contained in the Status Registers 1&2  (bits 11:0)		
		fifo_len = fifo_stat.u16[0];
		fifo_len &= LSM6DS3_FIFO_DIFF_MASK;
		fifo_len *= LSM6DS3_FIFO_BYTES_PER_SAMPLE;
		if( FIFO_DEBUG ) app_trace_log("FIFO Byte Count: %02u DataSet: %02u\r", fifo_len, next_data_set);
		
		if (fifo_len == 0)	{
			//Fifo Empty
			return 0;
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
			err = lsm6ds3_read( LSM6DS3_FIFO_DATA_OUT_L_ADDR, (read_len-temp_len), &fifo_read[temp_len] );
		}
		else {
			err = lsm6ds3_read( LSM6DS3_FIFO_DATA_OUT_L_ADDR, LSM6DS3_RX_TX_MAX_LENGTH, &fifo_read[temp_len] );
		}
		
		if (err != 0) {
			if(imu_debug) app_trace_log("FIFO - Buffer Read Error\r");
			return err;
		}
		
		temp_len += LSM6DS3_RX_TX_MAX_LENGTH;
	}

	lsm6ds3_parse_fifo_data( fifo_read, read_len, next_data_set );
	
	return 0;
}

static void lsm6ds3_parse_fifo_data( uint8_t * data_buffer, uint16_t read_len, uint16_t first_data_set )
{
	uint16_t fifo_offset = 0;
	uint16_t data_set_offset = first_data_set;
	
	if( FIFO_DEBUG ) app_trace_s_msg("FIFO Parse:");
	
	while( (fifo_offset+LSM6DS3_FIFO_BYTES_PER_DATA_SET) <= read_len) {
		
		switch ( fifo_pattern.list[data_set_offset] ) {
			case FIFO_GYRO:
				if( FIFO_DEBUG ) app_trace_s_msg(" G");
				save_gyro_data( &data_buffer[fifo_offset] );
				break;
				
			case FIFO_ACCEL:
				if( FIFO_DEBUG ) app_trace_s_msg(" A");
				save_accel_data( &data_buffer[fifo_offset] );
				break;
				
			case FIFO_EXTERN:
				if( FIFO_DEBUG ) app_trace_s_msg(" E");
				//save_extern_data( &data_buffer[fifo_offset] );
				break;
				
			case FIFO_STEPS:
				if( FIFO_DEBUG ) app_trace_s_msg(" S");
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
	
	if( FIFO_DEBUG ) app_trace_s_msg("\r");

	return;
}

int lsm6ds3_set_fifo_mode(enum fifo_mode fm)
{
	volatile int err;
	uint8_t reg_value;
	bool enable_fifo;

	switch (fm) {
		case BYPASS:
			reg_value = LSM6DS3_FIFO_MODE_BYPASS;
			enable_fifo = false;
			break;
			
		case CONTINUOS:
			reg_value = LSM6DS3_FIFO_MODE_CONTINUOS;
			enable_fifo = true;
			break;
			
		default:
			return ERR_INVALID_ARG;
	}

	err = lsm6ds3_set_fifo_enable(enable_fifo);
	if (err != 0) return err;

	err = lsm6ds3_write_data_with_mask(LSM6DS3_FIFO_MODE_ADDR, LSM6DS3_FIFO_MODE_MASK, reg_value);
	if (err != 0) return err;
	
	return 0;
}

static int lsm6ds3_set_fifo_decimators_and_threshold( void )
{
	int err;
	volatile uint8_t decimator = 0;
	volatile uint16_t min_odr = 416, max_odr = 0;
	volatile uint16_t pattern_len = 0;
	volatile uint16_t fifo_byte_len = 0, fifo_threshold;
	volatile uint16_t pattern_iterations;
	volatile uint16_t min_pattern_iter = LSM6DS3_FIFO_SIZE / LSM6DS3_FIFO_BYTES_PER_DATA_SET;
	
	//Determine the min and max data rates of the data that can be stored in the FIFO
	if( (imu_cfg.sensor_en&ACCEL_EN_MASK) != 0 ) {
		if( imu_cfg.accel.odr_hz == 0 ) {
			//can't be done
			imu_cfg.accel.odr_hz = 1;	//minimum rate
		}
		else if( imu_cfg.accel.odr_hz > 416 ) {
			imu_cfg.accel.odr_hz = 416;	//maximum rate
		}
		min_odr = MIN(min_odr, imu_cfg.accel.odr_hz);
		max_odr = MAX(max_odr, imu_cfg.accel.odr_hz);
	}
	if( (imu_cfg.sensor_en&GYRO_EN_MASK) != 0 ) {
		if( imu_cfg.gyro.odr_hz == 0 ) {
			//can't be done
			imu_cfg.gyro.odr_hz = 1;	//minimum rate
		}
		else if( imu_cfg.gyro.odr_hz > 416 ) {
			imu_cfg.gyro.odr_hz = 416;	//maximum rate
		}
		min_odr = MIN(min_odr, imu_cfg.gyro.odr_hz);
		max_odr = MAX(max_odr, imu_cfg.gyro.odr_hz);
	}
	//if( (imu_cfg.sensor_en&FIFO_SET_3) != 0 ) {
	//	if( set3_rate == 0 ) {
	//		//can't be done
	//	}
	//	min_odr = MIN(min_odr, set3_rate);
	//	max_odr = MAX(max_odr, set3_rate);
	//}
	//if( (imu_cfg.sensor_en&STEP_COUNTER_EN_MASK) != 0 ) {
	//	if( step_rate == 0 ) {
	//		//can't be done
	//	}
	//	min_odr = MIN(min_odr, step_rate);
	//}

	if( (imu_cfg.sensor_en&ACCEL_EN_MASK) ) {
		fifo_pattern.samps_pat[FIFO_ACCEL] = ( imu_cfg.accel.odr_hz/min_odr );	//result will be 1 or greater
		if( fifo_pattern.samps_pat[FIFO_ACCEL] == 0 ){
			if( imu_debug ) app_trace_s_msg("FIFO: Not Possible\r");
		}
		pattern_len += fifo_pattern.samps_pat[FIFO_ACCEL];
		pattern_iterations = MAX( (imu_cfg.accel.odr_hz/4)/fifo_pattern.samps_pat[FIFO_ACCEL], 1 );	//Set threshold for the number of readings in 0.25 seconds
		min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );
		//imu_cfg.accel.deltatime_ns = ( 1000000000ULL/imu_cfg.accel.odr_hz );
		decimator = MAX( max_odr/imu_cfg.accel.odr_hz, 1 );
	} 
	else {
		fifo_pattern.samps_pat[FIFO_ACCEL] = 0;
		decimator = 0;
	}
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_A_G_DECIMATOR_ADDR, LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK, decimator );
	if (err != 0) return err;

	if( (imu_cfg.sensor_en&GYRO_EN_MASK) ) {
		fifo_pattern.samps_pat[FIFO_GYRO] = ( imu_cfg.gyro.odr_hz/min_odr );
		if( fifo_pattern.samps_pat[FIFO_GYRO] == 0 ){
			if( imu_debug ) app_trace_s_msg("FIFO: Not Possible\r");
		}
		pattern_len += fifo_pattern.samps_pat[FIFO_GYRO];
		pattern_iterations = MAX( (imu_cfg.gyro.odr_hz/4)/fifo_pattern.samps_pat[FIFO_GYRO], 1 );	//Set threshold for the number of readings in 0.25 seconds
		min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );	
		//imu_cfg.gyro.deltatime_ns = ( 1000000000ULL/imu_cfg.gyro.odr_hz );
		decimator = MAX( max_odr/imu_cfg.gyro.odr_hz, 1 );
	} 
	else {
		fifo_pattern.samps_pat[FIFO_GYRO] = 0;
		decimator = 0;
	}
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_A_G_DECIMATOR_ADDR, LSM6DS3_FIFO_GYRO_DECIMATOR_MASK, decimator );
	if (err != 0) return err;
	
	//if( fifo_set_3 ) {
		//imu_cfg.fifo.samps_in_pattern[FIFO_EXTERN] = ( set3_odr/min_odr );
		//pattern_len += imu_cfg.fifo.samps_in_pattern[FIFO_EXTERN];
		//pattern_iterations = MAX( set3_fifo_length/imu_cfg.fifo.samps_in_pattern[FIFO_EXTERN], 1 );
		//min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );
		////deltatime.set3 = ( 1000000000ULL/set3_odr );
		//decimator = MAX( max_odr/set3_odr, 1 );
	//} 
	//else {
		//fifo_pattern.samps_pat[FIFO_EXTERN] = 0;
		//decimator = 0;
	//}
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_3_4_DECIMATOR_ADDR, LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK, decimator );
	//if (err != 0) return err;

	//if( (imu_cfg.sensor_en&STEP_COUNTER_EN_MASK) ) {
		//imu_cfg.fifo.samps_in_pattern[FIFO_STEPS] = ( step_odr/min_odr );
		//pattern_len += imu_cfg.fifo.samps_in_pattern[FIFO_STEPS];
		//pattern_iterations = MAX( sdata_step_c->fifo_length/imu_cfg.fifo.samps_in_pattern[FIFO_STEPS], 1 );
		//min_pattern_iter = MIN( min_pattern_iter, pattern_iterations );
		////sdata_step_c->deltatime = ( 1000000000ULL/step_odr );
		//decimator = MAX( max_odr/step_odr, 1 );
	//} else {
		//fifo_pattern.samps_pat[FIFO_STEPS] = 0;
		//decimator = 0; 
	//}
	//err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_3_4_DECIMATOR_ADDR, LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK, decimator );
	//if (err != 0) return err;
	
	if( pattern_len > FIFO_MAX_PATTERN ) {
		//Not enough memory reserved for FIFO pattern
		if( imu_debug ) app_trace_s_msg("FIFO Pattern Too Long\r");
	}

	fifo_threshold = pattern_len*min_pattern_iter*LSM6DS3_FIFO_SAMPLES_PER_DATA_SET;
	fifo_byte_len = fifo_threshold*LSM6DS3_FIFO_BYTES_PER_SAMPLE;

	if (fifo_byte_len > 0) {
		err = lsm6ds3_write( LSM6DS3_FIFO_THR_L_ADDR, 1, (uint8_t *) &fifo_threshold );
		if (err != 0) return err;

		err = lsm6ds3_write_data_with_mask( LSM6DS3_FIFO_THR_H_ADDR, LSM6DS3_FIFO_THR_H_MASK,*((uint8_t *)&fifo_threshold+1) );
		if (err != 0) return err;
		
		int pat_id = 0, iteration = 0;
		while( pat_id < pattern_len ) {
			int data_set;
			//create a list of the data set pattern so the fifo data can easily be parsed
			for( data_set=0; data_set<FIFO_DATA_SET_CNT; data_set++ ) {
				if( ((int)fifo_pattern.samps_pat[data_set] - iteration) > 0 ) {
					fifo_pattern.list[pat_id] = data_set;
					pat_id++;
				}
			}
			iteration++;
		}
		
		if( FIFO_DEBUG ) {
			app_trace_log("FIFO Pattern: ");
			for( pat_id=0 ; pat_id<(pattern_len-1); pat_id++ ) {
				app_trace_log("%02u, ", fifo_pattern.list[pat_id]);
			}
			app_trace_log("%02u\r", fifo_pattern.list[pat_id]);
		}
	}

	imu_cfg.fifo.data_set_thres = pattern_len;
	imu_cfg.fifo.buf_byte_thres = fifo_byte_len;	//number of bytes when Interrupt occurs
	imu_cfg.fifo.odr_hz = max_odr;
	
	//if(imu_debug) app_trace_log("FIFO Thr: %02u, Acc Pat: %02u, Gyr Pat: %02u\r", imu_cfg.fifo.buf_byte_thres, imu_cfg.fifo.samps_in_pattern[FIFO_ACCEL], imu_cfg.fifo.samps_in_pattern[FIFO_GYRO]);

	return fifo_byte_len;
}

int lsm6ds3_reconfigure_fifo( void )
{
	int err, fifo_len = 0;
	
	lsm6ds3_read_fifo(true);

	err = lsm6ds3_set_fifo_mode(BYPASS);	//Flushes Buffer and allows changes to be made
	if( err == 0 ) {
		fifo_len = lsm6ds3_set_fifo_decimators_and_threshold();
		if (fifo_len > 0) {
			err = lsm6ds3_set_fifo_mode(CONTINUOS);
			
			if( err != 0 ) return err;
		}
	}

	return 0;
}

bool write_external_sensor( uint8_t id, uint8_t reg_addr, uint8_t data )
{
	int err;
	uint8_t write_val, read_val = 0;
	
	//todo: Embedded functions may need disabling before accessing
	
	// Step 1: Enable access to the embedded function Registers
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS_ADDR, LSM6DS3_FUNC_CFG_REG2_MASK, LSM6DS3_EN_BIT );
	if( err != 0 ) return false;

	// Step 2: Write 0x38 into SLV0_ADD (LIS3MDL slave address = 0011100b (if SDO=0). Enable write operation (rw_0=0))
	write_val = id;
	lsm6ds3_write( LSM6DS3_SLAVE_0_ADDR, 1, &write_val );
	
	// Step 3: Write 0x22 into SLV0_SUBADD (0x22 is the LIS3MDL register to be written)
	write_val = reg_addr;
	lsm6ds3_write( LSM6DS3_SLAVE_0_SUBADDR, 1, &write_val );
	
	// Step 4: Write 0x00 into DATAWRITE_SRC_MODE_SUB_SLV0 (0x00 is the value to be written in register 0x22 of LIS3MDL
	//			to configure it in continuous conversion mode)
	write_val = data;
	lsm6ds3_write( LSM6DS3_DATAWR_MODE_SUB_SLAVE_0, 1, &write_val );

	// Step 5: Disable access to the embedded function Registers
	do{
		lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS_ADDR, LSM6DS3_FUNC_CFG_REG2_MASK, LSM6DS3_DIS_BIT );
		err = lsm6ds3_read( LSM6DS3_FUNC_CFG_ACCESS_ADDR, 1, &read_val );
		if( err != 0 ) return false;
	}
	while( (read_val&LSM6DS3_FUNC_CFG_REG2_MASK) != 0 );
	
	// Step 6: Enable Embedded functions
	lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_EN_ADDR, LSM6DS3_FUNC_EN_MASK, LSM6DS3_EN_BIT );
	
	// Step 7: Write 0x09 into MASTER_CONFIG (Enable internal pull-up on SDx/SCx lines, Sensor hub trigger signal is
	//			XL Data Ready, Enable auxiliary I2C master)
	lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PULLUP_MASK|LSM6DS3_MCFG_I2C_MASTER_MASK, 0x09 );
	
	// Step 8: Write 0x50 into CTRL1_XL	( Turn-on the accelerometer (for trigger signal))
	lsm6ds3_write_data_with_mask( LSM6DS3_ACCEL_ODR_ADDR, LSM6DS3_ACCEL_ODR_MASK, 0x05 );
	
	//Step 9: wait for message to send out on i2c lines
	delay_ms(10);
	
	//todo: make sure new values have been written?
	//todo: Slave 0 registers may need to be restored
	
	// Step 10: Disable auxiliary I2C master (todo: May need to stay on)
	lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_MASTER_MASK, LSM6DS3_DIS_BIT );
	
	// Step 11: Return Embedded Function to whatever it was before this call
	lsm6ds3_enable_embedded_func( false );
	
	// Step 12: Return Accel to whatever it was before this call
	uint16_t temp_odr = imu_cfg.accel.odr_hz;
	err = lsm6ds3_set_odr( LSM6DS3_ACCEL, &temp_odr );
	if( err != 0 ) return false;
	
	return true;
}

#if defined( LIS3MDL )
//If the LIS3MDL Magnetometer is connected, then it can be initialized by turning on the i2c Pass Through 
//mode. Without Pass Through, the LIS3MDL control registers must be written in a trickier way using the 
//Slave 0 registers of the LSM6DS3... Pass through is much cleaner.
bool lsm6ds3_init_lis3mdl( bool load_calibration )
{
	int err;
	uint8_t write_val, read_val = 0;
	
#if defined( LSM6DS3_I2C_INTERFACE )
	//Enable i2c Pass Through to initialize the external magnetometer sensor
	lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PASSTHR_MASK, LSM6DS3_EN_BIT );
	lis3mdl_init();
	err = lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PASSTHR_MASK, LSM6DS3_DIS_BIT );
	if( err != 0 ) return false;
#else
	//Can't turn on Pass Through, have to write the Control Registers indirectly through the Slave 0 Registers
	err = write_external_sensor( (LIS3MDL_I2C_ADDRESS<<1), LIS3MDL_CTRL_REG3, 0x00 );	//Put LIS3MDL into Continuos Mode
	if( err != 0 ) return false;
#endif
		
	// Enable access to the embedded function Registers
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_CFG_ACCESS_ADDR, LSM6DS3_FUNC_CFG_REG2_MASK, LSM6DS3_EN_BIT );
	if( err != 0 ) return false;
	
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
		if( err != 0 ) return false;
	}
	while( (read_val&LSM6DS3_FUNC_CFG_REG2_MASK) != 0 );
	
	//Have to wait for Embedded Function Register Access to be disabled before attempting to write Regular Registers
	if( load_calibration ) {
		//lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_IRON_CORRECT_MASK, LSM6DS3_EN_BIT );	//Enable Hard-iron correction
		//lsm6ds3_write_data_with_mask( LSM6DS3_CTRL9_XL_ADDR, 0x01, LSM6DS3_EN_BIT );	// Enable Soft-iron correction
	}
	
	// Enable Embedded functions
	err = lsm6ds3_write_data_with_mask( LSM6DS3_FUNC_EN_ADDR, LSM6DS3_FUNC_EN_MASK, LSM6DS3_EN_BIT );
	if( err != 0 ) return false;
	
	err = lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PULLUP_MASK|LSM6DS3_MCFG_I2C_MASTER_MASK, 0x09 );
	if( err != 0 ) return false;
	//lsm6ds3_write_data_with_mask( LSM6DS3_MCFG_ADDR, LSM6DS3_MCFG_I2C_PULLUP_MASK, LSM6DS3_EN_BIT );
	
	//BREAK_POINT;
	
	return true;
}
#endif
