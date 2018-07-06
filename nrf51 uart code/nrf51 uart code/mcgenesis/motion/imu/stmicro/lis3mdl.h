/*
 * lis3mdl.h
 *
 * Created: 8/17/2015 4:54:18 PM
 *  Author: matt
 */ 


#ifndef LIS3MDL_H_
#define LIS3MDL_H_

// LIS3MDL Digital magnetic sensor:
// http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF255198
//

#include "../hal_imu.h"

//#define LIS3MDL

#define DEF_MAGNET_OSR		20							//52 HZ
#define DEF_MAGNET_FSR		4							//+-4 Gauss
#define DEF_MAGNET_SENS		(32768/DEF_MAGNET_FSR)

//Magnetometer Registers
#define LIS3MDL_I2C_ADDRESS		0x1C
#define LIS3MDL_WHO_AM_I		0x0F  // Answer: 0x3D
#define LIS3MDL_CTRL_REG1		0x20
#define LIS3MDL_CTRL_REG2		0x21
#define LIS3MDL_CTRL_REG3		0x22
#define LIS3MDL_CTRL_REG4		0x23
#define LIS3MDL_CTRL_REG5		0x24
#define LIS3MDL_STATUS_REG		0x27
#define LIS3MDL_OUT_X_L			0x28
#define LIS3MDL_OUT_X_H			0x29
#define LIS3MDL_OUT_Y_L			0x2A
#define LIS3MDL_OUT_Y_H			0x2B
#define LIS3MDL_OUT_Z_L			0x2C
#define LIS3MDL_OUT_Z_H			0x2D
#define LIS3MDL_TEMP_OUT_L		0x2E
#define LIS3MDL_TEMP_OUT_H		0x2F  // data
#define LIS3MDL_INT_CFG			0x30
#define LIS3MDL_INT_SRC			0x31
#define LIS3MDL_INT_THS_L		0x32
#define LIS3MDL_INT_THS_H		0x33

//Magnetometer Register Masks
#define LIS3MDL_WHO_AM_I_MASK	0xFF
#define LIS3MDL_ST_MASK			0x01
#define LIS3MDL_DO_MASK			0x1C
#define LIS3MDL_OM_MASK			0x60
#define LIS3MDL_TEMP_EN_MASK	0x80
#define LIS3MDL_SOFT_RST_MASK	0x04
#define LIS3MDL_REBOOT_MASK		0x08
#define LIS3MDL_FS_MASK			0x60
#define LIS3MDL_MD_MASK			0x03
#define LIS3MDL_SIM_MASK		0x04
#define LIS3MDL_LP_MASK			0x20
#define LIS3MDL_BLE_MASK		0x02
#define LIS3MDL_OMZ_MASK		0x0C
#define LIS3MDL_BDU_MASK		0x40
#define LIS3MDL_XDA_MASK		0x01
#define LIS3MDL_YDA_MASK		0x02
#define LIS3MDL_ZDA_MASK		0x04
#define LIS3MDL_ZYXDA_MASK		0x08
#define LIS3MDL_XOR_MASK		0x10
#define LIS3MDL_YOR_MASK		0x20
#define LIS3MDL_ZOR_MASK		0x40
#define LIS3MDL_ZYXOR_MASK		0x80
#define LIS3MDL_IEN_MASK		0x01
#define LIS3MDL_LIR_MASK		0x02
#define LIS3MDL_IEA_MASK		0x04
#define LIS3MDL_ZIEN_MASK		0x20
#define LIS3MDL_YIEN_MASK		0x40
#define LIS3MDL_XIEN_MASK		0x80
#define LIS3MDL_INT_MASK		0x01
#define LIS3MDL_MROI_MASK		0x02
#define LIS3MDL_NTH_Z_MASK		0x04
#define LIS3MDL_NTH_Y_MASK		0x08
#define LIS3MDL_NTH_X_MASK		0x10
#define LIS3MDL_PTH_Z_MASK		0x20
#define LIS3MDL_PTH_Y_MASK		0x40
#define LIS3MDL_PTH_X_MASK		0x80

//Operational Modes
#define LIS3MDL_MD_CONTINUOS_OP	0x00
#define LIS3MDL_MD_SINGLE_CONV	0x01
#define LIS3MDL_MD_POWER_DOWN	0x03

int lis3mdl_init( void );
int lis3mdl_enable( bool );
int lis3mdl_set_odr( uint16_t );
int lis3mdl_set_fs( uint16_t );
int lis3mdl_set_pm( uint8_t );
uint8_t lis3mdl_get_pm( void );
uint16_t lis3mdl_get_fsr( void );
uint16_t lis3mdl_get_odr( void );
uint16_t lis3mdl_get_sens( void );

#endif /* LIS3MDL_H_ */
