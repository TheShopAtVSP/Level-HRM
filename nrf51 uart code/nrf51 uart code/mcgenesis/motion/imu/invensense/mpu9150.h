/*
 * mpu9150.h
 *
 * Created: 7/30/2014 3:38:02 PM
 *  Author: RichKl
 */ 


#ifndef MPU9150_H_
#define MPU9150_H_

#include "hal_twim.h"

#define MPU_READ	0x80

typedef struct mpu9150_state
{
  inv_pins_t	mpu_pins;								//  Pins on the MCU used to connect to the mpu91                                                                    
} mpu9150_state_t;

typedef struct mpu9150_sensors_t
{
	uint16_t	acel_xout_h;
	uint16_t	acel_yout_h;		                                                                  
	uint16_t	acel_zout_h;	
	uint16_t	gyro_xout_h;
	uint16_t	gyro_yout_h;		                                                                  
	uint16_t	gyro_zout_h;		
} mpu9150_sensors_t;

inv_error_code_t mpu9150_board_init(void);
status_code_t mpu9150_init(inv_pins_t *mpu9150stat, bool debug);
//status_code_t mpu9150_write(uint8_t uireg, uint8_t uidata);
//status_code_t mpu9150_read(uint8_t uireg, uint8_t * uidata);
//status_code_t mpu9150_burstread(uint8_t uireg, uint8_t * uidata, uint8_t len);

//extern struct mpu6000_raw_sensors mpu6000_raw_sensor_readings;
//
//TERRORS mpu6000Init( void );
//void mpu6000SetPower1( unsigned char );
//void mpu6000SetPower2( unsigned char );
//void mpu6000SetDLP( unsigned char );
//TERRORS mpu6000SetSampleRate( unsigned int );
//void mpu6000FifoOnOff( unsigned char );
//void mpu6000SetGyroRes( unsigned char );
//void mpu6000SetAccelRes( unsigned char );
//TERRORS mpu6000IntStatus( void );
//TERRORS read_sensors( void );
//unsigned int mpu6000FifoLen( void );
//unsigned char mpu6000ReadWriteByte( unsigned char, unsigned char );
//TERRORS mpu6000BurstRead( unsigned char, unsigned char *, unsigned char );

#endif /* MUP9150_H_ */