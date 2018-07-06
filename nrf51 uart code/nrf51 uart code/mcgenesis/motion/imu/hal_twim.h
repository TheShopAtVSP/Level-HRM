/*
 * hal_twim.h
 *
 * Created: 7/31/2014 9:55:54 AM
 *  Author: RichKl
 */ 


#ifndef HAL_TWIM_H_
#define HAL_TWIM_H_

#include <stdint.h>
#include "../global.h"
#include "sdk_errors.h"
#include "nrf_drv_twi.h"

#define TWIM_SCL_M				4   //!< Master SCL pin
#define TWIM_SDA_M				3   //!< Master SDA pin

#define MASTER_TWIM_INST		0    //!< TWIM interface used as a master accessing EEPROM memory


typedef struct {
	uint8_t				address;			// address of the Invense MPUxxx device
	uint32_t			speed;				// i2c speed
	uint32_t			interrupt_line;		// interrupt line
	uint32_t			interrupt_irq;		// interrupt
} T_IMU_PINS;

typedef struct {
	uint8_t i2c_id;
	uint8_t reg_addr;
	uint8_t length;
	uint8_t *buffer;
} T_TWIM_PACKET;

ret_code_t hal_twim_init( bool );
void hal_twim_uninit( void );
status_code_t hal_twim_write( T_TWIM_PACKET * );
status_code_t hal_twim_read( T_TWIM_PACKET * );

#endif /* HAL_TWIM_H_ */



