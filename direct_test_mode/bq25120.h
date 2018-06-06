/*
 * bq25120.h
 *
 * Created: 6/28/2017 11:07:31 PM
 *  Author: mattwo
 */ 

#ifndef BQ25120_H_
#define BQ25120_H_

#include <stdbool.h>
#include "sdk_errors.h"

typedef enum {
	BQ_READ_FAIL = 	0,
	BQ_READY =		1,
	BQ_CHARGING =	2,
	BQ_CHARGED =	3,
	BQ_FAULT =		4,
} T_BQ25120_STATUS;

ret_code_t bq25120_init( void );
ret_code_t bq25120_en_shipmode( void );
bool bq25120_i2c_wakeup( void );
void bq25120_charge_disable( bool terminate );
T_BQ25120_STATUS bq25120_read_status(void);

#endif /* BQ25120_H_ */
