/*
 * manufacture_test.h
 *
 * Created: 6/28/2017 6:01:00 PM
 *  Author: mattwo
 */ 

#ifndef MANUFACTURE_TEST_H_
#define MANUFACTURE_TEST_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	NO_FAILURE =				0x0000,
	CHARGER_NOT_DETECT =		0x0001,
	BAT_MEASURE_CIRCUIT =		0x0002,
	BAT_MEASURE_SWITCH =		0x0004,
	BAT_NOT_INCLUDED =			0x0008,
	BAT_MANAGER_CHIP_FAULT = 	0x0010,
	BAT_MANAGER_CHIP_INIT =		0x0020,
	FLASH_CHIP_INIT =			0x0040,
	IMU_CHIP_INIT =				0x0080,
	IMU_SELF_TEST =				0x0100,
	FAILURE_FLAG_10 =			0x0200,
	FAILURE_FLAG_11 =			0x0400,
	FAILURE_FLAG_12 =			0x0800,
	FAILURE_FLAG_13 =			0x1000,
	FAILURE_FLAG_14 =			0x2000,
	FAILURE_FLAG_15 =			0x4000,
	FAILURE_FLAG_16 =			0x8000,
} T_HW_FAILURE_FLAGS;

//Masks to clear bits before Running tests (Note: all flags can be Cleared, INIT flags are only tested on startup.)
#define BAT_TEST_MASK			~(CHARGER_NOT_DETECT|BAT_MEASURE_CIRCUIT|BAT_MEASURE_SWITCH|BAT_NOT_INCLUDED|BAT_MANAGER_CHIP_FAULT)		//Battery Test can set any of the first 5 tests
#define IMU_TEST_MASK			~(IMU_SELF_TEST)

typedef enum {
    TEST_NOT_STARTED = 0,
	TEST_IN_PROGRESS,
	TEST_COMPLETE,
} T_TEST_STATES;

typedef struct {
	T_TEST_STATES state;
	uint16_t hw_flags;
} T_MANUFACT_TEST_RESULTS;
	
void flag_hw_init_error( T_HW_FAILURE_FLAGS flag );
uint16_t get_hw_test_flags( void );
T_MANUFACT_TEST_RESULTS get_hw_test_status( void );
bool get_hw_test_request( void );
void set_hw_test_request( void );
void test_hardware( void );
#endif /* MANUFACTURE_TEST_H_ */
