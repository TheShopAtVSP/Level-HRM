/*
 * manufacture_test.c
 *
 * Created: 6/28/2017 5:50:00 PM
 *  Author: mattwo
 */ 
 
#include "manufacture_test.h"
#include "mem_manager.h"
#include "battery.h"
#include "hal_imu.h"
#include "nrf_log.h"

T_MANUFACT_TEST_RESULTS test = { .state = TEST_NOT_STARTED, .hw_flags = 0 };
static bool test_req = false;

uint16_t get_hw_test_flags( void )
{
	return test.hw_flags;
}

T_MANUFACT_TEST_RESULTS get_hw_test_status( void )
{
	return test;
}

bool get_hw_test_request( void )
{
	return test_req;
}

void set_hw_test_request( void )
{
	if( test_req == false )
	{
		test.state = TEST_NOT_STARTED;
	}
	test_req = true;
}

void flag_hw_init_error( T_HW_FAILURE_FLAGS flag )
{
	if( flag != NO_FAILURE )
	{
		test.hw_flags |= flag;
		app_trace_log(DEBUG_MED, "[HW_ERR_FLAG] 0x%04X flagged\r", flag);
	}
}

void test_hardware( void )
{
	ret_code_t res;
	
	if( !manufacture_mode() || test_req == false ) 
	{
		test.state = TEST_NOT_STARTED;
		return;
	}

	test.state = TEST_IN_PROGRESS;

	//Battery test has several failure modes, so it returns the correct code for us
	test.hw_flags &= BAT_TEST_MASK;				//clear flags that are about to be tested for
	test.hw_flags |= battery_self_test(true);	//~100 ms to run

	//IMU test only has 1 failure mode:
	test.hw_flags &= IMU_TEST_MASK;				//clear flags that are about to be tested for
	res = test_imu();
	if( res != NRF_SUCCESS )					//~1500 ms to run
	{
		test.hw_flags |= IMU_SELF_TEST;
	}			

	app_trace_log(DEBUG_MED, "[TEST_HW] Flags 0x%04X\r", test.hw_flags);

	test_req = false;
	
	test.state = TEST_COMPLETE;
}
