/** @file
 *
 * @defgroup Defines group of common functions that are globally useful
 * @{
 * @ingroup
 *
 */

#include "timing.h"
#include "app_util_platform.h"

// Real Time Clock Defines
//#define RTC						RTC2
//#define NRF_RTC 				CONCAT_2( NRF_, RTC )
//#define RTC_IRQHandler			CONCAT_2( RTC, _IRQHandler )
//#define RTC_IRQn				CONCAT_2( RTC, _IRQn )
#define RTC_REG_LEN				24
#define MAX_RTC_COUNTER_VAL     ((1<<RTC_REG_LEN) - 1)
#define RTC_CLK_FREQUENCY		(32768)

#define US_PER_HZ				(1000000UL)

static uint32_t unix_time = 0;
static uint32_t m_rtc_rollovers = 0;
static bool m_block_update = false;

/**@brief Function for handling the Real Time Clock interrupt.
 *
 * @details Tracks 1 second intervals to keep local Unix Time
 */
void RTC2_IRQHandler(void)
{
	if( NRF_RTC2->EVENTS_OVRFLW )
	{
		NRF_RTC2->EVENTS_OVRFLW = 0;
		m_rtc_rollovers++;
	}
	
	if( m_block_update )
	{	//To gaurantee glitchless operation, wait for Compare 1 Event after 
		//a set_unix_time() call.
		if( NRF_RTC2->EVENTS_COMPARE[1] )
		{
			NRF_RTC2->EVENTS_COMPARE[1] = 0;
			NRF_RTC2->EVENTS_COMPARE[0] = 0;
			
			//Compare[1] Events job is complete
			NRF_RTC2->EVTENCLR = RTC_EVTENCLR_COMPARE1_Msk;	
			NRF_RTC2->INTENCLR = RTC_INTENCLR_COMPARE1_Msk;
			
			m_block_update = false;
		}
	}
	else if( NRF_RTC2->EVENTS_COMPARE[0] )
	{
		unix_time += 1;
		NRF_RTC2->EVENTS_COMPARE[0] = 0;
		NRF_RTC2->CC[0] = ((NRF_RTC2->CC[0] + RTC_CLK_FREQUENCY)&MAX_RTC_COUNTER_VAL);	//set to interrupt in 1 second
	}
	else 
	{
		//nothing else right?
		
	}
}

/**@brief Function for starting the RTC timer that tracks unix time
 */
void unix_timer_init(void)
{
	NRF_RTC2->PRESCALER = 0;
    NVIC_SetPriority(RTC2_IRQn, APP_IRQ_PRIORITY_HIGH);	//lower priority interrupts may need accurate Unix Time
	
	NRF_RTC2->CC[0] = RTC_CLK_FREQUENCY;				//Use Compare 0 to generate 1 Second Ticks
	NRF_RTC2->EVTEN = RTC_EVTEN_COMPARE0_Msk | RTC_EVTEN_OVRFLW_Msk;
	NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC2->INTENCLR = RTC_INTENCLR_COMPARE1_Msk;	//Keep CC[1] Interrupt Off until needed
	NRF_RTC2->INTENSET = RTC_INTENSET_OVRFLW_Msk;	//Track rollovers to extended time reckoning
	
    NVIC_ClearPendingIRQ(RTC2_IRQn);
    NVIC_EnableIRQ(RTC2_IRQn);

    NRF_RTC2->TASKS_START = 1;
	nrf_delay_us(47);								//RTC may not start for the next 47 uSec
}

/**@brief Function for retrieving the current time in seconds.
 *
 * @param[out] uint32_t	Unix time (number of seconds since 1/1/1970)
 */
uint32_t get_unix_time( void )
{
	return unix_time;
}

/**@brief Function for setting the current time in seconds.
 *
 * @param[in] uint32_t	Unix time (number of seconds since 1/1/1970)
 */
void set_unix_time( uint32_t new_time )
{
	m_block_update = true;		//block unix_time from incrementing in the ISR
	
	//Set next Interrupt Out 1 Second
	NRF_RTC2->CC[0] = ((NRF_RTC2->COUNTER + RTC_CLK_FREQUENCY)&MAX_RTC_COUNTER_VAL);
	
	NRF_RTC2->CC[1] = ((NRF_RTC2->COUNTER + 5)&MAX_RTC_COUNTER_VAL); //Trigger to go Off in the near future
	NRF_RTC2->EVTENSET = RTC_EVTENSET_COMPARE1_Msk;	//Enable Event
	NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE1_Msk;	//Enable Interrupt
	
    unix_time = new_time;	//set new Unix Time base
}

uint32_t getSystemTimeMs( void )
{
	uint64_t time_ms = (m_rtc_rollovers<<RTC_REG_LEN) + NRF_RTC2->COUNTER;
	time_ms = (1000*time_ms)/RTC_CLK_FREQUENCY; 
	
	return time_ms;
}

/**@brief Function for retrieving the current number RTC ticks.
 *
 * @param[out] uint32_t	Number of Ticks since power on (or since last 32 bit rollover)
 *
 * @note with a 32 KHz clock, the 32 bit rollover will occur once every 36.4 hours.
 */
uint32_t get_rtc_cnt( void )
{
	uint32_t cnt = (m_rtc_rollovers<<RTC_REG_LEN) + NRF_RTC2->COUNTER;
	return cnt;
}

/**@brief Calculate future expiration time in tick counts (Max timer of 4,294.96 seconds ) 
 */
void get_expire_time( uint32_t wait_time_us, expire_timer_t * timer )
{	
	uint32_t stamp = get_rtc_cnt();		//get stamp first thing
	
	uint32_t wait_a_ticks = (uint64_t) RTC_CLK_FREQUENCY*wait_time_us/US_PER_HZ;
	timer->expiration = stamp + wait_a_ticks;
	timer->wait_a_ticks = wait_a_ticks;
}

/**@brief Cancal future expiration check
 */
void cancel_expire_time( expire_timer_t * timer )
{	
	timer->wait_a_ticks = 0;
}

/**@brief Check expiration time against current time
 */
bool check_expiration( expire_timer_t * timer )
{
	if( timer->wait_a_ticks > 0 )
	{
		uint32_t ticks_til_expire = ( timer->expiration - get_rtc_cnt() );	// = expiration - now
		
		if( ticks_til_expire > timer->wait_a_ticks )
		{	//expiration is in the past!!!
			return true;
		}
	}
    return false;
}

///
//! \fn
/// \brief
/// \param
/// \return .
///
//#define SQRT_ERR_REPORT 1
uint16_t fast_sqrt_32( uint32_t operand )
{
	uint16_t sqrt = 0;
	//uint16_t test_bit = 1 << (16 - (__CLZ(operand)>>1));	//better estimate of first bit
	uint16_t test_bit = 0x8000;
	uint32_t test_root;
	
	//Fast Integer Square Root
	do {
		test_root = sqrt|test_bit;
		if( (test_root*test_root) <= operand ) {
			sqrt |= test_bit;	//keep bit, it is part of the result
		}
		test_bit >>= 1;
	} while( test_bit != 0 );
	
	#if SQRT_ERR_REPORT
		if( (sqrt*sqrt) > operand )
		{
			APP_ERROR_HANDLER(NRF_ERROR_DATA_SIZE);
		}
	#endif
	
	return sqrt;
}
