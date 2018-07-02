//-----------------------------------------------------------------------------
// Si115x_functions.c
//-----------------------------------------------------------------------------
// WMM 12-17
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "hal_twim.h"
#include "led.h"
#include "si115x_functions.h"

#define	HRM_AVG_SAMPLES		4
#define HRM_DP_DATA_SIZE 	156
#define PD_DEBOUNCE_WIDTH 	7

extern uint8_t tx_payload[ ];
extern uint8_t current_hrm;
extern uint16_t hrm_ch1_avg, hrm_ch2_avg;
extern int16_t hrm_chan1_raw[], hrm_chan2_raw[], hrm_chan3_raw[], hrm_raw_index;
extern int16_t 	pd_emi_delta;
extern float silly_ratio;
/// extern uint16_t second_counter;
static const nrf_drv_twi_t m_twi_Si1153 = NRF_DRV_TWI_INSTANCE(0);
uint8_t 	reg[2], timer_count_out = 0;
uint16_t 	delay = 500;
uint32_t 	hrm_timer_start, hrm_current_time, hrm_add_time, hrm_run_time;
static int16_t		num_emi_peaks, num_absop_peaks, emi_peaks_xpos[HRM_DP_DATA_SIZE+1], absop_peaks_xpos[HRM_DP_DATA_SIZE+1];
static int16_t		emi_peaks[HRM_DP_DATA_SIZE+1], absop_peaks[HRM_DP_DATA_SIZE+1];
int8_t 		i, j, k, dh, old_num_of_emi_peaks, old_num_of_absop_peaks, pd_reduction_loop;
static int16_t 	avg_x_diff, avg_x_rng;
//static int16_t 	pd_absop_delta;
//static int16_t 	hrm_chan1_raw_avg[256], hrm_chan2_raw_avg[256];
double  	emi_peaks_dfp[128], emi_peaks_xpos_dfp[128], absop_peaks_dfp[128], absop_peaks_xpos_dfp[128], intgtr_gyr_p[156], intgtr_gyr_y[156];
///static int16_t		last4hrmavg[8], last4hrmavgidx;

/* Si1153 initialization. */
int16_t Si1153_Init(void)
{	
	int16_t    retval;            

    retval  = Si115xReset(); 
    Si115xDelay_10ms();
#define LEDX3
#ifdef LEDX3	
    retval += Si115xParamSet( PARAM_CH_LIST, 0x03); // Enable chan 1 and 2
	
	retval += Si115xParamSet(PARAM_LED1_A, 0x18);
	retval += Si115xParamSet(PARAM_LED2_A, 0x00);
	retval += Si115xParamSet(PARAM_LED3_A, 0x18);
	retval += Si115xParamSet(PARAM_LED1_B, 0x18);/// 50mA = 0x12
	retval += Si115xParamSet(PARAM_LED2_B, 0x00);/// 11mA = 0x08, 5.5 = 0x00, 22 = 0x18
	retval += Si115xParamSet(PARAM_LED3_B, 0x18);///2A = 100 mA
	
	retval += Si115xParamSet(PARAM_ADCCONFIG0, 0x62);	// mux 4xd 24.4 uSec
	retval += Si115xParamSet(PARAM_MEASCONFIG0, 0x01);//1 = LEDA 2 = LEDC 4 = LEDB
	retval += Si115xParamSet( PARAM_ADCSENS0, 0x20);//20
    retval += Si115xParamSet( PARAM_ADCPOST0, 0x40);// 24bit signed int
	
	retval += Si115xParamSet(PARAM_ADCCONFIG1, 0x62);	// mux 4xd 24.4 uSec
	retval += Si115xParamSet(PARAM_MEASCONFIG1, 0x02); //	0x02 LEDC// bank B led 2 (data sheet wong)  //6
    retval += Si115xParamSet( PARAM_ADCSENS1, 0x20); // HW/SW Gain deflt 0x20
	retval += Si115xParamSet( PARAM_ADCPOST1, 0x40);	// 24 bit
#else
retval += Si115xParamSet( PARAM_CH_LIST, 0x03); // Enable chan 1 and 2
	
	retval += Si115xParamSet(PARAM_LED1_A, 0x00);
	retval += Si115xParamSet(PARAM_LED2_A, 0x00);
	///retval += Si115xParamSet(PARAM_LED3_A, 0x3f);
	retval += Si115xParamSet(PARAM_LED1_B, 0x00);
	retval += Si115xParamSet(PARAM_LED2_B, 0x00);
	///retval += Si115xParamSet(PARAM_LED3_B, 0x3f);
	
	retval += Si115xParamSet(PARAM_ADCCONFIG0, 0x62);	// mux 4xd 24.4 uSec
	retval += Si115xParamSet(PARAM_MEASCONFIG0, 0x01);//1
	retval += Si115xParamSet( PARAM_ADCSENS0, 0x20);
    retval += Si115xParamSet( PARAM_ADCPOST0, 0x40);
	
	retval += Si115xParamSet(PARAM_ADCCONFIG1, 0x62);	// mux 4xd 24.4 uSec
	retval += Si115xParamSet(PARAM_MEASCONFIG1, 0x06); 	// bank B led 2 (data sheet wong)  //6
//    retval += Si115xParamSet( PARAM_ADCSENS1, 0x52);
	retval += Si115xParamSet( PARAM_ADCPOST1, 0x40);	// 24 bit
#endif

    return retval;	
}

// TWI events handler. 
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   

}

// TWI initialization. 
void Si1153_twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_Si1153_config = {
       .scl                = TWIM_SCL_M,
	   .sda                = TWIM_SDA_M,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_Si1153, &twi_Si1153_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_Si1153);
}


/***************************************************************************//**
 * @brief
 *   Waits until the Si115x is sleeping before proceeding
 ******************************************************************************/
static int16_t _waitUntilSleep(void)
{
  int16_t retval = -1;
  uint8_t count = 0;
  // This loops until the Si115x is known to be in its sleep state
  // or if an i2c error occurs
  while(count < 5)
  {
    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    if((retval&RSP0_CHIPSTAT_MASK) == RSP0_SLEEP)
      break;
    if(retval <  0)
      return retval;
    count++;
  }
  return 0;
}

/***************************************************************************//**
 * @brief
 *   Resets the Si115x/6x, clears any interrupts and initializes the HW_KEY
 *   register.
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xReset(void)
{
  int16_t retval = 0;

  // Do not access the Si115x earlier than 25 ms from power-up.
  // Uncomment the following lines if Si115xReset() is the first
  // instruction encountered, and if your system MCU boots up too
  // quickly.
  Si115xDelay_10ms();
  Si115xDelay_10ms();
  Si115xDelay_10ms();

  // Perform the Reset Command
  retval += Si115xWriteToRegister(SI115x_REG_COMMAND, 1);

  // Delay for 10 ms. This delay is needed to allow the Si115x
  // to perform internal reset sequence.
  Si115xDelay_10ms();

  return retval;
}

/***************************************************************************//**
 * @brief
 *   Helper function to send a command to the Si113x/4x
 ******************************************************************************/
static int16_t _sendCmd(HANDLE si115x_handle, uint8_t command)
{
  int16_t  response;
  int8_t   retval;
  uint8_t  count = 0;

  // Get the response register contents
  response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  if(response < 0)
  {
    return response;
  }

  response = response & RSP0_COUNTER_MASK;

  // Double-check the response register is consistent
  while(count < 5)
  {
    if((retval = _waitUntilSleep()) != 0)
      return retval;

    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);

    if((retval&RSP0_COUNTER_MASK) == response)
      break;
    else if(retval < 0)
      return retval;
    else
      response = retval & RSP0_COUNTER_MASK;

    count++;
  } // end loop

  // Send the Command
  if((retval == (Si115xWriteToRegister(SI115x_REG_COMMAND, command))
                != 0))
  {
    return retval;
  }

  count = 0;
  // Expect a change in the response register
  while(count < 5)
  {
    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    if((retval & RSP0_COUNTER_MASK) != response)
      break;
    else if(retval < 0)
      return retval;

    count++;
  } // end loop

  return 0;
}

/***************************************************************************//**
 * @brief
 *   Sends a NOP command to the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xNop(HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, 0x00);
}

/***************************************************************************//**
 * @brief
 *   Sends a FORCE command to the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xForce(HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, 0x11);
}

/***************************************************************************//**
 * @brief
 *   Sends a PSALSAUTO command to the Si113x/4x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xStart (HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, 0x13);
}

/***************************************************************************//**
 * @brief
 *   Reads a Parameter from the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] address
 *   The address of the parameter.
 * @retval <0
 *   Error
 * @retval 0-255
 *   Parameter contents
 ******************************************************************************/
int16_t Si115xParamRead(HANDLE si115x_handle, uint8_t address)
{
  // returns Parameter[address]
  int16_t retval;
  uint8_t cmd = 0x40 + (address & 0x3F);

  retval=_sendCmd(si115x_handle, cmd);
  if( retval != 0 )
  {
    return retval;
  }
  retval = Si115xReadFromRegister(SI115x_REG_RESPONSE1);
  return retval;
}

/***************************************************************************//**
 * @brief
 *   Writes a byte to an Si115x/6x Parameter
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] address
 *   The parameter address
 * @param[in] value
 *   The byte value to be written to the Si113x/4x parameter
 * @retval 0
 *   Success
 * @retval <0
 *   Error
 * @note This function ensures that the Si115x/6x is idle and ready to
 * receive a command before writing the parameter. Furthermore,
 * command completion is checked. If setting parameter is not done
 * properly, no measurements will occur. This is the most common
 * error. It is highly recommended that host code make use of this
 * function.
 ******************************************************************************/
int16_t Si115xParamSet(uint8_t address, uint8_t value)
{
  int16_t retval;
  //uint8_t buffer[2];
  int16_t response_stored;
  int16_t response;
  ret_code_t err_code;

  if((retval = _waitUntilSleep())!=0)
  {
    return retval;
  }

  response_stored = RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);

//  retval = Si115xWriteToRegister((0x80 + (address & 0x3F)), value);
	reg[0] = SI115x_REG_HOSTIN0;
	reg[1] = value;
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS, reg, sizeof(reg), false); 	
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay); 

	reg[0] = SI115x_REG_COMMAND;
	reg[1] = 0x80 + (address & 0x3F);
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS, reg, sizeof(reg), false); 	
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);

  // Wait for command to finish
  response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  while((response & RSP0_COUNTER_MASK) == response_stored)
  {
    response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  }

  if(retval < 0)
    return retval;
  else
    return 0;
}

/***************************************************************************//**
 * @brief
 *   Pause measurement helper function
 ******************************************************************************/
static int16_t _Pause (HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, 0x12);
}

/***************************************************************************//**
 * @brief
 *   Pauses autonomous measurements
 * @param[in] si115x_handle
 *  The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xPause(HANDLE si115x_handle)
{
  uint8_t countA, countB;
  int8_t  retval;

  //  After a RESET, if the Si115x receives a command (including NOP) before
  //  the Si115x has gone to sleep, the chip hangs. This first while loop
  //  avoids this.  The reading of the REG_RESPONS0 does not disturb
  //  the internal MCU.

  retval = 0; // initialize data so that we guarantee to enter the loop
  while((RSP0_CHIPSTAT_MASK & retval) != RSP0_SLEEP)
  {
    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  }

  countA = 0;
  while(countA < 5)
  {
    countB = 0;
    // Keep sending nops until the response is zero
    while(countB < 5)
    {
      retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
      if((retval & RSP0_COUNTER_MASK) == 0)
        break;
      else
      {
        // Send the NOP Command to clear any error...we cannot use
        // Si115xNop() because it first checks if REG_RESPONSE < 0 and
        // if so it does not perform the cmd. Since we have a saturation
        // REG_RESPONSE will be < 0
        Si115xWriteToRegister(SI115x_REG_COMMAND, 0x00);
      }
      countB++;
    } // end inner loop

    // Pause the device
    _Pause(si115x_handle);

    countB = 0;
    // Wait for response
    while(countB < 5)
    {
      retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
      if((retval & RSP0_COUNTER_MASK) != 0)
        break;
      countB++;
    }

    // When the PsAlsPause() response is good, we expect it to be a '1'.
    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    if((retval&RSP0_COUNTER_MASK) == 1 )
      break;  // otherwise, start over.
    countA++;
  } // end outer loop
  return 0;
}

/***************************************************************************//**
 * @brief
 *   Writes a byte to an Si115x/6x Parameter
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] param_addr
 *   The parameter address
 * @param[in] param_value
 *   The byte value to be written to the Si115x/6x parameter
 * @retval 0
 *   Success
 * @retval <0
 *   Error
 * @note This function ensures that the Si115x/6x is idle and ready to
 * receive a command before writing the parameter. Furthermore,
 * command completion is checked. If setting parameter is not done
 * properly, no measurements will occur. This is the most common
 * error. It is highly recommended that host code make use of this
 * function.
 ******************************************************************************/
void SetParam(HANDLE si115x_handle, uint8_t param_addr, uint8_t param_value)
{
  uint8_t temp;

  temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  while(!(temp & 0x20))
  {
    // wait for device to sleep
    temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  }
  if(temp & 0x10)
  {
    // if error code is present, NOP to clear
    Si115xWriteToRegister(SI115x_REG_COMMAND, CMD_NOP);
    while(temp & 0xDF)
    {
      // wait for device to sleep and clear
      temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    } // end loop
  } // end if

  Si115xWriteToRegister(
                        SI115x_REG_HOSTIN0,
                        param_value);
  Si115xWriteToRegister(
                        SI115x_REG_COMMAND,
                        CMD_PARAM_SET
                        | param_addr);
  while( (temp & 0x1f)
         == (Si115xReadFromRegister(SI115x_REG_RESPONSE0)
         & 0x1f) )
  {
    // Do Nothing 
    ;
  }
}

/***************************************************************************//**
 * @brief
 *   Reads a Parameter from the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] param_addr
 *   The address of the parameter.
 * @retval <0
 *   Error
 * @retval 0-255
 *   Parameter contents
 ******************************************************************************/
uint8_t QueryParam(HANDLE si115x_handle, uint8_t param_addr)
{
  uint8_t temp;

  temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  while(!(temp & 0x20))
  {
    // wait for device to sleep
    temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  }
  if(temp & 0x10)
  {
    // if error code is present, NOP to clear
    Si115xWriteToRegister(SI115x_REG_COMMAND, CMD_NOP);
    while(temp & 0xDF)
    {
      // wait for device to sleep and clear
      temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    }
  }
  Si115xWriteToRegister(SI115x_REG_COMMAND,
                        (CMD_PARAM_QUERY | param_addr));
  while((temp & 0x1f)
         == (Si115xReadFromRegister(SI115x_REG_RESPONSE0)
             & 0x1f))
  {
    ;
  }
  return Si115xReadFromRegister(SI115x_REG_HOSTOUT0);
}

/***************************************************************************//**
 * @brief
 *   Helper function to send a command to the Si113x/4x
 ******************************************************************************/
uint8_t SendCmd(HANDLE si115x_handle, uint8_t cmd)
{
  uint8_t temp;

  temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  while(!(temp & 0x20))
  {
    // wait for device to sleep
    temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  }
  if(temp & 0x10)
  {
    // if error code is present, NOP to clear
    Si115xWriteToRegister(SI115x_REG_COMMAND, CMD_NOP);
    while(temp & 0xDF)
    {
      // wait for device to sleep and clear
      temp = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    }
  }
  Si115xWriteToRegister(SI115x_REG_COMMAND, cmd);
  while((temp & 0x1f)
         == (Si115xReadFromRegister(SI115x_REG_RESPONSE0)
             & 0x1f))
  {
    ;
  }
  return Si115xReadFromRegister(SI115x_REG_RESPONSE0);
}

/*******************************************************************************
 ***************   Functions Needed by Si115x_functions.c   ********************
 ******************************************************************************/

int16_t Si115xWriteToRegister(uint8_t  address, uint8_t  value)
{
	ret_code_t err_code;
	
	reg[0] = address;
	reg[1] = value;
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS, reg, sizeof(reg), false); 	
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	
	return err_code;
}

int8_t Si115xReadFromRegister(uint8_t  address)
{
	uint8_t rxbyte;
	ret_code_t err_code;
	
	
	
	reg[0] = address;
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS,  reg, 1, true);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	err_code += nrf_drv_twi_rx(&m_twi_Si1153, SI115x_ADDRESS, &rxbyte, 1);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);

	///value[0] = (uint8_t)((err_code & 0xFF) | ((err_code >> 8) & 0xFF) | ((err_code >> 16) & 0xFF) | ((err_code >> 24) & 0xFF));
	///values[0] = (uint8_t)address;
	
	return rxbyte;
}

int16_t Si115xBlockWrite(uint8_t  address, uint8_t  length, uint8_t* values)
{
	ret_code_t err_code;
	int vloop;
	uint8_t bwdata[16];
	
	bwdata[0] = address;
	for(vloop=1; vloop <= length; vloop++)
	{
		bwdata[vloop] = values[vloop];
	}	
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS,  bwdata, length+1, true);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	
	return err_code;
}

int16_t Si115xBlockRead(uint8_t  address, uint8_t  length, uint8_t* values)
{
  int16_t retval;
  int16_t response;
  ret_code_t err_code;

  if((retval = _waitUntilSleep())!=0)
  {
    return retval;
  }

  response = RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);

	reg[0] = SI115x_REG_COMMAND;
	reg[1] = 0x40 + (address & 0x3F);
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS, reg, sizeof(reg), false); 	
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
  
	response = RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  
  
	response =  Si115xReadFromRegister(SI115x_REG_RESPONSE1);
					
	return response;				
}

// Delay function used by reset
void Si115xDelay_10ms(void)
{
	nrf_delay_ms(10);
}


/*==============================================*/
/* 				 Host Commands 					*/
/*==============================================*/

/*UART buffer size.*/
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

uint8_t reg[2];
extern uint8_t uart_rx_ubyte;
extern uint8_t uart_rev_buffer[1];

void SetIRLEDAnode (uint8_t led_switch)
{
	if(led_switch == 0) // Int IR LED
	{
		led(0,0,0,0,-1);
	}
	
	if(led_switch == 1) // Ext IR LED
	{
		led(1,1,1,-1,0);
	}
}

void hrm_read_cmd(void)
{		
	int time_out = 0;
	int t_o = 100;
	
	if(uart_rx_ubyte != 0x00)
	{	
		if(uart_rx_ubyte == 0x61) // a = internal led
		{
			SetIRLEDAnode(1);
		}
		if(uart_rx_ubyte == 0x62) // b = external led
		{
			SetIRLEDAnode(0);
		}
		
		if(uart_rx_ubyte == 0x63) // c = led enable chan1
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 0x63) && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_MEASCONFIG0, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 0x64) // d = led enable chan2
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 0x64) && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_MEASCONFIG1, uart_rx_ubyte);/// + 0x08);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 0x65) // e = current chan1
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 0x65) && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_LED1_A, uart_rx_ubyte);
				Si115xParamSet(PARAM_LED1_B, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 0x66) // f = current chan 2
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 0x66) && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_LED2_A, uart_rx_ubyte);
				Si115xParamSet(PARAM_LED2_B, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		
		if(uart_rx_ubyte == 0x67) // g = current chan 3
		{
			SetIRLEDAnode(1);
			while((uart_rx_ubyte == 0x67) && (time_out < t_o)){nrf_delay_ms(10); time_out++;}
			if(time_out < t_o)
			{
				Si115xParamSet(PARAM_LED3_A, uart_rx_ubyte);
				Si115xParamSet(PARAM_LED3_B, uart_rx_ubyte);
				nrf_delay_ms(100);
				SetIRLEDAnode(0);
			}
		}
		uart_rx_ubyte = 0x00;
	}
}

//========================================================================================================
/*
		HRM Post Processing 				
*/
//========================================================================================================

void Get_HRMs(void)
{
	int16_t 	trialidx, trialerrorcnt[4], emiandabsoppd[4], total_emiandabsops; 
	int16_t  	last_pvariance, pvariance;
	double  	phase2sf, phase2sfcof;
	int16_t		ser_input_size = 120, justanextraint16;
	
	// Added a three step loop that processes chan 1, then chan 2 then subtracted channel while counting the total added peaks and 
	// doubles. after all three have been processed the lower error count determines which emi + absop peaks are to be used as the HR.
	// trialidx 0-2, trialerrorcnt(0-2), trialhr0-2(emi+absop).
	// This allows the prdominate LED to win a fight and auto switches to subtract mode without using a threshold.
	// Later add a frame invalid and use the last current_hrm if errors are too high or if sub and result P2P is greater than xxx.
	// also trying: trialerrorcnt[] will be evaluated by  taking the starting (oldemiandabsoppd) pd and abs old - new
	// 				trialerrorcnt[trialidx] = abs(oldemiandabsoppd[trialidx] - (num_emi_peaks + num_absop_peaks)); 	
	for(trialidx = 0; trialidx < 3; trialidx++)	
	{	
		trialerrorcnt[trialidx] = 0;
		
		
		if(trialidx == 2) // do this one last so that we can skip it if trialerrorcnt(0 or 1) is low.
		{
			/// Auto Subtract ///
			
			for(i = 0; i < ser_input_size; i++)
			{
				hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * (silly_ratio)));
			}
			
			
//			phase2sfcof = 0.5;
//			pvariance = PeakDiff1D(hrm_chan1_raw, ser_input_size);
//			justanextraint16 = PeakDiff1D(hrm_chan2_raw, ser_input_size);
//			phase2sf = fabs((double)justanextraint16 / phase2sf);
//			
//			last_pvariance = 30000;
//			/// printf("%f\n",phase2sf);
//			for(int ol=0;  ol < 1; ol++)  /// 3
//			{
//				if(ol==0) /// 1
//					phase2sfcof = 0.05; /// 0.05
//				if(ol==1) /// 2
//					phase2sfcof = 0.001; /// 0.005
//					
//				last_pvariance = 30000;	 

//				while(abs(last_pvariance) > abs(pvariance))
//				{
//					last_pvariance = pvariance;
//					phase2sf += phase2sfcof;
//					for(i = 0; i < ser_input_size; i++)
//					{
//						hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * phase2sf));
//					}
//					pvariance = PeakDiff1D(hrm_chan3_raw, ser_input_size);
//				}
//				
//				last_pvariance = 30000;
//			
//				while(abs(last_pvariance) > abs(pvariance))
//				{
//					last_pvariance = pvariance;
//					phase2sf -= phase2sfcof;
//					for(i = 0; i < ser_input_size; i++)
//					{
//						hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * phase2sf));
//					}
//					pvariance = PeakDiff1D(hrm_chan3_raw, ser_input_size);
//				}
//			}
//			for(i = 0; i < ser_input_size; i++)
//			{
//				hrm_chan3_raw[i] = (int16_t)((double)hrm_chan1_raw[i] - ((double)hrm_chan2_raw[i] * (phase2sf + phase2sfcof)));
//			}
		}
		else  // do not do subtract and load chan 3 for next step. trialidx=0 will be chan1 and trialidx=1 will be chan2.
		{	 	
			for(i = 0; i < hrm_raw_index ; i++)
			{
				if(trialidx == 0)
					hrm_chan3_raw[i] = hrm_chan1_raw[i];
				if(trialidx == 1)
					hrm_chan3_raw[i] = hrm_chan2_raw[i];
			}
		}


#if 1
		// dH Threshold control
		/// Eliminate bounce by increasing delta (dh). Basicaly acts as a auto peak detector threshold control.
		/// It looks like starting from low to high sometimes eliminates wanted peaks. 
		/// If we start from high and look for a minimum number of peaks we could post process after substracting a proprtional
		/// amount to insure that we get some of the dis-qualified peaks. De bounce would have to then take the form of the missing peaks
		/// routine.  Adding more averaging may yeild a completly different result.
		
		pd_emi_delta = 35;
		do
		{	
			detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1);
			pd_emi_delta -= 2;
		}while((num_emi_peaks < 4) || (num_absop_peaks < 4));
		pd_emi_delta -= 2; // means delta - 2. should be porpotional...
		if(pd_emi_delta < 4)
			pd_emi_delta = 4;
		
//		pd_emi_delta = 3; // To play with going low to high.
//		do
//		{	
//			detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1);
//			pd_emi_delta += 2;
//		}while((num_emi_peaks > 5) || (num_absop_peaks > 5));
//		pd_emi_delta -= 2; // means delta - 2. should be porpotional...
//		if(pd_emi_delta > 35)
//			pd_emi_delta = 35;
		
		detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1); 
#endif	
		
   #if 0	
		/// Eliminate bounce by increasing delta (dh). Basicaly acts as a auto peak detector threshold control. 
		/// This is used with the Stitcher and works rather well, but is #ifdefed out for this  architecture.
		if(pd_emi_auto)
			pd_emi_delta = 4;
	//	sprintf(out_str, "Eliminate bounce1: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks);
	//	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
		do
		{	
			detect_peak(hrm_chan3_raw, hrm_raw_index, pd_emi_delta, 1);
			dh = 0;
			int8_t pd_reduction_loop = num_emi_peaks;
			if(pd_reduction_loop > num_absop_peaks)
				pd_reduction_loop = num_absop_peaks;
			for(i=0; i<(pd_reduction_loop - 1); i++)
			{
				if( ((emi_peaks_xpos[i+1] - emi_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) 
					|| ((absop_peaks_xpos[i+1] - absop_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) )
				{
					dh++;
				}
			}
			
			if(dh > 0)
			{
				pd_emi_delta++;	
			}
			
			if(dh == 25)
				dh = 0;
		}while(dh > 0);
	//	 sprintf(out_str, "Eliminate bounce2: Delta = %d, emip = %d, absopp = %d\n", pd_emi_delta, num_emi_peaks, num_absop_peaks); 
	//	 SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
#else
	///	detect_peak(hrm_chan3_raw, hrm_raw_index, 8, 1);
#endif	
		
#if 1		
		/// Missing pulse correction:
		// run thru the absop_peaks_xpos[num_absop_peaks[0]]. find delta x for each. avg delta x. if 1 is double the rest add 1 to num_absop_peaks[0].
		
		int8_t old_num_of_emi_peaks;
		int8_t old_num_of_absop_peaks;

		if(num_emi_peaks > 63)  // Don't diddle those kids.
			num_emi_peaks = 63;
		if(num_absop_peaks > 63)
			num_absop_peaks = 63;
		
		// Add phantom end points to emi and absop arrays. 150 is used for debug and not processed.
		emi_peaks_xpos[num_emi_peaks] = 135;
		emi_peaks[num_emi_peaks] = 2;
		num_emi_peaks++;
		emi_peaks_xpos[num_emi_peaks] = 150;
		emi_peaks[num_emi_peaks] = 2;
		num_emi_peaks++;
		for(i=num_emi_peaks; i>0; i--)      
		{
			emi_peaks_xpos[i] = emi_peaks_xpos[i - 1];
			emi_peaks[i] = emi_peaks[i - 1];
		}
		emi_peaks_xpos[0] =  -16;
		emi_peaks[0] = 2;
		absop_peaks_xpos[num_absop_peaks] = 135;
		absop_peaks[num_absop_peaks] = -2;
		num_absop_peaks++;
		absop_peaks_xpos[num_absop_peaks] = 150;
		absop_peaks[num_absop_peaks] = -2;
		num_absop_peaks++;
		for(i=num_absop_peaks; i>0; i--)      
		{
			absop_peaks_xpos[i] = absop_peaks_xpos[i - 1];
			absop_peaks[i] = absop_peaks[i - 1];
		}
		absop_peaks_xpos[0] =  -6;
		absop_peaks[0] = -2;
		
		//Start emi
		for(k=0; k<1; k++)
		{
			int16_t avg_x_diff = 0;
			old_num_of_emi_peaks = num_emi_peaks;
			
			if(old_num_of_emi_peaks > 0)
			{
				for(i=0; i<(old_num_of_emi_peaks - 1); i++) // Why limit x axis range???
				//for(i=0; i<(old_num_of_emi_peaks); i++)   
				{
					avg_x_diff += (emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]);   
				}
				avg_x_diff = avg_x_diff / old_num_of_emi_peaks;
				avg_x_rng = avg_x_diff + (avg_x_diff / 2);// + (avg_x_diff / 4);
			}
			
			for(i=0; i<=(old_num_of_emi_peaks /*- 1*/); i++)   /// 10 turned into 18 because avg x rng was too small or travelling shift.
			{
				if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) > avg_x_rng) 
				{
					//for(j=old_num_of_emi_peaks; j<=(i+1); j--)
					for(j=num_emi_peaks + 0; j>(i); j--)
					{
						emi_peaks_xpos[j] = emi_peaks_xpos[j - 1];
						emi_peaks[j] = emi_peaks[j - 1];
						 
					}
					//emi_peaks_xpos[i] = emi_peaks_xpos[i] + (avg_x_diff / (old_num_of_emi_peaks +1));
					emi_peaks_xpos[j+1] = emi_peaks_xpos[j] + (avg_x_diff); 
					emi_peaks[j+1] = 0;
					num_emi_peaks++;
					trialerrorcnt[trialidx]++;
				}
			}
		}
		
		// Get rid of the phantom data
		j = 0;
		for(i=0; i<(num_emi_peaks); i++)
		{	 
			if(emi_peaks_xpos[i] < 120)
			{
				emi_peaks_xpos[i] = emi_peaks_xpos[i+1];
				emi_peaks[i] = emi_peaks[i+1];
				j++;
			}
		}
		num_emi_peaks = j - 1;
		
		//Start absop
		for(k=0; k<1; k++)
		{	
			if(num_emi_peaks > 63)
			num_emi_peaks = 63;
			if(num_absop_peaks > 63)
			num_absop_peaks = 63;
			avg_x_diff = 0;
			old_num_of_absop_peaks = num_absop_peaks;
			
			if(old_num_of_absop_peaks > 0)
			{
				avg_x_diff = 0;
				for(i=0; i<(old_num_of_absop_peaks - 1); i++)
				//for(i=0; i<(old_num_of_absop_peaks); i++)
				{
					avg_x_diff += (absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]);	
				}
				avg_x_diff = avg_x_diff / old_num_of_absop_peaks;
				avg_x_rng = avg_x_diff + (avg_x_diff / 2);// + (avg_x_diff / 4);
			}
			
			for(i=0; i<(old_num_of_absop_peaks /*- 1*/); i++)
			{
				if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) > avg_x_rng)
				{
					
					for(j=num_absop_peaks; j>(i); j--)
					//for(j=old_num_of_absop_peaks + 1; j<=(i+1); j--) 
					{
						absop_peaks_xpos[j] = absop_peaks_xpos[j-1];
						absop_peaks[j] = absop_peaks[j-1];
					}
					//absop_peaks_xpos[i] = absop_peaks_xpos[i] + (avg_x_diff / (old_num_of_emi_peaks +1));
					absop_peaks_xpos[j+1] = absop_peaks_xpos[j] + (avg_x_diff);
					absop_peaks[j+1] = 0; 
					num_absop_peaks++;
					trialerrorcnt[trialidx]++;
				}
			}
			
			// Get rid of the phantom data
			j = 0;
			for(i=0; i<(num_absop_peaks); i++)
			{	 
				if(absop_peaks_xpos[i] < 120)
				{
					absop_peaks_xpos[i] = absop_peaks_xpos[i+1];
					absop_peaks[i] = absop_peaks[i+1];
					j++;
				}
			}
			num_absop_peaks = j - 1;
		}
#endif

#if 1	
		/// Eliminate Bounce.
		old_num_of_emi_peaks = num_emi_peaks;
		///	avg_x_rng = avg_x_rng/3; // might be changed to depend on the last HR * some multipl
		avg_x_rng = current_hrm / 8;   // 60hbpm @ 20sps => 10 for half cycle. 6 * 10 * 2 = 120.  60/10=6  = quarter cycle
		for(i=0; i<num_emi_peaks; i++)
		{
			if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) < avg_x_rng)
			{
				old_num_of_emi_peaks--;
				i=i+1;
				trialerrorcnt[trialidx]++;
//				sprintf(out_str, "Found extra emi: emi_peaks_xpos[i] %d\n", emi_peaks_xpos[i]);
//				SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
			}
		}
		num_emi_peaks = old_num_of_emi_peaks;
		
		old_num_of_absop_peaks = num_absop_peaks;
		for(i=0; i<num_absop_peaks; i++)
		{
			if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) < avg_x_rng)
			{
				old_num_of_absop_peaks--;
				i=i+1;
				trialerrorcnt[trialidx]++;
//				sprintf(out_str, "Found extra absop: absop_peaks_xpos[i] %d\n", absop_peaks_xpos[i]);
//				SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
			}
		}
		num_absop_peaks = old_num_of_absop_peaks;
		
//		sprintf(out_str, "Eliminate bounce final: Dh = %d, avg_x_rng = %d, emip = %d, absopp = %d\n", pd_emi_delta, avg_x_rng, num_emi_peaks, num_absop_peaks); 
//		SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);

		///	detect_peak(hrm_chan3_raw, hrm_raw_index, 8, 1);
#endif	
		emiandabsoppd[trialidx] = num_emi_peaks + num_absop_peaks;	
	} // end of trial loop 

	///============== Post processing =====================///

	/// Mode Selector
	int dp_mode = 0;

	//if((emiandabsoppd[0] >= emiandabsoppd[1]) && (emiandabsoppd[0] <= emiandabsoppd[2]))
	//	dp_mode = 0;
	//else if((emiandabsoppd[1] >= emiandabsoppd[0]) && (emiandabsoppd[1] <= emiandabsoppd[2]))
	//	dp_mode = 1;
	//else if((emiandabsoppd[2] >= emiandabsoppd[0]) && (emiandabsoppd[2] <= emiandabsoppd[1]))
	//	dp_mode = 2;
	//if((emiandabsoppd[0] >= emiandabsoppd[1]) && (emiandabsoppd[0] <= emiandabsoppd[2]))
	
	if(((emiandabsoppd[0] >= emiandabsoppd[1]) && (emiandabsoppd[0] <= emiandabsoppd[2])) || ((emiandabsoppd[0] <= emiandabsoppd[1]) && (emiandabsoppd[0] >= emiandabsoppd[2])))
		dp_mode = 0;
	//else if((emiandabsoppd[1] >= emiandabsoppd[0]) && (emiandabsoppd[1] <= emiandabsoppd[2]))
	else if(((emiandabsoppd[1] >= emiandabsoppd[0]) && (emiandabsoppd[1] <= emiandabsoppd[2])) || ((emiandabsoppd[1] <= emiandabsoppd[0]) && (emiandabsoppd[1] >= emiandabsoppd[2])))
		dp_mode = 1;
	else if(((emiandabsoppd[2] >= emiandabsoppd[0]) && (emiandabsoppd[2] <= emiandabsoppd[1])) || ((emiandabsoppd[2] <= emiandabsoppd[0]) && (emiandabsoppd[2] >= emiandabsoppd[1])))
		dp_mode = 2;

	//if((trialerrorcnt[0] <= trialerrorcnt[1]) && (trialerrorcnt[0] <= trialerrorcnt[2]))
	//	dp_mode = 0;
	//else if((trialerrorcnt[1] <= trialerrorcnt[0]) && (trialerrorcnt[1] <= trialerrorcnt[2]))
	//	dp_mode = 1;
	//else if((trialerrorcnt[2] <= trialerrorcnt[0]) && (trialerrorcnt[2] <= trialerrorcnt[1]))
	//	dp_mode = 2;
	//if(pvariance > 200)
	//	dp_mode = 3;

	total_emiandabsops = emiandabsoppd[dp_mode];
	//total_emiandabsops = (((double)(emiandabsoppd[0] * 5) + (double)(emiandabsoppd[1] * 5) + (double)(emiandabsoppd[2] * 5)) / 3);
	emiandabsoppd[3] = emiandabsoppd[dp_mode]; // save for skip frame //

    /// Average the hrm output.
	current_hrm = ((total_emiandabsops * 5) + (current_hrm * 3)) / 4;
}



/*========================================================================================================//

void Get_HRMs(void)
{
	/// CZ + MM => avg 4, delta 5, no width skip, stitching on, forward filter off. 
	int8_t 		i, j;
	int32_t		hrm_chan1_running_avg_accm, hrm_chan2_running_avg_accm;

#if 0
	/// Bias subtraction. ///
	for(i = 0; i < hrm_raw_index; i++)
	{
		hrm_chan1_raw[i] -= hrm_chan2_raw[i];
	}
#endif
	
	/// Average.
#if 1
	for(i = 0; i < (hrm_raw_index - HRM_AVG_SAMPLES); i++)
	{
		hrm_chan1_running_avg_accm = hrm_chan2_running_avg_accm = 0;
		for(j = 0; j < HRM_AVG_SAMPLES; j++)
		{
			hrm_chan1_running_avg_accm += hrm_chan1_raw[i+j];
			hrm_chan2_running_avg_accm += hrm_chan2_raw[i+j];
		}
		hrm_chan1_raw[i] = (int16_t)(hrm_chan1_running_avg_accm / HRM_AVG_SAMPLES);
		hrm_chan2_raw[i] = (int16_t)(hrm_chan2_running_avg_accm / HRM_AVG_SAMPLES);
	}
	
	/// Add nomalization here. ///
#endif
#if 1
	for(i = (hrm_raw_index - HRM_AVG_SAMPLES); i > (hrm_raw_index - HRM_AVG_SAMPLES - HRM_AVG_SAMPLES); i--)
	{
		hrm_chan1_running_avg_accm = hrm_chan2_running_avg_accm = 0;
		for(j = 0; j < HRM_AVG_SAMPLES; j++)
		{
			hrm_chan1_running_avg_accm += hrm_chan1_raw[i+j];
			hrm_chan2_running_avg_accm += hrm_chan2_raw[i+j];
		}
		hrm_chan1_raw[i] = (int16_t)(hrm_chan1_running_avg_accm / HRM_AVG_SAMPLES);
		hrm_chan2_raw[i] = (int16_t)(hrm_chan2_running_avg_accm / HRM_AVG_SAMPLES);
	}
	
	/// Add nomalization here. ///
#endif
				
	
#if 1
	/// Bias subtraction. ///
	for(i = 0; i < hrm_raw_index; i++)
	{
		hrm_chan1_raw[i] -= hrm_chan2_raw[i];
	}
#endif
	

#if 1	
	/// Eliminate bounce by increaseing delta (dh).
	pd_emi_delta = 4;
	do
	{	
		detect_peak(hrm_chan1_raw, hrm_raw_index, pd_emi_delta, 1);
		dh = 0;
		pd_reduction_loop = num_emi_peaks;
		if(pd_reduction_loop > num_absop_peaks)
			pd_reduction_loop = num_absop_peaks;
		for(i=0; i<(pd_reduction_loop - 1); i++)
		{
			if( ((emi_peaks_xpos[i+1] - emi_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) 
				|| ((absop_peaks_xpos[i+1] - absop_peaks_xpos[i]) < PD_DEBOUNCE_WIDTH) )
			{
				dh++;
			}
		}
		
		if(dh > 0)
		{
			pd_emi_delta++;	
		}
		
		if(dh == 25)
			dh = 0;
	}while(dh > 0);
#else
	detect_peak(hrm_chan1_raw, hrm_raw_index, 8, 1);
#endif	

#if 1		
	/// Missing pulse correction:
	// run thru the absop_peaks_xpos[num_absop_peaks[0]]. find delta x for each. avg delta x. if 1 is double the rest add 1 to num_absop_peaks[0].
	
	old_num_of_emi_peaks = num_emi_peaks;
	old_num_of_absop_peaks = num_absop_peaks;
	for(k=0; k<1; k++)
	{
		avg_x_diff = 0;
		for(i=0; i<(old_num_of_emi_peaks - 1); i++)
		{
			avg_x_diff += (emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]);	
		}
		avg_x_diff = avg_x_diff / old_num_of_emi_peaks;
		avg_x_rng = avg_x_diff + (avg_x_diff / 2) + (avg_x_diff / 4);
	
		for(i=0; i<(old_num_of_emi_peaks - 1); i++)
		{
			if((emi_peaks_xpos[i + 1] - emi_peaks_xpos[i]) > avg_x_rng) 
			{
				num_emi_peaks++;
				for(j=old_num_of_emi_peaks; j<=(i+1); j--)
				{
					emi_peaks_xpos[j] = emi_peaks_xpos[j-1];
				}
				emi_peaks_xpos[i] = emi_peaks_xpos[i] + avg_x_diff;
			}
		}
	
		avg_x_diff = 0;
		for(i=0; i<(old_num_of_absop_peaks - 1); i++)
		{
			avg_x_diff += (absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]);	
		}
		avg_x_diff = avg_x_diff / old_num_of_absop_peaks;
		avg_x_rng = avg_x_diff + (avg_x_diff / 2) + (avg_x_diff / 4);
	
		for(i=0; i<(old_num_of_absop_peaks - 1); i++)
		{
			if((absop_peaks_xpos[i + 1] - absop_peaks_xpos[i]) > avg_x_rng)
			{
				num_absop_peaks++;
				for(j=old_num_of_absop_peaks; j<=(i+1); j--)
				{
					absop_peaks_xpos[j] = absop_peaks_xpos[j-1];
				}
				absop_peaks_xpos[i] = absop_peaks_xpos[i] + avg_x_diff;
			}
		}
	}
#endif
	
    /// Average the hrm output.
	current_hrm = ((num_emi_peaks * 10) + (current_hrm * 10) + (num_absop_peaks * 10)) / 12;
///	current_hrm = ((num_emi_peaks * 10) + (current_hrm * 4) + (num_absop_peaks * 10)) / 6;
	//current_hrm = ((num_emi_peaks * 10) + (current_hrm * 3) + (num_absop_peaks * 0)) / 4;
}
/// ========================================================================================== */

int8_t detect_peak(
					const int16_t*   	data, 		/* the data */ 
					int8_t             	data_count, /* row count of data */  
					int16_t          	delta, 		/* delta used for distinguishing peaks */
					int8_t             	emi_first 	/* should we search emission peak first of absorption peak first? */
					)
{
    int     i;
    double  mx;
    double  mn;
    int     mx_pos = 0;
    int     mn_pos = 0;
    int     is_detecting_emi = emi_first;


    mx = data[0];
    mn = data[0];

    num_emi_peaks = 0;
    num_absop_peaks = 0;

    for(i = 1; i < data_count; ++i)
    {
        if(data[i] > mx)
        {
            mx_pos = i;
            mx = data[i];
        }
        if(data[i] < mn)
        {
            mn_pos = i;
            mn = data[i];
        }

        if(is_detecting_emi &&
                data[i] < mx - delta)
        {
            if(num_emi_peaks >= HRM_DP_DATA_SIZE) /* not enough spaces */
                return 1;

            emi_peaks[num_emi_peaks] = mx; //mx_pos
			emi_peaks_xpos[num_emi_peaks] = mx_pos;
			num_emi_peaks++;
			
            is_detecting_emi = 0;

            i = mx_pos - 1;

            mn = data[mx_pos];
            mn_pos = mx_pos;
        }
        else if((!is_detecting_emi) &&
                data[i] > mn + delta)
        {
            if(num_absop_peaks >= HRM_DP_DATA_SIZE)
                return 2;
			
			absop_peaks_xpos[num_absop_peaks] = mn_pos;
            absop_peaks[num_absop_peaks] = mn;
            num_absop_peaks++;

            is_detecting_emi = 1;
            
            i = mn_pos - 1;

            mx = data[mn_pos];
            mx_pos = mn_pos;
        }
    }

    return 0;
}


int16_t PeakDiff1D(const int16_t *arr, size_t length) 
{
    // returns the maximum value of array
    size_t i;
    int16_t maximum = arr[0];
	int16_t minimum = arr[0];
	int16_t peakdiff;
    for (i = 1; i < length; ++i) 
	{
        if (maximum < arr[i]) {
            maximum = arr[i];
        }
		if (minimum > arr[i]) {
            minimum = arr[i];
        }
    }
	peakdiff = maximum - minimum;
    return peakdiff;
}


