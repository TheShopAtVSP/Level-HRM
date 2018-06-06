//-----------------------------------------------------------------------------
// Si115x_functions.c
//-----------------------------------------------------------------------------
// WMM 12-17
//-----------------------------------------------------------------------------
#include <stdio.h>
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

//#define LEDX3

static const nrf_drv_twi_t m_twi_Si1153 = NRF_DRV_TWI_INSTANCE(0);
uint8_t reg[2];
int delay = 500;

/* Si1153 initialization. */
int16_t Si1153_Init(void)
{	
	int16_t    retval;            

    retval  = Si115xReset(); 
    Si115xDelay_10ms();

#ifdef LEDX3	
    retval += Si115xParamSet( PARAM_CH_LIST, 0x03); // Enable chan 1 and 2
	
	retval += Si115xParamSet(PARAM_LED1_A, 0x3f);
	retval += Si115xParamSet(PARAM_LED2_A, 0x3f);
	retval += Si115xParamSet(PARAM_LED3_A, 0x3f);
	retval += Si115xParamSet(PARAM_LED1_B, 0x3f);
	retval += Si115xParamSet(PARAM_LED2_B, 0x3f);
	retval += Si115xParamSet(PARAM_LED3_B, 0x3f);
	
	retval += Si115xParamSet(PARAM_ADCCONFIG0, 0x62);	// mux 4xd 24.4 uSec
	retval += Si115xParamSet(PARAM_MEASCONFIG0, 0x02);//1
	retval += Si115xParamSet( PARAM_ADCSENS0, 0x20);
    retval += Si115xParamSet( PARAM_ADCPOST0, 0x40);
	
	retval += Si115xParamSet(PARAM_ADCCONFIG1, 0x62);	// mux 4xd 24.4 uSec
	retval += Si115xParamSet(PARAM_MEASCONFIG1, 0x04); 	// bank B led 2 (data sheet wong)  //6
//    retval += Si115xParamSet( PARAM_ADCSENS1, 0x52);
	retval += Si115xParamSet( PARAM_ADCPOST1, 0x40);	// 24 bit
#else
retval += Si115xParamSet( PARAM_CH_LIST, 0x03); // Enable chan 1 and 2
	
	retval += Si115xParamSet(PARAM_LED1_A, 0x3f);
	retval += Si115xParamSet(PARAM_LED2_A, 0x3f);
	///retval += Si115xParamSet(PARAM_LED3_A, 0x3f);
	retval += Si115xParamSet(PARAM_LED1_B, 0x3f);
	retval += Si115xParamSet(PARAM_LED2_B, 0x3f);
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
	///      SetIRLEDAnode MEASRATE_H _L ro 24 for 1/52=19.2mSec * 800 uSec...
	///		 OR take 24 measurments and shift to impliment averaging using the ADCSENSx and ADCPOSTx regs.
	
	
//	retval += Si115xParamSet(PARAM_ADCCONFIG2, 0x62);
//	retval += Si115xParamSet(PARAM_MEASCONFIG2, 0x04);
//    retval += Si115xParamSet( PARAM_ADCCONFIG0, 0x62);//78 
//    retval += Si115xParamSet( PARAM_ADCSENS0, 0x09);//09
//    retval += Si115xParamSet( PARAM_ADCPOST0, 0x40);
//    retval += Si115xParamSet( PARAM_ADCCONFIG1, 0x4d);
//    retval += Si115xParamSet( PARAM_ADCSENS1, 0x52);	
//    retval += Si115xParamSet( PARAM_ADCPOST1, 0x40);	
//    retval += Si115xParamSet( PARAM_ADCCONFIG2, 0x41);	
//    retval += Si115xParamSet( PARAM_ADCSENS2, 0x61);
//    retval += Si115xParamSet( PARAM_ADCPOST2, 0x50);
//    retval += Si115xParamSet( PARAM_ADCCONFIG3, 0x4d);
//    retval += Si115xParamSet( PARAM_ADCSENS3, 0x07);
//    retval += Si115xParamSet( PARAM_ADCPOST3, 0x40);
//    retval += Si115xWriteToRegister( SI115x_REG_IRQ_ENABLE, 0x07);
/*	
	retval += Si115xParamSet( PARAM_CH_LIST, 0x0f);
	return retval;
	
	
if((retval = _waitUntilSleep())!=0)
  {
    return 0x44;//retval;
  }	
response_stored = RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);
//  if(response_stored != 0x00FF)
//	return 0x45;//response_stored; 

buffer[0] = 0x0f;//value;
buffer[1] = 0x80 + (PARAM_CH_LIST & 0x3F);
retval = Si115xBlockWrite(SI115x_REG_HOSTIN0, 2, (uint8_t*) buffer);  
 // Wait for command to finish
response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
return response;
*/

    return retval;	
}

// TWI events handler. 
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
//    ret_code_t err_code;
//    static sample_t m_sample;
//    
//    switch(p_event->type)
//    {
//        case NRF_DRV_TWI_EVT_DONE:
//            if ((p_event->type == NRF_DRV_TWI_EVT_DONE) &&
//                (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX))
//            {
//                if(m_set_mode_done != true)
//                {
//                    m_set_mode_done  = true;
//                    return;
//                }
//                m_xfer_done = false;
//                // Read 4 bytes from the specified address. 
//                ///err_code = nrf_drv_twi_rx(&m_twi_VCNL4020, REGISTER_PROX_VALUE, (uint8_t*)&m_sample, sizeof(m_sample));
//				err_code = nrf_drv_twi_rx(&m_twi_VCNL4020, REGISTER_PROX_VALUE, (uint8_t*)&m_sample, 1);
//                APP_ERROR_CHECK(err_code);
//            }
//            else
//            {
//                read_data(&m_sample);
//                m_xfer_done = true;
//            }
//            break;
//        default:
//            break;        
//    }   
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

/* GPIO initialization.
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

////////////////////////////////    err_code = nrf_drv_gpiote_out_init(IRINT_PIN, &out_config);
////////////////////////////////    APP_ERROR_CHECK(err_code);
////////////////////////////////	
////////////////////////////////	err_code = nrf_drv_gpiote_out_init(IREXT_PIN, &out_config);
////////////////////////////////    APP_ERROR_CHECK(err_code);
	
//	err_code = nrf_drv_gpiote_out_init(LED_1, &out_config);
//    APP_ERROR_CHECK(err_code);
//	
//	err_code = nrf_drv_gpiote_out_init(LED_2, &out_config);
//    APP_ERROR_CHECK(err_code);
//	
	SetIRLEDAnode (0);

//    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
//    in_config.pull = NRF_GPIO_PIN_PULLUP;
//    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
//    APP_ERROR_CHECK(err_code);
//    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}
*/

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
  
//  buffer[0] = value;
//  buffer[1] = 0x80 + (address & 0x3F);
//  retval = Si115xBlockWrite(SI115x_REG_HOSTIN0,
//                            2,
//                            (uint8_t*) buffer);
//  if(retval != 0)
//    return retval;

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
             & 0x1f));
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
//buffer[0] = 0x0f;//value;
//  buffer[1] = 0x80 + (PARAM_CH_LIST & 0x3F);
//  retval = Si115xBlockWrite(SI115x_REG_HOSTIN0,
//                            2,
//                            (uint8_t*) buffer);
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
//	ret_code_t err_code;
//	int16_t Si115xParamSet(uint8_t address, uint8_t value)
//{
  int16_t retval;
  //uint8_t buffer[2];
  //int16_t response_stored;
  int16_t response;
  ret_code_t err_code;

  if((retval = _waitUntilSleep())!=0)
  {
    return retval;
  }

  response = RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);

//  retval = Si115xWriteToRegister((0x80 + (address & 0x3F)), value);
//	reg[0] = SI115x_REG_HOSTIN0;
//	reg[1] = value;
//	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS, reg, sizeof(reg), false); 	
//	APP_ERROR_CHECK(err_code);
//	nrf_delay_us(delay); 

	reg[0] = SI115x_REG_COMMAND;
	reg[1] = 0x40 + (address & 0x3F);
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS, reg, sizeof(reg), false); 	
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
  
	response = RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  
  
	response =  Si115xReadFromRegister(SI115x_REG_RESPONSE1);
					
	return response;				
//  buffer[0] = value;
//  buffer[1] = 0x80 + (address & 0x3F);
//  retval = Si115xBlockWrite(SI115x_REG_HOSTIN0,
//                            2,
//                            (uint8_t*) buffer);
//  if(retval != 0)
//    return retval;

  // Wait for command to finish
//  response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
//  while((response & RSP0_COUNTER_MASK) == response_stored)
//  {
//    response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
//  }

//  if(retval < 0)
//    return retval;
//  else
//    return 0;
//}

	
//	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS,  &address, 1, true);
//	APP_ERROR_CHECK(err_code);
//	nrf_delay_us(delay);
//	err_code = nrf_drv_twi_rx(&m_twi_Si1153, SI115x_ADDRESS, values, length);
//	APP_ERROR_CHECK(err_code);
//	nrf_delay_us(delay);

	///values[0] = (uint8_t)((err_code & 0xFF) | (err_code >> 8) | (err_code >> 16) | (err_code >> 24));
	///values[0] = (uint8_t)address;
	//return err_code;
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

/* Indicates if reading operation from 4020 has ended. */
//static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
//static volatile bool m_set_mode_done = false;
uint8_t reg[2];
extern uint8_t uart_rx_ubyte;
extern uint8_t uart_rev_buffer[1];

/*
// UART events handler. 
static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


// UART initialization. 
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
                       uart_events_handler, APP_IRQ_PRIORITY_LOW, err_code);
    APP_ERROR_CHECK(err_code);
}
*/

void SetInterruptControl (unsigned char Command)
{
//	ret_code_t err_code;
//	
//	reg[0] = REGISTER_INTERRUPT_CONTROL;
//	reg[1] = Command;
//	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS, reg, sizeof(reg), false); 	
//    APP_ERROR_CHECK(err_code);
//	nrf_delay_us(delay);
}


uint16_t ReadProxiValue (void)
{
	uint16_t ret_distance = 0;
//	uint8_t  m_sample[2];
//	ret_code_t err_code;
//	
//	reg[0] = REGISTER_PROX_VALUE;
//	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS,  reg, 1, true);
//	APP_ERROR_CHECK(err_code);
//	nrf_delay_us(delay);
//	err_code = nrf_drv_twi_rx(&m_twi_VCNL4020, VCNL4020_ADDRESS, m_sample, 2);
//	APP_ERROR_CHECK(err_code);
//	nrf_delay_us(delay);
//	ret_distance = (uint16_t)m_sample[0] << 8 | (uint8_t)m_sample[1];

	return ret_distance;
}

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
