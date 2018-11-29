//-----------------------------------------------------------------------------
// drv_si115x.c
//-----------------------------------------------------------------------------
// WMM 12-17
//-----------------------------------------------------------------------------

#include "nrf.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "hal_twim.h"
#include "drv_si115x.h"

#define HRM_US_DELAY        500

//ADC Config Registers: (RES[7]=0, DEC[6:5], ADCMUX[4:0])
#define ADCFG_SML_IR		0x00
#define ADCFG_MED_IR		0x01
#define ADCFG_LRG_IR		0x02
#define ADCFG_MED_WHT		0x0B	//0b01011
#define ADCFG_LRG_WHT		0x0D	//0b01101
#define ADCFG_USEC24_4		0x60
#define ADCFG_USEC48_8		0x00
#define ADCFG_USEC97_6		0x20
#define ADCFG_USEC195		0x40
#define DEF_ADCFG			(ADCFG_USEC24_4 | ADCFG_LRG_IR)		

//ADC Sensitivity Registers: (HSIG[7], SW_GAIN[6:4], HW_GAIN[3:0])
#define ADSNS_HSIG			0x80
#define ADSNS_SW_GAIN_1		0x00
#define ADSNS_SW_GAIN_2		0x10
#define ADSNS_SW_GAIN_4		0x20
#define ADSNS_SW_GAIN_8		0x30
#define ADSNS_SW_GAIN_16	0x40
#define ADSNS_SW_GAIN_32	0x50
#define ADSNS_SW_GAIN_64	0x60
#define ADSNS_SW_GAIN_128	0x70
#define ADSNS_HW_GAIN_24_4U	0x00
#define ADSNS_HW_GAIN_48_8U	0x01
#define ADSNS_HW_GAIN_97_5U	0x02
#define ADSNS_HW_GAIN_195U	0x03
#define ADSNS_HW_GAIN_390U	0x04
#define ADSNS_HW_GAIN_780U	0x05
#define ADSNS_HW_GAIN_1_56M	0x06
#define ADSNS_HW_GAIN_3_12M	0x07
#define ADSNS_HW_GAIN_6_24M	0x08
#define ADSNS_HW_GAIN_12_5M	0x09
#define ADSNS_HW_GAIN_25_0M	0x0A
#define ADSNS_HW_GAIN_50_0M	0x0B
#define DEF_ADSNS			(ADSNS_SW_GAIN_4 | ADSNS_HW_GAIN_24_4U)	

//ADC Post Registers: (RES[7]=0, 24BIT[6], POSTSHIFT[5:3], NA[2], THR_EN[1:0])
#define ADPST_24BIT			0x40
#define ADPST_SHIFT_0		0x00
#define ADPST_SHIFT_1		0x08
#define ADPST_SHIFT_2		0x10
#define ADPST_SHIFT_3		0x18
#define ADPST_SHIFT_4		0x20
#define ADPST_SHIFT_5		0x28
#define ADPST_SHIFT_6		0x30
#define ADPST_SHIFT_7		0x38
#define ADPST_THR_OFF		0x00
#define ADPST_THR_EN0		0x01
#define ADPST_THR_EN1		0x02
#define ADPST_THR_EN2		0x03
#define DEF_ADPST			(ADPST_24BIT | ADPST_SHIFT_0 | ADPST_THR_OFF)	

// ADC Measure Config REgisters: (CNT_IND[7:6], LED_TRIM[5:4], BANK_SEL[3], LED_EN[2:0])
#define ADMSR_CNTR_NONE		0x00
#define ADMSR_CNTR0			0x40
#define ADMSR_CNTR1			0x80
#define ADMSR_CNTR2			0xC0
#define ADMSR_TRIM_NONE		0x00
#define ADMSR_TRIM_UP_9		0x20
#define ADMSR_TRIM_DN_10	0x30
#define ADMSR_BANK_A		0x00
#define ADMSR_BANK_B		0x08
#define ADMSR_LEDS_OFF		0x00
#define ADMSR_LED1_EN		0x01
#define ADMSR_LED2_EN		0x02	// led 2 (data sheet wrong) 
#define ADMSR_LED3_EN		0x04
#define DEF_ADMSR			(ADMSR_CNTR_NONE | ADMSR_TRIM_NONE | ADMSR_LEDS_OFF )

static const nrf_drv_twi_t m_twi_Si1153 = NRF_DRV_TWI_INSTANCE(0);
static uint8_t 	reg[2];

/* Si1153 initialization. */
int16_t Si1153_Init(void)
{	
	int16_t    retval;            

    retval  = Si115xReset(); 
    Si115xDelay_10ms();
	
	retval += Si115xParamSet( PARAM_CH_LIST, 0x03);    // Enable Channels 0 and 1
	
#define LEDX3
#ifdef LEDX3		
	retval += Si115xParamSet(PARAM_LED1_A, SI_CUR_22);
	retval += Si115xParamSet(PARAM_LED2_A, SI_CUR_5_5);
	retval += Si115xParamSet(PARAM_LED3_A, SI_CUR_22);
	retval += Si115xParamSet(PARAM_LED1_B, SI_CUR_22);
	retval += Si115xParamSet(PARAM_LED2_B, SI_CUR_5_5); 
	retval += Si115xParamSet(PARAM_LED3_B, SI_CUR_22);
#else
    retval += Si115xParamSet(PARAM_LED1_A, SI_CUR_50);
    retval += Si115xParamSet(PARAM_LED2_A, SI_CUR_100);
    ///retval += Si115xParamSet(PARAM_LED3_A, SI_CUR_354);
    retval += Si115xParamSet(PARAM_LED1_B, SI_CUR_50);
    retval += Si115xParamSet(PARAM_LED2_B, SI_CUR_100);      // 200mA is the max that Jason's LED B can take.
    ///retval += Si115xParamSet(PARAM_LED3_B, SI_CUR_354);
#endif

	//Config Channel 0
    retval += Si115xParamSet(PARAM_ADCCONFIG0, DEF_ADCFG);
    retval += Si115xParamSet(PARAM_ADCSENS0, DEF_ADSNS);
	retval += Si115xParamSet(PARAM_ADCPOST0, DEF_ADPST);
	retval += Si115xParamSet(PARAM_MEASCONFIG0, (DEF_ADMSR | ADMSR_BANK_A | ADMSR_LED1_EN));
    
	//Config Channel 1
    retval += Si115xParamSet(PARAM_ADCCONFIG1, DEF_ADCFG);
	retval += Si115xParamSet(PARAM_ADCSENS1, DEF_ADSNS);
    retval += Si115xParamSet(PARAM_ADCPOST1, DEF_ADPST);
	retval += Si115xParamSet(PARAM_MEASCONFIG1, (DEF_ADMSR | ADMSR_BANK_A | ADMSR_LED2_EN));


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
	nrf_delay_us(HRM_US_DELAY); 

	reg[0] = SI115x_REG_COMMAND;
	reg[1] = 0x80 + (address & 0x3F);
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS, reg, sizeof(reg), false); 	
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(HRM_US_DELAY);

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
	nrf_delay_us(HRM_US_DELAY);
	
	return err_code;
}

int8_t Si115xReadFromRegister(uint8_t  address)
{
	uint8_t rxbyte;
	ret_code_t err_code;
	
	
	
	reg[0] = address;
	err_code = nrf_drv_twi_tx(&m_twi_Si1153, SI115x_ADDRESS,  reg, 1, true);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(HRM_US_DELAY);
	err_code += nrf_drv_twi_rx(&m_twi_Si1153, SI115x_ADDRESS, &rxbyte, 1);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(HRM_US_DELAY);

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
	nrf_delay_us(HRM_US_DELAY);
	
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
	nrf_delay_us(HRM_US_DELAY);
  
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

