#ifndef SI115X_FUNCTIONS_H_
#define SI115X_FUNCTIONS_H_

#ifndef int16_t
#include <stdint.h>
#endif

#ifndef HANDLE
typedef void *HANDLE;
#endif






/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/

/*******************************************************************************
 ***************   Functions Needed by Si115x_functions.c   ********************
 ******************************************************************************/

int16_t Si115xWriteToRegister(  uint8_t  address,
                                uint8_t  value);

int8_t 	Si115xReadFromRegister( uint8_t  address);

int16_t Si115xBlockWrite(       uint8_t  address,
                                uint8_t  length,
                                uint8_t* values);

int16_t Si115xBlockRead(        uint8_t  address,
                                uint8_t  length,
                                uint8_t* values);

// Delay function used by reset
void Si115xDelay_10ms(          void);
void SetIRLEDAnode (uint8_t led_switch);
/*******************************************************************************
 ***************   Functions supplied by Si115x_functions.c   ******************
 ******************************************************************************/
int16_t _waitUntilSleep(void);

void Si1153_twi_init (void);

int16_t Si1153_Init(void);

int16_t  Si115xReset(void);

int16_t  Si115xNop(             HANDLE  si115x_handle);

int16_t  Si115xForce(           HANDLE  si115x_handle);

int16_t  Si115xPause(           HANDLE  si115x_handle);

int16_t  Si115xStart(           HANDLE  si115x_handle);

int16_t  Si115xParamSet(        uint8_t address, 
                                uint8_t value);

int16_t  Si115xParamRead(       HANDLE  si115x_handle, 
                                uint8_t address);

uint8_t  SendCmd(               HANDLE  si115x_handle, 
                                uint8_t cmd );

uint8_t  QueryParam(            HANDLE  si115x_handle, 
                                uint8_t param_addr );

void     SetParam(              HANDLE si115x_handle, 
                                uint8_t param_addr, 
                                uint8_t param_value);

void 	hrm_read_cmd(void);

void Get_HRMs(void);

int8_t detect_peak(
        const int16_t*   	data, /* the data */ 
        int8_t             	data_count, /* row count of data */  
        int16_t          	delta, /* delta used for distinguishing peaks */
        int8_t             	emi_first /* should we search emission peak first of
                                     absorption peak first? */
        );
		
//int detect_peak(
//        const int16_t*   	data, /* the data */ 
//        int8_t             	data_count, /* row count of data */  
//        int8_t*            	num_emi_peaks, /* number of emission peaks found */ 
//        int8_t*            	num_absop_peaks, /* number of absorption peaks found */
//        int16_t          	delta, /* delta used for distinguishing peaks */
//        int8_t             	emi_first /* should we search emission peak first or absorption peak first? */
//        );

/*******************************************************************************
 ************************** Si115x I2C Registers *******************************
 ******************************************************************************/
#define SI115x_ADDRESS         (0x53) //0x53 0101 0011 
//
// I2C Registers
//
#define SI115x_REG_PART_ID      0x00
#define SI115x_REG_REV_ID       0x01
#define SI115x_REG_MFR_ID       0x02
#define SI115x_REG_INFO0        0x03
#define SI115x_REG_INFO1        0x04
#define SI115x_REG_HOSTIN3      0x07
#define SI115x_REG_HOSTIN2      0x08
#define SI115x_REG_HOSTIN1      0x09
#define SI115x_REG_HOSTIN0      0x0A
#define SI115x_REG_COMMAND      0x0B
#define SI115x_REG_IRQ_ENABLE   0x0F
#define SI115x_REG_RESPONSE1    0x10
#define SI115x_REG_RESPONSE0    0x11
#define SI115x_REG_IRQ_STATUS   0x12
#define SI115x_REG_HOSTOUT0     0x13
#define SI115x_REG_HOSTOUT1     0x14
#define SI115x_REG_HOSTOUT2     0x15
#define SI115x_REG_HOSTOUT3     0x16
#define SI115x_REG_HOSTOUT4     0x17
#define SI115x_REG_HOSTOUT5     0x18
#define SI115x_REG_HOSTOUT6     0x19
#define SI115x_REG_HOSTOUT7     0x1A
#define SI115x_REG_HOSTOUT8     0x1B
#define SI115x_REG_HOSTOUT9     0x1C
#define SI115x_REG_HOSTOUT10    0x1D
#define SI115x_REG_HOSTOUT11    0x1E
#define SI115x_REG_HOSTOUT12    0x1F
#define SI115x_REG_HOSTOUT13    0x20
#define SI115x_REG_HOSTOUT14    0x21
#define SI115x_REG_HOSTOUT15    0x22
#define SI115x_REG_HOSTOUT16    0x23
#define SI115x_REG_HOSTOUT17    0x24
#define SI115x_REG_HOSTOUT18    0x25
#define SI115x_REG_HOSTOUT19    0x26
#define SI115x_REG_HOSTOUT20    0x27
#define SI115x_REG_HOSTOUT21    0x28
#define SI115x_REG_HOSTOUT22    0x29
#define SI115x_REG_HOSTOUT23    0x2A
#define SI115x_REG_HOSTOUT24    0x2B
#define SI115x_REG_HOSTOUT25    0x2C
#define SI115x_REG_OTP_CONTROL  0x2F
#define SI115x_REG_CHIP_STAT    0x30

/*******************************************************************************
 ************************** Si115x I2C Parameter Offsets ***********************
 ******************************************************************************/
#define PARAM_I2C_ADDR          0x00
#define PARAM_CH_LIST           0x01
#define PARAM_ADCCONFIG0        0x02
#define PARAM_ADCSENS0          0x03
#define PARAM_ADCPOST0          0x04
#define PARAM_MEASCONFIG0       0x05
#define PARAM_ADCCONFIG1        0x06
#define PARAM_ADCSENS1          0x07
#define PARAM_ADCPOST1          0x08
#define PARAM_MEASCONFIG1       0x09
#define PARAM_ADCCONFIG2        0x0A
#define PARAM_ADCSENS2          0x0B
#define PARAM_ADCPOST2          0x0C
#define PARAM_MEASCONFIG2       0x0D
#define PARAM_ADCCONFIG3        0x0E
#define PARAM_ADCSENS3          0x0F
#define PARAM_ADCPOST3          0x10
#define PARAM_MEASCONFIG3       0x11
#define PARAM_ADCCONFIG4        0x12
#define PARAM_ADCSENS4          0x13
#define PARAM_ADCPOST4          0x14
#define PARAM_MEASCONFIG4       0x15
#define PARAM_ADCCONFIG5        0x16
#define PARAM_ADCSENS5          0x17
#define PARAM_ADCPOST5          0x18
#define PARAM_MEASCONFIG5       0x19
#define PARAM_MEASRATE_H        0x1A
#define PARAM_MEASRATE_L        0x1B
#define PARAM_MEASCOUNT0        0x1C
#define PARAM_MEASCOUNT1        0x1D
#define PARAM_MEASCOUNT2        0x1E
#define PARAM_LED1_A            0x1F
#define PARAM_LED1_B            0x20
#define PARAM_LED2_A            0x23
#define PARAM_LED2_B            0x24
#define PARAM_LED3_A            0x21
#define PARAM_LED3_B            0x22
#define PARAM_THRESHOLD0_H      0x25
#define PARAM_THRESHOLD0_L      0x26
#define PARAM_THRESHOLD1_H      0x27
#define PARAM_THRESHOLD1_L      0x28
#define PARAM_THRESHOLD2_H      0x29
#define PARAM_THRESHOLD2_L      0x2A
#define PARAM_BURST             0x2B

#define CMD_NOP                 0x00
#define CMD_RESET               0x01
#define CMD_NEW_ADDR            0x02
#define CMD_FORCE_CH            0x11
#define CMD_PAUSE_CH            0x12
#define CMD_AUTO_CH             0x13
#define CMD_PARAM_SET           0x80
#define CMD_PARAM_QUERY         0x40

/*******************************************************************************
 *******    Si115x Register and Parameter Bit Definitions  *********************
 ******************************************************************************/
#define RSP0_CHIPSTAT_MASK      0xe0
#define RSP0_COUNTER_MASK       0x1f
#define RSP0_SLEEP              0x20

#endif /* SI115X_FUNCTIONS_H_ */
