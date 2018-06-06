/* VCNL4020HRMmain.c 
 * BIGLY Test code for our new 
 * SUPERDUPER FANTATIC INCREADABLE TREMENDOUS YUGE HRM
 
 i = internal led
 e = external led
 b = both
 c = current followed by 0 thru 20 = 0 to 200 mA
 r = Rate followed by 1 thru 7: 1=4 2=8 3=16 4=31 5=62 6=125 7=250 sps
 
 */

#include <stdio.h>
#include "boards.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "VCNL4020HRM.h"

/*Pins*/
#define I2C_SCL_PIN 7
#define I2C_SDA_PIN 30
//#define IRINT_PIN	LED_2
//#define IREXT_PIN	LED_1
#define IRINT_PIN	23
#define IREXT_PIN	22

/*UART buffer size.*/
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

/* Indicates if reading operation from 4020 has ended. */
//static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
//static volatile bool m_set_mode_done = false;
uint8_t reg[2];
int delay = 350;

/* TWI instance. */
static const nrf_drv_twi_t m_twi_VCNL4020 = NRF_DRV_TWI_INSTANCE(0);

/* UART events handler. */
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


/* UART initialization. */
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


/* VCNL4020 initialization. */
void VCNL4020_Init(void)
{
	//VCNL4020 Command Reg.  
	SetCommandRegister (COMMAND_ALL_DISABLE);
	
	//VCNL4020 Prox Rate Reg. 
	SetProximityRate (PROX_MEASUREMENT_RATE_250);
	
    //VCNL4020 Command Reg.  
	SetCommandRegister (COMMAND_PROX_ENABLE | COMMAND_SELFTIMED_MODE_ENABLE);
	
	//VCNL4020 Interupt Reg. 
	SetInterruptControl (INTERRUPT_THRES_SEL_PROX | INTERRUPT_THRES_ENABLE | INTERRUPT_COUNT_EXCEED_1);
	
	//VCNL4020 IRINT Current Reg. 
	SetCurrent (10);
}


/* TWI events handler. */
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
//                /* Read 4 bytes from the specified address. */
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

/* UART initialization. */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_VCNL4020_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_VCNL4020, &twi_VCNL4020_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_VCNL4020);
}

/* GPIO initialization. */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(IRINT_PIN, &out_config);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_gpiote_out_init(IREXT_PIN, &out_config);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_gpiote_out_init(LED_1, &out_config);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_gpiote_out_init(LED_2, &out_config);
    APP_ERROR_CHECK(err_code);
	
	SetIRLEDAnode (0);

//    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
//    in_config.pull = NRF_GPIO_PIN_PULLUP;
//    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
//    APP_ERROR_CHECK(err_code);
//    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}


					/////////////////////////////////////////
					/* =============== Main ============== */
					/////////////////////////////////////////
int main(void)
{
	uint16_t distance;
	uint8_t uart_rev_buffer[1] = {0};
	
	gpio_init();
	uart_config();
    twi_init();
    VCNL4020_Init();
	//printf("\n\rVCNL 4020 RS232 115200 Baud Driver\r\n");
    
    while(true)
    {
		while(!(ReadCommandRegister() & COMMAND_MASK_PROX_DATA_READY))
		{
			app_uart_get(uart_rev_buffer);  //printf("%d\n", (int)uart_rev_buffer[0]);
			
			if(uart_rev_buffer[0] == 0x69) // i = internal led
			{
				SetIRLEDAnode(0);
			}
			if(uart_rev_buffer[0] == 0x65) // e = external led
			{
				SetIRLEDAnode(1);
			}
			if(uart_rev_buffer[0] == 0x62) // b = both
			{
				SetIRLEDAnode(2);
			}
			
			if(uart_rev_buffer[0] == 0x63) // c = current followed by 0 thru 20 = 0 to 200 mA
			{
				uart_rev_buffer[0] = 0;
				while(uart_rev_buffer[0] == 0){app_uart_get(uart_rev_buffer);}
				SetCurrent (uart_rev_buffer[0]);	/// 0 - D ASCII - 0x30
			}
			
			if(uart_rev_buffer[0] == 0x72) // r = Rate followed by 1 thru 7: 1=4 2=8 3=16 4=31 5=62 6=125 7=250 sps
			{
				uart_rev_buffer[0] = 0;
		
				while(uart_rev_buffer[0] == 0){app_uart_get(uart_rev_buffer);}
				SetProximityRate (uart_rev_buffer[0]);
				if((uart_rev_buffer[0] & 0x01) == 0x01)
					nrf_drv_gpiote_out_clear(LED_2);
				else
					nrf_drv_gpiote_out_set(LED_2);
			}
					
			if(uart_rev_buffer[0] != 0x00)
			uart_rev_buffer[0] = 0;
		}	
		distance = ReadProxiValue();		
		//printf("i%d\n", distance);
		app_uart_put(0x69);	
		app_uart_put((uint8_t)(distance >> 8));	
		app_uart_put((uint8_t)(distance & 0xff));	
    }
}

//===============================================================================================//

void SetCommandRegister (unsigned char Command)
{
	ret_code_t err_code;
	
	reg[0] = REGISTER_COMMAND;
	reg[1] = Command;
	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS, reg, sizeof(reg), false); 	
    APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
}

uint8_t ReadCommandRegister (void)
{
	uint8_t return_value_uint8;
	ret_code_t err_code;
	
	reg[0] = REGISTER_COMMAND;
	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS,  reg, 1, false);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	err_code = nrf_drv_twi_rx(&m_twi_VCNL4020, VCNL4020_ADDRESS, &return_value_uint8, 1);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	return return_value_uint8;
}

void SetProximityRate (unsigned char Rate)
{
	ret_code_t err_code;
	
	SetCommandRegister (COMMAND_ALL_DISABLE);
	reg[0] = REGISTER_PROX_RATE;
	reg[1] = Rate;
	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS, reg, sizeof(reg), false); 	
    APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	SetCommandRegister (COMMAND_PROX_ENABLE | COMMAND_SELFTIMED_MODE_ENABLE);
}

void SetInterruptControl (unsigned char Command)
{
	ret_code_t err_code;
	
	reg[0] = REGISTER_INTERRUPT_CONTROL;
	reg[1] = Command;
	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS, reg, sizeof(reg), false); 	
    APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
}

void SetCurrent (unsigned char Current)
{
	ret_code_t err_code;
	
	reg[0] = REGISTER_PROX_CURRENT;
	reg[1] = Current;
	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS, reg, sizeof(reg), false); 	
    APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
}

uint16_t ReadProxiValue (void)
{
	uint16_t ret_distance;
	uint8_t  m_sample[2];
	ret_code_t err_code;
	
	reg[0] = REGISTER_PROX_VALUE;
	err_code = nrf_drv_twi_tx(&m_twi_VCNL4020, VCNL4020_ADDRESS,  reg, 1, true);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	err_code = nrf_drv_twi_rx(&m_twi_VCNL4020, VCNL4020_ADDRESS, m_sample, 2);
	APP_ERROR_CHECK(err_code);
	nrf_delay_us(delay);
	ret_distance = (uint16_t)m_sample[0] << 8 | (uint8_t)m_sample[1];

	return ret_distance;
}

void SetIRLEDAnode (uint8_t led_switch)
{
	if(led_switch == 0) // Int IR LED
	{
		nrf_drv_gpiote_out_clear(IRINT_PIN);
		nrf_drv_gpiote_out_clear(IREXT_PIN);
	}
	
	if(led_switch == 1) // Ext IR LED
	{
		nrf_drv_gpiote_out_set(IRINT_PIN);
		nrf_drv_gpiote_out_set(IREXT_PIN);
	}
	
	if(led_switch == 2) // both
	{
		nrf_drv_gpiote_out_clear(IRINT_PIN);
		nrf_drv_gpiote_out_set(IREXT_PIN);
	}
}


