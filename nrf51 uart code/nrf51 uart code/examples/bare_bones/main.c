#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf_gpio.h"


#define RANDOM_PIN                  30
#define TE_INDEX                    5
#define TWI_ADDRESS                 0x6A
#define TWI_SDA                     3
#define TWI_SCL                     4

#define SENSE_FIELD_POS				(6)
#define SENSE_FIELD_MASK			(0xC0)
#define NUMBER_OF_GPIO_TE 			(8)

static inline void sleep(void)
{
    __SEV();
    __WFE();
    __WFE();
}

static void gpiote_port_mode_init()
{
	//Use GPIOTE in PORT Mode
	NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_PORT_Msk;
	
	NRF_GPIO->PIN_CNF[ RANDOM_PIN ] = (NRF_GPIO_PIN_DIR_INPUT   << GPIO_PIN_CNF_DIR_Pos)
								| (NRF_GPIO_PIN_INPUT_CONNECT << GPIO_PIN_CNF_INPUT_Pos)
								| (NRF_GPIO_PIN_NOPULL  << GPIO_PIN_CNF_PULL_Pos)
								| (NRF_GPIO_PIN_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
								| (NRF_GPIO_PIN_SENSE_HIGH << GPIO_PIN_CNF_SENSE_Pos);
	
	NRF_GPIOTE->EVENTS_PORT = 1;	//As per Product Spec
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
}
	
static void gpiote_init()
{
	NRF_GPIOTE->CONFIG[TE_INDEX] =
		(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
		(RANDOM_PIN << GPIOTE_CONFIG_PSEL_Pos) |
		(GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
}


static void gpiote_disable()
{
    NRF_GPIOTE->CONFIG[TE_INDEX] = (GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos);
}


static void twi_init(void)
{
    NRF_TWIM0->PSEL.SCL = TWI_SCL;
    NRF_TWIM0->PSEL.SDA = TWI_SDA;
    NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100;
}


static void twi_read_chunk(uint8_t address, uint8_t * p_buffer, uint8_t length)
{
    NRF_TWIM0->ADDRESS = address;
    NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTRX_STOP_Msk;
    NRF_TWIM0->RXD.PTR = (uint32_t)p_buffer;
    NRF_TWIM0->RXD.MAXCNT = length;

    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->TASKS_STARTRX = 1;
    while (!NRF_TWIM0->EVENTS_STOPPED)
    {}
}


static void twi_disable(void)
{
    NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
}

#define LED_CNTRL	11
int main(void)
{
    uint8_t data[4];

	nrf_gpio_pin_clear( LED_CNTRL );
	nrf_gpio_cfg_output( LED_CNTRL );	
	
	gpiote_port_mode_init();

    twi_init();
    twi_read_chunk(TWI_ADDRESS, data, 4);
    twi_disable();
	
	//gpiote_disable();
    //gpiote_init();

    for (;;)
    {
        sleep();
    }
}
