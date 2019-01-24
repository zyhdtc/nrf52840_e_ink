/**
* Copyright (c) Pervasive Displays Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
**/

#include "Board_HW_Driver.h"
#include <HW_drivers/UART_Driver.h>
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
static  volatile uint32_t EPD_Counter;
const nrf_drv_timer_t TIMER_EPD = NRF_DRV_TIMER_INSTANCE(0);


/**
 * @brief Handler for timer events.
 */
void timer_epd_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    EPD_Counter++;
}


void Systick_Configuration(void)
{
		uint32_t time_ms = 1; // time in ms to be compared with
		uint32_t err_code = NRF_SUCCESS;
		uint32_t time_ticks;
		//Configure TIMER_EPD
		nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_EPD, &timer_cfg, timer_epd_event_handler);
    APP_ERROR_CHECK(err_code);
	
		time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_EPD, time_ms);

    nrf_drv_timer_extended_compare(&TIMER_EPD, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

/**
 * \brief Start Timer
 * \note
 * desired value: 1mSec
 * actual value:  1.000mSec
 */
void start_EPD_timer(void)
{
	nrf_drv_timer_enable(&TIMER_EPD);
	EPD_Counter=0;
}


/**
 * \brief Stop Timer
 */
void stop_EPD_timer(void)
{
	nrf_drv_timer_disable(&TIMER_EPD);
}
/**
 * \brief Get current Timer after starting a new one
 */
uint32_t get_EPD_time_tick(void)
{

	return (EPD_Counter);
}

/**
 * \brief Delay mini-seconds
 * \param ms The number of mini-seconds
 */
 void EPD_delay_ms(unsigned int ms)
{
	start_EPD_timer();
	while(get_EPD_time_tick()<=ms)
	{

	}
}

void delay_btwn_CS_H_L(void)
{
	//delay > 80ns
}

//static uint32_t clockValue;
void system_init(void)
{


}
/**
 * \brief PWM toggling.
 *
 * \param ms The interval of PWM toggling (mini seconds)
 */
#include "nrf_delay.h"
void PWM_run(uint16_t ms)
{

	start_EPD_timer();
	do
	{
		//~156K Hz    not accurate
		EPD_pwm_high();
		nrf_delay_us(6);
		EPD_pwm_low();
		nrf_delay_us(6);
	}
	while (get_EPD_time_tick() < ms);   //wait Delay Time

	stop_EPD_timer();

}
//******************************************************************
//* SPI  Configuration
//******************************************************************

#include "nrf_drv_spi.h"
#include "sdk_config.h"
bool spi_state=false;
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
//nrf_drv_spi_t const * const p_instance = &spi;

/**
 * \brief Configure SPI
 */
void epd_spi_init(nrf_drv_spi_frequency_t spi_baudrate)
{
//	USCI_B_SPI_disable(USCI_B0_BASE);
//	//config  i/o
//	config_gpio_dir_o(SPICLK_PORT, SPICLK_PIN);
//	config_gpio_dir_o(SPIMOSI_PORT, SPIMOSI_PIN);
//	config_gpio_dir_i(SPIMISO_PORT, SPIMISO_PIN);
//	//P3.0,1,2 option select
//	GPIO_setAsPeripheralModuleFunctionInputPin(
//	    GPIO_PORT_P3,
//	    GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2
//	);
//	//Initialize Master
//	USCI_B_SPI_initMasterParam param = {0};
//	param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
//	param.clockSourceFrequency = UCS_getSMCLK();
//	param.desiredSpiClock = spi_baudrate;
//	param.msbFirst = USCI_B_SPI_MSB_FIRST;
//	param.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
//	param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
//	spi_state=USCI_B_SPI_initMaster(USCI_B0_BASE, &param);
//	//Enable SPI module
//	if(spi_state)USCI_B_SPI_enable(USCI_B0_BASE);


		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin    = SPI_SS_PIN;
    spi_config.miso_pin  = SPI_MISO_PIN;
    spi_config.mosi_pin  = SPI_MOSI_PIN;
    spi_config.sck_pin   = SPI_SCK_PIN;
		spi_config.frequency = spi_baudrate;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));

}
/**
 * \brief Initialize SPI
 */
void EPD_spi_attach()
{
	EPD_flash_cs_high();
	EPD_cs_high();
	epd_spi_init(COG_SPI_baudrate);
}
/**
 * \brief Disable SPI and change to GPIO
 */
void EPD_spi_detach(void)
{
//	USCI_B_SPI_disable(USCI_B0_BASE);
//	GPIO_setAsOutputPin(
//	    GPIO_PORT_P3,
//	    GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2
//	);
	nrf_drv_spi_uninit(&spi);
	config_gpio_dir_o(SPICLK_PORT,SPICLK_PIN);
	config_gpio_dir_o(SPIMISO_PORT,SPIMISO_PIN);
	config_gpio_dir_o(SPIMOSI_PORT,SPIMOSI_PIN);
	SPIMISO_low();
	SPIMOSI_low();
	SPICLK_low();
	spi_state=false;
}
bool check_flash_spi(void)
{
	if(!spi_state) EPD_spi_attach();
	return spi_state;
}
/**
 * \brief SPI synchronous read
 */
uint8_t EPD_spi_read(unsigned char data)
{
	
	//NRF_SPI0 = ((NRF_SPI_Type*)  NRF_SPI0_BASE)
	nrf_spi_txd_set(NRF_SPI0, data);
	//wait utill spi finish transfer
	//hard-coded the delay time
	//while(!NRF_SPI0->EVENTS_READY);
	nrf_delay_us(10);
	volatile uint8_t rx_data = nrf_spi_rxd_get(NRF_SPI0);
	nrf_delay_us(10);
	//while(!NRF_SPI0->EVENTS_READY);
	return rx_data;
}

/**
 * \brief SPI synchronous write
 */
void EPD_spi_write(unsigned char data)
{

	EPD_spi_read(data);

}
void iTC_spi_sendReg(uint8_t register_index)
{
	EPD_cs_low();
	EPD_DC_low();
	EPD_spi_write(register_index);
	EPD_DC_high();
}
void iTC_spi_sendRegData(uint8_t register_data)
{
	EPD_spi_write(register_data);
}

void iTC_spi_send(uint8_t register_index,uint8_t *register_data,uint8_t len)
{
#if 0
	UartSend((const uint8_t *)&register_index,1);
	if(len>0)UartSend((const uint8_t *)register_data,len);
#endif
	EPD_cs_low();
	EPD_DC_low();
	nrf_delay_us(20);
	EPD_spi_write(register_index);
	EPD_DC_high();
	nrf_delay_us(20);
	while(len--)
	{
		EPD_spi_write(*register_data++);
	}

	EPD_cs_high();
}
void iTC_spi_send_array(Em_register_array_t *register_array,uint8_t len)
{
	uint8_t i;
	for(i=0; i<len; i++)
	{
		iTC_spi_send(register_array[i].register_index,register_array[i].register_data,register_array[i].len);
	}
}
/**
* \brief SPI command
*
* \param register_index The Register Index as SPI Data to COG
* \param register_data The Register Data for sending command data to COG
* \return the SPI read value
*/
uint8_t EPD_Gx_spi_r(uint8_t register_index, uint8_t register_data)
{
	uint8_t result;
	EPD_cs_low ();
	EPD_spi_write (0x70); // header of Register Index
	EPD_spi_write (register_index);
	EPD_cs_high ();
	delay_btwn_CS_H_L ();
	EPD_cs_low ();

	EPD_spi_write (0x73); // header of Register Data of read command
	result=EPD_spi_read (register_data);

	EPD_cs_high ();

	return result;
}
uint8_t EPD_Gx_spi_rid(void)
{
	uint8_t result;
	EPD_cs_low ();
	EPD_spi_write (0x71);
	result=EPD_spi_read (0x00);
	EPD_cs_high ();
	return result;
}

/**
* \brief SPI command if register data is larger than two bytes
*
* \param register_index The Register Index as SPI command to COG
* \param register_data The Register Data for sending command data to COG
* \param length The number of bytes of Register Data which depends on which
* Register Index is selected.
*/
void EPD_Gx_spi_send (unsigned char register_index, unsigned char *register_data,
                      unsigned length)
{
	EPD_cs_low ();
	EPD_spi_write (0x70); // header of Register Index
	EPD_spi_write (register_index);
	EPD_cs_high ();
	delay_btwn_CS_H_L ();
	EPD_cs_low ();

	EPD_spi_write (0x72); // header of Register Data of write command
	while(length--)
	{
		EPD_spi_write (*register_data++);
	}
	EPD_cs_high ();
}

/**
* \brief SPI command
*
* \param register_index The Register Index as SPI command to COG
* \param register_data The Register Data for sending command data to COG
*/
void EPD_Gx_spi_send_byte (uint8_t register_index, uint8_t register_data)
{
	EPD_cs_low ();
	EPD_spi_write (0x70); // header of Register Index
	EPD_spi_write (register_index);
	EPD_cs_high ();
	delay_btwn_CS_H_L ();
	EPD_cs_low ();
	EPD_spi_write (0x72); // header of Register Data
	EPD_spi_write (register_data);
	EPD_cs_high ();
}

/**
* \brief Configure GPIO
*/
void EPD_initialize_gpio(void)
{
	//COG Control IO Configuration
	Config_Busy_Pin();
	Config_EPD_cs_Pin();
	Config_EPD_rst_Pin();
	Config_EPD_panelon_Pin();
	Config_EPD_discharge_Pin();
	Config_EPD_border_Pin();
	Config_EPD_flash_cs_Pin();
	Config_EPD_pwm_Pin();
	Config_LED1_Pin();
	Config_BS1_Pin();
	BS1_low();//Must be LOW
	EPD_pwm_low();
	EPD_flash_cs_high();
	EPD_border_low();
}
/**
 * \brief Initialize the EPD hardware setting
 */
void EPD_display_hardware_init(void)
{
	EPD_initialize_gpio();
	EPD_Vcc_turn_off();
	EPD_cs_low();
	EPD_rst_low();
	EPD_discharge_low();
}

