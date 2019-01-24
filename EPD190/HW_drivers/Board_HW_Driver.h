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

//#include "driverlib.h"
#include <stdbool.h>
#include <stdint.h>
#include <nrf_gpio.h>

#ifndef BOARD_HW_DRIVER_H_
#define BOARD_HW_DRIVER_H_

typedef void (*Systick_handler)(uint32_t tick);
typedef struct Em_register_array
{
	uint8_t register_index;
	uint8_t len;
	uint8_t *register_data;
} Em_register_array_t;

#define	_BV(bit)   								(1 << (bit % 8)) /**< left shift 1 bit */
#define	_HIGH      								1            /**< signal high */
#define	_LOW       								!_HIGH       /**< signal low */

#define	config_gpio_dir_o(port, pin)  		  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(port, pin))    /**< set output direction for an IOPORT pin */
#define	config_gpio_dir_i(port, pin)  			nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(port, pin), (nrf_gpio_pin_pull_t)GPIO_PIN_CNF_PULL_Pulldown )  /**< set input direction for an IOPORT pin */
#define	set_gpio_high(port, pin)      			nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(port, pin)) /**< set HIGH for an IOPORT pin */
#define	set_gpio_low(port, pin)       			nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(port, pin))  /**< set LOW for an IOPORT pin */
#define	set_gpio_invert(port, pin)    			nrf_gpio_pin_toggle(NRF_GPIO_PIN_MAP(port, pin)) /**< toggle the value of an IOPORT pin */
#define	input_get(port, pin)          			(bool)(nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(port, pin)))   /**< get current value of an IOPORT pin */

/******************************************************************************
* GPIO Defines
*****************************************************************************/

#define EPD_CS_PORT             				0       /**< LaunchPad P0.29  */
#define EPD_CS_PIN              				29
#define SPICLK_PORT             				0       /**< LaunchPad P0.3 */
#define SPICLK_PIN              				3
#define SPIMISO_PORT            				0       /**< LaunchPad P0.28  */
#define SPIMISO_PIN             				28
#define SPIMOSI_PORT            				0       /**< LaunchPad P0.4  */
#define SPIMOSI_PIN             				4
#define Temper_PORT             				0       /**< LaunchPad P6.6 */
#define Temper_PIN              				2
#define EPD_BUSY_PORT           				0       /**< LaunchPad P2.7  */
#define EPD_BUSY_PIN            				31
#define PWM_PORT                				0       /**< LaunchPad P4.2  */
#define PWM_PIN                 				26
#define EPD_RST_PORT            				0       /**< LaunchPad P4.1 */
#define EPD_RST_PIN             				27
#define EPD_PANELON_PORT        				1       /**< LaunchPad P8.1  */
#define EPD_PANELON_PIN         				15
#define EPD_DISCHARGE_PORT      				1       /**< LaunchPad P2.3  */
#define EPD_DISCHARGE_PIN       				14
#define EPD_BORDER_PORT         				1       /**< LaunchPad P2.6  */
#define EPD_BORDER_PIN          				13

#define Flash_CS_PORT           				0       /**< LaunchPad P2.2  */
#define Flash_CS_PIN            				30

#define EPD_DC_PORT             				1       /**< LaunchPad P4.2  */
#define EPD_DC_PIN              				11
#define EPD_DB1_PORT             				1       /**< LaunchPad P2.6  */
#define EPD_DB1_PIN              				10
#define EPD_DB2_PORT             				1       /**< LaunchPad P6.5  */
#define EPD_DB2_PIN              				8

#define LED1_PORT           					1       /**< LaunchPad P1.0  */
#define LED1_PIN            				    07

#define BS1_PORT           				        1       /**< LaunchPad P7.4  */
#define BS1_PIN            				        06

#define SW2_PORT           						1       /**< LaunchPad P1.1  */
#define SW2_PIN            						05
#define SW1_PORT           						1       /**< LaunchPad P2.1  */
#define SW1_PIN            						04



#define Config_BS1_Pin() 					config_gpio_dir_o(BS1_PORT,BS1_PIN)
#define BS1_high() 								set_gpio_high(BS1_PORT,BS1_PIN)
#define BS1_low() 								set_gpio_low(BS1_PORT,BS1_PIN)
#define Enable_SW_Pin() 					config_gpio_dir_i(SW1_PORT,SW1_PIN |SW2_PIN )
#define Disable_SW_Pin()  				{config_gpio_dir_o(BS1_PORT,BS1_PIN);BS1_low();}
#define SW1_State()      					(bool)input_get(SW1_PORT,SW1_PIN
#define SW2_State()     		 			(bool)input_get(SW2_PORT,SW2_PIN
#define Config_LED1_Pin()					config_gpio_dir_o(LED1_PORT,LED1_PIN)
#define LED1_high()								set_gpio_high(LED1_PORT,LED1_PIN)
#define LED1_low()								set_gpio_low(LED1_PORT,LED1_PIN)
#define Config_Busy_Pin()					config_gpio_dir_i(EPD_BUSY_PORT,EPD_BUSY_PIN)
#define EPD_IsBusy()							(bool)input_get(EPD_BUSY_PORT,EPD_BUSY_PIN)
#define Config_EPD_cs_Pin()				config_gpio_dir_o(EPD_CS_PORT,EPD_CS_PIN)
#define EPD_cs_high()							set_gpio_high(EPD_CS_PORT,EPD_CS_PIN)
#define EPD_cs_low()							set_gpio_low(EPD_CS_PORT,EPD_CS_PIN)
#define Config_EPD_flash_cs_Pin()	config_gpio_dir_o(Flash_CS_PORT,Flash_CS_PIN)
#define EPD_flash_cs_high()				set_gpio_high(Flash_CS_PORT,Flash_CS_PIN)
#define EPD_flash_cs_low()				set_gpio_low(Flash_CS_PORT,Flash_CS_PIN)
#define Config_EPD_rst_Pin()			config_gpio_dir_o(EPD_RST_PORT,EPD_RST_PIN)
#define EPD_rst_high()						set_gpio_high(EPD_RST_PORT,EPD_RST_PIN);
#define EPD_rst_low()					 		set_gpio_low(EPD_RST_PORT,EPD_RST_PIN)
#define Config_EPD_discharge_Pin()		config_gpio_dir_o(EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN)
#define EPD_discharge_high()			set_gpio_high(EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN)
#define EPD_discharge_low()				set_gpio_low(EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN)
#define Config_EPD_panelon_Pin()	config_gpio_dir_o(EPD_PANELON_PORT,EPD_PANELON_PIN)
#define EPD_Vcc_turn_on()					set_gpio_high(EPD_PANELON_PORT,EPD_PANELON_PIN)
#define EPD_Vcc_turn_off() 				set_gpio_low(EPD_PANELON_PORT,EPD_PANELON_PIN)
#define Config_EPD_border_Pin() 	config_gpio_dir_o(EPD_BORDER_PORT,EPD_BORDER_PIN)
#define EPD_border_high()					set_gpio_high(EPD_BORDER_PORT,EPD_BORDER_PIN)
#define EPD_border_low()					set_gpio_low(EPD_BORDER_PORT,EPD_BORDER_PIN)
#define Config_EPD_pwm_Pin()			config_gpio_dir_o(PWM_PORT,PWM_PIN)
#define EPD_pwm_high()						set_gpio_high(PWM_PORT,PWM_PIN)
#define EPD_pwm_low()							set_gpio_low(PWM_PORT,PWM_PIN)
#define SPIMISO_low()							set_gpio_low(SPIMISO_PORT,SPIMISO_PIN)
#define SPIMOSI_low()							set_gpio_low(SPIMOSI_PORT,SPIMOSI_PIN)
#define SPICLK_low()							set_gpio_low(SPIMOSI_PORT,SPIMOSI_PIN)
#define Config_EPD_DC_Pin()				config_gpio_dir_o(EPD_DC_PORT,EPD_DC_PIN)
#define EPD_DC_high()							set_gpio_high(EPD_DC_PORT,EPD_DC_PIN)
#define EPD_DC_low()							set_gpio_low(EPD_DC_PORT,EPD_DC_PIN)
#define Config_EPD_DB1_Pin()			config_gpio_dir_i(EPD_DB1_PORT,EPD_DB1_PIN)	
#define Get_EPD_DB1()							(bool)input_get(EPD_DB1_PORT,EPD_DB1_PIN)
#define Config_EPD_DB2_Pin()			config_gpio_dir_i(EPD_DB2_PORT,EPD_DB2_PIN)	
#define Get_EPD_DB2()							(bool)input_get(EPD_DB2_PORT,EPD_DB2_PIN)
		


/** The SPI frequency of this kit (8MHz) */
#define COG_SPI_baudrate    					NRF_DRV_SPI_FREQ_8M


//========================================================================================================
void system_init(void);
uint32_t getnowtime(void);
void Systick_Configuration(void);
void systick_Start(void);
void systick_Stop(void);

void start_EPD_timer(void);
void stop_EPD_timer(void);
uint32_t get_EPD_time_tick(void);
void EPD_delay_ms(unsigned int ms);

void EPD_spi_attach (void);
void EPD_spi_detach (void);
bool check_flash_spi(void);
uint8_t EPD_spi_read(uint8_t data);
void EPD_spi_write (uint8_t data);

void iTC_spi_sendReg(uint8_t register_index);
void iTC_spi_sendRegData(uint8_t register_data);
void iTC_spi_send(uint8_t register_index,uint8_t *register_data,uint8_t len);
void iTC_spi_send_array(Em_register_array_t *register_array,uint8_t len);

void EPD_Gx_spi_send (unsigned char register_index, unsigned char *register_data, unsigned Length);
void EPD_Gx_spi_send_byte (uint8_t register_index, uint8_t register_data);
uint8_t EPD_Gx_spi_r(uint8_t register_index, uint8_t register_data);
uint8_t EPD_Gx_spi_rid(void);

void EPD_display_hardware_init (void);
void PWM_run(uint16_t time);

#define delay_ms(x)  EPD_delay_ms(x)

#endif /* BOARD_HW_DRIVER_H_ */
