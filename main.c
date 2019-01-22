/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "conf_EPD.h"
 
 
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#define SPI_INSTANCE  0 /**< SPI instance index. */
//static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
//static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

//#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
//void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
//                       void *                    p_context)
//{
//    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received:");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
//}
/**
 * \brief define the timing controller type and size of EPD
 * \note
 *  - eTC=external timing controller (driver IC)
 *  - iTC=internal timing controller */
// eTC
#define sz_eTC_144  0 //J7 Switches 0000 001x
#define sz_eTC_190  1 //J7 Switches 0000 000x
#define sz_eTC_200  2 //J7 Switches 0000 001x
#define sz_eTC_260  3 //J7 Switches 0000 000x
#define sz_eTC_271  4 //J7 Switches 0000 000x
// iTC
#define sz_iTC_215  5 //J7 Switches 1010 000x
#define sz_iTC_287  6 //J7 Switches 0101 010x
#define sz_iTC_420  7 //J7 Switches 0101 010x

/**
 * \brief define the EPD drivers with film material
 * \note
 *  - BW=black and white colors, Aurora film
 *  - BWR=black, white and red colors, Spectra-red film
 *  - a=Aurora Ma (V230)
 *  - b=Aurora Mb (V231) */
#define dr_eTC_BWa			0
#define dr_eTC_BWb			1
#define dr_iTC_BWb			2

#define USE_EPD_Type    dr_eTC_BWb
#define USE_EPD_Size    sz_eTC_190

#define  EPD_Type		dr_eTC_G2_Aurora_Mb
extern unsigned char const  Image_eTC_190_01[];
extern unsigned char const  Image_eTC_190_02[];
#define  EPD_Size     EPD_190_BW
#define Image1        (uint8_t *)&Image_eTC_190_01
#define Image2        (uint8_t *)&Image_eTC_190_02
int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

//    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//    spi_config.ss_pin   = SPI_SS_PIN;
//    spi_config.miso_pin = SPI_MISO_PIN;
//    spi_config.mosi_pin = SPI_MOSI_PIN;
//    spi_config.sck_pin  = SPI_SCK_PIN;
//    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI example started.");

    while (1)
    {
        // Reset rx buffer and transfer done flag
//        memset(m_rx_buf, 0, m_length);
//        spi_xfer_done = false;

//        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

//        while (!spi_xfer_done)
//        {
//            __WFE();
//        }

//        NRF_LOG_FLUSH();
				EPD_display_GU_from_pointer(Image2,Image1,GU_Mode,true);
        EPD_delay_ms(5000);
        EPD_display_GU_from_pointer(Image1,Image2,GU_Mode,true);
        EPD_delay_ms(5000);
        bsp_board_led_invert(BSP_BOARD_LED_0);
//        nrf_delay_ms(200);
    }
}
