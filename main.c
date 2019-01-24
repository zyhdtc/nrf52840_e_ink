/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */
#include "sdk_config.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "conf_EPD.h"
#include "nrf_drv_spi.h"

#define USE_EPD_Type    dr_eTC_BWb
#define USE_EPD_Size    sz_eTC_190

#define  EPD_Type		dr_eTC_G2_Aurora_Mb
extern unsigned char const  Image_eTC_190_01[];
extern unsigned char const  Image_eTC_190_02[];
#define  EPD_Size     EPD_190_BW
#define Image1        (uint8_t *)&Image_eTC_190_01
#define Image2        (uint8_t *)&Image_eTC_190_02
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Configure board. */
    bsp_board_init(BSP_INIT_LEDS);
		EPD_display_init();
    Systick_Configuration();
    EPD_delay_ms(1000);
    Set_AssignEPD_Drive(EPD_Type,EPD_Size,USE_Temperature_Sensor);
    /* Toggle LEDs. */
		int i = 0;
    while (true)
    {
        for (i = 0; i < 4; i++)
        {
            bsp_board_led_invert(0);
            EPD_delay_ms(200);
        }
				
				EPD_display_GU_from_pointer(Image2,Image1,GU_Mode,true);
        EPD_delay_ms(5000);
        EPD_display_GU_from_pointer(Image1,Image2,GU_Mode,true);
				        for (i = 0; i < 4; i++)
        {
            bsp_board_led_invert(1);
            EPD_delay_ms(200);
        }
        EPD_delay_ms(5000);
    }
}

/**
 *@}
 **/
