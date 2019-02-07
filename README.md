# nrf52840_e_ink

This project contains files to drive EPD Extension board GEN2  with nordic nRF52840-DK  evaluation board. 

Follow these steps to build the project
1. Install Keil uVision5(recommended IDE)
2. Download nRF5-SDK at  https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.2.0_9412b96.zip and unzip the package. nRF5-SDK has all the driver and library files, and example code for all peripheral component.
3. Open the blinky example at examples\peripheral\blinky\pca10056\blank\arm5_no_packs
4. Download the EPD190 folder. In Keil, create a new group in the project you open. Right click that group, choose add existing files and add all the files come from EPD190.
5. Add these nrf drivers to project
    \modules\nrfx\drivers\src\nrfx_timer.c
    \modules\nrfx\drivers\src\nrfx_spi.c
    \integration\nrfx\legacy\nrf_drv_spi.c.
6. In C:\Users\yz\Desktop\nrf52840DK\nRF5_SDK_15.2.0_9412b96\modules\nrfx\mdk\nrf52840_peripherals.h, delete or comment line 170:            #define SPIM_PRESENT
7. In project option, add following C/C++ include path:   
  \EPD190  
  \EPD190\HW_drivers  
  ..\..\..\..\..\..\integration\nrfx\legacy  
  ..\..\..\..\..\..\modules\nrfx\drivers\include
8. Connect wires between nRFDK and EXT2 board acoording the pin defination in "Board_HW_Driver.h".
9. Build and program the firmware.

# nRF5 SDK documentation
http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.2.0%2Findex.html&cp=4_0_0
