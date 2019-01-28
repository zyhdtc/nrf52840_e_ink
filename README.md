# nrf52840_e_ink

This project contains files to drive EPD Extension board GEN2  with nordic nRF52840-DK  evaluation board. 

Follow these steps to build the project
1. Download Keil uVision5 if you don't have one
2. Download nRF5-SDK at  https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.2.0_9412b96.zip and unzip the package
3. Open the blinky example at examples\peripheral\blinky\pca10056\blank\arm5_no_packs
4. Download the EPD190 folder. In Keil, create a new group in the project you open. Right click that group, choose add existing files and add all the files come from EPD190.
5. Add these nrf drivers to project
    \modules\nrfx\drivers\src\nrfx_timer.c
    \modules\nrfx\drivers\src\nrfx_spi.c
    \integration\nrfx\legacy\nrf_drv_spi.c
6. In project option, add following C/C++ include path:   
  \EPD190
  \EPD190\HW_drivers
  ..\..\..\..\..\..\integration\nrfx\legacy
  ..\..\..\..\..\..\modules\nrfx\drivers\include
7. Connect wires between nRFDK and EXT2 board acoording the pin defination in "Board_HW_Driver.h".
8. Build and program the firmware.

What did I do
