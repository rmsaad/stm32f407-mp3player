# STM32f407 MP3 Player Application

![MP3 Player](images/mp3_setup.jpg?raw=true "Title")

STMf407 based mp3 player application.

#### Functions
* Decode mp3 files from usb drive and play audio using DMA transfer callbacks.
* Dynamic volume adjustment.
* Previous Track, Next Track, and Pause/Play buttons
* LCD screen contrast control

This project utilizes ST's HAL and Audio BSP for the STM32F4 Series  
https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html

This project utilizes FreeRTOS to schedule button and ADC (volume) polling, decoding/playback, and LCD updates  
https://www.freertos.org/

This Project utilizes the minimp3 library to decode mp3 frames and decode VBR tags.   
https://github.com/lieff/minimp3

