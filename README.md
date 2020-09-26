# Logging tactile data collected by the STAG

## Introduction
Flashing this code on the STM32F769I-DISC1 board enables the collection and logging of tactile data.


## System requirements

If using the Makefile to compile the project, GNU Embedded Toolchain for Arm is required.
If using the Makefile to flash the code on the MCU (in terminal when inside the project folder: make flash), openocd is required.


## Additional information

The correct drivers for the application need to be added in a directory called /Drivers.
The project can also be created with the .ioc file in CubeMX, which will automatically add the neccessary drivers.


## How to use

When pushing the blue button on the STM32F769I-DISC1 board, tactile and IMU data is collected at a rate of 100Hz.
To improve latency the IMU data is sent directly while the tactile data is first written to SDRAM.
After a specified maximum number of collected frames - max_count - the application stops data collection and starts sending the tactile data over UART with a baudrate of 921600.
Currently 4096 frames is the maximum capacity of the SDRAM even though the total available number of bits should permit 8192 frames.
The reason for this is that the SDRAM has slots of 32bit and the tactile sensor values are 16bit.
An easy way to log the incoming UART data is using Tera Term's built-in log functionality.
However, moving any Tera Term window during an ongoing transmission leads to loss of data!


## Contact

Please email any questions or comments to [geigerf@student.ethz.ch](geigerf@student.ethz.ch).
