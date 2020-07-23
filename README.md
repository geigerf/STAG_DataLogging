# Logging tactile data collected by the STAG

## Introduction
Flashing this code on the STM32F769I-DISC1 board enables the collection and logging of tactile
data.


## System requirements

If using the Makefile to compile the project GNU Embedded Toolchain for Arm is required.
If using the Makefile to flash the code one the MCU (in terminal when inside the project folder: make flash)
openocd is required.


## Additional information

The correct drivers for the application need to added in a directory called /Drivers.
The project can also be created with the .ioc file in CubeMX, which will automatically add the
neccessary drivers.


## How to use

When pushing the blue button on the STM32F769I-DISC1 board, tactile and IMU data is collected with 100Hz.
To improve latency the IMU data is sent directly while the tactile data is first written to SDRAM.
After a specified maximum number of collected frames - max_count - the application stops
data collection and starts sending the tactile data over UART with a baudrate of 921600
(currently 4096 frames is the maximum capacity of the SDRAM).
An easy way to log the UART data is using Tera Term's built-in log functionality.


## History

* **July 2020**: Code uploaded to GitLab.


## Contact

Please email any questions or comments to [geigerf@student.ethz.ch](geigerf@student.ethz.ch).
