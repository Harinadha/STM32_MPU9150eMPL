=======================================================================================
               For More details, visit: http://harinadha.wordpress.com
=======================================================================================

The MPU9150 I2C Library provides simple and intuitive interfaces to MPU9150 on ARM 32-bit STM32F103xx family of microcontrollers. It has the I2C bit- and byte-level communication.

The code is written primarily to support the MPU9150 I2C device implementation.
This code has been tested using Olimex STM32-P103 developement board & Sparkfun MPU9150 breakout board. The dev board has STM32F103RB MCU. Five pins of MPU9150 breakout board are connected to the STM32-P103 as shown below. By connecting STM32-P103 board to computer USB port, you can send commands to the board & get the data from it.
USB Virtual COM port driver as well as GUI software needed for testing also available at: http://harinadha.wordpress.com/2013/06/25/mpu9150lib/ 
For USB connection ciruit, please check the schematics of Olimex STM32-P103 dev board.

MPU9150         STM32-P103
--------------------------
GND ------------- GND
VCC ------------- 3.3v
SCL ------------- SCL ( I2C2 )
SDA ------------- SDA ( I2C2 )
INT ------------- PC9

You may find the pre-compiled binary file (STM32_MPU9150eMPL.hex) in the Debug/Exe folder in case if you want to FLASH the program directly on to your board ( In this case, make sure your micro is STM32F103RB & all the connections are same as here).Documentation will be available in near future using Doxygen-style comments placed in each definition file. This documentation will be available in HTML format on the http://harinadha.wordpress.com website, which also holds additional helpful information.

Sample application code provides Accelerometer, Gyro, Magnetometer, 6-Axes DMP quaternion, Pedometer count, TAP, Android_Orientation & other data.
