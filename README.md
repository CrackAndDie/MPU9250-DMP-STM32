<p align="center">
  <a href="https://github.com/CrackAndDie/MPU-9250-DMP-STM32">
    <img src="https://user-images.githubusercontent.com/52558686/181789584-f6e33691-1383-49ee-aa5d-7c87300e72bf.png" alt="MPU9250" width="350" height="350">
  </a>
</p>
<h1 align="center">MPU-9250 Digital Motion Processor (DMP) for STM32</h1>

Advanced STM32 library for the Invensense MPU-9250 inertial measurement unit (IMU), which enables the sensor's digital motion processing (DMP) features. Along with configuring and reading from the accelerometer, gyroscope, and magnetometer, this library also supports the chip's DMP features like:

* Quaternion calculation
* Pedometer
* Gyroscope calibration
* Tap detection
* Orientation dtection 

This library uses I2C bus, not SPI!

Repository Contents
-------------------

* **/src** - Source files for the library (.c, .h).
	* **/src/util** - Source and headers for the MPU-9250 driver and dmp configuration. These are available and adapted from [Invensene's downloads page](https://www.invensense.com/developers/software-downloads/#sla_content_45).

How To Use
--------------

- Open file **stm32_mpu9250_i2c.c** and change the ```extern I2C_HandleTypeDef hi2c2;``` line by writing there your I2C Handler and everywhere in the file.
- Similar usage examples you can find [here](https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/tree/master/examples).

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE and SparkFun_LICENSE.md files for license information. 
