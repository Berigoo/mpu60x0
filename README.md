# MPU60x0
Object-oriented library for interfacing with MPU-6050 and
MPU-6000 using ESP-IDF framework. Supports both Hardware I2C and
Software I2C with roll-pitch calculation using Madgwick Filter.

## Features
- **Dual I2C Support**:
  - Hardware I2C (using ESP32's I2C peripherals)
  - Software I2C (bit-banged implementation)
  
- **Sensor Data Reading**:
  - scaled accelerometer data (X, Y, Z)
  - scaled gyroscope data (X, Y, Z)
  - Temperature readings
  
- **Orientation Calculation**:
  - Roll and pitch estimation using Madgwick Filter
  - Configurable filter parameters
  
- **Configuration Options**:
  - Adjustable accelerometer scale range
  - Adjustable gyroscope scale range 

## Installation

1. clone this repository into your project's `components` directory.
```shell
git clone https://github.com/Berigoo/mpu60x0.git 
```
2. Modify project root `CMakeLists.txt` (if not yet)
```CMake
#...
set(EXTRA_COMPONENT_DIRS components)
#...
```
3. Add the component
```Cmake
idf_component_register(SRCS "example.cpp"
                    INCLUDE_DIRS "."
		    REQUIRES mpu60x0) 
```