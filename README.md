# MPU60x0
Object oriented library for interfacing with Mpu-6050 and
Mpu-6000 using ESP-IDF framework. Support both Hardware I2C and
Software I2C. Also Roll-Pitch calculation using Madgwick Filter.

## Features
- Support Hard and Soft I2C
- Read acceleration and gyroscope data
- Read roll and pitch data
- Set scale range for accelerometer and gyroscope
- Set several madgwick filter variables

## Instalation

1. clone this repository into your `components` directory. Assume you are in your project root directory,
```shell
git clone https://github.com/Berigoo/mpu60x0.git components
```
2. Modify project root `CMakeLists.txt` (if not yet)
```CMake
#...
set(EXTRA_COMPONENT_DIRS components)
#...
```
3. Addd the component
```Cmake
idf_component_register(SRCS "example.cpp"
                    INCLUDE_DIRS "."
		    REQUIRES mpu60x0) 
```