idf_component_register(SRCS "mpu60x0.cpp" "MadgwickAHRS.cpp" "soft_i2c_master.c" "I2CDeviceHandle.cpp"
                    INCLUDE_DIRS "include" REQUIRES driver esp_timer) 
