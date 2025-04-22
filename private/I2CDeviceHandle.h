#ifndef I2CDEVICEHANDLE_H
#define I2CDEVICEHANDLE_H

#include "driver/i2c_types.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "../include/soft_i2c_master.h"
#include <cstdint>

class I2CDeviceHandle {
 public:
  virtual esp_err_t transmit(const uint8_t *data, size_t size) = 0;
  virtual esp_err_t receive(uint8_t *buffer, size_t readSize) = 0;
  virtual esp_err_t transmitReceive(const uint8_t *data, size_t size,
                                    uint8_t *buffer, size_t readSize) = 0;
  virtual ~I2CDeviceHandle() = default;
};

class HardI2C : public I2CDeviceHandle {
 private:
  i2c_master_dev_handle_t m_device;
  
 public:
  HardI2C(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t masterFreq);
  ~HardI2C();
   
  esp_err_t transmit(const uint8_t *data, size_t size) override;
  esp_err_t receive(uint8_t *buffer, size_t readSize) override;
  esp_err_t transmitReceive(const uint8_t *data, size_t size,
			    uint8_t *buffer, size_t readSize) override;
};

class SoftI2C : public I2CDeviceHandle {
 private:
  soft_i2c_master_bus_t m_bus;
  uint8_t m_deviceAddr;

 public:
 SoftI2C(soft_i2c_master_bus_t bus, uint8_t addr)
   : m_bus(bus), m_deviceAddr(addr) {};
  ~SoftI2C() = default;
    
  esp_err_t transmit(const uint8_t *data, size_t size) override;
  esp_err_t receive(uint8_t *buffer, size_t readSize) override;
  esp_err_t transmitReceive(const uint8_t *data, size_t size,
			    uint8_t *buffer, size_t readSize) override;  
};

#endif  // I2CDEVICEHANDLE_H
