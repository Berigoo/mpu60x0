#include "private/I2CDeviceHandle.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "portmacro.h"
#include "soft_i2c_master.h"
#include <cstdint>


HardI2C::HardI2C(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t masterFreq) {
  i2c_device_config_t conf = {};
  conf.device_address = addr;
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.scl_speed_hz = masterFreq;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &conf, &m_device));
}
esp_err_t HardI2C::transmit(const uint8_t *data, size_t size) {
  return i2c_master_transmit(m_device, data, size, 1000 / portTICK_PERIOD_MS);
}
esp_err_t HardI2C::receive(uint8_t *buffer, size_t readSize) {
  return i2c_master_receive(m_device, buffer, readSize,
                            1000 / portTICK_PERIOD_MS);
}
esp_err_t HardI2C::transmitReceive(const uint8_t *data, size_t size,
                                   uint8_t *buffer, size_t readSize) {
  return i2c_master_transmit_receive(m_device, data, size, buffer, readSize,
                                     1000 / portTICK_PERIOD_MS);
}
HardI2C::~HardI2C() {
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(m_device));
}



esp_err_t SoftI2C::transmit(const uint8_t *data, size_t size) {
  return soft_i2c_master_write(m_bus, m_deviceAddr, data, size);
}
esp_err_t SoftI2C::receive(uint8_t *buffer, size_t readSize) {
  return soft_i2c_master_read(m_bus, m_deviceAddr, buffer, readSize);
}
esp_err_t SoftI2C::transmitReceive(const uint8_t *data, size_t size,
                                   uint8_t *buffer, size_t readSize) {
  return soft_i2c_master_write_read(m_bus, m_deviceAddr, data, size, buffer,
                                    readSize);
}




