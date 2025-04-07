#include "include/mpu60x0.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/i2c_types.h"
#include "portmacro.h"
#include "private/types.h"
#include <cstdint>
#include "esp_log.h"

Mpu::Mpu(i2c_master_bus_handle_t masterBusHandle,
         bool useAlternativeAddr, uint32_t masterFreq){
  uint16_t addr = (useAlternativeAddr) ? MPU60X0_ADDR1 : MPU60X0_ADDR0;

  i2c_device_config_t conf = {};
  conf.device_address = addr;
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.scl_speed_hz = masterFreq;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      masterBusHandle, &conf, &m_handle));

  uint8_t data[] = {REG_PWR_MGMT_1, 0b10000000};
  ESP_ERROR_CHECK(
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS));

  ESP_LOGI("mpu", "address: %02x", addr);
}

Mpu::~Mpu() {
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(m_handle));
}

esp_err_t Mpu::readAccX(int16_t *out) {
  uint8_t buf[2];
  uint8_t reg = REG_ACCEL_XOUT_H;

  esp_err_t r = i2c_master_transmit_receive(m_handle, &reg, 1, buf, 2,
                                            1000 / portTICK_PERIOD_MS);
  if (r != ESP_OK)
    return r;
  
  *out = uint16_t((buf[0] << 8) | buf[1]);
  return ESP_OK;
};
