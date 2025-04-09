#include "include/mpu60x0.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "hal/i2c_types.h"
#include "portmacro.h"
#include "private/types.h"
#include "private/MadgwickAHRS.h"
#include <array>
#include <cstdint>
#include "esp_timer.h"
#include "esp_log.h"

Mpu::Mpu(i2c_master_bus_handle_t masterBusHandle, uint32_t masterFreq,
	 bool useAlternativeAddr, float filterPollFreqs,
	 float filterBeta) {
  uint16_t addr = (useAlternativeAddr) ? MPU60X0_ADDR1 : MPU60X0_ADDR0;

  i2c_device_config_t conf = {};
  conf.device_address = addr;
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.scl_speed_hz = masterFreq;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(
      masterBusHandle, &conf, &m_handle));

  m_madgwickFilter = new Madgwick(filterPollFreqs, filterBeta);

  m_filterTargetDelay = 1000000 / filterPollFreqs;

  // reset
  uint8_t data[] = {REG_PWR_MGMT_1, 0xF0};
  ESP_ERROR_CHECK(
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS));
  vTaskDelay(200 / portTICK_PERIOD_MS);
  
  // wakeup
  data[0] = REG_PWR_MGMT_1;
  data[1] = 0x0;		// TODO store device state
  ESP_ERROR_CHECK(
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS));
  calibrateGyro();
}

Mpu::~Mpu() {
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(m_handle));
  delete static_cast<Madgwick*>(m_madgwickFilter);
};


esp_err_t Mpu::getRawGyro(std::array<int16_t, 3> *out) {
  uint8_t buf[6];
  uint8_t reg = REG_GYRO_XOUT_H;
  
  esp_err_t r = i2c_master_transmit_receive(m_handle, &reg, 1, buf, 6,
                                            1000 / portTICK_PERIOD_MS);
  if (r != ESP_OK)
    return r;

  out->at(0) = uint16_t((buf[0] << 8) | buf[1]);
  out->at(1) = uint16_t((buf[2] << 8) | buf[3]);
  out->at(2) = uint16_t((buf[4] << 8) | buf[5]);
  
  return ESP_OK;    
};

esp_err_t Mpu::setGyroScale(mpu::gyroScale scale) {
  esp_err_t r;
  uint8_t data[2];
  data[0] = REG_GYRO_CONFIG;
  switch (scale) {
  case mpu::gyroScale::FS_500:
    data[1] = FS_SEL_500;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK)  m_gyroSens = 500.0f;
    break;
  case mpu::gyroScale::FS_1000:
    data[1] = FS_SEL_1000;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK)  m_gyroSens = 1000.0f;
    break;
  case mpu::gyroScale::FS_2000:
    data[1] = FS_SEL_2000;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_gyroSens = 2000.0f;
    break;
  default:
    data[1] = FS_SEL_250;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_gyroSens = 250.0f;
    break;
  }
  
  return r;
}

float Mpu::convertGyro(int16_t in) {
  return 1.0f * in /  m_gyroSens;
}

float Mpu::convertAcc(int16_t in) {
  return 1.0f * in / m_accSens;
}

esp_err_t Mpu::setAccScale(mpu::accScale scale) {
  esp_err_t r;
  uint8_t data[2];
  data[0] = REG_ACCEL_CONFIG;
  switch (scale) {
  case mpu::accScale::FS_4:
    data[1] = FS_SEL_4;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK)  m_accSens = 4.0f;
    break;
  case mpu::accScale::FS_8:
    data[1] = FS_SEL_8;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK)  m_accSens = 8.0f;
    break;
  case mpu::accScale::FS_16:
    data[1] = FS_SEL_16;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_accSens = 16.0f;
    break;
  default:
    data[1] = FS_SEL_2;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_accSens = 2.0f;
    break;
  }
  
  return r;
}

void Mpu::calibrateGyro() {
  ESP_LOGI("mpu", "calibrating...");
  
  float sumGyroX = 0.0f, sumGyroY = 0.0f, sumGyroZ = 0.0f;
  float sumAccX = 0.0f, sumAccY = 0.0f, sumAccZ = 0.0f;
  const int samples = 200;
  std::array<std::array<int16_t, 3>, samples> gi; // TODO
  std::array<std::array<int16_t, 3>, samples> ai;  

  for (int i = 0; i < samples; i++) {
    ESP_ERROR_CHECK(getRawGyro(&gi[i]));
    gi[i][0] = convertGyro(gi[i][0]);
    gi[i][1] = convertGyro(gi[i][1]);
    gi[i][2] = convertGyro(gi[i][2]);
    
    ESP_ERROR_CHECK(getRawAcc(&ai[i]));
    ai[i][0] = convertAcc(ai[i][0]);
    ai[i][1] = convertAcc(ai[i][1]);
    ai[i][2] = convertAcc(ai[i][2]);

    sumGyroX += gi[i][0];
    sumGyroY += gi[i][1];
    sumGyroZ += gi[i][2];
    
    sumAccX += ai[i][0];
    sumAccY += ai[i][1];
    sumAccZ += ai[i][2];    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  sumGyroX /= samples;
  sumGyroY /= samples;
  sumGyroZ /= samples;  
  
  sumAccX /= samples;
  sumAccY /= samples;
  sumAccZ /= samples;
  
  std::array<float, 3> gsd = {0,0,0}, asd={0,0,0};
  for (int i=0; i < samples; i++) {
    gsd[0] += pow(gi[i][0] - sumGyroX, 2);
    gsd[1] += pow(gi[i][1] - sumGyroY, 2);
    gsd[2] += pow(gi[i][2] - sumGyroZ, 2);

    asd[0] += pow(ai[i][0] - sumAccX, 2);
    asd[1] += pow(ai[i][1] - sumAccY, 2);
    asd[2] += pow(ai[i][2] - sumAccZ, 2);    
  }
  
  m_offsetGyro.x = sqrt(gsd[0] / (samples));
  m_offsetGyro.y = sqrt(gsd[1] / (samples));
  m_offsetGyro.z = sqrt(gsd[2] / (samples));

  m_offsetAcc.x = sqrt(asd[0] / (samples));
  m_offsetAcc.y = sqrt(asd[1] / (samples));
  m_offsetAcc.z = sqrt(asd[2] / (samples));

  ESP_LOGI("mpu", "calibrated");	       
}

void Mpu::update() {
  uint64_t now_us = esp_timer_get_time();
  uint64_t dt = now_us - m_lastUpdateTime;

  if (dt < m_filterTargetDelay)
    return;

  std::array<int16_t, 3> g;
  std::array<int16_t, 3> a;
  getRawGyro(&g);
  getRawAcc(&a);

  float gx = convertGyro(g[0]) - m_offsetGyro.x;
  float gy = convertGyro(g[1]) - m_offsetGyro.y;
  float gz = convertGyro(g[2]) - m_offsetGyro.z;

  float ax = convertAcc(a[0]) - m_offsetAcc.x;
  float ay = convertAcc(a[1]) - m_offsetAcc.y;
  float az = convertAcc(a[2]) - m_offsetAcc.z;

  static_cast<Madgwick *>(m_madgwickFilter)->updateIMU(gx, gy, gz, ax, ay, az);

  m_lastUpdateTime = now_us;
}

esp_err_t Mpu::getRawAcc(std::array<int16_t, 3> *out) {
  uint8_t buf[6];
  uint8_t reg = REG_ACCEL_XOUT_H;
  
  esp_err_t r = i2c_master_transmit_receive(m_handle, &reg, 1, buf, 6,
                                            1000 / portTICK_PERIOD_MS);
  if (r != ESP_OK)
    return r;

  out->at(0) = uint16_t((buf[0] << 8) | buf[1]);
  out->at(1) = uint16_t((buf[2] << 8) | buf[3]);
  out->at(2) = uint16_t((buf[4] << 8) | buf[5]);
  
  return ESP_OK;
}

float Mpu::getRoll(){
  return static_cast<Madgwick *>(m_madgwickFilter)->getRoll();
}

float Mpu::getPitch() {
  return static_cast<Madgwick *>(m_madgwickFilter)->getPitch();
}

float Mpu::getYaw() {
  return static_cast<Madgwick *>(m_madgwickFilter)->getYaw();
}
