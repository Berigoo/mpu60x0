#include "include/mpu60x0.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
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
  m_statePWR_MGMT_1 = 0xF0;
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};
  ESP_ERROR_CHECK(
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS));
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // wakeup
  m_statePWR_MGMT_1 = 0x0;
  data[0] = REG_PWR_MGMT_1;
  data[1] = m_statePWR_MGMT_1;
  ESP_ERROR_CHECK(
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS));
  calibrate();
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

  out->at(0) = int16_t((buf[0] << 8) | buf[1]);
  out->at(1) = int16_t((buf[2] << 8) | buf[3]);
  out->at(2) = int16_t((buf[4] << 8) | buf[5]);
  
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
    if (r == ESP_OK)  m_gyroSens = 65.5f;
    break;
  case mpu::gyroScale::FS_1000:
    data[1] = FS_SEL_1000;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK)  m_gyroSens = 32.8f;
    break;
  case mpu::gyroScale::FS_2000:
    data[1] = FS_SEL_2000;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_gyroSens = 16.4f;
    break;
  default:
    data[1] = FS_SEL_250;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_gyroSens = 131.0f;
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
    if (r == ESP_OK)  m_accSens = 8192.0f;
    break;
  case mpu::accScale::FS_8:
    data[1] = FS_SEL_8;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK)  m_accSens = 4096.0f; 
    break;
  case mpu::accScale::FS_16:
    data[1] = FS_SEL_16;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_accSens = 2048.0f;
    break;
  default:
    data[1] = FS_SEL_2;
    r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
    if (r == ESP_OK) m_accSens = 16384.0f;
    break;
  }
  
  return r;
}

void Mpu::calibrate(){
  ESP_LOGI("mpu", "calibrating...");
  
  float sumGyroX = 0.0f, sumGyroY = 0.0f, sumGyroZ = 0.0f;
  float sumAccX = 0.0f, sumAccY = 0.0f, sumAccZ = 0.0f;
  const int samples = 200;
  auto gi = new std::array<std::array<int16_t, 3>, samples>{};
  auto ai = new std::array<std::array<int16_t, 3>, samples> {};  

  for (int i = 0; i < samples; i++) {
    ESP_ERROR_CHECK(getRawGyro(&gi->at(i)));
    gi->at(i)[0] = convertGyro(gi->at(i)[0]);
    gi->at(i)[1] = convertGyro(gi->at(i)[1]);
    gi->at(i)[2] = convertGyro(gi->at(i)[2]);
    
    ESP_ERROR_CHECK(getRawAcc(&ai->at(i)));
    ai->at(i)[0] = convertAcc(ai->at(i)[0]);
    ai->at(i)[1] = convertAcc(ai->at(i)[1]);
    ai->at(i)[2] = convertAcc(ai->at(i)[2]);

    sumGyroX += gi->at(i)[0];
    sumGyroY += gi->at(i)[1];
    sumGyroZ += gi->at(i)[2];
    
    sumAccX += ai->at(i)[0];
    sumAccY += ai->at(i)[1];
    sumAccZ += ai->at(i)[2];    
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
    gsd[0] += pow(gi->at(i)[0] - sumGyroX, 2);
    gsd[1] += pow(gi->at(i)[1] - sumGyroY, 2);
    gsd[2] += pow(gi->at(i)[2] - sumGyroZ, 2);

    asd[0] += pow(ai->at(i)[0] - sumAccX, 2);
    asd[1] += pow(ai->at(i)[1] - sumAccY, 2);
    asd[2] += pow(ai->at(i)[2] - sumAccZ, 2);    
  }
  
  m_offsetGyro.x = sqrt(gsd[0] / (samples));
  m_offsetGyro.y = sqrt(gsd[1] / (samples));
  m_offsetGyro.z = sqrt(gsd[2] / (samples));

  m_offsetAcc.x = sqrt(asd[0] / (samples));
  m_offsetAcc.y = sqrt(asd[1] / (samples));
  m_offsetAcc.z = sqrt(asd[2] / (samples));

  delete gi;
  delete ai;
  
  ESP_LOGI("mpu", "calibrated");	       
}

void Mpu::update() {
  uint64_t now_us = esp_timer_get_time();
  uint64_t dt = now_us - m_lastUpdateTime;

  if (dt < m_filterTargetDelay)
    return;

  if (!(m_statePWR_MGMT_2 & 0x4F)){
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
  }


  m_lastUpdateTime = now_us;
}

esp_err_t Mpu::getRawAcc(std::array<int16_t, 3> *out) {
  uint8_t buf[6];
  uint8_t reg = REG_ACCEL_XOUT_H;
  
  esp_err_t r = i2c_master_transmit_receive(m_handle, &reg, 1, buf, 6,
                                            1000 / portTICK_PERIOD_MS);
  if (r != ESP_OK)
    return r;

  out->at(0) = int16_t((buf[0] << 8) | buf[1]);
  out->at(1) = int16_t((buf[2] << 8) | buf[3]);
  out->at(2) = int16_t((buf[4] << 8) | buf[5]);
  
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

esp_err_t Mpu::enableCycleMode(bool v) {
  if (v) {
    m_statePWR_MGMT_1 |= PWR_MGMT_CYCLE_MODE;
  } else {
    m_statePWR_MGMT_1 &= ~PWR_MGMT_CYCLE_MODE;
  }
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};
  
  esp_err_t r =
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);

  return r;
}

esp_err_t Mpu::enableSleepMode(bool v) {
  if (v) {
    m_statePWR_MGMT_1 |= PWR_MGMT_SLEEP_MODE;
  } else {
    m_statePWR_MGMT_1 &= ~PWR_MGMT_SLEEP_MODE;
  }
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};

  esp_err_t r =
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);

  return r;  
}

esp_err_t Mpu::reset() {
  m_statePWR_MGMT_1 = 0x80;
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};

  esp_err_t r =
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);

  if (r != ESP_OK)
    return r;
  vTaskDelay(200 / portTICK_PERIOD_MS);

  m_statePWR_MGMT_1 = 0x0;
  data[1] = m_statePWR_MGMT_1;
  r = i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);
  
  return r;
}

esp_err_t Mpu::disableTemp(bool v) {
  if (v) {
    m_statePWR_MGMT_1 |= PWR_MGMT_DISABLE_TEMP;
  } else {
    m_statePWR_MGMT_1 &= ~PWR_MGMT_DISABLE_TEMP;
  }    
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};

  esp_err_t r =
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);

  return r;
}

esp_err_t Mpu::enableLowPowerMode(mpu::wakeupFreq freq) {
  m_statePWR_MGMT_1 |=
      (PWR_MGMT_CYCLE_MODE & ~PWR_MGMT_SLEEP_MODE) | PWR_MGMT_DISABLE_TEMP;
  switch (freq) {
  case mpu::wakeupFreq::FS_5_HZ:
    m_statePWR_MGMT_2 |= PWR_MGMT_WK_FREQ_5;
    break;
  case mpu::wakeupFreq::FS_20_HZ:
    m_statePWR_MGMT_2 |= PWR_MGMT_WK_FREQ_20;
    break;
  case mpu::wakeupFreq::FS_40_HZ:
    m_statePWR_MGMT_2 |= PWR_MGMT_WK_FREQ_40;
    break;
  default:
    m_statePWR_MGMT_2 |= PWR_MGMT_WK_FREQ_1_25;
  }
  
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1, m_statePWR_MGMT_2};

  esp_err_t r =
      i2c_master_transmit(m_handle, data, 3, 1000 / portTICK_PERIOD_MS);

  return r;  
}

esp_err_t Mpu::disableGyro(bool v, uint8_t flags) {
  uint8_t accumulate = 0x0;
  if (flags & mpu::disableAxis::GYRO_X)
    accumulate |= mpu::disableAxis::GYRO_X;
  if (flags & mpu::disableAxis::GYRO_Y)
    accumulate |= mpu::disableAxis::GYRO_Y;
  if (flags & mpu::disableAxis::GYRO_Z)
    accumulate |= mpu::disableAxis::GYRO_Z;
  if (v) {
    m_statePWR_MGMT_2 |= accumulate;
  } else {
    m_statePWR_MGMT_2 &= ~accumulate;
  }    
  uint8_t data[] = {REG_PWR_MGMT_2, m_statePWR_MGMT_2};

  esp_err_t r =
      i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);

  return r;
}

esp_err_t Mpu::disableAcc(bool v, uint8_t flags) {
  uint8_t accumulate = 0x0;
  if (flags & mpu::disableAxis::ACC_X)
    accumulate |= mpu::disableAxis::ACC_X;
  if (flags & mpu::disableAxis::ACC_Y)
    accumulate |= mpu::disableAxis::ACC_Y;
  if (flags & mpu::disableAxis::ACC_Z)
    accumulate |= mpu::disableAxis::ACC_Z;  
  if (v) {
    m_statePWR_MGMT_2 |= accumulate;
  } else {
    m_statePWR_MGMT_2 &= ~accumulate;
  }      
  uint8_t data[] = {REG_PWR_MGMT_2, m_statePWR_MGMT_2};

  esp_err_t r =
    i2c_master_transmit(m_handle, data, 2, 1000 / portTICK_PERIOD_MS);

  return r;
}

esp_err_t Mpu::getRawTemp(int16_t *out) {
  uint8_t buf[2];
  uint8_t reg = REG_TEMP_OUT_H;
  
  esp_err_t r = i2c_master_transmit_receive(m_handle, &reg, 1, buf, 2,
                                            1000 / portTICK_PERIOD_MS);
  if (r != ESP_OK)
    return r;

  *out = int16_t((buf[0] << 8) | buf[1]);

  return ESP_OK;
}

float Mpu::getTemp() {
  int16_t o;
  getRawTemp(&o);
  return (o / 340) + 36.53f;
}
