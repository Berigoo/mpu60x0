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
#include "private/I2CDeviceHandle.h"
#include <array>
#include <cstdint>
#include "esp_timer.h"
#include "esp_log.h"
#include "soft_i2c_master.h"

Mpu::Mpu(i2c_master_bus_handle_t masterBusHandle, uint32_t masterFreq,
	 bool useAlternativeAddr, float filterPollFreqs,
	 float filterBeta) {
  uint16_t addr = (useAlternativeAddr) ? MPU60X0_ADDR1 : MPU60X0_ADDR0;

  m_i2cDeviceHandle = new HardI2C(masterBusHandle, addr, masterFreq);

  m_madgwickFilter = new Madgwick(filterPollFreqs, filterBeta);

  m_filterTargetDelay = 1000000 / filterPollFreqs;

  // reset
  m_statePWR_MGMT_1 = 0xF0;
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};
  ESP_ERROR_CHECK(
      m_i2cDeviceHandle->transmit(data, 2));
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // wakeup
  m_statePWR_MGMT_1 = 0x0;
  data[0] = REG_PWR_MGMT_1;
  data[1] = m_statePWR_MGMT_1;
  ESP_ERROR_CHECK(
      m_i2cDeviceHandle->transmit(data, 2));
  calibrate();
}

Mpu::Mpu(soft_i2c_master_bus_t masterBusHandle, uint32_t masterFreq,
	 bool useAlternativeAddr, float filterPollFreqs,
	 float filterBeta) {
  uint16_t addr = (useAlternativeAddr) ? MPU60X0_ADDR1 : MPU60X0_ADDR0;

  m_i2cDeviceHandle = new SoftI2C(masterBusHandle, addr);

  m_madgwickFilter = new Madgwick(filterPollFreqs, filterBeta);

  m_filterTargetDelay = 1000000 / filterPollFreqs;

  // reset
  m_statePWR_MGMT_1 = 0xF0;
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};
  ESP_ERROR_CHECK(
      m_i2cDeviceHandle->transmit(data, 2));
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // wakeup
  m_statePWR_MGMT_1 = 0x0;
  data[0] = REG_PWR_MGMT_1;
  data[1] = m_statePWR_MGMT_1;
  ESP_ERROR_CHECK(
      m_i2cDeviceHandle->transmit(data, 2));
  calibrate();
}

Mpu::~Mpu() {
  delete m_i2cDeviceHandle;
  delete static_cast<Madgwick*>(m_madgwickFilter);
};


esp_err_t Mpu::getRawGyro(std::array<int16_t, 3> *out) {
  uint8_t buf[6];
  uint8_t reg = REG_GYRO_XOUT_H;
  
  esp_err_t r = m_i2cDeviceHandle->transmitReceive(&reg, 1, buf, 6);

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
    r = m_i2cDeviceHandle->transmit(data, 2);
    if (r == ESP_OK)  m_gyroSens = 65.5f;
    break;
  case mpu::gyroScale::FS_1000:
    data[1] = FS_SEL_1000;
    r = m_i2cDeviceHandle->transmit(data, 2);
    if (r == ESP_OK)  m_gyroSens = 32.8f;
    break;
  case mpu::gyroScale::FS_2000:
    data[1] = FS_SEL_2000;
    r = m_i2cDeviceHandle->transmit(data, 2);
    if (r == ESP_OK) m_gyroSens = 16.4f;
    break;
  default:
    data[1] = FS_SEL_250;
    r = m_i2cDeviceHandle->transmit(data, 2);
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
    r = m_i2cDeviceHandle->transmit(data, 2);
    if (r == ESP_OK)  m_accSens = 8192.0f;
    break;
  case mpu::accScale::FS_8:
    data[1] = FS_SEL_8;
    r = m_i2cDeviceHandle->transmit(data, 2);
    if (r == ESP_OK)  m_accSens = 4096.0f; 
    break;
  case mpu::accScale::FS_16:
    data[1] = FS_SEL_16;
    r = m_i2cDeviceHandle->transmit(data, 2);
    if (r == ESP_OK) m_accSens = 2048.0f;
    break;
  default:
    data[1] = FS_SEL_2;
    r = m_i2cDeviceHandle->transmit(data, 2);
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

  for (int i = 0; i < samples; i++) {
    std::array<int16_t, 3> g;
    std::array<int16_t, 3> a;
    ESP_ERROR_CHECK(getRawGyro(&g));
    ESP_ERROR_CHECK(getRawGyro(&a));

    sumGyroX += g[0];
    sumGyroY += g[1];
    sumGyroZ += g[2];
    
    sumAccX += a[0];
    sumAccY += a[1];
    sumAccZ += a[2];

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  m_offsetGyro.x = sumGyroX / samples;
  m_offsetGyro.y = sumGyroY / samples;
  m_offsetGyro.z = sumGyroZ / samples;

  m_offsetAcc.x = sumAccX / samples;
  m_offsetAcc.y = sumAccY / samples;
  m_offsetAcc.z = sumAccZ / samples;
  
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

    float gx = convertGyro(
        firstOrderIIR(g[0] - m_offsetGyro.x, m_firstOrderIIR.gyroX));

    float gy = convertGyro(
        firstOrderIIR(g[1] - m_offsetGyro.y, m_firstOrderIIR.gyroY));

    float gz = convertGyro(
        firstOrderIIR(g[2] - m_offsetGyro.z, m_firstOrderIIR.gyroZ));


    float ax =
        convertAcc(firstOrderIIR(a[0] - m_offsetAcc.x, m_firstOrderIIR.accX));

    float ay =
        convertAcc(firstOrderIIR(a[1] - m_offsetAcc.y, m_firstOrderIIR.accY));

    float az =
        convertAcc(firstOrderIIR(a[2] - m_offsetAcc.z, m_firstOrderIIR.accZ));
    
    static_cast<Madgwick *>(m_madgwickFilter)->updateIMU(gx, gy, gz, ax, ay, az);
  }


  m_lastUpdateTime = now_us;
}

esp_err_t Mpu::getRawAcc(std::array<int16_t, 3> *out) {
  uint8_t buf[6];
  uint8_t reg = REG_ACCEL_XOUT_H;

  esp_err_t r = m_i2cDeviceHandle->transmitReceive(&reg, 1, buf, 6);
  
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
      m_i2cDeviceHandle->transmit(data, 2);

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
      m_i2cDeviceHandle->transmit(data, 2);

  return r;  
}

esp_err_t Mpu::reset() {
  m_statePWR_MGMT_1 = 0x80;
  uint8_t data[] = {REG_PWR_MGMT_1, m_statePWR_MGMT_1};

  esp_err_t r =
      m_i2cDeviceHandle->transmit(data, 2);

  if (r != ESP_OK)
    return r;
  vTaskDelay(200 / portTICK_PERIOD_MS);

  m_statePWR_MGMT_1 = 0x0;
  m_statePWR_MGMT_2 = 0x0;
  data[1] = m_statePWR_MGMT_1;
  r = m_i2cDeviceHandle->transmit(data, 2);
  
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
      m_i2cDeviceHandle->transmit(data, 2);

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
      m_i2cDeviceHandle->transmit(data, 3);

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
      m_i2cDeviceHandle->transmit(data, 2);

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
    m_i2cDeviceHandle->transmit(data, 2);

  return r;
}

esp_err_t Mpu::getRawTemp(int16_t *out) {
  uint8_t buf[2];
  uint8_t reg = REG_TEMP_OUT_H;

  esp_err_t r = m_i2cDeviceHandle->transmitReceive(&reg, 1, buf, 2);
    
  if (r != ESP_OK)
    return r;

  *out = int16_t((buf[0] << 8) | buf[1]);

  return ESP_OK;
}

float Mpu::getTemp() {
  int16_t o;
  getRawTemp(&o);
  return (o / 340.0f) /* + 36.53f */;
}

int16_t Mpu::firstOrderIIR(int16_t in, int16_t& prev, float alpha) {
  int16_t out = ((1 - alpha) * in) + (alpha * prev);
  prev = out;
  return out;
}
