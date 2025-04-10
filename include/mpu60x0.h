#ifndef MPU60X0_H
#define MPU60x0_H

#include <stdint.h>
#include <array>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

namespace mpu {
enum gyroScale { FS_250, FS_500, FS_1000, FS_2000 };
enum accScale { FS_2, FS_4, FS_8, FS_16 };
enum wakeupFreq { FS_1_25HZ, FS_5_HZ, FS_20_HZ, FS_40_HZ };
enum disableAxis {
  GYRO_X = 0x04,
  GYRO_Y = 0x02,
  GYRO_Z = 0x01,
  ACC_X = 0x20,
  ACC_Y = 0x10,
  ACC_Z = 0x08
};
// enum class disableGyroAxis { X = 0x04, Y = 0x02, Z = 0x01 };
// enum class disableAccAxis { X = 0x20, Y = 0x10, Z = 0x08 };
};

class Mpu {
private:
  i2c_master_dev_handle_t m_handle;
  void* m_madgwickFilter;
  float m_gyroSens = 131.0f;	// LSB / deg/s
  float m_accSens = 16384.0f;	// LSB / g
  uint64_t m_lastUpdateTime = 0; // us
  float m_filterTargetDelay = 0;
  struct {
    float x = -0.27f, y = 0.24f, z = 0.16f; // deg/s
  } m_offsetGyro;
  struct {
    float x = 0, y = 0, z = 0;	// g
  } m_offsetAcc;
  uint8_t m_statePWR_MGMT_1 = 0x0;
  uint8_t m_statePWR_MGMT_2 = 0x0;

  float convertGyro(int16_t in);
  float convertAcc(int16_t in);
public:
  Mpu(i2c_master_bus_handle_t masterBusHandle, uint32_t masterFreq = 400000,
      bool useAlternativeAddr = false, float filterPollFreqs = 512.0f,
      float filterBeta = 0.1f);
  ~Mpu();


  
  void calibrate();
  void update();
  esp_err_t reset();

  esp_err_t enableLowPowerMode(mpu::wakeupFreq freq);
  esp_err_t enableCycleMode(bool v);
  esp_err_t enableSleepMode(bool v);

  esp_err_t disableTemp(bool v);
  esp_err_t disableGyro(bool v, uint8_t flags);
  esp_err_t disableAcc(bool v, uint8_t flags);
  
  i2c_master_dev_handle_t getHandle() { return m_handle; };
  float getRoll();
  float getPitch();
  float getYaw();
  float getTemp();
  esp_err_t getRawGyro(std::array<int16_t, 3> *out);
  esp_err_t getRawAcc(std::array<int16_t, 3> *out);
  esp_err_t getRawTemp(int16_t *out);
  
  esp_err_t setGyroScale(mpu::gyroScale scale);
  esp_err_t setAccScale(mpu::accScale scale);  
};

#endif	// MPU60X0_H
