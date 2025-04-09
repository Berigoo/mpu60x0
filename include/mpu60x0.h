#ifndef MPU60X0_H
#define MPU60x0_H

#include <stdint.h>
#include <array>
#include "esp_err.h"
#include <any>

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

namespace mpu {
enum gyroScale { FS_250, FS_500, FS_1000, FS_2000 };
enum accScale { FS_2, FS_4, FS_8, FS_16 };
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

  float convertGyro(int16_t in);
  float convertAcc(int16_t in);
public:
  Mpu(i2c_master_bus_handle_t masterBusHandle, uint32_t masterFreq = 400000,
      bool useAlternativeAddr = false, float filterPollFreqs = 512.0f,
      float filterBeta = 0.1f);
  
  ~Mpu();

  void calibrate();
  void update();

  i2c_master_dev_handle_t getHandle() { return m_handle; };
  float getRoll();
  float getPitch();
  float getYaw();
  esp_err_t getRawGyro(std::array<int16_t, 3> *out);
  esp_err_t getRawAcc(std::array<int16_t, 3> *out);
  
  esp_err_t setGyroScale(mpu::gyroScale scale);
  esp_err_t setAccScale(mpu::accScale scale);  
};

#endif	// MPU60X0_H
