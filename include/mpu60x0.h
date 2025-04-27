#ifndef MPU60X0_H
#define MPU60X0_H

#include <cstdint>
#include <stdint.h>
#include <array>
#include <variant>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "soft_i2c_master.h"

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

class I2CDeviceHandle;

class Mpu {
private:
  I2CDeviceHandle* m_i2cDeviceHandle;
  void* m_madgwickFilter;
  float m_gyroSens = 131.0f;	// LSB / deg/s
  float m_accSens = 16384.0f;	// LSB / g
  uint64_t m_lastUpdateTime = 0; // us
  float m_filterTargetDelay = 0;
  struct {
    float x = -0.27f, y = 0.24f, z = 0.16f; // raw
  } m_offsetGyro;
  struct {
    float x = 0, y = 0, z = 0;	// raw
  } m_offsetAcc;
  struct {
    int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
    int16_t accX = 0, accY = 0, accZ = 0;    
  } m_firstOrderIIR;
  uint8_t m_statePWR_MGMT_1 = 0x0;
  uint8_t m_statePWR_MGMT_2 = 0x0;

  float convertGyro(int16_t in);
  float convertAcc(int16_t in);
  esp_err_t rawGyro(std::array<int16_t, 3> *in);
  esp_err_t rawAcc(std::array<int16_t, 3> *in);  
  int16_t firstOrderIIR(int16_t in, int16_t& prev, float alpha = 0.5f);
public:
  /**
   * @brief construct Mpu class with Hardware I2C bus
   *
   * @param masterBusHandle handle for I2C master bus
   * @param masterFreq bus clock frequency
   * @param useAlternativeAddr if true Mpu will use 0x69 otherwise 0x68 for its identity
   * @param filterPoolFreqs frequency for updating madgwick filter, in Hz
   * @param filterBeta (more stable)0 ~ 1(more responsive) to a change
   */
  Mpu(i2c_master_bus_handle_t masterBusHandle, uint32_t masterFreq = 400000,
      bool useAlternativeAddr = false, float filterPollFreqs = 512.0f,
      float filterBeta = 0.5f);
   /**
   * @brief construct Mpu class with Software I2C bus
   *
   * @param masterBusHandle handle for I2C master bus
   * @param useAlternativeAddr if true Mpu will use 0x69 otherwise 0x68 for its identity
   * @param filterPoolFreqs frequency for updating madgwick filter, in Hz
   * @param filterBeta (more stable)0 ~ 1(more responsive) to a change
   */
  Mpu(soft_i2c_master_bus_t masterBusHandle, bool useAlternativeAddr = false,
      float filterPollFreqs = 512.0f, float filterBeta = 0.5f);
  ~Mpu();

  void calibrate();
  /**
   * @brief update the madgwick filter for calculating euler angles. all axis are needed.
   */
  void update();
  /**
   * @brief reset all settings of mpu
   */
  esp_err_t reset();

  /**
   * @brief shortcut for waking up, enable cycle mode, and disable temperature
   *
   * @param freq cycle mode wake up frequency 
   */
  esp_err_t enableLowPowerMode(mpu::wakeupFreq freq);
  /**
   * @brief when enabled Mpu will cycles between sleep mode and waking up
   * to take a single sample of data 
   */
  esp_err_t enableCycleMode(bool v);
  /**
   * @brief sleep
   *
   * @param v true = enable. false = disable
   */ 
  esp_err_t enableSleepMode(bool v);

  /**
   * @brief disable Mpu temperature checking
   *
   * @param v true = disable
   */
  esp_err_t disableTemp(bool v);
  /**
   * @brief disable mpu for updating Gyro value for specific axis
   * Raw value of Disabled axis might be on stale value or zero
   *
   * @param v true = disable, false = enable
   * @param flags target gyro axis
   */
  esp_err_t disableGyro(bool v, uint8_t flags);
  /**
   * @brief disable mpu for updating Accelerometer value for specific axis
   * Raw value of Disabled axis might be on stale value or zero   
   *
   * @param v true = disable, false = enable
   * @param flags target acc axis
   */  
  esp_err_t disableAcc(bool v, uint8_t flags);
  
  /**
   * @brief in deg
   */
  float getRoll();
  float getPitch();
  /**
   * @brief need magnetometer
   */
  float getYaw();
  /**
   * @brief in deg/s
   */
  esp_err_t getGyro(std::array<float, 3> *out);
  /**
   * @brief in g
   */  
  esp_err_t getAcc(std::array<float, 3> *out);
  /**
   * @brief in celcius
   */
  esp_err_t getTemp(float *out);
  
  /**
   * @brief set scale of Gyroscope. might need to recalibrate
   */
  esp_err_t setGyroScale(mpu::gyroScale scale);
  /**
   * @brief set scale of Gyroscope. might need to recalibrate
   */  
  esp_err_t setAccScale(mpu::accScale scale);  
};

#endif	// MPU60X0_H
