#ifndef MPU60X0_H
#define MPU60x0_H

#include <stdint.h>
#include <array>
#include "esp_err.h"

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

class Mpu {
private:
  i2c_master_dev_handle_t m_handle;
  
public:
  Mpu(i2c_master_bus_handle_t masterBusHandle, bool useAlternativeAddr = false,
      uint32_t masterFreq = 400000);
  ~Mpu();

  esp_err_t getRawGyro(std::array<int16_t, 3> *out);
  i2c_master_dev_handle_t getHandle() { return m_handle; };
};

#endif	// MPU60X0_H
