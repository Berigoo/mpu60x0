#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "hal/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "mpu60x0.h"
#include "unity.h"
#include "esp_log.h"
#include <array>

static void init(i2c_master_bus_handle_t* out) {
  i2c_master_bus_config_t i2c_mst_config = {};
  i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_mst_config.i2c_port = I2C_NUM_0;
  i2c_mst_config.scl_io_num = GPIO_NUM_4; // TODO kconfig
  i2c_mst_config.sda_io_num = GPIO_NUM_5;
  i2c_mst_config.glitch_ignore_cnt = 7;
  i2c_mst_config.flags.enable_internal_pullup = true;

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, out));    
}

TEST_CASE("Init", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    Mpu mpu(bus_handle);
    TEST_ASSERT_NOT_EQUAL(nullptr, mpu.getHandle());
  }

  i2c_del_master_bus(bus_handle);
};

TEST_CASE("PROBING", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    esp_err_t r1 =
      i2c_master_probe(bus_handle, 0x68, 1000 / portTICK_PERIOD_MS);
    esp_err_t r2 =
        i2c_master_probe(bus_handle, 0x69, 1000 / portTICK_PERIOD_MS);
    TEST_ASSERT_EQUAL(true, (r1 == ESP_OK || r2 == ESP_OK));
  }

  i2c_del_master_bus(bus_handle);
}

TEST_CASE("GYRO", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    Mpu mpu(bus_handle);
    std::array<int16_t, 3> out = {-1, -1, -1};
    TEST_ASSERT_EQUAL(ESP_OK, mpu.getRawGyro(&out));
    ESP_LOGI("mpu", "out: %d, %d, %d", out[0], out[1], out[2]);
  }

  i2c_del_master_bus(bus_handle);
}

TEST_CASE("ACC", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    Mpu mpu(bus_handle);
    std::array<int16_t, 3> out = {-1, -1, -1};
    TEST_ASSERT_EQUAL(ESP_OK, mpu.getRawAcc(&out));
    ESP_LOGI("mpu", "out: %d, %d, %d", out[0], out[1], out[2]);
  }

  i2c_del_master_bus(bus_handle);
}

TEST_CASE("LOW MODE", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    Mpu mpu(bus_handle);
    TEST_ASSERT_EQUAL(ESP_OK,
                      mpu.enableLowPowerMode(mpu::wakeupFreq::FS_20_HZ));
    TEST_ASSERT_EQUAL(ESP_OK, mpu.enableCycleMode(false));
    TEST_ASSERT_EQUAL(ESP_OK, mpu.disableTemp(true));
    vTaskDelay(200 / portTICK_PERIOD_MS);
    int16_t o;
    mpu.getRawTemp(&o);
    int16_t staleVal = o;
    vTaskDelay(200 / portTICK_PERIOD_MS);
    mpu.getRawTemp(&o);
    TEST_ASSERT(o == 0 || o == staleVal);
  }

  i2c_del_master_bus(bus_handle);
}

TEST_CASE("SLEEP MODE", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    Mpu mpu(bus_handle);
    mpu.enableSleepMode(true);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    std::array<int16_t, 3> out;
    mpu.getRawGyro(&out);
    int16_t staleVal = out[0];
    vTaskDelay(200 / portTICK_PERIOD_MS);
    mpu.getRawGyro(&out);
    TEST_ASSERT(out[0] == staleVal || out[0] == 0);
    mpu.enableSleepMode(false);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    mpu.getRawGyro(&out);
    TEST_ASSERT(!(out[0] == staleVal || out[0] == 0));
    
  }

  i2c_del_master_bus(bus_handle);
}

TEST_CASE("DISABLE GYRO AXIS", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    Mpu mpu(bus_handle);
    mpu.disableGyro(true, mpu::disableAxis::GYRO_X | mpu::disableAxis::GYRO_Y |
                              mpu::disableAxis::GYRO_Z);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    std::array<int16_t, 3> staleVal;
    mpu.getRawGyro(&staleVal);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    std::array<int16_t, 3> val;
    mpu.getRawGyro(&val);
    TEST_ASSERT((val[0] == staleVal[0] && val[1] == staleVal[1] &&
                 val[2] == staleVal[2]) ||
                (val[0] == 0 && val[1] == 0 && val[2] == 0));
  }

  i2c_del_master_bus(bus_handle);
}

TEST_CASE("DISABLE ACC AXIS", "[mpu]") {
  i2c_master_bus_handle_t bus_handle;
  init(&bus_handle);
  
  {
    Mpu mpu(bus_handle);
    mpu.disableAcc(true, mpu::disableAxis::ACC_X | mpu::disableAxis::ACC_Y |
                              mpu::disableAxis::ACC_Z);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    std::array<int16_t, 3> staleVal;
    mpu.getRawAcc(&staleVal);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    std::array<int16_t, 3> val;
    mpu.getRawAcc(&val);
    TEST_ASSERT((val[0] == staleVal[0] && val[1] == staleVal[1] &&
                 val[2] == staleVal[2]) ||
                (val[0] == 0 && val[1] == 0 && val[2] == 0));
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }

  i2c_del_master_bus(bus_handle);
}
