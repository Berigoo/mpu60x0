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

// TEST_CASE("ACCEL X", "[mpu]") {
//   i2c_master_bus_handle_t bus_handle;
//   init(&bus_handle);
  
//   {
//     Mpu mpu(bus_handle);

//     int16_t o;
//     TEST_ASSERT_EQUAL(ESP_OK, mpu.readAccX(&o));
//     ESP_LOGI("[mpu]", "reading value: %d", o);    
//   }

//   i2c_del_master_bus(bus_handle);
// }

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

// TEST_CASE("GYRO X", "[mpu]") {
//   i2c_master_bus_handle_t bus_handle;
//   init(&bus_handle);
  
//   {
//     Mpu mpu(bus_handle);
//     int16_t out;
//     TEST_ASSERT_EQUAL(ESP_OK, mpu.getRawGyroX(&out));
//     ESP_LOGI("mpu", "out: %d", out);
//   }

//   i2c_del_master_bus(bus_handle);  
// }
