#include <stdio.h>
#include "driver/i2c_master.h"
#include "soc/gpio_num.h"
#include "soft_i2c_master.h"
#include "esp_log.h"
#include "mpu60x0.h"

extern "C" void app_main(void) {
  i2c_master_bus_handle_t hardI2c;
  soft_i2c_master_bus_t softI2c;
  
  {
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.scl_io_num = GPIO_NUM_4; 
    i2c_mst_config.sda_io_num = GPIO_NUM_5;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &hardI2c));
  }

  {
    soft_i2c_master_config_t conf = {};
    conf.scl_pin = GPIO_NUM_9;
    conf.sda_pin = GPIO_NUM_10;
    conf.freq = SOFT_I2C_200KHZ;

    ESP_ERROR_CHECK(soft_i2c_master_new(&conf, &softI2c));
  }

  Mpu mpu1(hardI2c, 400000, false, 512.0f, 0.8f);
  Mpu mpu2(softI2c, false, 512.0f, 0.8f);

  while (1) {
    mpu1.update();
    mpu2.update();

    ESP_LOGI("MAIN", "mpu1 pitch: %f | mpu2 pitcj: %f", mpu1.getPitch(), mpu2.getPitch());
  }
}
