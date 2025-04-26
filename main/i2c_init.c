#include "i2c_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

i2c_master_bus_handle_t vl53l0x_i2c_bus;

void init_i2c_master_vl53l0x(void) {
    const i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = VL53L0X_SDA_GPIO,
        .scl_io_num = VL53L0X_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &vl53l0x_i2c_bus));
    ESP_LOGI("I2C_INIT", "VL53L0X I2C master bus initialized on port 0 (SDA=%d, SCL=%d)", VL53L0X_SDA_GPIO, VL53L0X_SCL_GPIO);
}

void vl53l0x_hard_reset(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VL53L0X_XSHUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    gpio_set_level(VL53L0X_XSHUT_GPIO, 0); // Hold reset
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(VL53L0X_XSHUT_GPIO, 1); // Power up
    vTaskDelay(pdMS_TO_TICKS(10));        // Wait for boot
    ESP_LOGI("I2C_INIT", "VL53L0X XSHUT toggled HIGH");
}
