#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h" // Include the header where VL53L0X_DEV is defined
#include "vl53l0x_types.h" // Ensure this header defines VL53L0X_DEV
#include "vl53l0x_platform_log.h"
#include "driver/i2c.h"
#include "vl53l0x_platform.h"
#include "esp_log.h"
#include <string.h>

#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_SCL_IO 3
#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_FREQ_HZ 400000

static const char *TAG = "VL53L0X_I2C";

esp_err_t vl53l0x_i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    return ESP_OK;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t reg, uint8_t *pdata, uint32_t count) {
    uint8_t *buffer = malloc(count + 1);
    if (!buffer) return VL53L0X_ERROR_CONTROL_INTERFACE;
    buffer[0] = reg;
    memcpy(&buffer[1], pdata, count);

    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, dev->I2cDevAddr, buffer, count + 1, 1000 / portTICK_PERIOD_MS);
    free(buffer);

    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t reg, uint8_t *pdata, uint32_t count) {
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, dev->I2cDevAddr, &reg, 1, pdata, count, 1000 / portTICK_PERIOD_MS);
    return (ret == ESP_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WriteByte(VL53L0X_DEV dev, uint8_t reg, uint8_t data) {
    return VL53L0X_WriteMulti(dev, reg, &data, 1);
}

VL53L0X_Error VL53L0X_WriteWord(VL53L0X_DEV dev, uint8_t reg, uint16_t data) {
    uint8_t buffer[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53L0X_WriteMulti(dev, reg, buffer, 2);
}

VL53L0X_Error VL53L0X_WriteDWord(VL53L0X_DEV dev, uint8_t reg, uint32_t data) {
    uint8_t buffer[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data)
    };
    return VL53L0X_WriteMulti(dev, reg, buffer, 4);
}

VL53L0X_Error VL53L0X_ReadByte(VL53L0X_DEV dev, uint8_t reg, uint8_t *data) {
    return VL53L0X_ReadMulti(dev, reg, data, 1);
}

VL53L0X_Error VL53L0X_ReadWord(VL53L0X_DEV dev, uint8_t reg, uint16_t *data) {
    uint8_t buffer[2];
    VL53L0X_Error status = VL53L0X_ReadMulti(dev, reg, buffer, 2);
    *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    return status;
}

VL53L0X_Error VL53L0X_ReadDWord(VL53L0X_DEV dev, uint8_t reg, uint32_t *data) {
    uint8_t buffer[4];
    VL53L0X_Error status = VL53L0X_ReadMulti(dev, reg, buffer, 4);
    *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3];
    return status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev) {
    vTaskDelay(pdMS_TO_TICKS(1));
    return VL53L0X_ERROR_NONE;
}
