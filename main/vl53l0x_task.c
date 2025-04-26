
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_platform_log.h"
#include "driver/gpio.h"
#include "tcp_server.h"  // for `global_sock`
#include "lwip/sockets.h"
#include "i2c_init.h"  // for `init_i2c_master_vl53l0x()`

#define XSHUT_GPIO GPIO_NUM_3
#define TAG "VL53L0X_TASK"
#define VL53L0X_I2C_ADDR 0x29

VL53L0X_Dev_t sensor_dev;
VL53L0X_DEV dev = &sensor_dev;

void vl53l0x_task(void *pvParameters) {
    VL53L0X_RangingMeasurementData_t measurement;
    VL53L0X_Error status;

    // Init XSHUT pin
    gpio_reset_pin(XSHUT_GPIO);
    gpio_set_direction(XSHUT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(XSHUT_GPIO, 1);  // Power on

    vTaskDelay(pdMS_TO_TICKS(10));  // Stabilize

    dev->I2cDevAddr = 0x29;
    dev->comms_type = 1;
    dev->comms_speed_khz = 50;

    status = VL53L0X_WaitDeviceBooted(dev);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "Device boot wait failed: %d", status);
        vTaskDelete(NULL);
    }
    ESP_LOGI("VL53L0X_TASK", "Probing sensor at address 0x%02X...", VL53L0X_I2C_ADDR);

    status = VL53L0X_DataInit(dev);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "DataInit failed: %d", status);
        vTaskDelete(NULL);
    }

    status = VL53L0X_StaticInit(dev);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "StaticInit failed: %d", status);
        vTaskDelete(NULL);
    }

    status = VL53L0X_PerformRefCalibration(dev, NULL, NULL);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "Calibration failed: %d", status);
        vTaskDelete(NULL);
    }

    status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "SetMode failed: %d", status);
        vTaskDelete(NULL);
    }

    status = VL53L0X_StartMeasurement(dev);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "StartMeasurement failed: %d", status);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "VL53L0X running...");


    ESP_LOGI("VL53L0X_TASK", "Starting VL53L0X task");


    char packet[64];

    while (1) {
        status = VL53L0X_GetRangingMeasurementData(dev, &measurement);
        if (status == VL53L0X_ERROR_NONE) {
            uint16_t distance = measurement.RangeMilliMeter;
            ESP_LOGI(TAG, "Distance: %d mm", distance);

            if (global_sock != -1) {
                snprintf(packet, sizeof(packet), "[TOF] %d\n", distance);
                send(global_sock, packet, strlen(packet), 0);
            }

            VL53L0X_ClearInterruptMask(dev, 0);
        } else {
            ESP_LOGW(TAG, "Failed to read measurement: %d", status);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
