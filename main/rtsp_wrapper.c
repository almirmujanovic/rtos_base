#include "rtsp_wrapper.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RTSP_WRAPPER";

void rtsp_camera_stream_task(void *param) {
    ESP_LOGI(TAG, "RTSP camera streaming task started");
    
    while (1) {
        // RTSP server handles frame retrieval internally
        // Just keep the task alive
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}