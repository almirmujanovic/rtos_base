#include "./include/camera_task.h"
#include "./include/camera_config.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "./include/tcp_server.h"
#include <sys/socket.h>
#include <errno.h>

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

#define TAG "CAMERA_TASK" 

extern int global_sock;  // defined in tcp_server.c or exposed via header

void camera_task(void *param) {
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        vTaskDelete(NULL);
    }

    while (1) {
        if (global_sock < 0) {
            ESP_LOGW(TAG, "⚠️ No active client socket");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
    
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
    
        // Send frame size first
        uint32_t len = fb->len;
        int res = send(global_sock, &len, sizeof(len), 0);
        if (res <= 0) {
            ESP_LOGE(TAG, "Failed to send frame length (%u bytes). Error: %d", (unsigned int)len, errno);
            global_sock = -1;
            esp_camera_fb_return(fb);
            continue;
        }
    
        res = send(global_sock, fb->buf, fb->len, 0);
        ESP_LOGI(TAG, "📸 Captured frame (%u bytes)", fb->len);
        ESP_LOGI(TAG, "🔁 Sent frame length: %u (res=%d)", (unsigned int)len, res);
    
        if (res <= 0) {
            ESP_LOGE(TAG, "Failed to send frame buffer (%u bytes). Error: %d", (unsigned int)len, errno);
            global_sock = -1;
        }
    
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(100));  // ~10 FPS
    }
    
}

