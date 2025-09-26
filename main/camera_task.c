#include "./include/camera_task.h"
#include "./include/camera_config.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>  

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

#define TAG "CAMERA_TASK" 
extern int camera_sock;

// Sends all data over socket
static bool send_all(int sock, const void *data, size_t len) {
    size_t total_sent = 0;
    const uint8_t *ptr = (const uint8_t *)data;

    while (total_sent < len) {
        int sent = send(sock, ptr + total_sent, len - total_sent, 0);
        if (sent < 0) {
            ESP_LOGE(TAG, "send() failed: %s", strerror(errno));
            return false;
        }
        total_sent += sent;
    }
    return true;
}

void camera_task(void *param) {
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        vTaskDelete(NULL);
    }

    while (1) {
        if (camera_sock < 0) {
            ESP_LOGW(TAG, "No active client socket");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        struct timeval timeout = { .tv_sec = 0, .tv_usec = 50000 }; // 50ms timeout
        setsockopt(camera_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint32_t len = fb->len;

        if (!send_all(camera_sock, &len, sizeof(len))) {
            ESP_LOGE(TAG, "Failed to send frame length");
            close(camera_sock);
            camera_sock = -1;
            esp_camera_fb_return(fb);
            continue;
        }

        if (!send_all(camera_sock, fb->buf, fb->len)) {
            ESP_LOGE(TAG, "Failed to send frame data");
            close(camera_sock);
            camera_sock = -1;
        } else {
            ESP_LOGI(TAG, "Frame sent: %u bytes", (unsigned int)len);
        }

        esp_camera_fb_return(fb);
        //vTaskDelay(pdMS_TO_TICKS(10));  
    }
}