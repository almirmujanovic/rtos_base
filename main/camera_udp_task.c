#include "./include/camera_udp_task.h"
#include "./include/camera_config.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>  // for close()
#include "esp_timer.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

#define TAG "CAMERA_UDP_TASK" 

void camera_udp_task(void *pvParameters) {
    ESP_LOGI(TAG, "UDP camera task running on core %d", xPortGetCoreID());

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5000); // or any port you choose
    dest_addr.sin_addr.s_addr = inet_addr("192.168.137.112"); // Laptop IP

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("UDP", "Socket creation failed");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "UDP socket created, sending to %s:%d", inet_ntoa(dest_addr.sin_addr), ntohs(dest_addr.sin_port));


    while (1) {
        camera_fb_t *fb = NULL;
        uint64_t start = esp_timer_get_time(); // in ms
        fb = esp_camera_fb_get();
        uint64_t end = esp_timer_get_time();
        ESP_LOGI(TAG, "Frame capture time: %lld ms", (end - start) / 1000);

        if (!fb) {
            ESP_LOGW(TAG, "esp_camera_fb_get() returned NULL!");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "Captured frame: %d bytes", fb->len);

        // Send in chunks (if needed)
        const size_t max_packet = 1400;
        size_t offset = 0;
        int chunk_idx=0;

        while (offset < fb->len) {
            size_t chunk_size = (fb->len - offset > max_packet) ? max_packet : (fb->len - offset);
            int sent = sendto(sock, fb->buf + offset, chunk_size, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (sent < 0) {
                ESP_LOGE(TAG, "sendto failed: errno %d (%s), chunk %d, offset %d/%d", errno, strerror(errno), chunk_idx, offset, fb->len);
            } else {
                ESP_LOGI(TAG, "Sent UDP chunk: %d bytes (chunk %d, offset %d/%d)", sent, chunk_idx, offset, fb->len);
            }
            offset += chunk_size;
            chunk_idx++;
            vTaskDelay(20);
        }
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        
    }
}

void udp_test_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5000);
    dest_addr.sin_addr.s_addr = inet_addr("192.168.137.112"); // your laptop IP

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("UDP", "Socket creation failed");
        vTaskDelete(NULL);
    }
    ESP_LOGI("UDP_TEST", "UDP socket created, sending to %s:%d", inet_ntoa(dest_addr.sin_addr), ntohs(dest_addr.sin_port));

    while (1) {
        char msg[] = "ESP32 UDP test";
        int sent = sendto(sock, msg, strlen(msg), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            ESP_LOGE("UDP_TEST", "sendto failed: errno %d (%s)", errno, strerror(errno));
        } else {
            ESP_LOGI("UDP_TEST", "Sent UDP test packet: %d bytes", sent);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}