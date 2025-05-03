#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "string.h"
#include "errno.h"
#include "./include/tcp_server_task.h"

#define CAMERA_PORT 8080
static const char *TAG = "TCP_CAMERA";

int camera_sock = -1;  // shared with camera_task

void tcp_camera_stream_task(void *pvParameters) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create camera socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_port = htons(CAMERA_PORT);

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Camera socket bind failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
    }

    listen(listen_sock, 1);
    ESP_LOGI(TAG, "📸 Camera TCP server listening on port %d", CAMERA_PORT);

    while (1) {
        ESP_LOGI(TAG, " Waiting for camera client...");

        int sock = accept(listen_sock, NULL, NULL);
        if (sock < 0) {
            ESP_LOGE(TAG, "Accept failed: errno %d", errno);
            continue;
        }

        ESP_LOGI(TAG, "Camera client connected");
        camera_sock = sock;

        // Block until connection breaks (camera_task uses it)
        while (camera_sock >= 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ESP_LOGI(TAG, "Camera socket closed");
    }

    close(listen_sock);
    vTaskDelete(NULL);
}
