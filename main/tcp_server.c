#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "string.h"
#include "tcp_server.h"
#include <errno.h>

#define PORT 3333
static const char *TAG = "TCP_SERVER";

int global_sock = -1;

void tcp_server_task(void *pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    // Create listening socket
    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "❌ Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Bind socket
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "❌ Socket bind failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // Listen
    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "❌ Socket listen failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "📡 TCP server listening on port %d", PORT);

    while (1) {
        struct sockaddr_in6 source_addr;
        socklen_t addr_len = sizeof(source_addr);

        ESP_LOGI(TAG, " Waiting for a client to connect...");

        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, " Accept failed: errno %d", errno);
            continue;
        }

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, " Client connected from %s", addr_str);

        // Set the global socket
        global_sock = sock;

        // Wait for client disconnection
        while (1) {
            int ret = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, MSG_PEEK);
            if (ret == 0) {
                ESP_LOGW(TAG, " Client disconnected");
                break;
            } else if (ret < 0) {
                ESP_LOGE(TAG, " recv() error: errno %d", errno);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(500));  // poll interval
        }

        close(sock);
        global_sock = -1;
        ESP_LOGI(TAG, " Connection closed. Waiting for next client...");
    }

    // Cleanup (won’t be reached)
    close(listen_sock);
    vTaskDelete(NULL);
}
