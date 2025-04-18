#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_comm.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include <string.h>
#include "esp_mac.h"
#include "freertos/FreeRTOSConfig.h" // Ensure configTICK_RATE_HZ is defined
#include "uart_task.h"

static const char *TAG = "UART_TASK";

void uart_task(void *pvParameters) {
    char rx_buffer[128];

    while (1) {
        int len = uart_read_line(rx_buffer, sizeof(rx_buffer) - 1);
        if (len > 0) {
            ESP_LOGI(TAG, "Received from Arduino: %s", rx_buffer);

            // Optional: parse or forward to PC
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
