#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "./include/uart_comm.h"
#include "./include/uart_task.h"
#include "esp_log.h"
#include <string.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static const char *TAG = "UART_TASK";

extern QueueHandle_t uart_line_queue;

void uart_task(void *pvParameters) {
    char rx_buffer[128];

    while (1) {
        int len = uart_read_line(rx_buffer, sizeof(rx_buffer) - 1);
        if (len > 0) {
            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "UART Received: %s", rx_buffer);

            if (uart_line_queue != NULL) {
                // Forward to TCP server
                if (xQueueSend(uart_line_queue, &rx_buffer, pdMS_TO_TICKS(50)) != pdTRUE) {
                    ESP_LOGW(TAG, "UART queue full, dropped line");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Slight delay to avoid tight loop
    }
}
