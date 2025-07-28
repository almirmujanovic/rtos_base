#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "./include/uart_comm.h"
#include "./include/uart_task.h"
#include "esp_log.h"
#include <string.h>
#include "./include/websocket_server.h"
#include "mqtt_uart_bridge.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static const char *TAG = "UART_TASK";

extern QueueHandle_t uart_line_queue;

void uart_task(void *pvParameters) {
    char rx_buffer[128] = {0};

    while (1) {
    int len = uart_read_line(rx_buffer, sizeof(rx_buffer) - 1);
    if (len > 0) {
        rx_buffer[len] = '\0';
        if (strlen(rx_buffer) > 1) {
            char *line = strtok(rx_buffer, "\r\n");
            while (line != NULL) {
                if (strlen(line) > 1) {
                    ESP_LOGD(TAG, "UART Received: %s", line);
                    mqtt_publish_uart_data(line);  
                }
                line = strtok(NULL, "\r\n");
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));  //  fast
}

}
