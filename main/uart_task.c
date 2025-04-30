#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "./include/uart_comm.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include <string.h>
#include "esp_mac.h"
#include "freertos/FreeRTOSConfig.h" // Ensure configTICK_RATE_HZ is defined
#include "./include/uart_task.h"

static const char *TAG = "UART_TASK";
/*
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
}*/

void uart_task(void *pvParameters) {
    const char *test_msg = "Hello Arduino!\n";
    char rx_buf[128];

    while (1) {
        ESP_LOGI("UART_TEST", "Sending: %s", test_msg);
        uart_send_line(test_msg);

        // Čekaj odgovor
        int len = uart_read_line(rx_buf, sizeof(rx_buf) - 1);

        // Loguj sirovi rezultat
        ESP_LOGI("UART_TEST", "uart_read_line returned: %d", len);

        if (len > 0) {
            rx_buf[len] = '\0';  // Osiguraj null-terminaciju
            ESP_LOGI("UART_TEST", "Full string received: \"%s\"", rx_buf);
            for (int i = 0; i < len; i++) {
                char c = rx_buf[i];
                if (c >= 32 && c <= 126) {
                    // Printable character
                    ESP_LOGI("UART_TEST", "Byte[%d] = 0x%02X (%c)", i, c, c);
                } else {
                    // Non-printable (newline, carriage return, etc.)
                    ESP_LOGI("UART_TEST", "Byte[%d] = 0x%02X (non-printable)", i, c);
                }
            }
        } else {
            ESP_LOGW("UART_TEST", "No echo received");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Čekaj prije sljedeće poruke
    }
}


