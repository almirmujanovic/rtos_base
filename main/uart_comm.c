#include "./include/uart_comm.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define UART_PORT UART_NUM_1
#define UART_TX_PIN 20
#define UART_RX_PIN 21
#define UART_BAUD_RATE 115200
#define BUF_SIZE 512
#define UART_QUEUE_SIZE 30

static const char *TAG = "UART_COMM";
QueueHandle_t uart_line_queue = NULL;

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install driver with larger event queue
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, UART_QUEUE_SIZE, &uart_line_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // SPEED OPTIMIZATIONS (these functions exist):
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT, 1));           // Minimal timeout for speed
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_PORT, 1));    // Trigger on single byte
    
    ESP_LOGI(TAG, "✅ UART initialized (SPEED OPTIMIZED) on TX=%d RX=%d at %d baud", 
             UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

void uart_send_line(const char *data) {
    if (!data) return;
    
    int len = strlen(data);
    uart_write_bytes(UART_PORT, data, len);
    uart_write_bytes(UART_PORT, "\n", 1);
    
    ESP_LOGD(TAG, "📤 Sent to Arduino: %s", data);
}

int uart_read_line(char *buf, int max_len) {
    if (!buf || max_len <= 1) return 0;
    
    memset(buf, 0, max_len);
    int len = uart_read_bytes(UART_PORT, (uint8_t*)buf, max_len - 1, pdMS_TO_TICKS(50));
    
    if (len > 0) {
        buf[len] = '\0';
        while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n')) {
            buf[--len] = '\0';
        }
        return len;
    }
    
    return 0;
}