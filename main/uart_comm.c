#include "./include/uart_comm.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define UART_PORT UART_NUM_1
#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define UART_BAUD_RATE 115200

static const char *TAG = "UART_COMM";

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART initialized on TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);
}

void uart_send_line(const char *data) {
    uart_write_bytes(UART_PORT, data, strlen(data));
    uart_write_bytes(UART_PORT, "\n", 1); // Optional newline
}

int uart_read_line(char *buf, int max_len) {
    int len = uart_read_bytes(UART_PORT, (uint8_t *)buf, max_len, pdMS_TO_TICKS(100));
    if (len > 0) {
        buf[len] = '\0'; // Null-terminate
    }
    return len;
}
