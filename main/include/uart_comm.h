#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t uart_line_queue;

void uart_init(void);
void uart_send_line(const char *data);
int uart_read_line(char *buf, int max_len);
