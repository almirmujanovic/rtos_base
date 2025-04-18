// uart_comm.h

#pragma once

#include <stdio.h>

void uart_init();
void uart_send_line(const char *data);
int uart_read_line(char *buf, int max_len);
