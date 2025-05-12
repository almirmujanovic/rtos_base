#pragma once
#include "esp_http_server.h"

void start_websocket_server(httpd_handle_t server);
void websocket_send_to_pc(const char *message);
