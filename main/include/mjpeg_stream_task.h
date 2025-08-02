#pragma once
#include "esp_http_server.h"
static esp_err_t mjpeg_stream_handler(httpd_req_t *req);
static void start_mjpeg_http_server(void);
void mjpeg_stream_task(void *pvParameter);