#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

void tcp_camera_stream_task(void *pvParameters);