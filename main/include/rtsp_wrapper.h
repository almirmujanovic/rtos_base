#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

// C interface for RTSP server
esp_err_t rtsp_server_init(void);
esp_err_t rtsp_server_start(void);
esp_err_t rtsp_server_stop(void);
void rtsp_camera_stream_task(void *param);

#ifdef __cplusplus
}
#endif