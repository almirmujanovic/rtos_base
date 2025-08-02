#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_http_server.h"

httpd_handle_t start_camera_httpd(void);

#ifdef __cplusplus
}
#endif