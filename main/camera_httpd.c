#include "esp_camera.h"
#include "esp_log.h"
#include "sdkconfig.h" // Ensure logging configuration is included
#include "esp_heap_caps.h" // Include for MALLOC_CAP_INTERNAL
#include "esp_http_server.h"
#include "./include/camera_httpd.h"
#include "./include/camera_config.h"
#include "esp_camera.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif
static const char *TAG = "CAM_HTTPD";

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    static const char* stream_content_type = "multipart/x-mixed-replace;boundary=frame";
    static const char* stream_boundary = "\r\n--frame\r\n";
    static const char* stream_part = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
    char part_buf[64];

    res = httpd_resp_set_type(req, stream_content_type);

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        res = httpd_resp_send_chunk(req, stream_boundary, strlen(stream_boundary));
        if (res != ESP_OK) break;

        int header_len = snprintf(part_buf, sizeof(part_buf), stream_part, fb->len);
        res = httpd_resp_send_chunk(req, part_buf, header_len);
        if (res != ESP_OK) break;

        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);

        if (res != ESP_OK) break;
        vTaskDelay(33 / portTICK_PERIOD_MS); // ~30 FPS
    }

    return res;
}

httpd_handle_t camera_httpd = NULL;

httpd_handle_t start_camera_httpd(void) {

    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;

    httpd_uri_t uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = stream_handler
    };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &uri);
        return camera_httpd;
    }
    return NULL;
}

