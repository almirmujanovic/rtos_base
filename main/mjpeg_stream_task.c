#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "mjpeg_stream_task.h"
#include "esp_heap_caps.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

static const char *TAG = "MJPEG_TASK";

#define STREAM_BOUNDARY "123456789000000000000987654321"
#define STREAM_HEADER   "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"
#define STREAM_TYPE     "multipart/x-mixed-replace; boundary=" STREAM_BOUNDARY

static httpd_handle_t mjpeg_httpd = NULL;
static bool mjpeg_client_active = false;

esp_err_t mjpeg_stream_handler(httpd_req_t *req) {
    static const char *TAG = "MJPEG_HANDLER";
    char part_buf[128];
    esp_err_t res = ESP_OK;
        if (mjpeg_client_active) {
        httpd_resp_send_err(req, 503, "Stream in use");
        return ESP_FAIL;
    }

    mjpeg_client_active = true;

    ESP_LOGI(TAG, "Client connected for MJPEG stream");

    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=123456789000000000000987654321");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Prepare multipart header
        int len = snprintf(part_buf, sizeof(part_buf),
                        "\r\n--" STREAM_BOUNDARY "\r\n"
                        "Content-Type: image/jpeg\r\n"
                        "Content-Length: %u\r\n\r\n", fb->len);

        // First send the header
        res = httpd_resp_send_chunk(req, part_buf, len);
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Header send failed: %s", esp_err_to_name(res));
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(200));  // let socket drain
            continue;  // try again
        }

        // Then send the JPEG data
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "JPEG send failed: %s", esp_err_to_name(res));
            vTaskDelay(pdMS_TO_TICKS(200));  // try to recover
            continue;  // don’t break yet
        }

        ESP_LOGI(TAG, "Sent frame: %d bytes", fb->len);
        vTaskDelay(pdMS_TO_TICKS(100));  // ~10 FPS
    }


    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "MJPEG stream ended");
    mjpeg_client_active = false;
    return ESP_OK;

}


static void start_mjpeg_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;  // Use port 80 for browser/OpenCV compatibility
    config.stack_size = 8192;
    config.max_uri_handlers = 8;

    esp_err_t err = httpd_start(&mjpeg_httpd, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTPD start failed: %s", esp_err_to_name(err));
        return;
    }

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = mjpeg_stream_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(mjpeg_httpd, &stream_uri);

    ESP_LOGI(TAG, "MJPEG HTTP server started on port %d", config.server_port);
}

void mjpeg_stream_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting MJPEG stream task");

    start_mjpeg_http_server();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Task stays alive for now — can be expanded to handle server restart
    }
}
