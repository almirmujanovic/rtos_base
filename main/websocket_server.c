#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_camera.h"

static const char *TAG = "WS_SERVER";

static int client_fd = -1;
static httpd_handle_t client_handle = NULL;

// WebSocket handler: only manages connection/disconnection
static esp_err_t websocket_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        client_fd = httpd_req_to_sockfd(req);
        client_handle = req->handle;
        ESP_LOGI(TAG, "WebSocket client connected, fd=%d", client_fd);
        return ESP_OK;
    }
    // Ignore all non-GET (data) frames
    return ESP_OK;
}

// Register the /ws endpoint on the HTTP server
void start_websocket_server(httpd_handle_t server) {
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .is_websocket = true
    };

    esp_err_t ret = httpd_register_uri_handler(server, &ws_uri);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WebSocket server registered on port 81");
    } else {
        ESP_LOGE(TAG, "Failed to register WebSocket: %d", ret);
    }
}

// Streaming task: sends JPEG frames as binary WebSocket messages
void websocket_camera_stream_task(void *param) {
    while (1) {
        if (client_handle && client_fd >= 0) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                httpd_ws_frame_t frame = {
                    .type = HTTPD_WS_TYPE_BINARY,
                    .payload = fb->buf,
                    .len = fb->len
                };
                esp_err_t err = httpd_ws_send_frame_async(client_handle, client_fd, &frame);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "WebSocket send failed: %s", esp_err_to_name(err));
                    client_fd = -1;
                    client_handle = NULL;
                }
                esp_camera_fb_return(fb);
            }
            // Tune this delay for your bandwidth and reliability needs
            vTaskDelay(pdMS_TO_TICKS(66)); // ~15 FPS
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}