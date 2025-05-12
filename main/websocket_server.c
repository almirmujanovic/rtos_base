#include "esp_http_server.h"
#include "esp_log.h"
#include "string.h"
#include "./include/websocket_server.h"
#include "uart_comm.h"


static const char *TAG = "WS_SERVER";
static httpd_handle_t ws_server = NULL;

static int client_fd = -1;
static httpd_handle_t client_handle = NULL;

static esp_err_t websocket_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        client_fd = httpd_req_to_sockfd(req);
        client_handle = req->handle;
        ESP_LOGI(TAG, "WebSocket client connected, fd=%d", client_fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(httpd_ws_frame_t));
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.len = req->content_len;

    uint8_t *buf = NULL;
    if (frame.len > 0) {
        buf = malloc(frame.len + 1);
        frame.payload = buf;

        esp_err_t ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WebSocket receive error, assuming disconnection. Resetting.");
            client_fd = -1;
            client_handle = NULL;
            free(buf);
            return ret;
        }

        buf[frame.len] = '\0';
        ESP_LOGI(TAG, "Received from PC via WS: %s", (char*)buf);
        uart_send_line((char*)buf);
        free(buf);
    }

    return ESP_OK;
}

void websocket_send_to_pc(const char *message) {
    if (!client_handle || client_fd < 0) {
        ESP_LOGW("WS_SERVER", "⚠️ No WebSocket client connected. Skipping send.");
        return;
    }

    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)message,
        .len = strlen(message)
    };

    esp_err_t err = httpd_ws_send_frame_async(client_handle, client_fd, &frame);
    if (err != ESP_OK) {
        ESP_LOGE("WS_SERVER", "❌ WebSocket send failed: %s", esp_err_to_name(err));
        client_fd = -1;
        client_handle = NULL;  // 🧹 Clean up
    }
}



extern httpd_handle_t camera_httpd;  // Add this at the top

void start_websocket_server(httpd_handle_t server){
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

