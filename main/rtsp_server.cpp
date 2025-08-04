#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Add these includes for IP handling
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

// Full espp RTSP includes
#include "logger.hpp"
#include "rtsp_server.hpp"
#include "jpeg_frame.hpp"

#include "include/rtsp_wrapper.h"

static const char *TAG = "RTSP_SERVER";

// Global espp RTSP server instance
static std::unique_ptr<espp::RtspServer> rtsp_server = nullptr;
static std::string server_ip;
static bool server_running = false;

extern "C" {

// Function to get current IP address
std::string get_current_ip(void) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
            return std::string(ip_str);
        }
    }
    return "0.0.0.0";
}

esp_err_t rtsp_server_init(void) {
    if (rtsp_server != nullptr) {
        ESP_LOGW(TAG, "RTSP server already initialized");
        return ESP_OK;
    }

    try {
        // Always bind to 0.0.0.0 so lwIP can handle SYN queue on all ifaces
        ESP_LOGI(TAG, "Initializing RTSP server on all interfaces");
        espp::RtspServer::Config cfg;
        cfg.server_address = "0.0.0.0";
        cfg.port           = 8554;               // default RTSP port
        cfg.path           = "/mjpeg/1";         //  MJPEG path
        cfg.max_data_size  = 1200;               // cap each UDP packet < MTU
        cfg.log_level      = espp::Logger::Verbosity::INFO;
        rtsp_server = std::make_unique<espp::RtspServer>(cfg);

        ESP_LOGI(TAG, "✅ RTSP server initialized successfully");
            return ESP_OK;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "❌ Failed to initialize RTSP server: %s", e.what());
        return ESP_FAIL;
    }
}

esp_err_t rtsp_server_start(void) {
    if (rtsp_server == nullptr) {
        ESP_LOGE(TAG, "RTSP server not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    try {
        if (rtsp_server->start()) {
            server_running = true;
            ESP_LOGI(TAG, "🚀 RTSP server started on rtsp://<your-ip>:8554/mjpeg/1");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "❌ Failed to start RTSP server");
            return ESP_FAIL;
        }
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "❌ Failed to start RTSP server: %s", e.what());
        return ESP_FAIL;
    }
}

esp_err_t rtsp_server_stop(void) {
    if (rtsp_server) {
        server_running = false;
        rtsp_server->stop();
        rtsp_server.reset();
        ESP_LOGI(TAG, "🛑 RTSP server stopped");
    }
    return ESP_OK;
}

void rtsp_camera_stream_task(void *param) {
    ESP_LOGI(TAG, "🎥 RTSP camera streaming task started");
    uint32_t frame_count = 0;
    while (1) {
        if (server_running && rtsp_server) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                frame_count++;
                espp::JpegFrame jpeg_frame(
                    reinterpret_cast<const char*>(fb->buf),
                    fb->len
                );
                rtsp_server->send_frame(jpeg_frame);
                if ((frame_count % 50) == 0) {
                    ESP_LOGI(TAG, "📊 Frame #%lu: %ux%u, %zu bytes",
                        (unsigned long)frame_count, fb->width, fb->height, fb->len);
                }
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(50));
            } else {
                ESP_LOGW(TAG, "Camera capture failed");
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if ((frame_count % 100) == 0) {
        ESP_LOGI(TAG, "Heap free: %u, IRAM: %u, PSRAM: %u",
            (unsigned int)esp_get_free_heap_size(),
            (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
    }
}

}  // extern "C"
