#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

#include "logger.hpp"
#include "rtsp_server.hpp"
#include "jpeg_frame.hpp"

static const char *TAG = "RTSP_SERVER";

static std::unique_ptr<espp::RtspServer> rtsp_server;
static bool server_running = false;

//---------------------------------------------------------------------
// Helpers
//---------------------------------------------------------------------
static std::string get_current_ip() {
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!netif) return "0.0.0.0";
  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) return "0.0.0.0";
  char buf[16];
  snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip_info.ip));
  return std::string(buf);
}

//---------------------------------------------------------------------
// C API
//---------------------------------------------------------------------
extern "C" {

// Initialize the RTSP server instance
esp_err_t rtsp_server_init() {
  if (rtsp_server) {
    ESP_LOGW(TAG, "RTSP server already initialized");
    return ESP_OK;
  }

  std::string ip = get_current_ip();
  ESP_LOGI(TAG, "Initializing RTSP server on IP %s", ip.c_str());

  espp::RtspServer::Config cfg;
  cfg.server_address = "0.0.0.0";        // bind all interfaces
  cfg.port           = 8544;
  cfg.path           = "/mjpeg/1";
  cfg.max_data_size  = 1200;             // break frames <1 KB per RTP packet
  cfg.log_level      = espp::Logger::Verbosity::INFO;

  rtsp_server = std::make_unique<espp::RtspServer>(cfg);
  ESP_LOGI(TAG, "✅ RTSP server object created for rtsp://%s:%d%s",
           ip.c_str(), cfg.port, cfg.path.c_str());

  return ESP_OK;
}

// Start listening for clients (with 5 s accept timeout)
esp_err_t rtsp_server_start() {
  if (!rtsp_server) {
    ESP_LOGE(TAG, "Server not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  if (!rtsp_server->start(std::chrono::seconds(5))) {
    ESP_LOGE(TAG, "❌ Failed to start RTSP server");
    return ESP_FAIL;
  }
  server_running = true;
  ESP_LOGI(TAG, "🚀 RTSP server listening on rtsp://%s:%d%s",
           get_current_ip().c_str(),
           8544,
           "/mjpeg/1");
  return ESP_OK;
}

// Stop & destroy the server
esp_err_t rtsp_server_stop() {
  if (rtsp_server) {
    server_running = false;
    rtsp_server->stop();
    rtsp_server.reset();
    ESP_LOGI(TAG, "🛑 RTSP server stopped");
  }
  return ESP_OK;
}

//---------------------------------------------------------------------
// Camera streaming task
//---------------------------------------------------------------------
void rtsp_camera_stream_task(void *param) {
  ESP_LOGI(TAG, "🎥 RTSP camera stream task started");
  uint32_t frame_count = 0, dropped = 0;

  while (true) {
    if (!server_running || !rtsp_server) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    uint32_t t0 = esp_timer_get_time();  
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW(TAG, "Camera capture failed");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    frame_count++;
    size_t  size   = fb->len;
    uint32_t width  = fb->width;
    uint32_t height = fb->height;

    // Build and buffer the frame
    espp::JpegFrame frame(reinterpret_cast<const char*>(fb->buf), size);

    // Return buffer immediately
    esp_camera_fb_return(fb);

    // Send (overwrites any unsent frame); drop on error
    try {
      rtsp_server->send_frame(frame);
    } catch (const std::exception &e) {
      dropped++;
      ESP_LOGW(TAG, "Frame #%lu dropped: %s",
               (unsigned long)frame_count, e.what());
    }
    uint32_t t1 = esp_timer_get_time();  
    ESP_LOGI(TAG, "Frame total time: %u ms", (unsigned int)((t1 - t0) / 1000));
    // Stats every 50 frames
    if ((frame_count % 50) == 0) {
      ESP_LOGI(TAG,
        "📊 #%lu: %lux%lu, %zu bytes, dropped %lu",
        (unsigned long)frame_count,
        (unsigned long)width,
        (unsigned long)height,
        size,
        (unsigned long)dropped
      );
    }

    // Heap check every 100 frames
    if ((frame_count % 100) == 0) {
      ESP_LOGI(TAG,
        "Memory: Heap=%u, IRAM=%u, PSRAM=%u",
        (unsigned)esp_get_free_heap_size(),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM)
      );
    }

    // Yield so RTSP accept/session tasks can run
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

}  // extern "C"

