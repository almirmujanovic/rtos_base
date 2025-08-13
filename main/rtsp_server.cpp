#include "esp_log.h"
#include "esp_camera.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <memory>
#include <cmath>

#include "logger.hpp"
#include "rtsp_server.hpp"
#include "jpeg_frame.hpp"

static const char* TAG = "RTSP_SERVER";

// ---------- tunables ----------
static constexpr size_t RTP_PAYLOAD = 1350;          // ~1400B fits 1500 MTU w/ headers
static constexpr size_t MAX_FRAME_BYTES = 60*1024;   // hard size cap (VGA safe)
static constexpr size_t MAX_PKTS_PER_FRAME = 24;     // drop frames that would exceed this
static constexpr int    WARMUP_GOOD_FRAMES = 3;      // require N good frames before sending

// pacing bounds (ms)
static constexpr int PERIOD_MS_BASE = 50;            // ~20 fps target
static constexpr int PERIOD_MS_MIN  = 40;            // up to 25 fps if link is great
static constexpr int PERIOD_MS_MAX  = 120;           // down to ~8 fps if congested

// OV2640 quality (lower number = better quality = larger JPEG)
static constexpr int Q_LOWER_LIMIT = 24;             // best we'll allow automatically
static constexpr int Q_UPPER_LIMIT = 34;             // smallest we'll shrink to automatically

// adaptation window
static constexpr int   ADAPT_WINDOW_FRAMES = 80;     // ~4s @ 20 fps
static constexpr float DROP_HIGH = 0.06f;            // >6% gateway drops => back off
static constexpr float DROP_LOW  = 0.01f;            // <1% drops => ease up a little

// congestion detection via send time (us)
static constexpr int64_t SEND_CONGEST_US = 25000;    // if a single send takes >25 ms, NIC is busy
static constexpr int     CONGEST_HITS_FOR_BACKOFF = 2;

// -------------------------------

static std::unique_ptr<espp::RtspServer> rtsp_server;
static std::atomic<bool> server_running{false};

static std::string get_current_ip() {
  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!netif) return "0.0.0.0";
  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) return "0.0.0.0";
  char buf[16];
  snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip_info.ip));
  return std::string(buf);
}

static int get_rssi() {
  wifi_ap_record_t ap{};
  if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
  return 0;
}

extern "C" {

esp_err_t rtsp_server_init() {
  if (rtsp_server) return ESP_OK;
  std::string ip = get_current_ip();
  if (ip == "0.0.0.0") {
    ESP_LOGW(TAG, "No IP yet; skip RTSP init");
    return ESP_FAIL;
  }

  espp::RtspServer::Config cfg{};
  cfg.server_address = ip;
  cfg.port           = 8544;
  cfg.path           = "mjpeg/1";             // no leading slash
  cfg.max_data_size  = RTP_PAYLOAD;
  cfg.log_level      = espp::Logger::Verbosity::INFO;

  try {
    rtsp_server = std::make_unique<espp::RtspServer>(cfg);
    rtsp_server->set_session_log_level(espp::Logger::Verbosity::INFO);
    ESP_LOGI(TAG, "RTSP at rtsp://%s:%d/%s (payload=%u)",
             ip.c_str(), cfg.port, cfg.path.c_str(), (unsigned)cfg.max_data_size);
    return ESP_OK;
  } catch (const std::exception& e) {
    ESP_LOGE(TAG, "Failed to create RTSP server: %s", e.what());
    rtsp_server.reset();
    return ESP_FAIL;
  }
}

esp_err_t rtsp_server_start() {
  if (!rtsp_server) return ESP_FAIL;
  try {
    rtsp_server->start();
    server_running.store(true, std::memory_order_release);
    ESP_LOGI(TAG, "RTSP server started (UDP)");
    return ESP_OK;
  } catch (const std::exception& e) {
    ESP_LOGE(TAG, "Start failed: %s", e.what());
    server_running.store(false, std::memory_order_release);
    return ESP_FAIL;
  }
}

esp_err_t rtsp_server_stop() {
  if (!rtsp_server) return ESP_OK;
  try {
    server_running.store(false, std::memory_order_release);
    rtsp_server->stop();
    rtsp_server.reset();
    ESP_LOGI(TAG, "RTSP server stopped");
    return ESP_OK;
  } catch (const std::exception& e) {
    ESP_LOGE(TAG, "Stop failed: %s", e.what());
    return ESP_FAIL;
  }
}

void rtsp_camera_stream_task(void*) {
  ESP_LOGI(TAG, "Stream task started");

  TickType_t last_wake = xTaskGetTickCount();
  int period_ms = PERIOD_MS_BASE;

  int warmup_needed = WARMUP_GOOD_FRAMES;
  size_t ema_size = 0;                  // EMA of JPEG size
  uint32_t sent = 0, dropped = 0;
  uint32_t win_frames = 0, win_dropped = 0;

  int consec_congest = 0;               // consecutive “send took too long” events

  for (;;) {
    if (!server_running.load(std::memory_order_acquire) || !rtsp_server) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
      continue;
    }

    const size_t n = fb->len;
    const uint8_t* b = fb->buf;

    // JPEG sanity and burst gate
    const bool soi = (n >= 2 && b[0] == 0xFF && b[1] == 0xD8);
    const bool eoi = (n >= 2 && b[n-2] == 0xFF && b[n-1] == 0xD9);
    const size_t pkts = (n + RTP_PAYLOAD - 1) / RTP_PAYLOAD;

    bool ok = soi && eoi && (n <= MAX_FRAME_BYTES) && (pkts <= MAX_PKTS_PER_FRAME);

    // warmup: drop first few good frames to avoid early garbage
    if (ok && warmup_needed > 0) { warmup_needed--; ok = false; }

    if (ok) {
      const int64_t t0 = esp_timer_get_time();
      espp::JpegFrame frame(reinterpret_cast<const char*>(b), n);
      rtsp_server->send_frame(frame);  // internal logs will show ENOMEM if it happens
      const int64_t dt = esp_timer_get_time() - t0;

      sent++;
      ema_size = (ema_size == 0) ? n : (size_t)(0.9f * ema_size + 0.1f * n);

      // detect congestion via long send time (NIC/lwIP backpressure)
      if (dt > SEND_CONGEST_US) {
        if (++consec_congest >= CONGEST_HITS_FOR_BACKOFF) {
          // small immediate backoff to let TX drain
          period_ms = std::min(period_ms + 10, PERIOD_MS_MAX);
          sensor_t* s = esp_camera_sensor_get();
          if (s && s->status.quality < Q_UPPER_LIMIT) s->set_quality(s, s->status.quality + 1);
          consec_congest = 0;
        }
      } else {
        consec_congest = 0;
      }
    } else {
      dropped++;
      win_dropped++;
    }

    esp_camera_fb_return(fb);
    win_frames++;

    // windowed gentle adaptation (every ~4s)
    if (win_frames >= ADAPT_WINDOW_FRAMES) {
      const float drop = (float)win_dropped / (float)win_frames;

      // pace first (primary control)
      if (drop > DROP_HIGH) {
        period_ms = std::min(PERIOD_MS_MAX, period_ms + 10);
      } else if (drop < DROP_LOW) {
        period_ms = std::max(PERIOD_MS_MIN, period_ms - 5);
      }

      // tiny quality nudges
      sensor_t* s = esp_camera_sensor_get();
      if (s) {
        int q = s->status.quality;
        if (drop > DROP_HIGH || ema_size > 40*1024) {
          if (q < Q_UPPER_LIMIT) s->set_quality(s, q + 1);   // smaller JPEG
        } else if (drop < DROP_LOW && ema_size < 26*1024) {
          if (q > Q_LOWER_LIMIT) s->set_quality(s, q - 1);   // better quality
        }
      }

      // diag
      multi_heap_info_t hi{};
      heap_caps_get_info(&hi, MALLOC_CAP_8BIT);
      ESP_LOGI(TAG, "diag: RSSI=%d dBm, heap=%u, psram=%u, fps~%d, drop=%.1f%%, ema=~%u, period=%dms",
               get_rssi(),
               (unsigned)hi.total_free_bytes,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               (int)std::round(1000.0f / period_ms),
               drop*100.f,
               (unsigned)ema_size,
               period_ms);

      win_frames = 0;
      win_dropped = 0;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
  }
}

} // extern "C"
