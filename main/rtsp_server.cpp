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

extern "C" void udp_get_send_counters(uint32_t*, uint32_t*, uint32_t*, uint32_t*, bool reset);

static const char* TAG = "RTSP_SERVER";

// ---------- podesavanja ----------
// ---------- VGA tuned (larger RTP payload, smaller frames) ----------

// veći paket ali ispod MTU (~1500) s IP/UDP/RTP overheadom
static constexpr size_t RTP_PAYLOAD        = 1300;        // 1200–1350 je safe

// agresivnije granice veličine/bursta za VGA (cilj ~36 KB/frame)
static constexpr size_t MAX_FRAME_BYTES    = 48 * 1024;   // hard cap (drop iznad ovoga)
static constexpr size_t MAX_PKTS_PER_FRAME = 28;          // 28 * 1300 ≈ 36.4 KB burst cap

// kratko "zagrijavanje"
static constexpr int    WARMUP_GOOD_FRAMES = 3;

// pacing (FPS) – ~12.5 fps baseline
static constexpr int PERIOD_MS_BASE = 80;                 // ~12.5 fps
static constexpr int PERIOD_MS_MIN  = 80;                 // ne forsirati >12.5 fps na VGA
static constexpr int PERIOD_MS_MAX  = 220;                // dovoljno prostora za backoff

// OV2640 kvaliteta (niži broj = bolja kvaliteta = veći fajl)
static constexpr int Q_LOWER_LIMIT = 32;                  // najbolja dozvoljena 
static constexpr int Q_UPPER_LIMIT = 44;                  // najlošija dozvoljena

// spora adaptacija (prozor ~3s @ ~12.5 fps)
static constexpr int   ADAPT_WINDOW_FRAMES = 60;
static constexpr float DROP_HIGH = 0.05f;                 // >5% pre-send drop -> uspori
static constexpr float DROP_LOW  = 0.015f;                // <1.5% -> oprezno poboljšaj

// brzi indikator zagušenja: dugo slanje jednog frame paketa
static constexpr int64_t SEND_CONGEST_US = 25000;         // >25 ms => zagušenje
static constexpr int     CONGEST_HITS_FOR_BACKOFF = 2;

// hladni period nakon tvrdog faila
static constexpr int COOLDOWN_MS_ON_FAIL = 60;
static constexpr int SKIP_FRAMES_ON_FAIL = 1;
       // drop next N frames immediately
// --------------------------------

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
  cfg.path           = "mjpeg/1";             
  cfg.max_data_size  = RTP_PAYLOAD;
  cfg.log_level      = espp::Logger::Verbosity::INFO;

  try {
    rtsp_server = std::make_unique<espp::RtspServer>(cfg);
    rtsp_server->set_session_log_level(espp::Logger::Verbosity::WARN);
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
  size_t ema_size = 0;                      // velicina JPEG
  uint32_t sent = 0, dropped = 0;
  uint32_t win_frames = 0, win_dropped = 0;

  // brz feedback
  int consec_congest = 0;                   // consecutive long send() events
  int skip_frames = 0;                      // immediate drop budget after hard fails

  uint32_t prev_ok=0, prev_nomem=0, prev_nobufs=0, prev_eagain=0;
  udp_get_send_counters(&prev_ok, &prev_nomem, &prev_nobufs, &prev_eagain, /*reset=*/false);

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

    // provjere jpeg parametara
    // 
    const bool soi = (n >= 2 && b[0] == 0xFF && b[1] == 0xD8);
    const bool eoi = (n >= 2 && b[n-2] == 0xFF && b[n-1] == 0xD9);
    const size_t pkts = (n + RTP_PAYLOAD - 1) / RTP_PAYLOAD;

    bool ok = soi && eoi && (n <= MAX_FRAME_BYTES) && (pkts <= MAX_PKTS_PER_FRAME);

    // prvih par frameova se baca
    if (ok && warmup_needed > 0) { warmup_needed--; ok = false; }

    // immediate drop mode after hard fail last frame
    if (ok && skip_frames > 0) {
      ok = false;
      skip_frames--;
    }

    if (ok) {
      // snapshot UDP counters BEFORE send
      uint32_t b_ok=0, b_nomem=0, b_nobufs=0, b_eagain=0;
      udp_get_send_counters(&b_ok, &b_nomem, &b_nobufs, &b_eagain, /*reset=*/false);

      const int64_t t0 = esp_timer_get_time();
      espp::JpegFrame frame(reinterpret_cast<const char*>(b), n);
      rtsp_server->send_frame(frame);  // UDP path logs on failure
      const int64_t dt = esp_timer_get_time() - t0;

      // snapshot AFTER send and compute deltas for this frame
      uint32_t a_ok=0, a_nomem=0, a_nobufs=0, a_eagain=0;
      udp_get_send_counters(&a_ok, &a_nomem, &a_nobufs, &a_eagain, /*reset=*/false);

      const uint32_t df_nomem  = (a_nomem  - b_nomem);
      const uint32_t df_nobufs = (a_nobufs - b_nobufs);
      const uint32_t df_hard   = df_nomem + df_nobufs;

      sent++;
      ema_size = (ema_size == 0) ? n : (size_t)(0.9f * ema_size + 0.1f * n);

      // reakcija na failed send - pauza i bacanje sljedeceg framea
      if (df_hard > 0) {
        vTaskDelay(pdMS_TO_TICKS(COOLDOWN_MS_ON_FAIL));
        skip_frames = SKIP_FRAMES_ON_FAIL;

        // smanji brzinu  i kvalitet
        period_ms = std::min(period_ms + 10, PERIOD_MS_MAX);
        if (sensor_t* s = esp_camera_sensor_get()) {
          if (s->status.quality < Q_UPPER_LIMIT) s->set_quality(s, s->status.quality + 1);
        }
      }

      // fast heuristic: long send time indicates congestion
      if (dt > SEND_CONGEST_US) {
        if (++consec_congest >= CONGEST_HITS_FOR_BACKOFF) {
          period_ms = std::min(period_ms + 10, PERIOD_MS_MAX);
          if (sensor_t* s = esp_camera_sensor_get()) {
            if (s->status.quality < Q_UPPER_LIMIT) s->set_quality(s, s->status.quality + 1);
          }
          consec_congest = 0;
        }
      } else {
        consec_congest = 0;
      }

      // keep previous snapshot for optional diagnostics
      prev_ok = a_ok; prev_nomem = a_nomem; prev_nobufs = a_nobufs; prev_eagain = a_eagain;

    } else {
      dropped++;
      win_dropped++;
    }

    esp_camera_fb_return(fb);
    win_frames++;

    // prilagodjavanje svake 3s
    if (win_frames >= ADAPT_WINDOW_FRAMES) {
      const float drop = (float)win_dropped / (float)win_frames;

      // window-level UDP fail rate
      uint32_t ok=0, enomem=0, enobufs=0, eagain=0;
      udp_get_send_counters(&ok, &enomem, &enobufs, &eagain, /*reset=*/true);
      const uint32_t hard_fail = enomem + enobufs;
      const float    fail_rate = (ok + hard_fail) ? (float)hard_fail / (float)(ok + hard_fail) : 0.0f;

      // brzina
      if (drop > DROP_HIGH || fail_rate > 0.02f) {
        period_ms = std::min(PERIOD_MS_MAX, period_ms + 10);
      } else if (drop < DROP_LOW && fail_rate < 0.005f) {
        period_ms = std::max(PERIOD_MS_MIN, period_ms - 5);
      }

      // blago podesavanje kvaliteta 
      if (sensor_t* s = esp_camera_sensor_get()) {
        int q = s->status.quality;
        if (fail_rate > 0.02f || drop > DROP_HIGH || ema_size > 40*1024) {
          if (q < Q_UPPER_LIMIT) s->set_quality(s, q + 1);    // smaller JPEG
        } else if (fail_rate < 0.005f && drop < DROP_LOW && ema_size < 26*1024) {
          if (q > Q_LOWER_LIMIT) s->set_quality(s, q - 1);    // better quality
        }
      }

      // diagnostics
      multi_heap_info_t hi{};
      heap_caps_get_info(&hi, MALLOC_CAP_8BIT);
      ESP_LOGI(TAG, "diag: RSSI=%d dBm, heap=%u, psram=%u, fps~%d, drop=%.1f%%, ema=~%u, period=%dms, fail=%.2f%% (ENOMEM=%u ENOBUFS=%u)",
               get_rssi(),
               (unsigned)hi.total_free_bytes,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               (int)std::round(1000.0f / period_ms),
               drop*100.f,
               (unsigned)ema_size,
               period_ms,
               fail_rate*100.f, (unsigned)enomem, (unsigned)enobufs);

      win_frames = 0;
      win_dropped = 0;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
  }
}

} // extern "C"
