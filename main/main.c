#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_psram.h"
#include "lwip/ip4_addr.h"
#include "esp_mac.h"
#include "esp_system.h"
//#include "esp_http_server.h"
#include "mqtt_client.h"
//#include "websocket_server.h"

#include "./include/mqtt_uart_bridge.h"

#include "./include/rtsp_wrapper.h"
#include "./include/camera_config.h"
#include "./include/camera_task.h"
//#include "./include/camera_httpd.h"
#include "./include/uart_comm.h"
#include "./include/uart_task.h"

//#include "./include/mjpeg_stream_task.h"

// Define the WebSocket URL used for the server

static const char *TAG = "MAIN";

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

// WiFi credentials
#define WIFI_SSID "THINKPAD 0685"
#define WIFI_PASS "638\\Yg95"

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA starting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi STA connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconn = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "WiFi disconnected. Reason: %d", disconn->reason);
        esp_wifi_connect(); // Retry
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Testing network connectivity...");

        static bool server_started = false;

        if (!server_started) {
            server_started = true;

            uart_init();
            xTaskCreatePinnedToCore(uart_task, "UART task", 4096, NULL, 15, NULL, 1);

            if (init_camera() != ESP_OK) {
                ESP_LOGE(TAG, "Camera init failed. Aborting server startup.");
                return;
            }
            
            if (rtsp_server_init() == ESP_OK) {
                if (rtsp_server_start() == ESP_OK) {
                    xTaskCreatePinnedToCore(rtsp_camera_stream_task, "rtsp_stream", 
                                            12*1024, NULL, 3, NULL, 1);
                    ESP_LOGI(TAG, "RTSP server started successfully");
                } else {
                    ESP_LOGE(TAG, "Failed to start RTSP server");
                }
            } else {
                ESP_LOGE(TAG, "Failed to initialize RTSP server");
            }
            mqtt_app_start(); // Start MQTT client

        }
    }
}

void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
    .sta = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE); 
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_init_sta();
    
    ESP_LOGI("MAIN: HEAP", "Free heap: %u, internal: %u, PSRAM: %u",
        (unsigned int)esp_get_free_heap_size(),
        (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRId32 " bytes", (int32_t) esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
}



#define log_error_if_nonzero(message, ret) \
    if ((ret) != 0) { \
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, ret); \
    }

/*
void app_main(void) {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());   // ✅ Erase NVS partition
        ESP_ERROR_CHECK(nvs_flash_init());    // ✅ Retry init
    }
    wifi_init_sta();
    uart_init();
    
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed. Halting app.");
        return; // Don't start camera task if init fails
    }

    xTaskCreatePinnedToCore(uart_task, "UART Task", 4096, NULL, 15, NULL, 1);
   // xTaskCreatePinnedToCore(camera_udp_task, "UDP Camera Task", 8192, NULL, 5, NULL, 1);  

    ESP_LOGI("MAIN: HEAP", "Free heap: %u, internal: %u, PSRAM: %u",
        (unsigned int)esp_get_free_heap_size(),
        (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRId32 " bytes", (int32_t) esp_get_free_heap_size());    
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_WARN);           // Only warnings and errors
    esp_log_level_set("MAIN", ESP_LOG_INFO);        // Keep main info
    esp_log_level_set("MQTT_UART", ESP_LOG_INFO);   // Keep MQTT info
    esp_log_level_set("UART_TASK", ESP_LOG_WARN);

    esp_log_level_set("MQTT_CLIENT", ESP_LOG_WARN);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_WARN);
    esp_log_level_set("esp-tls", ESP_LOG_WARN);

    
   //  This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
   //  * Read "Establishing Wi-Fi or Ethernet Connection" section in
   //  * examples/protocols/README.md for more information about this function.
     

}*/