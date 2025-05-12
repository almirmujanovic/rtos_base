#include <stdio.h>
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
#include "esp_http_server.h"

#include "./include/camera_httpd.h"
#include "./include/camera_config.h"
#include "./include/camera_task.h"
#include "./include/uart_comm.h"
#include "./include/uart_task.h"
#include "./include/websocket_server.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

static const char *TAG = "MAIN";

// Replace with your WiFi credentials
#define WIFI_SSID "MojaTV_Full_352765"
#define WIFI_PASS "almir2002"

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, " WiFi STA starting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, " WiFi STA connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconn = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "WiFi disconnected. Reason: %d", disconn->reason);
        esp_wifi_connect(); // Retry
    }else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
    
        static bool server_started = false;
        if (!server_started) {
            server_started = true;

            if (init_camera() != ESP_OK) {
                ESP_LOGE(TAG, "Camera init failed. Aborting server startup.");
                return;  // Or optionally just skip the server startup
            }
        

             // Start HTTP server with MJPEG streaming
            httpd_handle_t httpd = start_camera_httpd();

             // Reuse the same HTTP server to attach WebSocket handler
            if (httpd != NULL) {
                start_websocket_server(httpd);
            } else {
                ESP_LOGE(TAG, "Failed to start shared HTTP server");
            }
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
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE); 

    // Print MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}



void app_main(void) {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());   // ✅ Erase NVS partition
        ESP_ERROR_CHECK(nvs_flash_init());    // ✅ Retry init
    }
    wifi_init_sta();
    uart_init();

    xTaskCreatePinnedToCore(uart_task, "UART Task", 4096, NULL, 10, NULL, 1);
 //   xTaskCreatePinnedToCore(camera_task, "Camera Task", 8192, NULL, 5, NULL, 1);  
    ESP_LOGI("MAIN: HEAP", "Free heap: %u, internal: %u, PSRAM: %u",
        (unsigned int)esp_get_free_heap_size(),
        (unsigned int) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

}
