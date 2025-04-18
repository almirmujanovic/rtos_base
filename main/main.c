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

#include "tcp_server.h"
#include "camera_config.h"
#include "camera_task.h"
#include "uart_comm.h"
#include "uart_task.h"

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
        ESP_LOGI(TAG, "🚀 WiFi STA starting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "✅ WiFi STA connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconn = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "❌ WiFi disconnected. Reason: %d", disconn->reason);
        esp_wifi_connect(); // Retry
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "🌐 Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
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

    // Print MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


void app_main(void) {

    ESP_LOGI("PSRAM", "PSRAM size: %d bytes", esp_psram_get_size());
    if (esp_psram_get_size() == 0) {
        ESP_LOGE("PSRAM", "PSRAM NOT FOUND!");
    } else {
        ESP_LOGI("PSRAM", "PSRAM is working fine.");
    }


    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "Initializing WiFI... ");
    wifi_init_sta();
    //uart_init();

    // Print MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Get STA MAC
    ESP_LOGI("MAIN", "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Print IP address
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI("MAIN", "IP Address: " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI("MAIN", "Gateway:    " IPSTR, IP2STR(&ip_info.gw));
        ESP_LOGI("MAIN", "Netmask:    " IPSTR, IP2STR(&ip_info.netmask));
    } else {
        ESP_LOGW("MAIN", "Failed to get IP info.");
    }

    vTaskDelay(pdMS_TO_TICKS(200));
/*
    if (init_camera() != ESP_OK) {
        ESP_LOGE("MAIN", "Camera init failed!");
        return;
    }
*/
    xTaskCreatePinnedToCore(tcp_server_task, "TCP Server", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(camera_task, "Camera Task", 8192, NULL, 5, NULL, 1);
    //xTaskCreatePinnedToCore(uart_task, "UART Task", 4096, NULL, 5, NULL, 0);
}
