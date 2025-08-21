#include "mqtt_client.h"
#include "esp_log.h"
#include "uart_comm.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "esp_timer.h"
#include <string.h>
#include <errno.h>

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

#define USE_BINARY_SENSORS 1

static const char *TAG = "MQTT_UART";
#define MQTT_BROKER_IP "192.168.1.14"

typedef struct {
    uint8_t vl53_distance;
    uint8_t hcsr04_1;
    uint8_t hcsr04_2;
    uint8_t hcsr04_3;
    bool data_ready;
} sensor_data_t;

// CONNECTION STATE TRACKING
static struct {
    bool connected;
    uint32_t msg_count;
    uint32_t error_count;
    uint32_t last_time;
    uint32_t disconnect_time;
} mqtt_stats = {0};

static esp_mqtt_client_handle_t global_mqtt_client = NULL;
static sensor_data_t current_sensors = {0};

// SPEED OPTIMIZED: Pre-allocated buffer to avoid stack allocation overhead
static uint8_t sensor_buffer[4] __attribute__((aligned(4)));

static inline uint8_t clamp_u8(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

void debug_mqtt_performance(void) {
    // Removed for speed - only errors now
}

// CONNECTION STATE CHECK - prevents publish when disconnected
bool mqtt_is_connected(void) {
    return mqtt_stats.connected && global_mqtt_client != NULL;
}

// SPEED OPTIMIZED: Only publish when connected
void mqtt_publish_sensor_data_binary(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3) {
    if (!mqtt_is_connected()) {
        // Silent drop when disconnected - no warnings
        return;
    }
    
    // Use pre-allocated aligned buffer for maximum speed
    sensor_buffer[0] = vl53;
    sensor_buffer[1] = hc1; 
    sensor_buffer[2] = hc2;
    sensor_buffer[3] = hc3;
    
    // Direct publish
    int result = esp_mqtt_client_publish(global_mqtt_client,
                           "/robot/sensors",
                           (const char*)sensor_buffer,
                           4, 0, 0);
    
    if (result >= 0) {
        mqtt_stats.msg_count++;
    } else {
        mqtt_stats.error_count++;
        // Connection might be lost
        mqtt_stats.connected = false;
    }
}

void mqtt_publish_sensor_data_text(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3) {
    if (!mqtt_is_connected()) {
        return; // Silent drop when disconnected
    }
    
    char sensor_msg[16];
    snprintf(sensor_msg, sizeof(sensor_msg), "%u,%u,%u,%u", vl53, hc1, hc2, hc3);
    
    int msg_id = esp_mqtt_client_publish(global_mqtt_client,
                                         "/robot/sensors",
                                         sensor_msg,
                                         0, 0, 0);
    if (msg_id >= 0) {
        mqtt_stats.msg_count++;
    } else {
        mqtt_stats.error_count++;
        mqtt_stats.connected = false;
    }
}

// Direct publish with connection check
void mqtt_publish_direct_sensor_data(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3) {
#if USE_BINARY_SENSORS
    mqtt_publish_sensor_data_binary(vl53, hc1, hc2, hc3);
#else
    mqtt_publish_sensor_data_text(vl53, hc1, hc2, hc3);
#endif
}

// SPEED OPTIMIZED: Inline parsing for maximum speed
void parse_and_publish_sensor_data_8bit(const char *line) {
    // Ultra-fast validation: check key positions directly
    if (line[0] != 'S' || line[1] != 'E' || line[7] != ',') return;

    // Only parse if connected
    if (!mqtt_is_connected()) return;

    // SPEED CRITICAL: Manual parsing optimized for our format
    const char* p = line + 8;  // Skip "SENSORS,"
    
    // Parse using bit shifts and avoiding multiplication
    int vl53 = 0, hc1 = 0, hc2 = 0, hc3 = 0;
    
    // Parse first number (vl53)
    while (*p >= '0' && *p <= '9') {
        vl53 = (vl53 << 3) + (vl53 << 1) + (*p++ - '0');  // vl53 * 10 + digit
    }
    if (*p++ != ',') return;
    
    // Parse second number (hc1)
    while (*p >= '0' && *p <= '9') {
        hc1 = (hc1 << 3) + (hc1 << 1) + (*p++ - '0');
    }
    if (*p++ != ',') return;
    
    // Parse third number (hc2)
    while (*p >= '0' && *p <= '9') {
        hc2 = (hc2 << 3) + (hc2 << 1) + (*p++ - '0');
    }
    if (*p++ != ',') return;
    
    // Parse fourth number (hc3)
    while (*p >= '0' && *p <= '9') {
        hc3 = (hc3 << 3) + (hc3 << 1) + (*p++ - '0');
    }

    // Inline clamp and publish immediately
    sensor_buffer[0] = (vl53 > 255) ? 255 : (vl53 < 0) ? 0 : (uint8_t)vl53;
    sensor_buffer[1] = (hc1 > 255) ? 255 : (hc1 < 0) ? 0 : (uint8_t)hc1;
    sensor_buffer[2] = (hc2 > 255) ? 255 : (hc2 < 0) ? 0 : (uint8_t)hc2;
    sensor_buffer[3] = (hc3 > 255) ? 255 : (hc3 < 0) ? 0 : (uint8_t)hc3;
    
    int result = esp_mqtt_client_publish(global_mqtt_client,
                           "/robot/sensors",
                           (const char*)sensor_buffer,
                           4, 0, 0);
    
    if (result >= 0) {
        mqtt_stats.msg_count++;
    } else {
        mqtt_stats.error_count++;
        mqtt_stats.connected = false;
    }
}

// ROUTER FOR UART LINES with connection check
void mqtt_publish_uart_data(const char *line) {
    if (!line) return;

    if (strncmp(line, "SENSORS,", 8) == 0) {
        parse_and_publish_sensor_data_8bit(line);
        return;
    }

    // Only publish status if connected
    if (!mqtt_is_connected()) {
        // Optionally log disconnection less frequently
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - mqtt_stats.disconnect_time > 5000) { // Every 5 seconds
            ESP_LOGW(TAG, "📴 MQTT disconnected - dropping status message");
            mqtt_stats.disconnect_time = now;
        }
        return;
    }

    // Non-sensor lines → status topic as text
    int msg_id = esp_mqtt_client_publish(global_mqtt_client, "/robot/status", line, 0, 0, 0);
    if (msg_id >= 0) {
        mqtt_stats.msg_count++;
    } else {
        mqtt_stats.error_count++;
        mqtt_stats.connected = false;
        ESP_LOGW(TAG, "Status publish failed - connection lost");
    }
}

// ENHANCED MQTT EVENT HANDLER with connection tracking
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "✅ MQTT_EVENT_CONNECTED");
        mqtt_stats.connected = true;
        mqtt_stats.error_count = 0;
        esp_mqtt_client_subscribe(client, "/robot/commands", 0);
        ESP_LOGI(TAG, "📡 Subscribed to /robot/commands");
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "❌ MQTT_EVENT_DISCONNECTED");
        mqtt_stats.connected = false;
        mqtt_stats.disconnect_time = esp_timer_get_time() / 1000;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "✅ MQTT_EVENT_SUBSCRIBED");
        break;

    case MQTT_EVENT_PUBLISHED:
        // Silent for speed
        break;

    case MQTT_EVENT_DATA: {
        char command[256] = {0};
        int copy_len = (event->data_len < 255) ? event->data_len : 255;
        strncpy(command, event->data, copy_len);
        command[copy_len] = '\0';
        ESP_LOGI(TAG, "📥 Command from PC: %s", command);
        uart_send_line(command);  // Forward to Arduino
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "❌ MQTT_EVENT_ERROR");
        mqtt_stats.connected = false;
        mqtt_stats.error_count++;
        if (event->error_handle && event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "TCP transport error: 0x%x", event->error_handle->esp_transport_sock_errno);
        }
        break;

    default:
        ESP_LOGD(TAG, "Other MQTT event id:%d", event->event_id);
        break;
    }
}

// CONNECTIVITY PROBE (unchanged)
static bool test_broker_connectivity(const char* host, int port) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(host);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return false;
    }

    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect to %s:%d errno %d", host, port, errno);
        close(sock);
        return false;
    }

    ESP_LOGI(TAG, "✅ Successfully connected to %s:%d", host, port);
    close(sock);
    return true;
}

// SPEED OPTIMIZED MQTT startup
void mqtt_app_start(void) {
    ESP_LOGI(TAG, "🚀 Testing MQTT broker connectivity...");
    if (!test_broker_connectivity(MQTT_BROKER_IP, 1883)) {
        ESP_LOGE(TAG, "❌ Cannot reach MQTT broker at %s:1883", MQTT_BROKER_IP);
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://" MQTT_BROKER_IP ":1883",
            },
        },
        .credentials = {
            .client_id = "esp32-robot-client",
        },
        .session = {
            .keepalive = 15,                    // Faster keepalive
            .disable_clean_session = false,
        },
        .network = {
            .timeout_ms = 1000,                 // Faster timeout
            .refresh_connection_after_ms = 0,
            .disable_auto_reconnect = false,    // Auto-reconnect enabled
        },
        .buffer = {
            .size = 2048,                       
            .out_size = 2048,
        },
        .task = {
            .priority = 10,
            .stack_size = 8192,
        }
    };

    global_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (global_mqtt_client == NULL) {
        ESP_LOGE(TAG, "❌ Failed to initialize MQTT client");
        return;
    }

    esp_err_t err = esp_mqtt_client_register_event(global_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to register MQTT event handler: %s", esp_err_to_name(err));
        return;
    }

    err = esp_mqtt_client_start(global_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start MQTT client: %s", esp_err_to_name(err));
        return;
    }

#if USE_BINARY_SENSORS
    ESP_LOGI(TAG, "✅ MQTT client started (BINARY MODE, connection tracking enabled)");
#else
    ESP_LOGI(TAG, "✅ MQTT client started (TEXT MODE, connection tracking enabled)");
#endif
}

// API for checking connection status
uint32_t mqtt_get_error_count(void) {
    return mqtt_stats.error_count;
}

uint32_t mqtt_get_message_count(void) {
    return mqtt_stats.msg_count;
}