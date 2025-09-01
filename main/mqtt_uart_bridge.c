#include "mqtt_client.h"
#include "esp_log.h"
#include "uart_comm.h"
#include "esp_timer.h"
#include <string.h>

#define BROKER_IP "192.168.137.1"
static const char *TAG = "MQTT_UART";

// Performance tracking
typedef struct {
    uint32_t msg_count;
    uint32_t error_count;
    bool connected;
    int64_t last_time;
} mqtt_stats_t;

static mqtt_stats_t mqtt_stats = {0};
static esp_mqtt_client_handle_t global_mqtt_client = NULL;

// ✅ ROBUST: Manual parsing instead of sscanf for reliability
void parse_and_publish_sensor_data_8bit(const char *line) {
    int64_t parse_start = esp_timer_get_time();
    
    // Quick validation: check format "SENSORS,"
    if (line[0] != 'S' || line[1] != 'E' || line[7] != ',') {
        ESP_LOGW(TAG, "Invalid sensor format: %s", line);
        return;
    }

    if (!global_mqtt_client) return;

    // ✅ FAST: Manual parsing optimized for "SENSORS,vl53,hc1,hc2,hc3"
    const char* p = line + 8;  // Skip "SENSORS,"
    
    int values[4] = {0};
    int value_idx = 0;
    
    // Parse each number manually for speed and reliability
    for (int i = 0; i < 4 && *p != '\0' && *p != '\n' && *p != '\r'; i++) {
        int num = 0;
        
        // Parse digits
        while (*p >= '0' && *p <= '9') {
            num = num * 10 + (*p - '0');
            p++;
        }
        
        values[i] = num;
        value_idx++;
        
        // Skip comma (except for last value)
        if (*p == ',') p++;
    }
    
    // Validate we got all 4 values
    if (value_idx != 4) {
        ESP_LOGW(TAG, "Incomplete sensor data: %s (got %d values)", line, value_idx);
        return;
    }
    
    int64_t parse_time = esp_timer_get_time() - parse_start;
    
    // ✅ TEXT FORMAT: Proven to work well
    char sensor_msg[20];
    snprintf(sensor_msg, sizeof(sensor_msg), "%d,%d,%d,%d", 
             values[0], values[1], values[2], values[3]);
    
    int64_t mqtt_start = esp_timer_get_time();
    int result = esp_mqtt_client_publish(global_mqtt_client, "/robot/sensors", 
                                       sensor_msg, 0, 0, 0);
    int64_t mqtt_time = esp_timer_get_time() - mqtt_start;
    
    if (result >= 0) {
        mqtt_stats.msg_count++;
        
        // Log timing every 100 messages to monitor performance
        static uint32_t timing_counter = 0;
        if (++timing_counter % 100 == 0) {
            ESP_LOGI(TAG, "⏱️ SENSOR_PARSE: parse=%lldμs, mqtt=%lldμs, data=[%d,%d,%d,%d]", 
                    parse_time, mqtt_time, values[0], values[1], values[2], values[3]);
        }
    } else {
        mqtt_stats.error_count++;
        ESP_LOGW(TAG, "❌ MQTT sensor publish failed: %d", result);
    }
}

// ✅ ENHANCED: UART data router with timing
void mqtt_publish_uart_data(const char *line) {
    if (!line || strlen(line) == 0) return;
    
    int64_t start_time = esp_timer_get_time();
    
    if (global_mqtt_client) {
        if (strncmp(line, "SENSORS,", 8) == 0) {
            // Handle sensor data with optimized parsing
            parse_and_publish_sensor_data_8bit(line);
        } else if (strncmp(line, "TIMING,", 7) == 0) {
            // Handle timing data from Arduino
            ESP_LOGI(TAG, "📊 ARDUINO_TIMING: %s", line);
            
            // Forward timing to status topic for Python dashboard
            int msg_id = esp_mqtt_client_publish(global_mqtt_client, "/robot/status", line, 0, 0, 0);
            if (msg_id >= 0) mqtt_stats.msg_count++;
        } else {
            // Regular status message
            int msg_id = esp_mqtt_client_publish(global_mqtt_client, "/robot/status", line, 0, 0, 0);
            if (msg_id >= 0) {
                mqtt_stats.msg_count++;
            } else {
                mqtt_stats.error_count++;
                ESP_LOGW(TAG, "❌ Status publish failed");
            }
        }
    }
    
    // Log slow processing
    int64_t total_time = esp_timer_get_time() - start_time;
    if (total_time > 5000) {  // >5ms is suspicious
        ESP_LOGW(TAG, "⚠️ SLOW_PROCESSING: %lldμs for: %s", total_time, line);
    }
}

// ✅ Performance monitoring
void mqtt_log_performance_stats(void) {
    static int64_t last_stats = 0;
    int64_t now = esp_timer_get_time();
    
    if (now - last_stats > 10000000) {  // Every 10 seconds
        float rate = mqtt_stats.msg_count * 1000000.0 / (now - mqtt_stats.last_time);
        ESP_LOGI(TAG, "📊 MQTT_PERFORMANCE: connected=%s, msg_rate=%.1f/s, total=%lu, errors=%lu, heap=%d", 
                mqtt_stats.connected ? "YES" : "NO", 
                rate,
                mqtt_stats.msg_count, 
                mqtt_stats.error_count,
                (int)esp_get_free_heap_size());
        last_stats = now;
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "✅ MQTT_EVENT_CONNECTED");
        mqtt_stats.connected = true;
        mqtt_stats.last_time = esp_timer_get_time();
        esp_mqtt_client_subscribe(client, "/robot/commands", 0);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "❌ MQTT_EVENT_DISCONNECTED");
        mqtt_stats.connected = false;
        break;
        
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "📡 MQTT_EVENT_SUBSCRIBED to /robot/commands");
        break;
        
    case MQTT_EVENT_PUBLISHED:
        // Don't log every publish for performance
        break;
        
    case MQTT_EVENT_DATA:
        {
            char command[256] = {0};
            int copy_len = event->data_len < 255 ? event->data_len : 255;
            strncpy(command, event->data, copy_len);
            command[copy_len] = '\0';
            
            ESP_LOGI(TAG, "📨 Command from PC: %s", command);
            uart_send_line(command);  // Forward to Arduino
        }
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "❌ MQTT_EVENT_ERROR");
        mqtt_stats.error_count++;
        mqtt_stats.connected = false;
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "TCP transport error: 0x%x", event->error_handle->esp_transport_sock_errno);
        }
        break;
        
    default:
        ESP_LOGD(TAG, "Other MQTT event id:%d", event->event_id);
        break;
    }
    
    // Call performance monitoring
    mqtt_log_performance_stats();
}

void mqtt_app_start(void) {
    ESP_LOGI(TAG, "🚀 Starting MQTT client with optimized text parsing...");
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://" BROKER_IP ":1883",
            },
        },
        .credentials = {
            .client_id = "esp32-robot-client",
        },
        .session = {
            .keepalive = 60,                    // ✅ Match working script
            .disable_clean_session = false,
        },
        .network = {
            .timeout_ms = 3000,                 // ✅ Reasonable timeout
            .refresh_connection_after_ms = 0,
            .disable_auto_reconnect = false,
        },
        .buffer = {
            .size = 2048,
            .out_size = 2048,
        },
        .task = {
            .priority = 18,                     // ✅ Lower than UART task to prevent blocking
            .stack_size = 8192,
        }
    };

    global_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (global_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    esp_err_t err = esp_mqtt_client_register_event(global_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(err));
        return;
    }
    
    err = esp_mqtt_client_start(global_mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "✅ MQTT client started (TEXT FORMAT, connection tracking enabled)");
}

// ✅ API functions for status checking
bool mqtt_is_connected(void) {
    return mqtt_stats.connected && global_mqtt_client != NULL;
}

uint32_t mqtt_get_message_count(void) {
    return mqtt_stats.msg_count;
}

uint32_t mqtt_get_error_count(void) {
    return mqtt_stats.error_count;
}