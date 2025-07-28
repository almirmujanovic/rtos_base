#include "mqtt_client.h"
#include "esp_log.h"
#include "uart_comm.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "esp_timer.h"
#include <string.h>
#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif

static const char *TAG = "MQTT_UART";

// Sensor data structure for 8-bit values
typedef struct {
    uint8_t vl53_distance;      // 0-255 cm
    uint8_t hcsr04_1;          
    uint8_t hcsr04_2;          
    uint8_t hcsr04_3;          
    bool data_ready;
} sensor_data_t;

static uint32_t msg_count = 0;
static uint32_t last_time = 0;
static esp_mqtt_client_handle_t global_mqtt_client = NULL;
static sensor_data_t current_sensors = {0};

void debug_mqtt_performance(void) {
    msg_count++;
    uint32_t now = esp_timer_get_time() / 1000;  // ms
    
    if (now - last_time > 5000) {  // Every 5 seconds
        float rate = (float)msg_count / 5.0f;
        ESP_LOGI(TAG, "MQTT Rate: %.1f msg/sec, Free heap: %d", rate, (int)esp_get_free_heap_size());
        msg_count = 0;
        last_time = now;
    }
}

// Binary format publishing (most efficient - 4 bytes)
void mqtt_publish_sensor_data_binary(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3) {
    if (global_mqtt_client) {
        uint8_t sensor_data[4] = {vl53, hc1, hc2, hc3};
        
        int msg_id = esp_mqtt_client_publish(global_mqtt_client, "/robot/sensors", 
                                           (const char*)sensor_data, 4, 0, 0);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "Sensor publish failed");
        }
        debug_mqtt_performance();
    }
}

// Text format publishing (human readable - ~12 characters)
void mqtt_publish_sensor_data_text(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3) {
    if (global_mqtt_client) {
        // Format: "45,150,255,75"
        char sensor_msg[16];
        snprintf(sensor_msg, sizeof(sensor_msg), "%d,%d,%d,%d", vl53, hc1, hc2, hc3);
        
        int msg_id = esp_mqtt_client_publish(global_mqtt_client, "/robot/sensors", sensor_msg, 0, 0, 0);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "Sensor publish failed");
        }
        debug_mqtt_performance();
    }
}

// Parse incoming 8-bit sensor data from Arduino
void parse_and_publish_sensor_data_8bit(const char *line) {
    // Expected Arduino format: "SENSORS,45,150,255,75"
    if (strncmp(line, "SENSORS,", 8) == 0) {
        int vl53, hc1, hc2, hc3;
        if (sscanf(line + 8, "%d,%d,%d,%d", &vl53, &hc1, &hc2, &hc3) == 4) {
            // Clamp values to 0-255 range
            uint8_t vl53_8 = (vl53 > 255) ? 255 : (vl53 < 0) ? 0 : vl53;
            uint8_t hc1_8 = (hc1 > 255) ? 255 : (hc1 < 0) ? 0 : hc1;
            uint8_t hc2_8 = (hc2 > 255) ? 255 : (hc2 < 0) ? 0 : hc2;
            uint8_t hc3_8 = (hc3 > 255) ? 255 : (hc3 < 0) ? 0 : hc3;
            
            // Use text format (easier to debug)
            mqtt_publish_sensor_data_text(vl53_8, hc1_8, hc2_8, hc3_8);
            
            ESP_LOGD(TAG, "Sensors: VL53=%d, HC1=%d, HC2=%d, HC3=%d", 
                    vl53_8, hc1_8, hc2_8, hc3_8);
        } else {
            ESP_LOGW(TAG, "Failed to parse sensor data: %s", line);
        }
    }
}

// Updated UART data handler with sensor parsing
void mqtt_publish_uart_data(const char *line) {
    if (global_mqtt_client) {
        // Check if it's sensor data
        if (strncmp(line, "SENSORS,", 8) == 0) {
            parse_and_publish_sensor_data_8bit(line);
        } else {
            // Regular message - publish as is to status topic
            int msg_id = esp_mqtt_client_publish(global_mqtt_client, "/robot/status", line, 0, 0, 0);
            if (msg_id < 0) {
                ESP_LOGW(TAG, "Status publish failed");
            }
            debug_mqtt_performance();
        }
    }
}

// Direct sensor data publishing function (if you want to call it directly)
void mqtt_publish_direct_sensor_data(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3) {
    mqtt_publish_sensor_data_text(vl53, hc1, hc2, hc3);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "/robot/commands", 0);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
        
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");  // Removed msg_id for performance
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;
        
    case MQTT_EVENT_PUBLISHED:
        // Commented out for performance
        // ESP_LOGI(TAG, "📤 MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Command received");
        char command[256] = {0};
        int copy_len = event->data_len < 255 ? event->data_len : 255;
        strncpy(command, event->data, copy_len);
        command[copy_len] = '\0';
        ESP_LOGI(TAG, "Command from PC: %s", command);
        uart_send_line(command);  // Send down to Arduino
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "TCP transport error: 0x%x", event->error_handle->esp_transport_sock_errno);
        }
        break;
        
    default:
        ESP_LOGD(TAG, "Other MQTT event id:%d", event->event_id);
        break;
    }
}

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
    
    // Set socket timeout
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

void mqtt_app_start(void) {
    ESP_LOGI(TAG, "Testing MQTT broker connectivity...");
    if (!test_broker_connectivity("192.168.137.1", 1883)) {
        ESP_LOGE(TAG, "Cannot reach MQTT broker at 192.168.137.1:1883");
        return;
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://192.168.137.1:1883",
            },
        },
        .credentials = {
            .client_id = "esp32-robot-client",
        },
        .session = {
            .keepalive = 30,                    // Reduced from 60 for faster detection
            .disable_clean_session = false,
        },
        .network = {
            .timeout_ms = 5000,                 // Faster timeout
            .refresh_connection_after_ms = 0,   // Disable auto-refresh
            .disable_auto_reconnect = false,
        },
        .buffer = {
            .size = 2048,                       // Larger buffer for batching
            .out_size = 2048,                   // Larger outbound buffer
        },
        .task = {
            .priority = 20,                      // Higher priority than UART task
            .stack_size = 8192,                 // Increased stack for high freq
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
    
    ESP_LOGI(TAG, "MQTT client starting with optimized 8-bit sensor config...");
}
