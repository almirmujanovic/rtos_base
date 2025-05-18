#include "mqtt_client.h"
#include "esp_log.h"
#include "uart_comm.h"
#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE
#endif


static const char *TAG = "MQTT_UART";
static esp_mqtt_client_handle_t global_mqtt_client = NULL;

void mqtt_publish_uart_data(const char *line) {
    if (global_mqtt_client) {
        int msg_id = esp_mqtt_client_publish(global_mqtt_client, "/robot/sensors", line, 0, 1, 0);
        ESP_LOGI(TAG, "Published to MQTT /robot/sensors, msg_id=%d", msg_id);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "/robot/commands", 0);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA received");
        char command[256] = {0};
        strncpy(command, event->data, event->data_len);
        command[event->data_len] = '\0';
        ESP_LOGI(TAG, "Command from PC: %s", command);
        uart_send_line(command);  // Send down to Arduino
        break;
    default:
        break;
    }
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.1.9",
        .broker.address.port = 1883,
        .credentials.client_id = "esp32-client"
    };

    global_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(global_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(global_mqtt_client);
}
