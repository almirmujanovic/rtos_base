#pragma once

void mqtt_app_start(void);
void mqtt_publish_uart_data(const char *message);

void mqtt_publish_sensor_data_text(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3);
void mqtt_publish_sensor_data_binary(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3);
void mqtt_publish_direct_sensor_data(uint8_t vl53, uint8_t hc1, uint8_t hc2, uint8_t hc3);
void debug_mqtt_performance(void);
