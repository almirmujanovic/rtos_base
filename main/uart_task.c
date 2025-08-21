#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "./include/uart_comm.h"
#include "./include/uart_task.h"
#include "esp_log.h"
#include <string.h>
#include "driver/uart.h"
#include "mqtt_uart_bridge.h"
#include "esp_task_wdt.h"  // Add watchdog support

#define LOG_LOCAL_LEVEL ESP_LOG_WARN
static const char *TAG = "UART_TASK";

extern QueueHandle_t uart_line_queue;

void uart_task(void *pvParameters) {
    ESP_LOGI(TAG, "🚀 UART task started (WATCHDOG SAFE)");
    
    // Register this task with the watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_send_line("STATUS");

    uart_event_t event;
    uint8_t* data_buffer = (uint8_t*) malloc(1024);
    static char line_buffer[256] = {0};
    static int line_pos = 0;
    
    while (1) {
        // CRITICAL FIX: Much shorter timeout + watchdog reset
        if (xQueueReceive(uart_line_queue, (void*)&event, pdMS_TO_TICKS(100))) {  // 100ms max
            
            // Reset watchdog immediately after receiving data
            esp_task_wdt_reset();
            
            switch (event.type) {
                case UART_DATA: {
                    // Read all data immediately with zero timeout
                    int len = uart_read_bytes(UART_NUM_1, data_buffer, event.size, 0);
                    
                    // Process each byte inline (optimized)
                    for (int i = 0; i < len; i++) {
                        char ch = (char)data_buffer[i];
                        
                        if (ch == '\n') {
                            if (line_pos > 0) {
                                line_buffer[line_pos] = '\0';
                                
                                // Remove \r if present
                                if (line_pos > 0 && line_buffer[line_pos-1] == '\r') {
                                    line_buffer[line_pos-1] = '\0';
                                }
                                
                                // IMMEDIATE MQTT publish (no logging for speed)
                                mqtt_publish_uart_data(line_buffer);
                            }
                            line_pos = 0;
                        } else if (ch >= 32 && ch <= 126 && line_pos < 255) {
                            line_buffer[line_pos++] = ch;
                        } else if (line_pos >= 255) {
                            line_pos = 0;  // Reset on overflow
                        }
                    }
                    break;
                }
                
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    uart_flush_input(UART_NUM_1);
                    line_pos = 0;
                    ESP_LOGW(TAG, "UART buffer overflow - flushed");
                    break;
                    
                default:
                    break;
            }
        } else {
            // CRITICAL: Reset watchdog on timeout too
            esp_task_wdt_reset();
        }
        
        // CRITICAL: Always yield to prevent watchdog trigger
        vTaskDelay(pdMS_TO_TICKS(1));  // Minimal delay but ensures yield
    }
    
    free(data_buffer);
    esp_task_wdt_delete(NULL);  // Unregister from watchdog before exit
}