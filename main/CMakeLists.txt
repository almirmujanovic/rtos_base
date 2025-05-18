idf_component_register(
    SRCS 
    "main.c" 
    
    "uart_comm.c" "uart_task.c" 
    "camera_config.c" "camera_task.c" 
    "websocket_server.c" "camera_httpd.c"
    "mqtt_uart_bridge.c" 
   
    
    INCLUDE_DIRS "." "include" 
    REQUIRES 
        esp32-camera
        nvs_flash 
        esp_netif 
        esp_wifi 
        esp_event 
        log 
        driver 
        freertos
        esp_psram
        lwip
        esp_http_server
        mqtt
)

