idf_component_register(
    SRCS "main.c" 
    "tcp_server.c" 
    "uart_comm.c" "uart_task.c" 
    "camera_config.c" "camera_task.c"
    INCLUDE_DIRS "."
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
)
