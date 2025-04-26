idf_component_register(
    SRCS 
    "vl53l0x_task.c" "main.c" 
    "tcp_server.c" 
    "uart_comm.c" "uart_task.c" 
    "camera_config.c" "camera_task.c"
    "i2c_init.c"
    INCLUDE_DIRS "." "include" "../components/vl53l0x/include"
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
)

set(EXTRA_COMPONENT_DIRS ./components/vl53l0x)


