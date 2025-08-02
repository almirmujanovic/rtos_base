idf_component_register(
    SRCS 
    "main.c" 
    "uart_comm.c" "uart_task.c" 
    "camera_config.c" "camera_task.c"
    "mqtt_uart_bridge.c" 
    "rtsp_server.cpp"
    "rtsp_wrapper.c"
    
    INCLUDE_DIRS "." "include" 
    REQUIRES 
        esp32-camera
        espp__rtsp
        espp__logger        
        espp__format  
        espp__socket
        nvs_flash 
        esp_netif 
        esp_wifi 
        esp_event 
        log 
        driver 
        freertos
        esp_psram
        lwip
        mqtt
)


# Enable full C++ support for espp components
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
target_compile_options(${COMPONENT_LIB} PRIVATE 
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>
    $<$<COMPILE_LANGUAGE:CXX>:-fexceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-frtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
)
# Disable fmt's locale-based formatting to avoid linking issues
target_compile_definitions(${COMPONENT_LIB} PRIVATE FMT_NO_LOCALE)

# Link C++ standard libraries
target_link_libraries(${COMPONENT_LIB} PUBLIC stdc++)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)