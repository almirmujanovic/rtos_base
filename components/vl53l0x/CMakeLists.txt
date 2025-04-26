idf_component_register(
    SRCS 
        "src/vl53l0x_api.c"
        "src/vl53l0x_platform.c"
        "src/vl53l0x_i2c_platform.c"
        "src/vl53l0x_api_core.c"
        "src/vl53l0x_api_strings.c"
        "src/vl53l0x_api_calibration.c"
        "src/vl53l0x_api_ranging.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
