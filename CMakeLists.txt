# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Enable C++ support globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rtos_base)

set(EXTRA_COMPONENT_DIRS "")
set(COMPONENT_EMBED_TXTFILES "")
set(COMPONENT_REQUIRES "")
set(COMPONENT_PRIV_REQUIRES "")

add_compile_definitions(-DMBEDTLS_CONFIG_FILE="mbedtls/esp_config.h")
add_compile_definitions(-DMBEDTLS_NO_TLS=1)

# Enable C++ features globally
target_compile_options(${CMAKE_PROJECT_NAME}.elf PRIVATE 
    $<$<COMPILE_LANGUAGE:CXX>:-fexceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-frtti>
)