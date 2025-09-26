#include "camera_config.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "CAMERA_CONFIG";

// Freenove ESP32-S3 Eye pin mapping
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y2_GPIO_NUM       11
#define Y3_GPIO_NUM       9
#define Y4_GPIO_NUM       8
#define Y5_GPIO_NUM       10
#define Y6_GPIO_NUM       12
#define Y7_GPIO_NUM       18
#define Y8_GPIO_NUM       17
#define Y9_GPIO_NUM       16

#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

esp_err_t init_camera(void) {
    camera_config_t config = {
        .pin_pwdn       = PWDN_GPIO_NUM,
        .pin_reset      = RESET_GPIO_NUM,
        .pin_xclk       = XCLK_GPIO_NUM,
        .pin_sccb_sda   = SIOD_GPIO_NUM,
        .pin_sccb_scl   = SIOC_GPIO_NUM,

        .pin_d0         = Y2_GPIO_NUM,
        .pin_d1         = Y3_GPIO_NUM,
        .pin_d2         = Y4_GPIO_NUM,
        .pin_d3         = Y5_GPIO_NUM,
        .pin_d4         = Y6_GPIO_NUM,
        .pin_d5         = Y7_GPIO_NUM,
        .pin_d6         = Y8_GPIO_NUM,
        .pin_d7         = Y9_GPIO_NUM,
        .pin_vsync      = VSYNC_GPIO_NUM,
        .pin_href       = HREF_GPIO_NUM,
        .pin_pclk       = PCLK_GPIO_NUM,

        .xclk_freq_hz   = 20000000,
        .ledc_timer     = LEDC_TIMER_1,
        .ledc_channel   = LEDC_CHANNEL_1,

        .pixel_format   = PIXFORMAT_JPEG,    
        .frame_size     = FRAMESIZE_VGA,  
        .jpeg_quality   = 35,                
        .fb_count       = 1,                 
        .grab_mode      = CAMERA_GRAB_LATEST,
        .fb_location    = CAMERA_FB_IN_PSRAM 
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    // Sensor tuning
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, FRAMESIZE_VGA); // Match config              
        s->set_saturation(s, -1);
        s->set_sharpness(s, -2);
        s->set_denoise(s, 2);
        s->set_vflip(s, 1);                   
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}
