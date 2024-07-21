#include <Arduino.h>
#include "esp_camera.h"
#include "ai_camera.h"
#include "driver/i2c.h"  // Include the header for I2C
#include "driver/gpio.h"

// Camera pin configuration for ESP32-S3
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    8
#define SIOD_GPIO_NUM    11
#define SIOC_GPIO_NUM    10
#define Y2_GPIO_NUM      7
#define Y3_GPIO_NUM      5
#define Y4_GPIO_NUM      4
#define Y5_GPIO_NUM      6
#define Y6_GPIO_NUM      15
#define Y7_GPIO_NUM      17
#define Y8_GPIO_NUM      18
#define Y9_GPIO_NUM      9
#define VSYNC_GPIO_NUM   3
#define HREF_GPIO_NUM    46
#define PCLK_GPIO_NUM    16

// Camera configuration structure
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,  // Lower the resolution to VGA
    /*
    The frame size can be set to one of these options:
        FRAMESIZE_UXGA (1600 x 1200)
        FRAMESIZE_QVGA (320 x 240)
        FRAMESIZE_CIF (352 x 288)
        FRAMESIZE_VGA (640 x 480)
        FRAMESIZE_SVGA (800 x 600)
        FRAMESIZE_XGA (1024 x 768)
        FRAMESIZE_SXGA (1280 x 1024)
    */
    .jpeg_quality = 20,  // Increase JPEG quality to reduce the size
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = I2C_NUM_0
};



/**
 * @brief Function to initialize the camera
 * Configures and initializes the camera module with the specified settings.
 * 
 * @return esp_err_t Returns ESP_OK on success, otherwise returns an error code.
 */
esp_err_t ai_camera_init() {
    printf("Initializing camera...\n");

    // Power up the camera if needed
    if (PWDN_GPIO_NUM != -1) {
        pinMode(PWDN_GPIO_NUM, OUTPUT);
        digitalWrite(PWDN_GPIO_NUM, LOW);
    }

    // Initialize the camera with the configuration
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        printf("Camera Init Failed with error 0x%x\n", err);
        return err;
    }


    printf("Camera initialized successfully.\n");
    return ESP_OK;
}
