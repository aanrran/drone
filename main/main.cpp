// #include "Arduino.h"
// #include "camera.h"
// #include "controller.h"
// #include "drone_server.h"

// extern "C" {
//     #include <stdio.h>
//     #include "freertos/FreeRTOS.h"
//     #include "freertos/task.h"
// }

// extern "C" {
//     void app_main(void);
// }

// void app_main(void) {
//     initArduino();
//     // Initialize the camera
//     init_camera();

//     // Initialize WiFi server on core 0
//     xTaskCreatePinnedToCore(setup_wifi_server, "WiFiServerTask", 8192, NULL, 1, NULL, 0);

//     // Initialize PWM and control task on core 1
//     setup_pwm();
//     xTaskCreatePinnedToCore(control_task, "ControlTask", 4096, NULL, 1, NULL, 1);
// }

#include <Arduino.h>
#include "wifi_server.h"
#include "WiFi.h"
#include "nvs_flash.h"

const char* ssid = "acer1664";
const char* password = "sdys3.14";

void wifi_task(void *pvParameters) {
    Serial.println("Starting Wi-Fi task...");

    if (wifi_server_init(ssid, password) != ESP_OK) {
        Serial.println("Wi-Fi server init failed");
        vTaskDelete(NULL);  // Delete this task if initialization fails
    }

    while (true) {
        delay(10000);
    }
}

extern "C" {
    void app_main(void);
}

void app_main(void) {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    xTaskCreatePinnedToCore(wifi_task, "wifiTask", 8192, NULL, 1, NULL, 0);  // Create Wi-Fi task on core 0

    while (true) {
        delay(10000);
    }
}

