#include <Arduino.h>
#include "wifi_server.h"
#include "controller_module.h"
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
    startControllerTask();  // Start the controller task on core 2

    while (true) {
        delay(10000);
    }
}
