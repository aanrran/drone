#include <Arduino.h>
#include "wifi_server.h"
#include "controller_module.h"
#include "WiFi.h"
#include "nvs_flash.h"

// Wi-Fi credentials
const char* ssid = "acer1664";
const char* password = "sdys3.14";

extern "C" {
    void app_main(void);
}

void app_main(void) {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    // Start Wi-Fi task on core 0
    xTaskCreatePinnedToCore(wifi_task, "wifiTask", 8192, NULL, 1, NULL, 0);
    // Start the controller task on core 1
    startControllerTask();

    while (true) {
        delay(10000);  // Keep the main task alive
    }
}
