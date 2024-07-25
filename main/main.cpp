#include <Arduino.h>
#include "wifi_server.h"
#include "controller_module.h"
#include "WiFi.h"
#include "nvs_flash.h"

#include "dc_motor_driver.h"

// Wi-Fi credentials
// const char* ssid = "acer1664";
// const char* password = "sdys3.14";
const char* ssid = "drone_driver";
const char* password = "z46809(W";
extern "C" {
    void app_main(void);
}

/**
 * @brief Application entry point
 * Initializes the serial communication, starts Wi-Fi and controller tasks, and keeps the main task alive.
 */
void app_main(void) {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    dc_motor_init();
    set_motor_pwm_duty(0,0,0,0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Start Wi-Fi task on core 1
    xTaskCreatePinnedToCore(wifi_task, "wifiTask", 20480, NULL, 24, NULL, 1);
    // Start the controller task on core 0
    startControllerTask();

    while (true) {

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
