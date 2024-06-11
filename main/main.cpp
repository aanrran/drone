#include <Arduino.h>
#include "wifi_server.h"
#include "controller_module.h"
#include "WiFi.h"
#include "nvs_flash.h"
#include "imu_mpu6050.h"


// Wi-Fi credentials
const char* ssid = "acer1664";
const char* password = "sdys3.14";

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

    // Create an instance of the IMU_MPU6050 class
    IMU_MPU6050 imu(GPIO_NUM_3, GPIO_NUM_46);
    // Initialize the MPU6050 sensor
    imu.mpu6050_init();

    // Start Wi-Fi task on core 0
    xTaskCreatePinnedToCore(wifi_task, "wifiTask", 8192, NULL, 1, NULL, 0);
    // Start the controller task on core 1
    startControllerTask();

    while (true) {

        // imu.mpu6050_printReadings();

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
