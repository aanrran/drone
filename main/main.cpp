#include "Arduino.h"
#include "camera.h"
#include "controller.h"
#include "drone_server.h"

extern "C" {
    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

extern "C" {
    void app_main(void);
}

void app_main(void) {
    initArduino();
    // Initialize the camera
    init_camera();

    // Initialize WiFi server on core 0
    xTaskCreatePinnedToCore(setup_wifi_server, "WiFiServerTask", 8192, NULL, 1, NULL, 0);

    // Initialize PWM and control task on core 1
    setup_pwm();
    xTaskCreatePinnedToCore(control_task, "ControlTask", 4096, NULL, 1, NULL, 1);
}
