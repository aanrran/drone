#include <Arduino.h>
#include "controller_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Define a queue to communicate joystick data between tasks
QueueHandle_t joystickQueue;

void PID_control(float x1, float y1, float x2, float y2) {
    Serial.printf("PID Control: x1: %.2f, y1: %.2f, x2: %.2f, y2: %.2f\n", x1, y1, x2, y2);
}

void controller_task(void *pvParameters) {
    Serial.println("Controller task started on core 2");

    float joystickData[4];

    while (true) {
        // Wait for joystick data from the queue
        if (xQueueReceive(joystickQueue, &joystickData, portMAX_DELAY)) {
            PID_control(joystickData[0], joystickData[1], joystickData[2], joystickData[3]);
        }
    }
}

void startControllerTask() {
    // Create a queue to hold joystick data
    joystickQueue = xQueueCreate(10, sizeof(float[4]));

    // Create the controller task pinned to core 2
    xTaskCreatePinnedToCore(controller_task, "ControllerTask", 4096, NULL, 1, NULL, 1);
}

void processJoystickData(float x1, float y1, float x2, float y2) {
    // Send joystick data to the queue
    float joystickData[4] = {x1, y1, x2, y2};
    xQueueSend(joystickQueue, &joystickData, portMAX_DELAY);
}
