#include <Arduino.h>
#include "controller_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"

// Timer configuration
#define TIMER_INTERVAL_US 10000  // Timer interval in microseconds

// Define a queue to communicate joystick data between tasks
QueueHandle_t joystickQueue;
static int joystickData[4] = {0, 0, 0, 0};  // Static array to hold joystick data

static gptimer_handle_t timer = NULL;  // Timer handle

/**
 * @brief Placeholder PID control function
 * This function reads the static joystick data and prints them if they are not all zero.
 */
void IRAM_ATTR PID_control() {
    // Read joystick data
    int x1 = joystickData[0];
    int y1 = joystickData[1];
    int x2 = joystickData[2];
    int y2 = joystickData[3];

    // Check if the joystick data is not all zero
    if (x1 != 0 || y1 != 0 || x2 != 0 || y2 != 0) {
        Serial.printf("PID Control: x1: %d, y1: %d, x2: %d, y2: %d\n", x1, y1, x2, y2);
    }
}

/**
 * @brief Timer interrupt service routine
 * This function is called by the timer interrupt and triggers the PID control function.
 */
static bool IRAM_ATTR onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    PID_control();  // Call the PID control function
    return true;
}

/**
 * @brief Controller task function
 * This function runs on core 2 and continuously reads joystick data from the queue.
 * If new data is received, it updates the static joystick data array.
 * If no new data is received, it resets the joystick data to zero.
 */
void controller_task(void *pvParameters) {
    Serial.println("Controller task started on core 2");

    float receivedData[4];  // Array to hold received joystick data

    while (true) {
        // Wait for joystick data from the queue with a timeout of 10 milliseconds
        if (xQueueReceive(joystickQueue, &receivedData, pdMS_TO_TICKS(10)) == pdPASS) {
            // Update the static joystickData array with new data
            for (int i = 0; i < 4; i++) {
                joystickData[i] = static_cast<int>(receivedData[i] * 100);  // Convert to integer representation
            }
        } else {
            // Reset joystickData to zero if no new data received
            for (int i = 0; i < 4; i++) {
                joystickData[i] = 0;
            }
        }
    }
}

/**
 * @brief Function to start the controller task and configure the timer
 * This function creates a queue for joystick data, starts the controller task on core 2,
 * and configures a high-priority timer to trigger the PID control function.
 */
void startControllerTask() {
    // Create a queue to hold joystick data
    joystickQueue = xQueueCreate(10, sizeof(float[4]));

    // Create the controller task pinned to core 2
    xTaskCreatePinnedToCore(controller_task, "ControllerTask", 4096, NULL, 1, NULL, 1);

    // Configure the timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz, 1 tick = 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    // Configure the alarm
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_INTERVAL_US,  // Set the alarm count for the interval
        .flags = {
            .auto_reload_on_alarm = true,  // Enable auto-reload
        }
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

    // Register the timer callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = onTimer,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    // Enable and start the timer
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

/**
 * @brief Function to process joystick data
 * This function sends joystick data to the queue for the controller task to process.
 * @param x1 First joystick X-axis
 * @param y1 First joystick Y-axis
 * @param x2 Second joystick X-axis
 * @param y2 Second joystick Y-axis
 */
void processJoystickData(float x1, float y1, float x2, float y2) {
    // Send joystick data to the queue
    float data[4] = {x1, y1, x2, y2};
    xQueueSend(joystickQueue, &data, portMAX_DELAY);
}
