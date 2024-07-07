#include <Arduino.h>
#include "controller_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "joysticks_console.h"
#include "dc_motor_driver.h"

// Timer configuration
#define TIMER_INTERVAL_US 10000  // Timer interval in microseconds

static gptimer_handle_t timer = NULL;  // Timer handle


/**
 * @brief Placeholder PID control function
 * This function reads the static joystick data and prints them if they are not all zero.
 */
void PID_control() {
    // Read joystick data
    float x1 = joystickData[0];
    float y1 = joystickData[1];
    float x2 = joystickData[2];
    float y2 = joystickData[3];

    // Check if the joystick data is not all zero. if not zero, print the reading
    if (x1 > 0.01 || y1 > 0.01 || x2 > 0.01 || y2 > 0.01||
        x1 < -0.01 || y1 < -0.01 || x2 < -0.01 || y2 < -0.01) {
        // printf("PID Reading: x1: %.2f, y1: %.2f, x2: %.2f, y2: %.2f\n",x1, y1, x2, y2);
    }
    set_motor_pwm_duty((x1*100), (y1*100), (x2*100), (y2*100));

}

/**
 * @brief Timer interrupt service routine
 * This function is called by the timer interrupt and triggers the PID control function.
 */
static bool IRAM_ATTR onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    // Notify the PID control task to run
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)user_ctx, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE); // Return whether we should yield at the end of ISR
}

/**
 * @brief PID control task function
 * This function runs on core 2 and handles the PID control.
 */
void PID_control_task(void *pvParameters) {
    Serial.println("PID control task started on core 2");
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for the notification from ISR
        PID_control();  // Call the PID control function
    }
}

/**
 * @brief Controller task function
 * This function runs on core 2 and continuously reads joystick data from the queue.
 * If new data is received, it updates the static joystick data array.
 * If no new data is received, it resets the joystick data to zero.
 */
void controller_task(void *pvParameters) {
    Serial.println("Controller task started on core 2");

    while (true) {
        joysticks_read();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Function to start the controller task and configure the timer
 * This function creates a queue for joystick data, starts the controller task on core 2,
 * and configures a high-priority timer to trigger the PID control function.
 */
void startControllerTask() {

    // Initialize the joystick filters
    joysticks_init();    

    // Create the controller task pinned to core 2
    xTaskCreatePinnedToCore(controller_task, "ControllerTask", 4096, NULL, 1, NULL, 0);

    // Create the PID control task pinned to core 2
    TaskHandle_t PIDControlTaskHandle;
    xTaskCreatePinnedToCore(PID_control_task, "PIDControlTask", 4096, NULL, 23, &PIDControlTaskHandle, 0);

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

    // Register the timer callback and pass the PID control task handle
    gptimer_event_callbacks_t cbs = {
        .on_alarm = onTimer,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, (void*)PIDControlTaskHandle));

    // Enable and start the timer
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}
