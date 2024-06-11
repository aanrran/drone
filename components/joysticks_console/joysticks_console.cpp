#include <Arduino.h>
#include "joysticks_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_dsp.h"
#include <stdio.h>
#include <algorithm> // For std::min and std::max in C++

// Define joysticks filter parameters
#define joysticks_SAMPLE_RATE 100  // Sample rate in Hz, this the rate on the PC side
#define joysticks_CUTOFF_FREQ 420    // Cutoff frequency in Hz
#define joysticks_FILTER_ORDER 1    // Filter order

// Joysticks filter coefficients
float joysticks_coefficients[5]; // biquad needs 5 coefficients

// Joysticks delay lines for each data stream
float joysticks_delayLineX1[4] = {0}; // biquad delay line size is 4
float joysticks_delayLineY1[4] = {0};
float joysticks_delayLineX2[4] = {0};
float joysticks_delayLineY2[4] = {0};

// Define a queue to communicate joystick data between tasks
QueueHandle_t joystickQueue;


/**
 * @brief Initialize joystick filters.
 * 
 * This function initializes the filter coefficients and delay lines 
 * for the joystick filters using a first-order low-pass filter.
 */
void joysticks_init() {
    // Create a queue to hold joystick data
    joystickQueue = xQueueCreate(10, sizeof(float[4]));
    // Initialize filter coefficients for a first-order low-pass filter
    dsps_biquad_gen_lpf_f32(joysticks_coefficients, joysticks_CUTOFF_FREQ / (float)(joysticks_SAMPLE_RATE / 2), 0.707); // Q factor set to 0.707 for Butterworth filter
}

/**
 * @brief Clamp a value to the range [-1, 1].
 * 
 * @param value The value to clamp.
 * @return The clamped value.
 */
float clamp(float value) {
    return std::max(-1.0f, std::min(1.0f, value));
}

/**
 * @brief Filter joystick data.
 * 
 * This function applies a low-pass filter to the raw joystick data 
 * and stores the filtered data in the provided output array.
 * 
 * @param joystickRawData An array of 4 raw joystick data points.
 * @param filteredJoystickData An array to store the 4 filtered joystick data points.
 */
void filterJoystickData(float joystickRawData[4], float filteredJoystickData[4]) {
    // Apply low-pass filter to each data point
    dsps_biquad_f32(&joystickRawData[0], &filteredJoystickData[0], 1, joysticks_coefficients, joysticks_delayLineX1);
    dsps_biquad_f32(&joystickRawData[1], &filteredJoystickData[1], 1, joysticks_coefficients, joysticks_delayLineY1);
    dsps_biquad_f32(&joystickRawData[2], &filteredJoystickData[2], 1, joysticks_coefficients, joysticks_delayLineX2);
    dsps_biquad_f32(&joystickRawData[3], &filteredJoystickData[3], 1, joysticks_coefficients, joysticks_delayLineY2);

    // Clamp the filtered data to the range [-1, 1]
    for (int i = 0; i < 4; i++) {
        filteredJoystickData[i] = clamp(filteredJoystickData[i]);
    }
}

/**
 * @brief read joystick and filter the data. Then, update the data to int
 * 
 * The function filter read and the joystick data. Then, convert to int
 */
void joysticks_read() {
        
    float receivedData[4];  // Array to hold received joystick data
    float filteredJoystickData[4];
    // Wait for joystick data from the queue with a timeout of 10 milliseconds
    if (xQueueReceive(joystickQueue, &receivedData, pdMS_TO_TICKS(10)) == pdPASS) {
        // Filter the joystick data
        filterJoystickData(receivedData, filteredJoystickData);
        // Update the static joystickData array with new data
        for (int i = 0; i < 4; i++) {
            joystickData[i] = static_cast<int>(filteredJoystickData[i] * 100);  // Convert to integer representation
        }
    } else {
        // Reset joystickData to zero if no new data received
        for (int i = 0; i < 4; i++) {
            receivedData[i] = 0;
            joystickData[i] = 0;
        }
        // update the filter with zero readings
        filterJoystickData(receivedData, filteredJoystickData);
    }

    // Check if the joystick data is not all zero. if not zero, print the reading
    if (filteredJoystickData[0] > 0 || filteredJoystickData[1] > 0 || filteredJoystickData[2] > 0 || filteredJoystickData[3] > 0) {
        Serial.printf("Joysticks Reading: x1: %.2f, y1: %.2f, x2: %.2f, y2: %.2f\n", (float)filteredJoystickData[0], (float)filteredJoystickData[1], (float)filteredJoystickData[2], (float)filteredJoystickData[3]);
    }
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
