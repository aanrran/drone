#include <Arduino.h>
#include "joysticks_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_dsp.h"
#include <stdio.h>
#include <algorithm> // For std::min and std::max in C++

// Define the FIR filter order and coefficients
#define FIR_ORDER 2
std::array<float, FIR_ORDER + 1> fir_coefficients = {1.0, 0.8, 0.5};

// Joysticks delay lines for each data stream
std::array<float, FIR_ORDER + 1> delayLineX1 = {0};
std::array<float, FIR_ORDER + 1> delayLineY1 = {0};
std::array<float, FIR_ORDER + 1> delayLineX2 = {0};
std::array<float, FIR_ORDER + 1> delayLineY2 = {0};

// Define a queue to communicate joystick data between tasks
QueueHandle_t joystickQueue;

float joystickData[4] = {0, 0, 0, 0}; // Define and initialize the array

/**
 * @brief Initialize joystick filters.
 * 
 * This function initializes the filter coefficients and delay lines 
 * for the joystick filters using a first-order low-pass filter.
 */
void joysticks_init() {
    // Create a queue to hold joystick data
    joystickQueue = xQueueCreate(10, sizeof(float[4]));
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
 * @brief Apply FIR filter to a single data point.
 * 
 * @param input The current raw data point.
 * @param delayLine The delay line array.
 * @return The filtered data point.
 */
float applyFIRFilter(float input, std::array<float, FIR_ORDER + 1>& delayLine) {
    // Shift delay line values
    for (int i = FIR_ORDER; i > 0; i--) {
        delayLine[i] = delayLine[i - 1];
    }
    delayLine[0] = input;

    // Apply FIR filter
    float output = 0.0;
    for (int i = 0; i <= FIR_ORDER; i++) {
        output += fir_coefficients[i] * delayLine[i];
    }

    return output;
}

/**
 * @brief Filter joystick data using an FIR filter.
 * 
 * This function applies an FIR filter to the raw joystick data 
 * and stores the filtered data in the provided output array.
 * 
 * @param joystickRawData An array of 4 raw joystick data points.
 * @param filteredJoystickData An array to store the 4 filtered joystick data points.
 */
void filterJoystickData(float joystickRawData[4], float filteredJoystickData[4]) {
    // Apply FIR filter to each data point
    filteredJoystickData[0] = clamp(applyFIRFilter(joystickRawData[0], delayLineX1));
    filteredJoystickData[1] = clamp(applyFIRFilter(joystickRawData[1], delayLineY1));
    filteredJoystickData[2] = clamp(applyFIRFilter(joystickRawData[2], delayLineX2));
    filteredJoystickData[3] = clamp(applyFIRFilter(joystickRawData[3], delayLineY2));
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
    } else {
        // Reset joystickData to zero if no new data received
        for (int i = 0; i < 4; i++) {
            receivedData[i] = 0;
        }
        // update the filter with zero readings
        filterJoystickData(receivedData, filteredJoystickData);

    }
    // Update the static joystickData array with new data
    for (int i = 0; i < 4; i++) {
        // joystickData[i] = static_cast<int>(filteredJoystickData[i] * 100);  // Convert to integer representation
        joystickData[i] = filteredJoystickData[i];
    }
    // Check if the joystick data is not all zero. if not zero, print the reading
    if (filteredJoystickData[0] > 0.01 || filteredJoystickData[1] > 0.01 || filteredJoystickData[2] > 0.01 || filteredJoystickData[3] > 0.01||
        filteredJoystickData[0] < -0.01 || filteredJoystickData[1] < -0.01 || filteredJoystickData[2] < -0.01 || filteredJoystickData[3] < -0.01
    ) {
        // Serial.printf("Joysticks Reading: x1: %.2f, y1: %.2f, x2: %.2f, y2: %.2f\n", (float)filteredJoystickData[0], (float)filteredJoystickData[1], (float)filteredJoystickData[2], (float)filteredJoystickData[3]);
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
